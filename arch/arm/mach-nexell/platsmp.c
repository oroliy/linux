// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5P6818 Cortex-A53 SMP bring-up.
 *
 * The iROM/firmware holding pen consumes the physical entry point from the
 * PWR scratch register and the target MPIDR affinity from the next scratch
 * register.  This is the protocol used by the vendor S5P6818 BSP.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/smp.h>
#include <asm/smp_plat.h>

#define S5P6818_SCR_ARM_SECOND_BOOT	0xc0010230
#define S5P6818_SCR_SMP_WAKE_CPU_ID	0xc0010234
#define S5P6818_SCR_SIGNATURE_RESET	0xc0010868
#define S5P6818_RESET_BASE		0xc0012000
#define S5P6818_WDT_BASE		0xc0019000
#define S5P6818_CCI_CTRL_OVERRIDE	0x0000
#define S5P6818_CCI_STATUS		0x000c
#define S5P6818_CCI_S0_SCR		0x1000
#define S5P6818_CCI_S1_SCR		0x2000
#define S5P6818_CCI_S2_SCR		0x3000
#define S5P6818_CCI_S3_SCR		0x4000
#define S5P6818_CCI_S4_SCR		0x5000
#define S5P6818_MAX_CPUS		8

#define S5P6818_WDT_WTCON		0x00
#define S5P6818_WDT_WTDAT		0x04
#define S5P6818_WDT_WTCNT		0x08
#define S5P6818_WDT_WTCLRINT		0x0c
#define S5P6818_WDT_WTCON_RSTEN		BIT(0)
#define S5P6818_WDT_WTCON_INTEN		BIT(2)
#define S5P6818_WDT_WTCON_DIV128		(3U << 3)
#define S5P6818_WDT_WTCON_ENABLE		BIT(5)
#define S5P6818_WDT_WTCON_PRESCALE(x)	((x) << 8)
#define S5P6818_WDT_PRESCALER		255U
/* ~10.7 s at the 200 MHz WDT input clock (bus PCLK). */
#define S5P6818_WDT_COUNT		65535U

extern void __secondary_startup(void);

volatile int pen_release = -1;

static void __iomem *s5p6818_second_boot;
static void __iomem *s5p6818_wake_cpu;
static void __iomem *s5p6818_signature_reset;
static void __iomem *s5p6818_cci;

/*
 * SMP bring-up runs before platform drivers are probed.  Arm the SoC WDT
 * here so a secondary CPU that wedges before initramfs can still recover the
 * board.  The watchdog driver preserves this running instance and the
 * watchdog core takes over keepalives once the driver is registered.
 */
static void __init s5p6818_arm_early_watchdog(void)
{
#if IS_ENABLED(CONFIG_NEXELL_WDT)
	void __iomem *reset;
	void __iomem *wdt;
	u32 reset_value;
	u32 wtcon;

	reset = ioremap(S5P6818_RESET_BASE, 0x0c);
	if (!reset) {
		pr_warn("S5P6818 SMP: unable to map watchdog reset lines\n");
		return;
	}

	/* WDT and WDT-POR are reset IDs 58 and 59, bank 1 bits 26 and 27. */
	reset_value = readl(reset + 0x04);
	reset_value |= BIT(26) | BIT(27);
	writel(reset_value, reset + 0x04);
	readl(reset + 0x04);
	udelay(10);

	wdt = ioremap(S5P6818_WDT_BASE, 0x1000);
	if (!wdt) {
		pr_warn("S5P6818 SMP: unable to map early watchdog\n");
		return;
	}

	wtcon = readl(wdt + S5P6818_WDT_WTCON);
	wtcon &= ~(0xffU << 8 | S5P6818_WDT_WTCON_INTEN |
		  S5P6818_WDT_WTCON_RSTEN | S5P6818_WDT_WTCON_ENABLE);
	wtcon |= S5P6818_WDT_WTCON_PRESCALE(S5P6818_WDT_PRESCALER) |
		 S5P6818_WDT_WTCON_DIV128 | S5P6818_WDT_WTCON_INTEN |
		 S5P6818_WDT_WTCON_RSTEN | S5P6818_WDT_WTCON_ENABLE;
	writel(0, wdt + S5P6818_WDT_WTCLRINT);
	writel(S5P6818_WDT_COUNT, wdt + S5P6818_WDT_WTDAT);
	writel(S5P6818_WDT_COUNT, wdt + S5P6818_WDT_WTCNT);
	writel(wtcon, wdt + S5P6818_WDT_WTCON);
	pr_info("S5P6818 SMP: early watchdog armed count=%u "
		"(~10.7s @ 200MHz bus PCLK)\n",
		readl(wdt + S5P6818_WDT_WTCNT));
#endif
}

/* The legacy S5P6818 boot monitor uses a linear 0..7 CPU ID. */
static u32 s5p6818_legacy_cpu_id(u32 mpidr)
{
	if (mpidr & 0x4400)
		mpidr |= 4;

	return mpidr & 0xf;
}

static void s5p6818_write_pen_release(int value)
{
	pen_release = value;
	smp_wmb();
	sync_cache_w(&pen_release);
}

static int __init s5p6818_smp_map_registers(void)
{
	s5p6818_second_boot = ioremap(S5P6818_SCR_ARM_SECOND_BOOT, sizeof(u32));
	s5p6818_wake_cpu = ioremap(S5P6818_SCR_SMP_WAKE_CPU_ID, sizeof(u32));
	s5p6818_signature_reset = ioremap(S5P6818_SCR_SIGNATURE_RESET,
					  sizeof(u32));
	if (!s5p6818_second_boot || !s5p6818_wake_cpu ||
	    !s5p6818_signature_reset) {
		pr_err("s5p6818 SMP: unable to map wake registers\n");
		return -ENOMEM;
	}

	return 0;
}

/*
 * CCI-400 setup.  NOTE: do not write the speculation-control register
 * (+0x4) from Linux — the runs that set it (2026-08-15 smpen-cci/clean/v3)
 * all reset the SoC at the first secondary-CPU release, while every run
 * using only the direct SCR writes below survived.  2ndboot sets the
 * speculation bits in SRAM context; Linux re-asserts only COR and the
 * cluster snoop/DVM words, matching the vendor kernel's nxp_cpu_arch_init.
 */
static void s5p6818_cci_init(void)
{
	writel_relaxed(0x8, s5p6818_cci + S5P6818_CCI_CTRL_OVERRIDE);
	writel_relaxed(0x0, s5p6818_cci + S5P6818_CCI_S0_SCR);
	writel_relaxed(0x0, s5p6818_cci + S5P6818_CCI_S1_SCR);
	writel_relaxed(0x0, s5p6818_cci + S5P6818_CCI_S2_SCR);
	writel_relaxed(0xc0000003, s5p6818_cci + S5P6818_CCI_S3_SCR);
	writel_relaxed(0xc0000003, s5p6818_cci + S5P6818_CCI_S4_SCR);
	wmb();

	pr_info("S5P6818 SMP: CCI configured ctrl=0x%08x cluster1=0x%08x "
		"cluster0=0x%08x\n",
		readl_relaxed(s5p6818_cci + S5P6818_CCI_CTRL_OVERRIDE),
		readl_relaxed(s5p6818_cci + S5P6818_CCI_S3_SCR),
		readl_relaxed(s5p6818_cci + S5P6818_CCI_S4_SCR));
}

static void s5p6818_secondary_init(unsigned int cpu)
{
	s5p6818_write_pen_release(-1);
}

static int s5p6818_boot_secondary(unsigned int cpu,
					 struct task_struct *idle)
{
	unsigned long timeout;
	u32 mpidr = cpu_logical_map(cpu);
	u32 legacy_id = s5p6818_legacy_cpu_id(mpidr);

	/* The firmware reads both values before releasing the target core. */
	writel_relaxed(__pa_symbol(__secondary_startup), s5p6818_second_boot);
	/* The boot monitor takes a logical CPU number, not its MPIDR. */
	writel_relaxed(cpu, s5p6818_wake_cpu);
	s5p6818_write_pen_release(legacy_id);
	pr_info("S5P6818 SMP: boot cpu=%u mpidr=0x%08x pen=0x%08x\n",
		cpu, mpidr, (u32)pen_release);
	wmb();
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	timeout = jiffies + HZ;
	while (time_before(jiffies, timeout)) {
		sync_cache_r(&pen_release);
		smp_rmb();
		if (pen_release == -1)
			break;
		udelay(10);
	}

	pr_info("S5P6818 SMP: cpu=%u wait done pen=%d\n",
		cpu, pen_release);

	return pen_release == -1 ? 0 : -ETIMEDOUT;
}

static void __init s5p6818_smp_init_cpus(void)
{
	unsigned int i;
	unsigned int ncores = min_t(unsigned int, nr_cpu_ids,
					    S5P6818_MAX_CPUS);

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);
}

static void __init s5p6818_smp_prepare_cpus(unsigned int max_cpus)
{
	phys_addr_t entry = __pa_symbol(__secondary_startup);

	s5p6818_arm_early_watchdog();

	if (s5p6818_smp_map_registers())
		return;

	{
		struct device_node *cci_node;

		cci_node = of_find_compatible_node(NULL, NULL, "arm,cci-400");
		if (cci_node) {
			s5p6818_cci = of_iomap(cci_node, 0);
			of_node_put(cci_node);
		}
	}
	if (s5p6818_cci)
		s5p6818_cci_init();
	else
		pr_warn("S5P6818 SMP: unable to map CCI-400\n");

	writel_relaxed(entry, s5p6818_second_boot);
	writel_relaxed(~0U, s5p6818_signature_reset);
	writel_relaxed(~0U, s5p6818_wake_cpu);
	wmb();
	pr_info("S5P6818 SMP: secondary entry=0x%pa max_cpus=%u\n",
		&entry, max_cpus);
}

static const struct smp_operations s5p6818_smp_ops __initconst = {
	.smp_init_cpus		= s5p6818_smp_init_cpus,
	.smp_prepare_cpus	= s5p6818_smp_prepare_cpus,
	.smp_secondary_init	= s5p6818_secondary_init,
	.smp_boot_secondary	= s5p6818_boot_secondary,
};

CPU_METHOD_OF_DECLARE(s5p6818_smp, "nexell,s5p6818-smp",
			      &s5p6818_smp_ops);
