// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nexell S5P6818 machine description.
 */

#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/reboot.h>

#include <asm/mach/arch.h>
#include <asm/spinlock.h>

#define S5P6818_CLKPWR_BASE	0xc0010000
#define S5P6818_PWRCONT		0x224
#define S5P6818_PWRMODE		0x228
#define S5P6818_SWRSTENB	(1U << 3)
#define S5P6818_SWRESET		(1U << 12)

static void __iomem *s5p6818_clkpwr;

static void s5p6818_restart(enum reboot_mode mode, const char *cmd)
{
	u32 value;

	if (!s5p6818_clkpwr)
		s5p6818_clkpwr = ioremap(S5P6818_CLKPWR_BASE, 0x400);
	if (!s5p6818_clkpwr) {
		pr_emerg("s5p6818: unable to map CLKPWR for restart\n");
		return;
	}

	pr_emerg("s5p6818: software reset\n");
	value = readl(s5p6818_clkpwr + S5P6818_PWRCONT);
	writel(value | S5P6818_SWRSTENB,
	       s5p6818_clkpwr + S5P6818_PWRCONT);
	writel(S5P6818_SWRESET, s5p6818_clkpwr + S5P6818_PWRMODE);

	while (1)
		cpu_relax();
}

static const char *const s5p6818_dt_compat[] __initconst = {
	"nexell,s5p6818",
	NULL,
};

static void __init s5p6818_init_early(void)
{
#ifdef CONFIG_NEXELL_S5P6818_NO_CROSS_CLUSTER_SEV
	/* S5P6818 cannot deliver SEV between its two Cortex-A53 clusters. */
	WRITE_ONCE(nexell_s5p6818_poll_lock_wait, true);
#endif
}

static void __init s5p6818_init_machine(void)
{
	s5p6818_clkpwr = ioremap(S5P6818_CLKPWR_BASE, 0x400);
	if (!s5p6818_clkpwr)
		pr_err("s5p6818: CLKPWR restart backend unavailable\n");

	of_platform_default_populate(NULL, NULL, NULL);
}

DT_MACHINE_START(S5P6818, "Nexell S5P6818")
	.init_early = s5p6818_init_early,
	.init_machine = s5p6818_init_machine,
	.restart = s5p6818_restart,
	.dt_compat = s5p6818_dt_compat,
MACHINE_END
