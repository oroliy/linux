// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nexell S5P6818 watchdog timer.
 *
 * The block uses the Samsung S3C watchdog register layout, but S5P6818
 * exposes its reset lines through the Nexell reset controller and is fed
 * by the 200 MHz bus PCLK (see the DT nexell,wdt-frequency property).
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/watchdog.h>

#define NEXELL_WDT_WTCON		0x00
#define NEXELL_WDT_WTDAT		0x04
#define NEXELL_WDT_WTCNT		0x08
#define NEXELL_WDT_WTCLRINT		0x0c

#define NEXELL_WDT_WTCON_RSTEN		BIT(0)
#define NEXELL_WDT_WTCON_INTEN		BIT(2)
#define NEXELL_WDT_WTCON_DIV128		(3U << 3)
#define NEXELL_WDT_WTCON_ENABLE		BIT(5)
#define NEXELL_WDT_WTCON_PRESCALE_MASK	(0xffU << 8)
#define NEXELL_WDT_WTCON_PRESCALE(x)	((x) << 8)

#define NEXELL_WDT_MAX_COUNT		0xffffU
#define NEXELL_WDT_MAX_PRESCALER		256U
#define NEXELL_WDT_CLOCK_DIVISOR		128U
#define NEXELL_WDT_DEFAULT_CLOCK		4000000UL
#define NEXELL_WDT_DEFAULT_TIMEOUT	5U

static unsigned int timeout = NEXELL_WDT_DEFAULT_TIMEOUT;
module_param(timeout, uint, 0644);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0644);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started");

struct nexell_wdt {
	void __iomem *base;
	struct clk *pclk;
	struct reset_control *reset;
	struct reset_control *por_reset;
	unsigned long rate;
	u16 count;
	u8 prescaler;
	spinlock_t lock;
	struct watchdog_device wdd;
};

static unsigned int nexell_wdt_max_timeout(struct nexell_wdt *wdt)
{
	u64 ticks;

	ticks = (u64)NEXELL_WDT_MAX_COUNT * NEXELL_WDT_MAX_PRESCALER *
		NEXELL_WDT_CLOCK_DIVISOR;
	ticks = div64_u64(ticks, wdt->rate);

	return min_t(u64, ticks, UINT_MAX);
}

static int nexell_wdt_calculate(struct nexell_wdt *wdt,
				unsigned int seconds, u16 *count, u8 *prescaler)
{
	u64 ticks, divisor, value;

	if (!seconds || seconds > wdt->wdd.max_timeout)
		return -EINVAL;

	ticks = DIV_ROUND_UP_ULL((u64)seconds * wdt->rate,
					 NEXELL_WDT_CLOCK_DIVISOR);
	divisor = DIV_ROUND_UP_ULL(ticks, NEXELL_WDT_MAX_COUNT);
	divisor = clamp_t(u64, divisor, 1, NEXELL_WDT_MAX_PRESCALER);
	value = DIV_ROUND_UP_ULL(ticks, divisor);
	if (!value || value > NEXELL_WDT_MAX_COUNT)
		return -EINVAL;

	*count = value;
	*prescaler = divisor - 1;
	return 0;
}

static void nexell_wdt_stop_locked(struct nexell_wdt *wdt)
{
	u32 wtcon;

	wtcon = readl(wdt->base + NEXELL_WDT_WTCON);
	wtcon &= ~(NEXELL_WDT_WTCON_ENABLE | NEXELL_WDT_WTCON_RSTEN |
		   NEXELL_WDT_WTCON_INTEN);
	writel(wtcon, wdt->base + NEXELL_WDT_WTCON);
	writel(0, wdt->base + NEXELL_WDT_WTCLRINT);
	readl(wdt->base + NEXELL_WDT_WTCON);
}

static int nexell_wdt_start(struct watchdog_device *wdd)
{
	struct nexell_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u32 wtcon;
	u32 wtcon_read;
	u32 count_after;

	spin_lock_irqsave(&wdt->lock, flags);
	nexell_wdt_stop_locked(wdt);
	wtcon = readl(wdt->base + NEXELL_WDT_WTCON);
	wtcon &= ~(NEXELL_WDT_WTCON_PRESCALE_MASK |
		   NEXELL_WDT_WTCON_INTEN | NEXELL_WDT_WTCON_RSTEN |
		   NEXELL_WDT_WTCON_ENABLE);
	wtcon |= NEXELL_WDT_WTCON_PRESCALE(wdt->prescaler) |
		 NEXELL_WDT_WTCON_DIV128 | NEXELL_WDT_WTCON_INTEN |
		 NEXELL_WDT_WTCON_RSTEN |
		 NEXELL_WDT_WTCON_ENABLE;
	writel(0, wdt->base + NEXELL_WDT_WTCLRINT);
	writel(wdt->count, wdt->base + NEXELL_WDT_WTDAT);
	writel(wdt->count, wdt->base + NEXELL_WDT_WTCNT);
	writel(wtcon, wdt->base + NEXELL_WDT_WTCON);
	wtcon_read = readl(wdt->base + NEXELL_WDT_WTCON);
	udelay(1000);
	count_after = readl(wdt->base + NEXELL_WDT_WTCNT);
	spin_unlock_irqrestore(&wdt->lock, flags);
	dev_info(wdt->wdd.parent,
		 "watchdog started: count=0x%04x prescaler=%u wtcon=0x%08x "
		 "wtcon_read=0x%08x cnt_after_1ms=0x%08x\n",
		 wdt->count, wdt->prescaler, wtcon, wtcon_read, count_after);

	return 0;
}

static int nexell_wdt_stop(struct watchdog_device *wdd)
{
	struct nexell_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);
	nexell_wdt_stop_locked(wdt);
	spin_unlock_irqrestore(&wdt->lock, flags);

	return 0;
}

static int nexell_wdt_ping(struct watchdog_device *wdd)
{
	struct nexell_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);
	writel(0, wdt->base + NEXELL_WDT_WTCLRINT);
	writel(wdt->count, wdt->base + NEXELL_WDT_WTCNT);
	spin_unlock_irqrestore(&wdt->lock, flags);

	return 0;
}

static int nexell_wdt_set_timeout(struct watchdog_device *wdd,
				  unsigned int seconds)
{
	struct nexell_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u16 count;
	u8 prescaler;
	u32 wtcon;
	int ret;

	ret = nexell_wdt_calculate(wdt, seconds, &count, &prescaler);
	if (ret)
		return ret;

	spin_lock_irqsave(&wdt->lock, flags);
	wdt->count = count;
	wdt->prescaler = prescaler;
	wtcon = readl(wdt->base + NEXELL_WDT_WTCON);
	wtcon &= ~NEXELL_WDT_WTCON_PRESCALE_MASK;
	wtcon |= NEXELL_WDT_WTCON_PRESCALE(prescaler);
	writel(count, wdt->base + NEXELL_WDT_WTDAT);
	writel(count, wdt->base + NEXELL_WDT_WTCNT);
	writel(wtcon, wdt->base + NEXELL_WDT_WTCON);
	writel(0, wdt->base + NEXELL_WDT_WTCLRINT);
	readl(wdt->base + NEXELL_WDT_WTCON);
	spin_unlock_irqrestore(&wdt->lock, flags);

	wdd->timeout = seconds;
	return 0;
}

static int nexell_wdt_restart(struct watchdog_device *wdd,
				      unsigned long action, void *data)
{
	struct nexell_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);
	nexell_wdt_stop_locked(wdt);
	writel(0, wdt->base + NEXELL_WDT_WTCLRINT);
	writel(0x80, wdt->base + NEXELL_WDT_WTDAT);
	writel(0x80, wdt->base + NEXELL_WDT_WTCNT);
	writel(NEXELL_WDT_WTCON_ENABLE | NEXELL_WDT_WTCON_DIV128 |
	       NEXELL_WDT_WTCON_INTEN | NEXELL_WDT_WTCON_RSTEN |
	       NEXELL_WDT_WTCON_PRESCALE(0x20),
	       wdt->base + NEXELL_WDT_WTCON);
	readl(wdt->base + NEXELL_WDT_WTCON);
	spin_unlock_irqrestore(&wdt->lock, flags);

	mdelay(500);
	return 0;
}

static const struct watchdog_info nexell_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "Nexell S5P6818 Watchdog",
};

static const struct watchdog_ops nexell_wdt_ops = {
	.owner = THIS_MODULE,
	.start = nexell_wdt_start,
	.stop = nexell_wdt_stop,
	.ping = nexell_wdt_ping,
	.set_timeout = nexell_wdt_set_timeout,
	.restart = nexell_wdt_restart,
};

static int nexell_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nexell_wdt *wdt;
	u32 pclk;
	u32 wdt_clock;
	u32 wtcon_before;
	u32 wtcon_after;
	bool boot_running;
	unsigned int default_timeout;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	wdt->pclk = devm_clk_get_optional_enabled(dev, "watchdog");
	if (IS_ERR(wdt->pclk))
		return dev_err_probe(dev, PTR_ERR(wdt->pclk),
				     "failed to enable watchdog clock\n");

	if (wdt->pclk)
		wdt->rate = clk_get_rate(wdt->pclk);
	if (!wdt->rate && !of_property_read_u32(dev->of_node,
						"nexell,wdt-frequency", &wdt_clock))
		wdt->rate = wdt_clock;
	if (!wdt->rate && !of_property_read_u32(dev->of_node,
						"nexell,pclk-frequency", &pclk))
		wdt->rate = pclk;
	if (!wdt->rate)
		wdt->rate = NEXELL_WDT_DEFAULT_CLOCK;

	wdt->por_reset = devm_reset_control_get_exclusive(dev, "wdt-por");
	if (IS_ERR(wdt->por_reset)) {
		dev_err(dev, "watchdog POR reset lookup failed: %ld\n",
			PTR_ERR(wdt->por_reset));
		return dev_err_probe(dev, PTR_ERR(wdt->por_reset),
				     "failed to get watchdog POR reset\n");
	}
	wdt->reset = devm_reset_control_get_exclusive(dev, "wdt");
	if (IS_ERR(wdt->reset)) {
		dev_err(dev, "watchdog reset lookup failed: %ld\n",
			PTR_ERR(wdt->reset));
		return dev_err_probe(dev, PTR_ERR(wdt->reset),
				     "failed to get watchdog reset\n");
	}

	ret = reset_control_deassert(wdt->por_reset);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to deassert watchdog POR reset\n");
	ret = reset_control_deassert(wdt->reset);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to deassert watchdog reset\n");

	spin_lock_init(&wdt->lock);
	wdt->wdd.info = &nexell_wdt_info;
	wdt->wdd.ops = &nexell_wdt_ops;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = nexell_wdt_max_timeout(wdt);
	if (!wdt->wdd.max_timeout)
		return dev_err_probe(dev, -EINVAL, "watchdog clock is too fast\n");

	watchdog_set_drvdata(&wdt->wdd, wdt);
	watchdog_init_timeout(&wdt->wdd, timeout, dev);
	default_timeout = min_t(unsigned int,
				 timeout ? timeout : NEXELL_WDT_DEFAULT_TIMEOUT,
				 wdt->wdd.max_timeout);
	if (!wdt->wdd.timeout || wdt->wdd.timeout > wdt->wdd.max_timeout)
		wdt->wdd.timeout = default_timeout;
	platform_set_drvdata(pdev, wdt);
	wtcon_before = readl(wdt->base + NEXELL_WDT_WTCON);
	boot_running = !!(wtcon_before & NEXELL_WDT_WTCON_ENABLE);
	if (!boot_running) {
		ret = nexell_wdt_stop(&wdt->wdd);
		if (ret)
			return dev_err_probe(dev, ret, "failed to stop watchdog\n");
	}
	wtcon_after = readl(wdt->base + NEXELL_WDT_WTCON);
	ret = nexell_wdt_set_timeout(&wdt->wdd, wdt->wdd.timeout);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set watchdog timeout\n");
	if (boot_running && IS_ENABLED(CONFIG_WATCHDOG_HANDLE_BOOT_ENABLED))
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);

	watchdog_set_nowayout(&wdt->wdd, nowayout);
	watchdog_set_restart_priority(&wdt->wdd, 128);
	wdt->wdd.parent = dev;

	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret)
		return ret;

	dev_info(dev,
		 "S5P6818 watchdog ready: clock=%lu Hz max=%u s "
		 "boot_running=%u wtcon_before=0x%08x wtcon_after=0x%08x\n",
		 wdt->rate, wdt->wdd.max_timeout, boot_running,
		 wtcon_before, wtcon_after);
	return 0;
}

static void nexell_wdt_shutdown(struct platform_device *pdev)
{
	struct nexell_wdt *wdt = platform_get_drvdata(pdev);

	if (wdt)
		nexell_wdt_stop(&wdt->wdd);
}

static const struct of_device_id nexell_wdt_of_match[] = {
	{ .compatible = "nexell,s5p6818-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, nexell_wdt_of_match);

static struct platform_driver nexell_wdt_driver = {
	.probe = nexell_wdt_probe,
	.shutdown = nexell_wdt_shutdown,
	.driver = {
		.name = "s5p6818-wdt",
		.of_match_table = nexell_wdt_of_match,
	},
};
module_platform_driver(nexell_wdt_driver);

MODULE_DESCRIPTION("Nexell S5P6818 watchdog timer driver");
MODULE_LICENSE("GPL");
