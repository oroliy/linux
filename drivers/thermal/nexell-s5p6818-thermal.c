// SPDX-License-Identifier: GPL-2.0-only
/* S5P6818 TMU: register layout and one-point trim from Nexell's BSP. */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

#include "thermal_hwmon.h"

#define TMU_TRIMINFO	0x00
#define TMU_CONTROL	0x20
#define TMU_STATUS	0x28
#define TMU_CURRENT_TEMP	0x40
#define TMU_INTEN	0xb0
#define TMU_INTSTAT	0xb4
#define TMU_INTCLEAR	0xb8
#define TMU_REMOTE_INTEN	0xc0
#define TMU_REMOTE_INTSTAT 0xc4
#define TMU_REMOTE_INTCLEAR 0xc8
#define TMU_EMUL_CON	0x100
#define TMU_CORE_EN	BIT(0)
#define TMU_TRIP_EN	BIT(12)
#define TMU_GAIN		GENMASK(11, 8)
#define TMU_FILTER	GENMASK(15, 13)
#define TMU_VREF		GENMASK(28, 24)
#define TMU_FUSE_TIEOFF	0x00

struct nexell_tmu {
	void __iomem *base;
	int trim;
};

static int nexell_tmu_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct nexell_tmu *tmu = thermal_zone_device_priv(tz);
	u32 raw;

	/* STATUS bit 0 means idle, not sample validity while sensing runs. */
	raw = readl(tmu->base + TMU_CURRENT_TEMP) & 0xff;
	/* Zero and full scale are not valid internal-probe measurements. */
	if (!raw || raw == 0xff)
		return -ENODATA;
	*temp = ((int)raw - tmu->trim + 25) * 1000;
	return 0;
}

static const struct thermal_zone_device_ops nexell_tmu_ops = {
	.get_temp = nexell_tmu_get_temp,
};

static void nexell_tmu_stop(void *data)
{
	struct nexell_tmu *tmu = data;
	u32 con = readl(tmu->base + TMU_CONTROL);

	writel(con & ~TMU_CORE_EN, tmu->base + TMU_CONTROL);
}

static int nexell_tmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct thermal_zone_device *tz;
	struct nexell_tmu *tmu;
	struct regmap *tieoff;
	struct clk *clk;
	u32 channel, value, con;
	int ret;

	tmu = devm_kzalloc(dev, sizeof(*tmu), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;
	tmu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tmu->base))
		return PTR_ERR(tmu->base);
	ret = of_property_read_u32(dev->of_node, "nexell,channel", &channel);
	if (ret || channel > 1)
		return -EINVAL;
	clk = devm_clk_get_enabled(dev, "tmu_apbif");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "TMU clock unavailable\n");
	tieoff = syscon_regmap_lookup_by_phandle(dev->of_node, "nexell,tieoff");
	if (IS_ERR(tieoff))
		return dev_err_probe(dev, PTR_ERR(tieoff), "TMU tieoff unavailable\n");

	/* SENSING_START reads factory trim into the latch; it does not burn fuses. */
	ret = regmap_update_bits(tieoff, TMU_FUSE_TIEOFF, BIT(channel), BIT(channel));
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(tieoff, TMU_FUSE_TIEOFF, value,
				       value & BIT(channel + 3), 1000, 10000);
	if (ret && ret != -ETIMEDOUT)
		return ret;
	tmu->trim = readl(tmu->base + TMU_TRIMINFO) & 0xff;
	/* Nexell's 4.4 TMU sensor configuration uses 45 for invalid trim. */
	if (ret || tmu->trim < 16 || tmu->trim > 76) {
		tmu->trim = 45;
		dev_warn(dev, "factory trim unavailable; using nominal trim 45\n");
	}

	/* Polling only: disable local IRQ, emulation and hardware trip output. */
	writel(0, tmu->base + TMU_INTEN);
	writel(readl(tmu->base + TMU_INTSTAT), tmu->base + TMU_INTCLEAR);
	writel(0, tmu->base + TMU_REMOTE_INTEN);
	writel(readl(tmu->base + TMU_REMOTE_INTSTAT),
	       tmu->base + TMU_REMOTE_INTCLEAR);
	writel(readl(tmu->base + TMU_EMUL_CON) & ~BIT(0),
	       tmu->base + TMU_EMUL_CON);
	con = readl(tmu->base + TMU_CONTROL);
	con &= ~(TMU_VREF | TMU_GAIN | TMU_FILTER | TMU_TRIP_EN | TMU_CORE_EN);
	con |= FIELD_PREP(TMU_VREF, 16) | FIELD_PREP(TMU_GAIN, 5) |
	       FIELD_PREP(TMU_FILTER, 4);
	writel(con, tmu->base + TMU_CONTROL);
	ret = readl_poll_timeout(tmu->base + TMU_STATUS, value,
				 value & BIT(0), 1000, 100000);
	if (ret)
		return dev_err_probe(dev, ret, "TMU not ready\n");
	writel(con | TMU_CORE_EN, tmu->base + TMU_CONTROL);
	ret = devm_add_action_or_reset(dev, nexell_tmu_stop, tmu);
	if (ret)
		return ret;
	/* Allow the first sample to complete before exposing it to readers. */
	msleep(100);
	tz = devm_thermal_of_zone_register(dev, 0, tmu, &nexell_tmu_ops);
	if (IS_ERR(tz))
		return dev_err_probe(dev, PTR_ERR(tz), "thermal zone unavailable\n");
	ret = devm_thermal_add_hwmon_sysfs(dev, tz);
	if (ret)
		return ret;
	dev_info(dev, "TMU%u temperature monitoring, trim=%d\n", channel, tmu->trim);
	return 0;
}

static const struct of_device_id nexell_tmu_match[] = {
	{ .compatible = "nexell,s5p6818-tmu" },
	{ }
};
MODULE_DEVICE_TABLE(of, nexell_tmu_match);

static struct platform_driver nexell_tmu_driver = {
	.probe = nexell_tmu_probe,
	.driver = {
		.name = "nexell-s5p6818-tmu",
		.of_match_table = nexell_tmu_match,
	},
};
module_platform_driver(nexell_tmu_driver);

MODULE_DESCRIPTION("Nexell S5P6818 internal temperature monitor");
MODULE_LICENSE("GPL");
