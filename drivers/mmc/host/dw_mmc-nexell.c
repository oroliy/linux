// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nexell S5P6818 extensions for the Synopsys DesignWare MMC controller.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "dw_mmc.h"
#include "dw_mmc-pltfm.h"

#define NEXELL_DW_MMC_CLKSEL	0x09c
#define NEXELL_DW_MMC_CLKCTRL	0x114

#define NEXELL_CLKSEL_SAMPLE(x)	((x) & 0xff)
#define NEXELL_CLKSEL_DRIVE(x)	(((x) & 0x3) << 16)
#define NEXELL_CLKSEL_DIV(x)	(((x) & 0x3) << 24)
#define NEXELL_MMC_CLK_DELAY(dly, shift, sample, sample_shift) \
	(NEXELL_CLKSEL_SAMPLE(dly) | \
	 NEXELL_CLKSEL_DRIVE(shift) | \
	 (((sample) & 0xff) << 8) | \
	 (((sample_shift) & 0x3) << 24))

struct nexell_dw_mmc_priv {
	u32 clk_delay;
	u32 clk_sel;
};

static int nexell_dw_mmc_parse_dt(struct dw_mci *host)
{
	struct nexell_dw_mmc_priv *priv;
	struct device_node *np = host->dev->of_node;
	u32 drive_delay = 0;
	u32 drive_shift = 2;
	u32 sample_delay = 0;
	u32 sample_shift = 1;
	u32 mmcboost = 0;

	priv = devm_kzalloc(host->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	of_property_read_u32(np, "nexell,drive-delay", &drive_delay);
	of_property_read_u32(np, "nexell,drive-shift", &drive_shift);
	of_property_read_u32(np, "nexell,sample-delay", &sample_delay);
	of_property_read_u32(np, "nexell,sample-shift", &sample_shift);
	of_property_read_u32(np, "nexell,mmc-boost", &mmcboost);

	priv->clk_delay = NEXELL_MMC_CLK_DELAY(drive_delay, drive_shift,
						 sample_delay, sample_shift);
	priv->clk_sel = NEXELL_CLKSEL_SAMPLE(0) |
				NEXELL_CLKSEL_DRIVE(0) |
				NEXELL_CLKSEL_DIV(mmcboost ? 1 : 3);
	host->priv = priv;

	return 0;
}

static int nexell_dw_mmc_init(struct dw_mci *host)
{
	struct nexell_dw_mmc_priv *priv = host->priv;
	u32 input_clock = host->bus_hz;

	/* The S5P6818 CIU input is internally divided by two. */
	host->bus_hz /= 2;
	if (!host->bus_hz)
		return -EINVAL;

	writel_relaxed(priv->clk_sel, host->regs + NEXELL_DW_MMC_CLKSEL);
	writel_relaxed(priv->clk_delay, host->regs + NEXELL_DW_MMC_CLKCTRL);
	readl_relaxed(host->regs + NEXELL_DW_MMC_CLKCTRL);

	dev_info(host->dev,
		 "Nexell DW MMC ready: input=%u Hz bus=%u Hz clksel=0x%08x clkctrl=0x%08x\n",
		 input_clock, host->bus_hz, priv->clk_sel, priv->clk_delay);

	return 0;
}

static const struct dw_mci_drv_data nexell_dw_mmc_drv_data = {
	.common_caps = MMC_CAP_CMD23,
	.init = nexell_dw_mmc_init,
	.parse_dt = nexell_dw_mmc_parse_dt,
};

static const struct of_device_id nexell_dw_mmc_match[] = {
	{ .compatible = "nexell,s5p6818-dw-mshc",
	  .data = &nexell_dw_mmc_drv_data },
	{ }
};
MODULE_DEVICE_TABLE(of, nexell_dw_mmc_match);

static int nexell_dw_mmc_probe(struct platform_device *pdev)
{
	return dw_mci_pltfm_register(pdev, &nexell_dw_mmc_drv_data);
}

static struct platform_driver nexell_dw_mmc_driver = {
	.probe = nexell_dw_mmc_probe,
	.remove_new = dw_mci_pltfm_remove,
	.driver = {
		.name = "dwmmc_nexell",
		.of_match_table = nexell_dw_mmc_match,
		.pm = &dw_mci_pltfm_pmops,
	},
};
module_platform_driver(nexell_dw_mmc_driver);

MODULE_DESCRIPTION("Nexell S5P6818 DesignWare MMC driver");
MODULE_LICENSE("GPL");
