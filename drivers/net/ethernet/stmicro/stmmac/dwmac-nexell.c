// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nexell S5P6818 glue for the Synopsys GMAC controller.
 *
 * The U-Boot board code selects the external GMAC RX clock on CLKGEN10.
 * Keep the same setting when Linux takes ownership of the MAC.
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/platform_device.h>

#include "stmmac.h"
#include "stmmac_platform.h"

#define NEXELL_CLKGEN_ENABLE		0x000
#define NEXELL_CLKGEN_STAGE0		0x004
#define NEXELL_CLKGEN_MODULE_GATE	BIT(2)
#define NEXELL_CLKGEN_SRC_MASK		(0x7U << 2)
#define NEXELL_CLKGEN_DIV_MASK		(0xffU << 5)
#define NEXELL_CLKGEN_INV		BIT(1)
#define NEXELL_CLKGEN_EXT_RX		(4U << 2)

struct nexell_gmac {
	void __iomem *clkgen;
};

static int nexell_gmac_init(struct platform_device *pdev, void *priv)
{
	struct nexell_gmac *gmac = priv;
	u32 stage;
	u32 enable;

	/* Match U-Boot: external RX clock, divisor 1, non-inverted. */
	enable = readl(gmac->clkgen + NEXELL_CLKGEN_ENABLE);
	enable &= ~NEXELL_CLKGEN_MODULE_GATE;
	writel(enable, gmac->clkgen + NEXELL_CLKGEN_ENABLE);

	stage = readl(gmac->clkgen + NEXELL_CLKGEN_STAGE0);
	stage &= ~(NEXELL_CLKGEN_SRC_MASK | NEXELL_CLKGEN_DIV_MASK |
		   NEXELL_CLKGEN_INV);
	stage |= NEXELL_CLKGEN_EXT_RX;
	writel(stage, gmac->clkgen + NEXELL_CLKGEN_STAGE0);

	enable |= NEXELL_CLKGEN_MODULE_GATE;
	writel(enable, gmac->clkgen + NEXELL_CLKGEN_ENABLE);
	readl(gmac->clkgen + NEXELL_CLKGEN_ENABLE);

	dev_info(&pdev->dev,
		 "GMAC clock: stage0=0x%08x enable=0x%08x\n",
		 readl(gmac->clkgen + NEXELL_CLKGEN_STAGE0),
		 readl(gmac->clkgen + NEXELL_CLKGEN_ENABLE));

	return 0;
}

static int nexell_gmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct nexell_gmac *gmac;
	struct device_node *clkgen_np;
	struct resource clkgen_res;
	int clkgen_index;
	int phy_addr;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	gmac = devm_kzalloc(&pdev->dev, sizeof(*gmac), GFP_KERNEL);
	if (!gmac)
		return -ENOMEM;

	clkgen_np = of_parse_phandle(pdev->dev.of_node,
				     "nexell,clock-controller", 0);
	if (!clkgen_np)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing nexell,clock-controller\n");
	clkgen_index = of_property_match_string(clkgen_np, "reg-names", "gmac");
	if (clkgen_index < 0 || of_address_to_resource(clkgen_np, clkgen_index,
								&clkgen_res)) {
		of_node_put(clkgen_np);
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "invalid GMAC clock resource\n");
	}
	gmac->clkgen = devm_ioremap(&pdev->dev, clkgen_res.start,
				    resource_size(&clkgen_res));
	of_node_put(clkgen_np);
	if (!gmac->clkgen)
		return -ENOMEM;

	plat_dat = devm_stmmac_probe_config_dt(pdev, stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	/* U-Boot resets the external RTL8211E before handing over the GMAC. */
	if (plat_dat->mdio_bus_data)
		plat_dat->mdio_bus_data->needs_reset = false;

	/* S5P6818 exposes the classic GMAC/DMA register layout. */
	plat_dat->has_gmac = 1;
	plat_dat->mdio_no_data_write = true;
	plat_dat->enh_desc = 0;
	plat_dat->bugged_jumbo = 1;
	plat_dat->force_sf_dma_mode = 1;
	plat_dat->multicast_filter_bins = 256;
	plat_dat->unicast_filter_entries = 1;
	plat_dat->init = nexell_gmac_init;
	plat_dat->bsp_priv = gmac;

	phy_addr = plat_dat->phy_addr;
	if (plat_dat->phy_node)
		phy_addr = of_mdio_parse_addr(&pdev->dev, plat_dat->phy_node);

	dev_info(&pdev->dev, "Nexell GMAC using RGMII and PHY address %d\n",
		 phy_addr);

	return devm_stmmac_pltfr_probe(pdev, plat_dat, &stmmac_res);
}

static const struct of_device_id nexell_gmac_match[] = {
	{ .compatible = "nexell,s5p6818-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, nexell_gmac_match);

static struct platform_driver nexell_gmac_driver = {
	.probe = nexell_gmac_probe,
	.remove_new = stmmac_pltfr_remove,
	.driver = {
		.name = "nexell-s5p6818-dwmac",
		.pm = &stmmac_pltfr_pm_ops,
		.of_match_table = nexell_gmac_match,
	},
};
module_platform_driver(nexell_gmac_driver);

MODULE_DESCRIPTION("Nexell S5P6818 DWMAC glue");
MODULE_LICENSE("GPL");
