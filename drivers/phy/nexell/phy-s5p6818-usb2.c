// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nexell S5P6818 shared USB 2.0 host PHY.
 *
 * EHCI and OHCI have separate controller register windows but share the
 * host PHY, clocks, reset and TIEOFF registers.  The provider exposes two
 * PHY instances so the controller-specific DMA burst setting remains
 * explicit while the power state is reference counted.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define NEXELL_USB_HOST_REFCLK		12000000UL

#define NX_HOST_CON0			0x14
#define NX_HOST_CON0_SS_WORD_IF		BIT(26)
#define NX_HOST_CON0_SS_WORD_IF_ENB	BIT(25)
#define NX_HOST_CON0_SS_WORD_IF_16	\
	(NX_HOST_CON0_SS_WORD_IF | NX_HOST_CON0_SS_WORD_IF_ENB)
#define NX_HOST_CON0_N_HOST_HSIC_RESET_SYNC	BIT(22)
#define NX_HOST_CON0_N_HOST_UTMI_RESET_SYNC	BIT(21)
#define NX_HOST_CON0_N_HOST_PHY_RESET_SYNC	BIT(20)
#define NX_HOST_CON0_N_AUXWELL_RESET_SYNC	BIT(19)
#define NX_HOST_CON0_N_OHCI_RESET_SYNC		BIT(18)
#define NX_HOST_CON0_N_RESET_SYNC		BIT(17)

#define NX_HOST_CON2			0x1c
#define NX_HOST_CON2_SS_ENA_INCRX_ALIGN	BIT(28)
#define NX_HOST_CON2_SS_ENA_INCR4		BIT(27)
#define NX_HOST_CON2_SS_ENA_INCR8		BIT(26)
#define NX_HOST_CON2_SS_ENA_INCR16		BIT(25)
#define NX_HOST_CON2_SS_DMA_BURST_MASK	\
	(NX_HOST_CON2_SS_ENA_INCR16 | NX_HOST_CON2_SS_ENA_INCR8 | \
	 NX_HOST_CON2_SS_ENA_INCR4 | NX_HOST_CON2_SS_ENA_INCRX_ALIGN)
#define NX_HOST_CON2_EHCI_SS_ENABLE_DMA_BURST	\
	(NX_HOST_CON2_SS_ENA_INCR16 | NX_HOST_CON2_SS_ENA_INCR8 | \
	 NX_HOST_CON2_SS_ENA_INCR4 | NX_HOST_CON2_SS_ENA_INCRX_ALIGN)
#define NX_HOST_CON2_OHCI_SS_ENABLE_DMA_BURST	\
	(NX_HOST_CON2_SS_ENA_INCR4 | NX_HOST_CON2_SS_ENA_INCRX_ALIGN)
#define NX_HOST_CON2_SS_FLADJ_VAL_0_OFFSET	21
#define NX_HOST_CON2_SS_FLADJ_VAL_OFFSET	3
#define NX_HOST_CON2_SS_FLADJ_VAL_NUM		6
#define NX_HOST_CON2_SS_FLADJ_VAL_MAX		0x7

#define NX_HOST_CON3			0x20
#define NX_HOST_CON3_POR_ENB		BIT(7)
#define NX_HOST_CON3_POR_MASK		GENMASK(8, 7)

#define NX_HOST_CON4			0x24
#define NX_HOST_CON4_WORDINTERFACE	BIT(9)
#define NX_HOST_CON4_WORDINTERFACE_ENB	BIT(8)
#define NX_HOST_CON4_WORDINTERFACE_16	\
	(NX_HOST_CON4_WORDINTERFACE | NX_HOST_CON4_WORDINTERFACE_ENB)

enum nexell_usb2_type {
	NEXELL_USB2_EHCI,
	NEXELL_USB2_OHCI,
};

struct nexell_usb2;

struct nexell_usb2_phy {
	struct nexell_usb2 *usb;
	enum nexell_usb2_type type;
};

struct nexell_usb2 {
	void __iomem *tieoff;
	struct clk *clk;
	struct clk *ref_clk;
	struct reset_control *reset;
	struct phy *phys[2];
	/* Serializes shared PHY power and controller-specific configuration. */
	struct mutex lock;
	unsigned int users;
};

static void nexell_usb2_set_fl_adj(struct nexell_usb2 *usb)
{
	const u32 fladj = 0x20;
	u32 value = fladj;
	int bit;

	for (bit = 0; bit < NX_HOST_CON2_SS_FLADJ_VAL_NUM; bit++)
		if (fladj & BIT(bit))
			value |= NX_HOST_CON2_SS_FLADJ_VAL_MAX <<
				(NX_HOST_CON2_SS_FLADJ_VAL_0_OFFSET -
				 bit * NX_HOST_CON2_SS_FLADJ_VAL_OFFSET);

	writel(value, usb->tieoff + NX_HOST_CON2);
}

static void nexell_usb2_configure(struct nexell_usb2 *usb,
					  enum nexell_usb2_type type)
{
	u32 value;

	nexell_usb2_set_fl_adj(usb);

	value = readl(usb->tieoff + NX_HOST_CON2);
	value &= ~NX_HOST_CON2_SS_DMA_BURST_MASK;
	value |= type == NEXELL_USB2_OHCI ?
		NX_HOST_CON2_OHCI_SS_ENABLE_DMA_BURST :
		NX_HOST_CON2_EHCI_SS_ENABLE_DMA_BURST;
	writel(value, usb->tieoff + NX_HOST_CON2);

	value = readl(usb->tieoff + NX_HOST_CON0);
	value &= ~GENMASK(26, 25);
	value |= NX_HOST_CON0_SS_WORD_IF_16;
	writel(value, usb->tieoff + NX_HOST_CON0);

	value = readl(usb->tieoff + NX_HOST_CON4);
	value &= ~GENMASK(9, 8);
	value |= NX_HOST_CON4_WORDINTERFACE_16;
	writel(value, usb->tieoff + NX_HOST_CON4);

	value = readl(usb->tieoff + NX_HOST_CON3);
	value &= ~NX_HOST_CON3_POR_MASK;
	value |= NX_HOST_CON3_POR_ENB;
	writel(value, usb->tieoff + NX_HOST_CON3);
	udelay(40);

	value = readl(usb->tieoff + NX_HOST_CON0);
	value |= NX_HOST_CON0_N_HOST_PHY_RESET_SYNC |
		 NX_HOST_CON0_N_HOST_UTMI_RESET_SYNC;
	value &= ~NX_HOST_CON0_N_HOST_HSIC_RESET_SYNC;
	writel(value, usb->tieoff + NX_HOST_CON0);
	udelay(2);

	value |= NX_HOST_CON0_N_RESET_SYNC |
		 NX_HOST_CON0_N_OHCI_RESET_SYNC |
		 NX_HOST_CON0_N_AUXWELL_RESET_SYNC;
	writel(value, usb->tieoff + NX_HOST_CON0);
	readl(usb->tieoff + NX_HOST_CON0);
}

static int nexell_usb2_phy_init(struct phy *phy)
{
	return 0;
}

static int nexell_usb2_phy_exit(struct phy *phy)
{
	return 0;
}

static int nexell_usb2_phy_power_on(struct phy *phy)
{
	struct nexell_usb2_phy *instance = phy_get_drvdata(phy);
	struct nexell_usb2 *usb = instance->usb;
	int ret;

	mutex_lock(&usb->lock);
	if (usb->users++)
		goto configure;

	ret = clk_set_rate(usb->clk, NEXELL_USB_HOST_REFCLK);
	if (ret)
		goto err_users;
	ret = clk_set_rate(usb->ref_clk, NEXELL_USB_HOST_REFCLK);
	if (ret)
		goto err_users;
	ret = clk_prepare_enable(usb->ref_clk);
	if (ret)
		goto err_users;
	ret = clk_prepare_enable(usb->clk);
	if (ret)
		goto err_disable_ref;
	ret = reset_control_deassert(usb->reset);
	if (ret)
		goto err_disable_clk;

configure:
	nexell_usb2_configure(usb, instance->type);
	mutex_unlock(&usb->lock);
	return 0;

err_disable_clk:
	clk_disable_unprepare(usb->clk);
err_disable_ref:
	clk_disable_unprepare(usb->ref_clk);
err_users:
	usb->users--;
	mutex_unlock(&usb->lock);
	return ret;
}

static int nexell_usb2_phy_power_off(struct phy *phy)
{
	struct nexell_usb2_phy *instance = phy_get_drvdata(phy);
	struct nexell_usb2 *usb = instance->usb;

	mutex_lock(&usb->lock);
	if (!usb->users)
		goto out;
	if (--usb->users)
		goto out;

	reset_control_assert(usb->reset);
	clk_disable_unprepare(usb->clk);
	clk_disable_unprepare(usb->ref_clk);
out:
	mutex_unlock(&usb->lock);
	return 0;
}

static const struct phy_ops nexell_usb2_phy_ops = {
	.init = nexell_usb2_phy_init,
	.exit = nexell_usb2_phy_exit,
	.power_on = nexell_usb2_phy_power_on,
	.power_off = nexell_usb2_phy_power_off,
	.owner = THIS_MODULE,
};

static struct phy *nexell_usb2_xlate(struct device *dev,
					     const struct of_phandle_args *args)
{
	struct nexell_usb2 *usb = dev_get_drvdata(dev);

	if (!usb || args->args_count != 1 || args->args[0] >= ARRAY_SIZE(usb->phys))
		return ERR_PTR(-EINVAL);

	return usb->phys[args->args[0]];
}

static int nexell_usb2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nexell_usb2 *usb;
	struct nexell_usb2_phy *instance;
	struct phy_provider *provider;
	int i;

	usb = devm_kzalloc(dev, sizeof(*usb), GFP_KERNEL);
	if (!usb)
		return -ENOMEM;

	mutex_init(&usb->lock);
	usb->tieoff = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(usb->tieoff))
		return PTR_ERR(usb->tieoff);

	usb->clk = devm_clk_get(dev, "usbhost");
	if (IS_ERR(usb->clk))
		return dev_err_probe(dev, PTR_ERR(usb->clk),
				     "failed to get USB host clock\n");
	usb->ref_clk = devm_clk_get(dev, "usbhost-ref");
	if (IS_ERR(usb->ref_clk))
		return dev_err_probe(dev, PTR_ERR(usb->ref_clk),
				     "failed to get USB reference clock\n");
	usb->reset = devm_reset_control_get_exclusive(dev, "host");
	if (IS_ERR(usb->reset))
		return dev_err_probe(dev, PTR_ERR(usb->reset),
				     "failed to get USB host reset\n");

	platform_set_drvdata(pdev, usb);
	for (i = 0; i < ARRAY_SIZE(usb->phys); i++) {
		instance = devm_kzalloc(dev, sizeof(*instance), GFP_KERNEL);
		if (!instance)
			return -ENOMEM;
		instance->usb = usb;
		instance->type = i;
		usb->phys[i] = devm_phy_create(dev, NULL, &nexell_usb2_phy_ops);
		if (IS_ERR(usb->phys[i]))
			return PTR_ERR(usb->phys[i]);
		phy_set_drvdata(usb->phys[i], instance);
	}

	provider = devm_of_phy_provider_register(dev, nexell_usb2_xlate);
	if (IS_ERR(provider))
		return PTR_ERR(provider);

	dev_info(dev, "S5P6818 shared USB host PHY registered\n");
	return 0;
}

static const struct of_device_id nexell_usb2_of_match[] = {
	{ .compatible = "nexell,s5p6818-usb-host-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, nexell_usb2_of_match);

static struct platform_driver nexell_usb2_driver = {
	.probe = nexell_usb2_probe,
	.driver = {
		.name = "s5p6818-usb2-phy",
		.of_match_table = nexell_usb2_of_match,
	},
};
module_platform_driver(nexell_usb2_driver);

MODULE_DESCRIPTION("Nexell S5P6818 USB 2.0 host PHY");
MODULE_LICENSE("GPL");
