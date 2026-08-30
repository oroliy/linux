// SPDX-License-Identifier: GPL-2.0-only
/* Nexell S5P6818 OHCI host controller glue. */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "ohci.h"

#define DRIVER_DESC "Nexell S5P6818 OHCI driver"

struct nexell_ohci {
	struct phy *phy;
};

#define hcd_to_nexell_ohci(hcd) \
	((struct nexell_ohci *)hcd_to_ohci(hcd)->priv)

static struct hc_driver __read_mostly nexell_ohci_hc_driver;

static const struct ohci_driver_overrides nexell_ohci_overrides __initconst = {
	.product_desc = "Nexell S5P6818 OHCI controller",
	.extra_priv_size = sizeof(struct nexell_ohci),
};

static int nexell_ohci_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct resource *res;
	struct nexell_ohci *priv;
	int irq;
	int ret;

	if (usb_disabled())
		return -ENODEV;

	ret = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	hcd = usb_create_hcd(&nexell_ohci_hc_driver, &pdev->dev,
			     dev_name(&pdev->dev));
	if (!hcd)
		return -ENOMEM;

	priv = hcd_to_nexell_ohci(hcd);
	priv->phy = devm_phy_get(&pdev->dev, "usb2-phy");
	if (IS_ERR(priv->phy)) {
		ret = PTR_ERR(priv->phy);
		goto err_put_hcd;
	}
	ret = phy_init(priv->phy);
	if (ret)
		goto err_put_hcd;
	ret = phy_power_on(priv->phy);
	if (ret)
		goto err_exit_phy;

	hcd->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(hcd->regs)) {
		ret = PTR_ERR(hcd->regs);
		goto err_power_off_phy;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	platform_set_drvdata(pdev, hcd);
	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret)
		goto err_power_off_phy;

	device_wakeup_enable(hcd->self.controller);
	return 0;

err_power_off_phy:
	phy_power_off(priv->phy);
err_exit_phy:
	phy_exit(priv->phy);
err_put_hcd:
	usb_put_hcd(hcd);
	return ret;
}

static void nexell_ohci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct nexell_ohci *priv = hcd_to_nexell_ohci(hcd);

	usb_remove_hcd(hcd);
	phy_power_off(priv->phy);
	phy_exit(priv->phy);
	usb_put_hcd(hcd);
}

static const struct of_device_id nexell_ohci_ids[] = {
	{ .compatible = "nexell,s5p6818-ohci" },
	{ }
};
MODULE_DEVICE_TABLE(of, nexell_ohci_ids);

static struct platform_driver nexell_ohci_driver = {
	.probe = nexell_ohci_probe,
	.remove_new = nexell_ohci_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		.name = "nexell-ohci",
		.of_match_table = nexell_ohci_ids,
	},
};

static int __init nexell_ohci_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	ohci_init_driver(&nexell_ohci_hc_driver, &nexell_ohci_overrides);
	return platform_driver_register(&nexell_ohci_driver);
}
module_init(nexell_ohci_init);

static void __exit nexell_ohci_exit(void)
{
	platform_driver_unregister(&nexell_ohci_driver);
}
module_exit(nexell_ohci_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
