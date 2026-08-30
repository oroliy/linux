// SPDX-License-Identifier: GPL-2.0-only
/* Nexell S5P6818 EHCI host controller glue. */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "ehci.h"

#define DRIVER_DESC "Nexell S5P6818 EHCI driver"

struct nexell_ehci {
	struct phy *phy;
};

#define hcd_to_nexell_ehci(hcd) \
	((struct nexell_ehci *)hcd_to_ehci(hcd)->priv)

static struct hc_driver __read_mostly nexell_ehci_hc_driver;

static int nexell_ehci_reset(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	ehci->caps = hcd->regs;
	return ehci_setup(hcd);
}

static const struct ehci_driver_overrides nexell_ehci_overrides __initconst = {
	.reset = nexell_ehci_reset,
	.extra_priv_size = sizeof(struct nexell_ehci),
};

static int nexell_ehci_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct resource *res;
	struct nexell_ehci *priv;
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

	hcd = usb_create_hcd(&nexell_ehci_hc_driver, &pdev->dev,
			     dev_name(&pdev->dev));
	if (!hcd)
		return -ENOMEM;

	priv = hcd_to_nexell_ehci(hcd);
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

static void nexell_ehci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct nexell_ehci *priv = hcd_to_nexell_ehci(hcd);

	usb_remove_hcd(hcd);
	phy_power_off(priv->phy);
	phy_exit(priv->phy);
	usb_put_hcd(hcd);
}

static const struct of_device_id nexell_ehci_ids[] = {
	{ .compatible = "nexell,s5p6818-ehci" },
	{ }
};
MODULE_DEVICE_TABLE(of, nexell_ehci_ids);

static struct platform_driver nexell_ehci_driver = {
	.probe = nexell_ehci_probe,
	.remove_new = nexell_ehci_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		.name = "nexell-ehci",
		.of_match_table = nexell_ehci_ids,
	},
};

static int __init nexell_ehci_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	ehci_init_driver(&nexell_ehci_hc_driver, &nexell_ehci_overrides);
	return platform_driver_register(&nexell_ehci_driver);
}
module_init(nexell_ehci_init);

static void __exit nexell_ehci_exit(void)
{
	platform_driver_unregister(&nexell_ehci_driver);
}
module_exit(nexell_ehci_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
