// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5P6818 display scanout handoff.
 *
 * The U-Boot values below are kept explicit until the native Linux display
 * driver grows a proper DRM mode/state object.  This stage owns the scanout
 * programming, while MIPI DSI and panel reset remain unchanged.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reset.h>

#define S5P6818_DISPLAY_RESET_COUNT	7

static const char * const s5p6818_display_reset_names[] = {
	"disp-top", "display", "mipi", "mipi-dsi", "mipi-csi",
	"mipi-phy-s", "mipi-phy-m",
};

struct s5p6818_display {
	void __iomem *top;
	void __iomem *mlc;
	void __iomem *dpc;
	void __iomem *clkgen;
	void __iomem *mipi;
	void __iomem *tieoff;
	struct reset_control_bulk_data resets[S5P6818_DISPLAY_RESET_COUNT];
};

static void s5p6818_set_clock_source_divisor(void __iomem *reg, u32 source,
						     u32 divisor)
{
	u32 value = readl(reg);

	value &= ~((0x7 << 2) | (0xff << 5));
	value |= (source & 0x7) << 2;
	value |= (divisor - 1) << 5;
	writel(value, reg);
}

static void s5p6818_display_program_scanout(
		struct s5p6818_display *display)
{
	u32 value;

	/* Display-top: primary MLC to MIPI, and primary MLC as the pad source. */
	writel(0x80000000, display->top + 0x08);
	writel(0x00000000, display->top + 0x24);

	/* MLC0 top and RGB layer 0: 1024x600 x8r8g8b8 at the U-Boot FB. */
	value = readl(display->mlc + 0x00);
	value |= BIT(1) | BIT(10) | BIT(11);
	value &= ~BIT(3);
	writel(value, display->mlc + 0x00);
	writel(0x025703ff, display->mlc + 0x04);
	writel(0x00000000, display->mlc + 0x08);
	writel(0x000003ff, display->mlc + 0x0c);
	writel(0x00000257, display->mlc + 0x10);
	writel(0x00000000, display->mlc + 0x14);
	writel(0x00000000, display->mlc + 0x18);
	writel(0x00000000, display->mlc + 0x1c);
	writel(0x00000000, display->mlc + 0x20);
	value = readl(display->mlc + 0x24);
	value &= ~(0xffff0000 | BIT(4) | BIT(5));
	value |= 0x46530000 | BIT(5);
	writel(value, display->mlc + 0x24);
	writel(4, display->mlc + 0x28);
	writel(4096, display->mlc + 0x2c);
	writel(0x46000000, display->mlc + 0x38);
	value = readl(display->mlc + 0x3c0);
	value &= ~0x0f;
	value |= 0x0b; /* BCLK always on, PCLK always on. */
	writel(value, display->mlc + 0x3c0);

	/* DPC0 timing and RGB888 output, matching the validated U-Boot mode. */
	writel(0x000004f9, display->dpc + 0xf8);
	writel(0x00000001, display->dpc + 0xfc);
	writel(0x00000051, display->dpc + 0x100);
	writel(0x00000450, display->dpc + 0x104);
	writel(0x0000027a, display->dpc + 0x108);
	writel(0x00000002, display->dpc + 0x10c);
	writel(0x00000016, display->dpc + 0x110);
	writel(0x0000026b, display->dpc + 0x114);
	value = readl(display->dpc + 0x118);
	value &= ~BIT(10);
	value |= BIT(12) | BIT(15);
	writel(value, display->dpc + 0x118);
	value = readl(display->dpc + 0x11c);
	value &= ~0x3fff;
	value |= 0x2300;
	writel(value, display->dpc + 0x11c);
	writel(0x00000000, display->dpc + 0x130);
	s5p6818_set_clock_source_divisor(display->dpc + 0x3c4, 2, 12);
	s5p6818_set_clock_source_divisor(display->dpc + 0x3cc, 7, 1);
	value = readl(display->dpc + 0x3c0);
	value |= BIT(2) | BIT(3);
	writel(value, display->dpc + 0x3c0);

	/* CLKGEN2 stage 1 is the MIPI video clock: source 2, divisor 12. */
	value = readl(display->clkgen + 0x00);
	value |= BIT(2) | BIT(3);
	writel(value, display->clkgen + 0x00);
	s5p6818_set_clock_source_divisor(display->clkgen + 0x0c, 2, 12);

	/* Commit the new plane state after all address/timing fields are visible. */
	wmb();
	value = readl(display->mlc + 0x24);
	writel(value | BIT(4), display->mlc + 0x24);
	value = readl(display->mlc + 0x00);
	writel(value | BIT(3), display->mlc + 0x00);
	wmb();
}

static int s5p6818_mipi_software_reset(void __iomem *mipi)
{
	u32 status;
	int ret;

	writel(0x00010001, mipi + 0x204);
	ret = readl_poll_timeout(mipi + 0x200, status,
				 !(status & BIT(20)), 1, 100000);
	writel(0x00000000, mipi + 0x204);

	return ret;
}

static int s5p6818_display_program_mipi_host(
		struct s5p6818_display *display)
{
	u32 value;
	int ret;

	/* Match U-Boot's MIPI SRAM tie-off and D-PHY analog setup. */
	value = readl(display->tieoff + 0x10);
	value &= ~GENMASK(6, 1);
	value |= (3 << 1) | (3 << 4);
	writel(value, display->tieoff + 0x10);
	writel(0x00000000, display->mipi + 0x24); /* CSIS_DPHYCTRL_1 */
	writel(22 << 24, display->mipi + 0x04); /* CSIS_DPHYCTRL */

	/* Keep the validated U-Boot PLL setup intact. */
	value = BIT(23) | (0x2281 << 1) | (0x8 << 24);
	writel(0x00000000, display->mipi + 0x254); /* PHYACCHR */
	writel(0xffffffff, display->mipi + 0x250); /* PLL stable timer */
	writel(0x00000000, display->mipi + 0x258); /* PHYACCHR1 */
	value |= readl(display->mipi + 0x24c) & ~0x0fffffff;
	writel(value, display->mipi + 0x24c); /* 480 MHz HS PLL */
	writel(0x1118000a, display->mipi + 0x208);

	ret = s5p6818_mipi_software_reset(display->mipi);
	if (ret)
		return ret;

	writel(0x91f80001, display->mipi + 0x208);

	/* Four data lanes plus the clock lane, matching U-Boot's 0x7f. */
	value = readl(display->mipi + 0x210);
	value &= ~0xff;
	value |= 0x7f;
	writel(value, display->mipi + 0x210);
	writel(0x00000000, display->mipi + 0x258);

	/* Video burst, RGB888, event sync, and the validated panel porches. */
	value = readl(display->mipi + 0x210);
	value &= ~0xffffff00;
	value |= 0x06807000;
	writel(value, display->mipi + 0x210);
	writel(0x000c0014, display->mipi + 0x21c); /* VFP=12, VBP=20 */
	writel(0x00a80050, display->mipi + 0x220); /* HFP=168, HBP=80 */
	writel(0x00c00002, display->mipi + 0x224); /* VS=3, HS=2 */
	value = readl(display->mipi + 0x218);
	value &= ~0x0fffffff;
	value |= 0x02580400;
	writel(value, display->mipi + 0x218); /* 1024x600, disabled for now */

	/* MIPI video clock: CLKGEN2 stage 1, source 2, divisor 12. */
	value = readl(display->clkgen + 0x00);
	value |= BIT(2) | BIT(3);
	writel(value, display->clkgen + 0x00);
	s5p6818_set_clock_source_divisor(display->clkgen + 0x0c, 2, 12);

	/* Keep the host in the active HS/video state used by U-Boot. */
	writel(0x80000000, display->top + 0x08);
	value = readl(display->mipi + 0x218);
	writel(value | BIT(31), display->mipi + 0x218);
	wmb();

	return 0;
}

static int s5p6818_display_probe(struct platform_device *pdev)
{
	struct s5p6818_display display;
	u32 top_mux;
	u32 mlc_control;
	u32 mlc_size;
	u32 mlc_layer;
	u32 mlc_address;
	u32 mlc_hstride;
	u32 mlc_vstride;
	u32 dpc_control;
	u32 dpc_hactive;
	u32 dpc_vactive;
	u32 clkgen_enable;
	u32 clkgen_stage1;
	u32 mipi_status;
	u32 dpc_clock;
	u32 dpc_clock_stage1;
	int ret;

	display.top = devm_platform_ioremap_resource_byname(pdev, "top");
	if (IS_ERR(display.top))
		return PTR_ERR(display.top);

	display.mlc = devm_platform_ioremap_resource_byname(pdev, "mlc");
	if (IS_ERR(display.mlc))
		return PTR_ERR(display.mlc);

	display.dpc = devm_platform_ioremap_resource_byname(pdev, "dpc");
	if (IS_ERR(display.dpc))
		return PTR_ERR(display.dpc);

	display.clkgen = devm_platform_ioremap_resource_byname(pdev, "clkgen");
	if (IS_ERR(display.clkgen))
		return PTR_ERR(display.clkgen);

	display.mipi = devm_platform_ioremap_resource_byname(pdev, "mipi");
	if (IS_ERR(display.mipi))
		return PTR_ERR(display.mipi);

	display.tieoff = devm_platform_ioremap_resource_byname(pdev, "tieoff");
	if (IS_ERR(display.tieoff))
		return PTR_ERR(display.tieoff);

	for (ret = 0; ret < S5P6818_DISPLAY_RESET_COUNT; ret++)
		display.resets[ret].id = s5p6818_display_reset_names[ret];
	ret = devm_reset_control_bulk_get_exclusive(&pdev->dev,
						   S5P6818_DISPLAY_RESET_COUNT,
						   display.resets);
	if (ret)
		return ret;

	/* These offsets mirror the validated U-Boot S5PXX18 register layouts. */
	top_mux = readl(display.top + 0x08);
	mlc_control = readl(display.mlc + 0x00);
	mlc_size = readl(display.mlc + 0x04);
	mlc_layer = readl(display.mlc + 0x24);
	mlc_hstride = readl(display.mlc + 0x28);
	mlc_vstride = readl(display.mlc + 0x2c);
	mlc_address = readl(display.mlc + 0x38);
	dpc_hactive = readl(display.dpc + 0x100);
	dpc_vactive = readl(display.dpc + 0x110);
	dpc_control = readl(display.dpc + 0x118);
	clkgen_enable = readl(display.clkgen + 0x00);
	clkgen_stage1 = readl(display.clkgen + 0x0c);
	mipi_status = readl(display.mipi + 0x200);
	dpc_clock = readl(display.dpc + 0x3c0);
	dpc_clock_stage1 = readl(display.dpc + 0x3cc);

	dev_dbg(&pdev->dev,
		"x6818 native display handoff: top=0x%08x mlc=0x%08x "
		"size=0x%08x layer=0x%08x addr=0x%08x stride=%u/%u\n",
		top_mux, mlc_control, mlc_size, mlc_layer, mlc_address,
		mlc_hstride, mlc_vstride);
	dev_dbg(&pdev->dev,
		"x6818 native display handoff: dpc=0x%08x hactive=0x%08x "
		"vactive=0x%08x dpcclk=0x%08x/0x%08x "
		"clkgen=0x%08x/0x%08x mipi=0x%08x\n",
		dpc_control, dpc_hactive, dpc_vactive, dpc_clock,
		dpc_clock_stage1, clkgen_enable, clkgen_stage1, mipi_status);

	ret = (top_mux == 0x80000000 && mlc_address == 0x46000000 &&
	       mlc_hstride == 4 && mlc_vstride == 4096) ? 0 : -EINVAL;
	if (ret) {
		dev_warn(&pdev->dev,
			 "x6818 native display handoff: unexpected scanout state\n");
		return ret;
	}

	dev_info(&pdev->dev, "x6818 native display handoff OK\n");

	ret = reset_control_bulk_assert(S5P6818_DISPLAY_RESET_COUNT,
						display.resets);
	if (ret)
		return ret;
	udelay(1);
	ret = reset_control_bulk_deassert(S5P6818_DISPLAY_RESET_COUNT,
						  display.resets);
	if (ret)
		return ret;

	s5p6818_display_program_scanout(&display);
	ret = s5p6818_display_program_mipi_host(&display);
	if (ret) {
		dev_err(&pdev->dev,
			"x6818 native display MIPI host reset timed out: %d\n", ret);
		return ret;
	}

	top_mux = readl(display.top + 0x08);
	mlc_control = readl(display.mlc + 0x00);
	mlc_size = readl(display.mlc + 0x04);
	mlc_layer = readl(display.mlc + 0x24);
	mlc_hstride = readl(display.mlc + 0x28);
	mlc_vstride = readl(display.mlc + 0x2c);
	mlc_address = readl(display.mlc + 0x38);
	dpc_control = readl(display.dpc + 0x118);
	dpc_hactive = readl(display.dpc + 0x100);
	dpc_vactive = readl(display.dpc + 0x110);
	clkgen_enable = readl(display.clkgen + 0x00);
	clkgen_stage1 = readl(display.clkgen + 0x0c);
	mipi_status = readl(display.mipi + 0x200);

	dev_dbg(&pdev->dev,
		"x6818 native display MIPI host: status=0x%08x "
		"clk=0x%08x config=0x%08x resol=0x%08x "
		"mvporch=0x%08x mhporch=0x%08x msync=0x%08x\n",
		mipi_status, readl(display.mipi + 0x208),
		readl(display.mipi + 0x210), readl(display.mipi + 0x218),
		readl(display.mipi + 0x21c), readl(display.mipi + 0x220),
		readl(display.mipi + 0x224));

	dev_dbg(&pdev->dev,
		"x6818 native display scanout configured: top=0x%08x "
		"mlc=0x%08x size=0x%08x layer=0x%08x addr=0x%08x "
		"stride=%u/%u dpc=0x%08x hactive=0x%08x vactive=0x%08x "
		"clkgen=0x%08x/0x%08x\n",
		top_mux, mlc_control, mlc_size, mlc_layer, mlc_address,
		mlc_hstride, mlc_vstride, dpc_control, dpc_hactive,
		dpc_vactive, clkgen_enable, clkgen_stage1);

	if (top_mux != 0x80000000 || mlc_address != 0x46000000 ||
	    mlc_hstride != 4 || mlc_vstride != 4096 ||
	    !(dpc_control & BIT(15)) || !(clkgen_enable & BIT(2)) ||
	    (readl(display.mipi + 0x210) & 0xff) != 0x7f ||
	    readl(display.mipi + 0x208) != 0x91f80001 ||
	    readl(display.mipi + 0x218) != 0x82580400) {
		dev_err(&pdev->dev,
			"x6818 native display scanout/MIPI: readback validation FAILED\n");
		return -EIO;
	}

	dev_info(&pdev->dev,
		 "x6818 native display scanout/MIPI host configured OK\n");
	return 0;
}

static const struct of_device_id s5p6818_display_of_match[] = {
	{ .compatible = "nexell,s5p6818-display" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5p6818_display_of_match);

static struct platform_driver s5p6818_display_driver = {
	.probe = s5p6818_display_probe,
	.driver = {
		.name = "s5p6818-display",
		.of_match_table = s5p6818_display_of_match,
	},
};
static int __init s5p6818_display_init(void)
{
	return platform_driver_register(&s5p6818_display_driver);
}
late_initcall(s5p6818_display_init);

static void __exit s5p6818_display_exit(void)
{
	platform_driver_unregister(&s5p6818_display_driver);
}
module_exit(s5p6818_display_exit);

MODULE_DESCRIPTION("Nexell S5P6818 native display handoff probe");
MODULE_LICENSE("GPL");
