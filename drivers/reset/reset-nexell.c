// SPDX-License-Identifier: GPL-2.0-only
/* Reset controller for the Nexell S5P6818 SoC. */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>

struct nexell_reset {
	void __iomem *base;
	spinlock_t lock;
	struct reset_controller_dev rcdev;
};

static inline struct nexell_reset *to_nexell_reset(
		struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct nexell_reset, rcdev);
}

static int nexell_reset_update(struct reset_controller_dev *rcdev,
			       unsigned long id, bool assert)
{
	struct nexell_reset *reset = to_nexell_reset(rcdev);
	unsigned int bank = id / (sizeof(u32) * BITS_PER_BYTE);
	unsigned int bit = id % (sizeof(u32) * BITS_PER_BYTE);
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&reset->lock, flags);
	value = readl(reset->base + bank * sizeof(u32));
	if (assert)
		value &= ~BIT(bit);
	else
		value |= BIT(bit);
	writel(value, reset->base + bank * sizeof(u32));
	spin_unlock_irqrestore(&reset->lock, flags);

	return 0;
}

static int nexell_reset_assert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	return nexell_reset_update(rcdev, id, true);
}

static int nexell_reset_deassert(struct reset_controller_dev *rcdev,
				 unsigned long id)
{
	return nexell_reset_update(rcdev, id, false);
}

static int nexell_reset_reset(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	int ret;

	ret = nexell_reset_assert(rcdev, id);
	if (ret)
		return ret;

	mdelay(1);

	return nexell_reset_deassert(rcdev, id);
}

static int nexell_reset_status(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct nexell_reset *reset = to_nexell_reset(rcdev);
	unsigned int bank = id / (sizeof(u32) * BITS_PER_BYTE);
	unsigned int bit = id % (sizeof(u32) * BITS_PER_BYTE);

	return !(readl(reset->base + bank * sizeof(u32)) & BIT(bit));
}

static const struct reset_control_ops nexell_reset_ops = {
	.assert = nexell_reset_assert,
	.deassert = nexell_reset_deassert,
	.reset = nexell_reset_reset,
	.status = nexell_reset_status,
};

static const struct of_device_id nexell_reset_dt_ids[] = {
	{ .compatible = "nexell,s5p6818-reset" },
	{ }
};
MODULE_DEVICE_TABLE(of, nexell_reset_dt_ids);

static int nexell_reset_probe(struct platform_device *pdev)
{
	struct nexell_reset *reset;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	reset = devm_kzalloc(&pdev->dev, sizeof(*reset), GFP_KERNEL);
	if (!reset)
		return -ENOMEM;

	reset->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reset->base))
		return PTR_ERR(reset->base);

	spin_lock_init(&reset->lock);
	reset->rcdev.owner = THIS_MODULE;
	reset->rcdev.ops = &nexell_reset_ops;
	reset->rcdev.of_node = pdev->dev.of_node;
	reset->rcdev.nr_resets = resource_size(res) * BITS_PER_BYTE;
	dev_info(&pdev->dev, "Nexell reset controller ready: %u lines\n",
		 reset->rcdev.nr_resets);

	return devm_reset_controller_register(&pdev->dev, &reset->rcdev);
}

static struct platform_driver nexell_reset_driver = {
	.probe = nexell_reset_probe,
	.driver = {
		.name = "nexell-reset",
		.of_match_table = nexell_reset_dt_ids,
	},
};
builtin_platform_driver(nexell_reset_driver);

MODULE_DESCRIPTION("Nexell S5P6818 reset controller");
MODULE_LICENSE("GPL");
