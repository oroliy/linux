// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nexell S5P6818 Common Clock Framework provider.
 *
 * The boot ROM and U-Boot establish the PLL and core divider state before
 * Linux is entered.  Linux therefore starts by observing that state and
 * only changes the peripheral CLKGEN source/divider and gate registers.
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include <dt-bindings/clock/nexell,s5p6818-clock.h>

#define NEXELL_CLKPWR_PLL0	0x008
#define NEXELL_CLKPWR_DVOREG_BUS	0x024
#define NEXELL_CLKPWR_PLL_SSCG0	0x048

#define NEXELL_CLKGEN_ENABLE	0x000
#define NEXELL_CLKGEN_STAGE0	0x004
#define NEXELL_CLKGEN_STAGE_STRIDE	0x008

#define CLKGEN_SRC_SHIFT	2
#define CLKGEN_SRC_MASK		GENMASK(4, 2)
#define CLKGEN_DIV_SHIFT	5
#define CLKGEN_DIV_MASK		GENMASK(12, 5)
#define CLKGEN_BCLK_GATE_MASK	GENMASK(1, 0)
#define CLKGEN_PCLK_GATE	BIT(3)
#define CLKGEN_MODULE_GATE	BIT(2)

#define NEXELL_MAX_DIV		256U
#define NEXELL_MAX_MAPPED_RESOURCES	16

struct nexell_clkgen_desc {
	unsigned int id;
	const char *name;
	const char *resource_name;
	u8 stage;
	bool gate_bclk;
	bool gate_pclk;
	bool is_bus_pclk;
};

struct nexell_clkgen_res {
	const char *name;
	void __iomem *base;
	u32 gate_mask;
	unsigned int gate_refs;
};

struct nexell_clkgen {
	struct clk_hw hw;
	struct nexell_clkgen_res *res;
	u8 stage;
	spinlock_t *lock;
};

struct nexell_clkctrl {
	struct device *dev;
	struct platform_device *pdev;
	void __iomem *clkpwr;
	spinlock_t lock;
	struct clk_hw_onecell_data *onecell;
	struct nexell_clkgen_res mapped[NEXELL_MAX_MAPPED_RESOURCES];
	unsigned int mapped_count;
};

static const char * const nexell_parent_names[] = {
	/* CLKGEN source encoding 0..3 is PLL0..PLL3; source 4 is XIN. */
	"pll0", "pll1", "pll2", "pll3", "xin",
};

static inline struct nexell_clkgen *to_nexell_clkgen(struct clk_hw *hw)
{
	return container_of(hw, struct nexell_clkgen, hw);
}

static inline void __iomem *clkgen_stage_reg(struct nexell_clkgen *clk)
{
	return clk->res->base + NEXELL_CLKGEN_STAGE0 +
	       clk->stage * NEXELL_CLKGEN_STAGE_STRIDE;
}

static unsigned long nexell_parent_rate(struct clk_hw *hw, unsigned int index)
{
	struct clk_hw *parent;

	parent = clk_hw_get_parent_by_index(hw, index);
	return parent ? clk_hw_get_rate(parent) : 0;
}

static unsigned long nexell_clkgen_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct nexell_clkgen *clk = to_nexell_clkgen(hw);
	u32 value;
	unsigned int div;

	value = readl(clkgen_stage_reg(clk));
	div = ((value & CLKGEN_DIV_MASK) >> CLKGEN_DIV_SHIFT) + 1;

	return parent_rate / div;
}

static u8 nexell_clkgen_get_parent(struct clk_hw *hw)
{
	struct nexell_clkgen *clk = to_nexell_clkgen(hw);

	return (readl(clkgen_stage_reg(clk)) & CLKGEN_SRC_MASK) >>
	       CLKGEN_SRC_SHIFT;
}

static int nexell_clkgen_set_parent(struct clk_hw *hw, u8 index)
{
	struct nexell_clkgen *clk = to_nexell_clkgen(hw);
	unsigned long flags;
	u32 value;

	if (index >= ARRAY_SIZE(nexell_parent_names))
		return -EINVAL;

	spin_lock_irqsave(clk->lock, flags);
	value = readl(clkgen_stage_reg(clk));
	value &= ~CLKGEN_SRC_MASK;
	value |= FIELD_PREP(CLKGEN_SRC_MASK, index);
	writel(value, clkgen_stage_reg(clk));
	readl(clkgen_stage_reg(clk));
	spin_unlock_irqrestore(clk->lock, flags);

	return 0;
}

static int nexell_clkgen_determine_rate(struct clk_hw *hw,
					struct clk_rate_request *req)
{
	unsigned long best_rate = 0, best_diff = ULONG_MAX;
	struct clk_hw *best_parent = NULL;
	unsigned int i, div;

	for (i = 0; i < ARRAY_SIZE(nexell_parent_names); i++) {
		unsigned long parent_rate = nexell_parent_rate(hw, i);

		if (!parent_rate)
			continue;

		for (div = 1; div <= NEXELL_MAX_DIV; div++) {
			unsigned long rate = parent_rate / div;
			unsigned long diff = (rate > req->rate) ?
				(rate - req->rate) : (req->rate - rate);

			if (diff < best_diff) {
				best_diff = diff;
				best_rate = rate;
				best_parent = clk_hw_get_parent_by_index(hw, i);
			}
		}
	}

	if (!best_parent)
		return -EINVAL;

	req->best_parent_hw = best_parent;
	req->best_parent_rate = clk_hw_get_rate(best_parent);
	req->rate = best_rate;

	return 0;
}

static int nexell_clkgen_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	struct nexell_clkgen *clk = to_nexell_clkgen(hw);
	unsigned long flags;
	u32 value;
	unsigned int div;

	if (!rate || !parent_rate)
		return -EINVAL;

	div = DIV_ROUND_CLOSEST(parent_rate, rate);
	div = clamp_t(unsigned int, div, 1, NEXELL_MAX_DIV);

	spin_lock_irqsave(clk->lock, flags);
	value = readl(clkgen_stage_reg(clk));
	value &= ~CLKGEN_DIV_MASK;
	value |= FIELD_PREP(CLKGEN_DIV_MASK, div - 1);
	writel(value, clkgen_stage_reg(clk));
	readl(clkgen_stage_reg(clk));
	spin_unlock_irqrestore(clk->lock, flags);

	return 0;
}

/*
 * Every clock derived from one CLKGEN window shares the single module-gate
 * bit in CLKENB (gmac/gmac-tx and usbhost/usbhost-ref are pairs like
 * this).  Gate transitions must follow the per-resource reference count,
 * not each clock's own enable count, or disabling one stage clock would
 * gate off its sibling.
 */
static int nexell_clkgen_enable(struct clk_hw *hw)
{
	struct nexell_clkgen *clk = to_nexell_clkgen(hw);
	struct nexell_clkgen_res *res = clk->res;
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(clk->lock, flags);
	if (res->gate_refs++ == 0) {
		value = readl(res->base + NEXELL_CLKGEN_ENABLE);
		value |= res->gate_mask;
		writel(value, res->base + NEXELL_CLKGEN_ENABLE);
		readl(res->base + NEXELL_CLKGEN_ENABLE);
	}
	spin_unlock_irqrestore(clk->lock, flags);

	return 0;
}

static void nexell_clkgen_disable(struct clk_hw *hw)
{
	struct nexell_clkgen *clk = to_nexell_clkgen(hw);
	struct nexell_clkgen_res *res = clk->res;
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(clk->lock, flags);
	if (!res->gate_refs) {
		spin_unlock_irqrestore(clk->lock, flags);
		return;
	}
	if (--res->gate_refs == 0) {
		value = readl(res->base + NEXELL_CLKGEN_ENABLE);
		value &= ~res->gate_mask;
		writel(value, res->base + NEXELL_CLKGEN_ENABLE);
		readl(res->base + NEXELL_CLKGEN_ENABLE);
	}
	spin_unlock_irqrestore(clk->lock, flags);
}

static const struct clk_ops nexell_clkgen_ops = {
	.enable = nexell_clkgen_enable,
	.disable = nexell_clkgen_disable,
	.get_parent = nexell_clkgen_get_parent,
	.set_parent = nexell_clkgen_set_parent,
	.recalc_rate = nexell_clkgen_recalc_rate,
	.determine_rate = nexell_clkgen_determine_rate,
	.set_rate = nexell_clkgen_set_rate,
};

static const char * const nexell_bus_pclk_parent[] = {
	"bus_pclk",
};

static unsigned long nexell_pclk_gate_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	return parent_rate;
}

static const struct clk_ops nexell_pclk_gate_ops = {
	.enable = nexell_clkgen_enable,
	.disable = nexell_clkgen_disable,
	.recalc_rate = nexell_pclk_gate_recalc_rate,
};

static unsigned long nexell_pll_rate(void __iomem *base, unsigned int pll,
					     unsigned long xin_rate)
{
	u32 value, sscg;
	u32 p, m, s, k;
	u64 rate;

	value = readl(base + NEXELL_CLKPWR_PLL0 + pll * sizeof(u32));
	sscg = readl(base + NEXELL_CLKPWR_PLL_SSCG0 + pll * sizeof(u32));
	p = (value >> 18) & 0x3f;
	m = (value >> 8) & 0x3ff;
	s = value & 0xff;
	k = (sscg >> 16) & 0xffff;

	if (!p || !m)
		return 0;

	rate = div_u64((u64)xin_rate * m, p);
	rate >>= s;
	if (pll > 1 && k)
		rate += div_u64((u64)xin_rate * k, p * 65536ULL) >> s;

	return (unsigned long)rate;
}

static unsigned long nexell_bus_pclk_rate(void __iomem *base,
					  struct clk_hw_onecell_data *onecell)
{
	u32 val = readl(base + NEXELL_CLKPWR_DVOREG_BUS);
	unsigned int pll = val & 0x7;
	unsigned int div0 = ((val >> 3) & 0x3f) + 1;
	unsigned int div1 = ((val >> 9) & 0x3f) + 1;
	unsigned long pll_rate = 0;

	if (pll < 4 && !IS_ERR_OR_NULL(onecell->hws[NEXELL_CLK_PLL0 + pll]))
		pll_rate = clk_hw_get_rate(onecell->hws[NEXELL_CLK_PLL0 + pll]);

	if (!pll_rate || !div0 || !div1)
		return 100000000;

	return (pll_rate / div0) / div1;
}

static const struct nexell_clkgen_desc nexell_clkgen_descs[] = {
	{ NEXELL_CLK_UART0, "uart0", "uart0", 0, false, false, false },
	{ NEXELL_CLK_TIMER0, "timer0", "timer0", 0, false, false, false },
	{ NEXELL_CLK_TIMER1, "timer1", "timer1", 0, false, false, false },
	{ NEXELL_CLK_SDMMC0, "sdmmc0", "sdmmc0", 0, false, true, false },
	{ NEXELL_CLK_SDMMC1, "sdmmc1", "sdmmc1", 0, false, true, false },
	{ NEXELL_CLK_SDMMC2, "sdmmc2", "sdmmc2", 0, false, true, false },
	{ NEXELL_CLK_GMAC, "gmac", "gmac", 0, false, false, false },
	{ NEXELL_CLK_GMAC_TX, "gmac-tx", "gmac", 1, false, false, false },
	{ NEXELL_CLK_USBHOST, "usbhost", "usbhost", 0, true, false, false },
	{ NEXELL_CLK_USBHOST_REF, "usbhost-ref", "usbhost", 1, false, false, false },
	{ NEXELL_CLK_MIPI, "mipi", "mipi", 0, false, false, false },
	{ NEXELL_CLK_I2C0, "i2c0", "i2c0", 0, false, true, true },
	{ NEXELL_CLK_I2C1, "i2c1", "i2c1", 0, false, true, true },
	{ NEXELL_CLK_I2C2, "i2c2", "i2c2", 0, false, true, true },
};

static int nexell_clkgen_register(struct nexell_clkctrl *ctrl,
					const struct nexell_clkgen_desc *desc)
{
	struct nexell_clkgen *clk;
	struct resource *res;
	struct clk_init_data init = { };
	struct nexell_clkgen_res *mapped = NULL;
	u32 gate_mask = CLKGEN_MODULE_GATE;
	unsigned int i;
	int ret;

	clk = devm_kzalloc(ctrl->dev, sizeof(*clk), GFP_KERNEL);
	if (!clk)
		return -ENOMEM;

	for (i = 0; i < ctrl->mapped_count; i++)
		if (!strcmp(ctrl->mapped[i].name, desc->resource_name)) {
			mapped = &ctrl->mapped[i];
			break;
		}
	if (!mapped) {
		if (ctrl->mapped_count >= ARRAY_SIZE(ctrl->mapped))
			return -E2BIG;
		res = platform_get_resource_byname(ctrl->pdev, IORESOURCE_MEM,
						   desc->resource_name);
		if (!res)
			return -EINVAL;
		mapped = &ctrl->mapped[ctrl->mapped_count++];
		mapped->name = desc->resource_name;
		mapped->base = devm_ioremap_resource(ctrl->dev, res);
		if (IS_ERR(mapped->base))
			return PTR_ERR(mapped->base);
		mapped->gate_refs = 0;
		mapped->gate_mask = 0;
	}

	/*
	 * Union the gate bits every sibling clock requires so the shared
	 * gate always satisfies all of its consumers.
	 */
	if (desc->gate_bclk)
		gate_mask |= CLKGEN_BCLK_GATE_MASK;
	if (desc->gate_pclk)
		gate_mask |= CLKGEN_PCLK_GATE;
	mapped->gate_mask |= gate_mask;

	clk->res = mapped;
	clk->stage = desc->stage;
	clk->lock = &ctrl->lock;
	init.name = desc->name;
	if (desc->is_bus_pclk) {
		init.ops = &nexell_pclk_gate_ops;
		init.parent_names = nexell_bus_pclk_parent;
		init.num_parents = ARRAY_SIZE(nexell_bus_pclk_parent);
		init.flags = CLK_SET_RATE_PARENT;
	} else {
		init.ops = &nexell_clkgen_ops;
		init.parent_names = nexell_parent_names;
		init.num_parents = ARRAY_SIZE(nexell_parent_names);
		init.flags = CLK_SET_RATE_PARENT;
	}
	clk->hw.init = &init;

	ret = devm_clk_hw_register(ctrl->dev, &clk->hw);
	if (ret)
		return ret;

	ctrl->onecell->hws[desc->id] = &clk->hw;
	return 0;
}

static int nexell_clk_probe(struct platform_device *pdev)
{
	struct nexell_clkctrl *ctrl;
	struct clk *xin;
	struct clk_hw *bus_pclk_hw;
	unsigned long bus_pclk;
	unsigned int i;
	int ret;

	ctrl = devm_kzalloc(&pdev->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->pdev = pdev;
	ctrl->clkpwr = devm_platform_ioremap_resource_byname(pdev, "clkpwr");
	if (IS_ERR(ctrl->clkpwr))
		return PTR_ERR(ctrl->clkpwr);

	ctrl->dev = &pdev->dev;
	spin_lock_init(&ctrl->lock);
	ctrl->onecell = devm_kzalloc(&pdev->dev,
			struct_size(ctrl->onecell, hws, NEXELL_CLK_MAX), GFP_KERNEL);
	if (!ctrl->onecell)
		return -ENOMEM;
	ctrl->onecell->num = NEXELL_CLK_MAX;
	for (i = 0; i < NEXELL_CLK_MAX; i++)
		ctrl->onecell->hws[i] = ERR_PTR(-ENOENT);

	xin = devm_clk_get(&pdev->dev, "xin");
	if (IS_ERR(xin))
		return PTR_ERR(xin);
	ctrl->onecell->hws[NEXELL_CLK_XIN] = __clk_get_hw(xin);

	for (i = 0; i < 4; i++) {
		char *name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "pll%u", i);
		unsigned long rate = nexell_pll_rate(ctrl->clkpwr, i,
							clk_get_rate(xin));

		if (!name)
			return -ENOMEM;
		/*
		 * A powered-down PLL reads P=0/M=0.  Skip it instead of
		 * failing the whole controller; consumers of its clocks
		 * simply keep their ERR_PTR(-ENOENT) entries.
		 */
		if (!rate) {
			dev_warn(&pdev->dev,
				 "PLL%u reads 0 Hz (powered down?); skipped\n",
				 i);
			continue;
		}
		ctrl->onecell->hws[NEXELL_CLK_PLL0 + i] =
			devm_clk_hw_register_fixed_rate(&pdev->dev, name, "xin", 0,
							 rate);
		if (IS_ERR(ctrl->onecell->hws[NEXELL_CLK_PLL0 + i]))
			return PTR_ERR(ctrl->onecell->hws[NEXELL_CLK_PLL0 + i]);
	}

	bus_pclk = nexell_bus_pclk_rate(ctrl->clkpwr, ctrl->onecell);
	bus_pclk_hw = devm_clk_hw_register_fixed_rate(&pdev->dev, "bus_pclk",
						       NULL, 0, bus_pclk);
	if (IS_ERR(bus_pclk_hw))
		return PTR_ERR(bus_pclk_hw);

	for (i = 0; i < ARRAY_SIZE(nexell_clkgen_descs); i++) {
		ret = nexell_clkgen_register(ctrl, &nexell_clkgen_descs[i]);
		if (ret)
			return ret;
	}

	ret = devm_of_clk_add_hw_provider(&pdev->dev, of_clk_hw_onecell_get,
						ctrl->onecell);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "S5P6818 clocks registered (PLL0=%lu Hz PLL1=%lu Hz BUS_PCLK=%lu Hz)\n",
		 clk_hw_get_rate(ctrl->onecell->hws[NEXELL_CLK_PLL0]),
		 clk_hw_get_rate(ctrl->onecell->hws[NEXELL_CLK_PLL1]),
		 bus_pclk);
	return 0;
}

static const struct of_device_id nexell_clk_of_match[] = {
	{ .compatible = "nexell,s5p6818-clk" },
	{ }
};
MODULE_DEVICE_TABLE(of, nexell_clk_of_match);

static struct platform_driver nexell_clk_driver = {
	.probe = nexell_clk_probe,
	.driver = {
		.name = "s5p6818-clk",
		.of_match_table = nexell_clk_of_match,
	},
};
builtin_platform_driver(nexell_clk_driver);

MODULE_DESCRIPTION("Nexell S5P6818 clock controller");
MODULE_LICENSE("GPL");
