// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nexell S5P6818 pinctrl and GPIO support.
 *
 * The register layout and the Nexell device-tree properties are shared with
 * the vendor 4.4 BSP and the U-Boot pinctrl driver.  The implementation here
 * uses the current Linux generic pinctrl and gpiolib interfaces.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include "../core.h"
#include "../pinmux.h"

#define NEXELL_GPIO_BANKS	6
#define NEXELL_GPIO_PINS	32
#define NEXELL_ALIVE_PINS	6
#define NEXELL_TOTAL_PINS	((NEXELL_GPIO_BANKS - 1) * \
				 NEXELL_GPIO_PINS + NEXELL_ALIVE_PINS)

#define GPIO_OUT		0x00
#define GPIO_OUTENB		0x04
#define GPIO_DETMODE0		0x08
#define GPIO_DETMODE1		0x0c
#define GPIO_INTENB		0x10
#define GPIO_DET		0x14
#define GPIO_DETMODEEX		0x28
#define GPIO_DETENB		0x3c
#define GPIO_PAD		0x18
#define GPIO_ALTFN0		0x20
#define GPIO_ALTFN1		0x24
#define GPIO_DRV1		0x48
#define GPIO_DRV0		0x50
#define GPIO_PULLSEL		0x58
#define GPIO_PULLENB		0x60
#define GPIO_SLEW_DISABLE	0x44
#define GPIO_DRV1_DISABLE	0x4c
#define GPIO_DRV0_DISABLE	0x54
#define GPIO_PULLSEL_DISABLE	0x5c
#define GPIO_PULLENB_DISABLE	0x64

#define ALIVE_PWRGATE		0x00
#define ALIVE_OUTENB_RESET	0x74
#define ALIVE_OUTENB_SET	0x78
#define ALIVE_OUTENB_READ	0x7c
#define ALIVE_PULLUP_RESET	0x80
#define ALIVE_PULLUP_SET	0x84
#define ALIVE_PULLUP_READ	0x88
#define ALIVE_OUT_RESET		0x8c
#define ALIVE_OUT_SET		0x90
#define ALIVE_OUT_READ		0x94
#define ALIVE_INPUT		0x11c

/* These are private values encoded in a pinctrl_map config entry. */
enum nexell_pinconf_param {
	NEXELL_PINCONF_PULL = PIN_CONFIG_END + 1,
	NEXELL_PINCONF_DRIVE,
	NEXELL_PINCONF_DIRECTION,
	NEXELL_PINCONF_VALUE,
};

struct nexell_gpio_bank {
	struct gpio_chip gc;
	struct irq_chip irq_chip;
	struct pinctrl_gpio_range range;
	void __iomem *base;
	unsigned int pin_base;
	unsigned int npins;
	const char *name;
	u8 index;
	bool alive;
	bool has_irq;
	spinlock_t lock;
};

struct nexell_pinctrl {
	struct device *dev;
	struct pinctrl_desc desc;
	struct pinctrl_dev *pctldev;
	struct pinctrl_pin_desc *pins;
	struct nexell_gpio_bank banks[NEXELL_GPIO_BANKS];
};

static const char * const nexell_bank_names[NEXELL_GPIO_BANKS] = {
	"gpioa", "gpiob", "gpioc", "gpiod", "gpioe", "alive",
};

/*
 * ALTFN value that selects GPIO mode per pin, from the vendor BSP pin
 * tables.  Most pins enter GPIO mode at function 0, but gpioc-0..27,
 * gpiob-11..31 and gpioe-25..31 use function 1 or 2; writing 0 there
 * would leave the pin muxed to a peripheral function.
 */
static const u8 nexell_gpio_altfn[NEXELL_GPIO_BANKS][NEXELL_GPIO_PINS] = {
	{ /* gpioa */
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
	},
	{ /* gpiob */
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		2, 2, 1, 2, 1, 2, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	},
	{ /* gpioc */
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		0, 0, 0, 0,
	},
	{ /* gpiod */
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
	},
	{ /* gpioe */
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1,
	},
	{ /* alive */
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
	},
};

static int nexell_gpio_of_xlate(struct gpio_chip *gc,
				const struct of_phandle_args *spec, u32 *flags)
{
	if (spec->args_count != 2 || spec->args[0] >= gc->ngpio)
		return -EINVAL;
	if (flags)
		*flags = spec->args[1];
	return spec->args[0];
}

static int nexell_read_u32(struct device_node *np, const char *name,
			   u32 *value)
{
	char legacy[32];
	int ret;

	ret = of_property_read_u32(np, name, value);
	if (!ret)
		return 0;

	if (strncmp(name, "nexell,", 7))
		return ret;

	snprintf(legacy, sizeof(legacy), "%s", name + 7);
	return of_property_read_u32(np, legacy, value);
}

static const char *nexell_pins_property(struct device_node *np)
{
	if (of_find_property(np, "nexell,pins", NULL))
		return "nexell,pins";
	if (of_find_property(np, "pins", NULL))
		return "pins";
	return NULL;
}

static int nexell_parse_pin(const char *name, unsigned int *pin)
{
	const char *dash;
	unsigned int number;
	char bank;
	int ret;

	dash = strchr(name, '-');
	if (!dash || dash == name || !dash[1])
		return -EINVAL;

	ret = kstrtouint(dash + 1, 0, &number);
	if (ret)
		return ret;

	if (!strncmp(name, "gpio", 4) && dash == name + 5) {
		bank = name[4];
		if (bank >= 'a' && bank <= 'e' && number < 32) {
			*pin = (bank - 'a') * NEXELL_GPIO_PINS + number;
			return 0;
		}
	}

	if (!strncmp(name, "alive", 5) && dash == name + 5 && number < 6) {
		*pin = 5 * NEXELL_GPIO_PINS + number;
		return 0;
	}

	return -EINVAL;
}

static struct nexell_gpio_bank *nexell_bank_for_pin(
		struct nexell_pinctrl *pc, unsigned int pin,
		unsigned int *offset)
{
	unsigned int bank;

	if (pin < 5 * NEXELL_GPIO_PINS) {
		bank = pin / NEXELL_GPIO_PINS;
		*offset = pin % NEXELL_GPIO_PINS;
	} else if (pin < NEXELL_TOTAL_PINS) {
		bank = 5;
		*offset = pin - 5 * NEXELL_GPIO_PINS;
	} else {
		return NULL;
	}

	return &pc->banks[bank];
}

static void nexell_update_bit(void __iomem *reg, unsigned int bit, bool set)
{
	u32 value = readl(reg);

	if (set)
		value |= BIT(bit);
	else
		value &= ~BIT(bit);
	writel(value, reg);
}

static void nexell_set_function(struct nexell_pinctrl *pc,
				unsigned int pin, unsigned int function)
{
	struct nexell_gpio_bank *bank;
	unsigned int offset;
	void __iomem *reg;
	unsigned long flags;
	u32 value;

	bank = nexell_bank_for_pin(pc, pin, &offset);
	if (!bank || bank->alive)
		return;

	reg = bank->base + (offset < 16 ? GPIO_ALTFN0 : GPIO_ALTFN1);
	spin_lock_irqsave(&bank->lock, flags);
	value = readl(reg);
	value &= ~(0x3 << ((offset % 16) * 2));
	value |= (function & 0x3) << ((offset % 16) * 2);
	writel(value, reg);
	spin_unlock_irqrestore(&bank->lock, flags);
}

static void nexell_set_pull(struct nexell_pinctrl *pc, unsigned int pin,
				    unsigned int pull)
{
	struct nexell_gpio_bank *bank;
	unsigned int offset;
	unsigned long flags;

	bank = nexell_bank_for_pin(pc, pin, &offset);
	if (!bank)
		return;

	spin_lock_irqsave(&bank->lock, flags);
	if (bank->alive) {
		if (pull == 1)
			writel(BIT(offset), bank->base + ALIVE_PULLUP_SET);
		else
			writel(BIT(offset), bank->base + ALIVE_PULLUP_RESET);
	} else {
		nexell_update_bit(bank->base + GPIO_PULLSEL_DISABLE, offset, true);
		nexell_update_bit(bank->base + GPIO_PULLENB_DISABLE, offset, true);
		if (pull == 2) {
			nexell_update_bit(bank->base + GPIO_PULLENB, offset, false);
			nexell_update_bit(bank->base + GPIO_PULLSEL, offset, false);
		} else {
			nexell_update_bit(bank->base + GPIO_PULLSEL, offset,
					  pull == 1);
			nexell_update_bit(bank->base + GPIO_PULLENB, offset, true);
		}
	}
	spin_unlock_irqrestore(&bank->lock, flags);
}

static void nexell_set_drive(struct nexell_pinctrl *pc, unsigned int pin,
				     unsigned int drive)
{
	struct nexell_gpio_bank *bank;
	unsigned int offset;
	unsigned long flags;

	bank = nexell_bank_for_pin(pc, pin, &offset);
	if (!bank || bank->alive)
		return;

	spin_lock_irqsave(&bank->lock, flags);
	nexell_update_bit(bank->base + GPIO_DRV1_DISABLE, offset, true);
	nexell_update_bit(bank->base + GPIO_DRV0_DISABLE, offset, true);
	nexell_update_bit(bank->base + GPIO_DRV1, offset, drive & 1);
	nexell_update_bit(bank->base + GPIO_DRV0, offset, drive & 2);
	spin_unlock_irqrestore(&bank->lock, flags);
}

static void nexell_set_direction(struct nexell_pinctrl *pc, unsigned int pin,
					bool input)
{
	struct nexell_gpio_bank *bank;
	unsigned int offset;
	unsigned long flags;

	bank = nexell_bank_for_pin(pc, pin, &offset);
	if (!bank)
		return;

	spin_lock_irqsave(&bank->lock, flags);
	if (bank->alive) {
		writel(BIT(offset), bank->base +
		       (input ? ALIVE_OUTENB_RESET : ALIVE_OUTENB_SET));
	} else {
		nexell_update_bit(bank->base + GPIO_OUTENB, offset, !input);
	}
	spin_unlock_irqrestore(&bank->lock, flags);
}

static void nexell_set_value(struct nexell_pinctrl *pc, unsigned int pin,
				     bool value)
{
	struct nexell_gpio_bank *bank;
	unsigned int offset;
	unsigned long flags;

	bank = nexell_bank_for_pin(pc, pin, &offset);
	if (!bank)
		return;

	spin_lock_irqsave(&bank->lock, flags);
	if (bank->alive)
		writel(BIT(offset), bank->base +
		       (value ? ALIVE_OUT_SET : ALIVE_OUT_RESET));
	else
		nexell_update_bit(bank->base + GPIO_OUT, offset, value);
	spin_unlock_irqrestore(&bank->lock, flags);
}

static int nexell_get_config(struct nexell_pinctrl *pc, unsigned int pin,
				     unsigned int param, unsigned int *value)
{
	struct nexell_gpio_bank *bank;
	unsigned int offset;
	u32 reg;

	bank = nexell_bank_for_pin(pc, pin, &offset);
	if (!bank)
		return -EINVAL;

	switch (param) {
	case NEXELL_PINCONF_PULL:
		if (bank->alive) {
			*value = !!(readl(bank->base + ALIVE_PULLUP_READ) &
				    BIT(offset));
		} else if (!(readl(bank->base + GPIO_PULLENB) & BIT(offset))) {
			*value = 2;
		} else {
			*value = !!(readl(bank->base + GPIO_PULLSEL) & BIT(offset));
		}
		return 0;
	case NEXELL_PINCONF_DRIVE:
		if (bank->alive)
			return -EOPNOTSUPP;
		*value = !!(readl(bank->base + GPIO_DRV1) & BIT(offset));
		*value |= !!(readl(bank->base + GPIO_DRV0) & BIT(offset)) << 1;
		return 0;
	case NEXELL_PINCONF_DIRECTION:
		if (bank->alive)
			reg = readl(bank->base + ALIVE_OUTENB_READ);
		else
			reg = readl(bank->base + GPIO_OUTENB);
		/* 1 = output, matching the DT "nexell,pin-dir" semantics. */
		*value = !!(reg & BIT(offset));
		return 0;
	case NEXELL_PINCONF_VALUE:
		if (bank->alive)
			reg = readl(bank->base + ALIVE_INPUT);
		else
			reg = readl(bank->base + GPIO_PAD);
		*value = !!(reg & BIT(offset));
		return 0;
	default:
		return -EINVAL;
	}
}

static int nexell_get_group_count(struct pinctrl_dev *pctldev)
{
	return pinctrl_generic_get_group_count(pctldev);
}

static const char *nexell_get_group_name(struct pinctrl_dev *pctldev,
					 unsigned int selector)
{
	return pinctrl_generic_get_group_name(pctldev, selector);
}

static int nexell_get_group_pins(struct pinctrl_dev *pctldev,
					 unsigned int selector,
					 const unsigned int **pins,
					 unsigned int *num_pins)
{
	return pinctrl_generic_get_group_pins(pctldev, selector, pins, num_pins);
}

static int nexell_map_reserve(struct pinctrl_map **map,
				      unsigned int *reserved, unsigned int count)
{
	struct pinctrl_map *new_map;

	if (*reserved >= count)
		return 0;

	new_map = krealloc(*map, count * sizeof(**map), GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	memset(new_map + *reserved, 0,
	       (count - *reserved) * sizeof(*new_map));
	*map = new_map;
	*reserved = count;
	return 0;
}

static int nexell_map_one(struct pinctrl_map **map,
				  unsigned int *reserved, unsigned int *num_maps,
				  const char *group, const char *function,
				  unsigned long *configs, unsigned int num_configs)
{
	struct pinctrl_map *entry;

	if (function) {
		if (nexell_map_reserve(map, reserved, *num_maps + 1))
			return -ENOMEM;
		entry = &(*map)[(*num_maps)++];
		entry->type = PIN_MAP_TYPE_MUX_GROUP;
		entry->data.mux.group = group;
		entry->data.mux.function = function;
	}

	if (!num_configs)
		return 0;

	if (nexell_map_reserve(map, reserved, *num_maps + 1))
		return -ENOMEM;
	entry = &(*map)[(*num_maps)++];
	entry->type = PIN_MAP_TYPE_CONFIGS_GROUP;
	entry->data.configs.group_or_pin = group;
	entry->data.configs.configs = kmemdup(configs,
					     num_configs * sizeof(*configs),
					     GFP_KERNEL);
	if (!entry->data.configs.configs) {
		(*num_maps)--;
		return -ENOMEM;
	}
	entry->data.configs.num_configs = num_configs;
	return 0;
}

static int nexell_subnode_to_map(struct device *dev, struct device_node *np,
					 struct pinctrl_map **map,
					 unsigned int *reserved,
					 unsigned int *num_maps)
{
	const char *pins_property, *pin_name;
	unsigned long *configs = NULL;
	unsigned int num_configs = 0;
	u32 value;
	int count, i, ret;
	bool has_function;

	pins_property = nexell_pins_property(np);
	if (!pins_property)
		return 0;

	has_function = !nexell_read_u32(np, "nexell,pin-function", &value);

	if (!nexell_read_u32(np, "nexell,pin-pull", &value)) {
		unsigned long *tmp;

		tmp = krealloc(configs, (num_configs + 1) * sizeof(*configs),
			       GFP_KERNEL);
		if (!tmp) {
			ret = -ENOMEM;
			goto out;
		}
		configs = tmp;
		configs[num_configs++] = pinconf_to_config_packed(
				(enum pin_config_param)NEXELL_PINCONF_PULL, value);
	}
	if (!nexell_read_u32(np, "nexell,pin-strength", &value)) {
		unsigned long *tmp;

		tmp = krealloc(configs, (num_configs + 1) * sizeof(*configs),
			       GFP_KERNEL);
		if (!tmp) {
			ret = -ENOMEM;
			goto out;
		}
		configs = tmp;
		configs[num_configs++] = pinconf_to_config_packed(
				(enum pin_config_param)NEXELL_PINCONF_DRIVE, value);
	}
	if (!nexell_read_u32(np, "nexell,pin-dir", &value)) {
		unsigned long *tmp;

		tmp = krealloc(configs, (num_configs + 1) * sizeof(*configs),
			       GFP_KERNEL);
		if (!tmp) {
			ret = -ENOMEM;
			goto out;
		}
		configs = tmp;
		configs[num_configs++] = pinconf_to_config_packed(
				(enum pin_config_param)NEXELL_PINCONF_DIRECTION, value);
	}
	if (!nexell_read_u32(np, "nexell,pin-val", &value)) {
		unsigned long *tmp;

		tmp = krealloc(configs, (num_configs + 1) * sizeof(*configs),
			       GFP_KERNEL);
		if (!tmp) {
			ret = -ENOMEM;
			goto out;
		}
		configs = tmp;
		configs[num_configs++] = pinconf_to_config_packed(
				(enum pin_config_param)NEXELL_PINCONF_VALUE, value);
	}

	count = of_property_count_strings(np, pins_property);
	if (count < 1) {
		kfree(configs);
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		unsigned int pin;

		ret = of_property_read_string_index(np, pins_property, i,
						    &pin_name);
		if (ret)
			goto out;
		ret = nexell_parse_pin(pin_name, &pin);
		if (ret)
			goto out;
		ret = nexell_map_one(map, reserved, num_maps, pin_name,
				     has_function ? np->full_name : NULL,
				     configs, num_configs);
		if (ret)
			goto out;
	}

	ret = 0;
out:
	kfree(configs);
	return ret;
}

static int nexell_dt_node_to_map(struct pinctrl_dev *pctldev,
					 struct device_node *np_config,
					 struct pinctrl_map **map,
					 unsigned int *num_maps)
{
		struct device_node *child;
		unsigned int reserved = 0;
		int ret;

	*map = NULL;
	*num_maps = 0;

	if (!of_get_child_count(np_config))
		return nexell_subnode_to_map(pctldev->dev, np_config, map,
					     &reserved, num_maps);

	for_each_child_of_node(np_config, child) {
		ret = nexell_subnode_to_map(pctldev->dev, child, map, &reserved,
					     num_maps);
		if (ret) {
			unsigned int i;

			for (i = 0; i < *num_maps; i++)
				if ((*map)[i].type == PIN_MAP_TYPE_CONFIGS_GROUP)
					kfree((*map)[i].data.configs.configs);
			kfree(*map);
			*map = NULL;
			*num_maps = 0;
			return ret;
		}
	}

	return 0;
}

static void nexell_dt_free_map(struct pinctrl_dev *pctldev,
				       struct pinctrl_map *map,
				       unsigned int num_maps)
{
	unsigned int i;

	for (i = 0; i < num_maps; i++)
		if (map[i].type == PIN_MAP_TYPE_CONFIGS_GROUP)
			kfree(map[i].data.configs.configs);
	kfree(map);
}

static const struct pinctrl_ops nexell_pinctrl_ops = {
	.get_groups_count = nexell_get_group_count,
	.get_group_name = nexell_get_group_name,
	.get_group_pins = nexell_get_group_pins,
	.dt_node_to_map = nexell_dt_node_to_map,
	.dt_free_map = nexell_dt_free_map,
};

static int nexell_set_mux(struct pinctrl_dev *pctldev,
				  unsigned int function, unsigned int group)
{
	struct nexell_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	struct function_desc *func;
	struct group_desc *grp;
	const unsigned int *pins;
	unsigned int num_pins, i, value;

	func = pinmux_generic_get_function(pctldev, function);
	grp = pinctrl_generic_get_group(pctldev, group);
	if (!func || !grp || !func->data)
		return -EINVAL;

	value = *(unsigned int *)func->data;
	pins = grp->grp.pins;
	num_pins = grp->grp.npins;
	for (i = 0; i < num_pins; i++)
		nexell_set_function(pc, pins[i], value);

	dev_dbg(pc->dev, "pinctrl: %s function=%u group=%s\n",
		func->func.name, value, grp->grp.name);
	return 0;
}

static int nexell_gpio_request_enable(struct pinctrl_dev *pctldev,
					      struct pinctrl_gpio_range *range,
					      unsigned int offset)
{
	struct nexell_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	struct nexell_gpio_bank *bank;

	bank = gpiochip_get_data(range->gc);
	nexell_set_function(pc, range->pin_base + offset,
			    nexell_gpio_altfn[bank->index][offset]);
	return 0;
}

static int nexell_gpio_set_direction(struct pinctrl_dev *pctldev,
					     struct pinctrl_gpio_range *range,
					     unsigned int offset, bool input)
{
	struct nexell_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	nexell_set_direction(pc, range->pin_base + offset, input);
	return 0;
}

static const struct pinmux_ops nexell_pinmux_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = nexell_set_mux,
	.gpio_request_enable = nexell_gpio_request_enable,
	.gpio_set_direction = nexell_gpio_set_direction,
};

static int nexell_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
				      unsigned long *configs,
				      unsigned int num_configs)
{
	struct nexell_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	unsigned int i, value, param;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		value = pinconf_to_config_argument(configs[i]);
		switch (param) {
		case NEXELL_PINCONF_PULL:
			nexell_set_pull(pc, pin, value);
			break;
		case NEXELL_PINCONF_DRIVE:
			nexell_set_drive(pc, pin, value);
			break;
		case NEXELL_PINCONF_DIRECTION:
			nexell_set_direction(pc, pin, !value);
			break;
		case NEXELL_PINCONF_VALUE:
			nexell_set_value(pc, pin, value);
			break;
		default:
			return -EOPNOTSUPP;
		}
	}

	return 0;
}

static int nexell_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
				      unsigned long *config)
{
	struct nexell_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	unsigned int value;
	int ret;

	ret = nexell_get_config(pc, pin, pinconf_to_config_param(*config),
				&value);
	if (ret)
		return ret;
	*config = pinconf_to_config_packed(pinconf_to_config_param(*config),
					   value);
	return 0;
}

static int nexell_pinconf_group_set(struct pinctrl_dev *pctldev,
					    unsigned int selector,
					    unsigned long *configs,
					    unsigned int num_configs)
{
	const unsigned int *pins;
	unsigned int num_pins, i;
	int ret;

	ret = pinctrl_generic_get_group_pins(pctldev, selector, &pins,
						     &num_pins);
	if (ret)
		return ret;
	for (i = 0; i < num_pins; i++) {
		ret = nexell_pinconf_set(pctldev, pins[i], configs, num_configs);
		if (ret)
			return ret;
	}
	return 0;
}

static int nexell_pinconf_group_get(struct pinctrl_dev *pctldev,
					    unsigned int selector,
					    unsigned long *config)
{
	const unsigned int *pins;
	unsigned int num_pins;
	int ret;

	ret = pinctrl_generic_get_group_pins(pctldev, selector, &pins,
						     &num_pins);
	if (ret || !num_pins)
		return ret ?: -EINVAL;
	return nexell_pinconf_get(pctldev, pins[0], config);
}

static const struct pinconf_ops nexell_pinconf_ops = {
	.pin_config_get = nexell_pinconf_get,
	.pin_config_set = nexell_pinconf_set,
	.pin_config_group_get = nexell_pinconf_group_get,
	.pin_config_group_set = nexell_pinconf_group_set,
};

static struct nexell_gpio_bank *nexell_gc_to_bank(struct gpio_chip *gc)
{
	return gpiochip_get_data(gc);
}

static int nexell_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct nexell_gpio_bank *bank = nexell_gc_to_bank(gc);
	u32 value;

	if (bank->alive)
		value = readl(bank->base + ALIVE_INPUT);
	else
		value = readl(bank->base + GPIO_PAD);
	return !!(value & BIT(offset));
}

static void nexell_gpio_set(struct gpio_chip *gc, unsigned int offset,
				    int value)
{
	struct nexell_gpio_bank *bank = nexell_gc_to_bank(gc);
	unsigned int pin = bank->pin_base + offset;
	struct nexell_pinctrl *pc = dev_get_drvdata(bank->gc.parent);

	nexell_set_value(pc, pin, value);
}

static int nexell_gpio_get_direction(struct gpio_chip *gc,
					     unsigned int offset)
{
	struct nexell_gpio_bank *bank = nexell_gc_to_bank(gc);
	u32 value;

	if (bank->alive)
		value = readl(bank->base + ALIVE_OUTENB_READ);
	else
		value = readl(bank->base + GPIO_OUTENB);
	return value & BIT(offset) ? GPIO_LINE_DIRECTION_OUT :
		GPIO_LINE_DIRECTION_IN;
}

static int nexell_gpio_direction_input(struct gpio_chip *gc,
					       unsigned int offset)
{
	struct nexell_gpio_bank *bank = nexell_gc_to_bank(gc);
	struct nexell_pinctrl *pc = dev_get_drvdata(bank->gc.parent);

	nexell_set_function(pc, bank->pin_base + offset,
			    nexell_gpio_altfn[bank->index][offset]);
	nexell_set_direction(pc, bank->pin_base + offset, true);
	return 0;
}

static int nexell_gpio_direction_output(struct gpio_chip *gc,
						unsigned int offset, int value)
{
	struct nexell_gpio_bank *bank = nexell_gc_to_bank(gc);
	struct nexell_pinctrl *pc = dev_get_drvdata(bank->gc.parent);

	nexell_set_function(pc, bank->pin_base + offset,
			    nexell_gpio_altfn[bank->index][offset]);
	nexell_set_value(pc, bank->pin_base + offset, value);
	nexell_set_direction(pc, bank->pin_base + offset, false);
	return 0;
}

/*
 * GPIO interrupt support.  Each GPIOA..GPIOE bank has one combined
 * interrupt line to the GIC with per-pin pending in GPIOxDET, a
 * per-pin enable pair in GPIOxINTENB/GPIOxDETENB and a 3-bit detect
 * mode in GPIOxDETMODE[x]/GPIOxDETMODEEX matching the vendor
 * NX_GPIO_INTMODE encoding (0=low, 1=high, 2=falling, 3=rising,
 * 4=both edges).
 */
static void nexell_gpio_irq_ack(struct irq_data *d)
{
	struct nexell_gpio_bank *bank = gpiochip_get_data(
		irq_data_get_irq_chip_data(d));

	writel(BIT(d->hwirq), bank->base + GPIO_DET);
}

static void nexell_gpio_irq_set_enable(struct irq_data *d, bool enable)
{
	struct nexell_gpio_bank *bank = gpiochip_get_data(
		irq_data_get_irq_chip_data(d));
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&bank->lock, flags);
	value = readl(bank->base + GPIO_INTENB);
	value &= ~BIT(d->hwirq);
	if (enable)
		value |= BIT(d->hwirq);
	writel(value, bank->base + GPIO_INTENB);

	value = readl(bank->base + GPIO_DETENB);
	value &= ~BIT(d->hwirq);
	if (enable)
		value |= BIT(d->hwirq);
	writel(value, bank->base + GPIO_DETENB);
	spin_unlock_irqrestore(&bank->lock, flags);
}

static void nexell_gpio_irq_mask(struct irq_data *d)
{
	nexell_gpio_irq_set_enable(d, false);
}

static void nexell_gpio_irq_unmask(struct irq_data *d)
{
	nexell_gpio_irq_set_enable(d, true);
}

static int nexell_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct nexell_gpio_bank *bank = gpiochip_get_data(
		irq_data_get_irq_chip_data(d));
	unsigned int offset = d->hwirq;
	unsigned long flags;
	u32 mode, value;

	switch (type) {
	case IRQ_TYPE_LEVEL_LOW:
		mode = 0;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		mode = 1;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		mode = 2;
		break;
	case IRQ_TYPE_EDGE_RISING:
		mode = 3;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		mode = 4;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&bank->lock, flags);
	value = readl(offset < 16 ? bank->base + GPIO_DETMODE0 :
				    bank->base + GPIO_DETMODE1);
	value &= ~(0x3 << ((offset % 16) * 2));
	value |= (mode & 0x3) << ((offset % 16) * 2);
	writel(value, offset < 16 ? bank->base + GPIO_DETMODE0 :
				    bank->base + GPIO_DETMODE1);

	value = readl(bank->base + GPIO_DETMODEEX);
	value &= ~BIT(offset);
	if (mode & 0x4)
		value |= BIT(offset);
	writel(value, bank->base + GPIO_DETMODEEX);

	writel(BIT(offset), bank->base + GPIO_DET);
	spin_unlock_irqrestore(&bank->lock, flags);

	irq_set_handler_locked(d, (type & IRQ_TYPE_EDGE_BOTH) ?
			       handle_edge_irq : handle_level_irq);

	return 0;
}

static void nexell_gpio_irq_handler(struct irq_desc *desc)
{
	struct nexell_gpio_bank *bank = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long pending;
	unsigned int offset;

	chained_irq_enter(chip, desc);

	pending = readl(bank->base + GPIO_DET);
	writel(pending, bank->base + GPIO_DET);
	for_each_set_bit(offset, &pending, bank->npins)
		generic_handle_domain_irq(bank->gc.irq.domain, offset);

	chained_irq_exit(chip, desc);
}

static int nexell_register_gpio_irq(struct nexell_pinctrl *pc,
				    struct nexell_gpio_bank *bank,
				    struct device_node *np,
				    unsigned int index)
{
	struct gpio_irq_chip *girq;
	struct device *dev = pc->dev;
	struct irq_chip *chip = &bank->irq_chip;
	int irq;

	irq = platform_get_irq_optional(to_platform_device(dev), index);
	if (irq < 0)
		return irq == -ENXIO ? 0 : irq;

	chip->name = devm_kasprintf(dev, GFP_KERNEL, "%s-irq", bank->name);
	if (!chip->name)
		return -ENOMEM;
	chip->irq_ack = nexell_gpio_irq_ack;
	chip->irq_mask = nexell_gpio_irq_mask;
	chip->irq_unmask = nexell_gpio_irq_unmask;
	chip->irq_set_type = nexell_gpio_irq_set_type;

	girq = &bank->gc.irq;
	girq->chip = chip;
	girq->handler = handle_bad_irq;
	girq->default_type = IRQ_TYPE_NONE;
	girq->num_parents = 1;
	girq->parents = devm_kcalloc(dev, 1, sizeof(*girq->parents),
				     GFP_KERNEL);
	if (!girq->parents)
		return -ENOMEM;
	girq->parents[0] = irq;
	girq->parent_handler = nexell_gpio_irq_handler;
	girq->parent_handler_data = bank;

	bank->has_irq = true;
	return 0;
}

static int nexell_add_functions(struct nexell_pinctrl *pc,
				struct device_node *parent)
{
	struct device_node *node;
	const char *pins_property, *pin_name;
	const char **groups;
	unsigned int count, i;
	u32 function;
	int ret;

	for_each_child_of_node(parent, node) {
		if (of_get_child_count(node)) {
			ret = nexell_add_functions(pc, node);
			if (ret)
				return ret;
			continue;
		}

		pins_property = nexell_pins_property(node);
		if (!pins_property || nexell_read_u32(node,
							     "nexell,pin-function",
							     &function))
			continue;

		count = of_property_count_strings(node, pins_property);
		if (!count)
			return -EINVAL;
		groups = devm_kcalloc(pc->dev, count, sizeof(*groups), GFP_KERNEL);
		if (!groups)
			return -ENOMEM;
		for (i = 0; i < count; i++) {
			ret = of_property_read_string_index(node, pins_property, i,
							    &pin_name);
			if (ret)
				return ret;
			groups[i] = pin_name;
		}

		ret = pinmux_generic_add_function(pc->pctldev, node->full_name,
						  groups, count,
						  devm_kmemdup(pc->dev, &function,
							       sizeof(function), GFP_KERNEL));
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int nexell_register_pin_groups(struct nexell_pinctrl *pc)
{
	unsigned int i;
	int ret;

	for (i = 0; i < NEXELL_TOTAL_PINS; i++) {
		ret = pinctrl_generic_add_group(pc->pctldev,
						pc->pins[i].name,
						&pc->pins[i].number, 1, NULL);
		if (ret < 0)
			return ret;
	}

	return nexell_add_functions(pc, pc->dev->of_node);
}

static int nexell_register_gpio(struct nexell_pinctrl *pc,
				struct device_node *np, unsigned int index)
{
	struct nexell_gpio_bank *bank = &pc->banks[index];
	int ret;

	bank->gc.label = bank->name;
	bank->gc.parent = pc->dev;
	bank->gc.fwnode = of_fwnode_handle(np);
	bank->gc.owner = THIS_MODULE;
	bank->gc.base = -1;
	bank->gc.ngpio = bank->npins;
	bank->gc.get = nexell_gpio_get;
	bank->gc.set = nexell_gpio_set;
	bank->gc.get_direction = nexell_gpio_get_direction;
	bank->gc.direction_input = nexell_gpio_direction_input;
	bank->gc.direction_output = nexell_gpio_direction_output;
#if IS_ENABLED(CONFIG_OF_GPIO)
	bank->gc.of_gpio_n_cells = 2;
	bank->gc.of_xlate = nexell_gpio_of_xlate;
#endif

	if (!bank->alive) {
		ret = nexell_register_gpio_irq(pc, bank, np, index);
		if (ret)
			return ret;
	}

	ret = devm_gpiochip_add_data(pc->dev, &bank->gc, bank);
	if (ret)
		return ret;

	bank->range.name = bank->name;
	bank->range.id = index;
	bank->range.base = bank->gc.base;
	bank->range.pin_base = bank->pin_base;
	bank->range.npins = bank->npins;
	bank->range.gc = &bank->gc;
	pinctrl_add_gpio_range(pc->pctldev, &bank->range);
	return 0;
}

static int nexell_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nexell_pinctrl *pc;
	struct device_node *child;
	unsigned int i, pin;
	int ret;

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;
	pc->dev = dev;
	platform_set_drvdata(pdev, pc);

	pc->pins = devm_kcalloc(dev, NEXELL_TOTAL_PINS, sizeof(*pc->pins),
				GFP_KERNEL);
	if (!pc->pins)
		return -ENOMEM;

	for (i = 0, pin = 0; i < NEXELL_GPIO_BANKS; i++) {
		struct resource *res;

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			return dev_err_probe(dev, -EINVAL,
					    "missing GPIO bank resource %u\n", i);
		pc->banks[i].base = devm_ioremap_resource(dev, res);
		if (IS_ERR(pc->banks[i].base))
			return PTR_ERR(pc->banks[i].base);
		pc->banks[i].name = nexell_bank_names[i];
		pc->banks[i].index = i;
		pc->banks[i].pin_base = pin;
		pc->banks[i].npins = i == 5 ? NEXELL_ALIVE_PINS :
			NEXELL_GPIO_PINS;
		pc->banks[i].alive = i == 5;
		spin_lock_init(&pc->banks[i].lock);
		if (pc->banks[i].alive)
			writel(1, pc->banks[i].base + ALIVE_PWRGATE);
		else {
			writel(0xffffffff, pc->banks[i].base + GPIO_SLEW_DISABLE);
			writel(0xffffffff, pc->banks[i].base + GPIO_DRV1_DISABLE);
			writel(0xffffffff, pc->banks[i].base + GPIO_DRV0_DISABLE);
			writel(0xffffffff, pc->banks[i].base + GPIO_PULLSEL_DISABLE);
			writel(0xffffffff, pc->banks[i].base + GPIO_PULLENB_DISABLE);
		}
		for (unsigned int offset = 0; offset < pc->banks[i].npins;
		     offset++, pin++) {
			pc->pins[pin].number = pin;
			pc->pins[pin].name = devm_kasprintf(dev, GFP_KERNEL, "%s-%u",
							    pc->banks[i].name, offset);
			if (!pc->pins[pin].name)
				return -ENOMEM;
		}
	}

	pc->desc.name = "nexell-s5p6818-pinctrl";
	pc->desc.pins = pc->pins;
	pc->desc.npins = NEXELL_TOTAL_PINS;
	pc->desc.pctlops = &nexell_pinctrl_ops;
	pc->desc.pmxops = &nexell_pinmux_ops;
	pc->desc.confops = &nexell_pinconf_ops;
	pc->desc.owner = THIS_MODULE;

	ret = devm_pinctrl_register_and_init(dev, &pc->desc, pc, &pc->pctldev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register pinctrl\n");

	ret = nexell_register_pin_groups(pc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register pin groups\n");

	for (i = 0; i < NEXELL_GPIO_BANKS; i++) {
		child = of_get_child_by_name(dev->of_node,
					    nexell_bank_names[i]);
		if (!child)
			return dev_err_probe(dev, -EINVAL,
					    "missing GPIO bank node %s\n",
					    nexell_bank_names[i]);
		ret = nexell_register_gpio(pc, child, i);
		of_node_put(child);
		if (ret)
			return dev_err_probe(dev, ret,
					    "failed to register GPIO bank %s\n",
					    nexell_bank_names[i]);
	}

	ret = pinctrl_enable(pc->pctldev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable pinctrl\n");

	dev_info(dev, "S5P6818 pinctrl registered: %u pins, %u GPIO banks\n",
		 NEXELL_TOTAL_PINS, NEXELL_GPIO_BANKS);
	return 0;
}

static const struct of_device_id nexell_pinctrl_of_match[] = {
	{ .compatible = "nexell,s5p6818-pinctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, nexell_pinctrl_of_match);

static struct platform_driver nexell_pinctrl_driver = {
	.probe = nexell_pinctrl_probe,
	.driver = {
		.name = "nexell-pinctrl",
		.of_match_table = nexell_pinctrl_of_match,
	},
};
module_platform_driver(nexell_pinctrl_driver);

MODULE_DESCRIPTION("Nexell S5P6818 pinctrl and GPIO driver");
MODULE_LICENSE("GPL");
