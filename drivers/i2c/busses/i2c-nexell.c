// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2016  Nexell Co., Ltd.
 * Author: Hyunseok, Jung <hsjung@nexell.co.kr>
 * Ported to Linux 6.12 LTS for S5P6818 / S5P4418
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/reset.h>

#define I2C_CLOCK_RATE		(100000)	/* 100kHz */
#define WAIT_ACK_TIME		(500)		/* wait 50 msec */

struct nx_i2c_register {
	unsigned int ICCR;	/* 0x00 : I2C Control Register */
	unsigned int ICSR;      /* 0x04 : I2C Status Register */
	unsigned int IAR;       /* 0x08 : I2C Address Register */
	unsigned int IDSR;      /* 0x0C : I2C Data Register */
	unsigned int STOPCON;   /* 0x10 : I2C Stop Control Register */
};

#define CLKSRC_CNT	2
#define CLKSRC_DIV16	16
#define CLKSRC_DIV256	256
#define CLKSCALE_MIN	2
#define CLKSCALE_MAX	16

struct nx_i2c_hw {
	int port;
	int irqno;
	int scl_io;
	int sda_io;
	int clksrc;
	int clkscale;
	void __iomem *base_addr;
};

#define I2C_TRANS_RUN		(1 << 0)
#define I2C_TRANS_DONE		(1 << 1)
#define I2C_TRANS_ERR		(1 << 2)

struct nx_i2c_param {
	struct nx_i2c_hw hw;
	spinlock_t lock;
	wait_queue_head_t wait_q;
	unsigned int condition;
	unsigned int rate;
	int no_stop;
	u8 pre_data;
	int request_ack;
	int timeout;

	struct i2c_adapter adapter;
	struct i2c_msg *msg;
	struct clk *clk;
	struct reset_control *rst;
	int trans_count;
	int trans_mode;
	int running;
	unsigned int trans_status;
	bool polling;
	struct pinctrl *pctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sda_dft;
	struct pinctrl_state *pins_scl_dft;
	struct pinctrl_state *pins_sda_gpio;
	struct pinctrl_state *pins_scl_gpio;

	int irq_count;
	int thd_count;
	int sda_delay;
	int retry_delay;
	struct device *dev;
	unsigned long clk_in;
};

#define I2C_TXRXMODE_SLAVE_RX	0
#define I2C_TXRXMODE_SLAVE_TX	1
#define I2C_TXRXMODE_MASTER_RX	2
#define I2C_TXRXMODE_MASTER_TX	3

#define I2C_ICCR_OFFS		0x00
#define I2C_ICSR_OFFS		0x04
#define I2C_IDSR_OFFS		0x0C
#define I2C_STOP_OFFS		0x10

#define ICCR_IRQ_CLR_POS	8
#define ICCR_ACK_ENB_POS	7
#define ICCR_IRQ_ENB_POS	5
#define ICCR_IRQ_PND_POS	4
#define ICCR_CLK_SRC_POS	6
#define ICCR_CLK_VAL_POS	0

#define ICSR_MOD_SEL_POS	6
#define ICSR_SIG_GEN_POS	5
#define ICSR_BUS_BUSY_POS	5
#define ICSR_OUT_ENB_POS	4
#define ICSR_ARI_STA_POS	3 /* Arbitration */
#define ICSR_ACK_REV_POS	0 /* ACK */

#define STOP_ACK_GEM_POS	2
#define STOP_DAT_REL_POS	1
#define STOP_CLK_REL_POS	0

#define _SETDATA(p, d)		writel((d), (p) + I2C_IDSR_OFFS)
#define _GETDATA(p)		readl((p) + I2C_IDSR_OFFS)
#define _BUSOFF(p)		(readl((p) + I2C_ICSR_OFFS) & ~(1 << ICSR_OUT_ENB_POS))
#define _ACKSTAT(p)		(readl((p) + I2C_ICSR_OFFS) & (1 << ICSR_ACK_REV_POS))
#define _ARBITSTAT(p)		(readl((p) + I2C_ICSR_OFFS) & (1 << ICSR_ARI_STA_POS))
#define _INTSTAT(p)		(readl((p) + I2C_ICCR_OFFS) & (1 << ICCR_IRQ_PND_POS))

static inline void nx_i2c_start_dev(struct nx_i2c_param *par)
{
	void __iomem *base = par->hw.base_addr;
	unsigned int ICSR = 0, ICCR = 0;

	ICSR = (1 << ICSR_OUT_ENB_POS);
	writel(ICSR, base + I2C_ICSR_OFFS);

	writel(par->pre_data, base + I2C_IDSR_OFFS);

	ICCR = readl(base + I2C_ICCR_OFFS);
	ICCR &= ~(1 << ICCR_ACK_ENB_POS);
	ICCR |= (1 << ICCR_IRQ_ENB_POS);
	writel(ICCR, base + I2C_ICCR_OFFS);

	ICSR = (par->trans_mode << ICSR_MOD_SEL_POS) |
	       (1 << ICSR_SIG_GEN_POS) | (1 << ICSR_OUT_ENB_POS);
	writel(ICSR, base + I2C_ICSR_OFFS);
}

static inline void nx_i2c_trans_dev(void __iomem *base, unsigned int ack,
				    int stop)
{
	unsigned int ICCR = 0, STOP = 0;

	ICCR = readl(base + I2C_ICCR_OFFS);
	ICCR &= ~(1 << ICCR_ACK_ENB_POS);
	ICCR |= (ack << ICCR_ACK_ENB_POS);
	writel(ICCR, base + I2C_ICCR_OFFS);

	if (stop) {
		STOP = readl(base + I2C_STOP_OFFS);
		STOP |= (1 << STOP_ACK_GEM_POS);
		writel(STOP, base + I2C_STOP_OFFS);
	}

	ICCR = readl(base + I2C_ICCR_OFFS);
	ICCR &= ~(1 << ICCR_IRQ_PND_POS);
	ICCR |= (1 << ICCR_IRQ_CLR_POS);
	ICCR |= (1 << ICCR_IRQ_ENB_POS);
	writel(ICCR, base + I2C_ICCR_OFFS);
}

static int nx_i2c_stop_scl(struct nx_i2c_param *par)
{
	void __iomem *base = par->hw.base_addr;
	unsigned int ICSR = 0, ICCR = 0, STOP = 0;
	int gpio = par->hw.scl_io;
	unsigned long start;
	int timeout = 5, ret = 0;

	STOP = (1 << STOP_CLK_REL_POS);
	writel(STOP, base + I2C_STOP_OFFS);

	ICSR = readl(base + I2C_ICSR_OFFS);
	ICSR &= ~(1 << ICSR_OUT_ENB_POS);
	ICSR = par->trans_mode << ICSR_MOD_SEL_POS;
	writel(ICSR, base + I2C_ICSR_OFFS);

	ICCR = (1 << ICCR_IRQ_CLR_POS);
	writel(ICCR, base + I2C_ICCR_OFFS);

	if (gpio_is_valid(gpio)) {
		gpio_direction_input(gpio);
		if (!gpio_get_value(gpio)) {
			if (par->pins_scl_gpio)
				pinctrl_select_state(par->pctrl, par->pins_scl_gpio);
			gpio_direction_output(gpio, 1);
			start = jiffies;
			while (!gpio_get_value(gpio)) {
				if (time_after(jiffies, start + timeout)) {
					if (gpio_get_value(gpio))
						break;
					ret = -ETIMEDOUT;
					goto _stop_timeout;
				}
				cpu_relax();
			}
		}
	}

_stop_timeout:
	if (!IS_ERR_OR_NULL(par->pins_default))
		pinctrl_select_state(par->pctrl, par->pins_default);
	else if (par->pins_scl_dft)
		pinctrl_select_state(par->pctrl, par->pins_scl_dft);
	return ret;
}

static inline void nx_i2c_stop_dev(struct nx_i2c_param *par, int nostop,
				   int read)
{
	void __iomem *base = par->hw.base_addr;
	unsigned int ICSR = 0, ICCR = 0;
	int delay = par->sda_delay;
	unsigned long flags;

	spin_lock_irqsave(&par->lock, flags);
	if (!nostop) {
		if (par->pins_sda_gpio)
			pinctrl_select_state(par->pctrl, par->pins_sda_gpio);
		if (gpio_is_valid(par->hw.sda_io))
			gpio_direction_output(par->hw.sda_io, 0);
		udelay(delay);
		nx_i2c_stop_scl(par);
		udelay(delay);
		if (gpio_is_valid(par->hw.sda_io))
			gpio_set_value(par->hw.sda_io, 1);
		if (!IS_ERR_OR_NULL(par->pins_default))
			pinctrl_select_state(par->pctrl, par->pins_default);
		else if (par->pins_sda_dft)
			pinctrl_select_state(par->pctrl, par->pins_sda_dft);
	} else {
		if (par->pins_sda_gpio)
			pinctrl_select_state(par->pctrl, par->pins_sda_gpio);
		if (gpio_is_valid(par->hw.sda_io))
			gpio_direction_output(par->hw.sda_io, 1);
		udelay(delay);
		if (par->pins_scl_gpio)
			pinctrl_select_state(par->pctrl, par->pins_scl_gpio);
		if (gpio_is_valid(par->hw.scl_io))
			gpio_direction_output(par->hw.scl_io, 1);
		ICSR = par->trans_mode << ICSR_MOD_SEL_POS;
		writel(ICSR, base + I2C_ICSR_OFFS);

		ICCR = (1 << ICCR_IRQ_CLR_POS);
		writel(ICCR, base + I2C_ICCR_OFFS);
		if (!IS_ERR_OR_NULL(par->pins_default)) {
			pinctrl_select_state(par->pctrl, par->pins_default);
		} else {
			if (par->pins_sda_dft)
				pinctrl_select_state(par->pctrl, par->pins_sda_dft);
			if (par->pins_scl_dft)
				pinctrl_select_state(par->pctrl, par->pins_scl_dft);
		}
	}
	spin_unlock_irqrestore(&par->lock, flags);
}

static inline void nx_i2c_wait_dev(struct nx_i2c_param *par, int *wait)
{
	void __iomem *base = par->hw.base_addr;
	unsigned int ICSR = 0;

	do {
		ICSR = readl(base + I2C_ICSR_OFFS);
		if (!(ICSR & (1 << ICSR_BUS_BUSY_POS)) &&
		    !(ICSR & (1 << ICSR_ARI_STA_POS)))
			break;
		mdelay(1);
	} while ((*wait)-- > 0);
}

static inline void nx_i2c_bus_off(struct nx_i2c_param *par)
{
	void __iomem *base = par->hw.base_addr;
	unsigned int ICSR = 0;

	ICSR &= ~(1 << ICSR_OUT_ENB_POS);
	writel(ICSR, base + I2C_ICSR_OFFS);
}

static inline void nx_i2c_set_clock(struct nx_i2c_param *par, int enable)
{
	int cksrc = (par->hw.clksrc == CLKSRC_DIV16) ? 0 : 1;
	int ckscl = par->hw.clkscale;
	void __iomem *base = par->hw.base_addr;
	unsigned int ICCR = 0, ICSR = 0;

	if (enable) {
		ICCR = readl(base + I2C_ICCR_OFFS);
		ICCR &= ~(0x0f | (1 << ICCR_CLK_SRC_POS));
		ICCR |= ((cksrc << ICCR_CLK_SRC_POS) | (ckscl - 1));
		writel(ICCR, base + I2C_ICCR_OFFS);
	} else {
		ICSR = readl(base + I2C_ICSR_OFFS);
		ICSR &= ~(1 << ICSR_OUT_ENB_POS);
		writel(ICSR, base + I2C_ICSR_OFFS);
	}
}

static inline int nx_i2c_wait_busy(struct nx_i2c_param *par)
{
	void __iomem *base = par->hw.base_addr;
	int wait = 500;
	int ret = 0;

	nx_i2c_wait_dev(par, &wait);
	if (0 == wait) {
		dev_err(par->dev, "Fail, i2c.%d is busy, arbitration %s ...\n",
			par->hw.port, _ARBITSTAT(base) ? "busy" : "free");
		ret = -1;
	}
	return ret;
}

static irqreturn_t nx_i2c_irq_thread(int irqno, void *dev_id)
{
	struct nx_i2c_param *par = dev_id;
	struct i2c_msg *msg = par->msg;
	void __iomem *base = par->hw.base_addr;
	u16 flags = par->msg->flags;
	int len = msg->len;
	int cnt = par->trans_count;

	par->thd_count++;

	if (_ARBITSTAT(base) != 0) {
		dev_err(par->dev,
			"Fail, arbit i2c.%d addr [0x%02x] data[0x%02x], trans[%2d:%2d]\n",
			par->hw.port, (msg->addr << 1), par->pre_data, cnt, len);
		par->trans_status = I2C_TRANS_ERR;
		goto __irq_end;
	}

	if (par->request_ack && _ACKSTAT(base)) {
		dev_dbg(par->dev,
			"Fail, noack i2c.%d addr [0x%02x] data[0x%02x], trans[%2d:%2d]\n",
			par->hw.port, (msg->addr << 1), par->pre_data, cnt, len);
		par->trans_status = I2C_TRANS_ERR;
		goto __irq_end;
	}

	if (I2C_TRANS_RUN == par->trans_status) {
		if (flags & I2C_M_RD) {
			int ack = (len == cnt + 1) ? 0 : 1;
			int last = (len == cnt + 1) ? 1 : 0;

			par->request_ack = 0;
			if (0 == cnt) {
				nx_i2c_trans_dev(base, ack, 0);
				par->trans_count += 1;
				return IRQ_HANDLED;
			}

			msg->buf[cnt - 1] = _GETDATA(base);

			if (len == par->trans_count) {
				par->trans_status = I2C_TRANS_DONE;
				goto __irq_end;
			} else {
				nx_i2c_trans_dev(base, ack, last);
				par->trans_count += 1;
				return IRQ_HANDLED;
			}
		} else {
			par->pre_data = msg->buf[cnt];
			par->request_ack = (msg->flags & I2C_M_IGNORE_NAK) ? 0 : 1;
			if (len == 0)
				par->trans_status = I2C_TRANS_DONE;
			else if (len == ++par->trans_count)
				par->trans_status = I2C_TRANS_DONE;

			_SETDATA(base, msg->buf[cnt]);
			nx_i2c_trans_dev(base, 0,
					 par->trans_status == I2C_TRANS_DONE ? 1 : 0);
			return IRQ_HANDLED;
		}
	}

__irq_end:
	nx_i2c_stop_dev(par, par->no_stop, (flags & I2C_M_RD ? 0 : 1));
	par->condition = 1;
	wake_up(&par->wait_q);
	return IRQ_HANDLED;
}

static irqreturn_t nx_i2c_irq_handler(int irqno, void *dev_id)
{
	struct nx_i2c_param *par = dev_id;
	unsigned int ICCR = 0;
	void __iomem *base = par->hw.base_addr;
	unsigned long flags;

	spin_lock_irqsave(&par->lock, flags);
	par->irq_count++;

	if (!par->running) {
		spin_unlock_irqrestore(&par->lock, flags);
		return IRQ_HANDLED;
	}

	ICCR = readl(base + I2C_ICCR_OFFS);
	ICCR &= ~((1 << ICCR_IRQ_PND_POS) | (1 << ICCR_IRQ_ENB_POS));
	ICCR |= (1 << ICCR_IRQ_CLR_POS);
	writel(ICCR, base + I2C_ICCR_OFFS);

	spin_unlock_irqrestore(&par->lock, flags);
	return IRQ_WAKE_THREAD;
}

static bool nx_i2c_poll_mode = true;
module_param_named(polling, nx_i2c_poll_mode, bool, 0444);
MODULE_PARM_DESC(polling,
	"Poll the IP pending flag instead of waiting for the completion IRQ (default: true)");

static int nx_i2c_trans_done(struct nx_i2c_param *par)
{
	struct i2c_msg *msg = par->msg;
	void __iomem *base = par->hw.base_addr;
	int wait, timeout, ret = 0;

	par->condition = 0;
	if (msg->len)
		wait = msg->len * msecs_to_jiffies(par->timeout);
	else
		wait = msecs_to_jiffies(par->timeout);

	if (par->polling) {
		/*
		 * On S5P6818 the master-transfer completion IRQ is not
		 * delivered in the current platform state, so wait_event
		 * always runs the full timeout.  Poll the IP pending flag
		 * and run the same handler body the IRQ would have run.
		 */
		int poll_loops = wait;

		while (poll_loops-- > 0) {
			if (par->condition ||
			    par->trans_status == I2C_TRANS_DONE ||
			    par->trans_status == I2C_TRANS_ERR) {
				par->condition = 1;
				break;
			}
			if (_INTSTAT(base)) {
				(void)nx_i2c_irq_thread(0, par);
				continue;
			}
			mdelay(1);
		}
	} else {
		timeout = wait_event_timeout(par->wait_q, par->condition, wait);
	}

	if (par->condition) {
		if (I2C_TRANS_ERR == par->trans_status)
			ret = -1;
	} else {
		dev_err(par->dev,
			"Fail, i2c.%d %s [0x%02x] cond(%d) pend(%s) arbit(%s) mode(%s) tran(%d:%d,%d:%d) wait(%dms)\n",
			par->hw.port, (msg->flags & I2C_M_RD) ? "R" : "W",
			msg->addr << 1, par->condition,
			_INTSTAT(base) ? "yes" : "no",
			_ARBITSTAT(base) ? "busy" : "free",
			par->polling ? "polling" : "irq",
			par->trans_count, msg->len,
			par->irq_count, par->thd_count, par->timeout);
		ret = -1;
	}

	if (ret < 0)
		nx_i2c_stop_dev(par, 0, 0);

	return ret;
}

static inline int nx_i2c_trans_data(struct nx_i2c_param *par,
				    struct i2c_msg *msg)
{
	u32 mode;
	u8 addr;

	if (msg->flags & I2C_M_TEN) {
		dev_err(par->dev, "Fail, i2c.%d does not support ten bit\n", par->hw.port);
		return -EINVAL;
	}
	if (msg->flags & I2C_M_RD) {
		addr = (msg->addr << 1) | 1;
		mode = I2C_TXRXMODE_MASTER_RX;
	} else {
		addr = (msg->addr << 1);
		mode = I2C_TXRXMODE_MASTER_TX;
	}

	par->msg = msg;
	par->condition = 0;
	par->pre_data = addr;
	par->request_ack = (msg->flags & I2C_M_IGNORE_NAK) ? 0 : 1;
	par->trans_count = 0;
	par->trans_mode = mode;
	par->trans_status = I2C_TRANS_RUN;

	nx_i2c_start_dev(par);
	return nx_i2c_trans_done(par);
}

static int nx_i2c_transfer(struct nx_i2c_param *par, struct i2c_msg *msg,
			   int num)
{
	int ret = -EAGAIN;

	nx_i2c_set_clock(par, 1);
	if (nx_i2c_wait_busy(par) < 0)
		goto err_i2c;

	if (nx_i2c_trans_data(par, msg) < 0)
		goto err_i2c;

	ret = msg->len;

err_i2c:
	if (ret != msg->len)
		msg->flags &= ~I2C_M_NOSTART;

	nx_i2c_wait_busy(par);
	nx_i2c_set_clock(par, 0);
	return ret;
}

static int nx_i2c_algo_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs,
			    int num)
{
	struct nx_i2c_param *par = adapter->algo_data;
	struct i2c_msg *tmsg = msgs;
	int i, j = num;
	int ret = -EAGAIN;
	int len = 0;
	int delay = par->retry_delay;

	par->running = 1;
	par->irq_count = 0;
	par->thd_count = 0;

	for (; j > 0; j--, tmsg++) {
		par->no_stop = (1 == j ? 0 : 1);
		len = tmsg->len;

		for (i = adapter->retries; i > 0; i--) {
			ret = nx_i2c_transfer(par, tmsg, num);
			if (ret == len)
				break;
			udelay(delay);
		}

		if (ret != len)
			break;
	}

	par->running = 0;
	if (ret == len)
		return num;

	return ret;
}

static u32 nx_i2c_algo_fn(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm nx_i2c_algo = {
	.master_xfer	= nx_i2c_algo_xfer,
	.functionality	= nx_i2c_algo_fn,
};

static void nx_i2c_set_clk_param(struct nx_i2c_param *par, unsigned long rate)
{
	unsigned int t_src = 0, t_div = 0;
	unsigned long calc_clk, t_clk = 0;
	unsigned long real_clk = 0, get_real_clk = 0, req_rate = 0;
	unsigned int i = 0, src = 0;
	int div = 0;

	req_rate = par->rate;
	t_clk = rate / CLKSRC_DIV16 / CLKSCALE_MIN;

	for (i = 0; i < CLKSRC_CNT; i++) {
		src = (i == 0) ? CLKSRC_DIV16 : CLKSRC_DIV256;
		for (div = CLKSCALE_MIN; div <= CLKSCALE_MAX; div++) {
			get_real_clk = rate / src / div;
			if (get_real_clk > req_rate)
				calc_clk = get_real_clk - req_rate;
			else
				calc_clk = req_rate - get_real_clk;

			if (calc_clk < t_clk) {
				t_clk = calc_clk;
				t_div = div;
				t_src = src;
				real_clk = get_real_clk;
			} else if (calc_clk == 0) {
				t_div = div;
				t_src = src;
				real_clk = get_real_clk;
				break;
			} else {
				break;
			}
		}
		if (calc_clk == 0)
			break;
	}

	par->hw.clksrc = t_src;
	par->hw.clkscale = t_div;
	par->clk_in = rate;

	dev_info(par->dev, "i2c.%d: %8ld Hz [pclk=%ld Hz, clksrc=%3d, clkscale=%2d]\n",
		 par->hw.port, real_clk, rate, par->hw.clksrc, par->hw.clkscale - 1);
}

static int nx_i2c_probe(struct platform_device *pdev)
{
	struct nx_i2c_param *par;
	unsigned long rate;
	int ret;

	par = devm_kzalloc(&pdev->dev, sizeof(*par), GFP_KERNEL);
	if (!par)
		return -ENOMEM;

	par->dev = &pdev->dev;
	par->hw.port = of_alias_get_id(pdev->dev.of_node, "i2c");
	if (par->hw.port < 0)
		par->hw.port = pdev->id >= 0 ? pdev->id : 0;

	par->hw.irqno = platform_get_irq(pdev, 0);
	if (par->hw.irqno < 0)
		return par->hw.irqno;

	par->hw.base_addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(par->hw.base_addr))
		return PTR_ERR(par->hw.base_addr);

	par->hw.sda_io = of_get_named_gpio(pdev->dev.of_node, "gpios", 0);
	par->hw.scl_io = of_get_named_gpio(pdev->dev.of_node, "gpios", 1);
	par->no_stop = 0;
	par->timeout = WAIT_ACK_TIME;
	par->polling = nx_i2c_poll_mode;

	of_property_read_u32(pdev->dev.of_node, "sda-delay", &par->sda_delay);
	if (!par->sda_delay)
		par->sda_delay = 1;

	of_property_read_u32(pdev->dev.of_node, "retry-delay", &par->retry_delay);
	of_property_read_u32(pdev->dev.of_node, "retry-cnt", &par->adapter.retries);
	if (!par->adapter.retries)
		par->adapter.retries = 3;

	of_property_read_u32(pdev->dev.of_node, "clock-frequency", &par->rate);
	if (!par->rate)
		par->rate = I2C_CLOCK_RATE;

	par->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(par->clk))
		return PTR_ERR(par->clk);

	ret = clk_prepare_enable(par->clk);
	if (ret)
		return ret;

	rate = clk_get_rate(par->clk);
	if (!rate || rate > 200000000) {
		dev_warn(par->dev, "invalid clock rate %lu Hz, clamping to 100 MHz\n", rate);
		rate = 100000000;
	}
	nx_i2c_set_clk_param(par, rate);

	par->rst = devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (!IS_ERR_OR_NULL(par->rst))
		reset_control_deassert(par->rst);

	spin_lock_init(&par->lock);
	init_waitqueue_head(&par->wait_q);

	ret = devm_request_threaded_irq(par->dev, par->hw.irqno,
					nx_i2c_irq_handler, nx_i2c_irq_thread,
					IRQF_SHARED, "nexell-i2c", par);
	if (ret) {
		dev_err(par->dev, "Fail, i2c.%d request irq %d\n",
			par->hw.port, par->hw.irqno);
		goto err_clk;
	}

	nx_i2c_bus_off(par);

	strscpy(par->adapter.name, "nexell-i2c", sizeof(par->adapter.name));
	par->adapter.owner = THIS_MODULE;
	par->adapter.nr = par->hw.port;
	par->adapter.algo = &nx_i2c_algo;
	par->adapter.algo_data = par;
	par->adapter.dev.parent = &pdev->dev;
	par->adapter.dev.of_node = pdev->dev.of_node;

	if (gpio_is_valid(par->hw.sda_io))
		devm_gpio_request(&pdev->dev, par->hw.sda_io, "i2c-sda");
	if (gpio_is_valid(par->hw.scl_io))
		devm_gpio_request(&pdev->dev, par->hw.scl_io, "i2c-scl");

	par->pctrl = devm_pinctrl_get(&pdev->dev);
	if (!IS_ERR(par->pctrl)) {
		par->pins_default = pinctrl_lookup_state(par->pctrl, PINCTRL_STATE_DEFAULT);
		if (!IS_ERR(par->pins_default)) {
			pinctrl_select_state(par->pctrl, par->pins_default);
		} else {
			par->pins_sda_dft = pinctrl_lookup_state(par->pctrl, "sda_dft");
			par->pins_scl_dft = pinctrl_lookup_state(par->pctrl, "scl_dft");
			par->pins_sda_gpio = pinctrl_lookup_state(par->pctrl, "sda_gpio");
			par->pins_scl_gpio = pinctrl_lookup_state(par->pctrl, "scl_gpio");

			if (!IS_ERR(par->pins_sda_dft))
				pinctrl_select_state(par->pctrl, par->pins_sda_dft);
			if (!IS_ERR(par->pins_scl_dft))
				pinctrl_select_state(par->pctrl, par->pins_scl_dft);
		}
	}

	ret = i2c_add_numbered_adapter(&par->adapter);
	if (ret) {
		dev_err(par->dev, "Fail, i2c.%d add adapter\n", par->hw.port);
		goto err_clk;
	}

	platform_set_drvdata(pdev, par);
	return 0;

err_clk:
	clk_disable_unprepare(par->clk);
	return ret;
}

static void nx_i2c_remove(struct platform_device *pdev)
{
	struct nx_i2c_param *par = platform_get_drvdata(pdev);

	i2c_del_adapter(&par->adapter);
	clk_disable_unprepare(par->clk);
	if (!IS_ERR_OR_NULL(par->rst))
		reset_control_assert(par->rst);
}

static const struct of_device_id nx_i2c_match[] = {
	{ .compatible = "nexell,s5p6818-i2c" },
	{ .compatible = "nexell,s5p4418-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, nx_i2c_match);

static struct platform_driver nx_i2c_driver = {
	.probe = nx_i2c_probe,
	.remove = nx_i2c_remove,
	.driver = {
		.name = "nexell-i2c",
		.of_match_table = nx_i2c_match,
	},
};

module_platform_driver(nx_i2c_driver);

MODULE_DESCRIPTION("Nexell S5P6818 / S5P4418 I2C bus driver");
MODULE_AUTHOR("Hyunseok Jung <hsjung@nexell.co.kr>");
MODULE_LICENSE("GPL");
