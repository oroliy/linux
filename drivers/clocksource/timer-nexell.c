// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nexell S5P6818 timer.
 *
 * The legacy BSP uses timer channel 0 as a free-running clocksource and
 * channel 1 as the system clockevent.  Both channels share the prescaler.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>
#include <linux/spinlock.h>
#include <linux/reset.h>

#define NEXELL_TIMER_CFG0		0x00
#define NEXELL_TIMER_CFG1		0x04
#define NEXELL_TIMER_TCON		0x08
#define NEXELL_TIMER_CNTB		0x0c
#define NEXELL_TIMER_CMPB		0x10
#define NEXELL_TIMER_CNTO		0x14
#define NEXELL_TIMER_STAT		0x44

#define NEXELL_TIMER_CHANNEL_STRIDE	0x0c

#define NEXELL_TIMER_TCON_AUTO		BIT(3)
#define NEXELL_TIMER_TCON_UP		BIT(1)
#define NEXELL_TIMER_TCON_RUN		BIT(0)

#define NEXELL_TIMER_CFG0_PRESCALE_MASK	GENMASK(7, 0)
#define NEXELL_TIMER_CFG1_MUX_MASK	GENMASK(3, 0)
#define NEXELL_TIMER_TCON_MODE_MASK	GENMASK(3, 1)

#define NEXELL_TIMER_STATUS_IRQ(ch)	BIT(ch)
#define NEXELL_TIMER_STATUS_CLEAR(ch)	BIT((ch) + 5)
#define NEXELL_TIMER_STATUS_CLEAR_MASK	GENMASK(9, 5)

#define NEXELL_TIMER_SOURCE_CHANNEL	0
#define NEXELL_TIMER_EVENT_CHANNEL	1

struct nexell_timer {
	void __iomem *base;
	unsigned long rate;
	struct clocksource clocksource;
	struct clock_event_device clockevent;
	/* Serializes clockevent programming with the timer IRQ handler. */
	spinlock_t lock;
};

static struct nexell_timer nexell_timer_data;
static struct nexell_timer *nexell_timer = &nexell_timer_data;
static struct delay_timer nexell_delay_timer;

static inline void __iomem *nexell_timer_channel(struct nexell_timer *timer,
							unsigned int channel,
							unsigned int offset)
{
	return timer->base + offset +
		NEXELL_TIMER_CHANNEL_STRIDE * channel;
}

static void nexell_timer_configure(struct nexell_timer *timer,
					   unsigned long pclk_rate,
					   unsigned long tick_rate)
{
	u32 value;
	unsigned int prescale;
	unsigned long actual_rate;

	prescale = DIV_ROUND_CLOSEST(pclk_rate, tick_rate);
	if (prescale < 1)
		prescale = 1;
	if (prescale > 256)
		prescale = 256;

	actual_rate = pclk_rate / prescale;
	if (actual_rate != tick_rate)
		pr_warn("nexell-timer: requested %lu Hz, using %lu Hz\n",
			tick_rate, actual_rate);
	timer->rate = actual_rate;

	/* Timer mux 0 selects PCLK on S5P6818. */
	value = readl(timer->base + NEXELL_TIMER_CFG0);
	value &= ~NEXELL_TIMER_CFG0_PRESCALE_MASK;
	value |= prescale - 1;
	writel(value, timer->base + NEXELL_TIMER_CFG0);

	value = readl(timer->base + NEXELL_TIMER_CFG1);
	value &= ~(NEXELL_TIMER_CFG1_MUX_MASK |
			   (NEXELL_TIMER_CFG1_MUX_MASK << 4));
	writel(value, timer->base + NEXELL_TIMER_CFG1);
}

static void nexell_timer_stop(struct nexell_timer *timer,
				      unsigned int channel)
{
	u32 value;
	unsigned int shift = channel ? channel * 4 + 4 : 0;

	value = readl(timer->base + NEXELL_TIMER_STAT);
	value &= ~(NEXELL_TIMER_STATUS_CLEAR_MASK |
			   NEXELL_TIMER_STATUS_IRQ(channel));
	value |= NEXELL_TIMER_STATUS_CLEAR(channel);
	writel(value, timer->base + NEXELL_TIMER_STAT);

	value = readl(timer->base + NEXELL_TIMER_TCON);
	value &= ~(NEXELL_TIMER_TCON_RUN << shift);
	writel(value, timer->base + NEXELL_TIMER_TCON);
}

static void nexell_timer_load_raw(struct nexell_timer *timer,
					  unsigned int channel, u32 value)
{
	writel(value, nexell_timer_channel(timer, channel, NEXELL_TIMER_CNTB));
	writel(value, nexell_timer_channel(timer, channel, NEXELL_TIMER_CMPB));
}

static void nexell_timer_load(struct nexell_timer *timer,
				      unsigned int channel, u32 count)
{
	if (!count)
		count = 1;

	nexell_timer_load_raw(timer, channel, count - 1);
}

static void nexell_timer_start(struct nexell_timer *timer,
				       unsigned int channel, bool irq_enabled,
				       bool auto_reload)
{
	u32 value;
	unsigned int shift = channel ? channel * 4 + 4 : 0;

	value = readl(timer->base + NEXELL_TIMER_STAT);
	value &= ~NEXELL_TIMER_STATUS_CLEAR_MASK;
	value &= ~NEXELL_TIMER_STATUS_IRQ(channel);
	value |= NEXELL_TIMER_STATUS_CLEAR(channel);
	if (irq_enabled)
		value |= NEXELL_TIMER_STATUS_IRQ(channel);
	writel(value, timer->base + NEXELL_TIMER_STAT);

	/* UP latches CNTB/CMPB; RUN starts the down-counter, AUTO reloads it. */
	value = readl(timer->base + NEXELL_TIMER_TCON);
	value &= ~(NEXELL_TIMER_TCON_MODE_MASK << shift);
	value |= NEXELL_TIMER_TCON_UP << shift;
	writel(value, timer->base + NEXELL_TIMER_TCON);

	value &= ~(NEXELL_TIMER_TCON_UP << shift);
	value |= NEXELL_TIMER_TCON_RUN << shift;
	if (auto_reload)
		value |= NEXELL_TIMER_TCON_AUTO << shift;
	writel(value, timer->base + NEXELL_TIMER_TCON);
}

static void nexell_timer_clear_pending(struct nexell_timer *timer,
					       unsigned int channel)
{
	u32 value;

	value = readl(timer->base + NEXELL_TIMER_STAT);
	value &= ~NEXELL_TIMER_STATUS_CLEAR_MASK;
	value |= NEXELL_TIMER_STATUS_CLEAR(channel);
	writel(value, timer->base + NEXELL_TIMER_STAT);
}

static u32 nexell_timer_read_counter(struct nexell_timer *timer)
{
	return ~readl_relaxed(nexell_timer_channel(timer,
							 NEXELL_TIMER_SOURCE_CHANNEL,
							 NEXELL_TIMER_CNTO));
}

static unsigned long nexell_timer_read_current_timer(void)
{
	return nexell_timer_read_counter(nexell_timer);
}

static u64 notrace nexell_timer_read_sched_clock(void)
{
	return nexell_timer_read_counter(nexell_timer);
}

static u64 nexell_timer_read_clocksource(struct clocksource *clocksource)
{
	struct nexell_timer *timer = container_of(clocksource,
						  struct nexell_timer, clocksource);

	return nexell_timer_read_counter(timer);
}

static int nexell_timer_set_state_shutdown(struct clock_event_device *event)
{
	struct nexell_timer *timer = container_of(event, struct nexell_timer,
							 clockevent);
	unsigned long flags;

	spin_lock_irqsave(&timer->lock, flags);
	nexell_timer_stop(timer, NEXELL_TIMER_EVENT_CHANNEL);
	spin_unlock_irqrestore(&timer->lock, flags);

	return 0;
}

static int nexell_timer_set_state_oneshot(struct clock_event_device *event)
{
	/* Entering oneshot mode must not stop the channel; set_next_event starts it. */
	return 0;
}

static int nexell_timer_set_state_periodic(struct clock_event_device *event)
{
	struct nexell_timer *timer = container_of(event, struct nexell_timer,
							 clockevent);
	unsigned long flags;
	unsigned long period = DIV_ROUND_CLOSEST(timer->rate, HZ);

	spin_lock_irqsave(&timer->lock, flags);
	nexell_timer_stop(timer, NEXELL_TIMER_EVENT_CHANNEL);
	nexell_timer_load(timer, NEXELL_TIMER_EVENT_CHANNEL, period);
	nexell_timer_start(timer, NEXELL_TIMER_EVENT_CHANNEL, true, true);
	spin_unlock_irqrestore(&timer->lock, flags);

	return 0;
}

static int nexell_timer_set_next_event(unsigned long delta,
						       struct clock_event_device *event)
{
	struct nexell_timer *timer = container_of(event, struct nexell_timer,
								 clockevent);
	unsigned long flags;

	spin_lock_irqsave(&timer->lock, flags);
	nexell_timer_stop(timer, NEXELL_TIMER_EVENT_CHANNEL);
	nexell_timer_load(timer, NEXELL_TIMER_EVENT_CHANNEL, delta);
	nexell_timer_start(timer, NEXELL_TIMER_EVENT_CHANNEL, true, false);
	spin_unlock_irqrestore(&timer->lock, flags);

	return 0;
}

static irqreturn_t nexell_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *event = dev_id;
	struct nexell_timer *timer = container_of(event, struct nexell_timer,
								 clockevent);

	/* STAT bit 5+ is a write-one-to-clear command, not a read status bit. */
	nexell_timer_clear_pending(timer, NEXELL_TIMER_EVENT_CHANNEL);
	event->event_handler(event);

	return IRQ_HANDLED;
}

static int __init nexell_timer_init(struct device_node *node)
{
	struct nexell_timer *timer;
	struct reset_control *reset;
	struct clk *clk;
	bool have_clk = false;
	u32 pclk_rate = 200000000;
	u32 tick_rate = 10000000;
	int irq;
	int ret;

	timer = nexell_timer;

	timer->base = of_io_request_and_map(node, 0, "nexell-timer");
	if (IS_ERR_OR_NULL(timer->base)) {
		ret = timer->base ? PTR_ERR(timer->base) : -ENOMEM;
		pr_err("nexell-timer: unable to map registers: %d\n", ret);
		return ret;
	}

	clk = of_clk_get_by_name(node, "timer");
	if (!IS_ERR(clk)) {
		ret = clk_prepare_enable(clk);
		if (ret)
			pr_warn("nexell-timer: unable to enable timer clock: %d\n",
				ret);
		else {
			pclk_rate = clk_get_rate(clk);
			have_clk = true;
		}
	}
	if (!have_clk)
		of_property_read_u32(node, "nexell,pclk-frequency", &pclk_rate);
	of_property_read_u32(node, "clock-frequency", &tick_rate);
	if (!pclk_rate || !tick_rate) {
		pr_err("nexell-timer: invalid clock frequencies\n");
		return -EINVAL;
	}

	spin_lock_init(&timer->lock);
	reset = of_reset_control_get_optional_exclusive(node, "timer");
	if (IS_ERR(reset) && PTR_ERR(reset) != -EPROBE_DEFER) {
		pr_err("nexell-timer: unable to get reset: %ld\n",
		       PTR_ERR(reset));
		return PTR_ERR(reset);
	}
	if (!IS_ERR_OR_NULL(reset)) {
		ret = reset_control_deassert(reset);
		if (ret)
			pr_warn("nexell-timer: unable to deassert reset: %d\n", ret);
	}
	nexell_timer_configure(timer, pclk_rate, tick_rate);

	nexell_timer_stop(timer, NEXELL_TIMER_SOURCE_CHANNEL);
	nexell_timer_stop(timer, NEXELL_TIMER_EVENT_CHANNEL);
	nexell_timer_load_raw(timer, NEXELL_TIMER_SOURCE_CHANNEL, U32_MAX);
	nexell_timer_start(timer, NEXELL_TIMER_SOURCE_CHANNEL, false, true);

	nexell_delay_timer.read_current_timer = nexell_timer_read_current_timer;
	nexell_delay_timer.freq = timer->rate;
	register_current_timer_delay(&nexell_delay_timer);

	timer->clocksource.name = "nexell-timer";
	timer->clocksource.rating = 300;
	timer->clocksource.read = nexell_timer_read_clocksource;
	timer->clocksource.mask = CLOCKSOURCE_MASK(32);
	timer->clocksource.flags = CLOCK_SOURCE_IS_CONTINUOUS;

	ret = clocksource_register_hz(&timer->clocksource, timer->rate);
	if (ret) {
		pr_err("nexell-timer: unable to register clocksource: %d\n", ret);
		return ret;
	}

	sched_clock_register(nexell_timer_read_sched_clock, 32, timer->rate);

	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		pr_err("nexell-timer: unable to map event IRQ\n");
		return -EINVAL;
	}

	timer->clockevent.name = "nexell-timer-event";
	/*
	 * ARM SMP supplies per-CPU dummy clockevents which depend on this
	 * single global timer as their broadcast source. Keep the hardware
	 * device below dummy_timer's rating (100), so it is released from
	 * CPU0's local tick slot and selected as the broadcast device.
	 */
	timer->clockevent.rating = 50;
	timer->clockevent.features = CLOCK_EVT_FEAT_PERIODIC |
		CLOCK_EVT_FEAT_ONESHOT;
	timer->clockevent.set_state_shutdown = nexell_timer_set_state_shutdown;
	timer->clockevent.set_state_oneshot = nexell_timer_set_state_oneshot;
	timer->clockevent.set_state_periodic = nexell_timer_set_state_periodic;
	timer->clockevent.tick_resume = nexell_timer_set_state_shutdown;
	timer->clockevent.set_next_event = nexell_timer_set_next_event;
	timer->clockevent.cpumask = cpumask_of(0);
	timer->clockevent.irq = irq;

	ret = request_irq(irq, nexell_timer_interrupt, IRQF_TIMER,
			  "nexell-timer", &timer->clockevent);
	if (ret) {
		pr_err("nexell-timer: unable to request IRQ %d: %d\n", irq, ret);
		return ret;
	}

	clockevents_config_and_register(&timer->clockevent, timer->rate, 1,
					U32_MAX);
	pr_info("nexell-timer: clocksource %lu Hz, event IRQ %d\n",
		timer->rate, irq);

	return 0;
}

TIMER_OF_DECLARE(nexell_s5p6818, "nexell,s5p6818-timer", nexell_timer_init);
