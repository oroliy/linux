/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_NEXELL_TIMER_H
#define _LINUX_NEXELL_TIMER_H

#include <linux/types.h>

#ifdef CONFIG_NEXELL_TIMER
bool nexell_timer_is_ready(void);
u64 nexell_timer_read_counter(void);
unsigned long nexell_timer_get_rate(void);
#else
static inline bool nexell_timer_is_ready(void)
{
	return false;
}

static inline u64 nexell_timer_read_counter(void)
{
	return 0;
}

static inline unsigned long nexell_timer_get_rate(void)
{
	return 0;
}
#endif

#endif /* _LINUX_NEXELL_TIMER_H */
