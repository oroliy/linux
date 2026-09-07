/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 ARM Ltd.
 */
#ifndef __ASM_TIMEX_H
#define __ASM_TIMEX_H

#include <asm/arch_timer.h>

/*
 * Use the current timer as a cycle counter since this is what we use for
 * the delay loop.
 */
#ifdef CONFIG_NEXELL_TIMER
#include <linux/nexell_timer.h>
#define get_cycles()	(nexell_timer_is_ready() ? nexell_timer_read_counter() : arch_timer_read_counter())
#else
#define get_cycles()	arch_timer_read_counter()
#endif

#include <asm-generic/timex.h>

#endif
