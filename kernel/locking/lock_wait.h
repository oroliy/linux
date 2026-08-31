/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LOCKING_LOCK_WAIT_H
#define __LOCKING_LOCK_WAIT_H

#ifndef arch_lock_cond_load_relaxed
#define arch_lock_cond_load_relaxed(ptr, cond_expr) \
	smp_cond_load_relaxed((ptr), cond_expr)
#endif

#ifndef arch_lock_cond_load_acquire
#define arch_lock_cond_load_acquire(ptr, cond_expr) \
	smp_cond_load_acquire((ptr), cond_expr)
#endif

#endif /* __LOCKING_LOCK_WAIT_H */
