// SPDX-License-Identifier: GPL-2.0-only
/* Nexell S5P6818 architecture-independent SoC quirks. */

#include <linux/init.h>
#include <linux/of.h>

bool nexell_s5p6818_poll_lock_wait;

static int __init nexell_s5p6818_init_quirks(void)
{
	if (of_machine_is_compatible("nexell,s5p6818")) {
		WRITE_ONCE(nexell_s5p6818_poll_lock_wait, true);
		pr_info("S5P6818: polling queued lock waiters across clusters\n");
	}

	return 0;
}
early_initcall(nexell_s5p6818_init_quirks);
