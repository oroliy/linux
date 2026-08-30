/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/linkage.h>
#include <linux/init.h>

/*
 * S5P6818 secondary-CPU entry (2ndboot pen release target).
 *
 * Matches the vendor 3.4/4.4 pen exactly: bare holding-pen, no ACTLR or
 * CPUECTLR writes.  The vendor kernels boot all 8 cores with no register
 * writes here, relying on 2ndboot's SMPEN/CCI setup.
 */
ENTRY(__secondary_startup)
	mrc	p15, 0, r0, c0, c0, 5
	tst	r0, #0x4400
	orrne	r0, r0, #4
	and	r0, r0, #0xf
	adr	r4, 1f
	ldmia	r4, {r5, r6}
	sub	r4, r4, r5
	add	r6, r6, r4
pen:	ldr	r7, [r6]
	cmp	r7, r0
	bne	pen

	b	secondary_startup

	.align
1:	.long	.
	.long	pen_release
ENDPROC(__secondary_startup)
