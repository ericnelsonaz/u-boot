/*
 * Utility functions for executing as an i.MX plugin
 *
 * Copyright (c) 2016 Nelson Integration, LLC
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm-offsets.h>
#include <config.h>
#include <asm/system.h>
#include <linux/linkage.h>

ENTRY(save_boot_params)
	ldr	r10, =plugin_state
	stmia	r10, {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, fp, sp, lr}
	b	save_boot_params_ret
ENDPROC(save_boot_params)

ENTRY(return_to_rom)
	ldr	r10, =plugin_state
	ldmia	r10, {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, fp, sp, lr}
	bx	lr
ENDPROC(return_to_rom)
