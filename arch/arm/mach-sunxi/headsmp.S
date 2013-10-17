/*
 * SMP support for A20
 *
 *  Copyright (C) 2013 Fan Rong <cinifr@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/linkage.h>
#include <linux/init.h>

.section ".text.head", "ax"
ENTRY(sun7i_secondary_startup)
	msr cpsr_fsxc,#0xd3
	b secondary_startup
ENDPROC(sun7i_secondary_startup)
