// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Copyright (c) 2025 Nick Chan
 */

#include <asm/mach/arch.h>

static const char * const s5l8747_dt_compat[] __initconst = {
	"samsung,s5l8747",
	NULL,
};

DT_MACHINE_START(S5L8747X, "Samsung S5L8747")
	.dt_compat	= s5l8747_dt_compat,
MACHINE_END
