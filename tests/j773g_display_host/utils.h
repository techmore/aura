/* SPDX-License-Identifier: MIT */
/* Host stand-in for src/utils.h: what the J773g display code uses. */
#ifndef UTILS_H
#define UTILS_H
#include <stdio.h>

#include "soc.h"
#include "types.h"
#define ARRAY_SIZE(s)  (sizeof(s) / sizeof((s)[0]))
#define ALIGN_UP(x, a) (((x) + ((a) - 1)) & ~((a) - 1))
#define BIT(x)         (1UL << (x))
#define SZ_16K         (16 * 1024)
extern u32 board_id, chip_id;
u64 timeout_calculate(u32 usec);
bool timeout_expired(u64 timeout);
void udelay(u32 d);
#define mdelay(m) udelay((m) * 1000)
#endif
