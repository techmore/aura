/* SPDX-License-Identifier: MIT */

#ifndef ANS_POWER_H
#define ANS_POWER_H

#include "types.h"

/* T8140: ANS's and APCIe's power cycled while the SoC stays awake, for an ANS restart. */
int ans_power_prepare(void);
/* The number of switch owners that didn't power-gate. */
int ans_power_down(void);
int ans_power_up(void);

#endif
