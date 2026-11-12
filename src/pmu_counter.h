/* SPDX-License-Identifier: MIT */

#ifndef PMU_COUNTER_H
#define PMU_COUNTER_H

#include "types.h"

int pmu_clear_panic_counter(void);
int pmu_read_boot_record(u8 *counts, u8 *error_stage);
#ifdef J773S_STAGE1_CRUMBS
void j773s_stage1_crumb(u8 stage);
#endif

#endif
