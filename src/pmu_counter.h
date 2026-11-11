/* SPDX-License-Identifier: MIT */

#ifndef PMU_COUNTER_H
#define PMU_COUNTER_H

#include "types.h"

int pmu_clear_panic_counter(void);
int pmu_read_boot_record(u8 *counts, u8 *error_stage);
/* TEST ONLY: J773s boot-hang breadcrumbs (main.c, J773S_CRUMBS). */
int pmu_set_boot_stage(u8 stage);

#endif
