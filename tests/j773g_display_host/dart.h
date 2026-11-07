/* SPDX-License-Identifier: MIT */
/* Host stand-in for src/dart.h: only the read-only locked-table walk exists here, so a DART
 * write or TLB operation in the code under test would not even link. */
#ifndef DART_H
#define DART_H
#include "types.h"
int dart_walk_locked_t8110(uintptr_t base, u64 size, u8 sid, u64 dram_base, u64 dram_size,
                           int (*visit)(u64 iova, u64 paddr, void *opaque), void *opaque);
#endif
