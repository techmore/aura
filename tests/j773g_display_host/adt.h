/* SPDX-License-Identifier: MIT */
/* Host stand-in for src/adt.h: tests/j773g_display_host.c models the ADT. */
#ifndef ADT_H
#define ADT_H
#include "types.h"
extern void *adt;
struct adt_segment_ranges {
    u64 phys;
    u64 iova;
    u64 remap;
    u32 size;
    u32 unk;
} PACKED;
int adt_path_offset(const void *adt, const char *path);
int adt_path_offset_trace(const void *adt, const char *path, int *offsets);
const void *adt_getprop(const void *adt, int nodeoffset, const char *name, u32 *lenp);
int adt_getprop_copy(const void *adt, int nodeoffset, const char *name, void *out, size_t len);
int adt_get_reg(const void *adt, int *path, const char *prop, int idx, u64 *addr, u64 *size);
bool adt_is_compatible(const void *adt, int nodeoffset, const char *compat);
#define ADT_GETPROP(adt, nodeoffset, name, val)                                                    \
    adt_getprop_copy(adt, nodeoffset, name, (val), sizeof(*(val)))
#endif
