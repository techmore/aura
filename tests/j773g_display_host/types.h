/* SPDX-License-Identifier: MIT */
/* Host stand-in for src/types.h. u64 is unsigned long, as on aura's aarch64 target. */
#ifndef TYPES_H
#define TYPES_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef unsigned long u64;
typedef int32_t s32;
typedef int64_t s64;
_Static_assert(sizeof(u64) == 8, "the J773g display host test requires an LP64 host");
#define PACKED    __attribute__((packed))
#define UNUSED(x) (void)(x)
#endif
