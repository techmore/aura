# SPDX-License-Identifier: MIT
"""dart_release_locked_l2(): which L2 tables a locked DART gives back at shutdown.

It runs for every locked DART on every chip (dart_shutdown_checked()), so its rule is pinned
here on the production body: only a table inside aura's heap range (is_heap(), an address-range
test) that maps nothing is unlinked, invalidated and then freed. A heap table that still maps a
page is kept and stays linked; a table outside the heap (the bootloader's) is never touched,
empty or not.
"""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]

HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef unsigned long u64;
#define BIT(n) (1ULL << (n))
#define GENMASK(h, l) (((~0ULL) << (l)) & (~0ULL >> (63 - (h))))
#define FIELD_PREP(m, v) (((u64)(v) << __builtin_ctzll(m)) & (m))
#define FIELD_GET(m, v) (((v) & (m)) >> __builtin_ctzll(m))
#define SZ_16K 0x4000
%(defines)s
typedef struct dart_dev dart_dev_t;
struct dart_params { int ttbr_count; u64 offset_mask; void (*tlb_invalidate)(dart_dev_t *); };
struct dart_dev { u64 regs; u8 device; const struct dart_params *params; u64 *l1[4]; };

/* Tables: [0] the root, [1..2] aura's heap, [3..4] the bootloader's. */
static u64 pool[5][SZ_16K / 8] __attribute__((aligned(SZ_16K)));
static bool is_heap(void *p) { return p == pool[1] || p == pool[2]; }
static int tlbis, frees;
static void *freed[4];
static dart_dev_t *shut;
static void invalidate(dart_dev_t *dart) { (void)dart; tlbis++; }
static void record_free(void *p)
{
    /* Unlinked and invalidated before the page goes back to the heap. */
    for (int i = 0; i < SZ_16K / 8; i++)
        assert(!(shut->l1[0][i] & DART_PTE_VALID) || (shut->l1[0][i] & ~DART_PTE_VALID) != (u64)p);
    assert(tlbis > frees);
    freed[frees++] = p;
}
#define free(p) record_free(p)
%(body)s
#undef free

static u64 pte(void *table, const struct dart_params *params)
{
    return FIELD_PREP(params->offset_mask, (u64)table >> DART_PTE_OFFSET_SHIFT) | DART_PTE_VALID;
}

int main(void)
{
    /* The host's table addresses must fit the PTE's offset field. */
    static const struct dart_params params = {2, GENMASK(63, 14), invalidate};
    dart_dev_t dart = {.regs = 0x1000, .device = 23, .params = &params, .l1 = {pool[0], NULL}};
    shut = &dart;
    pool[0][8] = pte(pool[1], &params); /* heap, empty */
    pool[0][9] = pte(pool[2], &params); /* heap, still maps one page */
    pool[2][5] = pte(pool[4], &params);
    pool[0][0] = pte(pool[3], &params); /* the bootloader's, empty */
    pool[0][1] = pte(pool[4], &params); /* the bootloader's, in use */
    pool[4][0] = pte(pool[3], &params);
    pool[0][2] = (u64)pool[1] & ~DART_PTE_VALID; /* not valid: never followed */
    u64 kept9 = pool[0][9], kept0 = pool[0][0], kept1 = pool[0][1], kept2 = pool[0][2];

    dart_release_locked_l2(&dart);
    assert(frees == 1 && freed[0] == pool[1] && tlbis == 1);
    assert(pool[0][8] == 0);
    assert(pool[0][9] == kept9 && pool[0][0] == kept0 && pool[0][1] == kept1 && pool[0][2] == kept2);

    /* Nothing left to release: no write, no invalidation. */
    dart_release_locked_l2(&dart);
    assert(frees == 1 && tlbis == 1 && pool[0][9] == kept9);
    puts("PASS locked L2 release");
    return 0;
}
'''


def test_only_empty_heap_tables_are_released(tmp_path):
    source = (ROOT / "src/dart.c").read_text()
    defines = "\n".join(re.search(r"^#define %s\b.*$" % name, source, re.M).group(0)
                        for name in ("DART_PTE_VALID", "DART_PTE_OFFSET_SHIFT"))
    start = source.index("static void dart_release_locked_l2(")
    body = source[start:source.index("\n}\n", start) + 3]
    code = tmp_path / "l2-release.c"
    code.write_text(HARNESS % {"defines": defines, "body": body})
    binary = tmp_path / "l2-release"
    compiled = subprocess.run(["cc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                               "-fsanitize=address,undefined", str(code), "-o", str(binary)],
                              capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr[-6000:]
    result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0 and "PASS locked L2 release" in result.stdout, \
        result.stdout[-2000:] + result.stderr[-3000:]
