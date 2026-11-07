# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run the production T8132 AOP handoff gate on J713 and on boards it does not qualify."""
from pathlib import Path

from test_j713_dapf_instance import block
from test_siliconforest_handoff import run_c

ROOT = Path(__file__).resolve().parents[2]

# What can go wrong: an unqualified board keeps an enabled AOP that no loader handed over
# (the OS then starts it without its DAPF or its DRAM segments); the gate touches the AOP
# or its DART on a board it does not qualify; a J713 refusal leaves it enabled.
HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "libfdt.h"
typedef uint32_t u32;
typedef unsigned long u64;
typedef struct { int unused; } asc_dev_t;
#define PACKED __attribute__((packed))
#define bail(...) do { printf(__VA_ARGS__); return -1; } while (0)
static void *dt;
static asc_dev_t the_asc;
static int touched, fail;
static bool running;
static asc_dev_t *asc_init(const char *path)
{ assert(!strcmp(path, "/arm-io/aop")); touched++; return fail == 1 ? NULL : &the_asc; }
static bool asc_cpu_running(asc_dev_t *asc) { return running; }
static void asc_free(asc_dev_t *asc) {}
static int dapf_init_t8132_halted(const char *path, int index)
{ assert(!strcmp(path, "/arm-io/dart-aop") && index == 1); touched++; return fail == 2 ? -1 : 0; }
static int dt_reserve_asc_segments(const char *adt_path, const char *alt, const char *path,
                                   bool remap, u64 base, bool dram_only)
{ assert(!strcmp(adt_path, "/arm-io/aop/iop-aop-nub") && remap && dram_only); touched++;
  return fail == 3 ? -1 : 0; }
static int dt_reserve_t8132_aop_windows(const char *path) { touched++; return fail == 4 ? -1 : 0; }
/*
 * The J773g ADT (macOS 26.6.2) facts the J773g admission checks: aop reg[0], its nub's
 * segment names, segment count and pre-loaded flag, dart-aop reg[1] (its DAPF bank) and
 * that bank's 16 filter slices. A case below breaks one of them.
 */
static bool live_j773g;
static bool kboot_is_j773g(const void *fdt) { return live_j773g; }
static void *adt;
enum { AOP = 1, NUB, DART };
static u64 aop_base, aop_size, dapf_base, dapf_size;
static const char *names;
static u32 segments_len, preloaded, preloaded_len, dapf_len;
static unsigned char segments[4 * 32], dapf[17 * 52];
static void j773g_adt(void)
{
    live_j773g = true;
    aop_base = 0x390600000; aop_size = 0x88000;   /* bus 0x190600000 + arm-io 0x200000000 */
    names = "__TEXT;__DATA;__ETEXT;__OS_LOG";
    segments_len = 4 * 32;
    preloaded = 1; preloaded_len = 4;
    dapf_base = 0x390f10000; dapf_size = 0x4000;   /* bus 0x190f10000 */
    dapf_len = 16 * 52;
}
static int adt_path_offset(const void *tree, const char *path)
{
    if (!strcmp(path, "/arm-io/aop")) return AOP;
    if (!strcmp(path, "/arm-io/aop/iop-aop-nub")) return NUB;
    if (!strcmp(path, "/arm-io/dart-aop")) return DART;
    return -1;
}
static int adt_path_offset_trace(const void *tree, const char *path, int *trace)
{ int node = adt_path_offset(tree, path); trace[0] = node; trace[1] = 0; return node; }
static int adt_get_reg(const void *tree, int *trace, const char *prop, int idx, u64 *base, u64 *size)
{
    assert(!strcmp(prop, "reg"));
    if (trace[0] == AOP && idx == 0) { *base = aop_base; *size = aop_size; return 0; }
    if (trace[0] == DART && idx == 1) { *base = dapf_base; *size = dapf_size; return 0; }
    return -1;
}
static const void *adt_getprop(const void *tree, int node, const char *name, u32 *len)
{
    if (node == NUB && !strcmp(name, "segment-names") && names) {
        *len = strlen(names) + 1; return names;
    }
    if (node == NUB && !strcmp(name, "segment-ranges") && segments_len) {
        *len = segments_len; return segments;
    }
    if (node == NUB && !strcmp(name, "pre-loaded") && preloaded_len) {
        *len = preloaded_len; return &preloaded;
    }
    if (node == DART && !strcmp(name, "dapf-instance-0") && dapf_len) {
        *len = dapf_len; return dapf;
    }
    return NULL;
}
'''

MAIN = r'''
static unsigned char blob[4096];
static void tree(const char *board, bool requested)
{
    char compatible[64];
    int len = snprintf(compatible, sizeof(compatible), "%s%c%s", board, 0, "apple,t8132") + 1;
    dt = blob;
    assert(!fdt_create_empty_tree(dt, sizeof(blob)));
    assert(!fdt_setprop(dt, 0, "compatible", compatible, len));
    int chosen = fdt_add_subnode(dt, 0, "chosen");
    if (requested)
        assert(!fdt_setprop(dt, chosen, "linux-enablement-mac,dapf-handoff",
                            "/arm-io/aop\0/arm-io/pmp", sizeof("/arm-io/aop\0/arm-io/pmp")));
    int aop = fdt_add_subnode(dt, 0, "aop@390600000");
    assert(aop >= 0 && !fdt_setprop_string(dt, aop, "compatible", "apple,t8132-aop"));
    assert(!fdt_setprop_string(dt, aop, "status", "okay"));
    touched = fail = 0;
    running = false;
    live_j773g = false;
}
static const char *status(void)
{
    return fdt_getprop(dt, fdt_path_offset(dt, "/aop@390600000"), "status", NULL);
}
int main(void)
{
    /* J713: a requested, halted AOP is handed over; any failed step disables it. */
    tree("apple,j713", true);
    assert(!dt_set_t8132_aop("/aop@390600000") && !strcmp(status(), "okay") && touched == 4);
    for (int f = 1; f <= 4; f++) {
        tree("apple,j713", true);
        fail = f;
        assert(!dt_set_t8132_aop("/aop@390600000") && !strcmp(status(), "disabled"));
    }
    tree("apple,j713", true);
    running = true;
    assert(!dt_set_t8132_aop("/aop@390600000") && !strcmp(status(), "disabled") && touched == 1);
    tree("apple,j713", false);
    assert(!dt_set_t8132_aop("/aop@390600000") && !strcmp(status(), "disabled") && !touched);
    /* Other T8132 boards: the AOP ends disabled and neither it nor its DART is touched,
     * whether or not the tree asks for the handoff. */
    static const char *const others[] = {"apple,j604", "apple,j623", "apple,j624",
                                         "apple,j715", "apple,j773g"};
    for (unsigned i = 0; i < sizeof(others) / sizeof(*others); i++)
        for (int requested = 0; requested <= 1; requested++) {
            tree(others[i], requested);
            assert(!dt_set_t8132_aop("/aop@390600000"));
            assert(!strcmp(status(), "disabled") && !touched);
        }
    /* J773g on 26.6.2 with its own ADT: the J713 handoff, refused the same way. */
    tree("apple,j773g", true); j773g_adt();
    assert(!dt_set_t8132_aop("/aop@390600000") && !strcmp(status(), "okay") && touched == 4);
    for (int f = 1; f <= 4; f++) {
        tree("apple,j773g", true); j773g_adt();
        fail = f;
        assert(!dt_set_t8132_aop("/aop@390600000") && !strcmp(status(), "disabled"));
    }
    tree("apple,j773g", false); j773g_adt();
    assert(!dt_set_t8132_aop("/aop@390600000") && !strcmp(status(), "disabled") && !touched);
    /* Any other AOP shape is refused before the AOP or its DART is touched. */
    for (int c = 0; c < 12; c++) {
        tree("apple,j773g", true); j773g_adt();
        switch (c) {
        case 0: aop_base = 0x290600000; break;
        case 1: aop_size = 0x84000; break;
        case 2: names = "__TEXT;__DATA;__OS_LOG"; break;
        case 3: names = NULL; break;
        case 4: segments_len = 3 * 32; break;
        case 5: segments_len = 0; break;
        case 6: preloaded = 0; break;
        case 7: preloaded_len = 0; break;
        case 8: dapf_base = 0x390f00000; break;
        case 9: dapf_size = 0xc000; break;
        case 10: dapf_len = 15 * 52; break;
        case 11: dapf_len = 17 * 52; break;
        }
        assert(!dt_set_t8132_aop("/aop@390600000"));
        assert(!strcmp(status(), "disabled") && !touched);
    }
    return 0;
}
'''


def production():
    kboot = (ROOT / "src/kboot.c").read_text()
    header = (ROOT / "src/adt.h").read_text()
    start = header.index("struct adt_segment_ranges {")
    layout = header[start:header.index("} PACKED;", start) + len("} PACKED;")]
    return layout + "\n" + "\n".join(block(kboot, name) for name in (
        "static int dt_check_j773g_aop(", "static int dt_set_t8132_aop(")) + "\n"


def test_t8132_aop_gate(tmp_path):
    run_c(tmp_path, "t8132-aop-gate",
          HARNESS + production() + MAIN)
