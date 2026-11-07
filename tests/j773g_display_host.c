/* SPDX-License-Identifier: MIT */
/*
 * Host test of the J773g main-DCP display handoff (src/kboot_display_j773g.c) against what the
 * U1 test kernel reads (its TEST ONLY patches 0012-0018: the DCP at /soc/dcp@30ae00000 with
 * apple,firmware-uuid, six asc-mem regions with the __OS_LOG one physical, iommu-addresses on
 * every DART-mapped one, the display subsystem's region plus its "framebuffer" region without
 * iommu-addresses, PIODMA's region, and the disp0/disp0_piodma aliases).
 *
 * The ADT is modelled from the J773g 26.6.2 dump (m1n1-display/adt/t8132-j773-26.6.2.txt): the
 * six iop-dcp-nub segments, /vram, the DART register windows and vm-base. The locked DARTs are a
 * model of what iBoot leaves (dart-dcp SID 23 maps every segment but __OS_LOG, dart-disp0 SID 0
 * maps segment 4 and SID 16 segment 5); only the read-only walk exists, so a DART write would
 * not link, and no PMGR function exists at all.
 *
 *   j773g-display-host <in.dtb> <scenario> [out.dtb]
 *   j773g-display-host list
 *
 * Each scenario prints what it checked and exits 0 if every check passed.
 */

#include "../build/build_cfg.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "adt.h"
#include "dart.h"
#include "firmware.h"
#include "kboot_display_j773g.h"
#include "soc.h"
#include "types.h"
#include "utils.h"

#include "libfdt/libfdt.h"

void *adt = (void *)1;
u32 chip_id = T8132, board_id = 0x2a;
struct fw_version_info os_firmware = {V26_6_2, "26.6.2", {26, 6, 2}, 3, "iBoot-13822.140.2"};
#ifdef J773G_DISPLAY_START
struct j773g_start_state j773g_start_state;
#endif

static int failures;
#define CHECK(cond, ...)                                                                           \
    do {                                                                                           \
        if (cond) {                                                                                \
            printf("ok   " __VA_ARGS__);                                                           \
        } else {                                                                                   \
            printf("FAIL " __VA_ARGS__);                                                           \
            failures++;                                                                            \
        }                                                                                          \
        printf("\n");                                                                              \
    } while (0)

/* --- the ADT model ------------------------------------------------------------------------ */

enum { N_ROOT, N_CHOSEN, N_NUB, N_DART_DCP, N_DART_DISP, N_VRAM, N_COUNT };
static const char *const paths[N_COUNT] = {
    "/", "/chosen", "/arm-io/dcp/iop-dcp-nub", "/arm-io/dart-dcp", "/arm-io/dart-disp0", "/vram"};

/* iop-dcp-nub segment-ranges, decoded from the 26.6.2 dump. */
static const struct adt_segment_ranges j773g_segments[7] = {
    {0x100002a8000, 0x0, 0x10001c4c000, 0x806000, 1},
    {0x103e4f94000, 0x806000, 0x10002454000, 0x325000, 0},
    {0x10003f0c000, 0xb2b000, 0x10003f0c000, 0x24000, 10},
    {0x103e5c1c000, ~0UL, 0x10000b70000, 0x1000000, 2},
    {0x103e5c18000, ~0UL, 0x10001b88000, 0x4000, 2},
    {0x103e5b58000, ~0UL, 0x10001b8c000, 0xc0000, 2},
    {0x103e5a98000, ~0UL, 0x10001c4c000 + 0x900000, 0x4000, 2}, /* a J713-like seventh */
};

static struct {
    bool j773gap;
    char target[8];
    u32 nsegs;
    struct adt_segment_ranges seg[7];
    char uuid[37];
    char names[64];
    u64 dram_base, dram_size;
    u64 dart_dcp[2], dart_disp[2], vram[2];
    u64 vm_base;
    u32 running, preloaded;
} m;
static int adt_reads;

static void adt_model_j773g(void)
{
    memset(&m, 0, sizeof(m));
    m.j773gap = true;
    strcpy(m.target, "J773g");
    m.nsegs = 6;
    memcpy(m.seg, j773g_segments, sizeof(m.seg));
    strcpy(m.uuid, "81BAF476-679D-3CC0-8C6A-E6B6FA659D5E");
    strcpy(m.names, "__TEXT;__DATA;__OS_LOG;unknown;unknown;unknown");
    m.dram_base = 0x10000000000;
    m.dram_size = 0x400000000;
    m.dart_dcp[0] = 0x30a340000;
    m.dart_dcp[1] = 0x20000;
    m.dart_disp[0] = 0x30a300000;
    m.dart_disp[1] = 0xc000;
    m.vram[0] = 0x103e5300000;
    m.vram[1] = 0x854000;
    m.vm_base = 0x10000000000;
    m.running = m.preloaded = 1;
}

int adt_path_offset(const void *tree, const char *path)
{
    adt_reads++;
    for (int i = 0; i < N_COUNT; i++)
        if (!strcmp(path, paths[i]))
            return i;
    return -1;
}

int adt_path_offset_trace(const void *tree, const char *path, int *offsets)
{
    int node = adt_path_offset(tree, path);
    offsets[0] = node;
    offsets[1] = 0;
    return node;
}

int adt_get_reg(const void *tree, int *path, const char *prop, int idx, u64 *addr, u64 *size)
{
    adt_reads++;
    const u64 *reg = path[0] == N_DART_DCP    ? m.dart_dcp
                     : path[0] == N_DART_DISP ? m.dart_disp
                     : path[0] == N_VRAM      ? m.vram
                                              : NULL;
    if (!reg || idx || strcmp(prop, "reg"))
        return -1;
    *addr = reg[0];
    if (size)
        *size = reg[1];
    return 0;
}

bool adt_is_compatible(const void *tree, int node, const char *compat)
{
    adt_reads++;
    return node == N_ROOT && m.j773gap && !strcmp(compat, "J773gAP");
}

const void *adt_getprop(const void *tree, int node, const char *name, u32 *lenp)
{
    adt_reads++;
    u32 len = 0;
    const void *val = NULL;
    if (node == N_ROOT && !strcmp(name, "target-type")) {
        val = m.target;
        len = strlen(m.target) + 1;
    } else if (node == N_CHOSEN && !strcmp(name, "dram-base")) {
        val = &m.dram_base;
        len = 8;
    } else if (node == N_CHOSEN && !strcmp(name, "dram-size")) {
        val = &m.dram_size;
        len = 8;
    } else if (node == N_NUB && !strcmp(name, "uuid")) {
        val = m.uuid;
        len = sizeof(m.uuid);
    } else if (node == N_NUB && !strcmp(name, "segment-names")) {
        val = m.names;
        len = strlen(m.names) + 1;
    } else if (node == N_NUB && !strcmp(name, "segment-ranges")) {
        val = m.seg;
        len = m.nsegs * sizeof(m.seg[0]);
    } else if (node == N_NUB && !strcmp(name, "running")) {
        val = &m.running;
        len = 4;
    } else if (node == N_NUB && !strcmp(name, "pre-loaded")) {
        val = &m.preloaded;
        len = 4;
    } else if ((node == N_DART_DCP || node == N_DART_DISP) && !strcmp(name, "vm-base")) {
        val = &m.vm_base;
        len = 8;
    }
    if (val && lenp)
        *lenp = len;
    return val;
}

int adt_getprop_copy(const void *tree, int node, const char *name, void *out, size_t len)
{
    u32 have = 0;
    const void *val = adt_getprop(tree, node, name, &have);
    if (!val || have != len)
        return -1;
    memcpy(out, val, len);
    return len;
}

/* --- the locked DARTs, as iBoot leaves them; read-only walk ------------------------------- */

static int walks;
static enum { DART_OK, DART_EXTRA, DART_MISSING, DART_WRONG_PHYS, DART_UNREADABLE } dart_fault;

int dart_walk_locked_t8110(uintptr_t base, u64 size, u8 sid, u64 dram_base, u64 dram_size,
                           int (*visit)(u64 iova, u64 paddr, void *opaque), void *opaque)
{
    walks++;
    if (size != 0xc000 || dram_base != m.dram_base || dram_size != m.dram_size)
        return -1;
    if (dart_fault == DART_UNREADABLE)
        return -1;
    u32 mask;
    if (base == 0x30a340000 && sid == 23)
        mask = 0x3b; /* every segment but __OS_LOG */
    else if (base == 0x30a300000 && sid == 0)
        mask = 0x10;
    else if (base == 0x30a300000 && sid == 16)
        mask = 0x20;
    else
        return -1;
    if (dart_fault == DART_EXTRA && sid == 0) {
        /* a /vram leaf at table offset 0x4000, as a loader session would leave it */
        if (visit(0x4000, m.vram[0], opaque))
            return -1;
    }
    for (u32 i = 0; i < 6; i++) {
        if (!(mask & (1u << i)))
            continue;
        u64 start = m.seg[i].remap & ((1UL << 36) - 1);
        u64 span = ALIGN_UP((u64)m.seg[i].size, SZ_16K);
        for (u64 off = 0; off < span; off += SZ_16K) {
            if (dart_fault == DART_MISSING && sid == 23 && i == 1 && off + SZ_16K == span)
                continue;
            u64 phys = m.seg[i].phys + off;
            if (dart_fault == DART_WRONG_PHYS && sid == 16 && off == 0)
                phys += SZ_16K;
            if (visit(start + off, phys, opaque))
                return -1;
        }
    }
    return 0;
}

int firmware_set_fdt(void *fdt, int node, const char *prop, const struct fw_version_info *ver)
{
    fdt32_t v[3] = {cpu_to_fdt32(ver->num[0]), cpu_to_fdt32(ver->num[1]),
                    cpu_to_fdt32(ver->num[2])};
    return fdt_setprop(fdt, node, prop, v, sizeof(v));
}

/* --- the FDT side --------------------------------------------------------------------------- */

#define DCP     "/soc/dcp@30ae00000"
#define PIODMA  "/soc/dcp@30ae00000/piodma"
#define DISPLAY "/soc/display-subsystem"

static unsigned char *blob;
static size_t blob_size;

static void load(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        perror(path);
        exit(2);
    }
    unsigned char in[1 << 16];
    size_t n = fread(in, 1, sizeof(in), f);
    fclose(f);
    blob_size = n + (64 << 10);
    blob = calloc(1, blob_size);
    if (fdt_open_into(in, blob, blob_size)) {
        fprintf(stderr, "bad dtb\n");
        exit(2);
    }
}

static int node(const char *path)
{
    return fdt_path_offset(blob, path);
}

static void set_knob(const char *value)
{
    if (value && fdt_setprop_string(blob, node("/chosen"), J773G_DISPLAY_KNOB, value))
        exit(2);
}

static bool str_prop(const char *path, const char *prop, const char *want)
{
    int len = 0;
    const char *v = fdt_getprop(blob, node(path), prop, &len);
    return v && len == (int)strlen(want) + 1 && !strcmp(v, want);
}

static u32 cell(const char *path, const char *prop, int i)
{
    int len = 0;
    const fdt32_t *v = fdt_getprop(blob, node(path), prop, &len);
    return v && len >= 4 * (i + 1) ? fdt32_ld(&v[i]) : 0;
}

static int cells(const char *path, const char *prop)
{
    int len = 0;
    return fdt_getprop(blob, node(path), prop, &len) ? len / 4 : -1;
}

static u32 phandle(const char *path)
{
    return fdt_get_phandle(blob, node(path));
}

/* One reserved-memory region: asc-mem, no-map, reg as given; returns its node. */
static int region(u32 ph, u64 addr, u64 size)
{
    int n = fdt_node_offset_by_phandle(blob, ph);
    int len = 0;
    const fdt64_t *reg = n < 0 ? NULL : fdt_getprop(blob, n, "reg", &len);
    if (!reg || len != 16 || fdt64_ld(&reg[0]) != addr || fdt64_ld(&reg[1]) != size ||
        fdt_node_check_compatible(blob, n, "apple,asc-mem") ||
        !fdt_getprop(blob, n, "no-map", NULL))
        return -1;
    return n;
}

/* The iommu-addresses tuples of a region, as (owner, iova, size), in order. */
static int tuples(int n, u32 owner[4], u64 iova[4], u64 size[4])
{
    int len = 0;
    const fdt32_t *v = fdt_getprop(blob, n, "iommu-addresses", &len);
    if (!v)
        return 0;
    if (len % 20 || len / 20 > 4)
        return -1;
    for (int i = 0; i < len / 20; i++) {
        owner[i] = fdt32_ld(&v[5 * i]);
        iova[i] = ((u64)fdt32_ld(&v[5 * i + 1]) << 32) | fdt32_ld(&v[5 * i + 2]);
        size[i] = ((u64)fdt32_ld(&v[5 * i + 3]) << 32) | fdt32_ld(&v[5 * i + 4]);
    }
    return len / 20;
}

/* What the U1 tree carried for each segment (asc-firmware, dcp_data, region57/94/95). */
static const struct {
    u64 phys, size, iova;
    int owners; /* 1: dcp; 2: dcp then the display subsystem; 3: dcp then piodma; 0: __OS_LOG */
} u1[6] = {
    {0x100002a8000, 0x808000, 0x10001c4c000, 1},
    {0x103e4f94000, 0x328000, 0x10002454000, 1},
    {0x10003f0c000, 0x24000, 0, 0},
    {0x103e5c1c000, 0x1000000, 0x10000b70000, 1},
    {0x103e5c18000, 0x4000, 0x10001b88000, 2},
    {0x103e5b58000, 0xc0000, 0x10001b8c000, 3},
};

static void check_published(void)
{
    u32 dcp = phandle(DCP), display = phandle(DISPLAY), piodma = phandle(PIODMA);
    CHECK(dcp && display && piodma, "dcp, display subsystem and piodma have phandles");
    CHECK(str_prop(DCP, "status", "okay"), "dcp enabled");
    CHECK(str_prop(DCP, "apple,firmware-uuid", "81BAF476-679D-3CC0-8C6A-E6B6FA659D5E"),
          "dcp apple,firmware-uuid is the J773g 26.6.2 image");
    CHECK(cells(DCP, "apple,firmware-version") == 3 &&
              cell(DCP, "apple,firmware-version", 0) == 26 &&
              cell(DCP, "apple,firmware-version", 1) == 6 &&
              cell(DCP, "apple,firmware-version", 2) == 2,
          "dcp apple,firmware-version 26.6.2");
    CHECK(cells(DCP, "apple,firmware-compat") == 3 && cell(DCP, "apple,firmware-compat", 0) == 26,
          "dcp apple,firmware-compat 26.6.2");
    CHECK(cells(DCP, "aurora,j773g-dcp-firmware-ready") == 1 &&
              cell(DCP, "aurora,j773g-dcp-firmware-ready", 0) == 1,
          "dcp aurora,j773g-dcp-firmware-ready = <1>");
    CHECK(!fdt_getprop(blob, node(DCP), "apple,t8132-dcpext-firmware-ready", NULL),
          "no dcpext readiness marker");
    CHECK(cells(DCP, "memory-region") == 6, "dcp has six memory regions");

    for (int i = 0; i < 6; i++) {
        int n = region(cell(DCP, "memory-region", i), u1[i].phys, u1[i].size);
        CHECK(n >= 0, "region %d at 0x%lx size 0x%lx, asc-mem, no-map", i, u1[i].phys, u1[i].size);
        if (n < 0)
            continue;
        u32 owner[4];
        u64 iova[4], size[4];
        int t = tuples(n, owner, iova, size);
        bool oslog = fdt_getprop(blob, n, "apple,dcp-os-log", NULL);
        if (!u1[i].owners) {
            CHECK(oslog && t == 0 && u1[i].size == 0x24000,
                  "region %d is __OS_LOG: apple,dcp-os-log, 0x24000, physical (no "
                  "iommu-addresses)",
                  i);
            continue;
        }
        CHECK(!oslog, "region %d is not marked as the OS log", i);
        int want = u1[i].owners == 1 ? 1 : 2;
        bool ok = t == want && owner[0] == dcp && iova[0] == u1[i].iova && size[0] == u1[i].size;
        if (ok && want == 2)
            ok = owner[1] == (u1[i].owners == 2 ? display : piodma) && iova[1] == u1[i].iova &&
                 size[1] == u1[i].size;
        CHECK(ok, "region %d iommu-addresses: %s at DVA 0x%lx (bit 40 set) as in U1", i,
              u1[i].owners == 1   ? "dcp"
              : u1[i].owners == 2 ? "dcp, display"
                                  : "dcp, piodma",
              u1[i].iova);
    }

    CHECK(str_prop(DISPLAY, "status", "okay") && cells(DISPLAY, "memory-region") == 2 &&
              cell(DISPLAY, "memory-region", 0) == cell(DCP, "memory-region", 4),
          "display subsystem enabled with segment 4 first");
    int len = 0;
    const char *names = fdt_getprop(blob, node(DISPLAY), "memory-region-names", &len);
    CHECK(names && len == sizeof("asc-firmware\0framebuffer") &&
              !memcmp(names, "asc-firmware\0framebuffer", len),
          "display memory-region-names: asc-firmware, framebuffer");
    int fb = region(cell(DISPLAY, "memory-region", 1), 0x103e5300000, 0x854000);
    CHECK(fb >= 0 && !fdt_getprop(blob, fb, "iommu-addresses", NULL),
          "framebuffer region is /vram 0x103e5300000+0x854000, asc-mem no-map, no "
          "iommu-addresses (U1 patch 0016)");
    CHECK(str_prop(PIODMA, "status", "okay") && cells(PIODMA, "memory-region") == 1 &&
              cell(PIODMA, "memory-region", 0) == cell(DCP, "memory-region", 5),
          "piodma enabled with segment 5");
    const char *group[] = {"/soc/iommu@30a300000", "/soc/iommu@30a340000", "/soc/mailbox@30ae08000",
                           "/soc/phy@39a700000"};
    for (size_t i = 0; i < ARRAY_SIZE(group); i++)
        CHECK(str_prop(group[i], "status", "okay"), "%s enabled", group[i]);
    CHECK(str_prop("/aliases", "dcp", DCP) && str_prop("/aliases", "disp0", DISPLAY) &&
              str_prop("/aliases", "disp0_piodma", PIODMA),
          "late aliases dcp, disp0, disp0_piodma name the validated nodes");
    CHECK(str_prop("/chosen", "aurora,j773g-display-handoff", "iboot-running"),
          "chosen aurora,j773g-display-handoff = iboot-running");
    CHECK(fdt_getprop(blob, node(DCP), "aurora,j773g-hdmi-test", NULL),
          "the tree's own kernel opt-in is kept");
}

/* --- scenarios ------------------------------------------------------------------------------ */

enum expect { PUBLISH, NOTHING, STARTED };

static struct j773g_display_env env = {
    .loader_base = 0x10001400000,
    .loader_size = 0x400000,
    .exec_base = 0x10010000000,
    .exec_size = 0x20000000,
};

static const char *knob = "handoff";
static int published_twice;

static void s_off_absent(void)
{
    knob = NULL;
}
static void s_off(void)
{
    knob = "off";
}
static void s_off_unknown(void)
{
    knob = "yes";
}
static void s_handoff(void)
{
}
static void s_repeat(void)
{
    published_twice = 1;
}
static void s_j713_tree(void)
{
    fdt_setprop_string(blob, 0, "compatible", "apple,j713");
}
static void s_j713_adt(void)
{
    m.j773gap = false;
    strcpy(m.target, "J713");
    m.nsegs = 7;
    strcpy(m.names, "__TEXT;__DATA;__OS_LOG;unknown;unknown;unknown;unknown");
}
static void s_seven_segments(void)
{
    m.nsegs = 7;
}
static void s_wrong_chip(void)
{
    chip_id = T8122;
}
static void s_wrong_board(void)
{
    board_id = 0x2b;
}
static void s_wrong_firmware(void)
{
    os_firmware.version = V26_5;
}
static void s_wrong_uuid(void)
{
    m.uuid[0] = '0';
}
static void s_wrong_kind(void)
{
    m.seg[4].unk = 0;
}
static void s_oslog_size(void)
{
    m.seg[2].size = 0x20000;
}
static void s_dart_window(void)
{
    m.dart_disp[1] = 0x4000;
}
static void s_dart_extra(void)
{
    dart_fault = DART_EXTRA;
}
static void s_dart_missing(void)
{
    dart_fault = DART_MISSING;
}
static void s_dart_phys(void)
{
    dart_fault = DART_WRONG_PHYS;
}
static void s_dart_unreadable(void)
{
    dart_fault = DART_UNREADABLE;
}
static void s_oslog_memreserve(void)
{
    fdt_add_mem_rsv(blob, 0x10003000000, 0x2000000);
}
static void s_oslog_loader(void)
{
    env.loader_base = 0x10003f00000;
}
static void s_oslog_exec(void)
{
    env.exec_base = 0x10003000000;
}
static void s_oslog_reserved(void)
{
    int n = fdt_add_subnode(blob, node("/reserved-memory"), "other@10003f20000");
    fdt64_t reg[2] = {cpu_to_fdt64(0x10003f20000), cpu_to_fdt64(0x4000)};
    fdt_setprop(blob, n, "reg", reg, sizeof(reg));
}
static void s_static_tree(void)
{
    fdt_setprop_u32(blob, node(DCP), "memory-region", 1);
}
static void s_alias_elsewhere(void)
{
    fdt_setprop_string(blob, node("/aliases"), "disp0", "/soc/dcp@30ae00000");
}
static void s_alias_same(void)
{
    fdt_setprop_string(blob, node("/aliases"), "dcp", DCP);
}
static void s_no_phy(void)
{
    fdt_delprop(blob, node(DCP), "phys");
}
static void s_display_compat(void)
{
    fdt_setprop_string(blob, node(DISPLAY), "compatible", "apple,t8132-display-subsystem");
}
static bool pack_after_knob;
static void s_no_space(void)
{
    /* Exactly full: every addition fails, the scratch copy must be discarded. */
    pack_after_knob = true;
}
static void s_start(void)
{
    knob = "start";
}
#ifdef J773G_DISPLAY_START
static void s_start_not_run(void)
{
    knob = "start";
}
static void s_start_untouched(void)
{
    knob = "start";
    j773g_start_state = (struct j773g_start_state){.ran = true, .stop = "not built"};
}
static void s_start_surface(void)
{
    knob = "start";
    j773g_start_state = (struct j773g_start_state){.ran = true, .touched = true, .surface = true};
}
static void s_start_failed(void)
{
    knob = "start";
    j773g_start_state = (struct j773g_start_state){.ran = true, .touched = true, .stop = "hpd"};
}
#endif

static const struct {
    const char *name;
    enum expect expect;
    void (*setup)(void);
} scenarios[] = {
    {"off-absent", NOTHING, s_off_absent},
    {"off", NOTHING, s_off},
    {"off-unknown", NOTHING, s_off_unknown},
    {"handoff", PUBLISH, s_handoff},
    {"repeat", PUBLISH, s_repeat},
    {"j713-tree", NOTHING, s_j713_tree},
    {"j713-adt", NOTHING, s_j713_adt},
    {"seven-segments", NOTHING, s_seven_segments},
    {"wrong-chip", NOTHING, s_wrong_chip},
    {"wrong-board", NOTHING, s_wrong_board},
    {"wrong-firmware", NOTHING, s_wrong_firmware},
    {"wrong-uuid", NOTHING, s_wrong_uuid},
    {"wrong-kind", NOTHING, s_wrong_kind},
    {"oslog-size", NOTHING, s_oslog_size},
    {"dart-window", NOTHING, s_dart_window},
    {"dart-extra-leaf", NOTHING, s_dart_extra},
    {"dart-missing-leaf", NOTHING, s_dart_missing},
    {"dart-wrong-phys", NOTHING, s_dart_phys},
    {"dart-unreadable", NOTHING, s_dart_unreadable},
    {"oslog-memreserve", NOTHING, s_oslog_memreserve},
    {"oslog-loader", NOTHING, s_oslog_loader},
    {"oslog-exec", NOTHING, s_oslog_exec},
    {"oslog-reserved", NOTHING, s_oslog_reserved},
    {"static-tree", NOTHING, s_static_tree},
    {"alias-elsewhere", NOTHING, s_alias_elsewhere},
    {"alias-same", PUBLISH, s_alias_same},
    {"no-phy", NOTHING, s_no_phy},
    {"display-compat", NOTHING, s_display_compat},
    {"no-space", NOTHING, s_no_space},
#ifdef J773G_DISPLAY_START
    {"start-not-run", NOTHING, s_start_not_run},
    {"start-untouched", NOTHING, s_start_untouched},
    {"start-surface", STARTED, s_start_surface},
    {"start-failed", STARTED, s_start_failed},
#else
    {"start-default-build", NOTHING, s_start},
#endif
};

int main(int argc, char **argv)
{
    if (argc == 2 && !strcmp(argv[1], "list")) {
        for (size_t i = 0; i < ARRAY_SIZE(scenarios); i++)
            printf("%s\n", scenarios[i].name);
        return 0;
    }
    if (argc < 3)
        return 2;
    size_t which = ARRAY_SIZE(scenarios);
    for (size_t i = 0; i < ARRAY_SIZE(scenarios); i++)
        if (!strcmp(argv[2], scenarios[i].name))
            which = i;
    if (which == ARRAY_SIZE(scenarios))
        return 2;

    load(argv[1]);
    adt_model_j773g();
    scenarios[which].setup();
    set_knob(knob);
    if (pack_after_knob)
        fdt_pack(blob);

    size_t total = fdt_totalsize(blob);
    unsigned char *before = malloc(total);
    memcpy(before, blob, total);
    adt_reads = walks = 0;

    int ret = kboot_j773g_display(blob, &env);
    printf("kboot_j773g_display() = %d; %d ADT reads, %d DART walks\n", ret, adt_reads, walks);

    switch (scenarios[which].expect) {
        case NOTHING:
            CHECK(ret == 0, "nothing published: the caller runs the stock path");
            CHECK(fdt_totalsize(blob) == total && !memcmp(before, blob, total),
                  "the FDT is byte-for-byte unchanged");
            if (!knob || !strcmp(knob, "off") || !strcmp(knob, "yes"))
                CHECK(!adt_reads && !walks, "off: no ADT read, no DART walk");
            break;
        case PUBLISH:
            CHECK(ret == 1, "published: the caller skips the stock path");
            CHECK(walks == 3, "three read-only DART stream walks (SIDs 23, 0, 16)");
            check_published();
            if (published_twice) {
                memcpy(before, blob, fdt_totalsize(blob));
                total = fdt_totalsize(blob);
                ret = kboot_j773g_display(blob, &env);
                CHECK(ret == 0 && !memcmp(before, blob, total),
                      "a second pass over a published tree refuses and changes nothing");
            }
            break;
        case STARTED:
            CHECK(ret == 1, "the loader session owns the display: the stock path is skipped");
            CHECK(!walks, "no DART walk after a loader session");
            CHECK(str_prop(DCP, "status", "disabled") && str_prop(DISPLAY, "status", "disabled") &&
                      !fdt_getprop(blob, node(DCP), "aurora,j773g-dcp-firmware-ready", NULL) &&
                      !fdt_getprop(blob, node(DCP), "memory-region", NULL) &&
                      !fdt_get_alias(blob, "dcp") && !fdt_get_alias(blob, "disp0"),
                  "the DCP group stays disabled, unmarked and alias-free for Linux");
            int resv = node("/reserved-memory"), count = 0, n;
            fdt_for_each_subnode(n, blob, resv) count++;
            CHECK(count == 1 + 7, "six segments and /vram reserved");
            CHECK(fdt_subnode_offset(blob, resv, "asc-firmware@10003f0c000") >= 0 &&
                      fdt_subnode_offset(blob, resv, "framebuffer@103e5300000") >= 0,
                  "__OS_LOG and /vram among them");
#ifdef J773G_DISPLAY_START
            CHECK(str_prop("/chosen", "aurora,j773g-display-handoff",
                           j773g_start_state.surface ? "simpledrm-only" : "cold-required"),
                  "chosen aurora,j773g-display-handoff = %s",
                  j773g_start_state.surface ? "simpledrm-only" : "cold-required");
#endif
            break;
    }

    if (argc > 3) {
        FILE *f = fopen(argv[3], "wb");
        fdt_pack(blob);
        fwrite(blob, 1, fdt_totalsize(blob), f);
        fclose(f);
    }
    printf("%s: %s\n", scenarios[which].name, failures ? "FAILED" : "passed");
    return failures ? 1 : 0;
}
