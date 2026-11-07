/* SPDX-License-Identifier: MIT */

/*
 * J773g (M4 Mac mini, T8132, board-id 42): the HDMI port is driven by the main DCP
 * (/arm-io/dcp, firmware 81BAF476-...) through the T8122 DPTX PHY and a DP-to-HDMI converter.
 * iBoot starts the DCP and leaves it running under PMGR automatic power management (DISP_CPU
 * ps 0x1000030f: AUTO_ENABLE, target active, idle; ADT running = 1), its two display DARTs
 * locked with the firmware mappings in place, and only the invisible 640x1136 dummy framebuffer
 * in /vram. A client wakes it over RTKit (wake, HELLO); its CPU is never started a second time
 * and DISP_CPU is never written (pmgr.c refuses). Stock aura keeps all of that
 * (dt_set_display(), T8132 branch).
 *
 * chosen.aura,j773g-display selects one of three modes:
 *
 *   off (or absent): nothing here runs beyond reading the knob; the FDT is untouched.
 *
 *   handoff (layer B1): publish the main-DCP handoff the kernel needs to take the DCP iBoot left
 *   running (the kernel wakes it and says HELLO), without powering, starting, resetting or
 *   talking to the DCP, the PHY or the SMC, and without reading PMGR. After exact
 *   identity checks and a read-only walk of both locked DARTs it reserves the six firmware
 *   segments (__OS_LOG by physical address) and /vram, publishes the firmware identity and the
 *   inherited mappings, enables the main-DCP nodes, adds the dcp/disp0/disp0_piodma aliases and
 *   marks the DCP aurora,j773g-dcp-firmware-ready. All of it is one transaction on a scratch
 *   copy: any failure leaves the FDT byte-for-byte as in "off" and logs why.
 *
 *   start (layer B2, make J773G_DISPLAY_START=1 only): see display_j773g.c. Here only the
 *   handoff that follows a loader-side session is published: the memory the running DCP may
 *   use, with the DCP node left disabled (simpledrm-only: Linux keeps the loader's scanout and
 *   does not open a DCP session of its own).
 */

#include "../build/build_cfg.h"

#include "kboot_display_j773g.h"
#include "adt.h"
#include "dart.h"
#include "firmware.h"
#include "malloc.h"
#include "soc.h"
#include "string.h"
#include "utils.h"

#include "libfdt/libfdt.h"

#define J773G_BOARD_ID 0x2a
#define J773G_UUID     "81BAF476-679D-3CC0-8C6A-E6B6FA659D5E"
#define J773G_SEGNAMES "__TEXT;__DATA;__OS_LOG;unknown;unknown;unknown"
#define J773G_SEGMENTS 6
#define J773G_OSLOG    2       /* __OS_LOG: physical, not in any DART */
#define J773G_SCANOUT  4       /* dart-disp0 SID 0 (and dart-dcp) */
#define J773G_PIODMA   5       /* dart-disp0 SID 16 (and dart-dcp) */
#define J773G_OSLOG_SZ 0x24000 /* the size the kernel accepts for the __OS_LOG region */

/* The kernel's fixed J773g ADT profile: segment kinds (ADT flags) in order. */
static const u32 j773g_kinds[J773G_SEGMENTS] = {1, 0, 10, 2, 2, 2};

/* Measured on J773g 26.6.2, and what the kernel's DART admission requires. */
#define J773G_DCP_COPROC   0x30ae00000UL
#define J773G_DART_DCP     0x30a340000UL
#define J773G_DART_DCP_SZ  0x20000UL
#define J773G_DART_DISP    0x30a300000UL
#define J773G_DART_DISP_SZ 0xc000UL /* the whole frontend; a 0x4000 window wedged the DART */
#define J773G_DART_WALK_SZ 0xc000UL /* registers the read-only walk may read */
#define J773G_DCP_SID      23
#define J773G_SCANOUT_SID  0
#define J773G_PIODMA_SID   16

/* The kernel finds the J773g main DCP by this path, not by an alias. */
#define J773G_DCP_PATH "/soc/dcp@30ae00000"

#define J773G_READY  "aurora,j773g-dcp-firmware-ready"
#define J773G_RESULT "aurora,j773g-display-handoff"
#define J773G_KTEST  "aurora,j773g-hdmi-test"

#define jlog(...) printf("FDT: J773g display: " __VA_ARGS__)

enum j773g_display_mode j773g_display_mode_parse(const char *value)
{
    if (!value || !strcmp(value, "off"))
        return J773G_MODE_OFF;
    if (!strcmp(value, "handoff"))
        return J773G_MODE_HANDOFF;
    if (!strcmp(value, "start"))
        return J773G_MODE_START;
    return J773G_MODE_OFF;
}

/* Everything the read-only checks established, used by the publication. */
struct j773g_facts {
    struct adt_segment_ranges seg[J773G_SEGMENTS];
    u64 span[J773G_SEGMENTS]; /* 16K-rounded size */
    char uuid[37];
    u64 vram, vram_size;
    u64 dram_base, dram_size;
    /* FDT node paths, so that offsets can be found again after every edit */
    char display[64], piodma[64], dcp_dart[64], disp_dart[64], mbox[64], phy[64];
    bool has_phy;
};

struct j773g_dart_view {
    const struct j773g_facts *facts;
    u32 allowed;
    u64 bytes[J773G_SEGMENTS];
};

static bool j773g_overlap(u64 a, u64 a_size, u64 b, u64 b_size)
{
    return a_size && b_size && a < b + b_size && b < a + a_size;
}

/* A valid leaf of a locked stream must belong to one of its allowed segments, physically. */
static int j773g_dart_page(u64 iova, u64 physical, void *opaque)
{
    struct j773g_dart_view *view = opaque;
    for (u32 i = 0; i < J773G_SEGMENTS; i++) {
        const struct adt_segment_ranges *seg = &view->facts->seg[i];
        u64 start = seg->remap & ((1ULL << 36) - 1);
        if (!(view->allowed & BIT(i)) || iova < start || iova - start >= view->facts->span[i])
            continue;
        if (physical != seg->phys + (iova - start))
            return -1;
        view->bytes[i] += SZ_16K;
        return 0;
    }
    return -1; /* Never publish beside an inherited mapping we cannot describe. */
}

/* Read the stream's tables only: no register or table write, no TLB operation. */
static const char *j773g_check_stream(const struct j773g_facts *f, u64 base, u8 sid, u32 allowed)
{
    struct j773g_dart_view view = {.facts = f, .allowed = allowed};

    if (dart_walk_locked_t8110(base, J773G_DART_WALK_SZ, sid, f->dram_base, f->dram_size,
                               j773g_dart_page, &view))
        return "a locked DART stream is not readable or maps something unexpected";
    for (u32 i = 0; i < J773G_SEGMENTS; i++)
        if ((allowed & BIT(i)) && view.bytes[i] != f->span[i])
            return "a locked DART stream does not map a whole firmware segment";
    return NULL;
}

static bool j773g_adt_reg(const char *path, u64 *addr, u64 *size)
{
    int trace[8];
    int node = adt_path_offset_trace(adt, path, trace);
    if (node < 0)
        return false;
    return adt_get_reg(adt, trace, "reg", 0, addr, size) >= 0;
}

/* The ADT and firmware facts, all read-only. */
static const char *j773g_check_adt(struct j773g_facts *f)
{
    u32 len = 0;

    if (!adt_is_compatible(adt, 0, "J773gAP"))
        return "the ADT root is not J773gAP";
    const char *target = adt_getprop(adt, 0, "target-type", &len);
    if (!target || len != sizeof("J773g") || memcmp(target, "J773g", sizeof("J773g")))
        return "the ADT target-type is not J773g";
    if (board_id != J773G_BOARD_ID)
        return "the board-id is not 42";
    if (os_firmware.version != V26_6_2)
        return "the OS firmware is not 26.6.2";

    int chosen = adt_path_offset(adt, "/chosen");
    if (chosen < 0 || ADT_GETPROP(adt, chosen, "dram-base", &f->dram_base) != sizeof(u64) ||
        ADT_GETPROP(adt, chosen, "dram-size", &f->dram_size) != sizeof(u64) || !f->dram_size ||
        f->dram_base > UINT64_MAX - f->dram_size)
        return "the ADT has no DRAM range";

    int nub = adt_path_offset(adt, "/arm-io/dcp/iop-dcp-nub");
    if (nub < 0)
        return "the ADT has no /arm-io/dcp/iop-dcp-nub";
    const char *uuid = adt_getprop(adt, nub, "uuid", &len);
    if (!uuid || len != sizeof(f->uuid) || uuid[len - 1] || strcmp(uuid, J773G_UUID))
        return "the DCP firmware UUID is not 81BAF476-679D-3CC0-8C6A-E6B6FA659D5E";
    memcpy(f->uuid, uuid, sizeof(f->uuid));
    const char *names = adt_getprop(adt, nub, "segment-names", &len);
    if (!names || (len != sizeof(J773G_SEGNAMES) && len != sizeof(J773G_SEGNAMES) - 1) ||
        memcmp(names, J773G_SEGNAMES, sizeof(J773G_SEGNAMES) - 1))
        return "the DCP firmware segment names are not the six J773g segments";
    const struct adt_segment_ranges *seg = adt_getprop(adt, nub, "segment-ranges", &len);
    if (!seg || len != sizeof(f->seg))
        return "the DCP firmware does not list exactly six segments";
    memcpy(f->seg, seg, sizeof(f->seg));

    u64 dram_end = f->dram_base + f->dram_size;
    for (u32 i = 0; i < J773G_SEGMENTS; i++) {
        const struct adt_segment_ranges *s = &f->seg[i];
        u64 span = ALIGN_UP((u64)s->size, SZ_16K);
        f->span[i] = span;
        if (s->unk != j773g_kinds[i] || !s->size || (s->phys & (SZ_16K - 1)) ||
            s->phys < f->dram_base || s->phys >= dram_end || span > dram_end - s->phys)
            return "a DCP firmware segment has the wrong kind or lies outside DRAM";
        if (i == J773G_OSLOG) {
            if (s->size != J773G_OSLOG_SZ)
                return "the DCP __OS_LOG segment is not 0x24000 bytes";
        } else if ((s->remap & (SZ_16K - 1)) || (s->remap >> 36) != 0x10 ||
                   (s->remap & ((1ULL << 36) - 1)) > (1ULL << 36) - span) {
            return "a mapped DCP firmware segment is outside the DART window";
        }
        for (u32 j = 0; j < i; j++) {
            if (j773g_overlap(s->phys, span, f->seg[j].phys, f->span[j]))
                return "two DCP firmware segments overlap physically";
            if (i != J773G_OSLOG && j != J773G_OSLOG &&
                j773g_overlap(s->remap, span, f->seg[j].remap, f->span[j]))
                return "two DCP firmware segments overlap in the DART";
        }
    }

    u64 addr, size;
    if (!j773g_adt_reg("/arm-io/dart-dcp", &addr, &size) || addr != J773G_DART_DCP ||
        size != J773G_DART_DCP_SZ)
        return "the ADT dart-dcp register window is not 0x30a340000/0x20000";
    if (!j773g_adt_reg("/arm-io/dart-disp0", &addr, &size) || addr != J773G_DART_DISP ||
        size != J773G_DART_DISP_SZ)
        return "the ADT dart-disp0 register window is not 0x30a300000/0xc000";
    if (!j773g_adt_reg("/vram", &f->vram, &f->vram_size) || !f->vram_size ||
        ((f->vram | f->vram_size) & (SZ_16K - 1)) || f->vram < f->dram_base ||
        f->vram >= dram_end || f->vram_size > dram_end - f->vram)
        return "the ADT /vram is missing, unaligned or outside DRAM";
    for (u32 i = 0; i < J773G_SEGMENTS; i++)
        if (j773g_overlap(f->vram, f->vram_size, f->seg[i].phys, f->span[i]))
            return "/vram overlaps a DCP firmware segment";
    return NULL;
}

/* The identity and firmware checks alone, for the loader-side start (B2). Read-only. */
const char *j773g_display_check_identity(void)
{
    struct j773g_facts *f = calloc(1, sizeof(*f));
    const char *why = !f ? "no memory" : chip_id != T8132 ? "not a T8132" : j773g_check_adt(f);
    free(f);
    return why;
}

static bool j773g_fdt_reg(void *dt, int node, u64 addr, u64 size)
{
    int len = 0;
    const fdt64_t *reg = fdt_getprop(dt, node, "reg", &len);
    return reg && len >= 16 && fdt64_ld(&reg[0]) == addr && fdt64_ld(&reg[1]) == size;
}

/* The single <phandle cell> entry of an iommus/mboxes/phys property, or a negative error. */
static int j773g_fdt_ref(void *dt, int node, const char *prop, int cells, u32 *arg)
{
    int len = 0;
    const fdt32_t *val = fdt_getprop(dt, node, prop, &len);
    if (!val || len != (1 + cells) * 4)
        return -FDT_ERR_BADVALUE;
    if (cells)
        *arg = fdt32_ld(&val[1]);
    return fdt_node_offset_by_phandle(dt, fdt32_ld(&val[0]));
}

static bool j773g_fdt_path(void *dt, int node, char *path, size_t len)
{
    return node >= 0 && !fdt_get_path(dt, node, path, len);
}

/* An alias that is absent or already names the node it is about to name. */
static bool j773g_alias_free(void *dt, const char *name, const char *path)
{
    const char *have = fdt_get_alias(dt, name);
    return !have || !strcmp(have, path);
}

/* The FDT the kernel will read: the loader-owned J773g main-DCP topology. */
static const char *j773g_check_fdt(void *dt, struct j773g_facts *f)
{
    int dcp = fdt_path_offset(dt, J773G_DCP_PATH);
    if (dcp < 0 || fdt_node_check_compatible(dt, dcp, "apple,t8132-dcp") ||
        !j773g_fdt_reg(dt, dcp, J773G_DCP_COPROC, SZ_16K))
        return "the FDT has no apple,t8132-dcp at " J773G_DCP_PATH;
    if (fdt_getprop(dt, dcp, "memory-region", NULL) ||
        fdt_getprop(dt, dcp, "apple,firmware-uuid", NULL))
        return "the FDT already describes the DCP firmware (a static handoff)";

    u32 sid = ~0u;
    int dart = j773g_fdt_ref(dt, dcp, "iommus", 1, &sid);
    if (dart < 0 || sid != J773G_DCP_SID ||
        fdt_node_check_compatible(dt, dart, "apple,t8110-dart") ||
        !j773g_fdt_reg(dt, dart, J773G_DART_DCP, J773G_DART_DCP_SZ) ||
        !j773g_fdt_path(dt, dart, f->dcp_dart, sizeof(f->dcp_dart)))
        return "the DCP is not on dart-dcp (0x30a340000/0x20000) stream 23";

    int piodma = fdt_subnode_offset(dt, dcp, "piodma");
    int disp = piodma < 0 ? piodma : j773g_fdt_ref(dt, piodma, "iommus", 1, &sid);
    if (disp < 0 || sid != J773G_PIODMA_SID ||
        fdt_node_check_compatible(dt, disp, "apple,t8110-dart") ||
        !j773g_fdt_reg(dt, disp, J773G_DART_DISP, J773G_DART_DISP_SZ) ||
        !j773g_fdt_path(dt, piodma, f->piodma, sizeof(f->piodma)) ||
        !j773g_fdt_path(dt, disp, f->disp_dart, sizeof(f->disp_dart)))
        return "the DCP has no piodma child on dart-disp0 (0x30a300000/0xc000) stream 16";
    if (fdt_getprop(dt, piodma, "memory-region", NULL))
        return "the FDT already describes the PIODMA memory";

    /* The scanout frontend: the one apple,display-subsystem on dart-disp0 stream 0. */
    int display = -1;
    for (int node = fdt_node_offset_by_compatible(dt, -1, "apple,display-subsystem"); node >= 0;
         node = fdt_node_offset_by_compatible(dt, node, "apple,display-subsystem")) {
        sid = ~0u;
        if (j773g_fdt_ref(dt, node, "iommus", 1, &sid) != disp || sid != J773G_SCANOUT_SID)
            continue;
        if (display >= 0)
            return "the FDT has two display subsystems on dart-disp0 stream 0";
        display = node;
    }
    if (display < 0 || !j773g_fdt_path(dt, display, f->display, sizeof(f->display)))
        return "the FDT has no apple,display-subsystem on dart-disp0 stream 0";
    if (fdt_getprop(dt, display, "memory-region", NULL))
        return "the FDT already describes the scanout memory";

    int mbox = j773g_fdt_ref(dt, dcp, "mboxes", 0, NULL);
    if (mbox < 0 || fdt_node_check_compatible(dt, mbox, "apple,asc-mailbox-v4") ||
        !j773g_fdt_path(dt, mbox, f->mbox, sizeof(f->mbox)))
        return "the DCP has no apple,asc-mailbox-v4 mailbox";
    int phy = j773g_fdt_ref(dt, dcp, "phys", 0, NULL);
    f->has_phy = phy >= 0;
    if (f->has_phy && (fdt_node_check_compatible(dt, phy, "apple,t8122-dptx-phy") ||
                       !j773g_fdt_path(dt, phy, f->phy, sizeof(f->phy))))
        return "the DCP's phys is not an apple,t8122-dptx-phy";
    if (!f->has_phy || !fdt_getprop(dt, dcp, "hdmi-pwren-gpios", NULL) ||
        !fdt_getprop(dt, dcp, "dp2hdmi-pwren-gpios", NULL))
        return "the DCP has no DPTX PHY or HDMI power GPIOs for the kernel";

    if (fdt_path_offset(dt, "/reserved-memory") < 0)
        return "the FDT has no /reserved-memory";
    if (!j773g_alias_free(dt, "dcp", J773G_DCP_PATH) ||
        !j773g_alias_free(dt, "disp0", f->display) ||
        !j773g_alias_free(dt, "disp0_piodma", f->piodma))
        return "an existing dcp, disp0 or disp0_piodma alias names another node";
    return NULL;
}

/*
 * A range the DCP may write once a client wakes it must not be anything else: a /memreserve/
 * entry (the loader, its retained heap, payloads, CTXR windows), another /reserved-memory node,
 * this stage's loaded file part, or the kernel's exec window.
 */
static const char *j773g_range_taken(void *dt, const struct j773g_display_env *env, u64 start,
                                     u64 size)
{
    if (j773g_overlap(start, size, env->loader_base, env->loader_size))
        return "the loader's own image";
    if (j773g_overlap(start, size, env->exec_base, env->exec_size))
        return "the kernel's CTXR exec window";
    int count = fdt_num_mem_rsv(dt);
    if (count < 0)
        return "an unreadable /memreserve/ table";
    for (int i = 0; i < count; i++) {
        uint64_t base, len;
        if (fdt_get_mem_rsv(dt, i, &base, &len) || j773g_overlap(start, size, base, len))
            return "a /memreserve/ entry";
    }
    int resv = fdt_path_offset(dt, "/reserved-memory");
    int node;
    fdt_for_each_subnode(node, dt, resv)
    {
        int len = 0;
        const fdt64_t *reg = fdt_getprop(dt, node, "reg", &len);
        if (!reg)
            continue;
        if (len != 16)
            return "an unparsable /reserved-memory node";
        if (j773g_overlap(start, size, fdt64_ld(&reg[0]), fdt64_ld(&reg[1])))
            return "another /reserved-memory node";
    }
    return NULL;
}

static const char *j773g_check_memory(void *dt, const struct j773g_display_env *env,
                                      const struct j773g_facts *f)
{
    static const char *const names[J773G_SEGMENTS] = {"__TEXT",    "__DATA",    "__OS_LOG",
                                                      "segment 3", "segment 4", "segment 5"};
    for (u32 i = 0; i < J773G_SEGMENTS; i++) {
        const char *what = j773g_range_taken(dt, env, f->seg[i].phys, f->span[i]);
        if (what) {
            jlog("%s [0x%lx, 0x%lx) overlaps %s\n", names[i], (u64)f->seg[i].phys,
                 (u64)(f->seg[i].phys + f->span[i]), what);
            return "a DCP firmware segment overlaps memory in use";
        }
    }
    const char *what = j773g_range_taken(dt, env, f->vram, f->vram_size);
    if (what) {
        jlog("/vram [0x%lx, 0x%lx) overlaps %s\n", f->vram, f->vram + f->vram_size, what);
        return "/vram overlaps memory in use";
    }
    return NULL;
}

/*
 * Every check that does not touch the FDT the kernel gets: identity, ADT, FDT topology, DCP
 * both locked DARTs (table reads only) and memory placement. PMGR is not read.
 */
static const char *j773g_check(void *dt, const struct j773g_display_env *env, struct j773g_facts *f,
                               bool for_kernel)
{
    if (chip_id != T8132)
        return "not a T8132";
    if (fdt_node_check_compatible(dt, 0, "apple,j773g"))
        return "the FDT is not an apple,j773g tree";
    const char *why = j773g_check_adt(f);
    if (!why && for_kernel)
        why = j773g_check_fdt(dt, f);
    if (why)
        return why;
    /*
     * After a loader session (B2) the DARTs also hold the loader's /vram leaves: only the memory
     * placement matters then, and nothing is offered to the kernel.
     */
    if (!for_kernel)
        return j773g_check_memory(dt, env, f);
    if ((why =
             j773g_check_stream(f, J773G_DART_DCP, J773G_DCP_SID,
                                BIT(0) | BIT(1) | BIT(3) | BIT(J773G_SCANOUT) | BIT(J773G_PIODMA))))
        return why;
    if ((why = j773g_check_stream(f, J773G_DART_DISP, J773G_SCANOUT_SID, BIT(J773G_SCANOUT))))
        return why;
    if ((why = j773g_check_stream(f, J773G_DART_DISP, J773G_PIODMA_SID, BIT(J773G_PIODMA))))
        return why;
    return j773g_check_memory(dt, env, f);
}

/* Publication helpers. Each looks its nodes up again: every edit can move offsets. */

static int j773g_phandle(void *dt, const char *path, u32 *phandle)
{
    int node = fdt_path_offset(dt, path);
    if (node < 0)
        return -1;
    *phandle = fdt_get_phandle(dt, node);
    if (*phandle)
        return 0;
    if (fdt_generate_phandle(dt, phandle))
        return -1;
    node = fdt_path_offset(dt, path);
    return node < 0 ? -1 : fdt_setprop_u32(dt, node, "phandle", *phandle);
}

static int j773g_add_reserved(void *dt, const char *name, u64 addr, u64 size, u32 *phandle)
{
    int resv = fdt_path_offset(dt, "/reserved-memory");
    if (resv < 0 || fdt_subnode_offset(dt, resv, name) >= 0)
        return -1;
    int node = fdt_add_subnode(dt, resv, name);
    if (node < 0 || fdt_generate_phandle(dt, phandle))
        return -1;
    node = fdt_subnode_offset(dt, fdt_path_offset(dt, "/reserved-memory"), name);
    fdt64_t reg[2] = {cpu_to_fdt64(addr), cpu_to_fdt64(size)};
    if (node < 0 || fdt_setprop_u32(dt, node, "phandle", *phandle) ||
        fdt_setprop(dt, node, "reg", reg, sizeof(reg)) ||
        fdt_setprop_string(dt, node, "compatible", "apple,asc-mem") ||
        fdt_setprop_empty(dt, node, "no-map"))
        return -1;
    return 0;
}

/* Append one <owner iova(2) size(2)> tuple to a reserved-memory node's iommu-addresses. */
static int j773g_add_iommu_address(void *dt, u32 region, u32 owner, u64 iova, u64 size)
{
    int node = fdt_node_offset_by_phandle(dt, region);
    if (node < 0 || fdt_appendprop_u32(dt, node, "iommu-addresses", owner))
        return -1;
    node = fdt_node_offset_by_phandle(dt, region);
    if (node < 0 || fdt_appendprop_u64(dt, node, "iommu-addresses", iova))
        return -1;
    node = fdt_node_offset_by_phandle(dt, region);
    if (node < 0 || fdt_appendprop_u64(dt, node, "iommu-addresses", size))
        return -1;
    return 0;
}

static int j773g_set_cells(void *dt, const char *path, const char *prop, const u32 *cells, int n)
{
    fdt32_t val[J773G_SEGMENTS + 1];
    int node = fdt_path_offset(dt, path);
    if (node < 0 || n > (int)ARRAY_SIZE(val))
        return -1;
    for (int i = 0; i < n; i++)
        val[i] = cpu_to_fdt32(cells[i]);
    return fdt_setprop(dt, node, prop, val, n * sizeof(*val));
}

static int j773g_set_status(void *dt, const char *path, const char *status)
{
    int node = fdt_path_offset(dt, path);
    return node < 0 ? -1 : fdt_setprop_string(dt, node, "status", status);
}

/*
 * Reserve the six segments and /vram. With for_kernel the DART-mapped ones carry the
 * iommu-addresses the kernel's admission validates (dcp for every mapped segment, the display
 * subsystem for the scanout segment, piodma for the PIODMA segment), exactly as iBoot mapped them.
 */
static int j773g_reserve(void *dt, const struct j773g_facts *f, u32 *regions, u32 *vram,
                         bool for_kernel)
{
    u32 dcp = 0, display = 0, piodma = 0;
    if (for_kernel &&
        (j773g_phandle(dt, J773G_DCP_PATH, &dcp) || j773g_phandle(dt, f->display, &display) ||
         j773g_phandle(dt, f->piodma, &piodma)))
        return -1;
    for (u32 i = 0; i < J773G_SEGMENTS; i++) {
        char name[48];
        snprintf(name, sizeof(name), "asc-firmware@%lx", (u64)f->seg[i].phys);
        if (j773g_add_reserved(dt, name, f->seg[i].phys, f->span[i], &regions[i]))
            return -1;
        if (i == J773G_OSLOG) {
            int node = fdt_node_offset_by_phandle(dt, regions[i]);
            if (node < 0 || fdt_setprop_empty(dt, node, "apple,dcp-os-log"))
                return -1;
            continue;
        }
        if (!for_kernel)
            continue;
        if (j773g_add_iommu_address(dt, regions[i], dcp, f->seg[i].remap, f->span[i]))
            return -1;
        if (i == J773G_SCANOUT &&
            j773g_add_iommu_address(dt, regions[i], display, f->seg[i].remap, f->span[i]))
            return -1;
        if (i == J773G_PIODMA &&
            j773g_add_iommu_address(dt, regions[i], piodma, f->seg[i].remap, f->span[i]))
            return -1;
    }
    char name[48];
    snprintf(name, sizeof(name), "framebuffer@%lx", f->vram);
    return j773g_add_reserved(dt, name, f->vram, f->vram_size, vram);
}

static int j773g_set_chosen(void *dt, const char *result)
{
    int chosen = fdt_path_offset(dt, "/chosen");
    return chosen < 0 ? -1 : fdt_setprop_string(dt, chosen, J773G_RESULT, result);
}

/* B1: everything the kernel's J773g main-DCP takeover reads, on the scratch copy. */
static int j773g_publish_handoff(void *dt, const struct j773g_facts *f)
{
    u32 regions[J773G_SEGMENTS], vram;
    if (j773g_reserve(dt, f, regions, &vram, true))
        return -1;

    /* The DCP: its six regions in segment order, firmware identity, readiness. */
    int dcp = fdt_path_offset(dt, J773G_DCP_PATH);
    if (dcp < 0 || fdt_setprop_string(dt, dcp, "apple,firmware-uuid", f->uuid))
        return -1;
    dcp = fdt_path_offset(dt, J773G_DCP_PATH);
    if (dcp < 0 || firmware_set_fdt(dt, dcp, "apple,firmware-version", &os_firmware) < 0)
        return -1;
    /* 26.6.2 is its own compatibility level, as dt_set_dcp_firmware() would publish it. */
    dcp = fdt_path_offset(dt, J773G_DCP_PATH);
    if (dcp < 0 || firmware_set_fdt(dt, dcp, "apple,firmware-compat", &os_firmware) < 0)
        return -1;
    if (j773g_set_cells(dt, J773G_DCP_PATH, "memory-region", regions, J773G_SEGMENTS))
        return -1;

    /* PIODMA: its segment. The scanout frontend: its segment, then the retained framebuffer. */
    if (j773g_set_cells(dt, f->piodma, "memory-region", &regions[J773G_PIODMA], 1))
        return -1;
    u32 scanout[2] = {regions[J773G_SCANOUT], vram};
    static const char scanout_names[] = "asc-firmware\0framebuffer";
    int display = fdt_path_offset(dt, f->display);
    if (j773g_set_cells(dt, f->display, "memory-region", scanout, 2) ||
        (display = fdt_path_offset(dt, f->display)) < 0 ||
        fdt_setprop(dt, display, "memory-region-names", scanout_names, sizeof(scanout_names)))
        return -1;

    /* Enable the whole main-DCP group; each node was validated above. */
    const char *const enable[] = {f->dcp_dart, f->disp_dart, f->mbox,       f->phy,
                                  f->piodma,   f->display,   J773G_DCP_PATH};
    for (size_t i = 0; i < ARRAY_SIZE(enable); i++)
        if (j773g_set_status(dt, enable[i], "okay"))
            return -1;

    /* Late aliases: only now, after the loader's own display decisions are final. */
    int aliases = fdt_path_offset(dt, "/aliases");
    if (aliases < 0)
        aliases = fdt_add_subnode(dt, 0, "aliases");
    if (aliases < 0 || fdt_setprop_string(dt, aliases, "dcp", J773G_DCP_PATH))
        return -1;
    aliases = fdt_path_offset(dt, "/aliases");
    if (aliases < 0 || fdt_setprop_string(dt, aliases, "disp0", f->display))
        return -1;
    aliases = fdt_path_offset(dt, "/aliases");
    if (aliases < 0 || fdt_setprop_string(dt, aliases, "disp0_piodma", f->piodma))
        return -1;

    dcp = fdt_path_offset(dt, J773G_DCP_PATH);
    if (dcp < 0 || fdt_setprop_u32(dt, dcp, J773G_READY, 1))
        return -1;
    return j773g_set_chosen(dt, "iboot-running");
}

/*
 * B2: the loader ran its own DCP session. Reserve everything the DCP may use, leave the DCP and its
 * nodes as the tree has them (disabled: Linux opens no DCP session of its own) and say what Linux
 * gets.
 */
static int j773g_publish_started(void *dt, const struct j773g_facts *f, bool surface)
{
    u32 regions[J773G_SEGMENTS], vram;
    if (j773g_reserve(dt, f, regions, &vram, false))
        return -1;
    return j773g_set_chosen(dt, surface ? "simpledrm-only" : "cold-required");
}

struct j773g_txn {
    const struct j773g_facts *facts;
    bool started, surface;
};

static int j773g_apply(void *dt, const struct j773g_txn *txn)
{
    return txn->started ? j773g_publish_started(dt, txn->facts, txn->surface)
                        : j773g_publish_handoff(dt, txn->facts);
}

/* Commit a complete update or nothing, like dt_display_transaction(). */
static int j773g_transaction(void *dt, const struct j773g_txn *txn)
{
    size_t capacity = fdt_totalsize(dt);
    void *scratch = malloc(capacity);
    if (!scratch)
        return -1;
    memcpy(scratch, dt, capacity);
    int ret = j773g_apply(scratch, txn);
    if (!ret)
        memcpy(dt, scratch, capacity);
    free(scratch);
    return ret;
}

static int j773g_handoff(void *dt, const struct j773g_display_env *env)
{
    struct j773g_facts *f = calloc(1, sizeof(*f));
    if (!f) {
        jlog("handoff refused (no memory); keeping the boot framebuffer\n");
        return 0;
    }
    const char *why = j773g_check(dt, env, f, true);
    struct j773g_txn txn = {.facts = f};
    if (!why && j773g_transaction(dt, &txn))
        why = "the device tree update failed (no space?)";
    if (why) {
        jlog("handoff refused (%s); keeping the boot framebuffer\n", why);
        free(f);
        return 0;
    }
    int dcp = fdt_path_offset(dt, J773G_DCP_PATH);
    jlog("main DCP handoff published (firmware %s, 6 segments, /vram 0x%lx+0x%lx); the DCP is "
         "as iBoot left it, for the kernel to wake%s\n",
         f->uuid, f->vram, f->vram_size,
         dcp >= 0 && fdt_getprop(dt, dcp, J773G_KTEST, NULL)
             ? ""
             : " (no " J773G_KTEST " on the node: a test kernel will not bind it)");
    free(f);
    return 1;
}

#ifdef J773G_DISPLAY_START
static int j773g_after_start(void *dt, const struct j773g_display_env *env)
{
    if (!j773g_start_state.ran || !j773g_start_state.touched) {
        jlog("start changed no hardware (%s); keeping the boot framebuffer\n",
             j773g_start_state.stop ? j773g_start_state.stop : "not run");
        return 0;
    }
    struct j773g_facts *f = calloc(1, sizeof(*f));
    const char *why = f ? j773g_check(dt, env, f, false) : "no memory";
    struct j773g_txn txn = {.facts = f, .started = true, .surface = j773g_start_state.surface};
    if (!why && j773g_transaction(dt, &txn))
        why = "the device tree update failed (no space?)";
    if (why)
        /* No teardown: the DCP stays as the session left it. The next boot must be cold. */
        jlog("WARNING: a loader DCP session ran but its memory is not reserved (%s); cold boot "
             "required\n",
             why);
    else
        jlog("loader DCP session handed over as %s (stopped at %s)\n",
             j773g_start_state.surface ? "simpledrm-only" : "cold-required",
             j773g_start_state.stop ? j773g_start_state.stop : "none");
    free(f);
    /* The stock path must not run either: it would publish the DCP for the kernel to wake. */
    return 1;
}
#endif

int kboot_j773g_display(void *dt, const struct j773g_display_env *env)
{
    int chosen = fdt_path_offset(dt, "/chosen");
    int len = 0;
    const char *value = chosen < 0 ? NULL : fdt_getprop(dt, chosen, J773G_DISPLAY_KNOB, &len);

    if (!value)
        return 0; /* the default: not even a log line */
    if (len < 1 || value[len - 1] || strlen(value) + 1 != (size_t)len) {
        jlog("malformed chosen." J773G_DISPLAY_KNOB "; treated as off\n");
        return 0;
    }
    enum j773g_display_mode mode = j773g_display_mode_parse(value);
    if (mode == J773G_MODE_OFF) {
        if (strcmp(value, "off"))
            jlog("unknown chosen." J773G_DISPLAY_KNOB "=%s; treated as off\n", value);
        return 0;
    }
    if (mode == J773G_MODE_START) {
#ifdef J773G_DISPLAY_START
        return j773g_after_start(dt, env);
#else
        printf("FDT: j773g-display=start requested but this build cannot start the DCP\n");
        return 0;
#endif
    }
    return j773g_handoff(dt, env);
}
