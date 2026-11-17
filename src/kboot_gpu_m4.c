/* SPDX-License-Identifier: GPL-2.0-only OR MIT */

#include "kboot.h"
#include "adt.h"
#include "exception.h"
#include "firmware.h"
#include "soc.h"
#include "utils.h"

#include "libfdt/libfdt.h"

/* Resource publication only. Never start, stop, reset or power the GPU here. */
struct gpu_m4_range {
    u64 base;
    u64 size;
};

static bool valid_range(struct gpu_m4_range r)
{
    return r.base && r.size && !((r.base | r.size) & (SZ_16K - 1)) && r.base + r.size > r.base &&
           r.base + r.size <= (1ULL << 42);
}

static int read_exact(int node, const char *name, void *out, u32 size)
{
    u32 len;
    const void *value = adt_getprop(adt, node, name, &len);
    if (!value || len != size)
        return -1;
    memcpy(out, value, size);
    return 0;
}

static int read_range(int node, const char *name, struct gpu_m4_range *r)
{
    char prop[64];
    snprintf(prop, sizeof(prop), "%s-base", name);
    if (read_exact(node, prop, &r->base, sizeof(r->base)))
        return -1;
    snprintf(prop, sizeof(prop), "%s-size", name);
    if (read_exact(node, prop, &r->size, sizeof(r->size)))
        return -1;
    return valid_range(*r) ? 0 : -1;
}

static int reserve_range(void *dt, struct gpu_m4_range range)
{
    int count = fdt_num_mem_rsv(dt);
    if (count < 0)
        return count;
    for (int i = 0; i < count; i++) {
        u64 base, size;
        if (fdt_get_mem_rsv(dt, i, &base, &size))
            return -1;
        if (base + size >= base && base <= range.base && base + size >= range.base + range.size)
            return 0;
    }
    return fdt_add_mem_rsv(dt, range.base, range.size);
}

/*
 * TEST ONLY, never sent: capture H4 of aurora/outbox/27.2/GPU-DCP-27.2-PLAN.md. Prints gfx-asc's
 * segment-ranges and the leaves of iBoot's firmware page tables (root at the start of the ADT
 * gpu-region) over the firmware's address space, as runs of contiguous pages with the same
 * attribute bits. Read-only: every load is in the gpu-region reservation and fault-guarded.
 */
#define G16_TEST_PA_MASK 0x3ffffffc000ULL

static bool g16_test_table(struct gpu_m4_range tables, u64 pa)
{
    return !(pa & (SZ_16K - 1)) && pa >= tables.base && pa + SZ_16K <= tables.base + tables.size;
}

static u64 g16_test_read(u64 pa, bool *ok)
{
    int before = exc_count;
    exc_guard = GUARD_MARK | GUARD_SILENT;
    u64 value = read64(pa);
    sysop("dsb sy");
    sysop("isb");
    exc_guard = GUARD_OFF;
    if (exc_count != before)
        *ok = false;
    return value;
}

static void g16_test_print_run(u64 start, u64 end, u64 pa, u64 attr)
{
    if (!pa && !attr)
        printf("TEST-ONLY G16 H4: UAT +0x%06lx..+0x%06lx unmapped\n", start, end);
    else
        printf("TEST-ONLY G16 H4: UAT +0x%06lx..+0x%06lx pa 0x%lx attr 0x%016lx\n", start, end, pa,
               attr);
}

static void g16_test_print_segments(const struct adt_segment_ranges *seg, u32 count,
                                    struct gpu_m4_range tables)
{
    u64 span = 0;
    for (u32 i = 0; i < count; i++) {
        printf("TEST-ONLY G16 H4: seg%u phys 0x%lx iova 0x%lx remap 0x%lx size 0x%x unk 0x%x\n", i,
               seg[i].phys, seg[i].iova, seg[i].remap, seg[i].size, seg[i].unk);
        if (seg[i].iova >= 0xfffffc0000000000 &&
            seg[i].iova - 0xfffffc0000000000 + seg[i].size > span)
            span = seg[i].iova - 0xfffffc0000000000 + seg[i].size;
    }
    /* One page past the image, bounded to the one leaf table that maps it. */
    span += SZ_16K;
    if (span > 2048 * SZ_16K)
        span = 2048 * SZ_16K;
    printf("TEST-ONLY G16 H4: gpu-region 0x%lx+0x%lx\n", tables.base, tables.size);
    bool ok = true;
    u64 l0 = g16_test_read(tables.base, &ok);
    u64 l1 = ok && g16_test_table(tables, l0 & G16_TEST_PA_MASK)
                 ? g16_test_read(l0 & G16_TEST_PA_MASK, &ok)
                 : 0;
    printf("TEST-ONLY G16 H4: root[0] 0x%016lx l2[0] 0x%016lx%s\n", l0, l1, ok ? "" : " (fault)");
    u64 leaf = l1 & G16_TEST_PA_MASK;
    if (!ok || (l0 & 3) != 3 || (l1 & 3) != 3 || !g16_test_table(tables, leaf)) {
        printf("TEST-ONLY G16 H4: no leaf table in gpu-region; not walking\n");
        return;
    }
    u64 start = 0, run_pa = 0, run_attr = 0;
    for (u64 off = 0; off < span; off += SZ_16K) {
        u64 pte = g16_test_read(leaf + (off / SZ_16K) * 8, &ok);
        if (!ok) {
            printf("TEST-ONLY G16 H4: fault reading the leaf at +0x%lx\n", off);
            return;
        }
        u64 pa = pte & G16_TEST_PA_MASK, attr = pte & ~G16_TEST_PA_MASK;
        bool same =
            off && attr == run_attr && (pte ? pa == run_pa + (off - start) : !run_pa && !run_attr);
        if (same)
            continue;
        if (off)
            g16_test_print_run(start, off, run_pa, run_attr);
        start = off;
        run_pa = pa;
        run_attr = attr;
    }
    g16_test_print_run(start, span, run_pa, run_attr);
}

static int publish_gpu_m4(void *dt)
{
    int gpu = fdt_path_offset(dt, "gpu");
    if (gpu < 0)
        return 0; /* A framebuffer-only DT needs no GPU handoff. */
    /*
     * The G16 handoff follows the 26.6.2 firmware, not the board; J713 as before. The J773g
     * also hands it off on any firmware its gate admits: the kernel identifies the loaded image
     * before it uses any of it.
     */
    if (chip_id != T8132 ||
        (fdt_node_check_compatible(dt, 0, "apple,j713") && os_firmware.version != V26_6_2 &&
         !kboot_is_j773g(dt)) ||
        fdt_node_check_compatible(dt, gpu, "apple,agx-t8132"))
        return -1;

    /* A failed refresh must never leave a stale completed-handoff marker. */
    int ret = fdt_delprop(dt, gpu, "apple,g16-handoff-version");
    if (ret && ret != -FDT_ERR_NOTFOUND)
        return ret;

    int sgx_path[8], asc_path[8];
    int sgx = adt_path_offset_trace(adt, "/arm-io/sgx", sgx_path);
    int asc = adt_path_offset_trace(adt, "/arm-io/gfx-asc", asc_path);
    if (sgx < 0 || asc < 0)
        return -1;
    u64 base, size;
    if (adt_get_reg(adt, sgx_path, "reg", 0, &base, &size) < 0 || base != 0x300000000 ||
        size != 0x4000000 || adt_get_reg(adt, asc_path, "reg", 0, &base, &size) < 0 ||
        base != 0x302600000 || size != 0x88000)
        return -1;

    u32 irq[4];
    if (read_exact(asc, "interrupts", irq, sizeof(irq)) || irq[0] != 994 || irq[1] != 993 ||
        irq[2] != 996 || irq[3] != 995)
        return -1;

    const char *properties[] = {
        "gpu-region",
        "gfx-shared-region",
        "gfx-handoff",
        "gfx-shared-l2-region",
    };
    const char *paths[] = {
        "/reserved-memory/uat-ttbs",    "/reserved-memory/uat-pagetables",
        "/reserved-memory/uat-handoff", "/reserved-memory/uat-shared-l2",
        "/reserved-memory/gpu-fw-text", "/reserved-memory/gpu-fw-data",
    };
    struct gpu_m4_range ranges[6];
    for (unsigned i = 0; i < 4; i++)
        if (read_range(sgx, properties[i], &ranges[i]))
            return -1;

    /*
     * iBoot loads the firmware before aura runs and names its segments in gfx-asc. The 25G83
     * image (macOS 26.6.2) has __TEXT and __DATA. The 26B123 image (macOS 27.2) adds an 0x8000
     * __DATA_SHARED_RO, which iBoot places right after __DATA, physically and in the firmware's
     * address space. Reserve __DATA and __DATA_SHARED_RO as one data region, as the M5 path
     * reserves its __DATA and shared-RO segments (g17_set_fw_data()); the kernel selects the
     * image layout from the published addresses. Only TEXT and DATA have firmware VAs and flags
     * the kernel checks: the shared-RO segment is covered by the data region.
     */
    static const char *const layouts[] = {"__TEXT;__DATA", "__TEXT;__DATA;__DATA_SHARED_RO"};
    u32 len, count = 0;
    const char *names = adt_getprop(adt, asc, "segment-names", &len);
    for (unsigned i = 0; names && i < sizeof(layouts) / sizeof(layouts[0]); i++)
        if (len == strlen(layouts[i]) + 1 && !memcmp(names, layouts[i], len))
            count = i + 2;
    if (!count)
        return -1;
    const struct adt_segment_ranges *segments = adt_getprop(adt, asc, "segment-ranges", &len);
    if (!segments || len != count * sizeof(*segments))
        return -1;
    g16_test_print_segments(segments, count, ranges[0]); /* TEST ONLY: capture H4 */

    fdt64_t vas[2];
    fdt32_t flags[2];
    for (unsigned i = 0; i < count; i++) {
        if (!valid_range((struct gpu_m4_range){segments[i].phys, segments[i].size}) ||
            segments[i].remap != segments[i].phys ||
            segments[i].iova + segments[i].size <= segments[i].iova)
            return -1;
        if (i && segments[i].iova != segments[i - 1].iova + segments[i - 1].size)
            return -1;
        if (i == 2) {
            if (segments[i].phys != segments[i - 1].phys + segments[i - 1].size)
                return -1;
            continue;
        }
        if (segments[i].unk != (i == 0 ? 1 : 0))
            return -1;
        fdt64_st(&vas[i], segments[i].iova);
        flags[i] = cpu_to_fdt32(segments[i].unk);
    }
    if (segments[0].iova != 0xfffffc0000000000)
        return -1;
    ranges[4] = (struct gpu_m4_range){segments[0].phys, segments[0].size};
    ranges[5] = (struct gpu_m4_range){segments[1].phys, segments[1].size};
    if (count == 3)
        ranges[5].size += segments[2].size;
    if (!valid_range(ranges[5]))
        return -1;

    /* Boot addresses are observed, never copied from one successful boot. */
    for (unsigned i = 0; i < 6; i++) {
        for (unsigned j = 0; j < i; j++)
            if (ranges[i].base < ranges[j].base + ranges[j].size &&
                ranges[j].base < ranges[i].base + ranges[i].size)
                return -1;
        if (fdt_path_offset(dt, paths[i]) < 0)
            return -1;
    }

    for (unsigned i = 0; i < 6; i++) {
        /* Protect the firmware through intermediate loaders as well as Linux. */
        if (reserve_range(dt, ranges[i]))
            return -1;
        int node = fdt_path_offset(dt, paths[i]);
        fdt64_t reg[2];
        fdt64_st(&reg[0], ranges[i].base);
        fdt64_st(&reg[1], ranges[i].size);
        if (fdt_setprop(dt, node, "reg", reg, sizeof(reg)) ||
            fdt_setprop_empty(dt, node, "no-map") || fdt_setprop_string(dt, node, "status", "okay"))
            return -1;
    }
    gpu = fdt_path_offset(dt, "gpu");
    if (fdt_setprop(dt, gpu, "apple,firmware-segment-vas", vas, sizeof(vas)) ||
        fdt_setprop(dt, gpu, "apple,firmware-segment-flags", flags, sizeof(flags)) ||
        fdt_setprop_u32(dt, gpu, "apple,g16-handoff-version", 1))
        return -1;
    if (count == 3)
        printf("FDT: GPU: M4 G16 __DATA_SHARED_RO (flag %u) reserved with __DATA: 0x%lx+0x%lx\n",
               segments[2].unk, ranges[5].base, ranges[5].size);
    printf("FDT: GPU: published M4 G16 memory resources without starting the GPU\n");
    return 0;
}

int dt_set_gpu_m4(void *dt)
{
    int gpu = fdt_path_offset(dt, "gpu");
    if (chip_id != T8132 || gpu < 0)
        return 0;

    if (!publish_gpu_m4(dt))
        return 0;

    /* Native graphics is optional. Never admit a partially refreshed epoch,
     * but keep the firmware framebuffer and unrelated boot devices usable.
     * Reservations already published are conservative and may remain. */
    const char *properties[] = {"apple,g16-handoff-version", "apple,firmware-segment-vas",
                                "apple,firmware-segment-flags"};
    gpu = fdt_path_offset(dt, "gpu");
    for (unsigned i = 0; i < sizeof(properties) / sizeof(properties[0]); i++) {
        int ret = fdt_delprop(dt, gpu, properties[i]);
        if (ret && ret != -FDT_ERR_NOTFOUND)
            return ret;
    }
    if (fdt_setprop_string(dt, gpu, "status", "disabled") && fdt_nop_node(dt, gpu))
        return -1; /* A corrupt DT cannot safely describe the failed device. */
    printf("FDT: GPU: unqualified M4 handoff, disabling native GPU\n");
    return 0;
}
