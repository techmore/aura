/* SPDX-License-Identifier: MIT */

#include "adt.h"
#include "malloc.h"
#include "sart.h"
#include "string.h"
#include "utils.h"

#define APPLE_SART_MAX_ENTRIES 16

struct sart_dev {
    uintptr_t base;
    u32 version;
    u32 protected_entries;
    /* This is probably a bitfield but the exact meaning of each bit is unknown. */
    u32 flags_allow;

    /* Every entry as sart_save() read it, for sart_replay() after a power cycle. */
    bool saved_valid;
    u8 saved_flags[APPLE_SART_MAX_ENTRIES];
    u64 saved_paddr[APPLE_SART_MAX_ENTRIES];
    size_t saved_size[APPLE_SART_MAX_ENTRIES];

    void (*get_entry)(sart_dev_t *sart, int index, u8 *flags, void **paddr, size_t *size);
    bool (*set_entry)(sart_dev_t *sart, int index, u8 flags, void *paddr, size_t size);
};

/* SARTv0 registers */
#define APPLE_SART0_CONFIG(idx)       (0x00 + 4 * (idx))
#define APPLE_SART0_CONFIG_FLAGS      GENMASK(28, 24)
#define APPLE_SART0_CONFIG_SIZE       GENMASK(18, 0)
#define APPLE_SART0_CONFIG_SIZE_SHIFT 12
#define APPLE_SART0_CONFIG_SIZE_MAX   GENMASK(18, 0)

#define APPLE_SART0_PADDR(idx)  (0x40 + 4 * (idx))
#define APPLE_SART0_PADDR_SHIFT 12

#define APPLE_SART0_FLAGS_ALLOW 0xf

/* SARTv2 registers */
#define APPLE_SART2_CONFIG(idx)       (0x00 + 4 * (idx))
#define APPLE_SART2_CONFIG_FLAGS      GENMASK(31, 24)
#define APPLE_SART2_CONFIG_SIZE       GENMASK(23, 0)
#define APPLE_SART2_CONFIG_SIZE_SHIFT 12
#define APPLE_SART2_CONFIG_SIZE_MAX   GENMASK(23, 0)

#define APPLE_SART2_PADDR(idx)  (0x40 + 4 * (idx))
#define APPLE_SART2_PADDR_SHIFT 12

#define APPLE_SART2_FLAGS_ALLOW 0xff

/* SARTv3 registers */
#define APPLE_SART3_CONFIG(idx) (0x00 + 4 * (idx))

#define APPLE_SART3_PADDR(idx)  (0x40 + 4 * (idx))
#define APPLE_SART3_PADDR_SHIFT 12

#define APPLE_SART3_SIZE(idx)  (0x80 + 4 * (idx))
#define APPLE_SART3_SIZE_SHIFT 12
#define APPLE_SART3_SIZE_MAX   GENMASK(29, 0)

#define APPLE_SART3_FLAGS_ALLOW 0xff

/* SARTv4 registers */
#define APPLE_SART4_CONFIG(idx) (0x00 + 4 * (idx))

#define APPLE_SART4_PADDR(idx)  (0x60 + 4 * (idx))
#define APPLE_SART4_PADDR_SHIFT 12

#define APPLE_SART4_SIZE(idx)  (0xc0 + 4 * (idx))
#define APPLE_SART4_SIZE_SHIFT 12
#define APPLE_SART4_SIZE_MAX   GENMASK(29, 0)

#define APPLE_SART4_FLAGS_ALLOW 0xff

static void sart0_get_entry(sart_dev_t *sart, int index, u8 *flags, void **paddr, size_t *size)
{
    u32 cfg = read32(sart->base + APPLE_SART0_CONFIG(index));
    *flags = FIELD_GET(APPLE_SART0_CONFIG_FLAGS, cfg);
    *size = (size_t)FIELD_GET(APPLE_SART0_CONFIG_SIZE, cfg) << APPLE_SART0_CONFIG_SIZE_SHIFT;
    *paddr =
        (void *)((u64)read32(sart->base + APPLE_SART0_PADDR(index)) << APPLE_SART0_PADDR_SHIFT);
}

static bool sart0_set_entry(sart_dev_t *sart, int index, u8 flags, void *paddr_, size_t size)
{
    u32 cfg;
    u64 paddr = (u64)paddr_;

    if (size & ((1 << APPLE_SART0_CONFIG_SIZE_SHIFT) - 1))
        return false;
    if (paddr & ((1 << APPLE_SART0_PADDR_SHIFT) - 1))
        return false;

    size >>= APPLE_SART0_CONFIG_SIZE_SHIFT;
    paddr >>= APPLE_SART0_PADDR_SHIFT;

    if (size > APPLE_SART0_CONFIG_SIZE_MAX)
        return false;

    cfg = FIELD_PREP(APPLE_SART0_CONFIG_FLAGS, flags);
    cfg |= FIELD_PREP(APPLE_SART0_CONFIG_SIZE, size);

    write32(sart->base + APPLE_SART0_PADDR(index), paddr);
    write32(sart->base + APPLE_SART0_CONFIG(index), cfg);

    return true;
}

static void sart2_get_entry(sart_dev_t *sart, int index, u8 *flags, void **paddr, size_t *size)
{
    u32 cfg = read32(sart->base + APPLE_SART2_CONFIG(index));
    *flags = FIELD_GET(APPLE_SART2_CONFIG_FLAGS, cfg);
    *size = (size_t)FIELD_GET(APPLE_SART2_CONFIG_SIZE, cfg) << APPLE_SART2_CONFIG_SIZE_SHIFT;
    *paddr =
        (void *)((u64)read32(sart->base + APPLE_SART2_PADDR(index)) << APPLE_SART2_PADDR_SHIFT);
}

static bool sart2_set_entry(sart_dev_t *sart, int index, u8 flags, void *paddr_, size_t size)
{
    u32 cfg;
    u64 paddr = (u64)paddr_;

    if (size & ((1 << APPLE_SART2_CONFIG_SIZE_SHIFT) - 1))
        return false;
    if (paddr & ((1 << APPLE_SART2_PADDR_SHIFT) - 1))
        return false;

    size >>= APPLE_SART2_CONFIG_SIZE_SHIFT;
    paddr >>= APPLE_SART2_PADDR_SHIFT;

    if (size > APPLE_SART2_CONFIG_SIZE_MAX)
        return false;

    cfg = FIELD_PREP(APPLE_SART2_CONFIG_FLAGS, flags);
    cfg |= FIELD_PREP(APPLE_SART2_CONFIG_SIZE, size);

    write32(sart->base + APPLE_SART2_PADDR(index), paddr);
    write32(sart->base + APPLE_SART2_CONFIG(index), cfg);

    return true;
}

static void sart3_get_entry(sart_dev_t *sart, int index, u8 *flags, void **paddr, size_t *size)
{
    *flags = read32(sart->base + APPLE_SART3_CONFIG(index));
    *size = (size_t)read32(sart->base + APPLE_SART3_SIZE(index)) << APPLE_SART3_SIZE_SHIFT;
    *paddr =
        (void *)((u64)read32(sart->base + APPLE_SART3_PADDR(index)) << APPLE_SART3_PADDR_SHIFT);
}

static bool sart3_set_entry(sart_dev_t *sart, int index, u8 flags, void *paddr_, size_t size)
{
    u64 paddr = (u64)paddr_;
    if (size & ((1 << APPLE_SART3_SIZE_SHIFT) - 1))
        return false;
    if (paddr & ((1 << APPLE_SART3_PADDR_SHIFT) - 1))
        return false;

    paddr >>= APPLE_SART3_PADDR_SHIFT;
    size >>= APPLE_SART3_SIZE_SHIFT;

    if (size > APPLE_SART3_SIZE_MAX)
        return false;

    write32(sart->base + APPLE_SART3_PADDR(index), paddr);
    write32(sart->base + APPLE_SART3_SIZE(index), size);
    write32(sart->base + APPLE_SART3_CONFIG(index), flags);

    return true;
}

static void sart4_get_entry(sart_dev_t *sart, int index, u8 *flags, void **paddr, size_t *size)
{
    *flags = read32(sart->base + APPLE_SART4_CONFIG(index));
    *size = (size_t)read32(sart->base + APPLE_SART4_SIZE(index)) << APPLE_SART4_SIZE_SHIFT;
    *paddr =
        (void *)((u64)read32(sart->base + APPLE_SART4_PADDR(index)) << APPLE_SART4_PADDR_SHIFT);
}

static bool sart4_set_entry(sart_dev_t *sart, int index, u8 flags, void *paddr_, size_t size)
{
    u64 paddr = (u64)paddr_;
    if (size & ((1 << APPLE_SART4_SIZE_SHIFT) - 1))
        return false;
    if (paddr & ((1 << APPLE_SART4_PADDR_SHIFT) - 1))
        return false;

    paddr >>= APPLE_SART4_PADDR_SHIFT;
    size >>= APPLE_SART4_SIZE_SHIFT;

    if (size > APPLE_SART4_SIZE_MAX)
        return false;

    write32(sart->base + APPLE_SART4_PADDR(index), paddr);
    write32(sart->base + APPLE_SART4_SIZE(index), size);
    write32(sart->base + APPLE_SART4_CONFIG(index), flags);

    return true;
}

sart_dev_t *sart_init(const char *adt_path)
{
    int sart_path[8];
    int node = adt_path_offset_trace(adt, adt_path, sart_path);
    if (node < 0) {
        printf("sart: Error getting SART node %s\n", adt_path);
        return NULL;
    }

    u64 base;
    if (adt_get_reg(adt, sart_path, "reg", 0, &base, NULL) < 0) {
        printf("sart: Error getting SART %s base address.\n", adt_path);
        return NULL;
    }

    const u32 *sart_version = adt_getprop(adt, node, "sart-version", NULL);
    const u32 sart_version_zero = 0;
    if (!sart_version) {
        if (adt_is_compatible(adt, node, "sart,t8015")) {
            sart_version = &sart_version_zero;
        } else {
            printf("sart: SART %s has no sart-version property\n", adt_path);
            return NULL;
        }
    }

    sart_dev_t *sart = calloc(1, sizeof(*sart));
    if (!sart)
        return NULL;

    sart->base = base;
    sart->version = *sart_version;

    switch (*sart_version) {
        case 0:
            sart->get_entry = sart0_get_entry;
            sart->set_entry = sart0_set_entry;
            sart->flags_allow = APPLE_SART0_FLAGS_ALLOW;
            break;
        case 2:
            sart->get_entry = sart2_get_entry;
            sart->set_entry = sart2_set_entry;
            sart->flags_allow = APPLE_SART2_FLAGS_ALLOW;
            break;
        case 3:
            sart->get_entry = sart3_get_entry;
            sart->set_entry = sart3_set_entry;
            sart->flags_allow = APPLE_SART3_FLAGS_ALLOW;
            break;
        case 4:
            sart->get_entry = sart4_get_entry;
            sart->set_entry = sart4_set_entry;
            sart->flags_allow = APPLE_SART4_FLAGS_ALLOW;
            break;
        default:
            printf("sart: SART %s has unknown version %d\n", adt_path, *sart_version);
            free(sart);
            return NULL;
    }

    printf("sart: SARTv%d %s at 0x%lx\n", *sart_version, adt_path, base);

    sart->protected_entries = 0;
    for (unsigned int i = 0; i < APPLE_SART_MAX_ENTRIES; ++i) {
        void *paddr;
        u8 flags;
        size_t sz;

        sart->get_entry(sart, i, &flags, &paddr, &sz);
        if (flags)
            sart->protected_entries |= 1 << i;
    }

    return sart;
}

void sart_free(sart_dev_t *sart)
{
    for (unsigned int i = 0; i < APPLE_SART_MAX_ENTRIES; ++i) {
        if (sart->protected_entries & (1 << i))
            continue;
        sart->set_entry(sart, i, 0, NULL, 0);
    }

    free(sart);
}

bool sart_get_allowed(sart_dev_t *sart, int index, u64 *paddr, size_t *sz)
{
    void *e_paddr;
    u8 e_flags;

    if (index < 0 || index >= APPLE_SART_MAX_ENTRIES)
        return false;
    sart->get_entry(sart, index, &e_flags, &e_paddr, sz);
    *paddr = (u64)e_paddr;
    return e_flags && *sz;
}

void sart_release(sart_dev_t *sart)
{
    /* Unlike sart_free(), leave every entry as it is: a live owner still uses them. */
    free(sart);
}

bool sart_add_allowed_region(sart_dev_t *sart, void *paddr, size_t sz)
{
    for (unsigned int i = 0; i < APPLE_SART_MAX_ENTRIES; ++i) {
        void *e_paddr;
        u8 e_flags;
        size_t e_sz;

        if (sart->protected_entries & (1 << i))
            continue;

        sart->get_entry(sart, i, &e_flags, &e_paddr, &e_sz);
        if (e_flags)
            continue;

        return sart->set_entry(sart, i, sart->flags_allow, paddr, sz);
    }

    printf("sart: no more free entries\n");
    return false;
}

bool sart_remove_allowed_region(sart_dev_t *sart, void *paddr, size_t sz)
{
    for (unsigned int i = 0; i < APPLE_SART_MAX_ENTRIES; ++i) {
        void *e_paddr;
        u8 e_flags;
        size_t e_sz;

        if (sart->protected_entries & (1 << i))
            continue;

        sart->get_entry(sart, i, &e_flags, &e_paddr, &e_sz);
        if (!e_flags)
            continue;
        if (e_paddr != paddr)
            continue;
        if (e_sz != sz)
            continue;

        return sart->set_entry(sart, i, 0, NULL, 0);
    }

    printf("sart: could not find entry to be removed\n");
    return false;
}

COLD_TEXT void sart_save(sart_dev_t *sart)
{
    for (unsigned int i = 0; i < APPLE_SART_MAX_ENTRIES; i++) {
        void *paddr;

        sart->get_entry(sart, i, &sart->saved_flags[i], &paddr, &sart->saved_size[i]);
        sart->saved_paddr[i] = (u64)paddr;
    }
    sart->saved_valid = true;
}

/*
 * Write back what sart_save() found in use, config first, then address, then size, as macOS does
 * before a run bit. Returns how many entries didn't read back, or -1 with nothing saved.
 */
COLD_TEXT int sart_replay(sart_dev_t *sart, int *written)
{
    int bad = 0, n = 0;

    *written = 0;
    if (!sart->saved_valid)
        return -1;

    for (unsigned int i = 0; i < APPLE_SART_MAX_ENTRIES; i++) {
        u8 flags = sart->saved_flags[i];
        u64 paddr = sart->saved_paddr[i];
        size_t size = sart->saved_size[i];
        void *now_paddr;
        size_t now_size;
        u8 now_flags;

        if (!flags && !size)
            continue;
        if (sart->version == 3) {
            write32(sart->base + APPLE_SART3_CONFIG(i), flags);
            write32(sart->base + APPLE_SART3_PADDR(i), paddr >> APPLE_SART3_PADDR_SHIFT);
            write32(sart->base + APPLE_SART3_SIZE(i), size >> APPLE_SART3_SIZE_SHIFT);
        } else if (sart->version == 4) {
            write32(sart->base + APPLE_SART4_CONFIG(i), flags);
            write32(sart->base + APPLE_SART4_PADDR(i), paddr >> APPLE_SART4_PADDR_SHIFT);
            write32(sart->base + APPLE_SART4_SIZE(i), size >> APPLE_SART4_SIZE_SHIFT);
        } else if (!sart->set_entry(sart, i, flags, (void *)paddr, size)) {
            bad++;
            continue;
        }
        n++;
        sart->get_entry(sart, i, &now_flags, &now_paddr, &now_size);
        if (now_flags == flags && (u64)now_paddr == paddr && now_size == size)
            continue;
        bad++;
        printf("sart: replay: entry %u reads 0x%x 0x%lx+0x%zx, wrote 0x%x 0x%lx+0x%zx\n", i,
               now_flags, (u64)now_paddr, now_size, flags, paddr, size);
    }
    *written = n;
    return bad;
}
