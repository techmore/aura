# SPDX-License-Identifier: MIT
"""T8132 with an external main display: aura never writes or resets the DCP's CPU domain.

Where the ADT's disp0 is "external" (the M4 Mac mini), iBoot leaves the main DCP running with
DISP_CPU under automatic power management (ps 0x1000030f: AUTO_ENABLE, target active, idle),
and a DISP_CPU write faults (Pavel Shago's J773g m1n1 bring-up). The production pmgr.c is
compiled against a synthetic power-state table: there every power transition and reset of
DISP_CPU is refused without a register write, whatever caller asks (pmgr_power_on, pmgr_reset,
pmgr_set_mode); the start-up cleanup still reads it and still enables its parent as before. A
T8132 Mac with a panel (disp0 not external), one without a disp0 and another chip with an
external disp0 are unchanged.
"""

from pathlib import Path
import subprocess

HARNESS = r"""
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#define BIT(n) (1UL << (n))
#define GENMASK(h, l) ((BIT((h) + 1) - 1) & ~(BIT(l) - 1))
#define FIELD_PREP(f, v) (((v) * ((f) & -(f))) & (f))
#define FIELD_GET(f, v) (((v) & (f)) / ((f) & -(f)))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define PACKED __attribute__((packed))
#define T8140 0x8140
#define T8132 0x8132
#define T8112 0x8112
#define ADT_GETPROP(a, n, p, v) (-1)

static int chip_id = T8132;
static bool has_disp0 = true, external = true;
static void *adt = (void *)1;
static const void *devices_prop;
static u32 devices_len;
static const u32 ps_regs[] = {0, 0, 0};
static u32 regs[8];
static int writes[8];

static int adt_path_offset(const void *a, const char *path)
{
    if (!strcmp(path, "/arm-io/disp0"))
        return has_disp0 ? 2 : -1;
    return 1;
}
static int adt_path_offset_trace(const void *a, const char *path, int *trace)
{
    trace[0] = 1;
    trace[1] = 0;
    return 1;
}
static bool adt_is_compatible(const void *a, int node, const char *compat) { return false; }
static const void *adt_getprop(const void *a, int node, const char *name, u32 *len)
{
    assert(node >= 0); /* a missing node is never read */
    if (node == 2)
        return !strcmp(name, "external") && external ? "" : NULL;
    if (!strcmp(name, "ps-regs")) {
        *len = sizeof(ps_regs);
        return ps_regs;
    }
    if (!strcmp(name, "devices")) {
        *len = devices_len;
        return devices_prop;
    }
    if (len)
        *len = 0;
    return NULL;
}
static int adt_get_reg(const void *a, const int *path, const char *prop, int idx, u64 *addr,
                       u64 *size)
{
    *addr = 0x2d0000000;
    return 0;
}
static int slot(uintptr_t addr)
{
    assert(addr >= 0x2d0000000 && (addr - 0x2d0000000) % 8 == 0 && (addr - 0x2d0000000) / 8 < 8);
    return (addr - 0x2d0000000) / 8;
}
static u32 read32(uintptr_t addr) { return regs[slot(addr)]; }
static void write32(uintptr_t addr, u32 v) { writes[slot(addr)]++; regs[slot(addr)] = v; }
static u32 mask32(uintptr_t addr, u32 clear, u32 set)
{
    int i = slot(addr);
    writes[i]++;
    regs[i] = (regs[i] & ~clear) | set;
    /* the domain follows its target at once */
    regs[i] = (regs[i] & ~0xf0u) | ((regs[i] & 0xf) << 4);
    return regs[i];
}
static u32 set32(uintptr_t addr, u32 set) { writes[slot(addr)]++; return regs[slot(addr)] |= set; }
static u32 clear32(uintptr_t addr, u32 clear) { writes[slot(addr)]++; return regs[slot(addr)] &= ~clear; }
static int poll32(uintptr_t addr, u32 mask, u32 target, u32 timeout)
{
    return (read32(addr) & mask) == target ? 0 : -1;
}
static void udelay(u32 us) {}
"""

TESTS = r"""
static struct pmgr_device table[2];

static void device(int i, u16 id, u16 parent, const char *name)
{
    memset(&table[i], 0, sizeof(table[i]));
    table[i].id2 = id;
    table[i].u16id.parent[0] = parent;
    table[i].addr_offset = (u8)i;
    strncpy((char *)table[i].name, name, sizeof(table[i].name));
}

static void boot(int chip, bool disp0, bool disp0_external)
{
    chip_id = chip;
    has_disp0 = disp0;
    external = disp0_external;
    device(0, 10, 0, "DISP_SYS");
    device(1, 11, 10, "DISP_CPU");
    devices_prop = table;
    devices_len = sizeof(table);
    memset(writes, 0, sizeof(writes));
    regs[0] = 0;          /* DISP_SYS: off */
    regs[1] = 0x1000030f; /* DISP_CPU as iBoot leaves it: auto, target active, idle */
    assert(pmgr_init() == 0);
}

int main(void)
{
    /* External main display: the cleanup reads DISP_CPU and enables its parent, as before, and
     * never writes it. */
    boot(T8132, true, true);
    assert(writes[0] == 1 && (regs[0] & 0xf) == 0xf);
    assert(writes[1] == 0 && regs[1] == 0x1000030f);
    assert(pmgr_power_on(0, "DISP_CPU") == -1);
    assert(pmgr_reset(0, "DISP_CPU") == -1);
    assert(pmgr_set_mode(0x2d0000008, 0x4) == -1);
    /* A request for the state it already reads is a no-op, as for T8140, and writes nothing. */
    assert(pmgr_set_mode(0x2d0000008, 0x0) == 0);
    assert(writes[1] == 0 && regs[1] == 0x1000030f);
    /* Its parent is an ordinary domain. */
    assert(pmgr_power_on(0, "DISP_SYS") == 0);

    /* The same SoC with a panel: DISP_CPU is not protected. */
    boot(T8132, true, false);
    assert(pmgr_power_on(0, "DISP_CPU") == 0 && writes[1] == 1);

    /* No disp0 in the ADT at all: not protected either. */
    boot(T8132, false, true);
    assert(pmgr_power_on(0, "DISP_CPU") == 0 && writes[1] == 1);

    /* Another chip with an external main display keeps the ordinary PMGR path. */
    boot(T8112, true, true);
    assert(pmgr_power_on(0, "DISP_CPU") == 0 && writes[1] == 1);
    assert(pmgr_reset(0, "DISP_CPU") == 0);
    puts("PASS T8132 external DISP_CPU guard");
    return 0;
}
"""


def test_t8132_external_disp_cpu_is_never_written(tmp_path):
    repo = Path(__file__).resolve().parents[2]
    header = (repo / "src/pmgr.h").read_text()
    source = (repo / "src/pmgr.c").read_text()
    strip = lambda text: "\n".join(line for line in text.splitlines()
                                   if not line.startswith("#include"))
    program = HARNESS + strip(header) + strip(source) + TESTS
    binary = tmp_path / "t8132-external-dcp-power-guard"
    subprocess.run(
        ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
         "-Wno-unused-function", "-x", "c", "-", "-o", str(binary)],
        input=program, text=True, check=True,
    )
    result = subprocess.run([str(binary)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS T8132 external DISP_CPU guard" in result.stdout
    assert "refusing protected display CPU power transition" in result.stdout
