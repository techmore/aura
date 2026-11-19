# SPDX-License-Identifier: MIT
"""Inherited-only chips never reach the display DART view, except a T8132 HDMI port left dark.

display_init() is compiled verbatim from src/display.c with host stubs for
everything it calls. On T8140 and a non-admitted T8132 it must return before the dummy
framebuffer path, so neither display_configure() nor any DCP or DART work runs,
even with the 640x1136 framebuffer that makes every other chip start the DCP. The
one T8132 exception is selected from the ADT (display_t8132_hdmi(), compiled
verbatim too): an external disp0, a DP-to-HDMI converter naming both rails, a
"dptx-phy,t8122" transmitter and the one firmware whose DCP port interface is
known. Each of those is refused in turn when it is missing, and a framebuffer
iBoot already lit is left alone.
display_dart_tables_inherited() must then answer false without looking at a
DART. A T6000 run is the positive control.
"""
from pathlib import Path
import os
import re
import subprocess

SOURCE = Path(os.environ.get("DART_DISPLAY_SOURCE", Path(__file__).resolve().parents[2]))


def function(text, signature):
    start = text.index(signature)
    end, depth = text.index("{", start) + 1, 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint32_t u32;
typedef unsigned long u64; /* as in aura: printed as long */
#define UNUSED(x) (void)(x)
#define FB_DEPTH_FLAG_RETINA 0x10000
%(chips)s
enum fw_version { V15_0B1 = 1, V26_6_2, V27_2, V_UNKNOWN };
#define FW_MAX V_UNKNOWN
/* DCP_RETAINED, verbatim from src/dcp/dptx_phy.h */
%(stage_gate)s
typedef struct {
    const char *dcp, *dcp_dart, *disp_dart, *dp2hdmi_gpio, *dptx_phy;
    u32 fw_version;
} display_config_t;

static u32 chip_id, board_id;
static bool is_mac = true;
static bool display_is_external, display_needs_power_cycle;
static struct { struct { u64 base, width, height, depth; } video; } cur_boot_args;
static struct { enum fw_version version; } os_firmware = {V15_0B1};
static int dummy_adt;
static void *adt = &dummy_adt;
static unsigned configures, t8152_inits, dart_queries, config_queries, adt_queries;
static unsigned admitted_configures;
static bool display_retained_admitted;
/* The ADT as far as the selection reads it. */
enum { NODE_OTHER = 1, NODE_DISP0, NODE_PHY, NODE_CONV };
static bool adt_external, adt_has_phy, adt_has_conv, adt_hdmi_rail, adt_dp2hdmi_rail;
static const char *adt_phy_compat;

static bool adt_is_compatible(void *tree, int node, const char *compat)
{
    (void)tree;
    assert(node >= 0); /* a missing node is never read */
    adt_queries++;
    return node == NODE_PHY && !strcmp(compat, adt_phy_compat);
}
static int adt_path_offset(void *tree, const char *path)
{
    (void)tree;
    adt_queries++;
    if (!strcmp(path, "/arm-io/disp0"))
        return NODE_DISP0;
    if (!strcmp(path, "/arm-io/dptx-phy"))
        return adt_has_phy ? NODE_PHY : -1;
    if (!strcmp(path, "/arm-io/dp2hdmi-gpio0"))
        return adt_has_conv ? NODE_CONV : -1;
    return NODE_OTHER;
}
static int adt_getprop_copy(void *tree, int node, const char *name, void *out, size_t len)
{
    (void)tree; (void)node; (void)name;
    memset(out, 0, len);
    return -1;
}
#define ADT_GETPROP(adt, node, name, val) adt_getprop_copy(adt, node, name, (val), sizeof(*(val)))
static const void *adt_getprop(void *tree, int node, const char *name, u32 *len)
{
    (void)tree; (void)len;
    assert(node >= 0);
    if ((node == NODE_DISP0 && !strcmp(name, "external") && adt_external) ||
        (node == NODE_CONV && !strcmp(name, "function-hdmi_pwr_en") && adt_hdmi_rail) ||
        (node == NODE_CONV && !strcmp(name, "function-dp2hdmi_pwr_en") && adt_dp2hdmi_rail))
        return "";
    return NULL;
}
static bool trial_running, wdt_armed, wdt_broken;
static unsigned wdt_events, armed_configures;
static bool aura_trial_enabled(void) { return trial_running; }
static int wdt_arm_display(void)
{
    wdt_events++;
    if (wdt_broken)
        return -1;
    wdt_armed = true;
    return 0;
}
static bool wdt_armed_now(void) { return wdt_armed; }
/* iBoot's record of the previous boot, as pmu_counter.c reads it. */
typedef uint8_t u8;
#define STAGE1_BOOT_STAGE_KERNEL_STARTED 0x30
static bool record_unreadable;
static u8 record_counts, record_stage;
static unsigned record_reads;
static int pmu_read_boot_record(u8 *counts, u8 *stage)
{
    record_reads++;
    if (record_unreadable)
        return -1;
    *counts = record_counts;
    *stage = record_stage;
    return 0;
}
%(stopped_early)s
#define wdt_armed() wdt_armed_now()
static void wdt_disable(void) { wdt_armed = false; wdt_events++; }
static int display_configure(const char *config)
{
    (void)config;
    configures++;
    admitted_configures += display_retained_admitted;
    armed_configures += wdt_armed;
    return 0;
}
static int display_init_t8152(void) { t8152_inits++; return 0; }
static void fb_clear_direct(void) {}
static bool firmware_sfw_in_range(enum fw_version lo, enum fw_version hi) { (void)lo; (void)hi; return false; }
static const display_config_t m1 = {"/arm-io/dcp", "/arm-io/dart-dcp", "/arm-io/dart-disp0", "", "", 0};
static const display_config_t display_config_t8132_hdmi = {
    "/arm-io/dcp", "/arm-io/dart-dcp", "/arm-io/dart-disp0", "/arm-io/dp2hdmi-gpio0",
    "/arm-io/dptx-phy", V26_6_2};
static const display_config_t *display_get_config(void) { config_queries++; return &m1; }
static bool dart_locked = true;
static bool dart_adt_locked(const char *path, int instance) { (void)path; (void)instance; dart_queries++; return dart_locked; }

%(selection)s

%(display_init)s

%(inherited)s

static void boot(u32 chip)
{
    chip_id = chip;
    cur_boot_args.video.base = 0x10000000;
    cur_boot_args.video.width = 640;
    cur_boot_args.video.height = 1136;
    cur_boot_args.video.depth = 30;
    configures = t8152_inits = dart_queries = config_queries = adt_queries = 0;
    admitted_configures = 0;
    wdt_events = armed_configures = 0;
    trial_running = wdt_armed = wdt_broken = record_unreadable = false;
    record_counts = record_stage = 0;
    record_reads = 0;
    /* A Mac with a panel on other firmware: nothing of the HDMI selection matches. */
    os_firmware.version = V15_0B1;
    adt_external = adt_has_phy = adt_has_conv = adt_hdmi_rail = adt_dp2hdmi_rail = false;
    adt_phy_compat = "dptx-phy,t8122";
}

/* A T8132 Mac whose ADT describes a fixed HDMI port this loader can light, left dark. */
static void boot_hdmi(void)
{
    boot(T8132);
    os_firmware.version = V26_6_2;
    adt_external = adt_has_phy = adt_has_conv = adt_hdmi_rail = adt_dp2hdmi_rail = true;
}

/* Not selected: as any other T8132, no modeset and no look at a DART. */
static void refused(const char *what)
{
    assert(display_init() == 0);
    if (configures || display_dart_tables_inherited() || dart_queries || config_queries) {
        printf("FAIL: selected although %%s\n", what);
        assert(0);
    }
}

int main(void)
{
#ifdef EXPECT_RETAINED
    _Static_assert(DCP_RETAINED == 1, "an ESP Stage 2 has the bring-up");
#endif
#if !DCP_RETAINED
    /* Built as a Stage 1: the same Mac is refused, whatever its ADT says. */
    boot_hdmi();
    refused("this is a Stage 1");
    puts("PASS inherited-only display chips");
    return 0;
#endif
    static const u32 inherited_only[] = {T8140, T8132};
    for (unsigned i = 0; i < 2; i++) {
        boot(inherited_only[i]);
        assert(display_init() == 0);
        assert(!configures && !t8152_inits && !wdt_events);
        assert(!display_dart_tables_inherited());
        assert(!dart_queries && !config_queries);
    }
    boot_hdmi();
    assert(display_init() == 0 && configures == 1);
    /* Admitted for that one call only: no other entry can start this DCP. */
    assert(admitted_configures == 1 && !display_retained_admitted);
    /* A plain boot bounds the HDMI bring-up with the watchdog, then disarms it. */
    assert(armed_configures == 1 && wdt_events == 2 && !wdt_armed);
    assert(display_dart_tables_inherited() && dart_queries == 1 && config_queries == 1);
    /* A trial runs under its own watchdog, which display_init() leaves armed. */
    boot_hdmi();
    trial_running = true;
    wdt_armed = true;
    assert(display_init() == 0 && configures == 1 && !wdt_events);
    /* The last boot ended before an OS, as one this bring-up hung would have: not tried again,
     * and the watchdog armed for it is disarmed. */
    boot_hdmi();
    record_counts = 1;
    record_stage = 0x10;
    assert(display_init() == 0 && !configures && record_reads == 1);
    assert(wdt_events == 2 && !wdt_armed);
    /* No record to read: not tried either. */
    boot_hdmi();
    record_unreadable = true;
    assert(display_init() == 0 && !configures && wdt_events == 2 && !wdt_armed);
    /* A failed boot that had reached the kernel, or panics alone, do not stop it. */
    boot_hdmi();
    record_counts = 1;
    record_stage = 0x30;
    assert(display_init() == 0 && configures == 1);
    boot_hdmi();
    record_counts = 0x20;
    assert(display_init() == 0 && configures == 1);
    /* No watchdog can be armed: the port stays as iBoot left it, in a trial too. */
    boot_hdmi();
    wdt_broken = true;
    assert(display_init() == 0 && !configures && wdt_events == 1 && !record_reads);
    boot_hdmi();
    trial_running = true;
    assert(display_init() == 0 && !configures && !wdt_events);

    /* Every property is needed: take each away in turn. */
    boot_hdmi();
    chip_id = T8140;
    refused("the chip is not T8132");
    boot_hdmi();
    adt_external = false;
    refused("disp0 is not external (a panel)");
    boot_hdmi();
    adt_has_phy = false;
    refused("the ADT has no DPTX PHY");
    boot_hdmi();
    adt_phy_compat = "dptx-phy,t8132"; /* as the 27.2 ADT names it */
    refused("the PHY is not a dptx-phy,t8122");
    boot_hdmi();
    adt_has_conv = false;
    refused("the ADT has no DP-to-HDMI converter");
    boot_hdmi();
    adt_hdmi_rail = false;
    refused("the converter names no HDMI rail");
    boot_hdmi();
    adt_dp2hdmi_rail = false;
    refused("the converter names no converter rail");
    boot_hdmi();
    os_firmware.version = V27_2;
    refused("the firmware is another one");

    /* iBoot lit the port: no modeset, though an earlier stage may have run one. */
    boot_hdmi();
    cur_boot_args.video.width = 1920;
    cur_boot_args.video.height = 1080;
    assert(display_init() == 0 && !configures);
    assert(display_dart_tables_inherited() && dart_queries == 1);
    boot_hdmi();
    cur_boot_args.video.height = 1080;
    assert(display_init() == 0 && !configures);
    boot_hdmi();
    cur_boot_args.video.width = 1920;
    assert(display_init() == 0 && !configures);
    /* Positive control: the same dummy framebuffer starts the DCP on T6000,
     * and the inherited-table question goes to its display DARTs. */
    boot(T6000);
    assert(display_init() == 0 && configures == 1);
    assert(display_dart_tables_inherited() && dart_queries == 1 && config_queries == 1);
    dart_locked = false;
    dart_queries = 0;
    assert(!display_dart_tables_inherited() && dart_queries == 2);
    puts("PASS inherited-only display chips");
    return 0;
}
'''


def test_t8140_t8132_return_before_the_dart_path(tmp_path):
    display = (SOURCE / "src/display.c").read_text()
    soc = (SOURCE / "src/soc.h").read_text()
    proxy = (SOURCE / "src/stage1_proxy.c").read_text()
    chips = "\n".join(re.findall(r"^#define (?:T8140|T8132|T8152|T6000|T6040|T6041|S5L8960X)\s+0x[0-9a-fA-F]+\s*$",
                                 soc, re.M))
    panel = function(display, "static bool display_internal_panel_absent(")
    init = function(display, "int display_init(void)")
    header = (SOURCE / "src/dcp/dptx_phy.h").read_text()
    stage_gate = re.search(r"^#if defined\(CHAINLOADING\).*?^#endif$", header, re.M | re.S).group(0)
    assert "DCP_RETAINED 0" in stage_gate and "DCP_RETAINED 1" in stage_gate
    selection = (function(display, "static bool display_t8132_hdmi(void)") + "\n" +
                 function(display, "static bool display_fb_is_placeholder(void)") + "\n" +
                 function(display, "static int display_init_retained_hdmi(void)"))
    inherited = function(display, "bool display_dart_tables_inherited(void)")
    # The early return the PR cites, ahead of the dummy framebuffer path.
    early = init.index("if (chip_id == T8140 || chip_id == T8132 ||")
    assert early < init.index("Dummy framebuffer found") and "return 0;" in init[early:init.index("}", early)]
    line = display[:display.index(init)].count("\n") + init[:early].count("\n") + 1
    print(f"display_init() returns early for T8140/T8132 at src/display.c:{line}")
    code = tmp_path / "inherited-only.c"
    code.write_text(HARNESS % {"chips": chips, "stage_gate": stage_gate,
                               "stopped_early": "static " + function(proxy, "bool stage1_previous_boot_stopped_early("),
                               "selection": selection, "display_init": panel + "\n" + init,
                               "inherited": inherited})
    binary = tmp_path / "inherited-only"
    compiled = subprocess.run([os.environ.get("CC", "cc"), "-std=gnu11", "-O1", "-Wall", "-Wextra",
                               "-Werror", "-Wno-unused-function", str(code), "-o",
                               str(binary)], capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr
    result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS inherited-only display chips" in result.stdout
    # The same harness as a Stage 1 build: the selection is compiled out.
    stage1 = tmp_path / "inherited-only-stage1"
    compiled = subprocess.run([os.environ.get("CC", "cc"), "-std=gnu11", "-O1", "-Wall", "-Wextra",
                               "-Werror", "-Wno-unused-function", "-Wno-unused-variable", "-DCHAINLOADING", str(code),
                               "-o", str(stage1)], capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr
    result = subprocess.run([str(stage1)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0 and "PASS" in result.stdout, result.stdout + result.stderr
    # And as an ESP Stage 2 (CHAINLOADING with ESP_STAGE2): the whole run again, selection in.
    esp = tmp_path / "inherited-only-esp"
    compiled = subprocess.run([os.environ.get("CC", "cc"), "-std=gnu11", "-O1", "-Wall", "-Wextra",
                               "-Werror", "-Wno-unused-function", "-Wno-unused-variable", "-DCHAINLOADING", "-DESP_STAGE2",
                               "-DEXPECT_RETAINED", str(code), "-o", str(esp)],
                              capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr
    result = subprocess.run([str(esp)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0 and "PASS" in result.stdout, result.stdout + result.stderr
