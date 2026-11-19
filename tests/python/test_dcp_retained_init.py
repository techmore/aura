# SPDX-License-Identifier: MIT
"""dcp_init() behind a DCP iBoot left running (no_reset), from src/dcp.c.

dcp_init() and dcp_hdmi_dptx_init() are compiled verbatim over stubs that log every power,
reset, SMC and RTKit request. A retained DCP is attached to without a power-on or a reset, its
HDMI rails are switched only when the converter node names both, its RTKit client gets the
DART's window bits and the oslog echo, and the dptx-port firmware gate takes its version from
the display configuration. A configuration without no_reset keeps the power-on and the reset.
"""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def function(source, signature):
    start = source.index(signature)
    return source[start:source.index("\n}", source.index("{", start)) + 2] + "\n"


HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;
#define T8152 0x8152
#define T8132 0x8132
enum { V_UNKNOWN, V13_5, V26_6_2, V27_2 };
#ifndef DCP_RETAINED
#define DCP_RETAINED 1
#endif
#define dcp_retained(x) (DCP_RETAINED && (x)->no_reset)
%(config)s
typedef struct {
    void *asc, *system_ep, *dptx_ep, *dpav_ep, *phy, *afk, *rtkit;
    void *dart_disp, *iovad_dcp, *dart_dcp;
    u32 die, dp2hdmi_pwr_gpio, hdmi_pwr_gpio;
    bool no_reset;
} dcp_dev_t;
typedef int smc_dev_t;
static u32 chip_id = T8132;
static struct { u32 version; const char *string; } os_firmware = {V26_6_2, "26.6.2"};
static int adt_dummy;
static void *adt = &adt_dummy;

/* Every request that touches hardware, in order. */
static char events[1024];
#define EV(...) snprintf(events + strlen(events), sizeof(events) - strlen(events), __VA_ARGS__)
static bool seen(const char *what) { return strstr(events, what); }

enum { NODE_PHY = 1, NODE_DART, NODE_MAPPER, NODE_CONV };
static bool has_hdmi_rail, has_dp2hdmi_rail;
static int fw_mappings;
static int adt_path_offset(void *a, const char *p)
{
    (void)a;
    if (strstr(p, "dptx-phy"))
        return NODE_PHY;
    if (strstr(p, "dp2hdmi"))
        return NODE_CONV;
    return NODE_DART;
}
static bool adt_is_compatible(void *a, int n, const char *c) { (void)a; (void)n; (void)c; return false; }
static int adt_first_child_offset(void *a, int n) { (void)a; (void)n; return NODE_MAPPER; }
static int adt_getprop_copy(void *a, int n, const char *name, void *out, size_t len)
{
    (void)a;
    memset(out, 0, len);
    if (n == NODE_MAPPER && !strcmp(name, "reg")) {
        *(u32 *)out = 23;
        return len;
    }
    if (n == NODE_CONV && len == 16 && ((!strcmp(name, "function-hdmi_pwr_en") && has_hdmi_rail) ||
                                        (!strcmp(name, "function-dp2hdmi_pwr_en") && has_dp2hdmi_rail))) {
        ((u32 *)out)[2] = name[9] == 'h' ? 0x0f : 0x1a; /* the gpio word */
        return len;
    }
    return -1;
}
#define ADT_GETPROP(adt, node, name, val) adt_getprop_copy(adt, node, name, (val), sizeof(*(val)))
static const void *adt_getprop(void *a, int n, const char *name, u32 *len)
{
    (void)a; (void)n; (void)name;
    *len = 0;
    return NULL;
}
static int pmgr_adt_power_enable(const char *path) { EV("power:%%s ", path); return 0; }
static int pmgr_power_on(int die, const char *name) { (void)die; EV("on:%%s ", name); return 0; }
static int pmgr_reset(int die, const char *name) { (void)die; EV("reset:%%s ", name); return 0; }
static int t8152_psd_power_on(u32 service) { (void)service; assert(0); return -1; }
static void mdelay(unsigned ms) { (void)ms; }
static int dart_storage[2];
static void *dart_init_adt_display(const char *path, int instance, int sid) { (void)instance; (void)sid; return &dart_storage[!!strstr(path, "disp0")]; }
static u64 dart_vm_base(void *d) { (void)d; return 0x10000000000; }
static u64 dart_dva_base(void *d) { (void)d; return 1ULL << 40; }
static void dart_setup_pt_region(void *d, const char *path, int sid, u64 base) { (void)d; (void)path; (void)sid; (void)base; }
static void dart_shutdown(void *d) { (void)d; EV("dart-shutdown "); }
static void *iovad_init(u64 base, u64 limit) { (void)base; (void)limit; return &adt_dummy; }
static void iovad_shutdown(void *i, void *d) { (void)i; (void)d; }
static int dcp_create_firmware_mappings(const display_config_t *cfg, dcp_dev_t *dcp) { (void)cfg; (void)dcp; return fw_mappings; }
static void *asc_init(const char *path) { (void)path; return &adt_dummy; }
static bool asc_is_v8(void *asc) { (void)asc; return false; }
static void *rtkit_init(const char *name, void *asc, void *dart, void *iovad, void *sart, bool sram) { (void)name; (void)asc; (void)dart; (void)iovad; (void)sart; (void)sram; return &adt_dummy; }
static void rtkit_set_dva_base(void *rtk, u64 base) { (void)rtk; EV("dva-base:%%llx ", (unsigned long long)base); }
static void rtkit_set_oslog_echo(void *rtk, bool echo) { (void)rtk; EV("oslog-echo:%%d ", echo); }
static bool rtkit_boot(void *rtk) { (void)rtk; EV("rtkit-boot "); return true; }
static bool rtkit_boot_timed(void *rtk, u64 usec) { (void)rtk; (void)usec; assert(0); return false; }
static bool rtkit_has_owned_buffers(void *rtk) { (void)rtk; return false; }
static bool rtkit_quiesce(void *rtk) { (void)rtk; EV("rtkit-quiesce "); return true; }
static void rtkit_free(void *rtk) { (void)rtk; }
static void *afk_epic_init(void *rtk) { (void)rtk; return &adt_dummy; }
static int afk_epic_shutdown(void *afk) { (void)afk; return 0; }
static smc_dev_t smc_storage;
static smc_dev_t *smc_init(void) { EV("smc-init "); return &smc_storage; }
static int smc_write_u32(smc_dev_t *smc, u32 key, u32 value) { (void)smc; EV("smc:%%x=%%x ", key, value); return 0; }
static int smc_write_u64(smc_dev_t *smc, u32 key, u64 value) { (void)smc; (void)key; (void)value; assert(0); return -1; }
static void smc_shutdown(smc_dev_t *smc) { (void)smc; }
static void *dptx_phy_init(const char *path, u32 index) { (void)path; (void)index; EV("phy-init "); return &adt_dummy; }
static void *dcp_dpav_init(dcp_dev_t *dcp) { (void)dcp; return &adt_dummy; }
static void *dcp_dptx_init(dcp_dev_t *dcp, u32 ports) { (void)dcp; (void)ports; return &adt_dummy; }
static int dcp_dpav_shutdown(void *ep) { (void)ep; return 0; }
static int dcp_dptx_shutdown(void *ep) { (void)ep; return 0; }

%(structs)s
static char dcp_pmgr_dev[16] = "DISP0_CPU0";
static u32 dcp_die;
static bool dcp_running;
%(bodies)s

static const display_config_t retained = {
    .dcp = "/arm-io/dcp", .dcp_dart = "/arm-io/dart-dcp", .disp_dart = "/arm-io/dart-disp0",
    .dp2hdmi_gpio = "/arm-io/dp2hdmi-gpio0", .dptx_phy = "/arm-io/dptx-phy", .pmgr_dev = "DISP_CPU",
    .dcp_alias = "dcp", .fw_version = V26_6_2, .num_dptxports = 2, .no_reset = true,
};

static dcp_dev_t *start(const display_config_t *cfg)
{
    events[0] = 0;
    dcp_running = false;
    return dcp_init(cfg);
}

int main(void)
{
    display_config_t other = retained;
    other.no_reset = false;
    fw_mappings = 1; /* mappings were created: a DCP this loader owns is then reset */
    has_hdmi_rail = has_dp2hdmi_rail = true;

#if DCP_RETAINED
    /* Attached to, not powered and not reset; the PHY's own gate is still raised. */
    dcp_dev_t *dcp = start(&retained);
    assert(dcp && dcp->no_reset);
    assert(!seen("power:/arm-io/dcp") && !seen("reset:") && seen("power:/arm-io/dptx-phy"));
    assert(seen("dva-base:10000000000 ") && seen("oslog-echo:1 "));
    assert(strstr(events, "oslog-echo:1 ") < strstr(events, "rtkit-boot "));
    /* The HDMI rail before the converter, both switched on. */
    assert(seen("smc:f=1000001 smc:1a=1000001 ") && seen("phy-init "));
    free(dcp);

    /* A converter node that names only one rail: nothing is written to the SMC, the DCP is
     * quiesced again and the bring-up fails. */
    has_hdmi_rail = false;
    assert(!start(&retained) && !seen("smc") && seen("rtkit-quiesce ") && !seen("phy-init "));
    has_hdmi_rail = true;
    has_dp2hdmi_rail = false;
    assert(!start(&retained) && !seen("smc") && !seen("phy-init "));
    has_dp2hdmi_rail = true;

    /* The dptx-port interface is known for the configuration's firmware only: refused before
     * any request on another, and 13.5 remains the default of a configuration naming none. */
    os_firmware.version = V27_2;
    assert(!start(&retained) && !events[0]);
    os_firmware.version = V13_5;
    assert(!start(&retained) && !events[0]);
    other.fw_version = 0;
    dcp = start(&other);
    assert(dcp);
    free(dcp);
    os_firmware.version = V26_6_2;
    assert(!start(&other) && !events[0]);
    other.fw_version = V26_6_2;
#endif

    /* Control (and a Stage 1 build, where no_reset is compiled out): powered on and reset, no
     * window bits, no oslog echo, the converter rails by the ordinary path. */
    dcp_dev_t *owned = start(DCP_RETAINED ? &other : &retained);
    assert(owned);
    assert(seen("power:/arm-io/dcp") && seen("reset:DISP_CPU "));
    assert(!seen("dva-base") && !seen("oslog-echo") && !seen("=1000001"));
    free(owned);
    puts("PASS dcp_init retained");
    return 0;
}
'''


def build_and_run(tmp_path, retained):
    dcp = (ROOT / "src/dcp.c").read_text()
    dcph = (ROOT / "src/dcp.h").read_text()
    config = dcph[dcph.index("typedef struct {"):dcph.index("} display_config_t;")] + "} display_config_t;\n"
    structs = "".join(re.search(r"^struct %s \{.*?^\};\n" % name, dcp, re.M | re.S).group(0)
                      for name in ("adt_function_smc_gpio", "adt_function_smc_pw8"))
    bodies = "".join(function(dcp, signature) for signature in (
        "static void dcp_mark_running(", "static int dcp_hdmi_dptx_init(", "dcp_dev_t *dcp_init("))
    code = tmp_path / f"dcp-init{retained}.c"
    code.write_text(HARNESS % {"config": config, "structs": structs, "bodies": bodies})
    binary = tmp_path / f"dcp-init{retained}"
    compiled = subprocess.run(["cc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                               "-Wno-unused-variable", f"-DDCP_RETAINED={retained}",
                               "-fsanitize=address,undefined", str(code), "-o", str(binary)],
                              capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr[-6000:]
    result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0 and "PASS dcp_init retained" in result.stdout, \
        result.stdout[-2000:] + result.stderr[-3000:]


def test_retained_dcp_is_attached_without_power_or_reset(tmp_path):
    build_and_run(tmp_path, 1)


def test_stage1_build_has_no_retained_path(tmp_path):
    build_and_run(tmp_path, 0)
