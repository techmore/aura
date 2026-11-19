# SPDX-License-Identifier: MIT
"""The HDMI bring-up behind a DCP iBoot left running, as display.c runs it.

display_get_config(), display_start_dcp(), display_configure() and display_shutdown() are
compiled verbatim, on the host models of test_t8152_display_fb.py, for a T8132 Mac whose ADT
describes a fixed HDMI port: the configuration the selection returns, the refusal of every entry
but display_init()'s, the second HPD assertion, the longer wait for a sink, the window bits on
the DVA given to the DCP, the single modeset and the quiesce at shutdown.
"""
import os
import subprocess

import pytest

import test_t8152_display_fb as fb

SOURCE = fb.SOURCE


def swap(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)


def unit():
    ibh = (SOURCE / "src/dcp_iboot.h").read_text()
    dcph = (SOURCE / "src/dcp.h").read_text()
    disp = (SOURCE / "src/display.c").read_text()
    types = ibh[ibh.index("enum DCPEOTF {"):ibh.index("dcp_iboot_if_t *dcp_ib_init")]
    config = dcph[dcph.index("typedef struct {"):dcph.index("} display_config_t;")] + "} display_config_t;\n"
    state = disp[disp.index("static dcp_dev_t *dcp;"):disp.index("#define USE_DCPEXT 1")]
    leaves = fb.LEAVES
    leaves = swap(leaves, "static const display_config_t *display_get_config(void) { has_dcp = true; return t8152_config; }",
                  "static const display_config_t *display_get_config(void);\nstatic const display_config_t *started_with;")
    leaves = swap(leaves, '''    (void)cfg;
    assert(dcps_started < 4);
    dcp_dev_t *d = &dcps[dcps_started];
    d->dart_disp = &darts[2 * dcps_started];
    d->dart_dcp = &darts[2 * dcps_started + 1];
    /* iBoot maps the boot framebuffer for the first DCP only. */
    if (!dcps_started) {
        add_map(d->dart_disp, BOOT_DVA, vram_pa, vram_size);
        add_map(d->dart_dcp, BOOT_DVA, vram_pa, vram_size);
    }
''', '''    assert(dcps_started < 4);
    dcp_dev_t *d = &dcps[dcps_started];
    started_with = cfg;
    d->no_reset = cfg->no_reset;
    d->dart_disp = &darts[2 * dcps_started];
    d->dart_dcp = &darts[2 * dcps_started + 1];
    /* iBoot leaves its placeholder mapped for the frontend only, not in the DCP's own DART. */
    add_map(d->dart_disp, BOOT_DVA, vram_pa, vram_size);
''')
    leaves = swap(leaves, 'static void dcp_shutdown(dcp_dev_t *d, bool sleep) { (void)d; (void)sleep; assert(!"DCP start failed"); }',
                  "static int dcp_shutdowns, dcp_sleeps;\n"
                  "static void dcp_shutdown(dcp_dev_t *d, bool sleep) { (void)d; dcp_shutdowns++; dcp_sleeps += sleep; }\n"
                  "static void dcp_ib_shutdown(dcp_iboot_if_t *ib) { (void)ib; }")
    start = leaves.index("static int display_shutdown(dcp_shutdown_mode mode)")
    leaves = leaves[:start] + leaves[leaves.index("static int dcp_connect_dptx"):]
    leaves = swap(leaves, '''    struct mapping *m = find_map(d, iova);
    assert(m && m->size == size && "unmap of a range that is not mapped");
    m->live = false;
    return true;''', '''    struct mapping *m = find_map(d, iova);
    assert(!(iova >> 36) && "table operations take the low 36 bits");
    /* As dart.c: a range that is not mapped is already gone. */
    if (!m)
        return true;
    assert(m->size == size);
    m->live = false;
    return true;''')
    leaves = swap(leaves, "static u64 dart_dva_base(dart_dev_t *d) { (void)d; return 0; }",
                  "#define DVA_WINDOW (1UL << 40)\nstatic u64 dart_dva_base(dart_dev_t *d) { (void)d; return DVA_WINDOW; }")
    leaves = swap(leaves, '''    (void)ib;
    *timing_cnt = nmodes;
    *color_cnt = 1;
    return 1;''', '''    (void)ib;
    /* A fixed HDMI sink behind the converter answers late, or never. */
    if (!sink_after || ++polls < sink_after) {
        *timing_cnt = *color_cnt = 0;
        return 0;
    }
    *timing_cnt = nmodes;
    *color_cnt = 1;
    return 1;''')
    leaves = swap(leaves, "static int nmodes, set_mode_failures;", "static int nmodes, set_mode_failures, set_modes, polls, sink_after;")
    leaves = swap(leaves, '''    (void)ib; (void)t; (void)c;
    if (set_mode_failures) {''', '''    (void)ib; (void)t; (void)c;
    set_modes++;
    if (set_mode_failures) {''')
    leaves = swap(leaves, '''    u64 dva = layer->planes[0].addr;
    struct mapping *a = find_map(dcp->dart_disp, dva), *b = find_map(dcp->dart_dcp, dva);
    assert(a && b && "scanout DVA is not mapped on both DARTs");''', '''    u64 dva = layer->planes[0].addr;
    assert((dva & ~((1UL << 36) - 1)) == DVA_WINDOW && "the DCP is given the DART's window bits");
    dva &= (1UL << 36) - 1;
    struct mapping *a = find_map(dcp->dart_disp, dva);
    assert(a && "scanout DVA is not mapped for the frontend");''')
    leaves = swap(leaves, "static int adt_path_offset(void *a, const char *p) { (void)a; (void)p; return -1; }", '''enum { NODE_DISP0 = 1, NODE_PHY, NODE_CONV, NODE_DCP };
static const char *adt_phy_compat = "dptx-phy,t8122";
static int adt_path_offset(void *a, const char *p)
{
    (void)a;
    if (!strcmp(p, "/arm-io/disp0"))
        return NODE_DISP0;
    if (!strcmp(p, "/arm-io/dptx-phy"))
        return NODE_PHY;
    if (!strcmp(p, "/arm-io/dp2hdmi-gpio0"))
        return NODE_CONV;
    if (!strcmp(p, "/arm-io/dcp"))
        return NODE_DCP;
    return -1;
}
static const void *adt_getprop(void *a, int n, const char *p, u32 *len)
{
    (void)a; (void)len;
    return (n == NODE_DISP0 && !strcmp(p, "external")) ||
           (n == NODE_CONV && (!strcmp(p, "function-hdmi_pwr_en") || !strcmp(p, "function-dp2hdmi_pwr_en"))) ? "" : NULL;
}''')
    leaves = swap(leaves, 'static bool adt_is_compatible(void *a, int n, const char *c) { (void)a; assert(!n && !strcmp(c, "J873gAP")); return is_j873g; }',
                  "static bool adt_is_compatible(void *a, int n, const char *c) { (void)a; return n == NODE_PHY && !strcmp(c, adt_phy_compat); }")
    display = "".join((
        fb.define(disp, "DISPLAY_STATUS_DELAY"), fb.define(disp, "DISPLAY_STATUS_RETRIES"),
        fb.define(disp, "COMPARE"), fb.define(disp, "abs"),
        state, "#define USE_DCPEXT 1\n",
        fb.block(disp, disp.index("static const display_config_t display_config_m2 = {")) + ";\n",
        fb.block(disp, disp.index("static const display_config_t display_config_m2_pro_max = {")) + ";\n",
        fb.block(disp, disp.index("static const display_config_t display_config_m2_ultra = {")) + ";\n",
        leaves,
        fb.function(disp, "u64 display_mode_fb_size("),
        fb.function(disp, "static void display_choose_timing_mode("),
        fb.function(disp, "static void display_choose_color_mode("),
        fb.function(disp, "static bool display_t6030_hands_off("),
        fb.function(disp, "static bool display_unmap_fb("),
        fb.function(disp, "static uintptr_t display_map_fb("),
        fb.function(disp, "static bool display_t8132_hdmi("),
        fb.function(disp, "const display_config_t *display_get_config("),
        fb.function(disp, "int display_start_dcp("),
        fb.block(disp, disp.index("struct display_options {")) + ";\n",
        fb.function(disp, "int display_parse_mode("),
        fb.function(disp, "static int display_swap("),
        fb.function(disp, "int display_configure("),
        fb.function(disp, "int display_shutdown("),
    ))
    prefix = fb.PREFIX + r'''
#define T8140 0x8140
#define T8132 0x8132
#ifndef DCP_RETAINED
#define DCP_RETAINED 1
#endif
#define dcp_retained(x) (DCP_RETAINED && (x)->no_reset)
#define V13_5 1305
#define V27_2 2720
static struct { u32 version; } os_firmware = {V26_6_2};
'''
    return prefix + types + config + display + CASES


CASES = r'''
/* The Mac of the selection: T8132, external disp0, placeholder framebuffer, 1920x1080 sink. */
static void machine(void)
{
    chip_id = T8132;
    display_is_external = true;
    vram_size = 640 * 1136 * 4 + 24 * SZ_16K;
    vram_pa = (u64)aligned_alloc(SZ_16K, vram_size);
    cur_boot_args.video = (struct boot_video){vram_pa, 0, 640 * 4, 640, 1136, 30};
    memset(modes, 0, sizeof(modes));
    modes[0] = (dcp_timing_mode_t){.valid = 1, .width = 1920, .height = 1080, .fps = 60 << 16};
    nmodes = 1;
    sink_after = 1;
}

static void selected_config(void)
{
    machine();
    const display_config_t *conf = display_get_config();
    assert(conf == &display_config_t8132_hdmi && has_dcp);
    assert(conf->no_reset && conf->fw_version == V26_6_2);
    assert(!strcmp(conf->dptx_phy, "/arm-io/dptx-phy") && !strcmp(conf->pmgr_dev, "DISP_CPU"));
    /* Not selected (the 27.2 ADT's PHY name): the default configuration, which resets. */
    adt_phy_compat = "dptx-phy,t8132";
    conf = display_get_config();
    assert(conf == &display_config_m1 && !conf->no_reset);
    /* Every other configuration resets and powers its DCP. */
    assert(!display_config_m2.no_reset && !display_config_j873_hdmi.no_reset);
    assert(display_config_m2.fw_version == V13_5);
}

static void only_from_display_init(void)
{
    machine();
    /* display=, the proxy and the hypervisor reach these without display_init()'s bounds. */
    assert(display_start_dcp() == -1 && !dcps_started);
    assert(display_configure(NULL) == -1 && !dcps_started && !set_modes);
    assert(display_configure("1920x1080") == -1 && !dcps_started);
}

static void bring_up(void)
{
    machine();
    sink_after = 120; /* 12 s: beyond the 10 s every other DPTX sink gets */
    display_retained_admitted = true;
    assert(display_configure(NULL) == 1);
    display_retained_admitted = false;
    assert(started_with == &display_config_t8132_hdmi && dcps_started == 1 && dcp->no_reset);
    assert(hpd_pokes == 2 && "HPD is asserted, then replayed once");
    assert(polls == 120 && set_modes == 1);
    assert(cur_boot_args.video.width == 1920 && cur_boot_args.video.height == 1080);
    assert(find_map(dcp->dart_disp, BOOT_DVA) && find_map(dcp->dart_disp, BOOT_DVA)->pa == cur_boot_args.video.base);
    assert(scanout == BOOT_DVA && "scanned out at the frontend's DVA");

    /* One modeset per boot: a second one (display=, the proxy) is refused before any request. */
    hpd_pokes = polls = 0;
    assert(display_configure("1920x1080") == -1);
    display_retained_admitted = true;
    assert(display_configure(NULL) == -1);
    display_retained_admitted = false;
    assert(set_modes == 1 && !hpd_pokes && !polls && dcps_started == 1);

    /* At the handoff the DCP is quiesced, never slept: nothing here can reset it. */
    assert(display_shutdown(DCP_SLEEP_IF_EXTERNAL) == 0);
    assert(dcp_shutdowns == 1 && !dcp_sleeps && !iboot);
    /* And it is not started again afterwards (hv start-up). */
    assert(display_start_dcp() == -1 && dcps_started == 1);
}

static void no_sink(void)
{
    machine();
    sink_after = 0;
    display_retained_admitted = true;
    assert(display_configure(NULL) == 0);
    assert(polls == 0 && hpd_pokes == 2 && !set_modes);
    assert(cur_boot_args.video.width == 640 && "the placeholder stays");
}

static void no_sink_wait(void)
{
    /* Counted through the status query: 150 of them, 15 s. */
    machine();
    sink_after = 1000;
    display_retained_admitted = true;
    assert(display_configure(NULL) == 0);
    assert(polls == 150 && !set_modes);
}

static void other_dcp_sleeps(void)
{
    /* Control: an external DCP this loader started itself is slept, as before. */
    machine();
    display_retained_admitted = true;
    assert(display_configure(NULL) == 1);
    dcp->no_reset = false;
    assert(display_shutdown(DCP_SLEEP_IF_EXTERNAL) == 0 && dcp_shutdowns == 1 && dcp_sleeps == 1);
}

static void shutdown_exemption(void)
{
    /* A T8132 the selection refuses is left alone at shutdown even with a session open. */
    machine();
    display_retained_admitted = true;
    assert(display_configure(NULL) == 1 && iboot);
    adt_phy_compat = "dptx-phy,t8132";
    assert(display_shutdown(DCP_SLEEP_IF_EXTERNAL) == 0 && !dcp_shutdowns && iboot);
    adt_phy_compat = "dptx-phy,t8122";
    assert(display_shutdown(DCP_SLEEP_IF_EXTERNAL) == 0 && dcp_shutdowns == 1 && !iboot);
}

static void stage1_build(void)
{
    /* Built as a Stage 1 (DCP_RETAINED 0), the same Mac gets the default configuration. */
    machine();
    assert(display_get_config() == &display_config_m1);
}

static const struct { const char *name; void (*run)(void); } cases[] = {
#if DCP_RETAINED
    {"selected-config", selected_config},
    {"only-from-display-init", only_from_display_init},
    {"bring-up", bring_up},
    {"no-sink", no_sink},
    {"no-sink-wait", no_sink_wait},
    {"other-dcp-sleeps", other_dcp_sleeps},
    {"shutdown-exemption", shutdown_exemption},
#else
    {"stage1-build", stage1_build},
#endif
};

int main(int argc, char **argv)
{
    assert(argc == 2);
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        if (strcmp(argv[1], cases[i].name))
            continue;
        cases[i].run();
        printf("PASS %s\n", cases[i].name);
        return 0;
    }
    fprintf(stderr, "unknown case %s\n", argv[1]);
    return 2;
}
'''

CASES_LIST = ("selected-config", "only-from-display-init", "bring-up", "no-sink", "no-sink-wait",
              "other-dcp-sleeps", "shutdown-exemption")


def build(work, retained):
    code = work / f"unit{retained}.c"
    code.write_text(unit())
    program = work / f"unit{retained}"
    result = subprocess.run([os.environ.get("CC", "cc"), "-std=gnu11", "-O1", "-g", "-Wall", "-Wextra",
                             "-Werror", "-Wno-unused-function", "-Wno-unused-variable",
                             "-Wno-unused-const-variable", f"-DDCP_RETAINED={retained}",
                             "-fsanitize=address,undefined", "-fno-sanitize-recover=undefined",
                             str(code), "-o", str(program)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr[-6000:]
    return program


@pytest.fixture(scope="module")
def binary(tmp_path_factory):
    return build(tmp_path_factory.mktemp("display-retained-hdmi"), 1)


@pytest.mark.parametrize("case", CASES_LIST)
def test_case(binary, case):
    result = subprocess.run([str(binary), case], capture_output=True, text=True, timeout=60,
                            env=dict(os.environ, ASAN_OPTIONS="detect_leaks=0"))
    assert result.returncode == 0 and ("PASS " + case) in result.stdout, \
        result.stdout[-3000:] + result.stderr[-3000:]


def test_not_built_into_a_stage1(tmp_path):
    program = build(tmp_path, 0)
    result = subprocess.run([str(program), "stage1-build"], capture_output=True, text=True, timeout=60)
    assert result.returncode == 0 and "PASS stage1-build" in result.stdout, result.stdout + result.stderr
