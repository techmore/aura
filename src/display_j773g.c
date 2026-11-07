/* SPDX-License-Identifier: MIT */

/*
 * Layer B2: aura lights the J773g HDMI port itself, from the main DCP (/arm-io/dcp), before
 * Linux. Built only with make J773G_DISPLAY_START=1; a default build has none of this code and
 * kboot logs that it cannot start the DCP.
 *
 * iBoot leaves the DCP running under PMGR automatic power management. The loader never writes
 * its power state (DISP_CPU writes fault; pmgr.c refuses them on J773g), never resets it and
 * never puts it to sleep: it wakes the running firmware over RTKit, as Linux does, and ends its
 * session with AP QUIESCED then IOP QUIESCED, echoing every oslog notify so that the DCP acks.
 * The DCP then keeps running with the scanout up, and any later client (Linux) only sends wake
 * and HELLO. Only the DPTX PHY is powered by the loader.
 *
 * The session is a fixed sequence of bounded frontiers, each logged with its deadline, its pass
 * signal and its stop condition. A frontier that fails or overruns its deadline ends the run:
 * no retry, no teardown, no sleep or reset (display_shutdown() stays a no-op on T8132). The
 * state is left as it is, the boot continues on the dummy framebuffer, and once a frontier has
 * changed state the next boot must be a cold one. kboot_j773g_display() then reserves the memory
 * the DCP may use and keeps the DCP node disabled, so that Linux opens no session of its own.
 *
 * Every frontier that is not implemented in this tree is listed before the run, and the run is
 * refused before the first write if any is missing: a partial ladder must never open a DCP
 * session it cannot take to a quiesced surface. Firmware-dependent layouts are the 26.6.2 ones
 * (the OS firmware the predicates require), not 27.0's AFK v2 / iBoot-service variants. The
 * recipe the missing frontiers must follow (aurora council-bootmenu/track-b-gates.md 1.2, and
 * Pavel Shago's J773g m1n1 bring-up, read for facts only, GPL-2.0-or-later):
 *
 *   map:      /vram (0x854000 at 0x103e5300000) leaf-only at table offset 0x4000 in dart-dcp
 *             SID 23 and dart-disp0 SID 0, inside iBoot's existing L2 tables; never a TTBR,
 *             TCR or L1 write; bounded T8110 TLB completion. The DVA the DCP is given is
 *             vm-base | 0x4000 (bit 40). __OS_LOG is the firmware's own channel.
 *   rtkit:    wake the running ASC and HELLO (v12); rtkit_set_dva_base(vm-base bits >= 36) and
 *             rtkit_set_oslog_echo(); NC buffers from the /vram tail; no CPU start.
 *   prefix:   compact AFK endpoints in the Linux 26.6 order: system 0x20, IOMFB 0x37,
 *             DPDEV 0x27, DPAVCTRL 0x24, DPSAC 0x26, AV 0x29, remote-alloc 0x2d, iBoot 0x23,
 *             EPIC2C 0x2c, DPAVSERV 0x28, expert 0x22, EPIC25 0x25 (DPTX 0x2a comes last).
 *   phy:      power /arm-io/dptx-phy; dptx-phy,t8122 at 0x19a700000/0x19a740000,
 *             dp-switch-dfp-endpoint 4, die 0.
 *   connect:  DPTX 0x2a, port 0 to target 4.
 *   hpd:      GET_HPD while servicing the DCP between polls: HPD plus nonzero timing/colour
 *             counts in 15 s (the 26.6.2 converter raises HPD itself); three errors stop.
 *   set-power (optional, off): SET_POWER before connect is a 27.0 step, unknown on 26.6.2.
 *   mode:     a mode that fits /vram, SET_MODE.
 *   surface:  SET_SURFACE on disp0-service at the /vram DVA, w30r, colorspace 2, SDR.
 *   quiesce:  endpoint shutdowns, AP QUIESCED, IOP QUIESCED with the oslog echo; the DCP stays
 *             running.
 */

#include "../build/build_cfg.h"

#ifdef J773G_DISPLAY_START

#include "adt.h"
#include "fb.h"
#include "kboot_display_j773g.h"
#include "smc.h"
#include "string.h"
#include "utils.h"
#include "xnuboot.h"

struct j773g_start_state j773g_start_state;

struct j773g_session {
    u64 vram, vram_size;
    u64 dva_base; /* vm-base bits >= 36 of both display DARTs: every DVA the DCP gets has them */
    u64 dva;      /* the surface's DVA in both DARTs: dva_base | 0x4000 */
    struct boot_video video;
};

struct j773g_frontier {
    const char *name;
    u32 deadline_ms;
    bool writes;   /* changes state: from here on the next boot must be cold */
    bool optional; /* not run by default (no evidence it is needed on 26.6.2) */
    /* NULL: not implemented in this tree (the run is refused before any write) */
    int (*run)(struct j773g_session *s, u64 deadline);
    const char *pass;
    const char *stop;
};

/* F0 (read-only): the exact J773g identity, firmware and memory layout. */
static int j773g_f_identity(struct j773g_session *s, u64 deadline)
{
    UNUSED(deadline);
    const char *why = j773g_display_check_identity();
    if (why) {
        printf("j773g-start: identity: %s\n", why);
        return -1;
    }
    int trace[8];
    if (adt_path_offset_trace(adt, "/vram", trace) < 0 ||
        adt_get_reg(adt, trace, "reg", 0, &s->vram, &s->vram_size) < 0)
        return -1;

    /* Both DARTs translate the same window: vm-base 1 TiB on J773g 26.6.2. */
    static const char *const darts[] = {"/arm-io/dart-dcp", "/arm-io/dart-disp0"};
    for (int i = 0; i < 2; i++) {
        u64 vm_base = 0;
        int node = adt_path_offset(adt, darts[i]);
        if (node < 0 || ADT_GETPROP(adt, node, "vm-base", &vm_base) != sizeof(vm_base))
            return -1;
        vm_base &= ~((1UL << 36) - 1);
        if (!vm_base || (i && vm_base != s->dva_base))
            return -1;
        s->dva_base = vm_base;
    }
    s->dva = s->dva_base | SZ_16K;
    printf("j773g-start: DVA base 0x%lx, surface DVA 0x%lx\n", s->dva_base, s->dva);
    return 0;
}

/*
 * F1 (read-only): the DCP is as iBoot leaves it, preloaded and running. PMGR is not touched: the
 * CPU domain is under automatic power management and is never written.
 */
static int j773g_f_iboot(struct j773g_session *s, u64 deadline)
{
    UNUSED(s);
    UNUSED(deadline);
    u32 running = 0, preloaded = 0;
    int nub = adt_path_offset(adt, "/arm-io/dcp/iop-dcp-nub");
    if (nub < 0 || ADT_GETPROP(adt, nub, "running", &running) != sizeof(running) ||
        ADT_GETPROP(adt, nub, "pre-loaded", &preloaded) != sizeof(preloaded))
        return -1;
    printf("j773g-start: iBoot DCP running %u, pre-loaded %u\n", running, preloaded);
    return running == 1 && preloaded == 1 ? 0 : -1;
}

/* The SMC GPIO functions of /arm-io/dp2hdmi-gpio0: pKW4(key, value). */
struct j773g_smc_function {
    u32 phandle;
    char four_cc[4];
    u32 key;
    u32 value;
};

#define J773G_SMC_KEY(a, b, c, d) (((u32)(a) << 24) | ((u32)(b) << 16) | ((u32)(c) << 8) | (d))
#define J773G_RAIL_ON             0x1000001

/*
 * The HDMI rail (gP0f), then the DP-to-HDMI converter (gP1a), each written 0x1000001 (Linux's
 * value), then 100 ms for the converter. UNPROVEN in a loader: Linux drives both high in every
 * HDMI boot; m1n1 uses 0x800001 for both before 27.0 and the ADT arguments on 27.0.
 */
static int j773g_f_rails(struct j773g_session *s, u64 deadline)
{
    UNUSED(s);
    static const char *const props[] = {"function-hdmi_pwr_en", "function-dp2hdmi_pwr_en"};
    const u32 keys[] = {J773G_SMC_KEY('g', 'P', '0', 'f'), J773G_SMC_KEY('g', 'P', '1', 'a')};
    struct j773g_smc_function fn[2];
    int node = adt_path_offset(adt, "/arm-io/dp2hdmi-gpio0");

    for (int i = 0; i < 2; i++)
        if (node < 0 || adt_getprop_copy(adt, node, props[i], &fn[i], sizeof(fn[i])) < 0 ||
            fn[i].key != keys[i])
            return -1;
    smc_dev_t *smc = smc_init();
    if (!smc)
        return -1;
    int ret = 0;
    for (int i = 0; i < 2 && !ret; i++) {
        ret = smc_write_u32(smc, keys[i], J773G_RAIL_ON);
        printf("j773g-start: SMC %s <- 0x%x: %d\n", props[i] + 9, J773G_RAIL_ON, ret);
    }
    smc_shutdown(smc);
    if (ret)
        return -1;
    mdelay(100);
    return timeout_expired(deadline) ? -1 : 0;
}

/* The console moves to the scanout; nothing else of the loader's state changes. */
static int j773g_f_console(struct j773g_session *s, u64 deadline)
{
    UNUSED(deadline);
    if (!s->video.base || !s->video.width || !s->video.height || !s->video.stride ||
        s->video.stride * s->video.height > s->vram_size)
        return -1;
    cur_boot_args.video = s->video;
    memcpy((void *)boot_args_addr, &cur_boot_args, sizeof(cur_boot_args));
    fb_reinit();
    return 0;
}

static const struct j773g_frontier j773g_frontiers[] = {
    {"identity", 10, false, false, j773g_f_identity,
     "J773g, 26.6.2, six segments, DART windows, one vm-base", "any mismatch"},
    {"iboot", 10, false, false, j773g_f_iboot, "ADT says the DCP is preloaded and running",
     "not running or not preloaded"},
    {"map", 50, true, false, NULL, "/vram leaves read back in both DARTs at table offset 0x4000",
     "occupied leaf, L2 outside the pt-region, readback mismatch, TLB timeout"},
    {"rtkit", 2000, true, false, NULL, "wake and HELLO, RTKit v12, stable state",
     "wrong version, crash, timeout"},
    {"prefix", 2000, true, false, NULL, "twelve services announced and opened in the 26.6 order",
     "missing, duplicate or unexpected service, three protocol errors"},
    {"rails", 200, true, false, j773g_f_rails, "gP0f and gP1a acknowledged, 100 ms waited",
     "wrong ADT key, SMC error"},
    {"phy", 500, true, false, NULL, "dptx-phy powered, T8122 PHY ready for target 4 die 0",
     "wrong target/type, timeout"},
    {"set-power", 1000, true, true, NULL, "SET_POWER(on) acknowledged (27.0 step)",
     "service error, timeout"},
    {"connect", 2000, true, false, NULL, "DPTX 0x2a open, port 0 connected to target 4",
     "link training error, callback out of order, timeout"},
    {"hpd", 15000, false, false, NULL, "HPD with nonzero timing and colour counts",
     "15 s, three command errors"},
    {"mode", 3000, true, false, NULL, "SET_MODE succeeds", "bad counts, timeout, link loss"},
    {"surface", 1000, true, false, NULL, "SET_SURFACE on disp0-service succeeds",
     "service error, fault, timeout"},
    {"quiesce", 2000, true, false, NULL, "AP and IOP QUIESCED acked, the DCP still running",
     "no ack (oslog not echoed?), crash, timeout"},
    {"console", 1000, false, false, j773g_f_console,
     "boot_args.video is the surface, console redrawn", "geometry outside /vram"},
};

int j773g_display_start(void)
{
    struct j773g_session s;
    size_t missing = 0;

    memset(&s, 0, sizeof(s));
    j773g_start_state = (struct j773g_start_state){.ran = true};

    for (size_t i = 0; i < ARRAY_SIZE(j773g_frontiers); i++)
        if (!j773g_frontiers[i].run && !j773g_frontiers[i].optional) {
            printf("j773g-start: frontier %s is not implemented in this tree\n",
                   j773g_frontiers[i].name);
            missing++;
        }
    if (missing) {
        printf("j773g-start: %zu frontiers missing; refused before the first hardware write\n",
               missing);
        j773g_start_state.stop = "not built";
        return -1;
    }

    for (size_t i = 0; i < ARRAY_SIZE(j773g_frontiers); i++) {
        const struct j773g_frontier *f = &j773g_frontiers[i];
        if (f->optional) {
            printf("j773g-start: [%zu] %s: optional, not run\n", i, f->name);
            continue;
        }
        printf("j773g-start: [%zu] %s: deadline %u ms, pass: %s\n", i, f->name, f->deadline_ms,
               f->pass);
        /* Counted before the call: a failed write may still have changed something. */
        if (f->writes)
            j773g_start_state.touched = true;
        u64 deadline = timeout_calculate(f->deadline_ms * 1000);
        int ret = f->run(&s, deadline);
        if (ret || timeout_expired(deadline)) {
            printf("j773g-start: [%zu] %s stopped (%s): %s; no teardown%s\n", i, f->name,
                   ret ? "failed" : "deadline", f->stop,
                   j773g_start_state.touched ? ", next boot must be cold" : "");
            j773g_start_state.stop = f->name;
            return -1;
        }
        printf("j773g-start: [%zu] %s passed\n", i, f->name);
    }
    j773g_start_state.surface = true;
    return 0;
}

#endif
