/* SPDX-License-Identifier: MIT */

#ifndef KBOOT_DISPLAY_J773G_H
#define KBOOT_DISPLAY_J773G_H

#include "types.h"

/* The /chosen property (config line chosen.aura,j773g-display=<mode>) that selects the mode. */
#define J773G_DISPLAY_KNOB "aura,j773g-display"

enum j773g_display_mode {
    J773G_MODE_OFF,     /* absent, "off" or unknown: stock behaviour, nothing published */
    J773G_MODE_HANDOFF, /* publish the main-DCP handoff; the loader never touches the DCP */
    J773G_MODE_START,   /* the loader lights HDMI itself (J773G_DISPLAY_START builds only) */
};

enum j773g_display_mode j773g_display_mode_parse(const char *value);

/* Memory the loader knows is live or in use at the handoff, which no DCP segment may overlap. */
struct j773g_display_env {
    u64 loader_base, loader_size; /* this stage's loaded file part */
    u64 exec_base, exec_size;     /* the CTXR exec window the kernel runs in, or 0 */
};

/*
 * The T8132 branch of dt_set_display(). Reads chosen.aura,j773g-display from the FDT; with the
 * knob absent or "off" it returns 0 without reading the ADT or changing a byte. Returns 1 when it
 * published a complete J773g main-DCP handoff (the caller skips the stock T8132 path), 0 when
 * nothing was published (the FDT is byte-for-byte what it was; the caller runs the stock path).
 */
int kboot_j773g_display(void *dt, const struct j773g_display_env *env);

/*
 * Layer B2 (only with make J773G_DISPLAY_START=1). The loader's own DCP session, run by kboot
 * before the FDT is prepared; it records how far it got here. kboot_j773g_display() then publishes
 * the matching simpledrm-only or cold-required handoff.
 */
struct j773g_start_state {
    bool ran;         /* j773g_display_start() was entered */
    bool touched;     /* a frontier changed state: a DART leaf, an RTKit/AFK session, SMC, PHY */
    bool surface;     /* the scanout is up, boot_args.video describes it and the DCP is
                         quiesced (running, oslog echoed) */
    const char *stop; /* the frontier that stopped the run, or NULL */
};

extern struct j773g_start_state j773g_start_state;
int j773g_display_start(void);

/* NULL when chip, ADT identity, firmware, segments and DART windows are the J773g's. */
const char *j773g_display_check_identity(void);

#endif
