/* SPDX-License-Identifier: MIT */
/* Host stand-in for src/xnuboot.h: the boot video the start's console frontier sets. */
#ifndef XNUBOOT_H
#define XNUBOOT_H
#include "types.h"
struct boot_video {
    u64 base;
    u64 display;
    u64 stride;
    u64 width;
    u64 height;
    u64 depth;
};
struct boot_args {
    struct boot_video video;
};
extern struct boot_args cur_boot_args;
extern u64 boot_args_addr;
#endif
