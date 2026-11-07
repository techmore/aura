/* SPDX-License-Identifier: MIT */
/* Host stand-in for src/firmware.h: the versions the J773g display code compares. */
#ifndef FIRMWARE_H
#define FIRMWARE_H
#include "types.h"
enum fw_version {
    V_UNKNOWN = 0,
    V26_5 = 101,
    V26_6_2 = 102,
};
struct fw_version_info {
    enum fw_version version;
    const char *string;
    u32 num[5];
    size_t num_length;
    const char *iboot;
};
extern struct fw_version_info os_firmware;
int firmware_set_fdt(void *fdt, int node, const char *prop, const struct fw_version_info *ver);
#endif
