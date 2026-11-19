/* SPDX-License-Identifier: MIT AND GPL-2.0-only */

#ifndef DCP_DPTX_PHY_H
#define DCP_DPTX_PHY_H

#include "../../build/build_cfg.h"

#include "../types.h"

/*
 * The HDMI bring-up behind a DCP iBoot left running, and the T8122 transmitter it needs, are not
 * built into a Stage 1: until they have run on hardware in this form they stay in the stages
 * that can be replaced without access to the Mac.
 */
#if defined(CHAINLOADING) && !defined(ESP_STAGE2)
#define DCP_RETAINED 0
#else
#define DCP_RETAINED 1
#endif

typedef struct dptx_phy dptx_phy_t;

typedef struct dptx_drive_settings {
    u32 format;
    u32 voltage;
    u32 pre_emphasis;
} dptx_drive_settings_t;

int dptx_phy_activate(dptx_phy_t *phy);
int dptx_phy_deactivate(dptx_phy_t *phy);
int dptx_phy_set_route(dptx_phy_t *phy, bool enable);
int dptx_phy_set_active_lane_count(dptx_phy_t *phy, u32 num_lanes);
u32 dptx_phy_get_active_lane_count(dptx_phy_t *phy);
int dptx_phy_set_drive_settings(dptx_phy_t *phy, const dptx_drive_settings_t *settings, u32 count);
int dptx_phy_set_link_rate(dptx_phy_t *phy, u32 link_rate);
u32 dptx_phy_max_link_rate(dptx_phy_t *phy);

u32 dptx_phy_dcp_output(dptx_phy_t *phy);

dptx_phy_t *dptx_phy_init(const char *phy_path, u32 dcp_index);
void dptx_phy_shutdown(dptx_phy_t *phy);

#endif /* DCP_DPTX_PHY_H */
