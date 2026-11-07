/* SPDX-License-Identifier: MIT */

/*
 * ANS's and APCIe's power cycled with the SoC awake, for an ANS restart. Linux's sleep and wake
 * writes in macOS's order, without what only a real sleep undoes: APCIE_SYS_GP and APCIE_GP stay
 * on, since SYS_GP's power switch doesn't reopen on a power word write, and the clock generator
 * keeps running, since only the sleep resets it. Every step prints its result.
 */

#include "ans_power.h"
#include "adt.h"
#include "pmgr.h"
#include "tunables.h"
#include "utils.h"

#define APCIE_PATH "/arm-io/apcie"

#define PS_BASE         0x300700000UL
#define PS_AUTO_ENABLE  BIT(28)
#define PS_BUSY         BIT(11)
#define PS_WAS_PWRGATED BIT(8)
#define PS_FLAGS        GENMASK(9, 8)
#define PS_ACTUAL       GENMASK(7, 4)
#define PS_TARGET       GENMASK(3, 0)
#define PS_TIMEOUT_US   5000

/* The PMGR's power switches: blocks of 0x80 in the pwrgate page, status at +0x70. */
#define PWRGATE_BASE     0x300724000UL
#define PWRGATE_SIZE     0x4000
#define SWITCH_STATUS(n) ((n) * 0x80 + 0x70)
#define SWITCH_GATED     BIT(30)

/* reg[2], the PHY block: its common block and the PHY's own control */
#define PHY_COMMON          0x4000
#define PHY_COMMON_REF_GOOD BIT(31) /* the reference PLL is locked */
#define PHY_CTRL            0x8000
#define PHY_CTRL_CLK0REQ    BIT(0)
#define PHY_CTRL_CLK1REQ    BIT(1)
#define PHY_CTRL_CLK0ACK    BIT(2)
#define PHY_CTRL_CLK1ACK    BIT(3)
#define PHY_CTRL_RESET      BIT(4)
#define PHY_CTRL_UP         BIT(27)
#define PHY_RUN             0x8004
#define PHY_RUN_EN          BIT(0)
#define PHY_RUN_EN2         BIT(4)
#define PHY_READY           0x8008

/* reg[3], PHY-IP */
#define PHYIP_PLL_CTL  0x90
#define PHYIP_PLL_HOLD BIT(16)

/* reg[4], AXI2AF */
#define AXI_EN0 0x104

/* reg[6], the PCIe clock generator, left running */
#define CLK_CTRL 0x0
#define CLK_OFF  BIT(1)

/* reg[1], the root complex */
#define RC_ZERO        0x04
#define RC_READY_START 0x50
#define RC_READY_CFG   0x54
#define RC_READY_STAT  0x58

#define APCIE_POLL_US 50000

enum { D_ANS, D_APCIE_GP, D_SYS_GP, D_ST, D_SYS_ST, D_PHY_SW, D_COUNT };

static const struct {
    const char *name;
    u32 offset;
    int sw;     /* power switch, -1 for none */
    bool owner; /* its word records the switch's gate */
} domains[D_COUNT] = {
    [D_ANS] = {"ANS", 0x278, 70, true},
    [D_APCIE_GP] = {"APCIE_GP", 0x258, -1, false},
    [D_SYS_GP] = {"APCIE_SYS_GP", 0x2a0, 40, true},
    [D_ST] = {"APCIE_ST", 0x310, 70, false},
    [D_SYS_ST] = {"APCIE_SYS_ST", 0x3a0, 41, true},
    [D_PHY_SW] = {"APCIE_PHY_SW", 0x3c0, 41, false},
};

/* The ADT's /arm-io/apcie windows used here, each checked against its expected address. */
enum { R_RC = 1, R_PHY = 2, R_PHYIP = 3, R_AXI = 4, R_CIO3PLL = 5, R_CLKGEN = 6, R_COUNT };

static const struct {
    u64 addr;
    u64 min_size;
} apcie_regs[R_COUNT] = {
    [R_RC] = {0x394000000, RC_READY_STAT + 4},    [R_PHY] = {0x397000000, PHY_READY + 4},
    [R_PHYIP] = {0x397040000, PHYIP_PLL_CTL + 4}, [R_AXI] = {0x396000000, AXI_EN0 + 4},
    [R_CLKGEN] = {0x395044000, CLK_CTRL + 4},
};

static const struct {
    const char *prop;
    int reg;
} apcie_tunables[] = {
    {"apcie-axi2af-tunables", R_AXI},
    {"apcie-phy-tunables", R_PHY},
    {"apcie-phy-ip-pll-tunables", R_PHYIP},
    {"apcie-phy-ip-auspma-tunables", R_PHYIP},
};

static bool prepared;
static u64 apcie[R_COUNT];
static u64 apcie_size[R_COUNT];

static u64 ps(int d)
{
    return PS_BASE + domains[d].offset;
}

static bool ps_on(int d)
{
    return FIELD_GET(PS_ACTUAL, read32(ps(d))) == 0xf;
}

static u32 switch_status(int d)
{
    return domains[d].sw < 0 ? 0 : read32(PWRGATE_BASE + SWITCH_STATUS(domains[d].sw));
}

COLD_TEXT static void log_words(const char *when)
{
    printf("ans-power: %s: ANS 0x%x APCIE_GP 0x%x SYS_GP 0x%x ST 0x%x SYS_ST 0x%x PHY_SW 0x%x; "
           "switches 40 0x%x 41 0x%x 70 0x%x\n",
           when, read32(ps(D_ANS)), read32(ps(D_APCIE_GP)), read32(ps(D_SYS_GP)), read32(ps(D_ST)),
           read32(ps(D_SYS_ST)), read32(ps(D_PHY_SW)), switch_status(D_SYS_GP),
           switch_status(D_SYS_ST), switch_status(D_ANS));
}

/*
 * iBoot leaves SYS_ST and PHY_SW to the PMGR's own idling, so SYS_ST can read off with switch 41
 * closed while ANS runs. Hold all six on first, without AUTO_ENABLE, as pcie_init() does.
 */
COLD_TEXT static int ans_power_live(void)
{
    log_words("entry");
    if (pmgr_adt_power_enable_traced(APCIE_PATH))
        printf("ans-power: couldn't power %s\n", APCIE_PATH);
    log_words("held on");

    for (int d = 0; d < D_COUNT; d++) {
        if (!ps_on(d) || (domains[d].owner && (switch_status(d) & SWITCH_GATED))) {
            printf("ans-power: %s isn't on\n", domains[d].name);
            return -1;
        }
    }
    printf("ans-power: clock generator 0x%x, PHY control 0x%x run 0x%x\n",
           read32(apcie[R_CLKGEN] + CLK_CTRL), read32(apcie[R_PHY] + PHY_CTRL),
           read32(apcie[R_PHY] + PHY_RUN));
    return 0;
}

COLD_TEXT int ans_power_prepare(void)
{
    int path[8];

    if (prepared)
        return ans_power_live();
    if (chip_id != T8140) {
        printf("ans-power: only T8140 is described\n");
        return -1;
    }

    for (int d = 0; d < D_COUNT; d++) {
        uintptr_t addr = pmgr_device_psreg(0, domains[d].name);
        if (addr != ps(d)) {
            printf("ans-power: PMGR %s at 0x%lx, expected 0x%lx\n", domains[d].name, addr, ps(d));
            return -1;
        }
    }

    u64 base, size;
    if (pmgr_adt_path_offset_trace(adt, path) < 0 ||
        adt_get_reg(adt, path, "reg", 22, &base, &size) < 0 || base != PWRGATE_BASE ||
        size < PWRGATE_SIZE) {
        printf("ans-power: no pwrgate page at 0x%lx in the ADT's PMGR reg[22]\n", PWRGATE_BASE);
        return -1;
    }

    if (adt_path_offset_trace(adt, APCIE_PATH, path) < 0) {
        printf("ans-power: no %s\n", APCIE_PATH);
        return -1;
    }
    for (int r = R_RC; r < R_COUNT; r++) {
        if (!apcie_regs[r].addr)
            continue;
        if (adt_get_reg(adt, path, "reg", r, &apcie[r], &apcie_size[r]) < 0 ||
            apcie[r] != apcie_regs[r].addr || apcie_size[r] < apcie_regs[r].min_size) {
            printf("ans-power: %s reg[%d] at 0x%lx size 0x%lx, expected 0x%lx\n", APCIE_PATH, r,
                   apcie[r], apcie_size[r], apcie_regs[r].addr);
            return -1;
        }
    }
    for (size_t i = 0; i < ARRAY_SIZE(apcie_tunables); i++) {
        if (tunables_validate_local(APCIE_PATH, apcie_tunables[i].prop,
                                    apcie_size[apcie_tunables[i].reg])) {
            printf("ans-power: %s missing or invalid\n", apcie_tunables[i].prop);
            return -1;
        }
    }

    prepared = true;
    printf("ans-power: T8140 APCIe and ANS power words, switches and tables checked\n");
    return ans_power_live();
}

/* As macOS lowers a domain: AUTO_ENABLE off first if set, then target 0, the flags cleared. */
COLD_TEXT static void ps_lower(int d)
{
    u32 val = read32(ps(d));

    if (val & PS_AUTO_ENABLE) {
        val &= ~(PS_AUTO_ENABLE | PS_FLAGS);
        write32(ps(d), val);
    }
    write32(ps(d), (val & ~PS_TARGET) | PS_FLAGS);
}

/* Whether a lowered domain went off, and on a switch's owner whether it really gated. */
COLD_TEXT static bool ps_lowered(int d)
{
    u32 val;

    if (poll32(ps(d), PS_ACTUAL, 0, PS_TIMEOUT_US) < 0) {
        printf("ans-power: down, %s: didn't go off (0x%x)\n", domains[d].name, read32(ps(d)));
        return false;
    }
    if (!domains[d].owner) {
        printf("ans-power: down, %s: off (0x%x), a member of switch %d\n", domains[d].name,
               read32(ps(d)), domains[d].sw);
        return true;
    }
    u64 timeout = timeout_calculate(10000);
    while (((val = read32(ps(d))) & (PS_WAS_PWRGATED | PS_BUSY)) != PS_WAS_PWRGATED &&
           !timeout_expired(timeout))
        ;
    u32 sw = switch_status(d);
    bool gated = (val & (PS_WAS_PWRGATED | PS_BUSY)) == PS_WAS_PWRGATED && (sw & SWITCH_GATED);
    printf("ans-power: down, %s: %s (0x%x), switch %d status 0x%x\n", domains[d].name,
           gated ? "power-gated" : "off, NOT power-gated", val, domains[d].sw, sw);
    return gated;
}

/*
 * The target alone. AUTO_ENABLE stays off, as pcie_init() leaves these domains: the PMGR idling one
 * during the PHY's bring-up would gate the PHY under it.
 */
COLD_TEXT static int ps_raise(int d)
{
    u64 reg = ps(d);

    write32(reg, (read32(reg) & ~(PS_AUTO_ENABLE | GENMASK(11, 0))) | PS_TARGET);
    int ret = poll32(reg, PS_ACTUAL, PS_ACTUAL, PS_TIMEOUT_US);
    udelay(10);

    u32 val = read32(reg), sw = switch_status(d);
    if (!ret && (FIELD_GET(PS_ACTUAL, val) != 0xf || (domains[d].owner && (sw & SWITCH_GATED))))
        ret = -1;
    printf("ans-power: up, %s: %d (0x%x), switch status 0x%x\n", domains[d].name, ret, val, sw);
    return ret;
}

COLD_TEXT static int step(const char *what, int ret, u32 val)
{
    printf("ans-power: up, %s: %d (0x%x)\n", what, ret, val);
    return ret;
}

COLD_TEXT int ans_power_down(void)
{
    u64 ctrl = apcie[R_PHY] + PHY_CTRL;
    u64 run = apcie[R_PHY] + PHY_RUN;
    u64 clk = apcie[R_CLKGEN] + CLK_CTRL;
    int not_gated = 0;

    if (!prepared)
        return -1;
    log_words("before");

    /* A poll here only paces the steps: the power goes next whatever it reads. */
    set32(ctrl, PHY_CTRL_RESET);
    clear32(ctrl, PHY_CTRL_CLK1REQ);
    poll32(ctrl, PHY_CTRL_CLK1ACK, 0, APCIE_POLL_US);
    clear32(ctrl, PHY_CTRL_CLK0REQ);
    poll32(ctrl, PHY_CTRL_CLK0ACK, 0, APCIE_POLL_US);
    clear32(ctrl, PHY_CTRL_UP);
    clear32(run, PHY_RUN_EN2);
    clear32(run, PHY_RUN_EN);
    printf("ans-power: down, PHY quiesced: control 0x%x run 0x%x\n", read32(ctrl), read32(run));
    ps_lower(D_PHY_SW);

    /* Kept running: it survives SYS_ST's switch, and only a real sleep resets it. */
    printf("ans-power: down, clock generator kept running: 0x%x\n", read32(clk));
    ps_lower(D_SYS_ST);
    ps_lower(D_ST);
    ps_lower(D_ANS);

    static const int lowered[] = {D_PHY_SW, D_SYS_ST, D_ST, D_ANS};
    for (size_t i = 0; i < ARRAY_SIZE(lowered); i++)
        if (!ps_lowered(lowered[i]) && domains[lowered[i]].owner)
            not_gated++;
    log_words("down");

    return not_gated;
}

COLD_TEXT int ans_power_up(void)
{
    u64 phy = apcie[R_PHY];
    u64 ctrl = phy + PHY_CTRL;
    u64 run = phy + PHY_RUN;
    u64 clk = apcie[R_CLKGEN] + CLK_CTRL;
    u64 rc = apcie[R_RC];
    u32 writes[2];
    int ret;

    if (!prepared)
        return -1;
    if (ps_raise(D_ANS))
        return -1;

    if (!ps_on(D_APCIE_GP) || !ps_on(D_SYS_GP) || (switch_status(D_SYS_GP) & SWITCH_GATED))
        return step("APCIE_GP and SYS_GP kept on", -1, read32(ps(D_SYS_GP)));
    step("APCIE_GP and SYS_GP kept on, AXI enable", 0, read32(apcie[R_AXI] + AXI_EN0));

    if (ps_raise(D_ST) || ps_raise(D_SYS_ST))
        return -1;

    if (tunables_apply_local(APCIE_PATH, "apcie-axi2af-tunables", R_AXI))
        return step("AXI tunables", -1, 0);

    if (step("clock generator", (read32(clk) & CLK_OFF) ? -1 : 0, read32(clk)))
        return -1;

    if (ps_raise(D_PHY_SW))
        return -1;

    if (tunables_apply_local(APCIE_PATH, "apcie-phy-tunables", R_PHY))
        return step("PHY tunables", -1, 0);
    step("PHY tunables, PHY control", 0, read32(ctrl));
    /* Without it the PHY can't come ready: named here, and not fatal until RUN_EN2 is. */
    step("reference PLL good",
         poll32(phy + PHY_COMMON, PHY_COMMON_REF_GOOD, PHY_COMMON_REF_GOOD, APCIE_POLL_US),
         read32(phy + PHY_COMMON));
    set32(ctrl, PHY_CTRL_CLK0REQ);
    ret = poll32(ctrl, PHY_CTRL_CLK0ACK, PHY_CTRL_CLK0ACK, APCIE_POLL_US);
    if (step("PHY CLK0 request acknowledged", ret, read32(ctrl)))
        return -1;
    set32(ctrl, PHY_CTRL_CLK1REQ);
    ret = poll32(ctrl, PHY_CTRL_CLK1ACK, PHY_CTRL_CLK1ACK, APCIE_POLL_US);
    if (step("PHY CLK1 request acknowledged", ret, read32(ctrl)))
        return -1;
    clear32(ctrl, PHY_CTRL_RESET);

    set32(run, PHY_RUN_EN);
    step("PHY reset released, RUN_EN set", 0, read32(run));

    /* Lost with SYS_ST's switch: macOS's way, PLL then lanes, between the two run bits. */
    if (tunables_apply_local_changed(APCIE_PATH, "apcie-phy-ip-pll-tunables", apcie[R_PHYIP],
                                     &writes[0]) ||
        tunables_apply_local_changed(APCIE_PATH, "apcie-phy-ip-auspma-tunables", apcie[R_PHYIP],
                                     &writes[1]))
        return step("PHY-IP tables", -1, 0);
    clear32(apcie[R_PHYIP] + PHYIP_PLL_CTL, PHYIP_PLL_HOLD);
    printf("ans-power: up, PHY-IP tables: %u PLL and %u lane writes, PLL control 0x%x\n", writes[0],
           writes[1], read32(apcie[R_PHYIP] + PHYIP_PLL_CTL));

    set32(run, PHY_RUN_EN2);
    ret = poll32(phy + PHY_READY, BIT(0), BIT(0), APCIE_POLL_US);
    if (step("RUN_EN2, PHY ready", ret, read32(phy + PHY_READY)))
        return -1;
    set32(phy + PHY_COMMON, BIT(0));
    set32(ctrl, PHY_CTRL_UP);

    write32(rc + RC_ZERO, 0);
    write32(rc + RC_READY_CFG, 0x140);
    write32(rc + RC_READY_START, 1);
    ret = poll32(rc + RC_READY_STAT, BIT(0), BIT(0), APCIE_POLL_US);
    if (step("RC ready", ret, read32(rc + RC_READY_STAT)))
        return -1;

    log_words("up");
    return 0;
}
