# SPDX-License-Identifier: MIT
"""J773g loader-side HDMI start (layer B2, make J773G_DISPLAY_START=1).

A default build has no loader-start code at all: the object, the kboot call and the frontiers
exist only with the make flag. With it, src/display_j773g.c is compiled against host stand-ins
whose every function counts its calls: while any required frontier is not implemented the run
is refused before even the read-only frontiers, so no hardware can be touched, and the optional
27.0-only SET_POWER frontier is not counted as missing.
"""
from pathlib import Path
import re
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]


def run(binary):
    env = {"ASAN_OPTIONS": "detect_leaks=0", "PATH": "/usr/bin:/bin"}
    return subprocess.run([str(binary)], capture_output=True, text=True, env=env)


def test_default_build_has_no_loader_start():
    makefile = (ROOT / "Makefile").read_text()
    assert re.search(r"ifeq \(\$\(J773G_DISPLAY_START\),1\)\nCFG \+= J773G_DISPLAY_START\nendif",
                     makefile)
    assert re.search(r"ifeq \(\$\(J773G_DISPLAY_START\),1\)\nOBJECTS \+= display_j773g\.o\nendif",
                     makefile)
    assert not re.search(r"(?<![_a-z])display_j773g\.o",
                         makefile.split("OBJECTS := \\")[1].split("\n\n")[0])
    start = (ROOT / "src/display_j773g.c").read_text()
    body = start[start.index('#include "../build/build_cfg.h"'):]
    assert body.split("\n", 2)[2].startswith("#ifdef J773G_DISPLAY_START")
    assert body.rstrip().endswith("#endif")
    kboot = (ROOT / "src/kboot.c").read_text()
    call = kboot.index("j773g_display_start();")
    assert kboot.rfind("#ifdef J773G_DISPLAY_START", 0, call) > kboot.rfind("#endif", 0, call)


START_HARNESS = r'''
#include <assert.h>
#include <string.h>
#include "kboot_display_j773g.h"
#include "adt.h"
#include "fb.h"
#include "smc.h"
#include "utils.h"
#include "xnuboot.h"
void *adt = (void *)1;
u32 chip_id = 0x8132, board_id = 0x2a;
struct boot_args cur_boot_args;
u64 boot_args_addr;
static int calls;
const char *j773g_display_check_identity(void) { calls++; return NULL; }
int adt_path_offset(const void *a, const char *p) { calls++; return -1; }
int adt_path_offset_trace(const void *a, const char *p, int *o) { calls++; return -1; }
int adt_get_reg(const void *a, int *p, const char *n, int i, u64 *x, u64 *s) { calls++; return -1; }
int adt_getprop_copy(const void *a, int n, const char *p, void *o, size_t l) { calls++; return -1; }
smc_dev_t *smc_init(void) { calls++; return NULL; }
int smc_write_u32(smc_dev_t *s, u32 k, u32 v) { calls++; return -1; }
void smc_shutdown(smc_dev_t *s) { calls++; }
void fb_reinit(void) { calls++; }
u64 timeout_calculate(u32 usec) { return usec; }
bool timeout_expired(u64 t) { return false; }
void udelay(u32 d) { calls++; }
int main(void)
{
    assert(j773g_display_start() == -1);
    assert(j773g_start_state.ran && !j773g_start_state.touched && !j773g_start_state.surface);
    assert(!strcmp(j773g_start_state.stop, "not built"));
    assert(calls == 0); /* not even the read-only frontiers ran */
    return 0;
}
'''


def test_start_runner_refuses_before_the_first_write(tmp_path):
    cc = shutil.which("cc")
    if not cc:
        pytest.skip("no host C compiler")
    src = tmp_path / "src"
    src.mkdir()
    (tmp_path / "build").mkdir()
    (tmp_path / "build/build_cfg.h").write_text("#define J773G_DISPLAY_START\n")
    shutil.copy(ROOT / "src/display_j773g.c", src)
    shutil.copy(ROOT / "src/kboot_display_j773g.h", src)
    for header in (ROOT / "tests/j773g_display_host").glob("*.h"):
        shutil.copy(header, src)
    (src / "main.c").write_text(START_HARNESS)
    binary = tmp_path / "start"
    subprocess.run([cc, "-std=gnu11", "-Wall", "-Werror", "-Wno-unused-parameter",
                    "-Wno-format", "-I", str(src), str(src / "main.c"),
                    str(src / "display_j773g.c"), "-o", str(binary)], check=True)
    result = run(binary)
    assert result.returncode == 0, result.stdout + result.stderr
    missing = re.findall(r"frontier (\S+) is not implemented", result.stdout)
    assert missing == ["map", "rtkit", "prefix", "phy", "connect", "hpd", "mode", "surface",
                       "quiesce"]
    assert "set-power" not in missing
    assert "refused before the first hardware write" in result.stdout
