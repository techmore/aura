# SPDX-License-Identifier: MIT
"""J773g (M4 Mac mini) main-DCP display: chosen.aura,j773g-display=off|handoff|start.

Builds tests/j773g_display_host.c with the tree's src/kboot_display_j773g.c, the host stand-ins
of tests/j773g_display_host/ and aura's libfdt (ASan, UBSan), twice: as a default build and as a
make J773G_DISPLAY_START=1 build. Every scenario runs on tests/t8132-j773g-display.dts (the U1
test tree's main-DCP group in its loader-owned shape) against an ADT modelled on the J773g 26.6.2
dump. The published tree is kept as tmp artifacts (dtb and dts).

Also checked from the sources: dt_set_display() runs the J773g hook before, and otherwise
exactly as, the stock T8132 path, and the J773g display code never calls PMGR.
"""
from pathlib import Path
import re
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]
LIBFDT = ["fdt.c", "fdt_ro.c", "fdt_rw.c", "fdt_wip.c", "fdt_sw.c", "fdt_empty_tree.c",
          "fdt_strerror.c", "fdt_addresses.c"]
HOST = ROOT / "tests/j773g_display_host.c"
DTS = ROOT / "tests/t8132-j773g-display.dts"
START_LINE = "FDT: j773g-display=start requested but this build cannot start the DCP"


def build(tmp, start):
    cc = shutil.which("cc")
    if not cc:
        pytest.skip("no host C compiler")
    src = tmp / "src"
    src.mkdir(parents=True)
    (tmp / "build").mkdir()
    (tmp / "build/build_cfg.h").write_text("#define J773G_DISPLAY_START\n" if start else "\n")
    for name in ("kboot_display_j773g.c", "kboot_display_j773g.h"):
        shutil.copy(ROOT / "src" / name, src)
    shutil.copy(HOST, src)
    for header in (ROOT / "tests/j773g_display_host").glob("*.h"):
        shutil.copy(header, src)
    binary = tmp / "j773g-display-host"
    subprocess.run(
        [cc, "-std=gnu11", "-D_GNU_SOURCE", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
         "-Wno-unused-parameter", "-Wno-sign-compare", "-Wno-format-truncation",
         "-Wno-unused-function", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
         "-fno-sanitize-recover=undefined",
         "-I", str(src), "-I", str(ROOT / "src"),
         str(src / "j773g_display_host.c"), str(src / "kboot_display_j773g.c"),
         *(str(ROOT / "src/libfdt" / f) for f in LIBFDT), "-o", str(binary)],
        check=True,
    )
    return binary


@pytest.fixture(scope="module")
def dtb(tmp_path_factory):
    dtc = shutil.which("dtc")
    if not dtc:
        pytest.skip("no dtc")
    out = tmp_path_factory.mktemp("j773g-dtb") / "t8132-j773g-display.dtb"
    subprocess.run([dtc, "-q", "-I", "dts", "-O", "dtb", "-o", str(out), str(DTS)], check=True)
    return out


@pytest.fixture(scope="module", params=[False, True], ids=["default-build", "start-build"])
def host(request, tmp_path_factory):
    return request.param, build(tmp_path_factory.mktemp("j773g"), request.param)


def run(binary, *args):
    env = {"ASAN_OPTIONS": "detect_leaks=0", "PATH": "/usr/bin:/bin"}
    return subprocess.run([str(binary), *map(str, args)], capture_output=True, text=True, env=env)


def scenarios(binary):
    return run(binary, "list").stdout.split()


def test_every_scenario_passes(host, dtb, tmp_path):
    start, binary = host
    names = scenarios(binary)
    required = {"off-absent", "off", "handoff", "j713-tree", "j713-adt", "wrong-firmware",
                "wrong-uuid", "seven-segments", "dart-extra-leaf", "oslog-memreserve",
                "no-space"}
    required |= {"start-surface", "start-failed", "start-untouched"} if start else \
        {"start-default-build"}
    assert required <= set(names)
    for name in names:
        out = tmp_path / f"{name}.dtb"
        result = run(binary, dtb, name, out)
        assert result.returncode == 0, f"{name}\n{result.stdout}{result.stderr}"
        assert f"{name}: passed" in result.stdout
        if name == "start-default-build":
            assert START_LINE in result.stdout
        if name.startswith("off"):
            assert "J773g display" not in result.stdout or name == "off-unknown"


def test_published_tree_artifact(dtb, tmp_path):
    """The published tree, decompiled, carries what the U1 kernel reads."""
    binary = build(tmp_path / "b", False)
    out = tmp_path / "j773g-handoff.dtb"
    result = run(binary, dtb, "handoff", out)
    assert result.returncode == 0, result.stdout + result.stderr
    dts = subprocess.run(["dtc", "-q", "-I", "dtb", "-O", "dts", str(out)], check=True,
                         capture_output=True, text=True).stdout
    (tmp_path / "j773g-handoff.dts").write_text(dts)
    for line in ('apple,firmware-uuid = "81BAF476-679D-3CC0-8C6A-E6B6FA659D5E";',
                 'aurora,j773g-display-handoff = "iboot-running";',
                 'dcp = "/soc/dcp@30ae00000";', 'disp0 = "/soc/display-subsystem";',
                 'disp0_piodma = "/soc/dcp@30ae00000/piodma";',
                 "asc-firmware@100002a8000 {", "asc-firmware@10003f0c000 {",
                 "framebuffer@103e5300000 {", "apple,dcp-os-log;"):
        assert line in dts, line
    assert "dcpext-firmware-ready" not in dts


def test_t8132_branch_runs_the_hook_first_and_the_stock_path_unchanged():
    kboot = (ROOT / "src/kboot.c").read_text()
    start = kboot.index("static int dt_set_display(void)")
    body = kboot[start:kboot.index("\n}\n", start)]
    branch = body[body.index("if (chip_id == T8132) {"):]
    branch = branch[:branch.index("return 0;\n    }") + len("return 0;\n    }")]
    assert re.sub(r"\s+", " ", branch) == re.sub(r"\s+", " ", """if (chip_id == T8132) {
        /* J773g with chosen.aura,j773g-display set; otherwise the FDT is left untouched. */
        if (dt_set_j773g_display())
            return 0;
        if (dt_set_m4_dcp_handoff())
            printf("FDT: M4 DCP handoff refused; keeping the boot framebuffer\\n");
        else
            dt_reserve_j713_dcpext();
        return 0;
    }""")


def test_j773g_display_code_never_calls_pmgr():
    """The DCP's CPU domain is under automatic power management and is never written."""
    for path in sorted((ROOT / "src").glob("*display_j773g.c")):
        name = path.name
        code = re.sub(r"/\*.*?\*/", "", path.read_text(), flags=re.S)
        assert "pmgr" not in code, name
        assert "pmgr.h" not in code, name
