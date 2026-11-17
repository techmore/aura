# SPDX-License-Identifier: MIT
"""The live post-M4 ANS takeover is admitted only for J773g."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_live_ans_takeover_is_j773g_scoped_and_composes_with_passed_off_path():
    source = (ROOT / "src/payload.c").read_text()
    assert "nvme_started = !boot_spec && nvme_post_m4_passed_off()" in source
    takeover = source[source.index("bool nvme_takeover") :]
    takeover = takeover[: takeover.index("goto boot_failed;")]
    assert "kboot_is_j773g_machine()" in takeover
    assert "nvme_post_m4_untouched()" in takeover
    assert "nvme_started || nvme_takeover" in takeover
