# SPDX-License-Identifier: MIT
"""The post-M4 ANS takeover is admitted by the J773g machine predicate only."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_ans_takeover_is_j773g_scoped():
    source = (ROOT / "src/payload.c").read_text()
    takeover = source[source.index("if (!boot_spec && kboot_is_j773g_machine()") :]
    takeover = takeover[: takeover.index("\n", takeover.index("goto boot_failed;"))]
    assert "kboot_is_j773g_machine()" in takeover
    assert "nvme_post_m4_untouched()" in takeover
    assert "chip_id == T8132" not in takeover
