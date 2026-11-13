# SPDX-License-Identifier: MIT
"""Locked DART initialization and Gen3 adoption split the DVA window identically."""

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]


def test_locked_vm_base_split_is_shared_and_correct(tmp_path):
    source = (ROOT / "src/dart.c").read_text()
    signature = "static u64 dart_split_locked_vm_base(u64 vm_base, u64 *dva_base)"
    start = source.index(signature)
    helper = source[start:source.index("\n}", start) + 2]
    assert source.count("dart_split_locked_vm_base(dart->vm_base, &dart->dva_base)") == 2
    harness = f"""
#include <assert.h>
#include <stdint.h>
typedef uint64_t u64;
#define GENMASK(h, l) (((~0ULL) << (l)) & (~0ULL >> (63 - (h))))
{helper}
int main(void)
{{
    u64 dva = 0;
    assert(dart_split_locked_vm_base(0x123456789abULL, &dva) == 0x3456789abULL);
    assert(dva == 0x12000000000ULL);
    assert(dart_split_locked_vm_base(0xfffffffffULL, &dva) == 0xfffffffffULL);
    assert(dva == 0);
    return 0;
}}
"""
    binary = tmp_path / "dart-locked-vm-base"
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-x", "c", "-", "-o", str(binary)], input=harness, text=True, check=True)
    subprocess.run([str(binary)], check=True)
