"""Build only local mathematical probes, with existing ROS headers/gtest."""
from functools import lru_cache
import json
from pathlib import Path
import subprocess

import numpy as np

from adapter import extrinsic

ROOT = Path(__file__).resolve().parents[2]
BASE_PIN = "bd6cf689d3d5aca5c8edd430ff760ca5ceb27496"
BUILD = ROOT / "build/full_base_heading_adapter"


def command(args, **kwargs):
    return subprocess.run(args, check=True, capture_output=True, text=True,
                          timeout=60, **kwargs)


@lru_cache(maxsize=1)
def build():
    paths = ["src/rm_localization_adapters/src/canonical_odometry.cpp",
             "src/rm_localization_adapters/include/rm_localization_adapters/canonical_odometry.hpp",
             "src/rm_localization_adapters/test/test_canonical_odometry.cpp"]
    command(["git", "-C", str(ROOT), "diff", "--exit-code", BASE_PIN, "--", *paths])
    BUILD.mkdir(parents=True, exist_ok=True)
    includes = [ROOT / "src/rm_localization_adapters/include"]
    includes += sorted(p for p in Path("/opt/ros/jazzy/include").iterdir() if p.is_dir())
    flags = ["g++", "-std=c++17", "-O0", "-Wall", "-Wextra", "-Wpedantic",
             *["-I" + str(p) for p in includes]]
    executable = BUILD / "planar_probe"
    command([*flags, str(Path(__file__).with_name("native_probe.cpp")),
             str(ROOT / paths[0]), "-o", str(executable)])
    gtest = BUILD / "canonical_tests"
    command([*flags, str(ROOT / paths[2]), str(ROOT / paths[0]),
             "-lgtest_main", "-lgtest", "-pthread", "-o", str(gtest)])
    return executable, gtest


def planar(cases):
    executable, _ = build()
    rows = [str(len(cases))]
    for c in cases:
        values = np.r_[c["startup"].raw.T_raw_sensor.ravel(),
                       c["raw"].T_raw_sensor.ravel(), c["home"].ravel(),
                       extrinsic(c["calibration"], c["joint"].angle_rad).ravel(), c["center"],
                       c["startup"].heading.yaw_rad, c["heading"].yaw_rad]
        rows.append(" ".join(format(v, ".17g") for v in values))
    return json.loads(command([str(executable)], input="\n".join(rows)).stdout)
