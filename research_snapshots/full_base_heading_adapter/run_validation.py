#!/usr/bin/env python3
"""Repeatable offline validation and compact evidence; never initializes submodules."""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from adapter import Policy, Status, extrinsic, solve
from fixtures import cases, make_case, pose_error
from native import BASE_PIN, BUILD, ROOT, build, command, planar
from test_adapter import recover

FAST_PIN = "e7864a63a7e2a1ad62ec8fd75fcb9149db08e321"


def source_audit(fast_source):
    # Read pinned Git blobs even if another task changes its checkout later.
    paths = ["src/rm_localization_adapters/src/canonical_odometry.cpp",
             "src/rm_localization_adapters/src/lio_adapter.cpp",
             "src/rm_lio_bringup/launch/fast_lio_multi_phase2a.launch.py",
             "src/rm_lio_bringup/config/fast_lio_multi_single_mid360.yaml"]
    hashes = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths}
    command(["git", "-C", str(ROOT), "diff", "--exit-code", BASE_PIN, "--", *paths])
    pin = command(["git", "-C", str(ROOT), "ls-tree", "HEAD", "src/fast_lio_multi"]).stdout.split()[2]
    if pin != FAST_PIN:
        raise RuntimeError("unexpected FAST-LIO submodule pin")
    blob = command(["git", "-C", str(fast_source), "show",
                    FAST_PIN + ":src/laserMapping_bundle.cpp"]).stdout
    required = ["odomAftMapped.child_frame_id = \"body\";",
                "odomAftMapped.header.stamp = get_ros_time(publish_lidar_time);",
                "T_world_to_zero = T_WB.inverse();",
                "T_ZB = T_world_to_zero * T_WB;",
                "T_WB.block<3, 3>(0, 0) = state_point.rot.toRotationMatrix();",
                "T_WB.block<3, 1>(0, 3) = state_point.pos;"]
    if not all(token in blob for token in required):
        raise RuntimeError("pinned publication source changed")
    local_submodule = command(["git", "-C", str(ROOT), "submodule", "status"]).stdout.strip().splitlines()
    # No branch/checkout mutation. Source checkout state is provenance only.
    fast_head = command(["git", "-C", str(fast_source), "rev-parse", "HEAD"]).stdout.strip()
    fast_branch = command(["git", "-C", str(fast_source), "branch", "--show-current"]).stdout.strip()
    fast_dirty = command(["git", "-C", str(fast_source), "status", "--porcelain"]).stdout.strip()
    return dict(base_pin=BASE_PIN, fast_lio_pin=FAST_PIN, production_source_sha256=hashes,
                pinned_bundle_source_sha256=hashlib.sha256(blob.encode()).hexdigest(),
                local_submodules=local_submodule,
                fast_source_checkout=dict(branch=fast_branch or "DETACHED", commit=fast_head,
                                          dirty=bool(fast_dirty), access="read-only pinned git show"),
                design_reference=dict(commit="0a96ea4a5f4c32b8f99540634d2e02d87c28fe6e",
                                      path="experiments/neu_factor_graph/base_state.py",
                                      usage="mathematical reference only; no imports/cherry-pick/merge"))


def result_row(c, result, baseline):
    row = dict(case=c["name"], status=result.status.value, reason=result.reason,
               planar=pose_error(np.array(baseline["pose"]).reshape(4, 4), c["truth_base0"]),
               full=None, diagnostics=result.diagnostics)
    if result.status == Status.SOLVED:
        row["full"] = pose_error(result.T_odom_base, c["truth_odom"])
    known = recover(c, joint=c["joint"])
    row["known_joint_status"] = known.status.value
    row["known_joint"] = pose_error(known.T_odom_base, c["truth_odom"])
    return row


def run(fast_source, output):
    _, gtest = build()
    gtest_output = command([str(gtest)]).stdout
    (BUILD / "canonical_tests.log").write_text(gtest_output)
    if "[  PASSED  ]" not in gtest_output:
        raise RuntimeError("canonical baseline tests were not run")
    suite = unittest.defaultTestLoader.discover(str(Path(__file__).parent), pattern="test_*.py")
    with (BUILD / "unit_tests.log").open("w") as log:
        test_result = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
    if not test_result.wasSuccessful():
        raise RuntimeError("candidate tests failed; see build/full_base_heading_adapter/unit_tests.log")
    data = cases()
    native = planar(data)
    rows = [result_row(c, recover(c), b) for c, b in zip(data, native)]
    near = data[6]
    loose = Policy(min_constraint_derivative=1e-6, max_heading_gain=1e6)
    nominal = recover(near, policy=loose)
    perturbed = dict(near, heading=replace(near["heading"], yaw_rad=near["heading"].yaw_rad + 1e-7))
    response = recover(perturbed, policy=loose)
    response_gain = np.linalg.norm(Rotation.from_matrix(
        nominal.T_odom_base[:3, :3].T @ response.T_odom_base[:3, :3]).as_rotvec()) / 1e-7

    stale = dict(data[2], heading=replace(data[2]["heading"],
                                        yaw_rad=data[2]["heading"].yaw_rad - .02*6.,
                                        stamp_ns=data[2]["raw"].stamp_ns - 20_000_000))
    rejected = recover(stale)
    falsely_stamped = dict(stale, heading=replace(stale["heading"], stamp_ns=stale["raw"].stamp_ns))
    incorrect = recover(falsely_stamped)

    dynamic = make_case(start_rpy=(0, 0, 0), rpy=(0, 0, 0), position=(0, 0, 0), delta=.6)
    helper = np.array(planar([dynamic])[0]["sensor_initial_dynamic_pose"]).reshape(4, 4)
    # Additional independently measured joint resolves the two-root witness.
    ambiguous = recover(data[7])
    candidate_poses = [np.array(x["T_odom_base"]) for x in ambiguous.diagnostics["candidates"]]
    sensor_residual = max(float(np.max(abs(
        T @ extrinsic(data[7]["calibration"], data[7]["calibration"].home_angle_rad + candidate["joint_delta_rad"]) -
        data[7]["truth_odom"] @ extrinsic(data[7]["calibration"], data[7]["joint"].angle_rad))))
        for T, candidate in zip(candidate_poses, ambiguous.diagnostics["candidates"]))
    report = dict(scope="synthetic offline rigid-chain/source audit; no ROS runtime or real episodes",
                  provenance=source_audit(fast_source),
                  unit_tests=dict(run=test_result.testsRun, failures=len(test_result.failures),
                                  errors=len(test_result.errors)),
                  canonical_gtest_summary=[line for line in gtest_output.splitlines()
                                           if "[  PASSED  ]" in line],
                  cases=rows,
                  sensitivity=dict(roll_deg=89., perturbation_rad=1e-7,
                                   measured_rotation_gain_rad_per_rad=float(response_gain),
                                   analytic_gain=float(1/np.cos(np.deg2rad(89))),
                                   default_status=recover(near).status.value,
                                   thresholds="research demonstration only"),
                  timing=dict(offset_ms=20., heading_rate_rad_s=6.,
                              correctly_stamped_status=rejected.status.value,
                              falsely_retimestamped_status=incorrect.status.value,
                              falsely_retimestamped_error=pose_error(incorrect.T_odom_base, stale["truth_odom"]),
                              existing_planar_rate_propagation="source audited; not replayed as runtime"),
                  dynamic_sensor_initial_helper=dict(pure_gimbal_delta_rad=.6,
                                                      error=pose_error(helper, dynamic["truth_base0"])),
                  tilted_axis_two_root_witness=dict(pose_difference=pose_error(*candidate_poses),
                                                    max_sensor_reconstruction_abs=sensor_residual),
                  real_episodes=0, active_accept=False, covariance=None,
                  decision="mathematics/interface conditional Go; hardware No-Go pending measurement")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(dict(unit_tests=report["unit_tests"], canonical_tests=report["canonical_gtest_summary"],
                          statuses={row["case"]: row["status"] for row in rows}, output=str(output)), indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fast-lio-source", type=Path, required=True,
                        help="read-only Git checkout containing the fixed FAST-LIO commit")
    parser.add_argument("--output", type=Path, default=BUILD / "summary.json")
    args = parser.parse_args()
    run(args.fast_lio_source.resolve(), args.output.resolve())
