#!/usr/bin/env python3
"""Decisive full-state controls, not localization accuracy or weight tuning.

Compiles the unchanged V canonical conversion function into a private file
probe. All poses/calibrations here are declared mathematical fixtures, not
new-car measurements. Only the compact audit result is a long-term asset.
"""
import argparse
import json
import subprocess
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from backend import exp6, inv, log6
from base_state import (from_heading, from_measured_joint, moving_extrinsic,
                        normalized_sensor, yaw_rotation)
from audit_noise_model import heading_projection, jacobian
from noise_model import adjoint

ROOT = Path(__file__).resolve().parents[2]
V_PIN = "bd6cf689d3d5aca5c8edd430ff760ca5ceb27496"


def pose(p, rpy):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = p
    return T


def heading(R):
    if np.linalg.norm(R[:2, 0]) < 1e-12:
        raise ValueError("base forward axis vertical; heading is undefined")
    return float(np.arctan2(R[1, 0], R[0, 0]))


def error(estimated, truth):
    return {"position_error_m": float(np.linalg.norm(estimated[:3, 3]-truth[:3, 3])),
            "rotation_error_deg": float(np.rad2deg(np.linalg.norm(
                log6(inv(truth) @ estimated)[3:])))}


def actual_baseline(cases, output):
    paths = ["src/rm_localization_adapters/src/canonical_odometry.cpp",
             "src/rm_localization_adapters/include/rm_localization_adapters/canonical_odometry.hpp"]
    subprocess.run(["git", "-C", str(ROOT), "diff", "--exit-code", V_PIN, "--", *paths],
                   check=True, capture_output=True)
    code = r'''
#include <iomanip>
#include <iostream>
#include "rm_localization_adapters/canonical_odometry.hpp"
tf2::Transform read_transform() {
  double a[16]; for(double &v:a) std::cin >> v;
  return tf2::Transform(tf2::Matrix3x3(a[0],a[1],a[2],a[4],a[5],a[6],a[8],a[9],a[10]),
                        tf2::Vector3(a[3],a[7],a[11]));
}
void write_transform(const tf2::Transform &T) {
  for(int i=0;i<4;++i) for(int j=0;j<4;++j) {
    if(i||j) std::cout << ',';
    std::cout << (i==3 ? (j==3 ? 1.0 : 0.0) : (j==3 ? T.getOrigin()[i] : T.getBasis()[i][j]));
  }
}
int main() {
  int n; std::cin >> n; std::cout << std::setprecision(17) << '[';
  for(int i=0;i<n;++i) {
    auto start=read_transform(), current=read_transform(), home=read_transform();
    double x,y,z,h0,h; std::cin >> x >> y >> z >> h0 >> h;
    auto result=rm_localization_adapters::compute_base_transform_from_chassis_heading(
      start,current,home,tf2::Vector3(x,y,z),h0,h);
    if(i) std::cout << ',';
    std::cout << "{\"pose\":["; write_transform(result.base_initial_to_base);
    std::cout << "],\"gimbal_delta_rad\":" << result.gimbal_yaw_rad << '}';
  }
  std::cout << ']';
}
'''
    output.mkdir(parents=True, exist_ok=True)
    cpp, executable = output / "canonical_state_witness.cpp", output / "canonical_state_witness"
    cpp.write_text(code)
    # Existing ROS headers only. Linking a mathematical function does not run ROS.
    includes = [ROOT / "src/rm_localization_adapters/include"]
    includes += sorted(p for p in Path("/opt/ros/jazzy/include").iterdir() if p.is_dir())
    cmd = ["g++", "-std=c++17", "-O0", *["-I"+str(p) for p in includes],
           str(cpp), str(ROOT / paths[0]), "-o", str(executable)]
    subprocess.run(cmd, check=True, capture_output=True, text=True, timeout=60)
    rows = [str(len(cases))]
    for c in cases:
        values = np.r_[c["raw_start"].ravel(), c["raw_current"].ravel(), c["home"].ravel(),
                       c["center"], c["heading_start"], c["heading"]]
        rows.append(" ".join(format(v, ".17g") for v in values))
    result = subprocess.run([str(executable)], input="\n".join(rows), check=True,
                            capture_output=True, text=True, timeout=10)
    return json.loads(result.stdout)


def make_case(name, rpy_start, rpy_current, p, gamma):
    # Reuse the existing canonical heading unit-test mounting geometry. This
    # is explicitly *not* CAD A/B or a claimed sensor/IMU calibration.
    home = pose([.18, -.11, .35], [.25, -.2, .6])
    center = np.array([.03, -.02, .08])
    raw_start = pose([-.3, .4, .2], [.17, -.23, .51])
    H0 = pose([0, 0, 0], rpy_start)
    H1 = pose([0, 0, 0], rpy_current)
    B = inv(H0) @ H1
    B[:3, 3] = p
    A = B @ moving_extrinsic(home, center, gamma)
    raw_current = raw_start @ inv(home) @ A
    return {"name": name, "home": home, "center": center,
            "raw_start": raw_start, "raw_current": raw_current, "truth": B,
            "heading_start": heading(H0[:3, :3]), "heading": heading(H1[:3, :3]),
            "R_heading_base0": H0[:3, :3], "true_gimbal_delta_rad": gamma}


def recover(c, *, raw=None, h=None, reference=None):
    return from_heading(c["raw_start"], c["raw_current"] if raw is None else raw,
                        c["home"], c["center"], c["heading"] if h is None else h,
                        R_heading_base0=c["R_heading_base0"] if reference is None else reference)


def run(output):
    # Fixed cases before results. No noise/weight/grid search.
    cases = [
        make_case("flat_gimbal_heading_wrap", [0, 0, 2.9], [0, 0, 3.7], [1.2, -.7, 0], -1.1),
        make_case("flat_pi_boundary", [0, 0, 3.10], [0, 0, -3.10], [.1, .2, 0], 2.8),
        make_case("tilt_from_level_start", [0, 0, .3], [.3, -.25, 1.1], [1.2, -.7, .2], -1.1),
        make_case("tilted_start_home", [.2, -.18, 2.9], [.2, -.18, 2.9], [0, 0, 0], 0),
        make_case("tilted_start_motion", [.2, -.18, 2.9], [.32, -.12, -2.6], [.7, -.2, .15], .9),
        make_case("sideways_joint_ambiguity", [0, 0, 0], [np.pi/2, 0, .4], [.7, -.2, .15], .6)]
    native = actual_baseline(cases, output.parent / "base_state_witness")
    rows = []
    parity = 0.
    for c, old in zip(cases, native):
        old_pose = np.array(old["pose"]).reshape(4, 4)
        mirror = heading_projection(c["raw_start"], c["raw_current"], c["home"], c["center"],
                                    c["heading_start"], c["heading"])
        parity = max(parity, float(np.max(abs(old_pose-mirror))))
        proposed = recover(c)
        measured = from_measured_joint(c["raw_start"], c["raw_current"], c["home"], c["center"],
                                       c["true_gimbal_delta_rad"])
        row = {"case": c["name"], "actual_planar_adapter": error(old_pose, c["truth"]),
               "measured_joint_geometry_control": error(measured, c["truth"]),
               "heading_candidate_status": proposed["status"],
               "heading_constraint_norm": proposed["heading_constraint_norm"],
               "heading_candidate": None, "active_accept": False}
        if proposed["T_base0_base"] is not None:
            row["heading_candidate"] = error(proposed["T_base0_base"], c["truth"])
            assert row["heading_candidate"]["position_error_m"] < 1e-9
            assert row["heading_candidate"]["rotation_error_deg"] < 1e-7
        rows.append(row)
        assert row["measured_joint_geometry_control"]["position_error_m"] < 1e-9
        assert row["measured_joint_geometry_control"]["rotation_error_deg"] < 1e-7
    assert parity < 1e-10
    assert rows[-1]["heading_candidate_status"] == "ambiguous_joint_decomposition"
    for row in rows[:2]:
        assert row["actual_planar_adapter"]["position_error_m"] < 1e-9
        assert row["actual_planar_adapter"]["rotation_error_deg"] < 1e-7

    # Starting at home cannot validate a level assumption: both return I at
    # start, then the incorrect gravity/heading relationship biases the motion.
    tilted = cases[4]
    wrong_reference = recover(tilted, reference=yaw_rotation(tilted["heading_start"]))
    startup_counterexample = error(wrong_reference["T_base0_base"], tilted["truth"])
    startup_counterexample["status_despite_wrong_reference"] = wrong_reference["status"]
    try:
        from_heading(tilted["raw_start"], tilted["raw_current"], tilted["home"], tilted["center"], tilted["heading"])
    except ValueError:
        unknown_start_rejected = True
    else:
        unknown_start_rejected = False
    assert unknown_start_rejected and startup_counterexample["rotation_error_deg"] > 1.

    # Two different chassis/joint decompositions explain EXACTLY the same
    # sensor transform and directed heading at sideways roll. No GT selection.
    singular = cases[-1]
    A = normalized_sensor(singular["raw_start"], singular["raw_current"], singular["home"])
    alternatives = [A @ inv(moving_extrinsic(singular["home"], singular["center"], g)) for g in [.6, 1.]]
    continuum = {"example_joint_angles_rad": [.6, 1.],
                 "base_pose_difference": error(alternatives[1], alternatives[0]),
                 "heading_difference_rad": abs(heading(alternatives[1][:3, :3])-heading(alternatives[0][:3, :3])),
                 "sensor_reconstruction_max_abs": max(float(np.max(abs(B @ moving_extrinsic(
                     singular["home"], singular["center"], g)-A))) for B, g in zip(alternatives, [.6, 1.]))}
    assert continuum["heading_difference_rad"] < 1e-10 and continuum["sensor_reconstruction_max_abs"] < 1e-10

    # Conditional local derivative: raw current pose right error + heading.
    # No real covariance is provided; don't substitute independent joint noise.
    normal = cases[2]
    B = recover(normal)["T_base0_base"]
    J = jacobian(lambda x: log6(inv(B) @ recover(normal,
        raw=normal["raw_current"] @ exp6(x[:6]), h=normal["heading"]+x[6])["T_base0_base"]), 7)
    old = heading_projection(normal["raw_start"], normal["raw_current"], normal["home"], normal["center"],
                             normal["heading_start"], normal["heading"])
    J_old = jacobian(lambda x: log6(inv(old) @ heading_projection(normal["raw_start"],
        normal["raw_current"] @ exp6(x[:6]), normal["home"], normal["center"],
        normal["heading_start"], normal["heading"]+x[6])), 7)
    E_current = moving_extrinsic(normal["home"], normal["center"], normal["true_gimbal_delta_rad"])
    fixed_joint_J = jacobian(lambda x: log6(inv(B) @ (
        normalized_sensor(normal["raw_start"], normal["raw_current"] @ exp6(x), normal["home"]) @ inv(E_current))), 6)
    # A physical yaw-joint-only motion changes the raw sensor pose, not base.
    # Its right sensor tangent must lie in the conditional base J's nullspace.
    axis = np.array([0., 0., 1.])
    joint_tangent = adjoint(inv(E_current)) @ np.r_[-np.cross(axis, normal["center"]), axis]
    joint_null_error = float(np.max(abs(J[:, :6] @ joint_tangent)))
    # Arbitrary deterministic raw parent basis and heading-world yaw origin
    # must not change the recovered B0 pose when transformed consistently.
    G = pose([2., -.4, .7], [.2, -.3, .8])
    changed_raw = from_heading(G @ normal["raw_start"], G @ normal["raw_current"],
        normal["home"], normal["center"], normal["heading"], R_heading_base0=normal["R_heading_base0"])
    changed_heading = recover(normal, h=normal["heading"]+.7,
                              reference=yaw_rotation(.7) @ normal["R_heading_base0"])
    basis_error = max(float(np.max(abs(changed_raw["T_base0_base"]-B))),
                      float(np.max(abs(changed_heading["T_base0_base"]-B))))
    derivatives = {"conditional_full_base_rank": int(np.linalg.matrix_rank(J)),
                   "actual_planar_mirror_rank": int(np.linalg.matrix_rank(J_old)),
                   "full_base_J_right_sensor_plus_heading": J.tolist(),
                   "holding_derived_joint_fixed_J_max_difference": float(np.max(abs(J[:, :6]-fixed_joint_J))),
                   "joint_only_motion_base_J_null_max_abs": joint_null_error,
                   "consistent_raw_parent_or_heading_yaw_origin_max_abs": basis_error,
                   "conditioning_inputs": "known home/center/startup reference, fixed raw startup; their uncertainty and cross-time terms are not zero facts",
                   "covariance": None}
    assert derivatives["conditional_full_base_rank"] == 6
    assert derivatives["actual_planar_mirror_rank"] == 4
    assert derivatives["holding_derived_joint_fixed_J_max_difference"] > .1
    assert joint_null_error < 1e-7 and basis_error < 1e-10

    # Two predeclared angles illustrate sensitivity, not a threshold sweep.
    sensitivities = []
    for roll in (0., np.deg2rad(89.)):
        c = make_case("heading_sensitivity", [0, 0, 0], [roll, 0, .4], [.7, -.2, .15], .6)
        delta = 1e-7
        lo, hi = recover(c, h=c["heading"]-delta), recover(c, h=c["heading"]+delta)
        sensitivity = np.linalg.norm(log6(inv(lo["T_base0_base"]) @ hi["T_base0_base"])[3:])/(2*delta)
        sensitivities.append({"roll_deg": float(np.rad2deg(roll)),
                              "heading_constraint_norm": recover(c)["heading_constraint_norm"],
                              "attitude_response_rad_per_heading_rad": float(sensitivity),
                              "pitch_zero_analytic_response": float(1/np.cos(roll))})
        assert abs(sensitivity-1/np.cos(roll)) < 1e-5
    assert sensitivities[1]["attitude_response_rad_per_heading_rad"] > 50.

    # An uncompensated 20 ms error at 6 rad/s: algebra alone cannot see a
    # heading clock/rate error. Existing nearestHeading does rate propagation;
    # this control is not an allegation that it uses stale heading unchanged.
    shifted = recover(cases[0], h=cases[0]["heading"]-.020*6.)
    time_counterexample = error(shifted["T_base0_base"], cases[0]["truth"])
    time_counterexample.update({"uncompensated_time_offset_s": .020, "heading_rate_rad_s": 6.,
                                "status": shifted["status"], "existing_rate_propagation_not_replayed": True})
    result = {"source_pin": V_PIN, "scope": "synthetic rigid-chain controls; actual unchanged canonical C++ function, no ROS runtime/new-car data/graph precision claim",
              "fixture_origin": "existing test_canonical_odometry.cpp heading-fusion home/center; not real A/B extrinsics",
              "fixture_home": cases[0]["home"].tolist(), "fixture_center": cases[0]["center"].tolist(),
              "actual_cpp_vs_planar_mirror_max_abs": parity, "cases": rows,
              "wrong_level_startup_reference": startup_counterexample, "unknown_startup_reference_rejected": unknown_start_rejected,
              "singular_continuum_witness": continuum, "conditional_jacobian": derivatives,
              "heading_sensitivity": sensitivities, "timing_counterexample": time_counterexample,
              "real_episodes": 0, "decision": "Modify", "graph_weights_changed": False, "active_accept": False}
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({"output": str(output), "cases": rows, "wrong_level_startup_reference": startup_counterexample,
                      "heading_sensitivity": sensitivities, "decision": "Modify"}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "build/neu_factor_graph/base_state_audit.json")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    run(args.output)
