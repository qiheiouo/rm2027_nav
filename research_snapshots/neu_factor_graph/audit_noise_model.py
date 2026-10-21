#!/usr/bin/env python3
"""Source-backed covariance audit and decisive chart/correlation checks.

No localization benchmark or weight sweep. Native source remains read-only;
generated C++ using its headers and publication fragment stays under build/.
"""
import argparse
import copy
import json
import subprocess
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from backend import exp6, inv, log6, solve
from noise_model import (adjoint, native_mixed_to_right, relative_right_covariance,
                         sensor_to_base_right, sqrt_information)

PIN = "e7864a63a7e2a1ad62ec8fd75fcb9149db08e321"
ROOT = Path(__file__).resolve().parents[2]


def native_witness(source, output):
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if revision != PIN:
        raise ValueError("native audit requires the fixed V FAST-LIO gitlink")
    paths = ["src/laserMapping_bundle.cpp", "include/use-ikfom.hpp", "include/IKFoM_toolkit/mtk/types/SOn.hpp", "include/IKFoM_toolkit/mtk/types/vect.hpp"]
    subprocess.run(["git", "-C", str(source), "diff", "--exit-code", "HEAD", "--", *paths], check=True, capture_output=True)
    text = (source/paths[0]).read_text()
    start = text.index("pubOdomAftMapped->publish(odomAftMapped);", text.index("void publish_odometry("))
    end = text.index("\n    geometry_msgs::msg::TransformStamped trans;", start)
    # Only the audited publication order / loop are extracted, not rewritten.
    fragment = text[start:end]
    cpp = r'''
#include <array>
#include <iostream>
#include <memory>
#include <vector>
#include <omp.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "use-ikfom.hpp"
struct Message { struct Pose { std::array<double,36> covariance{}; } pose; };
struct Publisher {
  std::vector<Message> copies;
  void publish(const Message &message) { copies.push_back(message); }
};
struct Kalman { Eigen::Matrix<double,6,6> P;
  const Eigen::Matrix<double,6,6> &get_P() const { return P; }
} kf;
Message odomAftMapped;
auto pubOdomAftMapped = std::make_shared<Publisher>();
void publication_fragment() {
''' + fragment + r'''
}
void diagonal(const Message &m) {
  std::cout << '[';
  for(int i=0;i<6;++i) { if(i) std::cout << ','; std::cout << m.pose.covariance[7*i]; }
  std::cout << ']';
}
int main() {
  state_ikfom state;
  kf.P.setZero();
  kf.P.diagonal() << .09,.01,.04,.0001,.0004,.0009;
  publication_fragment();
  kf.P *= 10;
  publication_fragment();
  std::cout << "{\"native_pos_index\":" << state.pos.IDX
            << ",\"native_rot_index\":" << state.rot.IDX
            << ",\"first_published_diagonal\":";
  diagonal(pubOdomAftMapped->copies[0]);
  std::cout << ",\"second_published_diagonal\":";
  diagonal(pubOdomAftMapped->copies[1]);
  std::cout << ",\"current_unpublished_diagonal\":";
  diagonal(odomAftMapped);
  std::cout << '}';
}
'''
    code, executable = output/"native_covariance_witness.cpp", output/"native_covariance_witness"
    code.write_text(cpp)
    build = subprocess.run(["g++", "-std=c++17", "-O0", "-fopenmp", "-I/usr/include/eigen3", "-I"+str(source/"include"), str(code), "-o", str(executable)], capture_output=True, text=True, timeout=60)
    if build.returncode:
        raise RuntimeError(build.stderr[-4000:])
    witness = json.loads(subprocess.check_output([str(executable)], text=True))
    assert witness["native_pos_index"] == 0 and witness["native_rot_index"] == 3
    assert witness["first_published_diagonal"] == [0]*6
    assert np.allclose(witness["second_published_diagonal"], [.0001,.0004,.0009,.09,.01,.04])
    witness.update({"source_pin":revision, "source_paths":paths,
                    "scope":"actual native state header and publication fragment; copied-message stub, not ROS runtime or field replay"})
    return witness


def jacobian(function, dimension, step=1e-7):
    eye = np.eye(dimension)*step
    return np.column_stack([(function(e)-function(-e))/(2*step) for e in eye])


def heading_projection(T_raw_start, T_raw_sensor, E, center, heading_start, heading):
    """Algebra mirror of the fixed adapter's planar heading conversion only.

    Used to expose model rank, not as a new adapter or a hardware calibration.
    """
    current = E@inv(T_raw_start)@T_raw_sensor
    rb = Rotation.from_euler("z", heading-heading_start).as_matrix()
    relative = rb.T@current[:3,:3]@E[:3,:3].T
    yaw = np.arctan2(relative[1,0], relative[0,0])
    joint = Rotation.from_euler("z", yaw).as_matrix()
    B = np.eye(4)
    B[:3,:3] = rb
    B[:3,3] = current[:3,3]-rb@(center+joint@(E[:3,3]-center))
    return B


def algebra_audit():
    T = exp6(np.array([2.,-.5,.3,.1,-.2,.7]))
    E = exp6(np.array([.2,-.08,.14,.05,-.1,.2]))
    P = np.diag([.09,.01,.04,.0001,.0004,.0009])
    right = native_mixed_to_right(T, P)

    def native_error(x):
        perturbed = T.copy()
        perturbed[:3,3] += x[:3]
        perturbed[:3,:3] = T[:3,:3]@Rotation.from_rotvec(x[3:]).as_matrix()
        return log6(inv(T)@perturbed)

    native_j = jacobian(native_error, 6)
    B = T@inv(E)
    extrinsic_j = jacobian(lambda x: log6(inv(B)@(T@exp6(x)@inv(E))), 6)
    base_cov = sensor_to_base_right(E, right, extrinsic_conditioned=True)
    Z = exp6(np.array([.3,.1,-.02,.01,.03,.2]))
    relative_j = jacobian(lambda x: log6(inv(Z)@(inv(exp6(x[:6]))@Z@exp6(x[6:]))), 12)
    expected_j = np.column_stack([-adjoint(inv(Z)), np.eye(6)])
    Pi, Pj, cross = np.eye(6)*.01, np.eye(6)*.01, np.eye(6)*.0099
    independent = relative_right_covariance(np.eye(4), Pi, Pj, independent=True)
    correlated = relative_right_covariance(np.eye(4), Pi, Pj, cross)
    joint = np.block([[Pi,cross],[cross.T,Pj]])
    known_relative = relative_right_covariance(Z, Pi, Pj, cross)
    L = sqrt_information(known_relative)

    # Deterministic parent basis change cannot change a right-body chart.
    G = exp6(np.array([4.,-3.,.1,.2,.1,-.8]))
    Gp = np.eye(6); Gp[:3,:3] = G[:3,:3]
    changed_parent = native_mixed_to_right(G@T, Gp@P@Gp.T)
    zero_rotation = Rotation.from_euler("z", np.pi/2).as_matrix()
    center = np.array([.01,-.02,.1])
    Bheading = heading_projection(np.eye(4), T, E, center, .1, .7)
    Jh = jacobian(lambda x: log6(inv(Bheading)@heading_projection(np.eye(4), T@exp6(x[:6]), E, center, .1, .7+x[6])), 7)
    Ch = Jh@np.eye(7)@Jh.T
    rejected = []
    cases = {
        "unknown_cross_time":lambda:relative_right_covariance(Z,Pi,Pj),
        "unknown_extrinsic_uncertainty":lambda:sensor_to_base_right(E,right,extrinsic_conditioned=False),
        "zero_covariance_as_confidence":lambda:sqrt_information(np.zeros((6,6))),
        "heading_projected_singular_covariance":lambda:sqrt_information(Ch)}
    for name, call in cases.items():
        try: call()
        except ValueError: rejected.append(name)
    results = {
        "native_chart_covariance_fd_max_abs":float(np.max(abs(right-native_j@P@native_j.T))),
        "sensor_to_base_adjoint_fd_max_abs":float(np.max(abs(adjoint(E)-extrinsic_j))),
        "relative_jacobian_fd_max_abs":float(np.max(abs(expected_j-relative_j))),
        "relative_joint_covariance_fd_max_abs":float(np.max(abs(known_relative-relative_j@joint@relative_j.T))),
        "whitening_identity_max_abs":float(np.max(abs(L@known_relative@L.T-np.eye(6)))),
        "deterministic_parent_basis_invariance_max_abs":float(np.max(abs(right-changed_parent))),
        "same_absolute_marginals":np.diag(Pi).tolist(),
        "independent_increment_variances":np.diag(independent).tolist(),
        "correlated_increment_variances":np.diag(correlated).tolist(),
        "variance_ratio_with_identical_marginals":float(independent[0,0]/correlated[0,0]),
        "native_world_position_variances":np.diag(P[:3,:3]).tolist(),
        "zero_start_90deg_position_variances":np.diag(zero_rotation@P[:3,:3]@zero_rotation.T).tolist(),
        "heading_projection_output_rank":int(np.linalg.matrix_rank(Jh)),
        "heading_projection_roll_pitch_jacobian_max_abs":float(np.max(abs(Jh[3:5]))),
        "heading_chart_scope":"planar algebra mirror; no actual adapter execution or hardware inference",
        "conditional_base_covariance":base_cov.tolist(), "rejected_missing_models":rejected}
    for key in [k for k in results if k.endswith("max_abs")]:
        assert results[key] < 1e-7, (key, results[key])
    assert len(rejected)==4 and results["heading_projection_output_rank"]==4
    return results


def per_edge_check():
    data = json.loads((ROOT/"experiments/neu_factor_graph/evidence/drift_noise_input.json").read_text())
    original = solve(data)
    edge_data = copy.deepcopy(data)
    common = edge_data.pop("odom_sqrt_information")
    origin = edge_data.pop("odom_noise_model_origin")
    edge_data["odom_factors"] = [{"id":f"edge_{i}", "from_id":a["id"], "to_id":b["id"],
        "from_timestamp_ns":a["timestamp_ns"], "to_timestamp_ns":b["timestamp_ns"],
        "context":copy.deepcopy(data["context"]), "sqrt_information":common, "noise_model_origin":origin}
        for i,(a,b) in enumerate(zip(data["samples"],data["samples"][1:]))]
    result = solve(edge_data)
    delta = float(np.max(abs(np.array(original["T_map_base"])-np.array(result["T_map_base"]))))
    assert delta < 1e-8
    # Closed-form 3-node translational special case proves weights are applied
    # per edge; this is an algebra control, not localization tuning/benchmark.
    control = copy.deepcopy(edge_data)
    control["samples"] = control["samples"][:3]
    control["odom_factors"] = control["odom_factors"][:2]
    for i,sample in enumerate(control["samples"]):
        pose = np.eye(4); pose[0,3] = i
        sample["T_odom_base"] = pose.tolist()
    for i,edge in enumerate(control["odom_factors"]):
        edge["sqrt_information"] = np.diag([10 if i==0 else 1/.3,1000,1000,1000,1000,1000]).tolist()
        edge["noise_model_origin"] = "analytic 3-node weight-allocation control; not estimated noise"
    control["scan_factors"] = []
    for i in [0,2]:
        pose = np.eye(4); pose[0,3] = 0 if i==0 else 2.2
        control["scan_factors"].append({"id":f"anchor_{i}","query_id":f"anchor_query_{i}",
            "node_id":i,"timestamp_ns":control["samples"][i]["timestamp_ns"],
            "context":copy.deepcopy(control["context"]),"basin_status":"single_basin_assumed",
            "T_map_base":pose.tolist(),"sqrt_information":(np.eye(6)*1000).tolist(),
            "noise_model_origin":"analytic anchor variance 1e-6, not real pose certainty"})
    allocation = solve(control)
    values = np.array(allocation["T_map_base"])[:,0,3]
    total_variance = .01+.09+2e-6
    expected = np.array([.2*1e-6/total_variance,
                         1+.2*(1e-6+.01)/total_variance,2.2-.2*1e-6/total_variance])
    allocation_error = float(np.max(abs(values-expected)))
    assert allocation_error < 1e-7 and allocation["solver_success"]
    return {"common_vs_equal_per_edge_pose_max_abs":delta, "cost":result["cost"],
            "edge_count":len(edge_data["odom_factors"]), "active_accept":result["active_accept"],
            "nonuniform_closed_form_pose_max_abs_error":allocation_error,
            "nonuniform_increment_adjustments_m":(np.diff(values)-1).tolist(),
            "nonuniform_control_scope":"known 3-node linear special case, not trajectory performance"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fast-lio-source", type=Path, default=Path("/home/qihei/rm2027_navigation/src/fast_lio_multi"))
    parser.add_argument("--output", type=Path, default=ROOT/"build/neu_factor_graph/noise_audit")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    evidence = {"scope":"Task 1 source and noise-model math; no weight tuning, no new real data",
                "native_source_witness":native_witness(args.fast_lio_source.resolve(), args.output),
                "covariance_algebra":algebra_audit(), "private_per_edge_input":per_edge_check()}
    path = args.output/"noise_audit.json"
    path.write_text(json.dumps(evidence, indent=2, allow_nan=False)+"\n")
    print(json.dumps(evidence, indent=2, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
