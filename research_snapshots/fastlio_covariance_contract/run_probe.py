#!/usr/bin/env python3
"""Compile real native headers and extracted publisher bodies; no DDS process.

Only the publisher/broadcaster transport and time conversion are test doubles.
Generated code and binaries are restricted to this worktree's build directory.
"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "src/fast_lio_multi"
OLD = "e7864a63a7e2a1ad62ec8fd75fcb9149db08e321"
MODES = ("bundle", "adaptive", "async")


def git(*args):
    return subprocess.check_output(["git", "-C", str(SOURCE), *args], text=True)


def body(text, signature):
    start = text.index(signature)
    opening = text.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[opening + 1:end - 1]


def fixtures(revision):
    output = []
    hashes = {}
    for mode in MODES:
        path = f"src/laserMapping_{mode}.cpp"
        text = git("show", f"{OLD}:{path}") if revision == "old" else (SOURCE/path).read_text()
        hashes[path] = hashlib.sha256(text.encode()).hexdigest()
        # Anchor after publish_map, excluding the commented-out bundle template.
        pose = body(text[text.index("void publish_map("):], "\nvoid set_posestamp(")
        publish = body(text, "void publish_odometry(")
        output.append(f"""
namespace {revision}_{mode} {{
nav_msgs::msg::Odometry odomAftMapped;
template<typename T> void set_posestamp(T &out) {{ {pose} }}
void publish_odometry(const std::shared_ptr<CapturePublisher> &pubOdomAftMapped,
                     std::unique_ptr<CaptureBroadcaster> &tf_br) {{ {publish} }}
}} // namespace {revision}_{mode}
""")
        if mode == "bundle":
            start = text.index("Eigen::Matrix4d T_WB = Eigen::Matrix4d::Identity();")
            end = text.index("pub_quat.normalize();", start) + len("pub_quat.normalize();")
            output.append(f"void {revision}_zero_pose_update() {{\n{text[start:end]}\n}}\n")
    return "".join(output), hashes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-only", action="store_true")
    parser.add_argument("--ros-prefix", type=Path, default=Path("/opt/ros/jazzy"))
    args = parser.parse_args()
    build = ROOT / "build/fastlio_covariance_contract_probe"
    build.mkdir(parents=True, exist_ok=True)
    old, old_hashes = fixtures("old")
    (build/"old_publishers.inc").write_text(old)
    hashes = {"old": old_hashes}
    if not args.baseline_only:
        new, hashes["candidate"] = fixtures("candidate")
        (build/"candidate_publishers.inc").write_text(new)
    includes = ["/usr/include/eigen3", str(SOURCE/"include"), str(build)]
    includes += [str(p) for p in sorted((args.ros_prefix/"include").iterdir()) if p.is_dir()]
    command = ["g++", "-std=c++17", "-O0", "-fopenmp", "-fmax-errors=4", "-Wno-deprecated-declarations"]
    if not args.baseline_only:
        command += ["-DTEST_CANDIDATE"]
    command += ["-I" + p for p in includes]
    executable = build / ("baseline_probe" if args.baseline_only else "contract_probe")
    command += [str(Path(__file__).with_name("contract_probe.cpp")), "-o", str(executable)]
    compiled = subprocess.run(command, timeout=90, capture_output=True, text=True)
    (build/"compile.log").write_text(compiled.stdout + compiled.stderr)
    if compiled.returncode:
        raise RuntimeError(compiled.stderr[-4000:])
    result = json.loads(subprocess.check_output([str(executable)], text=True))
    if not args.baseline_only:
        # The production helper stays compatible with the submodule's C++14.
        # The ROS Jazzy comparison headers used by the full probe need C++17.
        compatibility = build/"helper_cxx14.cpp"
        compatibility.write_text('''#include <omp.h>
#include "use-ikfom.hpp"
#include "pose_covariance.hpp"
void check() {
  esekfom::esekf<state_ikfom,12,input_ikfom> filter;
  auto output = fast_lio::native_pose_covariance(
    filter.get_P().block<6,6>(0,0), Eigen::Matrix3d::Identity(),
    filter.get_x().rot.toRotationMatrix());
  (void)output;
}
''')
        subprocess.run(["g++", "-std=c++14", "-fopenmp", "-fsyntax-only",
                        "-I/usr/include/eigen3", "-I"+str(SOURCE/"include"),
                        str(compatibility)], check=True, capture_output=True, text=True, timeout=60)
        result["candidate"]["helper_cxx14_syntax"] = "passed"
    result["provenance"] = {
        "superproject_base": "bd6cf689d3d5aca5c8edd430ff760ca5ceb27496",
        "submodule_old_commit": OLD,
        "submodule_head_at_run": git("rev-parse", "HEAD").strip(),
        "source_sha256": hashes,
        "native_header_sha256": hashlib.sha256((SOURCE/"include/use-ikfom.hpp").read_bytes()).hexdigest(),
        "ros_pose_message_sha256": hashlib.sha256((args.ros_prefix/"share/geometry_msgs/msg/PoseWithCovariance.msg").read_bytes()).hexdigest(),
        "ros_tf2_header_sha256": hashlib.sha256((args.ros_prefix/"include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp").read_bytes()).hexdigest(),
        "native_headers_unchanged_from_old": not bool(git("diff", OLD, "--", "include/use-ikfom.hpp", "include/IKFoM_toolkit").strip()),
        "compiler": subprocess.check_output(["g++", "--version"], text=True).splitlines()[0],
        "ros_prefix": str(args.ros_prefix),
        "scope": "real state/ESKF/message/tf2 headers and extracted publisher/zero-start bodies; copied-message transport stubs; no ROS executor or field replay",
    }
    if not args.baseline_only:
        result["provenance"]["candidate_helper_sha256"] = hashlib.sha256((SOURCE/"include/pose_covariance.hpp").read_bytes()).hexdigest()
    evidence = ROOT / "experiments/fastlio_covariance_contract/evidence"
    evidence.mkdir(exist_ok=True)
    destination = evidence / ("baseline.json" if args.baseline_only else "contract.json")
    destination.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (subprocess.CalledProcessError, RuntimeError) as error:
        raise SystemExit(str(error))
