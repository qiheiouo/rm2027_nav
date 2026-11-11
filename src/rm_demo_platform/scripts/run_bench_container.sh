#!/usr/bin/env bash
set -euo pipefail
action=${1:-check}
case "$action" in check|run) ;; *) echo 'Usage: run_bench_container.sh [check|run]' >&2; exit 2;; esac
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
results="$repo/build/demo_validation"
mkdir -p "$results"
# Deliberately no host networking, shared IPC, devices, privileged mode or old overlay.
exec docker run --rm --pull never --network none --ipc private \
  --cap-drop ALL --security-opt no-new-privileges --user "$(id -u):$(id -g)" \
  --cpus 2 --memory 3g \
  -e ROS_DOMAIN_ID=137 -e ROS_LOCALHOST_ONLY=1 \
  -e ROS_LOG_DIR=/results/ros_logs -e PYTHONDONTWRITEBYTECODE=1 \
  -e CMAKE_BUILD_PARALLEL_LEVEL=2 \
  -e RM_DEMO_ISOLATED_BENCH=1 \
  --mount "type=bind,src=$repo,dst=/source,readonly" \
  --mount "type=bind,src=$results,dst=/results" \
  --workdir /results rm2027_navigation:humble bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    colcon --log-base /results/colcon_logs build \
      --base-paths /source/src/rm_competition_interfaces /source/src/rm_description \
        /source/src/rm_localization_adapters /source/src/rm_serial_driver /source/src/rm_demo_platform \
      --build-base /results/build --install-base /results/install \
      --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
    source /results/install/setup.bash
    if [ "$1" = check ]; then
      colcon --log-base /results/test_logs test --build-base /results/build \
        --base-paths /source/src/rm_localization_adapters /source/src/rm_serial_driver \
        --packages-select rm_localization_adapters rm_serial_driver \
        --ctest-args -R "^test_(canonical_odometry|protocol|competition_v2_protocol|competition_v2_reference)$" --output-on-failure
      colcon test-result --test-result-base /results/build --verbose
      ros2 run rm_demo_platform check_bench.py
    else
      exec ros2 launch rm_demo_platform demo_bench.launch.py enable_test:=true
    fi
  ' demo-bench "$action"
