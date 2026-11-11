#!/usr/bin/env bash
# Explicit hardware acquisition in the existing locally built Humble environment.
set -euo pipefail
stage=${1:-inventory}
case "$stage" in inventory|serial|lidar|tf|fixed_lio) ;; *) echo 'Stage: inventory|serial|lidar|tf|fixed_lio' >&2; exit 2;; esac
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
results="$repo/build/demo_validation"
image=rm2027_navigation:humble
options=(--network none)
args=(stage:=inventory)
if [ "$stage" != inventory ]; then
  if [ "${RM_DEMO_ENABLE_HARDWARE_IO:-}" != 1 ]; then
    echo 'Set RM_DEMO_ENABLE_HARDWARE_IO=1 only for confirmed, physically inhibited hardware acquisition.' >&2
    exit 2
  fi
  session=$(realpath -e -- "${2:?Absolute prepared session directory required}")
  # Read and validate before Docker/device creation. No ROS or IO from preflight.
  export PYTHONPATH="$repo/src/rm_demo_platform${PYTHONPATH:+:$PYTHONPATH}"
  python3 "$repo/src/rm_demo_platform/scripts/demo_phase1.py" preflight --config-dir "$session/config" --stage "$stage"
  image=$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["demo"].get("container_image") or "")' "$session/session.yaml")
  if [ -z "$image" ]; then echo 'session.demo.container_image required' >&2; exit 2; fi
  if [ -z "${ROS_DOMAIN_ID:-}" ]; then echo 'Explicit isolated ROS_DOMAIN_ID required' >&2; exit 2; fi
  options=(--network host --mount "type=bind,src=$session,dst=$session")
  if [ "$stage" != lidar ]; then
    device=$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["parameters"]["device"])' "$session/config/demo_serial.yaml")
    if [ ! -c "$device" ]; then echo 'Confirmed serial device must be present' >&2; exit 2; fi
    options+=(--device "$device:$device" --group-add "$(stat -c %g "$device")")
  fi
  image_id=$(docker image inspect --format '{{.Id}}' "$image")
  options+=(-e "RM_DEMO_CONTAINER_IMAGE=$image" -e "RM_DEMO_CONTAINER_IMAGE_ID=$image_id")
  args=("stage:=$stage" enable_hardware_io:=true "session_dir:=$session")
fi
if [ ! -f "$results/install/setup.bash" ]; then echo 'Run run_bench_container.sh acq first' >&2; exit 2; fi
# Host network is intentional only for opted-in physical acquisition. No privileged mode.
exec docker run --rm --pull never --ipc private --cap-drop ALL --security-opt no-new-privileges \
  --user "$(id -u):$(id -g)" -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-137}" \
  -e ROS_LOG_DIR=/results/hardware_logs -e PYTHONDONTWRITEBYTECODE=1 \
  --mount "type=bind,src=$repo,dst=/source,readonly" \
  --mount "type=bind,src=$results,dst=/results" \
  "${options[@]}" --workdir /results "$image" bash -c '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /results/install/setup.bash
    exec ros2 launch rm_demo_platform demo_hardware.launch.py "$@"
  ' demo-hardware "${args[@]}"
