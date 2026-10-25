#!/usr/bin/env bash
set -euo pipefail
DST_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
DST_MODE=${1:?build/checks/witness/sanity/pilot/repeat}
DST_RUN_ID=${2:-$DST_MODE}
shift
if (($#)); then shift; fi
DST_DATA_ROOT=${DST_DATA_ROOT:-/tmp/rm2027_dynamic_state_time_data_20261025}
DST_BUILD_ROOT=${DST_BUILD_ROOT:-$DST_DATA_ROOT/build_cache}
mkdir -p "$DST_DATA_ROOT" "$DST_BUILD_ROOT"
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL \
  --security-opt no-new-privileges --ulimit core=0 --user "$(id -u):$(id -g)" \
  --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
  -v "$DST_ROOT:/study:ro" -v "$DST_BUILD_ROOT:/cache:rw" -v "$DST_DATA_ROOT:/data:rw" \
  -w /study/research/dynamic_state_time \
  -e ROS_DOMAIN_ID=173 -e ROS_LOCALHOST_ONLY=1 -e ROS_LOG_DIR=/cache/ros_logs \
  -e RCUTILS_LOGGING_USE_STDOUT=0 -e OPENBLAS_NUM_THREADS=1 -e PYTHONDONTWRITEBYTECODE=1 \
  rm2027_navigation:humble -c '
    source /opt/ros/humble/setup.bash
    if [ "$1" = build ]; then
      cmake -S . -B /cache/cmake -DCMAKE_BUILD_TYPE=Release
      cmake --build /cache/cmake -j2
      dpkg-query -W ros-humble-nav2-mppi-controller ros-humble-spatio-temporal-voxel-layer > /data/dependencies.txt
    elif [ "$1" = checks ]; then
      python3 checks.py --build /cache/cmake --output "/data/$2"
    else
      DST_SELECTED_MODE=$1
      DST_SELECTED_RUN=$2
      shift 2
      python3 run.py --build /cache/cmake --mode "$DST_SELECTED_MODE" --output "/data/$DST_SELECTED_RUN" "$@"
    fi
  ' bash "$DST_MODE" "$DST_RUN_ID" "$@"
