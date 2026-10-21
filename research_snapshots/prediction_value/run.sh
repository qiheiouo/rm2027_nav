#!/usr/bin/env bash
set -euo pipefail
PVT_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
PVT_MODE=${1:-build}
mkdir -p "$PVT_ROOT/build/prediction_value_cache" "$PVT_ROOT/experiments/prediction_value/evidence"
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL \
 --security-opt no-new-privileges --ulimit core=0 --user "$(id -u):$(id -g)" \
 --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
 -v "$PVT_ROOT:/study:ro" -v "$PVT_ROOT/build/prediction_value_cache:/check:rw" \
 -v "$PVT_ROOT/experiments/prediction_value/evidence:/evidence:rw" \
 -w /study/experiments/prediction_value -e PVT_MODE="$PVT_MODE" \
 -e ROS_DOMAIN_ID=172 -e ROS_LOCALHOST_ONLY=1 -e ROS_LOG_DIR=/check/ros_logs \
 -e RCUTILS_LOGGING_USE_STDOUT=0 -e OPENBLAS_NUM_THREADS=1 \
 rm2027_navigation:humble -c '
source /opt/ros/humble/setup.bash
if [ "$PVT_MODE" = build ]; then
 cmake -S . -B /check/cmake -DCMAKE_BUILD_TYPE=Release
 cmake --build /check/cmake -j2
 dpkg-query -W ros-humble-nav2-mppi-controller ros-humble-spatio-temporal-voxel-layer > /evidence/dependencies.txt
elif [ "$PVT_MODE" = checks ]; then
 python3 checks.py --binary /check/cmake/native_consumer --output /evidence
else
 python3 run.py --binary /check/cmake/native_consumer --phase "$PVT_MODE" --output /evidence
fi
'
