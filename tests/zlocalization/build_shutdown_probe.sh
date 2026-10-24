#!/usr/bin/env bash
# Source the ROS/overlay environment first. All outputs stay in the given build directory.
set -euo pipefail
mode=${1:?baseline|asan|tsan}
probe_build=${2:?build directory}
probe_gtsam=${3:?private GTSAM prefix}
probe_flags='-O1 -g1 -DNDEBUG'
probe_link=''
case "$mode" in
  baseline) ;;
  asan)
    # Match the Eigen allocation ABI of the uninstrumented GTSAM/PCL libraries.
    probe_flags+=' -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer -fno-pie -DEIGEN_MALLOC_ALREADY_ALIGNED=1'
    probe_link='-fsanitize=address,undefined -no-pie' ;;
  tsan)
    probe_flags+=' -fsanitize=thread -fno-omit-frame-pointer -fno-pie'
    probe_link='-fsanitize=thread -no-pie' ;;
  *) echo 'unknown instrumentation mode' >&2; exit 2 ;;
esac
export CMAKE_PREFIX_PATH="$probe_gtsam:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$probe_gtsam/lib:${LD_LIBRARY_PATH:-}"
cmake -S external/z_localization/src/backend -B "$probe_build" \
  -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS_RELEASE="$probe_flags" -DCMAKE_EXE_LINKER_FLAGS="$probe_link" \
  -DGTSAM_DIR="$probe_gtsam/lib/cmake/GTSAM"
cmake --build "$probe_build" --target backend_node -j2
