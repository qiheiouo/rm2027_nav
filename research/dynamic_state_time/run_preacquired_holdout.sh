#!/usr/bin/env bash
set -euo pipefail
DST_SCRIPT_ROOT=$(cd "$(dirname "$0")" && pwd)
DST_DATA_ROOT=${DST_DATA_ROOT:?set an external evidence directory}
DST_RUN_ID=${1:?new run directory name}
export PYTHONDONTWRITEBYTECODE=1
mkdir -p "$DST_DATA_ROOT/build_cache"
g++ -O3 -DNDEBUG -std=c++17 "$DST_SCRIPT_ROOT/reserve_consumer.cpp" -o "$DST_DATA_ROOT/build_cache/reserve_consumer"
git -C "$DST_SCRIPT_ROOT" show 21b9e56076b23f17f1140b9ed68bff5edce9b5b6:research/dynamic_state_time/search_consumer.cpp |
  g++ -O3 -DNDEBUG -std=c++17 -x c++ - -o "$DST_DATA_ROOT/build_cache/search_baseline"
python3 "$DST_SCRIPT_ROOT/preacquired_holdout.py" --build "$DST_DATA_ROOT/build_cache" --output "$DST_DATA_ROOT/$DST_RUN_ID" "${@:2}"
