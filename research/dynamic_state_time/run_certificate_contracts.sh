#!/usr/bin/env bash
set -euo pipefail
DST_SCRIPT_ROOT=$(cd "$(dirname "$0")" && pwd)
DST_DATA_ROOT=${DST_DATA_ROOT:?set a new external evidence directory}
export PYTHONDONTWRITEBYTECODE=1
python3 - "$DST_DATA_ROOT" "$DST_SCRIPT_ROOT/../.." <<'PY'
import pathlib,sys
data,repo=map(lambda x:pathlib.Path(x).resolve(),sys.argv[1:])
assert not data.is_relative_to(repo), 'evidence/build must be outside Git'
data.mkdir(parents=True,exist_ok=False)
PY
mkdir -p "$DST_DATA_ROOT/build_cache" "$DST_DATA_ROOT/legacy_build"
git -C "$DST_SCRIPT_ROOT" show f1203c3949f8d48237a9ea0125c895f7f5a8ef2c:research/dynamic_state_time/reserve_consumer.cpp > "$DST_DATA_ROOT/legacy_build/reserve_consumer.cpp"
git -C "$DST_SCRIPT_ROOT" show f1203c3949f8d48237a9ea0125c895f7f5a8ef2c:research/dynamic_state_time/search_consumer.cpp > "$DST_DATA_ROOT/legacy_build/search_consumer.cpp"
g++ -O3 -DNDEBUG -std=c++17 "$DST_DATA_ROOT/legacy_build/reserve_consumer.cpp" -o "$DST_DATA_ROOT/legacy_build/reserve_consumer"
git -C "$DST_SCRIPT_ROOT" show 21b9e56076b23f17f1140b9ed68bff5edce9b5b6:research/dynamic_state_time/search_consumer.cpp |
  g++ -O3 -DNDEBUG -std=c++17 -x c++ - -o "$DST_DATA_ROOT/legacy_build/search_baseline"
g++ -O3 -DNDEBUG -std=c++17 "$DST_SCRIPT_ROOT/reserve_consumer.cpp" -o "$DST_DATA_ROOT/build_cache/reserve_consumer"
ln -s "$DST_DATA_ROOT/legacy_build/search_baseline" "$DST_DATA_ROOT/build_cache/search_baseline"
python3 "$DST_SCRIPT_ROOT/replay_certificate_fixtures.py" --build "$DST_DATA_ROOT/build_cache" \
  --legacy-build "$DST_DATA_ROOT/legacy_build" --output "$DST_DATA_ROOT/replay"
python3 "$DST_SCRIPT_ROOT/test_certificate_harness.py" --build "$DST_DATA_ROOT/build_cache" --output "$DST_DATA_ROOT/harness_contracts"
