#!/usr/bin/env bash
# Build only the supported Jazzy simulation profile, in a reproducible shell.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [[ ${1:-} != --isolated ]]; then
  exec env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 \
    bash --noprofile --norc "$0" --isolated "$@"
fi
shift
if [[ $# -gt 1 || ( $# -eq 1 && $1 != --clean ) ]]; then
  echo "Usage: tools/build_simulation.sh [--clean]" >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
if [[ ${1:-} == --clean ]]; then
  python3 - <<'PY'
from pathlib import Path
import shutil
import subprocess

root = Path.cwd().resolve()
targets = [root / name for name in ('build', 'install', 'log')]
registered = [Path(line[9:]) for line in subprocess.check_output(
    ['git', 'worktree', 'list', '--porcelain'], text=True).splitlines()
    if line.startswith('worktree ')]
# Historical worktrees/evidence must be moved out before using this workflow.
for target in targets:
    if target.is_symlink():
        raise SystemExit(f'Refusing to clean symlink: {target}')
    if not target.exists():
        continue
    if not (target / 'COLCON_IGNORE').is_file():
        raise SystemExit(f'Refusing unmanaged directory: {target}; inspect and preserve its contents first')
    if any(path == target or target in path.parents for path in registered) or any(target.rglob('.git')):
        raise SystemExit(f'Refusing to remove a repository/worktree inside {target}')
for target in targets:
    if target.exists():
        shutil.rmtree(target)
PY
fi
# The launch bundles also declare dependencies for legacy hardware profiles.
# Limit discovery to this profile: --packages-select with all src/ discovered
# would still require the unbuilt Humble packages' environment hooks.
colcon build --symlink-install --executor sequential \
  --base-paths src/rm_description src/rm_nav_config src/rm_new_car_simulation \
               src/rm_navigation_bringup src/rm_navigation_launch \
               src/rm_competition_interfaces src/rm_competition_mission src/rm_pursuit \
               src/rm_localization_adapters src/rm_relocalization_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
python3 tools/check_simulation_bringup.py
