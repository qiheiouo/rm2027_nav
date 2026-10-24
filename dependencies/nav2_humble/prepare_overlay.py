#!/usr/bin/env python3
"""Copy only the repaired packages from the exact Nav2 pin and apply local patches."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

PIN = 'a097086719c88f781aa59788eca29ac6ca5e56db'
PACKAGES = ('nav2_planner', 'nav2_controller')
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('source', type=Path)
parser.add_argument('output', type=Path)
a = parser.parse_args()
source, output = a.source.resolve(), a.output.resolve()
assert subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip() == PIN, 'Nav2 pin mismatch'
assert not subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain', '--untracked-files=no']), 'Nav2 source must be clean'
assert not output.exists(), 'Use a new output directory; existing data is never removed'
output.mkdir(parents=True)
shutil.copy2(source / "LICENSE", output / "LICENSE")
for package in PACKAGES:
    shutil.copytree(source / package, output / package)
env = dict(os.environ, GIT_CEILING_DIRECTORIES=str(output.parent))
for patch in sorted((Path(__file__).parent / 'patches').glob('*.patch')):
    subprocess.run(['git', 'apply', '--check', str(patch.resolve())], cwd=output, env=env, check=True)
    subprocess.run(['git', 'apply', str(patch.resolve())], cwd=output, env=env, check=True)
print(f'Prepared Nav2 {PIN} in {output}')
