#!/usr/bin/env python3
"""Read fixed Git objects into this task's ignored build directory, no checkout."""
import hashlib
import json
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[2]
base = 'd5673e3a1cfe6937e4126e1df894ee6d66e63e07'
source_path = 'src/rm_localization_adapters/src/lio_adapter.cpp'
source = subprocess.check_output(['git', '-C', str(root), 'show', base+':'+source_path])
directory = root/'build/covariance_baseline_source'
directory.mkdir(parents=True, exist_ok=True)
(directory/'lio_adapter.cpp').write_bytes(source)
for path in ['src/rm_localization_adapters/src/canonical_odometry.cpp',
             'src/rm_localization_adapters/include/rm_localization_adapters/canonical_odometry.hpp']:
    assert (root/path).read_bytes() == subprocess.check_output(['git', '-C', str(root), 'show', base+':'+path]), path
(directory/'manifest.json').write_text(json.dumps(dict(base=base, path=source_path,
    sha256=hashlib.sha256(source).hexdigest(), shared_pose_library='verified byte-identical to fixed baseline'), indent=2)+'\n')
