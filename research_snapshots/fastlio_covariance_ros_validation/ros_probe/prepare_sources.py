#!/usr/bin/env python3
"""Extract ONLY the inaccessible pose-preparation block, unchanged.

publish_odometry and the entire production translation unit are included as-is.
"""
import hashlib
import json
import sys
from pathlib import Path

source, output = map(Path, sys.argv[1:])
text = (source/"src/laserMapping_bundle.cpp").read_text()
start = text.index("Eigen::Matrix4d T_WB = Eigen::Matrix4d::Identity();")
end = text.index("pub_quat.normalize();", start) + len("pub_quat.normalize();")
block = text[start:end]
include = output/"zero_bundle.inc"
if not include.exists() or include.read_text() != block:
    include.write_text(block)
(output/"source_manifest.json").write_text(json.dumps({
    "zero_block_sha256": hashlib.sha256(block.encode()).hexdigest(),
    "whole_source_sha256": {mode: hashlib.sha256((source/f"src/laserMapping_{mode}.cpp").read_bytes()).hexdigest()
                            for mode in ("bundle", "adaptive", "async")},
    "scope": "whole production source included; deterministic native state injected after filtering; unchanged zero-start block; no sensor simulation",
}, indent=2)+"\n")
