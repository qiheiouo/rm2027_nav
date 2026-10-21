#!/usr/bin/env python3
"""Build private probes only; no source checkout, installation or ROS runtime."""
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"neu_localization_l1"))
from run_baseline import SOURCE_PIN, QUALITY_PIN

root=Path(__file__).resolve().parents[2]
if subprocess.check_output(["git","diff",SOURCE_PIN,"--","src"],cwd=root,text=True).strip():
    raise SystemExit("src differs from fixed new-car baseline")
quality=root/"build/l1_reference_quality"
for rel in ["include/rm_gicp_relocalization/registration_quality.hpp","src/registration_quality.cpp"]:
    path=quality/rel;path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(subprocess.check_output(["git","show",f"{QUALITY_PIN}:src/rm_gicp_relocalization/{rel}"],cwd=root))
for directory,build,extra in [("neu_localization_l1","l1_probe",[f"-DPROJECT_ROOT={root}",f"-DQUALITY_ROOT={quality}"]),
                              ("neu_localization_l2","l2_coarse",[])]:
    subprocess.run(["cmake","-S",str(root/"experiments"/directory),"-B",str(root/"build"/build),
        "-DCMAKE_BUILD_TYPE=Release","-DCMAKE_CXX_FLAGS=-pipe"]+extra,check=True)
    subprocess.run(["cmake","--build",str(root/"build"/build),"-j1"],check=True)
