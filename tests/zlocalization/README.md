# Z-Localization parity software checks

Use the existing ROS Humble image, no devices, a private localhost ROS domain,
GTSAM 4.2.x in a private prefix and the existing pinned RM Livox overlay.
Build backend/interfaces/frontend/bringup and the candidate's direct dependencies.
Do not build or run the bundled duplicate driver. Apply the parent scan-end patch
before building backend. GCC 11 at `-O1` needs
`-Wno-error=maybe-uninitialized` for upstream frontend `window.cpp`; keep the
warning visible. This is a build limitation, not an estimator change.

In the sourced overlay, from the repository root:

```bash
python3 -m pytest -q src/rm_zlocalization_candidate/test
ROS_DOMAIN_ID=191 python3 tests/zlocalization/laptop_smoke.py --event restart --output build/parity/legacy
ROS_DOMAIN_ID=192 python3 tests/zlocalization/laptop_smoke.py --native --event epoch --output build/parity/native
ROS_DOMAIN_ID=193 python3 tests/zlocalization/laptop_smoke.py --prior --output build/parity/prior
ROS_DOMAIN_ID=194 python3 tests/zlocalization/laptop_smoke.py --descriptor gprobe --loop --output build/parity/gprobe_loop
ROS_DOMAIN_ID=195 python3 tests/zlocalization/shadow_smoke.py --output build/parity/shadow
ROS_DOMAIN_ID=196 python3 tests/zlocalization/diagnostics_smoke.py --output build/parity/paused_clock
```

The native boundary fixture publishes `FrontendFrame/Session` itself and sets
`start_zlio:=false`. The separate shadow smoke starts real ZLIO and feeds synthetic
PointCloud2/IMU, checks native reset and absence of canonical/command publishers.
Neither is a new-car bag test. Use new output directories for repeated runs.

Run upstream descriptor, relocalization, localization/loop and map-tools tests
from `build/.../backend/CTestTestfile.cmake`, and frontend sensor-pipeline tests
from the frontend test build. These test real algorithms on controlled fixtures;
record exact selected targets and results in the parity report. Large labeled
evaluation datasets and hardware tests are not implicit dependencies.

The diagnostics smoke directly starts the installed executable so SIGINT reaches
the tested process. It fixes `/clock` at one second, keeps transmitting the same
source stamp, and verifies steady-time expiry, malformed-state reporting and clean
exit. The known upstream `bringup` composition case `driver=true` expects the
bundled driver launch and is excluded in RM; record its exclusion explicitly.
Do not report the full upstream composition target as passing in this overlay.
