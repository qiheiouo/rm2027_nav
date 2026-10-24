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

## Shutdown investigation (phase 1, not a fix)

`build_shutdown_probe.sh baseline|asan|tsan BUILD_DIR GTSAM_PREFIX` builds the
pinned backend only. Source ROS and the interfaces overlay, apply the existing
scan-end patch, and use GCC 11 / GTSAM 4.2 (system Eigen). The ASan build explicitly
matches Eigen's allocation ABI with the uninstrumented GTSAM/PCL libraries;
third-party library internals are outside sanitizer coverage. No suppressions.

```bash
bash tests/zlocalization/build_shutdown_probe.sh asan build/shutdown/asan "$GTSAM_PREFIX"
ASAN_OPTIONS=abort_on_error=1:detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
python3 tests/zlocalization/shutdown_stress.py --backend-share "$BACKEND_SHARE" \
  --executable build/shutdown/asan/backend_node --output build/shutdown/run1 --runs 100 --prediction
```

`BACKEND_SHARE` must be the installed backend share (both config/backend.yaml and
launch/backend.launch.py), or an equivalent private symlink tree pointing to the
pinned source; the upstream source package alone lacks config/backend.yaml.

Run inside an isolated ROS domain/container with no hardware or network access.
`mixed` cycles idle, scan, large clouds/world visualization, loop/maintenance.
Each non-idle trial must observe an actual backend projection before SIGINT.
`--peer-exit --scenario stream` additionally stops a subscriber immediately before
the backend; `--prediction` activates the high-rate callback separately. The peer
stops its spin loop synchronously before destroying its context: default rclpy
asynchronous SIGINT shutdown itself failed in this environment. The fixture does
not catch arbitrary exceptions. Startup/readiness/exit deadlines and every return
code are checked; timed-out processes are killed and marked failed. Workload timing
varies the signal point; no shutdown grace sleep, retry-to-pass or ignored failure.
