# STM V2 chassis-attitude host validation — 2026-11-13

Status: **Host software ready / STM firmware pending / Hardware Go = NO**.

## Fixed inputs and isolation

- Source baseline: `integration/demo-navigation-validation` at
  `de05e0858de078f5e035757f0cc8d48b63f34c52`.
- Runtime image (read-only input):
  `rm2027_navigation:demo-validation-de05e085-20261112`, image ID
  `sha256:d821f860b572dc1165e012dbb81debf16bb095de71b6278a4a980255e36e4d62`.
- No image was built, retagged or overwritten.
- Validation containers used `--network none`, private IPC, all capabilities dropped,
  `no-new-privileges`, no device mounts, `ROS_LOCALHOST_ONLY=1`, an isolated ROS domain,
  2 CPUs and 3 GiB RAM.
- The source tree was mounted read-only. No STM, serial device or motion output was used.
- Builds used `CMAKE_BUILD_PARALLEL_LEVEL=1`, `MAKEFLAGS=-j1` and
  `colcon build --parallel-workers 1`.

## Build result

- The interface/serial targeted build passed.
- The workspace build passed for 22 packages when the optional
  `rm_new_car_simulation` package was explicitly excluded. That optional package cannot
  configure in the fixed baseline image because `gz_sim_vendor` is absent; no host or
  image dependency was installed to hide that limitation.
- The pre-existing FAST-LIO/PCL compiler warnings remain outside this change.

## Functional results

The final combined result contains 66 non-Livox leaf test cases and four Livox driver
fixtures, for 70 unique functional cases. All have zero failures, zero errors and zero
skips. The 66-case group includes the current protocol, reference codec, localization,
Demo Phase 1, timestamp provenance and runtime/lifecycle cases.

Five leaf cases are new relative to the source baseline:

1. canonical C++ `0x86` golden-vector round trip;
2. canonical C++ full-orientation fixture matrix;
3. canonical C++ malformed/invalid `0x86` matrix;
4. C11 reference `0x86` golden-vector round trip;
5. ROS adapter lifecycle/fail-closed runtime fixture.

Coverage includes flat/roll/pitch/yaw/combined/ramp/continuous orientations; truncated,
CRC-corrupt and wrong-length frames; NaN and invalid quaternion; reserved flags;
duplicate/out-of-order/lost samples; uint32 sequence/time wrap; reset/reboot and retired
boot replay; component invalid, offline, stale and disconnect behavior. Invalid data never
produces a substitute zero or stale `sensor_msgs/Imu` sample.

For traceability, `colcon test-result` reported 78 tests for the expanded main pass and
73 after the ACQ supplemental pass. The second command reuses the same result directory
and reruns a package subset, so those wrapper-inclusive summaries must not be added.
The 70 count above is the de-duplicated leaf-case count.

## Bench and safety result

- ACQ disabled: `SOFTWARE_BENCH_PASS`; no provenance records; zero-disabled chassis
  frames, canonical SE(3), TF owners, gimbal loss and attitude loss all passed.
- ACQ enabled: `SOFTWARE_BENCH_PASS`; 2721 provenance records; the same safety and
  lifecycle checks passed.
- Default inventory produced zero runtime nodes. Generic nonzero `/cmd_vel` injection did
  not obtain V2 authority. No hardware acceptance was claimed.
- Production motion authorization remains false and the emitted chassis command remains
  zero-disabled.

## Deliberately separate non-acceptance checks

An exploratory all-package `colcon test` also ran. Functional package tests passed, but
the vendored Livox package's broad lint target reports existing copyright/style/schema
issues, and `rm_map_tools` contains one deliberate skip when an unpackaged field map is
absent. These are not silently counted as acceptance. The explicit functional collection
above is the zero-failure/zero-error/zero-skip gate for this change.

## Remaining blocker

The protocol is defined and the host side is ready. Phase 1A remains limited by STM
firmware implementation/sign-off, MCU-toolchain golden-vector evidence, real captures,
physical frame/gravity/direction confirmation, measured rate/latency, static IMU-to-base
calibration and passive receive-only hardware validation. Hardware Go remains **NO**.
