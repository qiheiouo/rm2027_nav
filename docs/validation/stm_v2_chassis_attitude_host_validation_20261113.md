# STM V2 chassis-attitude Host Freeze v1 validation — 2026-11-13

Status: **Host software ready / STM firmware pending / Hardware Go = NO**.

## Fixed inputs and isolation

- Source baseline: `integration/demo-navigation-validation` at
  `de05e0858de078f5e035757f0cc8d48b63f34c52`.
- Code-under-test commit: `b462e1c` (`修复：解决 STM V2 消息编号冲突`)；
  最终冻结文档位于包含本报告的后续提交，交付 HEAD 由 `git rev-parse HEAD`
  和最终交付报告记录（Git commit 不能在自身 tree 中预先记录自己的 SHA）。
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
- The final clean single-worker workspace build took 477.568 seconds (7 minutes
  57 seconds) in the 2-CPU/3-GiB validation container. This is validation timing,
  not a replacement for the earlier MiniPC baseline build measurement.

## Cross-branch message-ID audit

All 14 current `origin/*` heads and retained registry history were audited after an
explicit no-submodule-recursion fetch. `origin/codex/three-source-target-arbitration`
at `f09d91fe6c0eb506bc9ea8202310d27a874e816f` contains real C++/C11 codecs,
transport, fixtures and runtime tests for `0x86 RadarTarget` and `0x87 OperatorTarget`,
with capability bits 6 and 7. It is therefore a mergeable candidate, not a superseded
document-only allocation.

The original attitude allocation in `8287309` (`0x86`, bit 6) was a real collision.
Because STM firmware has not implemented chassis attitude, the lowest-risk resolution
was to retain the target candidate allocation and move the complete Host attitude
contract to `0x88`, capability bit 8. Audit found no use of `0x88` or bit 8 in any
remote head, retained registry ancestor, source, reference codec, test, research or
validation record. The envelope, 55-byte payload and all field semantics are unchanged.

The new 65-byte golden packet has type `0x88` and CRC numeric `0xbe90`, encoded on wire
as `90 be`. Canonical C++ and independently compiled C11 reference tests both matched
the same payload and packet byte-for-byte.

## Functional results

The final combined result contains 66 non-Livox leaf test cases and four Livox driver
fixtures, for 70 unique functional cases. All have zero failures, zero errors and zero
skips. The 66-case group includes the current protocol, reference codec, localization,
Demo Phase 1, timestamp provenance and runtime/lifecycle cases.

Five leaf cases are new relative to the source baseline and now use `0x88`:

1. canonical C++ `0x88` golden-vector round trip;
2. canonical C++ full-orientation fixture matrix;
3. canonical C++ malformed/invalid `0x88` matrix;
4. C11 reference `0x88` golden-vector round trip;
5. ROS adapter lifecycle/fail-closed runtime fixture.

Coverage includes flat/roll/pitch/yaw/combined/ramp/continuous orientations; truncated,
CRC-corrupt and wrong-length frames; NaN and invalid quaternion; reserved flags;
duplicate/out-of-order/lost samples; uint32 sequence/time wrap; reset/reboot and retired
boot replay; component invalid, offline, stale and disconnect behavior. Invalid data never
produces a substitute zero or stale `sensor_msgs/Imu` sample.

For traceability, the final selected Host collection contains exactly 66 leaf cases:
15 localization-adapter cases, 38 serial/protocol/runtime cases and 13 Demo Phase 1
cases. Four separately selected Livox fixtures also passed. `colcon test-result` reported
78 wrapper-inclusive tests for each ACQ-off/on main pass and 73 after the ACQ supplemental
pass; those reused CTest wrappers must not be added. The acceptance total remains 70
de-duplicated functional cases, all with zero failures, zero errors and zero skips.

An additional 51 related functional cases from mission, MID360 bridge, relocalization,
dog-hole, pursuit, referee and system-monitor packages also passed; they are useful
regression evidence but are not added to the frozen 70-case acceptance count.

The first post-migration run correctly exposed a fixture heartbeat still advertising
bit 6; after changing it to bit 8, a repeated run exposed an asynchronous test assertion
that could read the preceding status after the new Imu arrived. The fixture now waits for
the matching reset/boot status. The serial package then passed twice consecutively and
the complete final collection passed. No production admission rule was relaxed.

## Bench and safety result

- ACQ disabled: `SOFTWARE_BENCH_PASS`; final 2026-11-16 recheck observed 390 canonical
  samples, 791 chassis Imu samples, 401 joint samples, 1377 zero-disabled frames and no provenance records; canonical
  SE(3), five TF edges/owners, gimbal loss and attitude loss all passed.
- ACQ enabled: `SOFTWARE_BENCH_PASS`; final 2026-11-16 recheck observed 394 canonical
  samples, 791 chassis Imu samples, 405 joint samples, 1377 zero-disabled frames and
  2719 provenance records; the same safety and
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

The protocol is frozen and the host side is ready. Phase 1A remains limited by
**STM firmware implementation + hardware joint verification pending**: firmware
implementation/sign-off, MCU-toolchain golden-vector evidence, real captures,
physical frame/gravity/direction confirmation, measured rate/latency, static IMU-to-base
calibration and passive receive-only hardware validation. Hardware Go remains **NO**.
