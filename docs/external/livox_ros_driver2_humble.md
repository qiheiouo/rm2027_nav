# Livox ROS2 Humble Driver Intake Record

This record documents the external MID360 ROS driver added for Phase 1 hardware integration.

## Source

- Maintained URL: `https://github.com/qiheiouo/livox_ros_driver2_humble.git`
- Original intake URL: `https://gitee.com/SMBU-POLARBEAR/livox_ros_driver2_humble.git`
- Local path: `src/livox_ros_driver2_humble`
- Integration type: Git submodule
- Branch at intake: `master`
- Commit at intake: `2a2029a6e62a2196b280be6ec00bb2418065b8e0`
- Final-freeze pin before this exception: `2a2029a6e62a2196b280be6ec00bb2418065b8e0`
- AUD-020 local repair pin: `9bdedb484b1dd9bd7420423a728812dcf8cc8267`
- AUD-020 local repair branch: `codex/aud020-bounded-shutdown-20261020`
- AUD-050 local repair pin: `ffb5c721c3356817a84f1c6eeaf2f66c680be24e`
- AUD-050 local repair branch: `codex/aud050-ring-queue-20261020`
- Published maintenance branch: `rm2027/aud020-aud050-20261020`
- Acquisition instrumentation branch: `codex/localization-acquisition-provenance`
- Acquisition instrumentation pin: `24e989f43afd918f333ea20d526ed7ecc6615bb5`
- Instrumentation publication: local commit only; not pushed by this task.
- ROS package name: `livox_ros_driver2`
- License: MIT
- Internal submodules: none observed; the driver repository has no `.gitmodules`.

## Why This Driver

The target runtime is ROS2 Humble and MID360. This fork was already reviewed during open-source research and matches the driver API expected by `rm_mid360_driver_bridge`:

- executable: `livox_ros_driver2_node`
- package: `livox_ros_driver2`
- config parameter: `user_config_path`
- point cloud frame parameter: `frame_id`
- output modes including `xfer_format=4`

## Required Linux SDK Install

Install Livox-SDK2 on Ubuntu 22.04 before building the driver:

```bash
sudo apt update
sudo apt install -y build-essential cmake git libapr1-dev libpcl-dev

mkdir -p ~/third_party
cd ~/third_party
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
git checkout --detach f5d9375f84efe2b15bc0a052d3e18482ed13adf4
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

If GitHub is unavailable on the target network, use an explicitly reviewed Livox-SDK2 mirror and record its URL and commit before relying on it.

## Clone And Submodule Commands

Fresh clones should include submodules:

```bash
git clone --recurse-submodules https://gitee.com/qiheiovo/rm2027_navigation.git
cd rm2027_navigation
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

There is no second-level submodule initialization requirement inside `livox_ros_driver2_humble` at this intake commit.

## Local Shutdown Repair

The AUD-020 exception makes the exit flag atomic, protects semaphore counts
with their mutex, wakes both point-cloud and IMU consumers on exit, and checks
the exit request again after each wait. Driver destruction requests exit before
joining workers, and the initial three-second poll delay is interruptible.
Lifecycle fixtures cover failed start, no input, stopped input, active synthetic
queue traffic, repeated construction/destruction, and the normal ROS point/IMU
publication path without busy waiting.

The preserved commit chain is published to the project-controlled GitHub
maintenance branch `rm2027/aud020-aud050-20261020`. A fresh HTTPS clone was
verified to resolve the exact final pin without the old main repository or its
archived refs. Roll back by restoring the gitlink
`2a2029a6e62a2196b280be6ec00bb2418065b8e0`. Offline tests cannot prove the
vendor SDK's successful hardware initialization/deinitialization lifecycle;
that remains a hardware acceptance boundary rather than a reason to widen this
repair into a driver redesign.

## Local Point-cloud Queue Repair

The AUD-050 exception keeps the existing per-LiDAR SPSC point-cloud queue.
The single `PubHandler::RawDataProcess` worker owns the write index, while the
single point-cloud polling worker owns the read index. The indices are now
atomic: publishing a completed slot uses a release store and the consumer uses
an acquire load; releasing a consumed slot uses the inverse release/acquire
pair before producer reuse. Queue capacity, full/empty rules, FIFO order,
semaphore wakeups, and the mutex-protected IMU queue are unchanged.

The dedicated fixture covers empty/full behavior, high-frequency transfer,
unequal producer/consumer rates, FIFO payload integrity, and sanitizer runs.
The existing publication and lifecycle fixtures remain the regression guard
for normal point/IMU output and AUD-020 bounded shutdown. Roll back AUD-050 by
restoring the gitlink to `9bdedb484b1dd9bd7420423a728812dcf8cc8267`;
roll back both local exceptions by restoring the original intake pin.

## Build Notes

The acquisition pin adds default-disabled, diagnostic-only packet/output time
provenance on top of AUD-050. The time selector and existing sensor messages are
unchanged. Metadata follows the existing aggregation and queues, capped at 512
packet records per cloud; no raw payload copy or extra sensor/TF/control owner
is introduced. See the [acquisition contract](../operations/localization_raw_acquisition.md)
and the fork README/message definitions. Restore gitlink
`ffb5c721c3356817a84f1c6eeaf2f66c680be24e` to remove this instrumentation.
The new local fork commit must be published before another machine can fetch
the updated gitlink from GitHub; the isolated checkout retains it here.

With Livox-SDK2 installed:

```bash
source /opt/ros/humble/setup.bash
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Without Livox-SDK2, non-hardware packages can still be checked by skipping the driver:

```bash
colcon build --symlink-install --packages-skip livox_ros_driver2
```

## Boundary Rules

The driver is a sensor input dependency only. It must not publish localization TF, canonical odometry, chassis commands, navigation goals, serial packets, referee data, or behavior-tree commands.

`rm_mid360_driver_bridge` owns the project-side topic remaps:

- `/livox/left/lidar`
- `/livox/right/lidar`
- `/livox/left/pointcloud`
- `/livox/right/pointcloud`
- `/livox/lio_imu_raw`

`rm_localization_adapters/imu_frame_adapter` remains responsible for converting the selected raw driver IMU into canonical `/livox/lio_imu` with `header.frame_id=lio_imu_link`.

## Known Frame Caveat

The inspected driver uses the `frame_id` parameter for point cloud messages, but its IMU message frame may not follow that parameter. The project therefore keeps the IMU adapter boundary instead of trusting driver IMU frame names directly.
