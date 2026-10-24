# Z-Localization / ZLIO isolated candidate

**Integration only; DO NOT MERGE; no Hardware Go.** This parity branch extends
frozen RM baseline `ddabf8b` (documentation checkpoint `7999717`). The complete
[upstream migration matrix](../../docs/zlocalization_upstream_parity_20261024.md)
distinguishes availability, tested software behavior and missing real-data evidence.
The 2026-10-23 freeze documents remain historical evidence for the older baseline.

Both RM launch entries default to `enabled:=false` and start no nodes. No driver,
Nav2, mission, serial or command publisher is included. FAST-LIO remains the
existing platform default. Native ZLIO is an explicit alternate frontend.

## Build

Use the existing ROS Humble environment, GTSAM **4.2.x**, Eigen, PCL, nanoflann,
yaml-cpp, TBB and OpenMP. Keep `external/COLCON_IGNORE`. Build only selected
upstream packages and source the existing pinned **RM Livox** overlay for its
message dependency; do not build the bundled duplicate driver.

```bash
git submodule update --init external/z_localization
git -C external/z_localization apply ../../patches/zlocalization_scan_end_pose.patch
source /opt/ros/humble/setup.bash
source "$LIVOX_OVERLAY/setup.bash"
colcon build --base-paths external/z_localization/src/localization_interfaces \
  external/z_localization/src/backend external/z_localization/src/frontend \
  external/z_localization/src/bringup --cmake-args -DBUILD_TESTING=ON \
  -DGTSAM_DIR="$GTSAM_PREFIX/lib/cmake/GTSAM" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG -Wno-error=maybe-uninitialized'
source install/setup.bash
colcon build --base-paths src/rm_competition_interfaces \
  src/rm_localization_adapters src/rm_relocalization_bridge \
  src/rm_lio_bringup src/rm_zlocalization_candidate --cmake-args -DBUILD_TESTING=ON
```

GCC 11 at `-O1` reports an upstream `std::optional` `maybe-uninitialized`
warning in `window.cpp`; the flag above keeps it visible without turning it into
an error. Estimator mathematics are unchanged. The three-line backend patch
publishes scan-end pose from the same committed projection for either frontend.
Apply it once; the recorded submodule pin remains clean upstream.

## FAST-LIO candidate

Start the measured source-time dynamic gimbal/full-base TF first. Select only
this global backend, without AMCL/GICP/stub or another canonical bridge.

```bash
ros2 launch rm_zlocalization_candidate z_localization_candidate.launch.py \
  enabled:=true prior_map:=/absolute/path/to/prior
```

`start_fastlio:=true` includes existing FAST-LIO with registered cloud enabled
and delayed zero-start disabled. Externally started FAST-LIO must also retain a
stable numeric origin. `start_lio_adapter:=false` preserves an already selected
canonical/full-base owner. See the [interface contract](../../docs/zlocalization_interface_contract_20261023.md).

## Native ZLIO shadow

Use a separate localhost ROS domain for each A/B run. Supply a confirmed sensor
configuration and acceleration scale (`1.0` for SI, `9.81` for g); neither is
assumed to be a new-car calibration. No physical IO is started.

```bash
ros2 launch rm_zlocalization_candidate zlio_shadow.launch.py \
  enabled:=true frontend_config:="$CALIBRATED_ZLIO_CONFIG" \
  imu_acceleration_scale:="$IMU_ACCELERATION_SCALE" mode:=mapping use_sim_time:=true
```

Set `mode:=localization prior_map:=/absolute/prior` for independent query bags.
`input_type:=livox|pointcloud2`, `lidar_topic`, `imu_topic`, `livox_use_timebase`
and `point_time_field/scale/absolute` expose native timestamp semantics.
Services are `/zlocalization/shadow/{start,stop,reset_internal,upstream_reset_request,
relocalize,save_map}`. Frontend instance/epoch, predictions, poses, diagnostics,
visualization and upstream TF stay under this private prefix. Backend's private
map→zlio_odom is quarantined; frontend TF is disabled. The native reset transaction
works here, independently of RM canonical state.

## Restricted native canonical candidate

```bash
ros2 launch rm_zlocalization_candidate z_localization_candidate.launch.py \
  enabled:=true frontend:=zlio frontend_config:="$CALIBRATED_ZLIO_CONFIG" \
  imu_acceleration_scale:="$IMU_ACCELERATION_SCALE" prior_map:=/absolute/prior
```

The existing `lio_adapter` converts atomic scan-end IMU poses using real dynamic
TF and derives chassis twist. `map_odom_from_global_pose` remains the sole
map→odom owner. `start_zlio:=false` accepts an external native producer adhering
to the same private frame/session contract; it is used by the boundary fixture.

**Any instance/epoch change, reset request, processing failure or nonmonotonic
frame sequence/time latches the canonical adapter invalid. Restart the complete
frontend/canonical/backend/adapter/bridge group.** No automatic canonical reset
continuity is claimed. High-frequency native prediction is available in shadow;
the canonical candidate deliberately consumes synchronized scan-end evidence only.
Native model covariance is retained on original atomic and private upstream
odometry outputs. The raw input handed to the existing canonical adapter marks
pose covariance missing because that owner does not propagate it through its
startup basis and moving lever arm. Do not treat canonical odom covariance as a
calibrated uncertainty. Global-pose covariance still uses the explicit lever-arm
propagation. Mechanical/clock uncertainty remains unmeasured.

## Optional upstream capabilities

- `descriptor:=scan_context|gprobe` selects the descriptor; defaults to SC.
  A prior bundle must contain compatible descriptor metadata.
- `enable_loop:=true` enables native loop/PCM in **mapping** only. Localization
  against immutable prior rejects this combination.
- `visualization:=true`, `world_cloud:=true`, `rviz:=true` independently enable
  diagnostic outputs or the upstream backend RViz configuration with private-topic
  remaps. GUI display is not part of the headless acceptance evidence.
- `/zlocalization/diagnostics` is a read-only DiagnosticArray with status,
  source age, versions and failure reasons. Steady-time expiry also works while
  a bag clock is paused. It grants no motion authority.
- UWB, TDT legacy outputs, fixed sensor→body calibration application and bundled
  driver startup remain excluded from RM profiles. Software map/body-calibration
  tools remain directly callable upstream.

The [offline map workflow](../../docs/zlocalization_offline_workflow_20261023.md)
uses the same map tools; add `enabled:=true` to its older candidate launch examples.
See [software reproduction](../../tests/zlocalization/README.md) and the
[new-car merge gates](../../docs/zlocalization_upstream_parity_20261024.md#新车-bag-到来后的合入门槛).
