# FAST-LIO ROS covariance validation

Scope and decisions: [report](../../docs/research/fastlio_covariance_ros_validation_20261008.md).
Raw production publication boundary Go; overall canonical covariance Modify.
No sensor simulator, EKF change, factor graph, Task 1/2, merge or push.

Run only in this task's independent worktree. The source pin is `ea63e62`; the
baseline adapter control is generated read-only from superproject `d5673e3a`.
`prepare_baseline.py` verifies that the shared pose library/header are byte-identical
to that baseline. No checkout, overwrite of the candidate adapter, or operation
on another worktree is required.

Host preparation (from the independent worktree):

```bash
mkdir -p build/runtime_humble
touch build/COLCON_IGNORE
python3 experiments/fastlio_covariance_ros_validation/prepare_baseline.py
docker run -d --init --name rm2027_fastlio_covariance_ros_validation \
  --network none --ipc private --cpus 2 --memory 6g --pids-limit 512 \
  -e ROS_DOMAIN_ID=116 -e ROS_LOCALHOST_ONLY=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e OMP_NUM_THREADS=1 -e MAKEFLAGS=-j1 \
  -e ROS_LOG_DIR=/workspace/rm2027_navigation/build/runtime_humble/ros_logs \
  --mount "type=bind,src=$PWD,dst=/workspace/rm2027_navigation" \
  --entrypoint bash rm2027_navigation:humble -lc 'sleep infinity'
```

Use an available Domain and unique container name if this task is already
running; the captured experiment used 116. Only this worktree is mounted. The
image contains the real Livox SDK and Humble dependencies. Normal colcon scanning
ignores the experiment folder, so build the probe with explicit `--paths`.

Inside this container, from `/workspace/rm2027_navigation`:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base build/logs_humble build \
  --build-base build/covariance_humble --install-base build/install_humble \
  --packages-select livox_ros_driver2 rm_competition_interfaces fast_lio_multi \
    rm_description rm_mid360_driver_bridge rm_localization_adapters rm_relocalization_bridge \
  --executor sequential --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release \
    '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG' -DBUILD_TESTING=ON
source build/install_humble/setup.bash
colcon --log-base build/logs_probe_humble build \
  --paths experiments/fastlio_covariance_ros_validation/ros_probe \
  --build-base build/probe_humble --install-base build/install_humble \
  --packages-select rm_fastlio_covariance_ros_validation --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF
source build/install_humble/setup.bash
python3 experiments/fastlio_covariance_ros_validation/validate_ros.py --phase baseline
python3 experiments/fastlio_covariance_ros_validation/validate_ros.py --phase candidate
colcon --log-base build/logs_tests_humble test \
  --build-base build/covariance_humble --install-base build/install_humble \
  --packages-select rm_localization_adapters rm_relocalization_bridge --executor sequential
colcon test-result --test-result-base build/covariance_humble --verbose
```

The ROS runner launches each production executable for no-input startup smoke,
then the three whole-source publication-boundary executables. Fixture commands
set the real post-filter ESKF state/P and execute the original publisher. They
do not exercise the original input/deskew/filter callbacks. The three production
sources and unchanged bundle zero-start block have SHA manifests.

The baseline control and candidate adapter receive actual Odometry from the
publication fixture. TF and heading are real ROS messages. Raw 36-element arrays,
stamps, poses and assertions are saved under
`docs/research/fastlio_covariance_ros_validation/evidence/`; non-finite JSON values
are represented as null, after the ROS-side NaN checks. Process/build logs remain
in this task's ignored runtime directory; selected evidence logs are committed.

The runner cleans up only subprocess groups it started. On the host, stop/remove
only this task's container after testing:

```bash
docker stop rm2027_fastlio_covariance_ros_validation
docker rm rm2027_fastlio_covariance_ros_validation
```

The included incremental FAST-LIO bundle requires old commit `e7864a63` in the
new independent submodule clone. Branch/commit restoration and gitlink rollback
are documented in the report. Do not run cleanup, checkout or reset against the
primary checkout or another task's submodule.
