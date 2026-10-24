# Needs-Evidence Closure diagnostics

Integration、无硬件。结论见
[campaign 报告](../../docs/validation/needs_evidence_closure_20261024.md)。

`COLCON_IGNORE` 隔离第三方缺陷诊断，避免把已知错误固化成默认生产验收标准。
此目录的 CTest **通过表示成功刻画当前依赖行为，包括缺陷仍存在**；不是修复通过。
升级依赖后应重新裁决并将相应反例改成正确性回归，不能为保留旧结果回退修复。
只有 `rm_relocalization_bridge/test/test_producer_exit_runtime.py` 是默认包级注册的
正确性回归：gate 死亡、过期状态重放、进程重启/新测量恢复以及有界清理。

## 复现

使用既有 `rm2027_navigation:humble` 镜像，无网络/设备、私有 IPC、非 root、
cap-drop ALL、2 CPU/2 GiB。把本分支挂到 `/campaign:ro`，固定 main 子模块挂到
`/parent:ro`，10-21 独立回归目录挂到 `/baseline:ro`，独立输出目录挂到
`/results:rw`。不启动 Livox SDK/驱动或真实串口。

```bash
source /opt/ros/humble/setup.bash
source /baseline/build/fastlio_validation/install_driver/local_setup.bash
export ROS_LOG_DIR=/results/ros_log
ulimit -c 0
cmake -S /campaign/tests/needs_evidence_closure -B /results/native \
  -DFASTLIO_SOURCE=/parent/src/fast_lio_multi \
  -DLIVOX_SOURCE=/parent/src/livox_ros_driver2_humble \
  -DFASTLIO_LIBRARY=/baseline/build/fastlio_validation/backend/fast_lio_multi/libfastlio_preprocess.a
cmake --build /results/native -j1
ctest --test-dir /results/native --output-on-failure
```

`FASTLIO_LIBRARY` 复用固定 9aa8c79 / ikd 0438b0d 的已有预处理库；callbacks、sync、
FOV 和 IMU 方法从同 pin 源码重新编译。需要独立重建时先使用现有
`tests/fastlio_deterministic/build_and_test.sh` 生成该库/消息依赖，再改上述路径。
这里没有复制、替换或修改第三方生产方法。

包级回归只需构建 `rm_relocalization_bridge` 及直接依赖，source 对应 install 后执行：

```bash
PYTEST_ADDOPTS='--basetemp=/results/producer_tmp' \
  ctest --test-dir /results/bridge_build/rm_relocalization_bridge \
  -R '^test_producer_exit_runtime$' --repeat until-fail:3 --output-on-failure
```

原生详细数值在 `native/Testing/Temporary/LastTest.log`；planner 在
`native/planner/result.json`；包级退出、重放和清理观测在 `producer_tmp/**/evidence.json`。
输出均留在 build，不进入 Git。
