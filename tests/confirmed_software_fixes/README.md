# 已确认软件缺陷回归

Integration，无网络/设备/运动；真实 ROS action 和被修复的方法，输入为合成受控夹具。
此目录由 `COLCON_IGNORE` 隔离；它是显式正确性回归入口，通过表示满足正确合同。

AUD-016：原配置实际 planner + Navfn/global StaticLayer，首图缺失时取消，
要求在不恢复地图的前提下进入 CANCELED 且空路径。分别覆盖 ToPose/ThroughPoses；
随后恢复地图并确认新 goal 成功。实际 controller 使用显式 currentness layer 和计数
controller plugin，收到 setPlan 事件后取消，要求无 compute、无非零输出且 CANCELED；
恢复 currentness 后新 goal 必须能计算并正常取消。输出仅在 fixture topic，没有硬件。
测试 deadline 仅判定失败和回收，不用于实现取消或绕过竞态。

在已 source 修复 overlay 的 Humble 1.1.20 环境：

```bash
cmake -S tests/confirmed_software_fixes -B build/confirmed_regression \
  -DCMAKE_INSTALL_PREFIX="$PWD/build/confirmed_fixture"
cmake --build build/confirmed_regression -j1
cmake --install build/confirmed_regression
source build/confirmed_fixture/share/confirmed_software_fixes/local_setup.bash
ctest --test-dir build/confirmed_regression --output-on-failure
```

JSON 和进程日志写入 build；不提交生成产物。测试必须独占 ROS domain。
