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

AUD-052：配置同一测试目录时加 `-DFASTLIO_SOURCE=/absolute/path/to/src/fast_lio_multi`，
source 既有 `livox_ros_driver2` 消息依赖。真实 ESKF + 完整共享 ImuProcess，无理想 predict
替代。`aud052_deskew` 验证单雷达1/2、双雷达，零/正 offset、50ms 浮点边界、单点、
重复/乱序、空云及匀速旋转；对每点断言三维解析误差 <1e-6m，并检查帧末状态。

AUD-056：`aud056_*` 使用真实 ROS client、完整 BtActionNode 和事件闸门服务器，
不使用 sleep 排序。覆盖接受超时、接受前 halt、迟到拒绝、拒绝取消、对象销毁、
已接受目标停止、更新目标超时、成功与取消交错；要求旧请求归属持续、终态前不重发、
旧成功/反馈不写回，新请求可恢复；还覆盖终态结果故意缺失时的 context 退出。`on_cancelled` 检查收到有效取消结果。
要验证真正安装的 Spin 插件，加
`-DNAV2_SPIN_PLUGIN=/absolute/overlay/lib/libnav2_spin_action_bt_node.so`。
`NAV2_BT_SOURCE` 仅用于开发时对原/修复 header 做 A/B；最终验收不用该覆盖，
直接通过修复 `nav2_behavior_tree` 安装包编译并加载实际插件。若用手动 CMake 安装，
还需显式传入 `-Dnav2_behavior_tree_DIR=/absolute/overlay/share/nav2_behavior_tree/cmake`，
确认配置输出为该安装目录，避免下游仍选中 apt header。
