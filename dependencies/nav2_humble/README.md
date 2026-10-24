# Nav2 Humble 修复 overlay

- 来源：https://github.com/ros-navigation/navigation2
- 版本：1.1.20，commit `a097086719c88f781aa59788eca29ac6ca5e56db`
- 许可证：Apache-2.0，源码复制保留原 notice；完整许可证见源仓库 `LICENSE`。
- 范围：仅复制 `nav2_planner`、`nav2_controller`、`nav2_behavior_tree`，应用本目录 patch；其余包使用既有 Humble 1.1.20。
- AUD-016：costmap 等待响应取消、server 停用和 context 退出；恢复后先检查取消再计算。
- AUD-056：保留超时/停止请求的 future；将未完结请求的回调组交给局部清理任务，
  迟到接受即取消、终态前阻断本 BT node 重入，过期回调不再写回对象；销毁后仍有归属。
- 无 action/topic/参数/插件 ID、消息 ABI 或比赛策略变更；不增加取消超时，不提高接受超时。
- 该 overlay 必须构建并 source 后才生效；系统 apt 安装本身没有被修改。

```bash
# 先取得上述固定源码，不修改系统包。source 原 Humble 环境。
python3 dependencies/nav2_humble/prepare_overlay.py /path/to/navigation2 /path/to/new-overlay-src
colcon build --paths /path/to/new-overlay-src/nav2_planner /path/to/new-overlay-src/nav2_controller \
  /path/to/new-overlay-src/nav2_behavior_tree \
  --build-base build/nav2_fixes --install-base install/nav2_fixes --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/nav2_fixes/local_setup.bash
ros2 pkg prefix nav2_planner
ros2 pkg prefix nav2_controller
ros2 pkg prefix nav2_behavior_tree
```

`prepare_overlay.py` 拒绝 pin 不同、有 tracked 改动的源和已存在的输出目录。
回退：在新 shell 仅 source 原 Humble 和原项目 install。更新 Nav2 时需逐项重审/重跑，
不得向其他版本静默套用补丁。component 与独立 executable 均来自同一修复 library。

回归见 [tests](../../tests/confirmed_software_fixes/README.md)，结论见
[验证记录](../../docs/validation/confirmed_software_fixes_20261024.md)。

模板修复必须重编 `nav2_behavior_tree` 的 action plugins；不能只替换安装 header，
或把旧 apt action plugin 与新模板派生类混用。上面的包构建包括所有插件。
资源受限时可直接串行构建（先 source 既有 Humble/同版 Nav2 依赖）：

```bash
cmake -S /path/to/new-overlay-src/nav2_behavior_tree -B build/nav2_bt_fixes \
  -DCMAKE_INSTALL_PREFIX="$PWD/install/nav2_bt_fixes" -DBUILD_TESTING=OFF
cmake --build build/nav2_bt_fixes -j1
cmake --install build/nav2_bt_fixes
source install/nav2_bt_fixes/share/nav2_behavior_tree/local_setup.bash
# 下游手动 CMake 配置时显式选择该安装包（避免找到 apt 的旧 header）：
# -Dnav2_behavior_tree_DIR="$PWD/install/nav2_bt_fixes/share/nav2_behavior_tree/cmake"
```

未知请求没有“等到期就安全”的规则；若服务端始终无响应/不终结，本 node 保持失败屏障。
节点销毁后清理任务保留 client/node/group 到拒绝/终态或 ROS context 退出；不访问已销毁 BT。
本修复不保证服务端接受到取消间没有内部执行，也不替代最终速度撤权/物理停车。
