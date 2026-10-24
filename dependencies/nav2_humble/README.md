# Nav2 Humble 修复 overlay

- 来源：https://github.com/ros-navigation/navigation2
- 版本：1.1.20，commit `a097086719c88f781aa59788eca29ac6ca5e56db`
- 许可证：Apache-2.0，源码复制保留原 notice；完整许可证见源仓库 `LICENSE`。
- 范围：仅复制 `nav2_planner`、`nav2_controller`，应用本目录 patch；其余包使用既有 Humble 1.1.20。
- AUD-016：costmap 等待响应取消、server 停用和 context 退出；恢复后先检查取消再计算。
- 无 action/topic/参数/插件 ID、消息 ABI 或比赛策略变更；不增加取消超时，不提高接受超时。
- 该 overlay 必须构建并 source 后才生效；系统 apt 安装本身没有被修改。

```bash
# 先取得上述固定源码，不修改系统包。source 原 Humble 环境。
python3 dependencies/nav2_humble/prepare_overlay.py /path/to/navigation2 /path/to/new-overlay-src
colcon build --paths /path/to/new-overlay-src/nav2_planner /path/to/new-overlay-src/nav2_controller \
  --build-base build/nav2_fixes --install-base install/nav2_fixes --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/nav2_fixes/local_setup.bash
ros2 pkg prefix nav2_planner
ros2 pkg prefix nav2_controller
```

`prepare_overlay.py` 拒绝 pin 不同、有 tracked 改动的源和已存在的输出目录。
回退：在新 shell 仅 source 原 Humble 和原项目 install。更新 Nav2 时需逐项重审/重跑，
不得向其他版本静默套用补丁。component 与独立 executable 均来自同一修复 library。

回归见 [tests](../../tests/confirmed_software_fixes/README.md)，结论见
[验证记录](../../docs/validation/confirmed_software_fixes_20261024.md)。
