# 统一仿真构建与启动

Current：ROS 2 **Jazzy + Gazebo Harmonic（gz-sim8）**，Integration 仿真入口。
从正式仓库根目录执行下列命令。唯一推荐入口为
`rm_navigation_bringup/simulation.launch.py`；不修改导航算法，也不继续 Dynamic Navigation 研究。

## 环境与工作区

系统需要 `/opt/ros/jazzy`，以及 Gazebo Harmonic vendors、ros_gz_bridge、ros_gz_interfaces、
Nav2（含 MPPI/Navfn/RViz plugins）、RViz、robot_state_publisher、xacro、ament/colcon、
Python numpy/scipy/yaml。使用项目现有依赖，不 source 旧 workspace，不混用 Humble。

源码只放在 `src/`。根目录 `build/`、`install/`、`log/` 只放本次 colcon 产物；
colcon 会在这些标准输出目录自动建立 `COLCON_IGNORE`。`.gitignore` 仅影响 Git，
不能隔离 package discovery。源码 worktree 应放在仓库外（如 `~/.codex/worktrees/`），
禁止放回 `build/`。`research_snapshots/COLCON_IGNORE` 保持冻结研究隔离。

## Build

```bash
tools/build_simulation.sh --clean
```

脚本在清空继承环境的 shell 中仅 source `/opt/ros/jazzy/setup.bash`，执行：

```bash
colcon build --symlink-install --executor sequential \
  --base-paths src/rm_description src/rm_nav_config src/rm_new_car_simulation \
               src/rm_navigation_bringup src/rm_navigation_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

五个包是完整的**此仿真 profile**，不是全仓硬件栈。两个 launch 包同时保存 Humble
硬件组合的依赖；所以固定 source paths，而不是先发现全仓再 `--packages-select`，
也不把 `--packages-up-to` 用于跨 ROS 版本的组合包。安装后会检查全仓 discovery、
五个包的实际 prefix、launch/配置/地图生成资产及该仿真的依赖。
硬件/旧车 launch 的全部运行依赖不在此构建集合中。

`--clean` 只删除标准输出目录；遇到没有 colcon 标记的目录、symlink 或嵌套 Git/worktree
就拒绝，必须先检查并保留其中数据。增量构建可省略 `--clean`。不要在仿真仍运行时 clean。

`build/jazzy_sim_install` 是旧的三包选择性构建产物，已退出正式 workflow；
不要 source、复制或迁移其安装文件。

## Source

建议启动一个清空 ROS/overlay 历史环境、只保留显示连接信息的终端：

```bash
env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 \
  DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" bash --noprofile --norc
```

在仓库根目录执行唯一 source 命令（setup 已记录 `/opt/ros/jazzy` underlay）：

```bash
source install/setup.bash
```

检查安装和资产：

```bash
python3 tools/check_simulation_bringup.py
ros2 pkg list
```

`colcon list` 查源码，`ros2 pkg list` 查当前 shell 的安装索引。源码中存在 package
不等于它已被 build 或 source。若上述检查指出 prefix 不在本仓 `install/`，先修复 shell，
不要复制 install 文件或增加同名兼容 package。

## Simulation

```bash
ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=rm2027_simulation \
  ros2 launch rm_navigation_bringup simulation.launch.py
```

默认启动 Gazebo server、静态规则尺寸测试场、机器人/关节、GPU LiDAR 代理、
仿真真值定位、地图服务器、Navfn、MPPI、behavior、velocity smoother、lifecycle manager
和 RViz。`headless:=false` 可同时打开 Gazebo GUI；`use_rviz:=false` 可关闭 RViz。
`headless` 只控制 Gazebo GUI，与 RViz 开关独立。GPU LiDAR 使用 headless rendering，
仍需要可用 EGL/OpenGL；纯 Nav2 软件检查可显式 `sensors:=false`。

场地固定为已有 `rules2026_course`，机器人初始位置 `(-3, -3)` m。
world/URDF/occupancy/PCD/manifest 来自同一次生成，默认写入 `/tmp/rm2027_simulation`；
可用 `output:=/absolute/path` 指定。不要在同一 domain、partition 或 output 目录同时运行两套。
地图是保守静态投影，不证明坡顶、洞口和多层自主导航。

TF owner 保持 S2 合同：仿真 oracle 独占 `map → odom → base_link`，
robot_state_publisher 只负责关节/传感器树；`/odometry/simulation` 明示物理真值。
这里不运行 AMCL、FAST-LIO 或真实传感器定位。
controller/behavior → `/cmd_vel_nav` → 唯一 velocity smoother → `/cmd_vel`
→ ros_gz_bridge → 模拟车。没有 ros2_control controller_manager；底盘由现有 Gazebo 插件模拟。
Nav2 lifecycle manager 按 map、controller、planner、behavior、smoother、navigator 顺序配置/激活，
客户端等待 navigator active 后再发目标。Nav2 的障碍层仍为 static + inflation；
LiDAR 仅供几何观察，不据此声称动态障碍避让。

在 RViz 的 **Nav2 Goal** 工具选择平地目标，或在相同 source/domain 环境中发送：

```bash
ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 action send_goal \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: -1.0, y: -3.0}, orientation: {w: 1.0}}}}'
```

用启动终端的 Ctrl+C 关闭整套仿真。Simulation Go 不等于 Hardware Go。

## Headless smoke

```bash
ros2 run rm_new_car_simulation smoke_baseline.py --unified --output /tmp/rm2027-smoke
```

它启动相同统一入口，关闭两个 GUI，保留 LiDAR；使用独立 ROS domain/Gazebo partition。
检查生成地图与 `/map` 的占用格一致、点云及 TF、Nav2 active、实际 odometry 订阅、
单目标到达、behavior 经 smoother 的路由、TF owner，以及退出后的进程残留。
返回码 0 才表示通过；日志和 `result.json` 写在 output 下，不提交版本库。
RViz smoke 在有显示服务的环境中给同一命令增加 `--rviz`。完整 GUI 仿真不进入单元测试。

## 历史入口与故障根因

- `rm_navigation_launch` 和 `rm_navigation_bringup` 都是真实源码包，CMake 已有 launch
  安装规则。2026-10-25 的构建只选择 `rm_description`、`rm_nav_config`、
  `rm_new_car_simulation`，所以两个组合包没有出现在那个 install prefix。
- 同日第一次构建的日志记录了 8 份 `rm_description`：正式 `src/`、6 个嵌套 worktree，
  以及历史验证的 `ab_source/src/`。后来 `--base-paths src` 避开了污染，但没有补齐组合包。
- 原 `simulation.launch.py` 的场景分发已原样移到
  `rm_simulation/launch/humble_scenarios.launch.py`；它依赖 Fortress / `ign gazebo 6`，
  仅供原 Humble 环境历史复验。`field_geometry_simulation*.launch.py` 也属于这条旧链，
  不是本机 Jazzy 的替代入口，不为兼容旧命令恢复错误接口。
- 2026-10-27 已移除 3 个无独有数据的 worktree，另外 3 个含子模块/历史证据的 worktree
  及旧 build 内容保留在 `~/.codex/worktrees/rm2027-legacy-build-20261027/`，分支均保留。
  6 条不存在的 `/tmp` worktree 登记已 prune；未动 `historical-debt-round2`。
- 迁移文档提交由 `archive/migration-provenance-e6e85d3` 和
  `archive/migration-provenance-55cd582` 两个 tag 保护，不依赖 reflog/GC。

## 本轮验证

2026-10-27，在本机 `/opt/ros/jazzy`、Gazebo Harmonic 环境实测：

| 验证 | 结果 |
| --- | --- |
| 干净 shell 根目录 discovery | 22 个源码包，全部位于 `src/`，无重复 |
| 从空标准输出目录构建 | 5/5 包成功；两个 launch 包均有正确安装索引和 launch 文件 |
| 安装/依赖/生成资产检查 | 通过；world、URDF、occupancy、PCD、manifest 可生成 |
| Headless 完整链 | 通过，LiDAR 2880 点/帧，最近 10 帧 TF 查询全部成功 |
| 静态场景 | `/map` 与生成 PGM 的占用格一致，600×340，0.05 m/cell |
| Nav2 goal | `(-3,-3)` → `(-1,-3)`，action SUCCEEDED；最终距离 0.122 m |
| RViz 完整链 | OpenGL 4.6，模型加载无错误；同目标 SUCCEEDED，最终距离 0.126 m |
| 命令与 TF owner | controller/behavior 经唯一 smoother；两个根 TF edge 各一个 owner |
| Ctrl+C shutdown | 两种运行均返回 0，无节点异常退出、无残留进程 |
| 相关回归 | 既有 3 项 geometry 测试通过；clean 的未管理目录/嵌套 Git/symlink 拒绝保护通过 |

运行中修复了 URDF mesh 缺少 `file://` URI 导致的 RViz 模型加载失败，以及 Python TF
脚本在异步关闭 context 时的订阅退出异常。smoke 现在将模型错误和节点异常退出判为失败。
决定性日志和结果分别在 `/tmp/rm2027-smoke-headless-final-20261027/` 与
`/tmp/rm2027-smoke-rviz-fixed-20261027/`，为本机临时产物，不进入版本资产。

结论仅为上述静态场景的 Integration Simulation Go。Gazebo 独立 GUI 窗口未单独验收；
RViz 使用真实显示服务验证。未验证硬件、真实定位、动态避障、完整赛事场地或多层自主通行；
这也不是全仓 Humble 软件栈的构建通过。


## 事件重规划候选

本分支从统一仿真基线增加显式候选，不改变默认 `navigation_policy:=baseline`：

```bash
tools/build_simulation.sh
source install/setup.bash
ros2 launch rm_navigation_bringup simulation.launch.py navigation_policy:=event_driven
```

事件重规划阶段构建范围为 8 包，包含 Mission、interfaces、pursuit；
本分支的 LIO 健康候选扩大到下文所述 10 包。Humble 的 Mission 使用
BT.CPP 3，Jazzy 使用 BT.CPP 4；本轮实际构建/运行环境为 Jazzy。
事件配置保持生成的 CAD footprint、物理模型、Navfn 和 MPPI；增加同一 CAD 雷达位置的
水平扫描仿真夹具（360 rays，既有 10 Hz）供 local/global ObstacleLayer。原始 3D
点云继续独立发布。0.5 m 最小标记距离排除偏置传感器看到的车体；标记 8 m、清除 10 m
是本次软件验证范围，不是新车真实传感器覆盖承诺。

Mission 启动为 disabled/hold，仿真定位状态来自显式 truth fixture。候选运行时 RViz
没有 GoalTool，不运行原来的直接发 action 的 smoke。预置巡逻点 `(-1,-3,0)` 与
home `(-3,-3,0)` 仅属于本场景，可使用 `/mission/set_mode` 选择巡逻/home/hold；
动态测试探针经原子 pursuit candidate 提供目标，绝不直接调用 NavigateToPose。

```bash
python3 src/rm_new_car_simulation/scripts/validate_event_replanning.py \
  --scenario blocked --output /tmp/rm2027-event-blocked
```

可选场景：`normal blocked no_route removed local repeated session home safety new_goal`。
每次使用独立 ROS domain/Gazebo partition；日志、完整路径、BT 失败原因、action UUID
与状态时间线和退出结果写在 output，不提交大体积日志。`repeated` 场景明确注入
100 m / 0.8 s 不可能满足的 progress checker 条件，仅验证失败预算，不修改正式参数。
其他场景采用 0.10 m / 3 s 平移进度（两格地图；纯转向不能续期），Wait 1.5 s，3 次重试。

[本轮证据、旧实现语义及遗漏能力审计](../validation/event_replanning_port_20261027.md)。
该选项证明软件接口与受控几何夹具下行为，不授予任何硬件运动权限。

## LIO 输入健康候选

同一构建脚本现包含 10 包，增加 localization adapters / relocalization bridge。
默认 truth 与事件重规划入口保持原行为。运行完整健康链测试：

```bash
tools/build_simulation.sh
source install/setup.bash
python3 src/rm_localization_adapters/test/test_input_health_runtime.py \
  --upstream --finite-difference --output /tmp/rm2027-lio-health-runtime
python3 src/rm_new_car_simulation/scripts/validate_lio_health_navigation.py \
  --output /tmp/rm2027-lio-health-navigation
```

第二个脚本使用 `navigation_policy:=event_driven localization_source:=lio_health_fixture`，
给真实 adapter/bridge 提供可控 Gazebo 派生 raw odometry 与 global pose。
Nav2 只消费经过健康门的 `/odometry/lio`；truth TF/status 被关闭。脚本覆盖导航中跳变，
以及 FollowPath 失败等待中的断流，检测隔离、hold、停止继续规划、旧 action 终止后重建
路径、恢复到达和进程退出。不单独运行 fixture publisher 时，该组合保持无效/hold。
不要把测试真值接入硬件域；配置内的限值仅用于此软件验证。

[机制、F1–F12、历史证据与限制](../validation/lio_input_health_port_20261028.md)。

### Correction gate 定向验证

在已完成本页统一构建/source 的 Jazzy 环境中：

```bash
python3 src/rm_relocalization_bridge/test/test_correction_runtime.py --output /tmp/correction_runtime
python3 src/rm_new_car_simulation/scripts/validate_correction_navigation.py --output /tmp/correction_navigation
```

第一个运行真实 adapter / quality gate / correction bridge，验证 C1–C12、合法大初值、
显式新 session、运动与渐进漂移。第二个自动选择
`navigation_policy:=event_driven localization_source:=correction_gate_fixture`，验证
Mission / Nav2 中 C13–C15：拒绝、显式会话撤销、Wait 中锁存及恢复。各脚本使用隔离
ROS domain，第二个另隔离 Gazebo partition，并清理自己的进程。结果与日志留在 `/tmp`，
不进入 Git。来源是物理真值注入夹具，不是真实 AMCL/LIO 算法精度验收。

手工运行该 profile 必须另提供测试输入，否则 invalid 是预期行为。显式重定位的服务
顺序和源时间限制见 [bridge 合同](../../src/rm_relocalization_bridge/README.md#optional-correction-gate-and-explicit-relocalization)；
不要通过只发布 `/initialpose` 或调大普通创新门限解除锁存。
