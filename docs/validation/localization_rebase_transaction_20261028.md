# 显式 map→odom rebase transaction

2026-10-28 开始，2026-10-29 收尾；Integration；基线 `63aab10`（代码基线 `9c19560`）。
本轮先反例，后最小修复；不实现 reseed、backend reset、source selection 或自动恢复编排。

## 已复现的缺口

真实 adapter、quality gate、bridge、Mission，软件 NavigateToPose server 接受 cancel 后
延迟 2 s 才 terminal。原 C=+20 m，通过现有 prepare/new-backend 准入改为 −15 m。
运行 `test_rebase_ordering_runtime.py --expect-gap`：TF 在旧 action 仍 active 时改变，
比 terminal 早 1.839855 s；replacement 确实在 terminal 后，迟到 success 没推进巡逻点。
四个进程均正常退出。证明 commit barrier 缺失，不声称已测得真实 controller 暴冲。
首次受限沙箱无法创建 DDS socket；用同一隔离 ROS domain 脚本在允许本机 socket 的环境
复核成功。这是环境限制，不是定位失败。

## 目标事务合同

`ACTIVE → PREPARE(合格候选冻结) → REVOKED/DRAIN_OLD_NAV → COMMIT_MAP_ODOM
→ PRIME_NEW_SESSION → ACTIVE`。任一步失效停留 invalid，显式新请求才可重试。

- 普通小 correction 保留现有 0.4 m / 0.3 rad fixture 门，不换 session。
- 显式请求使用 bridge 已观察、quality-gated、同刻 odom 匹配且连续一致的候选；请求
  绑定旧 canonical session 与候选时刻，不能提交调用方未经验证的任意 pose。
- 只在候选合格后预留新 session、撤销 validity；bridge 保持旧 C，不发布新 C。
- Mission 保留任务意图，等待旧 Navigate/Spin 及相关 Nav2 子 action 终止。cancel ACK
  不能作为 terminal。事务回执必须绑定当前 bridge producer/session，旧回执无效。
- commit 前复核源、健康、同一候选几何与新测量有效期；连续 odom 不重置，C 由同一
  bridge 在互斥区内替换。TF 在 commit 后可供缓存 prime，但 validity 尚未恢复。
- 只失效 map 下的动态障碍缓存，保留 static map 与 odom 局部障碍、rolling window、
  footprint 几何；旧传感器时刻不得重新填入新全局障碍层。缓存完成新代数据后才授权。
- Mission 在新 session 对当前意图重新 ComputePath；旧 action/path 不保留。失败/超时
  不自动回滚授权，不自动尝试其他 source。

## 数学与缓存审计

`C_new = T_map_base_new(t) * inverse(T_odom_base(t))`，完整 SE(3) 合成；平面限制由已选
quality gate 负责，不在 bridge 偷偷投影。yaw residual 使用最短角；当前 odom cache
为有界最近邻匹配（默认 ±0.05 s），没有插值，不用 now 的 odom 配旧 pose。该时间误差
仍需真实时间同步验证。map-only commit 不改变 odom/base 或其 twist 的差分历史。

全局 ObstacleLayer 在观测进入时已转换到 map，并保存栅格；改 TF 不会搬移旧栅格。
`observation_persistence=0` 也保留最近一帧，而非空缓存；普通 reset 清栅格、重置
更新时间，未清 observation list。因此不能仅调用 clear_entire_costmap 证明代次隔离。
来源：[ObservationBuffer](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/src/observation_buffer.cpp)、
[ObstacleLayer](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/plugins/obstacle_layer.cpp)。
局部障碍存在连续 odom 坐标下，保留；静态地图仍是同一 map/revision，应保留而非清空。

## 验证结果

以下为本轮实际执行记录，数值只说明受控夹具下的集成行为。原始过程日志保留在临时
输出目录，不进入 Git；长期保留本表、可重复运行的测试及实现。

### R1–R7

| 项目 | 结果与关键证据 |
| --- | --- |
| R1 TF 先改 | **缺口成立**：原 admission-only 路径的 C 在旧 Navigate action terminal 前 1.839855 s 改变。未声称该软件反例证明了实车暴冲。修复后等待 terminal；物理 active FollowPath 场景也检查此顺序。 |
| R2 cancel barrier | 修复后软件提交晚于 terminal 约 0.250 s；实际 Nav2 planner 延迟夹具中，上层 Navigate 在 5.6618 s terminal，但 ComputePath 到 7.5446 s 才 terminal，C 在 7.6662 s 提交。证明只等上层 action 或 cancel ACK 都不足够。 |
| R3 迟到 planner | 延迟旧 Navfn 结果，子 action drain 完成后才提交，新会话重新 ComputePath；旧结果不成为新 FollowPath 的路径。 |
| R4 迟到 controller/success | 软件 action 故意在取消后返回 SUCCEEDED，Mission 不推进旧巡逻点；真实 controller 场景上层在 6.1974 s terminal，FollowPath 在 6.5253 s terminal，C 在 6.6951 s 提交。 |
| R5 cache | 实际 ObstacleLayer/ObservationBuffer 测试覆盖旧栅格失效、旧观测晚到不回填、新观测回填及第二次 cutoff；独立 static master cost 保留。仿真用真实全局层回执，局部层保持 odom frame。 |
| R6 twist | odom 不重置，不从跳变的 global pose 求速度。event 场景事务窗口最大 odom 步长 0.009009 m、速度 0.450474 m/s；另有 SE(3)、yaw wrap、过期/不匹配 odom 单元回归。 |
| R7 repeated | 两次显式事务 session 单调递增；第一事务回执不能提交第二事务，busy 请求和陈旧候选被拒绝。连续两次物理 rebase 后 3 次规划到达同一 Mission 目标。 |

### S1–S12

每个场景独立 ROS domain/Gazebo partition。统一场景目标为 (−1, −3) m，到达误差
门限 0.25 m；yaw 偏差绕出生点旋转，避免测试初始位姿落出走廊。候选来自同时刻物理
真值加受控偏差，并非 AMCL/GICP 定位精度测试。

| 项目 | 场景 / 结果 | 到达误差 m | ComputePath 数 |
| --- | --- | ---: | ---: |
| S1 normal | 无 rebase，通过 | 0.131204 | 1 |
| S2 translation | 0.8 m 平移修正，通过 | 0.127132 | 2 |
| S3 yaw | 0.5 rad 修正，通过 | 0.124588 | 2 |
| S4 SE(2) | 0.6 m、0.3 m、0.5 rad 联合修正，通过 | 0.133283 | 2 |
| S5 active FollowPath | S2/S3/S4 均从 active FollowPath 发起，旧 action 终止后提交 | 同上 | 同上 |
| S6 planner pending | 已计算的旧路径延迟 2 s 返回，通过 | 0.123164 | 2 |
| S7 event replan | progress failure 后 Wait 期间发起，Wait 先终止，无规划风暴 | 0.136073 | 2 |
| S8 late callbacks | controller cancel 延迟装饰夹具及软件迟到 success，通过 | 0.136422 | 2 |
| S9 repeated | 最终连续两次合法 rebase，真实 TF owner 探针复核通过 | 0.117019 | 3 |
| S10 invalid | NaN 候选拒绝，不变 session/C；随后合法流程到达 | 0.125343 | 1 |
| S11 latched | 普通好帧不能解锁；仅显式、连续一致、合格候选事务可解锁 | 同 S10 | 同 S10 |
| S12 shutdown | 9 个场景均有正常退出记录，但另有两次 Gazebo 退出超时；稳定性未通过 | — | — |

保留的异常：首次 translation 功能通过（误差 0.126329 m），但 Gazebo 退出超时，
被测试脚本清理；同配置最小复核正常退出，对应表中 S2。最终 repeated 复核再次发生
同类退出超时：功能通过、误差 0.115331 m，但 Gazebo 未响应 SIGINT/SIGTERM，最终
退出码 −9；其余全部节点正常退出。一次同配置复核正常退出，对应表中 S9。
这两次失败不能被成功复核覆盖；退出问题根因未定位，S12 保留 Modify。
另一次 TF owner 探针尝试误用 Python message-info API，测试提前异常，
不计入功能验收；改用仓库已有 C++ GID 探针复核，两个 canonical edge 均只有一个
publisher，探针退出码 0。controller 延迟夹具设置 1 s，但 TF 失效可能使旧 controller
提前终止；R4 记录的是实际观察到的时间，不宣称保持足 1 s。

### 直接回归与最终复核

- 统一 Jazzy 仿真构建：10 个 package 成功；bringup 检查覆盖 22 个源码 package、
  10 个 install prefix 与所需资产/依赖。
- bridge：27 个单元用例及 4 个测试包装，共 31 项，0 failure/error/skip；nav config：
  1 个实际缓存用例及 1 个包装，共 2 项，0 failure/error/skip。
- 既有 Mission/Arbitration runtime 回归：1 passed（28.12 s）。
- 最终审阅新增 PRIME 边界反例：相对准备锚点都在 0.4 m 内的两个候选，彼此却可
  相差 0.78 m。恢复 ACTIVE 前必须再次相对**已提交 C**通过普通 gate，不能夹带
  第二次大修正；失败保持 invalid，不自动恢复。最终软件回归通过该反例、旧事务
  回执重放、busy/陈旧请求和迟到 success；C 比 terminal 晚 0.209553 s 提交，4 个
  生产节点退出码均 0。
- 既有 correction runtime C1–C12（含 NaN/Inf、时序、源不可信、7 rad/s、累计漂移
  及中断一致性窗口）最终回归全部通过，3 个节点退出码均 0。
- 最终 repeated 正常退出复核：session 1677000000 → 2525000000 ns，commit 在
  6.2727/7.0727 s；旧 FollowPath 在 6.0879/6.6726 s 已 terminal。事务窗口最大
  odom 步长 0.001922 m、速度 0.096071 m/s；map→odom 与 odom→base_link 的实际
  publisher GID 数均为 1，validity 仅 bridge 发布，探针退出码 0，无存活子进程。
- 4 个修改/新增 Python 文件及 2 个 plugin XML 解析通过；最终 diff 无空白错误。

## 适用边界与冻结决策

**裁决：Software Go；事务仿真功能通过，整体 Simulation Modify（S12 退出不稳定）；
Hardware Blocked。** 不扩大到无条件完整仿真验收，也不通过忽略退出码掩盖失败。
当前实现及最小复核已足以关闭事务顺序缺口；Gazebo 退出问题保留为后续环境/生命周期
调查项，本轮不改无关仿真器或追加恢复状态机。

软件及功能结论范围仅为 `rebase_fixture` 完整组合、已安装 Jazzy Nav2、单一 Mission
拥有列出的 action、同一 map/revision、受控且保留采集时间的 backend。原
`correction_gate_fixture` admission reset 仍不是导航中大修正事务；不得把本结论
扩展到该旧入口、任意远端 action server、重写时间戳的旧算法结果或认证 ROS 图。

Mission-level intent 保留，旧 action/path 不保留；新 session 重新规划。未新增第二个
TF、canonical odom、定位 validity、最终速度或 Mission intent owner。全局动态层
按 commit cutoff 失效，static map、odom 局部层、rolling window、footprint 保留。
观测缺失或持久旧观测尚未过期会阻止 prime，不能靠超时假装成功。

**Hardware Blocked**：没有真实 AMCL/GICP candidate、真实时间同步、大修正时实际
停车/制动、真实 TF/sensor/costmap 连续性及最终底盘运动授权证据。软件 terminal
只证明 action 生命周期结束，不证明车辆机械静止。当前 ±0.05 s 采样容差和创新门限
都是集成候选参数；本轮不访问真实串口，不重启或重置 FAST-LIO。

没有新车真实恢复数据，继续 autonomous reseed / orchestration 缺乏收益证据。
事务逻辑已补齐并通过直接验证，定位恢复开发线冻结为
**`FROZEN_WAIT_HARDWARE_RECOVERY_DATA`**；不进入 reseed。S12 的仿真门禁独立保留，
冻结不表示该问题已解决。下一方向为 Mission / Arbitration 系统验证，本轮完成直接
相关既有回归；完整仿真验收前仍需处理上述退出不稳定，不追加新的恢复开发阶段。

## 最小复现与 Git

构建及运行入口见 [仿真操作](../operations/simulation.md#显式-map-only-rebase-定向验证)。
在 `source install/setup.bash` 后执行：

```bash
python3 src/rm_relocalization_bridge/test/test_rebase_ordering_runtime.py --expect-gap --output /tmp/rebase-before
python3 src/rm_relocalization_bridge/test/test_rebase_ordering_runtime.py --output /tmp/rebase-after
python3 src/rm_relocalization_bridge/test/test_correction_runtime.py --output /tmp/rebase-correction
python3 src/rm_new_car_simulation/scripts/validate_rebase_navigation.py --scenario repeated --output /tmp/rebase-repeat
```

物理矩阵其余参数为 `normal translation yaw combined planner controller event invalid`。
`--expect-gap` 选择旧配置用于反例，不是安全运行入口。所有夹具清理自己的进程；
不要与实车同域运行。临时输出不作为长期资产，关键观测已写入上表。

分支 `codex/localization-rebase-transaction`，基于 `63aab10`；本报告与实现同次提交，
提交号以 Git 为准，避免自引用。未 merge/push；main 必须保持
`d37f8d6e5ebe53533b7fc62aca8bb84392bee31b`。
