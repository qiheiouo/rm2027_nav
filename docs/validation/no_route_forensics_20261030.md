# no_route FollowPath Timeout Forensics

2026-10-30，Integration / isolated simulation。基于
`29a2fa93bcaff08f04ec3dc8d77b591bf042fed3`，诊断分支
`codex/no-route-terminal-forensics`。没有访问硬件、修改生产节点、调整导航参数或改写原验收。

## 裁决与停止边界

**Case B：Mission / Arbitration Software Go；Simulation Modify；Hardware Blocked。**
原 `no_route` 的 70 s 判据在最终带观测复核中再次失败，但核心软件合同正常：第四次
Follow 在窗口结束时仍有连续反馈和米级平移，没有最后一次 FAILURE 可向上传播；
后续经 Mission HOLD，真实 controller 返回 CANCELED，Mission 收到终态并撤权。
根因定位为 **E：harness 把有限失败重试数误当作绝对完成时限**；D：动态观测不完整、
机器人仍可绕围挡移动，是使这一假设失效的场景条件。没有证据支持 A/B/C 的终态链故障。

独立保留 **OPEN_SIM_INFRASTRUCTURE — no_route harness deadline assumption**：
原场景和失败结果不删除，70 s 不延长，也不把此次失败改成 pass。该场景不能单独证明
“物理无路必在给定墙钟窗口触发四次 controller failure”。不改变正式 Mission semantics，
不把有限观测写成“机器人将永远运动”的证明。若未来需要绝对任务 deadline，必须另行
定义业务合同；本轮不新增这个功能。

本轮没有生产修复，仅增加诊断工具、实际 BT 边界测试及合同澄清。按用户 Case B 条件，
软件系统线标记 **`SYSTEM_RELIABILITY_SOFTWARE_FROZEN_WAIT_HARDWARE`**。
可以进入独立经典规划/启发式搜索 **Research**；本轮只给出准入结论，不启动新研究。
Simulation 仍有 no_route harness issue 和独立 S12，不能宣称“所有物理验收通过、只剩 S12”。
冻结不包含任何实机/比赛 Go，也不证明任意不可达目标会在有限绝对时长内自动结束。

## 条件与 A/B

- A：当前候选 `29a2fa9`；B：父提交 `5f52d8e06310e4932863d11f306b6647d2e0d05a`。
- B 仅把父提交 `src/rm_competition_mission` 导出并在独立目录构建，source 其 overlay；
  已核对 `ros2 pkg prefix rm_competition_mission` 指向该目录。该包的公共接口不变。
  两提交间唯一生产 C++ 变化是 Mission；BT、Nav2、场景、launch、配置及原 event validator 相同。
- 全部使用原 `no_route`：目标 `(2,-3)`，首次规划后动态生成原四面围挡；原 70 wall-second
  exhaustion 窗口（四次生成完成后开始），原 Wait/retry/速度/障碍位置不变。
- ROS Jazzy / Nav2 1.3.10；Navfn + MPPI；`regenerate_noises=false`。源码使用 xtensor
  默认 mt19937（默认初始化），没有新设随机种子。原场景没有 seed 参数，Gazebo launch
  没有显式 seed，因此只能保持原默认随机行为，**不能声称恢复了历史运行的精确 Gazebo seed
  或确定性调度**。传感器、控制循环与异步障碍插入的相位仍可不同。
- 原配置没有 referee publisher，A 的 life generation 始终 0，Mission goal generation=1、
  localization session=100000000。B 尚无这些 detail 字段；以唯一 Navigate UUID 和状态核对。
- 所有 Navigate 请求都由 Mission 产生。诊断探针只订阅反馈、请求 GetResult 和读取参数/
  lifecycle；没有 SendGoal/CancelGoal client。ROS 图把 GetResult 也列为 action client，
  所以在原所有权检查完成后才接入它，保留原检查且不形成第二个任务 owner。

| 轮次 | 版本/观测 | 第四 Follow 执行开始→终态，s | 结果 |
| --- | --- | --- | --- |
| A1 | 当前，原脚本 | 50.8466→66.8467 ABORTED | 原判据通过 |
| B1 | 父提交，原脚本 | BT 最后 FAILURE 66.4207 | 原判据通过 |
| A2 | 当前，附加只读观测 | 51.7733→63.4741 ABORTED | 原判据通过 |
| B2 | 父提交，附加只读观测 | 53.4447→64.1451 ABORTED | 原判据通过 |
| A3 | 当前，附加只读观测 | 47.2571→76.2587 ABORTED | 原判据通过 |
| B3 | 父提交，附加只读观测 | 47.5635→65.0616 ABORTED | 原判据通过 |

时间均为各自 Probe 建立后的 monotonic wall elapsed，不是跨轮绝对时间。
每轮 1 Navigate / 4 Compute / 4 Follow / 3 Wait；最终 goal_retry_limit，继续观察 5 s 无新请求。
六轮均退出干净、无异常进程/残留。A3 第四次持续约 29 s，仍按原 deadline 通过；
可见 3 s progress allowance 并不等价于 3 s action duration。

最初三组配对没有复现；最终脚本复核捕获超时，故最终分类为 **间歇复现**，不是稳定复现
或已经修复。追加的父提交对照见末节；不能仅用前三组成功对照排除回归。

一次早期探针启动被原 action-owner 检查挡住（GetResult 客户端被图 API 列为 action client），
未发送导航请求；已修正观测接入时机，**不计入有效 no_route 样本**。
中途执行环境中断清空了 `/tmp`；上述六轮和旧轨迹的数字来自中断前已读取的结果。
全量原始文件不再可用。最终诊断脚本复核与确定性耗尽结果保存在本仓忽略的
`log/no-route-forensics/`，下文保存最小关键证据；不把原始日志/地图提交 Git。

## 第四个 Follow：失败历史与闭合样本

旧复核 UUID `4df9a70bcbfe55147786064e7162ad70`：

| 时间，s | 已有观测 |
| --- | --- |
| 7.3613→19.6627 | 第 1 Follow 执行→ABORTED，之后 Wait/Compute |
| 21.3208→31.4224 | 第 2 Follow 执行→ABORTED，之后 Wait/Compute |
| 33.0812→48.2819 | 第 3 Follow 执行→ABORTED，之后第 3 Wait |
| 49.9341 | 第 4 条路径发布，199 点，Compute 成功 |
| 49.9417 | 第 4 Follow 首次观察到 EXECUTING（不是 SendGoal ack 的精确时刻） |
| 49.9549→78.7560 | 位姿持续变化，累计采样折线 5.4225 m；没有第四次 progress FAILURE 日志 |
| 78.7560 | 测试停止前仍 status=2、Mission goal_active，authority=1；不是已宣告任务完成 |
| 关闭阶段 | 日志出现 Follow cancel 请求及 Nav2 shutdown；探针已不 spin，缺少可靠终态回执 |

另一旧失败 UUID `f69811338d917cace501fd066d9875fc`：54.3738 s 开始，78.7649 s
仍在运动；采样累计 5.7802 m。没有第 5 Compute/第 4 Wait；不能把 shutdown 后的 cancel
或 result-wait 警告当成超时前已存在的 action 故障。旧数据没有 Follow feedback 通道，
因此其反馈是否连续、cancel ACK 与最终 result **没有直接证据**，不补造完整时间线。

A2 闭合样本 UUID `d16ed078c257e98a7922490f1875f286`：

| 时间，s | 观测 |
| --- | --- |
| 51.7605 | 第 4 路径发布 |
| 51.7733 | Follow EXECUTING |
| 51.7752→63.3745 | 117 个 Follow feedback；ROS time 45.894→57.266 s；路径剩余距离 4.764→1.596 m |
| 63.4741 / 63.4749 | status ABORTED / GetResult 返回 status=6、error_code=105（FAILED_TO_MAKE_PROGRESS） |
| 63.4933 | BT FollowPath RUNNING→FAILURE |
| 63.5039 | Navigate ABORTED |
| 63.5719 | Mission goal_retry_limit，navigation_authority=0；意图 branch 保留 pursuit |
| 68.5919 | 完成额外 5 s 稳定观察，无新 goal/Wait/Compute |

A2 的第四次移动 3.1623 m，命令平移速度中位数 0.2962 m/s、最大 0.5692 m/s。
planner/controller/bt_navigator/两个 costmap 在导航阶段均为 active；局部/全局地图收到
207/103 次发布或更新（全场景计数）。Nav feedback 的 number_of_recoveries 最终为 3。
反馈发生在控制循环通过 costmap-current 与 progress 检查之后；这个样本没有持续卡在
costmap wait，也没有 action result 遗失。原始取消服务 introspection 未启用；诊断脚本
若遇到超时，会保留原失败后通过 Mission HOLD 单独检查 CANCELING/CANCELED，不改原 verdict。

## 实际合同与分层判断

权威 [仲裁合同](../contracts/navigation_target_arbitration_2027.md) 和正式
[event_driven_replanning.xml](../../src/rm_nav_config/behavior_trees/event_driven_replanning.xml)
一致：Sequence(Compute, Follow) 的 FAILURE 才进入 1.5 s Wait，最多 3 次 recovery；
每次重新执行 Compute，获得独立 action UUID/path。最后一次失败直接使 RecoveryNode FAILURE，
Nav2 结束 Navigate，Mission 外层预算 1 把同意图锁在 goal_retry_limit。

实际 [Nav2 RecoveryNode 1.3.10](https://github.com/ros-navigation/navigation2/blob/1.3.10/nav2_behavior_tree/plugins/control/recovery_node.cpp)
在 child RUNNING 时直接返回 RUNNING。没有额外“重试已用完就立即取消第 4 次”的合同。
第 4 次返回 FAILURE 后子 action 已 terminal，不需要第二次 cancel；owner 主动撤销时
[BtActionNode halt](https://github.com/ros-navigation/navigation2/blob/1.3.10/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp)
才取消尚在运行的 Follow。没有等待第 5 planner event 的分支。

| 层 | 结论与限制 |
| --- | --- |
| A Mission/Arbitration | 已测 owner=1，generation/session 没有变化；成功样本及时消费 terminal、authority=0。未观察到屏障或代次缺陷。 |
| B BT | 正式 XML + 已安装 RecoveryNode 测试直接通过；第四次 failure 立即传播，不再 Wait/Compute。没有发现预算耗尽空档。 |
| C Controller/action | 最终失败窗口仍有 307 个正常反馈，最后失败没有发生；HOLD 后立即有 CANCELED result。运行参数正确，未见 action/server 故障；不能证明任意不可达目标上的无限运动。 |
| D Costmap/scenario | 物理目标完全围住，但动态地图可保留被遮挡的空格；并不保证 planner 返回 no-path，四次均得到候选。A2 最终地图后墙/下墙中心仍为 0，前墙/上墙为 99，目标格为 0。该末帧不是规划瞬间地图，不能据此断言 planner 穿越当时已知致命格。 |
| E Simulation/harness | 最终复现确认 70 s 到期时 ROS clock/反馈/地图持续推进，controller 的进度检查仍通过。没有对应的 action 总时限合同；等待终态的 harness 假设不成立。这是此次 timeout 的直接来源。 |

## Progress checker 与 exhaustion

实际读到的参数：`nav2_controller::SimpleProgressChecker`，radius **0.10 m**、allowance
**3.0 s ROS time**，controller **10 Hz**，failure_tolerance **0.3 s**、costmap_update_timeout
**0.3 s**，MPPI regenerate_noises=false。与 [1.3.10 源码](https://github.com/ros-navigation/navigation2/blob/1.3.10/nav2_controller/plugins/simple_progress_checker.cpp)
一致：相对 baseline 平移 **大于** radius 就重设基准和时钟；不要求目标距离减小，纯旋转不续期。

旧两次失败的采样平移速度中位数分别 0.2056、0.1100 m/s，最大 0.5557、0.6496 m/s；
不是只有接近零命令/毫米噪声。对保存的约 5 Hz odometry 做 0.10 m 基准重建，得到
44/43 次重置候选，最长间隔 2.0438/2.6560 wall s，末段间隔 0.6118/0.4097 s。
这是**降采样 odometry 推断，不是 controller TF 输入或内部基准的逐周期证明**。
未证明“虚假 progress 永远不 terminal”，没有调整 threshold。

新增 `test_event_retry_exhaustion.cpp` 直接链接已安装 RecoveryNode、加载正式 XML，
只替换 action leaves 为可控 fixture，四项通过：

1. 前 3 次 Follow 失败后，第 4 次保持 RUNNING 100 ticks，不出现第 5 plan；第 4 次失败立即结束。
2. 第 4 次 Compute 失败直接结束，不使用旧 path 执行 Follow。
3. 第 4 次 Follow 成功仍能 SUCCESS，不能误判为已耗尽失败。
4. owner halt 能停止 pending 的第 4 Follow。

已有 `repeated` 物理 fixture 单独使用测试进度条件 100 m / 0.8 s（**不用于 no_route**），
真实 Mission + Nav2 端到端验证第四次失败、外层收束、5 s 无新请求及干净退出。最终结果见下。
这证明已出现 FAILURE 后的 transaction，不证明物理 no_route 必在 70 s 内产生 FAILURE。

## 验证范围与 S12

- rm_nav_config 构建通过；4 个 BT cases 和同包 rebase obstacle layer 回归全部通过
  （CTest 2/2 targets，最终复核 0.78 s）。
- 当前生产候选重新运行 **9 life + 14 system = 23 passed，107.69 s**；包含软件 D1–D12
  对应边界及固定种子 7/19/41 的长序列。没有用它替代 D6/D7 物理交叉验证。
- 本轮没有生产修复，不触发用户第 9 节的修复后全物理回归。normal / blocked / local、
  life_replan / life_rebase、localization/correction/rebase 的原有通过证据继续来自
  [上一轮报告](mission_life_authority_20261029.md)和[事件迁移报告](event_replanning_port_20261027.md)，
  **不是本轮重新运行**；不把本轮说成完整部署矩阵通过。
- S12 继续独立为 **OPEN_SIM_INFRASTRUCTURE — Gazebo shutdown timeout**，权威记录在
  [系统报告](competition_mission_arbitration_system_20261029.md#s12-独立基础设施-issue)。
  本轮全部八次有效 no_route 均干净退出，没有把 active Follow 超时合并到 shutdown issue；也没有关闭 S12。

## 复现与资产

```bash
source install/setup.bash
python3 src/rm_new_car_simulation/scripts/validate_event_replanning.py \
  --scenario no_route --output "$PWD/log/no-route-forensics/original"
python3 src/rm_new_car_simulation/scripts/diagnose_no_route.py \
  --scenario no_route --output "$PWD/log/no-route-forensics/observed"
ctest --test-dir build/rm_nav_config --output-on-failure
python3 src/rm_new_car_simulation/scripts/validate_event_replanning.py \
  --scenario repeated --output "$PWD/log/no-route-forensics/exhaustion"
```

父提交对照：`git archive 5f52d8e src/rm_competition_mission` 导出到临时目录，colcon 使用
独立 build/install/log bases 构建该包；先 source 当前 install，再 source 父包 local_setup。
运行前核对 package prefix；其余 packages 使用相同当前 install。没有切换主 checkout、
覆盖现有 install 或创建额外永久 worktree。首批临时 parent 构建随环境中断丢失；最终对照已在忽略目录重建。

Git 只包含诊断工具、测试、合同澄清和本记录。没有 production fix，原 validator/BT/YAML
没有修改。没有 push/merge；main 保持 `d37f8d6e5ebe53533b7fc62aca8bb84392bee31b`。
最终 commit 以本分支提交为准，不把自身 SHA 写入其内容形成循环。

## 最终持久化复核

### 当前候选最终复现（A4）

原判据 **FAIL**，scenario wall time 78.9450 s；原 verdict 79.0335 s。
这是窗口截止后通过 Mission HOLD 做诊断，绝不把 79 s 后取消当成自然 bounded exhaustion。
原始结果在 `log/no-route-forensics/final-observer/`；选择性记录保存于
[最小机器可读证据](no_route_forensics_20261030.json)，包含各 Compute/Follow/Nav/Wait UUID、
全部状态转换、GetResult、参数及反馈摘要。

第四 Follow UUID `b6613846d6f674c8a46ef7f28d8591f7`，上层 Navigate UUID
`fd7f5e168c0d7545566ffa0a3ec28213`：

| wall elapsed，s | 完整复现链 |
| --- | --- |
| 7.5150→22.1048 | Follow 1 ABORTED，GetResult error_code=105；第 1 Wait 后重算 |
| 23.7628→34.4638 | Follow 2 ABORTED，GetResult error_code=105；第 2 Wait 后重算 |
| 36.1266→46.7238 | Follow 3 ABORTED，GetResult error_code=105；第 3 Wait 后重算 |
| 48.3630→48.3712 | Compute 4 EXECUTING→SUCCEEDED；GetResult error_code=0 |
| ROS 42.358 s | 第 4 Follow goal_info 接受时间戳；不同于表内墙钟域 |
| 48.3877 | 第 4 Follow EXECUTING，BT 也进入 RUNNING |
| 48.3879→78.9850 | 307 个反馈，最大墙钟间隔 0.1015 s；ROS time 42.363→72.352 s |
| 79.0335 | 原 70 s exhaustion 窗口失败，Navigate/Follow 仍 active；Mission goal_active |
| 79.3054 | **后置诊断干预**：发送 Mission HOLD，请求取消当前意图 |
| 79.3813 | Mission goal_canceling，authority=0 |
| 79.3829→79.3843 | Navigate CANCELING→CANCELED，证明取消被上层 action server 接受 |
| 79.3845 / 79.3863 | Follow CANCELED / GetResult status=5、error_code=0；controller 日志明确 Cancellation was successful |
| 79.3864 | Navigate GetResult status=5 |
| 79.4809 | Mission HOLD / goal_cancel_complete，所有观测 action 均 terminal |
| 79.5236 起 | 最终速度命令为零，保持至观察结束；不作为硬件停车证明 |

截止前第四次累计 5.9205 m，平移命令中位数 0.3187 m/s、最大 0.6506 m/s；
不是毫米噪声。按完整 odometry 重建 54 个 >0.10 m 的基准更新候选，最长间隔
2.5304 wall s；仍明确不是 private checker baseline 的原位采样。
更直接的证据是 10 Hz feedback：实际 controller 在每次发布前已通过 progress check，
若失败会抛出 FailedToMakeProgress 并终止，不会继续正常反馈。

第四次期间五个 lifecycle 节点均 active，local/global costmap 分别 102/51 次更新；
ROS clock 从 wall 50.0605 s / ROS 44 s 推进到 wall 78.6265 s / ROS 72 s，无暂停。
Nav feedback 的 recovery count 固定为 3，无第 5 Compute、第 4 Wait，也没有提前宣布完成。
第四次没有 recovery/failure；截止前未观察到取消，HOLD 干预后收到取消终态。GetResult observer 最终收到
全部 12 个 action 结果，干净退出，无 residual。物理围挡后墙中心仍未被地图标记，
但不能用末帧反推 planner 当时的完整地图。

### 确定性 exhaustion 集成

独立 `repeated` fixture：**PASS**，1 Navigate / 4 Compute / 4 Follow / 3 Wait。
第四 Follow `2bc8f0d86d0932cd8eb724a3f19ec050` 于 15.8368→16.7387 s ABORTED，
BT 最终 FAILURE 16.7566 s；Navigate `5598b6fde0092c870b3b946baf37c40a`
于 16.7667 s ABORTED，Mission 16.8350 s goal_retry_limit / authority=0。
所有 12 个 action terminal；继续 5 s 无新请求；外部 Navigate owner=Mission，最大活动
Navigate=1；无异常进程/残留。这项结果没有替换原 no_route 的失败。

### 最后父提交对照

父提交 B4 原判据 **PASS**；第四 Follow UUID `11c6020a4a981cf5c3de7b9dba247891`，
43.5983→60.6004 s ABORTED，GetResult 60.6014 s / error_code=105；170 个反馈，
Mission 60.6965 s goal_retry_limit。scenario wall time 65.6441 s，含额外 5 s 无新请求，
干净退出。独立 overlay 的 package prefix 也写入机器可读证据。

最终原配置有效样本 **8 次：A 3 pass / 1 timeout，B 4 pass / 0 timeout**；
两轮历史 A timeout 另列，不重复计入。没有两边失败的实验事实，故**不标记已证实的
`pre-existing regression`**，也不声称两提交的失败概率相同。

对“仅新提交捕获失败”复核实际因果链：该 profile 无 referee 输入，life_generation 恒为 0；
DEAD/REVIVED handler 未执行，追加的 life-result 检查在尚无 result 时也未执行。
HOLD 之前 Mission generation/session/意图保持不变，没有新 Navigate 或 cancel；
29a2fa9 没有修改 planner/controller/BT/参数或 Follow 的调用者。最初的 HOLD watermark
修改在单调递增时钟下等价于父提交。新增状态文本不会把持续有反馈的 RUNNING 变成
缺失 terminal。轨迹/观测相位差异和随机/调度限制仍在，但本次 timeout 已由
“有合法 progress、没有总 deadline”的合同和 wire 结果直接解释；不归因于生命授权缺陷。

最终 A4/B4、确定性 fixture 均干净退出，S12 保持独立。原 no_route **没有修复为全通过**，
其 deadline 验收留给独立 harness issue；软件冻结依据是 Case B 的合同闭合证据。

观测限制：cancel 服务未开启 wire introspection；表内 wall time 是探针接收时间，
Navigate CANCELING 与 Follow CANCELED/result 加 controller 日志证明服务器处理了取消，
不伪造精确 CancelGoal request/ack 时间。A4 在记录原 verdict 后曾先序列化地图（约
0.272 s），故不把后置 HOLD 当作精确取消延迟基准。最终诊断工具把磁盘序列化推迟到
终态后，先复制紧凑地图数组，避免观测过程打断候选续租；这不影响已经记录的原超时。
源码语法及导入、快照与存储检查通过；不把这一工具整理当作生产修复。
