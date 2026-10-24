# 软件清债候选独立 merge-readiness review（2026-10-24）

## 决策与基线

**原始 `fed2cff`：Modify。加上本 review 分支的边界修正后：Go，仅限 main 的软件集成；不是实车/比赛 Deployment Go。**

独立比较 `main@d37f8d6` 到完整候选链
`399840c → e761239 → ddf988b → 8e53cc6 → fed2cff`。未将原 Work 报告中的通过数字或
Go 判断作为证据；重新检查源码、固定 upstream 和实际 Humble launch，并重新构建/运行测试。

修正位置：`codex/software-debt-review-20261024`；原候选、main 和主工作区均未改动。
没有合并、推送、设备访问、真实传感器输入或物理运动。阶段为 main 合入前 Integration 审查。

## 原候选中的阻塞项

下列行号对应 **fed2cff 修改前**；本 review 已作局部修正。

| 优先级 | 位置 | 触发、影响与独立证据 | 最小修正 |
| --- | --- | --- | --- |
| P1 | `src/rm_lio_bringup/launch/fast_lio_multi_phase2a.launch.py:74–78` | generic GICP 只打开 body flag，master scan flag 仍为 false。固定 upstream `9aa8c79` 的 `laserMapping_bundle.cpp:1346` 要求两者同时为 true，因此没有实际 body 输入。新增 launch 参数反例失败；原合成 cloud 测试绕过了 producer。 | 请求 body 时同时打开 upstream master flag。upstream 会额外发 raw-world scan；明确它不属于 canonical odom，也不供 GICP 消费。默认两旗标关闭保持不变。 |
| P1 | `src/rm_competition_mission/src/competition_mission_node.cpp:179`、`:491`、`:511`；`src/rm_pursuit/rm_pursuit/pursuit_goal_planner.py:115` | 负 stamp/deadline 未校验即构造 ROS Time。真实 Mission 两次 exit -6，planner callback 抛 ValueError；owner 退出不能保证撤销已发 action。未来一小时样本还会先推进 watermark，使正常目标持续被忽略。修改前四个异常/恢复检查失败。 | 构造 ROS Time、更新 watermark 前检查时间格式和 source age；非法 safety stamp 关闭 gate。忽略非法候选不续租旧候选；允许随后正常样本恢复。 |
| P2 | `src/rm_lio_bringup/config/lio_adapter_fast_lio_multi.yaml:15–17` 和 `src/rm_navigation_bringup/launch/phase2a_lio_bringup.launch.py:45–49` | first-source 假设写入共用默认 adapter；dual/bundle、single/async、single/adaptive 都会选中它。dual backend 仍有延迟 rebase，其他方法也未按该假设验收。三个新增 profile 选择检查失败，违背“只调整 single bundle”的范围。 | 恢复共用 YAML，新增专属 single-bundle YAML，仅该组合默认选择。显式 adapter override 继续生效；不修复或提升其他 profile。 |
| P2 | `src/rm_simulation/launch/phase1_5_gazebo.launch.py:140–143` | 仿真开始引用 bringup 的 wrapper，却未声明依赖；仅安装其声明依赖时找不到该入口。直接补依赖又会与 bringup 已有的 simulation 依赖形成环。 | 将实现放入双方已有依赖 `rm_nav_config`，安装 launch 资产和声明 launch 运行依赖；原 bringup 文件保留转发兼容入口。 |

## 各重点的结论

- **AUD-060**：选定 gimbal YAML 的 use_input/input_topic 得以保留，显式 CLI 覆盖最后生效。
  真实 joint state/TF 的 derived、显式 false、定制 topic 三种测试通过。默认 placeholder 未改变。
- **AUD-041/061**：single bundle 关闭延迟 zero-start，first-source 冻结左侧 parent 映射；
  body 点来自 inspected upstream 的 IMU 坐标，backend 内部 `body` 仅映射至 `lio_imu_link`，
  通过源时刻 canonical TF 进入 odom。45 个跨越原 40-scan 边界的合成 odom 样本和非单位
  安装/姿态下的实际 GICP 转换通过。没有公共 body TF，也没有新增 canonical TF owner。
  producer 门控由实际 launch 参数和固定 upstream 条件交叉核对；本次没有运行真实 FAST-LIO 输入链。
- **AUD-014**：source stamp 和 steady receipt 双重租约；零、重复、旧、未来、暂停和时钟回退
  都不能保持 readiness。实际 monitor callback 通过。只检查诊断 readiness，不授予运动权限。
- **AUD-015**：完整 6×6 finite/symmetry/PSD，零方差维的 cross term 约束、rank-deficient 合法矩阵、
  大无关方差均覆盖。`ddf988b` 按每维标准差归一化才闭合尺度漏洞。
- **AUD-055**：实际 Humble controller/behavior 共同发布 `/cmd_vel_nav`；最终 `/cmd_vel`
  publisher 集合严格为 `{velocity_smoother}`，且 smoother 不订阅最终输出；实际输入可经 smoother 输出。
  新的共享 wrapper 保留 node-qualified remap 和非 composed 模式，没有修改调参/模型。
- **arbitration**：任务层仍只有 Mission 的 NavigateToPose/Spin client；pursuit 只有候选。
  pending acceptance、撤权、迟到 handle、取消 ACK 早于 result、取消被拒绝、取消后收到 SUCCEEDED
  均保持 terminal barrier。未观察到并发 active action；取消后旧成功不推进 patrol index。
  stale/lost/producer 消失、同 ID jitter、更新限频和不同 ID replacement 均有真实 runtime 证据。
- **minimal Mission**：一个有 ROS/steady deadline 的临时槽，supply/recovery 用配置、special 用合法 map pose；
  幂等重传不续租、不复活最新已完成任务，替换无恢复栈。home 优先、持续 latch，到达后 hold；
  缺 home 不落入 pursuit/patrol。失权期间无替代 goal，恢复重算未过期意图。测试通过。
- **状态竞争/双 owner**：node 默认单线程执行，action generation 和 pending/active barrier 保留；
  runtime 替代请求均晚于旧终态。没有新增速度、TF、串口 motion owner。此结论不覆盖同域外部
  手动 Nav2 client、dog-hole manager 或另一 Mission；合同继续禁止共同控制。

## 关闭状态与旧车兼容边界

三个实际入口 `competition_2027_navigation`、`old_car_2026_competition`、
`old_car_2026_three_point_spin_test` 使用默认关闭值时全部正常退出，零子进程。
安全配置的临时点仍未设置，Mission 启动 disabled。非 GICP 时 scan flags 保持关闭。

旧车 FAST-LIO、adapter、AMCL baseline/candidate、serial/chassis 和旧车 Nav2 launch 源码均未被该链修改；
旧车 AMCL 四项配置检查与原三点 Spin 六项配置/一项 runtime 检查通过。

**已启用的 Mission 并非行为完全兼容**：旧车 pursuit 现在固定 radar-only，旧 Pose/Bool producer
不再被 Mission 消费；RETURN_HOME 到达/HP 恢复后也不会自动恢复原任务，须显式释放 mode。
这些是候选的明确语义变化，旧 `use_target_mock` 也不能在 radar-only 入口产生有效追踪。
需要 generic mock/研究输入时使用已有 no-hardware/research 入口。不能据关闭状态测试声称旧车实车回归完成。

修正后 dual/async/adaptive 的默认 adapter 已恢复原选择；其延迟 rebase、动态安装或其他已有局限
没有在本轮关闭。自定义 BT 必须显式接入临时任务节点，不在本轮验收范围。

## 独立执行的验证

环境：现有 Humble 镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`，
源码只读挂载、独立结果目录、非 root、network none、private IPC，无设备映射。
从独立输出目录构建九个直接相关包；共享 wrapper 修正后补构建 `rm_nav_config`，重建受影响包。
GICP/本轮 C++ 使用 `-O0 -g0`，不作性能结论。

| 最终检查 | 结果 |
| --- | --- |
| localization adapters | 15 GTest 通过 |
| relocalization math / full covariance | 5 + 12 GTest 通过 |
| old-car AMCL profile compatibility | 4 pytest 通过 |
| bringup launch / actual gimbal / GICP / velocity route / defaults | 17 pytest 通过 |
| Mission 原有 minimal/arbitration/Spin + 新输入/取消边界 | 15 pytest 通过 |
| readiness（含实际 callback）/ pursuit math | 7 pytest 通过 |
| 合计 | 75 项；0 failures、0 errors、0 skipped；10 个不同注册 CTest 均通过 |
| 399840c 原始 covariance 独立反例 | **预期失败**：大 z 方差 `1e12` 时，非法 x/y block 被错误接受；当前实现同例通过 |
| diff whitespace | `git diff --check` 通过 |

原候选的失败证据：launch 检查 4 failed / 3 passed；输入 owner 检查 4 failed / 1 passed，
其中取消拒绝/旧成功交接在原链已通过，未为它修改 runtime。初期环境尚未完成及测试清理错误的尝试
不计为产品证据；表中失败来自排除环境原因后的复核。

结果目录（未进 Git）：`build/software_debt_review_results/`，相对**主工作区**。
主要文件为 `pre_fix_launch.xml`、`pre_fix_owner.xml/log`、`covariance_399840c.log`、各包 test_results
和构建日志。关键反例及最终结果已记入本报告，不长期保存重复 build/log。

容器内复现主检查（`/workspace` 为本 review 源码，`/results` 为独立输出）：

```bash
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
colcon --log-base /results/log build --base-paths \
  src/rm_competition_interfaces src/rm_description src/rm_lio_bringup \
  src/rm_localization_adapters src/rm_pursuit src/rm_relocalization_bridge \
  src/rm_competition_mission src/rm_gicp_relocalization \
  src/rm_nav_config src/rm_navigation_bringup \
  --build-base /results/build --install-base /results/install \
  --executor sequential --cmake-args -DBUILD_TESTING=ON \
  -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O0 -g0'
source /results/install/setup.bash
export ROS_LOG_DIR=/results/ros_log ROS_DOMAIN_ID=54
unset RM_COMPETITION_MISSION_SOURCE_DIR RM_RELOCALIZATION_SOURCE_DIR
ctest --test-dir /results/build/rm_navigation_bringup --output-on-failure
ctest --test-dir /results/build/rm_localization_adapters --output-on-failure
ctest --test-dir /results/build/rm_relocalization_bridge --output-on-failure
ctest --test-dir /results/build/rm_competition_mission --output-on-failure
PYTHONPATH=src/rm_system_monitor:$PYTHONPATH python3 -m pytest -q \
  src/rm_system_monitor/test/test_readiness.py \
  src/rm_system_monitor/test/test_readiness_odom_runtime.py \
  src/rm_pursuit/test/test_pursuit_math.py -o cache_dir=/results/cache
```

## 提交集合、依赖与合入方式

推荐的最小完整交付：**原五提交按原序 + 本 review 的一个边界修正提交**。
`fed2cff` 是文档收口，无运行时依赖，但应保留其说明并以本独立复核修正原结论。
不需要合入其他 research、posture 或原 Work 的临时 evidence 分支。

| 提交/部分 | 必须同时采用的内容 |
| --- | --- |
| `399840c` 软件清债 | `ddf988b` 的 covariance 归一化；本 review 的 producer 门控、profile 隔离、wrapper 依赖修正 |
| `e761239` 仲裁 | PursuitCandidate interface、planner、Mission、launch policy 同步；本 review 的输入格式/age 修正 |
| `ddf988b` | 依赖 `399840c` 的 full matrix 校验；其 Spin 测试按 `e761239` 的 terminal barrier 写成 |
| `8e53cc6` minimal Mission | `e761239` 单 owner/取消/lease + SetTemporaryMission interface、service、两棵 BT、配置一起采用 |
| review 修正 | 以上完整候选；专属 YAML、共享 wrapper 的安装/依赖与消费者必须一起采用 |

**禁止把 `399840c` 脱离 `ddf988b` 单独采用**。独立编译该原始实现并运行关键反例已经证明错误接受。
只想采用软件清债而不要仲裁/Mission 时，应另作内容提取，不能直接声称原 commit 任意组合已经验收。

若 main 仍是 `d37f8d6`，建议审阅后 **ff-only 到 review 分支末端**，避免只 ff 到 `fed2cff`。
无需 rebase 原候选链。若 main 已前进，则在最新 main 的 integration 分支按序 cherry-pick
完整五提交与修正，冲突解决后重跑直接相关检查。只有确实要拆掉 Mission/仲裁时才重新整理提交；
不得遗漏关键修正，也不为整理历史改写既有公共链。

## 剩余限制与风险

- 雷达真实 ingress、可信 ROS publisher 管理、Mission 对 AUD-003 正向 owner/session 授权、
  实际下游取消 SLA/物理停车尚未验证。当前 V2 guard fixture 不是硬件准入证明。
- 不合作的 Nav2 server 拒绝取消/永不返回终态时，Mission 会持续等待；这是避免双 owner 的保守边界。
- 单 bundle source-time/cloud 合同有软件证据；本次没有构建或回放真实 FAST-LIO，也未验证真实标定、
  动态机械输入精度、真实 GICP 地图收敛或 covariance 物理可信度（AUD-012/013）。
- 未运行完整 Gazebo 世界、整车、所有自定义 profile 或硬件异常测试；额外 raw-world scan 的资源代价
  未测量。共享 wrapper 的实际 Humble 路由已运行，不等于仿真/实车完整导航验收。
- 旧车启用 Mission 后的 radar-only/home latch 变化应在下一次明确授权的旧车集成验证中确认；
  本轮不会追加实车阶段，也不将既有未覆盖 profile 提升为已验收。
