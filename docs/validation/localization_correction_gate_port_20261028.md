# AMCL correction gate / fault latch 选择性迁移

日期：2026-10-28。阶段：Integration。裁决：**限定合同内 Software / Simulation Go；
Hardware Blocked**。本轮为已有功能迁移，不验证新定位算法精度，不运行硬件。

分支 `codex/localization-correction-gate-port`，父提交
`85e4516850e1361647a874814a70c88310f60ccc`；此前依赖为 unified simulation `8375c8b`
→ event replan/arbitration `e901f1b` → LIO health `85e4516`。
main 保持 `d37f8d6e5ebe53533b7fc62aca8bb84392bee31b`，不 merge/push。
当前接口权威为 [bridge README](../../src/rm_relocalization_bridge/README.md#optional-correction-gate-and-explicit-relocalization)
和 [topic contract](../contracts/topic_contract_2027.md#correction-gate-候选2026-10-28)。

## 1. 历史源码事实

只读审计冷归档 `/home/qihei/.local/share/Trash/files/rm2027_navigation`，没有在那里开发。
`fix/old-car-amcl-correction-gate` 对应对象 `e15b6bdaf6f1b40c37cf1c725925a4d31d44eca6`；
当前归档没有同名本地 branch，使用固定对象读取。旧 `origin/main-old-car`
`820179457b70789b0f46697758dbe3d947ec1d4a` 的 bridge **没有创新门/锁存/reseed**，
不能把独立修复分支能力归到旧 main。

关键演进：`3614be38` 隔离错误修正，`62f6e0dc` 加自主恢复，`dbb3dc74` 处理静止
差分 twist 尖峰；后续 `51e3b233` 包含 odom rebase，`8fdecf24` 接 LIO validity，
`a037fac1` 增加连续异常确认，`41e6d7b3` 增加高速角速度修正 hold，
`46c521b2d2f8691f3cfd159b99fad8e7dc764ce7` 汇总 09-03 结果。没有整分支 cherry-pick。

| 项目 | 实际旧实现 |
| --- | --- |
| source / frame | 已由 AMCL quality gate 提供的 `/localization/global_pose`，map→base；bridge 本身可消费 GICP 同类输出，不依赖雷达 |
| candidate | 最近 canonical odom 源时刻匹配（默认 ±0.05 s）；`C=T_map_base*inverse(T_odom_base)` |
| residual | 比较 candidate 与最后 accepted 的 **map/odom 平移原点 XY 距离**、wrapped yaw；旧车门限 0.35 m / 0.35 rad |
| quality / time | covariance、finite、planar、freshness 在上游 gate；bridge 有匹配/年龄限制和 upstream Bool；没有当前原子 session/lease 合同 |
| baseline | 第一个候选无创新基准，直接建立；小 covariance 不能证明地图模式正确 |
| fault | e15 版一次超限即锁存；后续旧车配置连续 3 帧，通用默认 1 帧。丢弃异常、保留旧 C；锁存 false validity 并停止新 map→odom TF |
| clear | 正常下一帧不会清 latch；人工 initialpose 或 reset 清基准。自动 reseed 的同 stamp initialpose 被识别，避免清掉可信旧基准 |
| motion / recovery | 使用 LIO twist；≤0.08 m/s、≤0.15 rad/s，保持 0.6 s；连续超限 0.12 s 才确认运动；以旧 C×当前 odom 生成初值，XY 方差 0.09、yaw 方差 0.0685；settle 0.5 s、连续 5 帧；cooldown 2 s |
| 后续高速 hold | 源时刻匹配角速度 ≥3 rad/s 时冻结全部 AMCL 修正、保留可信 TF，停转后再更新；不是 LIO isolation |

旧 recovery 已和 reseed 混在 bridge 节点里。旧车 gate 数字、驻车判断、自动 initialpose、
高速 hold、rebase 均未原样搬入新车。本轮仅提取创新拒绝、连续异常锁存和明确 reset 边界。

## 2. 历史证据分级

读取旧树中的 `docs/validation/old_car_map_odom_correction_gate_20260825.md`、
`old_car_high_spin_localization_engineering_report_20260825.md` 和
`old_car_fast_lio_high_spin_validation_20260903.md`；以下为**历史记录可核查的结果**，
不是本轮重新实测。原始大 bag 未在记录路径恢复，本轮未运行历史 bag replay。

| 等级 | 可恢复事实 | 不能据此推出 |
| --- | --- | --- |
| A 实车在线 correction | 08-25 恢复后持续接受 global pose，操作员确认位置正确；09-03 最终高速段结束后修正恢复，停车附近单步 0.159 m / 0.205 rad | 无外部真值，不是新车精度或长期可靠性验收 |
| B 实车 fault/recovery | 08-25 日志 4 次锁存并 recovery completed；最大拒绝 1.928 m / 2.873 rad；3 次首次 seed 后 1.052–1.095 s，困难次 11 次 seed / 21.803 s | 从首次故障计时分别有更长等待，不能把 1.1 s 当全部故障恢复时间；不能单独归功于 gate |
| C 实车自然故障 | 自动输出的 AMCL 错峰曾在低 covariance 下进入 canonical correction；高速试验中 LIO 正常而 AMCL 单帧/连续错峰，gate 真正拒绝过错误修正 | 受控自转诱发的自然算法失败，不等于任意工况故障注入；不能断言 LIO 总正确 |
| D bag / 离线分析 | 09-03 按源时刻对齐的角速度离线反事实分析支持 ≥3 rad/s hold；此前记录包含 replay/诊断 | 本轮无原 bag 重放，不把离线反事实当在线恢复成功 |
| E software | 旧纯数学、10 检查点注入→15 自主恢复检查→静止尖峰回归；源代码和报告可读 | 不能代替实车 |
| F 仅文档 / 未关闭 | 至少 10 次独立触发、精确恢复误差、真实制动/整机 repeatability 等验收项 | 计划不等于完成 |

关键反例：单帧立即 latch 在 09-03 的 5.5 rad/s、9.05 s 试验造成 5.45 s TF 缺口，
下一正常帧仅晚约 0.154 s。改 3 帧后，5.5 rad/s、34.10 s 中 1–2 帧异常
（最大 0.625 m / 1.415 rad）被隔离且 TF 连续；约 7 rad/s 仍出现连续 3 帧，
造成 12.15 s 中断。最终 angular-hold 单个包 130.29 s、高速 52.751 s、峰值
7.273 rad/s、map→odom 最大间隔 61.8 ms，属于该组合的一次现场 smoke。
这些证据支持保护机制有价值，也说明立即锁存可能误拒、恢复存在长尾；不授予任何新车 Hardware Go。

## 3. 当前架构已有与缺失

| 边界 | 基线 85e4516 事实 / 本轮处理 |
| --- | --- |
| 可信 LIO | adapter 原子健康包、隔离、连续恢复和 generation 已存在；不修改其实现 |
| global source | 选择 AMCL 或 GICP 的后端 status（内嵌原始 pose），无多个同时 correction 的受支持组合；配置错误仍可能造成重复源，不能共启 |
| 已有 gate | 通用 gate 已有 frame、时间、finite、平面与 covariance 门；GICP 自身有 convergence、fitness、相对 guess 的 2 m/1 rad jump 门。**缺通用可信 canonical 创新门、稳定 latch 和显式事务** |
| GICP 质量 | fitness 是真实匹配指标；现有 covariance 是 clamp(score,0.01,0.50) 派生代理，不是标定测量协方差。本轮不制造或提升其证据等级 |
| authority | bridge 独占 map→odom / unified status；adapter 独占 odom→base；候选源无 TF 权。保持不变 |
| 旧 reset | 只撤销有效性和设置 cutoff，没有完整 prepare/新后端准入/commit 事务；正常 correction 不换 session |
| late data | 已有源时间、水位、epoch、pending 队列，但新 gate 若复用“每个 upstream valid 都续租”的路径会让拒绝候选续租。本轮把 lease 绑定到真正 accepted pose，pending 自带原 deadline，并在提交时再次检查时间 |
| Mission / event replan | 已有旧会话撤销、terminal-before-replacement、旧结果拒绝和事件 BT；没有改动它们的源码 |
| Z research | `codex/z-localization-integration-20261023` 有独立候选合同：私有 TF quarantine、map/base 转换、status；invalidate 只暂停，legacy reset 不可冒充 ResetFrontend。当前依赖链没有该 package；未迁入、未改其合同 |

## 4. 本轮合同与 G1–G8

`OBSERVED → ELIGIBLE → GATED_ACCEPT / REJECT / FAULT_LATCHED` 由 bridge 内部与诊断
String 表达，String 不授权。另有 VERIFYING、PREPARED、ELIGIBLE_WAIT_LIO。
只有 GATED_ACCEPT 可以更新 canonical correction。

| 判据 | 实现 / 边界 |
| --- | --- |
| G1 freshness | 上游原始 stamp/deadline、ROS+steady lease、0.5 s pose age、0.2 s pending wait；拒绝不续租 |
| G2 session | pin 一个 backend producer+epoch，严格序列和测量水位；显式 prepare 后只收更晚 admission epoch；测量晚于该 epoch 与 bridge cutoff |
| G3 finite | pose/quaternion/transform 有限合法检查；检查结果后才赋值 trusted/canonical |
| G4 translation | 当前候选 map/base 与 `C_trusted*T_odom_base(t)` 的 **3D位置距离**；AMCL 平面时与 XY 一致，不忽略 GICP 高度 |
| G5 yaw | atan2(sin(delta),cos(delta)) 最短角 |
| G6 motion | 匹配同源时刻 odom，正常平移/旋转进入预测而非误差；不添加静止假设、速度 hold 或停转算法。±0.05 s 最近邻残余误差需硬件评估 |
| G7 quality | 保留选定后端的真实质量检查；创新门没有新增假 covariance/score，也不能用 covariance 小代替一致性 |
| G8 temporal | 普通模式连续 3 个 eligible residual 矛盾才锁存；startup/显式事务连续 3 个 mutually consistent 候选才建立 baseline，间隔超过 status_timeout 重启确认 |

Latch 禁止后续候选改 C；正常帧、后端重连、LIO 恢复、单独 initialpose 都不能解除。
只有显式 prepare 重开验证。Latch 撤销统一 validity 和新 map→odom，Mission 经既有
接口取消动作；gate 不发布停车命令、不修改 LIO、不自动 reseed。quality/source
失效继续走其原有失效合同，并不伪造 innovation 或强制给每类故障设置 latch。

A：LIO healthy+候选错，拒绝候选；连续错才使 unified invalid，LIO 继续可信输出。
B：LIO invalid+候选好，只记录可用于未来恢复的 eligibility，不排队提交。
C：LIO recovering 时同样不提交；既有 health 连续恢复完成后还需新的 global evidence。
D：双方不可信时保持 invalid。已经设置的 correction latch 跨 LIO 恢复保留。

## 5. 普通 correction 与显式 relocalization

普通 correction 不改 session，只接受与当前预测一致的小更新；可以累积超过单步阈值。
显式事务沿用 `/localization/reset_map_to_odom` 为 prepare，立即预留更晚 bridge session、
撤销旧 validity、清匹配/待提交数据；再显式调用选定 quality gate 新增的
`<valid_topic>/reset_session` 创建更晚 admission epoch，连续新可信候选才 commit。
三步期间统一 validity 为 false，旧动作必须 terminal 后才能发新动作。

这两个 Trigger **均不重置匹配算法、不发 initialpose**。必要的 AMCL/GICP 手工 seed
继续用原算法接口；操作方需先停/排空旧计算，保留真实 acquisition stamp，再打开
新准入并送入明确的新候选。直接给旧源调大门限不构成事务，单独 seed 不清 latch。

重要限制：raw PoseWithCovarianceStamped 没有算法 transaction ID；无法识别旧算法
结果被错误改写成新 acquisition stamp。准入 epoch 不等于算法已完成 reset 的证明。
限定 Go 要求单受控 producer、真实源时间、排空旧工作。未来有原生 reset metadata
才能扩大到任意异步后端事务。GICP 真实匹配与其大初值能力本轮只有源码审计，未执行
真实地图注册；运行验证使用真实 quality gate 和合成候选。

## 6. 验证结果

| 场景 | 结果及证据 |
| --- | --- |
| C1 正常小修正 | PASS：真实节点接受 +0.1 m，map/odom 更新正确 |
| C2 stale | PASS：旧 2 s 原始候选拒绝，C 不变，LIO healthy |
| C3 old session | PASS：旧 backend epoch 新 envelope 仍不被接受 |
| C4 NaN/Inf | PASS：两种位置异常及零 quaternion 拒绝，不污染 C |
| C5 单大异常 | PASS：+5 m 被丢弃，原 validity/修正保持于原 lease 内 |
| C6 连续大异常 | PASS：3 个矛盾 latch；下一批正常帧仍 invalid |
| C7 好坏交替 | PASS：节点级 6 轮/纯函数 20 轮，不闪烁或改入坏 pose |
| C8 显式合法重定位 | PASS：+20.1 m 基准经 prepare/new admission 转为 −15 m；旧 backend 在等待期间不能 commit |
| C9 重定位后迟到旧数据 | PASS：旧 epoch 的 valid/invalid 回调都 inert；同 epoch 旧 sequence 也拒绝 |
| C10 healthy LIO+坏修正 | PASS：锁存期间 health 仍 true，物理仿真 canonical odom 继续增长 |
| C11 invalid LIO+好修正 | PASS：失去 LIO 时好修正不恢复；前 0.11 s 恢复输入仍 invalid；双方不可信也 invalid |
| C12 source 消失 | PASS：停止 global 输入 1.4 s 后 unified invalid，LIO healthy |
| C13 导航进行时拒绝 | PASS：3 次单帧异常，额外 Navigate/ComputePath 均 0 |
| C14 导航进行时显式提交 | PASS：prepare 撤销旧 action；旧 CANCELED terminal 在 replacement 前；新 session 后完成目标 |
| C15 event replan | PASS：实际 FollowPath failure→Wait 期间连续异常 latch；invalid 段无新增 ComputePath，健康 LIO 持续；好帧不解锁，显式事务恢复 |

False-reject：startup +20 m（纯函数还覆盖 30 m/2 rad）；explicit +20.1→−15 m；
逐步累计 +2 m（纯函数 +3 m）；yaw +3.13→−3.13；移动+7 rad/s；远 odom 原点旋转
均 PASS。一次性巨大 drift 不自动接受，需要显式事务，这是有意的 authority 边界。
不保证所有合法大跳都能通过普通 gate，也不证明一致的错误初值能被识别。

实际运行结果：

- bridge 构建成功；10 个 correction、5 个 transform/cache、7 个 quality 单测和
  4 个配置检查通过（26 个测试用例；colcon 汇总含 4 个 suite wrapper 显示 30）。
- 统一构建 10 packages 成功；bringup 检查 22 source packages、10 install prefixes，
  无重复包、资源和运行依赖齐备。
- `test_correction_runtime.py`：真实 adapter+quality gate+bridge，C1–C12 及附加场景
  通过；三个进程退出码均 0；唯一 validity owner 为 bridge。中断验证 1.2 s 后，必须重新积累 3 帧。
- `test_input_health_runtime.py --upstream` 回归 PASS：silence、stale、rollback、future、
  NaN、Inf、jump、frame、freeze 隔离恢复，旧 generation 拒绝，7 rad/s 和新 backend epoch；
  两进程退出码 0。无 gate 的原接口仍可用。
- Mission 仲裁回归 1 passed / 29.66 s；32 goals、29 cancels，最大 active goal=1；
  包含故意杀 planner 的故障场景，该 -9 是注入预期结果。
- 统一 Gazebo/真实 adapter/quality gate/bridge/Mission/Nav2 smoke PASS：到点误差
  0.126465 m，最终 epoch 检查版本复验为 0.121214 m；C13–C15 均通过。invalid 段指令末 10 帧各分量 <0.03，连续 2 s 物理位置
  漂移 <0.1 m，无新增规划；clean shutdown，无非预期进程死亡。

复现（仓库根目录，先按 simulation operations 完成 Jazzy source）：

```bash
tools/build_simulation.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --base-paths src/rm_relocalization_bridge --packages-select rm_relocalization_bridge
colcon test-result --test-result-base build/rm_relocalization_bridge --verbose
python3 src/rm_relocalization_bridge/test/test_correction_runtime.py --output /tmp/correction_runtime_final
python3 src/rm_localization_adapters/test/test_input_health_runtime.py --upstream --output /tmp/correction_lio_regression
RM_COMPETITION_MISSION_SOURCE_DIR="$PWD/src/rm_competition_mission" python3 -m pytest -q -s src/rm_competition_mission/test/test_navigation_arbitration_runtime.py
python3 src/rm_new_car_simulation/scripts/validate_correction_navigation.py --output /tmp/correction_navigation
```

本轮临时 JSON/log 在上述 `/tmp` 输出目录及 `/tmp/correction_navigation_final`；它们可清理，不作为永久依赖。本报告保留
裁决所需关键数值，代码保留可重跑的输入与断言。不提交 bag/build/install/log/cache。
初次普通 `--packages-select` 构建受无关 Humble driver 未构建影响；改用项目规定的
有限 `--base-paths` 后成功，未安装额外依赖或修改系统环境。

## 7. 参数分类与裁决边界

| 类别 | 参数 / 规则 | 状态 |
| --- | --- | --- |
| A 软件协议 | 原始 stamp 不改写、严格 source 水位、epoch/producer/sequence、finite、单 authority、拒绝不得续租 | 本轮已验证；不是可调硬件阈值 |
| B 模型推导 | SE(3) correction 合成、同源时刻预测、yaw wrap、commit 前时间复核 | 纯数学/软件验证；不说明传感器误差为零 |
| C Hardware Blocked | fixture 0.4 m / 0.3 rad、3 异常/3 验证、匹配 ±0.05 s、年龄/lease/等待窗口、motion 不确定度、AMCL covariance / GICP fitness阈值、LIO health 数值 | 新车实测分布/标定/误拒与漏检率缺失，不能沿用旧车数字作为最终值 |

限定 Go 的理由：新增保护有真实旧故障依据；没有第二 authority；坏候选不写 C，
旧会话不复活；健康 LIO 不连带隔离；普通和显式入口均可用；Mission/event-replan
交互通过。默认仍关闭，只有独立 correction fixture 开启。**不是 Hardware Go**。

未证明：真实 AMCL/GICP 配准成功率、真实时间同步/动态外参/uncertainty、缓慢错误漂移、
一致错误 startup、任意重新打时间戳的旧算法结果、多源错误共启、真实底盘制动、长期
误拒率。实际地图错误模式与正确模式必须靠后续测量和受控初值验证，不能只用门限区分。

## 8. 下一阶段建议（仅审计）

- Autonomous reseed：先定义谁持有 seed 权、允许源和 LIO health 前提，以及如何证明
  algorithm reset/新测量；不得从 admission reset 自动推出允许 reseed。
- Jump/rebase：审计 raw/canonical/全局三种基准与动作撤销次序；明确车辆持续运动时的
  TF 连续性和地图目标语义，不能只改 frame 或扩大门限。
- Automatic recovery orchestration：先审计 recovery source 选择、取消/prepare/verify/
  commit/resume owner 及超时；复用统一 validity，不新建并行授权链。

本轮未实现以上三项，也未引入 Z recovery、radar fallback 或新定位算法。
