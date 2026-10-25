# RM2027 Software Integration Staging（2026-10-24）

## 当前结果：两项 staging 缺陷已闭合（2026-10-25）

从 `codex/rm2027-software-integration-staging-20261024@965b97e` 继续，只处理本 staging 新发现的 P1/P2。**P1、P2 均 Software Closed；C 总体仍为 Software Modify**。没有重新审计全仓、开发新功能或处理硬件；main 保持 `d37f8d6`，未 merge/push。

- P1：`e89c596`（修复：绑定任务动作与定位会话屏障）。先补 arbitration 合同，再把 NavigateToPose/Spin 绑定发送时的定位 generation。新 producer/session/epoch 或失效在 callback 内立即请求 cancel，保留原 pending/active handle 到 rejection/terminal；旧 success 只释放旧屏障，不推进巡逻、触发 Spin 或完成临时任务。清跨会话 completed/failed/pursuit 缓存；LOST 恢复还要求新于 COMMITTED 的 pursuit，防止 invalid 间隙沿用 cached TF 的候选。比赛优先级、持久目标、取消 ACK/终态语义不变。
- P2：`a252578`（修复：统一定位协方差准入与撤销）。把 AUD-015 原 finite/symmetry/PSD 代码提为共用 `pose_covariance.hpp`，用于 A gate、Z canonical 输入及杆臂传播输出、bridge atomic 与 legacy/direct 入口。保留 per-dimension normalization、`1e-10` tolerance、zero/missing、rank-deficient 与原 variance/profile 限值。新增的是现有 Eigen 的声明/导出，不安装新工具链。坏矩阵不 clamp/补值；立即 invalid、清 held pose/pending work、撤销 source，严格更新的合法观测才恢复。

| 本轮复验 | 最终结果 |
| --- | --- |
| 受影响 interfaces/Mission/canonical/bridge | 4 包构建、安装/头文件导出通过；使用现有固定 Humble/Nav2/Livox/GTSAM 依赖。没有重建无关模块 |
| 原 `3 passed / 3 failed` 组合检查 | **6/6 passed**；原两个 P1 边界和 covariance failure 全转为通过，原断言未删除/改为 xfail/skip |
| P1 事件控制回归 | **6/6 passed**：active epoch/source、pending acceptance、cancel 与迟到 SUCCEEDED、LOST/recovery/间隙缓存、100 次追加快速 epoch、退役 source/epoch 不回滚、Spin 旧成功不推进；终态前无替代请求，max active navigation=1 |
| P2 四条实际 ROS 入口 | **4/4 passed，56 scenarios**：atomic bridge、canonical→bridge、direct legacy pose、Z legacy embedded status；每条 7 类合法兼容 + 7 类非法矩阵。负特征值/非对称/NaN/Inf/负方差/tolerance 外均立即 invalid 且清 pose；同 source 修订/新 emission heartbeat 不恢复，严格新 PSD source 可恢复 |
| A/staging 原核心 | 原 **75 个内部用例全部通过**，另新增 2 个 covariance 边界 GTest；当前四包合计 **77 passed**。Mission/arbitration 15 项在最后缓存修正后完整复跑通过 |
| B 关键定向 | **14/14 CTest passed**，包含 AUD-016 实际 server cancel、AUD-052 51 个数值场景、AUD-056 9 类调度及重建 Spin plugin；固定 overlay/依赖没有改变 |
| Z/readiness/pursuit | **45 passed**（Z 38 + readiness/pursuit 7），没有重跑未修改的地图研究/硬件工作 |
| Z canonical/reset/session | wire 100 快速 barrier、pending TF/reset/旧 epoch/旧 pose/重排/clock high-water 通过；真实 backend 10 reset + frontend replacement/22 RPC 通过；35 个直接相关 upstream session/reset/warm/cold 用例通过 |
| gate off / shadow / clock | native epoch latch、legacy publisher restart、两个默认关闭入口、actual ZLIO shadow 和 paused/backwards clock smoke 均通过；没有新公共 TF/cmd_vel owner，默认 gates 仍 false |
| 定向 sanitizer | 共享 covariance/A gate **14 GTest ASan+UBSan passed**，无 findings/suppression；不据此声称第三方库、S03 或 race-free 已验收 |

测试夹具首轮有一次 Spin 未启动：top-level composition 按既有策略覆盖了 YAML 的 tree 选择。改为实际 Mission executable 显式选择仓库原 Spin 树后，6 项事件检查通过；未修改 production launch/BT/比赛策略。所有事件 deadline 仅用于测试失败与回收，生产实现无新增 sleep、取消超时解锁或以 ACK 代替终态。

最新运行代码 checkpoint 为 `a2525784e40ff0dd071f3bc2dd35c22016f09def`；后续报告/evidence/测试说明提交不改变运行逻辑。原始失败证据保留在 [聚合 JSON](software_integration_staging_evidence_20261025.json) 的原字段，最新有效结果位于 `repair_20261025`。完整本地日志集中在主工作区 `build/software_integration_staging_results_20261024/repair_20261025/`，不进入 Git。

**C 没有升级为 Software Go，也没有新增 main 合入建议。** 本轮只关闭 staging 消费端/输入合法性两个缺口；S03 历史 shutdown -11 的软件 blocker 仍 unresolved，M11 真实 prior/map/landmarks 接受仍未完成。feature gate、F14 native opt-in 支持边界和全部原硬件门继续保留。A/B 原软件结论不变，B 的依赖发布/加载条件也未在本轮处理；不合 main，不声称 Hardware Go。

## 965b97e 首轮组合记录（历史）

以下保留初轮实际提交、依赖、冲突和失败证据；其中 P1/P2 的 open 状态、`3 failed` 与合入决定中相应缺口已由上节覆盖。S03/M11、B 依赖发布及硬件门没有被覆盖。

开始于 2026-10-24，完成于 2026-10-25（Asia/Shanghai）。阶段：Integration；只作组合软件结论。main 未合并、未推送；不接设备，不作 Deployment Go / Hardware Go。

## 基线与分支

- 实际 main：`d37f8d6e5ebe53533b7fc62aca8bb84392bee31b`。
- 开始时主工作区：`experiment/dynamic-surface-members@b440138f0f6074669714abee7d6602ed0fc93c23`，porcelain 为空；保留原分支、HEAD 和工作区。
- staging：`codex/rm2027-software-integration-staging-20261024`，独立 worktree `build/software_integration_staging_20261024`。
- 组合运行代码 checkpoint：`d74948c06c8ef633c084567e079443da54488d42`。后续提交只保存本报告、最小聚合证据、组合回归和 launch 测试夹具兼容，不修改候选生产逻辑。
- A+B、无 C checkpoint：`a5354ca`；只提取临时源码快照重新构建，不另建永久分支。
- 构建/日志：主工作区 `build/software_integration_staging_results_20261024/`，不进入 Git。开始状态保存在该目录 `baseline.json`。

## 完整组合集合

按下表顺序整合；A 直接 fast-forward staging，B/C 逐个 cherry-pick 并保留原 SHA 来源。

| 候选 | 原提交（完整 SHA） | staging 提交 |
| --- | --- | --- |
| A | `399840c897ba4028e707e481ed5758a33ea1ca99` | `399840c` |
| A | `e76123935fcdbbd796d37c6ccca67bf74d2918bc` | `e761239` |
| A | `ddf988b120f1535f9983a73b59d85963ead462d7` | `ddf988b` |
| A | `8e53cc6dedc16ef09ffb5b1a82f802413bdf82da` | `8e53cc6` |
| A | `fed2cff2604961aee6c3e7bc467f6c7eac9b33a6` | `fed2cff` |
| A | `ea514742cc1cf43312356ffbc3cef1c2937d33d3` | `ea51474` |
| B | `1247e2e50d47550d46ca9f4b15b586e58f277a7e` | `6cddf78` |
| B | `cbe2a1f06444be6d07db5f9ba9802ac5915fb1aa` | `e15d08e` |
| B | `8124b4619c2cce05d110e89fd30cd4ccf4ae0f9e` | `a5354ca` |
| C | `ddabf8b474adc33d8937c127adf7812ef1360232` | `0a31cb0` |
| C | `474572e30c421d217fd6a921ec27792c23c39ade` | `d056913` |
| C | `52e4e5acb22d586553651af3efdfad2aab25f368` | `b447e28` |
| C | `7deded059bb2bb3408b9c922c9a1b3388d91e492` | `bd7563f` |
| C | `fd500983f73fff7f3c4fc786a7068010f090358d` | `d7a6a0e` |
| C | `cebb150eb6f9c1b340daa82bcef5271d3ab82615` | `965c0e9` |
| C | `c13463c52963315368294f524496652ca0dab0fe` | `d74948c` |

B 的 `cbe2a1f` 同时固定 FAST-LIO 子模块 `8fb3296dba33b8924bd8e92c15df88e0a3e246dc`；它不在父仓提交图内。嵌套 ikd-Tree 保持既有 `0438b0d`，Livox 保持 `ffb5c72`。
C 保持 upstream `76e126328b670adc92981d9b4900cc059a6f2343`，按候选既有顺序应用 scan-end pose 和 session frame 两个父仓补丁；构建时临时物化，不另改估计器。

已读取 `dd8cbd2..c13463c` 实际历史。`7999717` 是平行治理/文档重构 checkpoint，main 已有对应 `d37f8d6`；不重复迁入治理变更，也不把 main 的 Current 标签换成研究分支标签。C 必要最小集合是表中七提交，包含迁移入口、parity、shutdown 证据、旧证据隔离、坐标 checker、结论同步、F14。不是单取 F14。

没有纳入 omni、posture、dynamic-surface、其他历史实验或 hardware-only 修改。Omni 保持 `FROZEN_WAIT_CHASSIS_PARAMETERS`。

## 整合前的依赖图

以下依赖在 cherry-pick 前记录于结果目录 `dependency_plan.md`。

```text
main -> A:399840c -> e761239 -> ddf988b -> 8e53cc6 -> fed2cff -> ea51474
     -> B:1247e2e -> cbe2a1f[FAST-LIO 8fb3296] -> 8124b46
     -> C:ddabf8b -> 474572e -> 52e4e5a -> 7deded0 -> fd50098 -> cebb150 -> c13463c
```

- **Commit**：399840c 必须带 ddf988b 的 covariance 归一化；arbitration/minimal Mission 不拆；ea51474 必需。B 的两个 Nav2 patch 与所有重建 BT action plugins 成套。C 两个父仓 patch 按原顺序，生成消息和全部 consumer 同次构建。
- **Package**：rm_competition_interfaces → canonical adapter / bridge / Mission；pursuit → Mission。共享速度 launch 安装在 rm_nav_config，bringup / simulation 均依赖它，避免反向依赖环。upstream localization_interfaces → backend / frontend → candidate schema translation；RM canonical owner 不新增 upstream schema 的构建依赖。
- **Launch/config**：single+bundle 才选 first-source 专属 YAML；其他既有 profile 不提升。GICP body 请求同时打开 FAST-LIO master/body flags。Z legacy 可请求 registered cloud，ZLIO shadow 使用原生私有输入；同域不同时启用两套 canonical 定位组合。
- **Runtime ownership**：lio_adapter 唯一 odom→base；bridge 唯一 map→odom；RSP 管 mechanical tree；backend TF quarantine。Mission 唯一任务级 NavigateToPose/Spin client，pursuit 只发候选。controller/behavior → cmd_vel_nav → smoother → 唯一最终 cmd_vel。AUD-056 管 Nav2 内部 BT 请求，arbitration 管外部 Mission action，层次不同；两者都不能把 cancel ACK 当终态。
- **Feature gates**：Z 的两个 enabled 默认 false；canonical_session_transactions 默认 false，true 需 native ZLIO + managed 既有 owner。Mission/雷达组合默认关闭、启动 disabled；supply/recovery 坐标留空。无开关授予物理运动权限。

## 冲突与处理

共五个冲突位置，均为机械冲突；未删除测试、放宽条件或替换整份文件。

| 来源 | 文件 | 处理 |
| --- | --- | --- |
| ddabf8b | docs/operations/runtime_profiles.md | 识别旧 docs/runtime_profiles 路径迁移；保留 main Current 说明和 C 区段，修复相对链接 |
| 474572e | docs/README.md | 保留 A debt 与 C parity 两个索引 |
| 474572e | docs/contracts/tf_contract_2027.md | 保留 A single-bundle/GICP 与 C quarantine/session 区段 |
| 474572e | docs/operations/runtime_profiles.md | 保留 A/B 与 C 的独立 profile/限制说明 |
| c13463c | src/rm_competition_interfaces/CMakeLists.txt | 同时注册 SetTemporaryMission 与 SelectLocalizationSource；保留 PursuitCandidate/LocalizationFrame，并取 nav_msgs 依赖并集 |

自动合并的 FAST-LIO launch 保留 A 的 master/body gate 和 C 的 scan/zero-start override。文本无冲突不代表语义兼容；session 与 covariance 问题由下面的实际组合回归判定。

## 环境与复现入口

复用项目现有 Humble 镜像 `81b325bebf2f`（Nav2 1.1.20），非 root、network none、private IPC、cap-drop ALL、no-new-privileges，无设备映射。源码只读、结果目录可写。ROS 回归使用独立 localhost domain；无真实 driver/UART/MCU。
从本次空输出目录构建，不复用候选包对象或旧生成消息；只复用镜像系统依赖、固定 Livox SDK 与 GTSAM 4.2 私有依赖库。性能无结论。回归脚本、命令和完整日志集中在结果目录：`build_stack.sh`、`build_nav2.sh`、`build_z.sh`、`test_stack.sh`、`test_confirmed.sh`、`test_epoch.sh`、`test_combination.sh`、`test_z.sh`、`test_sanitizer.sh`、`test_z_asan.sh`、`build_test_ab.sh`。

初期构建编排错误单独记录，不计作产品失败：Z 先加载未生成的顶层 setup；sanitizer 只加载单包 setup 未得到完整 CMake prefix；FAST-LIO 嵌套 ikd-Tree 尚未初始化；rm_lio_bringup 构建前 FAST-LIO 未装入对应 overlay。均按固定依赖顺序修正，未改生产实现或门槛。另外测试调用器曾重复设置由 CTest APPEND_ENV 提供的 source path，导致配置文件路径拼接错误；移除重复设置后只重跑失败/后续检查。rm_simulation 的已声明直接依赖 rm_dog_hole 也纳入本轮构建。用户中断前的一次 Z 容器 exit 137（OOMKilled=false）没有源级错误，不计通过，恢复后继续验证。

## 组合结果与决策

**组合结论：Software Modify。建议 selective merge；不能 ff-only 整个 staging 到 main。**

[最小聚合证据](software_integration_staging_evidence_20261025.json)保存计数、关键观察和反例；完整日志/build 不进入 Git。不同层次计数不能简单相加成为“整车通过数”。

| 范围 | 实际结果 | 证明边界 |
| --- | --- | --- |
| clean build / install/export | Livox 1、FAST-LIO 1（bundle/async/adaptive 三目标）、Nav2 修复 overlay 3、RM/直接依赖 20、upstream backend/frontend/bringup 3 均成功 | 本次空输出目录构建；依赖编排错误修正后完成，不用历史候选对象充当 clean build。Jazzy/Gazebo8 独立载体不在受影响 Humble 范围 |
| canonical / session | 15 原 canonical + 7 F14 GTest 通过 | 坐标转换、twist、covariance 传播、10,000 epoch stress / 1,000 prepare、旧 ticket / source / producer 隔离 |
| global pose / profile | 5 math + 12 full covariance + 4旧车 profile 用例通过 | A 的 finite/symmetry/PSD、尺度归一化有效；不表示 C atomic 路径满足同一 gate，见 P2 |
| launch / FAST-LIO / GICP / route | 17 pytest 通过 | YAML/CLI、single-bundle 隔离、45 样本跨 40-scan 边界、实际 GICP body→canonical 转换、master/body flags、controller/behavior 共用 smoother、关闭态 profile |
| arbitration / minimal Mission | 15 pytest 通过；上述四包合计 75 个内部用例通过 | 单 task owner、pending/active/late handle、取消拒绝/ACK/终态屏障、旧 success、临时任务/返航 latch、radar-disabled/default-off；session 新增反例另列，不被这些通过覆盖 |
| Z / map checker / readiness / pursuit | 108 passed / 1 skipped | 包含 Z 38、地图相关 63、readiness/pursuit 7；skip 为仓库未打包的真实 field baseline，不计真实 M11 |
| B confirmed correctness | 14/14 CTest 通过 | AUD-016 三类实际 server；AUD-052 51 个真实 ESKF 数值场景；AUD-056 九类调度 + 实际重编 Spin plugin。未混用旧 apt 模板派生插件 |
| F14 upstream focused | 17 session/reset unit + 18 reset/warm/cold flow 通过 | 仅直接相关 35 个用例；不宣称上游全部算法/ROS/driver composition 都验收 |
| native gate on | wire smoke + actual backend RPC smoke 通过 | 100 快速 epoch barrier、pending TF reset、旧 epoch/UUID/失序 callback、LOST 旧 pose、revision notice、pause/resume、clock high-water；实际后端 10 resets + 1 frontend replacement，22 RPC calls，新旧 owner 不重启 |
| gate off / legacy | native epoch latch、legacy publisher restart 两条真实链通过 | 输入失联撤销 status 并停止刷新 correction TF，保留 gate=false 锁定合同；correction 矩阵最大误差 1.11e-16 |
| Z shadow | 20 native frames、181 map sensor odometry，reset 到 epoch 1；公共 TF/canonical/command publishers=0 | 实际 ZLIO + synthetic IMU/PointCloud2；没有驱动、Nav2、serial 或额外 canonical owner |
| clock / diagnostics | paused expiry、invalid 心跳、fresh resume、backwards latch、畸形 state 检查通过 | stale 心跳不续 source lease，diagnostics 不授予运动权限 |
| cancel → smoother | 1 pass | 实际 controller + currentness fixture → 单一最终 smoother；终态 CANCELED 后约 0.0936s 观测归零，地图恢复后旧 compute=0、归零后非零=0；减速沿用原参数，非立即物理停车 SLA |
| repeated default off | 两个 Z 入口各 3 次，6 次均 exit 0 / 零子进程 | A 的三个关闭态 competition profile 也在 launch 回归通过；无默认启用 Z/Mission/Nav2/硬件 |
| lifecycle / sanitizer | backend 12 normal mixed + 3 peer-exit + 8 ASan/UBSan mixed，23/23 exit 0，无 timeout/findings；F14 7 + AUD-056 10 定向 sanitizer cases 通过 | 当前工作量未复现 S03，不关闭历史故障；第三方共享库未插桩，未得到 TSan/race-free 或目标机可靠性结论 |
| 新组合负例 | **3 failed / 3 passed** | 两个 session 边界失败 + covariance 失败；controller/smoother 与两个关闭态用例通过；失败均保留正常 assertion，没有 xfail/skip |
| C-free A+B checkpoint | 10 包独立重建，68 个四包内部用例通过；同一 Nav2 修复 overlay / B correctness 结果适用 | 用实际 a5354ca 源码快照重新生成旧接口并重建 consumer，避免以带 C 的二进制代表无 C 子集 |

最后检查：main 仍为开始 SHA，原主工作区干净；不合 main、不 push。所有任务 ROS 容器均已退出，没有活动命令/设备会话。

### 跨候选交互判定

| 交互 | 判定 |
| --- | --- |
| Z validity 与 arbitration | invalid/lease expiry 会停止 Mission，正对照通过；**session_id/epoch 换代语义未接入 Mission，P1**；C covariance gate 也不等价于 A，P2 |
| reset/relocalization 与旧 pose/action | F14 canonical/bridge 已拒旧证据、重排和旧 ticket；Mission 不应继续旧会话计算的 action，但当前会继续。不能因下层 F14 Resolved 宣称消费端闭合 |
| future/stale timestamp 与 Mission | A 输入格式/age/watermark 用例和原仲裁用例通过；C 时间回退/暂停/新鲜恢复检查通过。该通过不消除 session 缺口 |
| AUD-056 与 arbitration cancellation | 分属 Nav2 内 BT 请求与外层任务 owner；两个层次的拒绝/迟到/终态屏障分别通过；未发现重复授权或绕过 terminal 的证据。未把假 Nav2 的 Mission 测试说成完整真实 Nav2 世界端到端导航 |
| smoother 与 Mission/behavior 切换 | 实际 controller/behavior 统一入口，最终 cmd_vel owner 唯一；实际 controller cancel 后旧请求不恢复。Mission/Spin 的 owner 交接有 runtime 证据；硬件末端和完整比赛世界仍未验证 |
| FAST-LIO publication 与 Z shadow | A master/body flags 保留；C legacy scan/zero-start override 保留。原生 shadow 无 FAST-LIO/canonical owner；A/B 只在各自隔离域运行，不引入第二套驱动 |
| feature gates 全关 | 关闭态默认零新增 Z/mission 进程；gate=false 路径、旧车 profile 和 A+B 独立快照通过。A 已接受的 single-bundle/radar-only/home-latch 行为仍按原候选合同，不把已启用 Mission 宣称为旧车行为完全等价 |

## 新发现的问题与保留门槛

| 等级 / 状态 | 问题与决定性证据 | 影响 / 最小后续条件 |
| --- | --- | --- |
| P0 | 本次未发现 | 不等于全仓或硬件无 P0 |
| **P1 / open** | Mission 忽略 LocalizationStatus.session_id/session_epoch。同一 bridge producer、合法 sequence/lease 下切 canonical epoch 或 source，各连续发送 42 条新会话状态，旧 NavigateToPose 均未 cancel；随后 invalid status 能取消同一动作 | A+C native 事务联合启用阻塞。短暂 invalid 可在两个 tick 之间被新 valid 覆盖，depth-1 status 也不保证看见每个 invalid；须由 consumer 显式处理会话边界并验证旧 action/缓存退休。此轮不扩展生产状态机 |
| **P2 / open** | C atomic bridge 只做 finite 检查。PSD 正对照通过；x/y block [[1,2],[2,1]]（det=-3，特征值 -1 与 3）被标 valid；观察到 585 条含此矩阵的 valid status。A 的 full covariance gate 会拒绝同类输入 | opt-in 私有输入合同不一致；canonical producer 也只有 finite 检查，不能依赖 A gate 已关闭 AUD-015 来关闭 C。应统一接受合同并定向修复；这是受控错误输入反例，没有真实 estimator 产生该矩阵的证据 |
| **P2 / dependency gate** | 2026-10-24 与跨日收尾只读查询 FAST-LIO 远端：全部公布 refs 均不包含 8fb3296；本地 ancestry 核对为 false | AUD-052 不能立即以不可获取 gitlink 进入 main。先明确授权发布依赖并验证新 checkout 可获取；此任务没有 push 任意分支 |
| P3 / resolved fixture | A 的三个 publication-gate 用例直接调用 setup，没有加载 C 新声明的 zero_start_pose 默认值；首先失败于缺少 launch config，并非 producer 行为 | 最小更新夹具，先执行该 launch 的 DeclareLaunchArgument，再覆盖待测 flags；17 项完整重跑通过，无断言删除或门槛放宽 |
| inherited / unresolved | S03 历史 shutdown -11 仍无归因；本轮23次未复现不构成修复。M11 只有软件 checker，真实 prior/map/独立 landmarks 未验收 | C 整体 Software Modify 不变；原 S03/M11/真实输入门继续保留。F14 gated software transaction Resolved 也不代表 Mission consumer 已闭合 |

## Main 合入决定（未执行）

- **A：Software Go，可作为完整六提交候选进入 main 的软件评审/合入。** 399840c 不脱离 ddf988b；仲裁/minimal Mission 不拆；ea51474 必带。A+B 独立构建/回归提供了 C-free 的直接复核。启用 radar-only/返航 latch 的旧车语义仍按既有文档，不作硬件结论。
- **B：软件正确性 Go，交付有前置条件。** AUD-016/056 必须成套构建并 source 固定 Nav2 overlay，包含所有 BT action plugins，才能生效。AUD-052 必须先发布 8fb3296、保证新 checkout 获取，再采用 cbe2a1f；因此完整 B 集合目前还不具备无条件立即合入条件。没有为避开此门去重写或拆分公共历史。
- **C：暂不合入 main，保持 feature-gated / Software Modify。** upstream Missing=0 与 F14 Resolved 按原 parity 范围成立；它们不关闭 S03/M11，也不覆盖本轮发现的 P1/P2 consumer/input 差异。enabled 和 canonical_session_transactions 默认仍 false。
- **建议 selective merge。** 若只接 A，可在 main 未前进且获明确授权后 ff-only 到 ea51474；不是 ff 到 fed2cff。完整 A+B 的干净 checkpoint 为 a5354ca，须先满足 B 依赖发布/overlay 条件。**不建议 ff-only 到本 staging 末端；C 暂不合入。** 当前任务不执行任何 main merge/push。

以下继续保持 acceptance gate：real radar producer；AUD-003 authorization owner/session；MCU authorization / physical stop；real FAST-LIO/new-car bag；real prior/map/landmarks；chassis parameters；target-machine resource/performance。没有新增硬件接受、Deployment Go、Hardware Go 或 Omni 解冻。
