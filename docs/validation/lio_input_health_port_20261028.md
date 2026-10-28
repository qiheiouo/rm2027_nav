# LIO 输入健康与坏输出隔离迁移（2026-10-28）

阶段：Integration。裁决：**限定合同内 Software / Simulation Go；Hardware Blocked**。
只证明坏源隔离、连续恢复和已有 Mission 会话交接，不证明真实 FAST-LIO 精度、任意冻结
检测、物理停车或自主重定位。默认硬件配置没有被启用本候选。

依赖：`unified simulation@8375c8bdf98fbae323c18949a45db86f722fbe5f`
→ `event-driven replanning@e901f1b26479c4325562343047cf14acbbcf0d30`
→ 本候选 `codex/lio-input-health-isolation-port`。正式工作区是
`/home/qihei/projects/rm2027_navigation_clean`；不合并、不推送 main。
main 保持 `d37f8d6e5ebe53533b7fc62aca8bb84392bee31b`。
本报告与源码同一提交，交付回复给出最终 SHA；不为证据建立额外分支/tag。

## 1. 历史源码事实

只读来源为旧仓 `/home/qihei/.local/share/Trash/files/rm2027_navigation`。
精确候选 `fix/old-car-fast-lio-high-spin@46c521b2d2f8691f3cfd159b99fad8e7dc764ce7`。
旧 `main-old-car@820179457b70789b0f46697758dbe3d947ec1d4a` 是 09-01 版本，
**不包含** 09-03 候选；健康 helper 在旧 main 中不存在，不能把候选当作已进入旧主线。

核对文件：`rm_localization_adapters/{include/.../odometry_input_health.hpp,
src/odometry_input_health.cpp,src/lio_adapter.cpp,test/test_odometry_input_health.cpp}`，
以及旧 bridge 的 `handleOdometryValid`、车型 YAML 和
`test_odometry_health_withholds_global_validity_without_losing_correction`。

| 项目 | 46c521b2 的真实机制 / 缺口 |
| --- | --- |
| freshness | ROS receipt minus measurement stamp；旧车 age >0.20 s 或 future >0.05 s 锁存；零 stamp/非有限 XYZ 归入 stale |
| silence | adapter 以 steady clock 观察 raw receipt，>0.20 s 锁存；100 ms 诊断定时器 |
| latch / isolation | canonical pose 转换后运行门；拒绝则不发 canonical odom/odom→base TF，重置 twist history；发 latched Bool 与诊断 |
| recovery | 连续 5 帧新鲜且距最后可信 XYZ ≤0.75 m 后自动开门；并非 AMCL reseed |
| pose jump | **仅锁存后**检查恢复锚点距离；正常新鲜流会直接移动锚点，没有正常流跳变检测 |
| time / generation | helper 无严格递增/重复 stamp 检查；同一新鲜 stamp 可重复累计恢复；无 producer epoch / generation |
| NaN / Inf | helper 检查 XYZ；adapter 另有 pose/quaternion 检查，但不是覆盖全部速度字段的健康合同 |
| velocity / spin / freeze | helper 没有速度/角速度一致性或冻结检测；高转速本身不是 LIO 失效条件 |
| covariance | 不参与此 input-health gate；不能从小协方差推出定位正确 |
| frame / TF | 复用当时 adapter 的变换逻辑；健康 helper 不核验 frame 连续性，不改变任何 TF owner |
| global validity | bridge 的 `odometry_valid_topic` 消费 Bool；false 阻止 map→odom/global-valid；true 可直接恢复**保留的旧 correction**，没有新 session 交接 |
| backend diagnostics | calc_time、point_number 仅进入诊断，不是额外的判坏/恢复门限 |

历史报告中“LIO 跳变仍 fail-closed”的概述不能代替上述源码限制。
本轮迁移隔离与有限恢复的思想，并补齐现有会话合同需要的检查；没有照搬 Bool 授权链、
20 Hz Livox driver 改动或高转速 AMCL correction hold。

## 2. 历史证据分级

引用的历史记录：[09-03 FAST-LIO](old_car_fast_lio_high_spin_validation_20260903.md)、
[08-25 correction gate](old_car_map_odom_correction_gate_20260825.md)、
[08-25 工程报告](old_car_high_spin_localization_engineering_report_20260825.md)。
以下数值来自这些已保存报告；本轮检查所列旧仓 `artifacts/bags/` 目录，相关三个包均已
不在该位置。**没有重新解包/回放原始 bag，也没有声称重新核实全部原始现场日志。**

| 级别 | 可支持的历史事实 | 不能据此认定 |
| --- | --- | --- |
| A 实车主动坏输入注入 | 未找到 LIO NaN/断流/时间回退的实车主动注入闭环记录。受控高速动作不等同于注入坏 ROS 输出 | 不能认定完整坏源→Nav2停权→物理停车→恢复已实车闭合 |
| B 实车自然故障 | R03 `20260901_r03_spin_recovery_02`：约6.44 rad/s 时积压，odom age 最大3.495 s，约10 m跳变；FAST-LIO报“No point/Too few”；全局 correction gate 隔离大创新 | 这是 input-health 候选之前的失败，不证明其已成功拦截所有坏 raw pose |
| B 正常高转速及短故障 | `20260903_high_spin_native20_bundle_01` 202.56 s，4006 raw/4025 canonical；3.1 rad/s×12.55 s 与5.5 rad/s×9.05 s 的 LIO有效；两次短健康门事件位于动作外并伴录包负载，仍待复验 | 两次诊断事件没有完整底盘停权时间线，不能推导坏 pose 从未进入Nav2 |
| B 后续高转速 | `native20_hold_01` 130.29 s；≥3 rad/s连续52.751 s，峰值7.273 rad/s；1304个LIO-valid及1865个global-valid全true，1303个health全healthy；raw age P99/max=38.6/86.7 ms，mapTF最大gap61.8 ms | 支持该旧车20 Hz/修正冻结配置的正常高速连续性；不等于坏LIO注入/立即home验收 |
| C 离线实包分析 | 三帧确认后的最高速包显示持续AMCL错误；按角速度离线对齐推演≥3 rad/s冻结correction可拦截候选，随后另做现场hold验证 | 离线推演/回放不是Hardware Go，不能作为本轮新车阈值标定 |
| D 软件 | 旧helper四个gtest覆盖正常锚点、stale后连续恢复、恢复距锚点过远、future；旧bridge合成Bool false/true验证停TF及复用保留correction | 没有验证当前session、旧action和原子证据；不是实车注入 |
| E 设计/待验 | 高速结束立即home、独立重复、完整停车时间线、故障后大位移恢复范围仍未闭合 | 不得提升为新车硬件准入 |

旧记录能确认 high-spin 曾有真实 LIO 积压/发散，也有 AMCL 错峰这一不同故障。
不能把 correction gate 的实车隔离成功全部归给 LIO health。

## 3. 当前 main 与候选的覆盖差异

审计基点是实际 `main@d37f8d6`，再分别看本任务基点 `e901f1b`，不是把所有研究分支叠加。

| 能力 | main 覆盖 | 本次处理 |
| --- | --- | --- |
| `/localization/status` identity/epoch/sequence、source deadline、steady lease | 已有等价通信合同与单一bridge authority | 复用，给bridge增加LIO源前置证据 |
| canonical pose、frame变换、FD twist | 部分覆盖：有限pose、特定reference模式时间/重启保护、FD限速已有；通用输入健康缺失 | 在原adapter中隔离，不增加TF/odom owner |
| canonical covariance / validity研究 | research文档和COLCON_IGNORE快照存在，当前topic合同仍标pose covariance未验收、固定twist covariance provisional | 不搬研究能力，不伪造F9质量阈值 |
| Mission session handoff | main有lease但不等价于完整handoff；`e901f1b`才包含generation、取消等terminal、旧结果隔离 | 直接复用前项候选；本轮不新增Mission客户端 |
| AMCL / GICP | 后端gate与bridge两阶段有效性已有；不等价于LIO坏源门或correction innovation latch | 接入现有selected-status；修正同producer新epoch序号重置处理 |
| Z-Localization | 此main无该运行实现/合同树；`codex/z-localization-integration-20261023`等是独立候选 | 不引入，更不实现Z恢复 |
| FAST-LIO feature gates | bundle/no-input、backend TF隔离及特定reference约束已有；真实时序/外参未证明 | 不改算法/driver/async/adaptive开关 |
| input-health、通用jump/freeze/recovery generation | 缺失（部分协议检查散见别处） | 本候选补齐可观测范围；旧Bool链由原子源证据取代 |

## 4. 本次合同与设计

权威接口定义在 [adapter README](../../src/rm_localization_adapters/README.md#candidate-lio-input-health--bad-output-isolation)、
[bridge README](../../src/rm_relocalization_bridge/README.md#optional-lio-source-health-prerequisite)、
[topic合同](../contracts/topic_contract_2027.md#lio-输入健康候选2026-10-28)；此处只记录决策。

1. 原adapter在canonical发布前检查source时间、有限pose/twist、frame连续性、canonical位姿差。
   无可信raw twist时只用位姿/协议；可信base-frame twist显式启用后增加方向/幅度一致性及冻结检测。
2. HEALTHY允许输出；冻结的短确认窗标记SUSPECT；明确异常立即INVALID并隔离；连续可信
   样本为RECOVERING。确认冻结前的窗口是明确检测延迟，不承诺凭第一帧区分静止与冻结。
3. 新 `LioSourceStatus` 原子携带测量与源健康，不发布第二份统一有效性。invalid不携带可用pose。
   bridge启用后不再订阅独立odom用于匹配。失效清缓存、停止新TF、统一status=false并换session。
4. 恢复必须连续至少两帧（夹具5帧），严格新stamp、无silence、同frame、有限且动力学一致，
   同时接近最后可信平移/姿态锚点；第一帧不复权。future不推进高水位，rollback不复活旧session。
   bridge还要失效后的新global pose与健康odom匹配，不能靠旧correction或heartbeat恢复。
5. 旧epoch/generation/sequence envelope到达后惰性丢弃；旧raw stamp可使源再次失效但不能复权。
   暂停/回退时不能产生更大epoch则延后到时钟前进，绝不在同epoch内清零sequence。
6. invalid优先于路径失败：已有Mission取消Navigate/Spin，等旧terminal，丢弃旧session结果，
   才按当前Mission目标重新规划。health checker无action client、无速度、无串口、无独立急停。

没有AMCL reseed、GICP恢复、jump/rebase、自动map修正、Z恢复或radar定位fallback。
全局pose仍由已有backend/测试fixture提供；源恢复不主动制造全局修正。

## 5. F1–F12 与定向验证

环境：ROS 2 Jazzy / Gazebo Harmonic；真实adapter、bridge、Mission、Nav2进程；输入为
可控Odometry或统一物理真值派生夹具，不运行真实FAST-LIO/传感器/串口。

- 10包仿真构建及安装闭包检查通过：22个源码包，无重复，10个预期prefix。
- 直接相关45项通过：health gtest 14、canonical 15、bridge math 5、global gate 7、profile 4。
- 实际节点矩阵：直接global pose + passthrough通过；selected backend status + FD twist亦通过，
  包括同producer新backend epoch序号重置。两节点退出码均0。
- 既有Mission仲裁/延迟旧结果回归通过（1项综合测试，27.68 s）；原测试覆盖pending/active旧session结果。
- 物理仿真两次定向运行分别记录439/446帧canonical输出，均通过；下表时间使用最终446帧运行。

| 故障 | 结果 / 边界 |
| --- | --- |
| F1 normal | PASS：正常流、连续7 rad/s和导航运动未见非注入失效；正常到达，无额外ComputePath循环 |
| F2 stale/silence | PASS：实际FD链停止源后0.297 s失效；canonical停止，global健康持续也不能复权；steady-only断流另有单测 |
| F3 rollback | PASS：旧stamp拒绝，0.047 s失效；重复/乱序/ROS时钟回退单测；保留高水位 |
| F4 future | PASS：+5 s拒绝，0.030 s失效；正常时间数据可经连续恢复，不等待错误future高水位 |
| F5 NaN/Inf | PASS：pose NaN、twist Inf分别约0.048/0.050 s失效，未进入canonical；单测另含非法stamp |
| F6 jump | PASS：+10 m在发布前拒绝，实际FD链0.047 s失效；恢复不能跟随坏锚点；仿真canonical未出现跳变 |
| F7 high spin | PASS（软件边界）：7 rad/s连续一致输入通过；错误角速度方向/大角跳拒绝；旧车积压现象的时间异常由F2–F4覆盖。未重现真实FAST-LIO算力/点云根因 |
| F8 frozen | PASS（有可信motion cue）：stamp推进、pose冻结、raw v=0.5 m/s约0.396 s后失效；仍冻结不能恢复。无可信cue时不可观测，Hardware Blocked |
| F9 covariance | N/A质量门：当前covariance未验收；单测确认大数值不被冒充健康语义。不是“协方差退化保护已完成” |
| F10 recovery | PASS：前几帧不复权，连续5帧（FD再需初始化样本）及新global pose后有效，新session成立 |
| F11 delayed old | PASS：每种故障恢复后重放旧健康及旧invalid envelope均不改变新会话；旧raw/reordered单测拒绝；旧action结果由既有回归覆盖 |
| F12 active navigation | PASS：运动中跳变约0.030 s进入hold；等待重规划时断流约0.336 s进入hold；观察窗内零速且模拟位移<0.1 m；恢复重建后目标误差0.126 m |

协议异常延迟从夹具切换开始量到观测状态，含20 Hz输入采样，不是实时系统上界。
source timeout为0.3 s，加adapter 20 ms / bridge 50 ms / Mission tick与action取消调度；
物理停车时延必须另做硬件测量。canonical TF缓存仍可能可查询，健康以统一status为准。

### 事件重规划的最小时间线

以下为同一测试进程的steady相对秒；四次Navigate对应四次ComputePath，没有invalid期间的重规划：

| 事件 | 时间 / 行为 |
| --- | --- |
| 正常初始路径 | 6.450 s，实际开始平移 |
| 注入跳变→invalid | 7.330 s；旧Navigate于7.350 s为CANCELED |
| 连续恢复与新global pose | 11.458 s valid；新Navigate/ComputePath为11.549 s |
| 恢复目标完成 | 15.429 s SUCCESS；距离0.126 m |
| 第二目标FollowPath故障 | 16.471 s ABORTED，16.489 s进入Wait |
| 等待期间断流失效 | 16.811 s invalid；旧Navigate16.840 s CANCELED，Wait16.891 s CANCELED |
| 恢复后新目标/新路径 | 20.961 s valid，21.049/21.050 s新action/path；随后测试主动hold退出 |

每次invalid观察窗保持3 s无新ComputePath、无新canonical；恢复只按当前Mission目标建路径。
实际status publisher仅 `map_odom_from_global_pose`；launch关闭truth TF/status并将Nav2 odom切到lio。
最终无异常进程死亡、无所属进程组残留、正常SIGINT退出。

复现（仓库根，构建后source）与最小日志位置：

```bash
tools/build_simulation.sh
source install/setup.bash
colcon test --base-paths src/rm_competition_interfaces src/rm_localization_adapters \
  src/rm_relocalization_bridge --packages-select rm_localization_adapters rm_relocalization_bridge
python3 src/rm_localization_adapters/test/test_input_health_runtime.py \
  --upstream --finite-difference --output /tmp/rm2027_lio_health_upstream
RM_COMPETITION_MISSION_SOURCE_DIR="$PWD/src/rm_competition_mission" \
  python3 -m pytest -q -s src/rm_competition_mission/test/test_navigation_arbitration_runtime.py
python3 src/rm_new_car_simulation/scripts/validate_lio_health_navigation.py \
  --output /tmp/rm2027_lio_health_navigation_final
```

临时日志不入库；中断后续作检查时上述 `/tmp` 结果已不存在，保留本报告中的关键观测及
源码复现入口，未声称重新读取或重跑已清理的日志。首次轻量harness通过`ros2 run`只给父进程SIGINT导致退出超时，改为直接
启动安装的节点后正常退出；不是功能验收通过的退出记录。一次手动pytest缺少CMake规定的
源码环境变量，在测试启动前失败，按上面命令补齐后通过。两项均为测试调用问题，未掩盖产品失败。

## 6. 参数、误隔离与 Hardware Blocked

| 类别 | 本轮处理 |
| --- | --- |
| A 协议/时间语义 | 有限值、合法stamp、严格递增、generation/sequence、不得续旧source deadline、steady silence、至少两帧恢复可软件确定 |
| A/C 时间预算数值 | 夹具age/silence/bridge lease=0.3 s、future=0.05 s、5帧恢复；语义已测，真实时钟、频率、链路/计算延迟统计未标定 |
| B 动力学 | 夹具4 m/s +0.15 m step slack、12 rad/s +0.15 rad slack、恢复0.8 m/0.8 rad；需要新车范围、外参/杆臂与制动漂移验证 |
| B/C motion consistency | 可信raw twist才启用；误差2 m/s、3 rad/s，冻结0.4 s、0.1 mm/0.1 mrad精度与0.1 m/s/0.3 rad/s cue均是测试值，不是生产阈值 |
| C uncertainty/传感器 | covariance语义、FAST-LIO真实退化、点时间/IMU同步、速度独立性与冻结误报率均Hardware Blocked；未新建加速度模型 |

误隔离风险：时间抖动、超配置速度、插值/外参误差、真实运动超过恢复锚点，以及不可信raw
速度均可能触发hold；旧车录包负载下的短事件提示必须实测。降低这些风险不能靠放行坏pose。
无独立运动证据的pose/twist共同冻结、慢漂移、同frame隐藏rebase和被重新打新stamp的旧数据
不可由本门保证检出；必须保持单一受控backend producer。原有reference/heading latch仍可能
要求整条managed pipeline重启，本轮不会自动清除它们。高转速正常窗口通过不等于全工况零误报。

新车仍需真实bag、高速旋转、负载/延迟、外参、底盘运动范围、故障后的最终物理停车及恢复
重新建图路径验证。此处“恢复”是来源重新满足合同，不是恢复已发散的FAST-LIO内部状态。
因此给出限定软件/仿真Go，不给Hardware Go，不扩展到部署。

## 7. 下一层候选（仅记录）

| 层次 | 历史证据与建议 |
| --- | --- |
| A correction gate / fault latch | `e15b6bda`：旧车真实错峰被隔离，后续三帧确认/高速hold另有现场证据；建议下一项先单独迁移创新门与锁存，仍由现有map/odom owner执行 |
| B reseed | 08-25报告4次recovery completed且操作员确认；3次约1.05–1.10 s，困难一次11次reseed/21.803 s；是实车smoke，独立10次及停车时序门未闭合 |
| C jump/rebase | `51e3b233` 有基于最后可信位姿的受限恢复软件候选；不能把它称为已证明能恢复任意发散LIO；R03受控多次恢复边界待补 |
| D automatic orchestration | 旧车状态机/软件注入较成熟，但新session、运动授权、停车确认与新车backend生命周期需要独立整合；应在A及必要硬件证据后决定，不随本轮引入 |

保留当前候选分支、源码/配置/测试和本报告；没有新worktree需要清理。
