# RM2027 Prediction Value Test：历史复原与受限原生MPPI对照

2026-10-13，Asia/Shanghai，Research。本轮到 P3 停止，独立研究入口，
不恢复R1–R4/MPC开发，不定义R5，不迁入正式新车，不push/merge。

**最终决策：Modify。** 值得保留一个最小的时间感知预测输入/消费方向；目前不
支持复杂预测、恢复冻结控制器或全面替换MPPI。未来运动信息在受限原生闭环里
确有安全价值，但完美预测未稳定优于简单CV。此前把预测准确度或控制器更换
直接等价为导航收益，确实高估了其价值。完整STVL与真实新车证据仍不足。

## P0：身份、规范与资产

已读取稳定起点的 AGENTS、development_workflow、分级验证、研究资产与Git规范、
runtime_profiles、project_structure索引、新车阶段冻结和相关运行边界；另读取
main 的 repository_workspaces 和原工作区 WORKTREE_ROLE。采用 Research 的
最小判定标准，不全仓审计、不改定位/FAST-LIO/V2/MCU/AUD-003。

实际查询的本地引用（远端名称为缓存引用，未执行fetch或冒称现场远端查询）：

| 资产 | 完整源码身份 |
| --- | --- |
| 新车研究稳定起点/双雷达修复 | 4541e3754242c5353e2e5316b44f181f5e9f2721 |
| integration/new-car-software-remote-20261012 | 41b12f2d7f823c6989c6aaddd2c7266be9f3e683 |
| integration/new-car-dual-candidate-20261012 | 412aa4915ca4af8a5d6a9a2a1e73ba84553dc080 |
| origin/main-new-car | bd6cf689d3d5aca5c8edd430ff760ca5ceb27496 |
| origin/main-old-car | 820179457b70789b0f46697758dbe3d947ec1d4a |
| main/origin main公共入口 | fcd9a05b03c9539b4b5ea44de0f09d30c4468455 |
| 起点 FAST-LIO gitlink | 69f1070328c431f4c7b8bd586eca074141da3aae |
| 旧V1/风险冻结 peeled tag | e680b1430db6efd8dc601d5585ee207e5f9b42a4 |
| Temporal MPC冻结 peeled tag | 04291a410f193c009e043af88e014cf420e1f68b |
| R4冻结 peeled tag | bdb8b8d00308be4384ea94e8b060d60fd2173d30 |
| 历史索引及R2读取版本 | 9e08122a6df758b7f0131a5e2de5813a17f0e97c |

原主目录有9个dirty/untracked surface-member资产，原样保留，不带进新worktree。
R3/R4原始输入和本地构建仍在原worktree；本轮只读，不清理。本轮worktree
`/home/qihei/rm2027_navigation/build/prediction_value_20261013`，分支
`experiment/prediction-value-20261013`，直接从稳定修复分出，没有整体研究merge。
原有引用快照位于 experiment 的 protected_refs_before.txt。

## 历史因果矩阵：事实与假设分别记

编号沿用历史索引：R1是聊天future occupancy层设计；旧V1实际属于time critic。
R3与Temporal MPC为同一冻结路线，不能当两套独立实测方法。

| 路线/目标与复用 | 预测来源/未来时间/是否union | 传递与实际消费 | 已运行能力/失败 | 已证实根因与未证实项 | 可复用/不迁入/维护判断 |
| --- | --- | --- | --- | --- | --- |
| STVL+标准MPPI：反应式基线 | 当前3D观测的mark/clear/decay；无未来障碍轨迹；STVL的temporal是记忆年龄 | 当前2D costmap→CostCritic等；无公共prediction subscriber | 旧车配置、R4 Gazebo A24/A25运行成立；A25 S2优于R4；A26单通道5/5contact | A26失败事实成立，单独哪项目标/感知/采样导致未完成消融；不能说所有场景已足够 | 保留正式反应式导航/参数和真实失败；不用碰撞前时间冒称效率 |
| R1：future occupancy写空间层 | 理论KF/CV未来帧；合并会失去时间 | 审计没有可构建加载的future occupancy Nav2层 | 完整R1闭环A/B未运行 | 永久墙/false block是机制推断，不是已证实R1实测根因 | 保留设计索引；不建立/迁入未来并集层 |
| 旧V1及差分风险：HWS风格tracker→时间critic | 因果scan、关联、KF；age+(k+1)dt；按步CV/CA包络，不是空间union | PredictionV1Critic确实进入native成本；后续AR/mixture/proposal主要离线反事实 | 固定首例有净空/recovery收益；重复碰撞/90s不到达；冻结周期有安全候选仍输出破门 | 硬罚共同前缀打平、某状态白噪声候选无安全覆盖、排序PASS后SG/聚合FAIL均有固定输入反例；未来中心/完整形状不可靠。不能把不同周期原因合成唯一根因 | 复用source-time合同、geometry、native证据；不迁AR/CA/mixture或遥测fork；冻结维护 |
| R2：原装MPPI+DynamicObstacleCritic+guard | public position/velocity/v2 anchor，CV重算全30×.1s；不读显示prediction[]；无union | pluginlib原生消费/成本/weights/SG与SDK输出曾精确核对 | baseline18.692s和critic19.679s均几何重叠；guard35s只推进.452m；visible接触，nominal不到达 | 初版圆完整箱体0/261覆盖、3s中心误差中位2.006m证明输入模型不足；nominal当前覆盖改善却换向未来支持失败；guard静态/unknown保守堵住任务。新试次不能仅凭碰撞归sampler | 复用纯几何、连续时间成本/检查；不迁完整guard、nominal大圆或第二消费者；暂停 |
| R3/Temporal MPC：静态T-DT压缩→短时QP→执行重验 | public预测/source age；fixed-yaw1.5s/15节点/20Hz；无未来union | 已有最小RealtimeMPC.solve；异步worker/重锚/移位/原生控制plugin确已执行 | 多轮真实Gazebo均失败/取消；离线部分可行。新v2修复旧vy投影拒绝后仍因未来净空回退 | 原制动dt模型错误由独立replay证实；新预测/状态重锚改变余量、旧序列老化改变余量均有2×2反事实；95.474ms输出观察gap超门。余量替换不等于可加的物理唯一因果 | 可读用solver/时钟/执行诊断；不替换MPPI、不重跑/改冻结路线；冻结 |
| R4：HWS式raster/分时soft field/free progress→局部QP | canonical KF+同次members observed raster，绝对stage epoch；1.5sCV，不是并集 | C++库、WorldFollowAdapter，runtime wrapper替换XY/委托native angular；确实执行 | A23动态20/20无contact，B0 16/20；A24对齐后均10/10无contact；A25 B0 S2更快且更宽裕；A26恢复cohort R4 gate5/5、全目标2/5，B0contact5/5 | A23速度/circle联合混杂经A24/A25否定独立优势；A26三次近目标去重plan<2点degenerate path确定自身输入缺陷；native角委托先异常曾截断等待观察；未证明简单reactive不可替代 | 可读用stage契约、消费库、失败输入；不迁新owner/lease/fallback/全controller；low-level冻结Stop |
| HWS上游时间消费参照 | cloud静态差分→Hungarian CV KF+local raster→decayed future CostMaps | planner窗口union为hard view；follower按k·dt选CostMaps；自有FDDP/LPV/MPC | 上游源码完整；本地没有迁其renderer/maps完整后半段；本地A/B不能代替上游同条件验证 | 源steady clock/latest TF与消费source-age未见完整严格对齐；是源码风险，不是本轮测得的上游闭环失败；“浙江大学成功”没有本地同条件证据 | 复用时间索引思想；不整体迁上游、更不将纵向模型直接代全向vy |
| NEU/T-DT规划迁移 | YAstar/MinimumSnap/Sfc静态前后端；annotated ETA及shadow动态准入接口 | dynamic_clearance report没有正式执行consumer；已迁静态规划器不是时间障碍控制 | 静态闭环/失败回退有证据；R4初1.4m corridor被Sfc raw support拒绝 | 特定窄通道证书拒绝由原Sfc调用复现；没有证明NEU静态前端就是动态预测失败 | 保留已迁静态规划与接口；不整体导入/替换全局架构 |

12项检查的横向结论：预测source/time、union、消息和MPPI消费见上表；旧V1/R2/R4
都有实际消费，绝非全部“忘记接线”。轨迹与障碍通常按未来同刻评分，但consumer
epoch→SG returned offset→smoother→plant的全链仍存在差异。候选不足和优化/
排序/非凸聚合失败在旧V1有固定反例，不能自动归给其他路线。控制延迟/过期在
R3真实时序有失败，R2guard有stale/watchdog统计。过度膨胀与false block在R2
nominal/guard有任务反例，R1只是机制风险。R4 Sfc拒绝是静态前端/输入限制。
离线可行而闭环失败在V1/R3已有证据；不能以预测精度、测试数量或无碰撞停车
覆盖未到达。本轮不会以未测量的“模型一定是根因”代替对照。

历史权威读取入口（均在旧资产，可由上述ref恢复，本轮未复制大型历史树）：

- archive `docs/dynamic_navigation/dynamic_obstacle_prediction_full_history_20261004.md`、
  `hws_migration_audit_20261003.md`、`hws_vs_current_prediction_analysis.md`。
- e680 `docs/dynamic_navigation/research_stage_conclusion_20261001.md` 与
  `standard_output_cost_alignment_20261001.md`：三层反例/最后限定阳性。
- archive `docs/dynamic_obstacle_critic/validation.md`、
  `observation_diameter_consumption_experiment.md`、`dynamic_surface_reveal_experiment.md`。
- R3 `docs/dynamic_navigation/temporal_mpc_freeze_20261004.md`、
  `temporal_mpc_gazebo_results_20261004.md`、`temporal_mpc_timing_results_20261004.md`。
- R4 `docs/dynamic_navigation/r4_{finite_closed_loop_comparison,matched_closed_loop_comparison,
  clearance_efficiency_pareto,final_corridor_research}.md`及紧凑trials/summary；A24、
  A25、A26均是实际运行，非本轮新运行，原失败/原始输入保留。

## 基线、固定消费者与P0可行性

旧车正式source profile为 `nav2_old_car_2026_left_stvl.yaml`，Omni MPPI七标准
critics，10Hz、dt=.1、30节点、300候选、1迭代、std=.2/.2/.4、temperature=.3、
gamma=.015，vx[-.3,.5]、vy±.5、wz±1.2。smoother20Hz，axy=.8、aw=2；
local8×8m/.05m，footprint(.32,.27)+.02，STVL voxel_decay=.5s/linear、
双mark/clear观测和.30m inflation/factor6。原运行入口是 old_car_full_navigation
→competition→validation→Nav2；full入口实际默认three_point_spin_test（3m/s/
10rad/s HIGH-RISK），不是本研究基础低速profile。不能仅凭文件存在宣称现场
一直使用同一低速参数。本轮不启动上述硬件入口。

新车固定集成仍以普通VoxelLayer为待硬件验收障碍入口，STVL并未默认接受；
`docs/new_car_hardware_acceptance_20261012.md`第8步明确先验当前layer，之后再
决定STVL。基础软件稳定不等于新车完整动态导航基线已建。

离线镜像实查 MPPI `1.1.20-1jammy.20260607.083249`、STVL
`2.3.4-1jammy.20260607.082704`。ROS Humble/GCC11.4/Python3.10；host只有
Jazzy，但本轮不重建ROS、不混ROS版本。固定本地1.1.20的optimizer/critics/
noise源码与安装接口用于调用检查。上游现行版本有Eigen优化与optimal trajectory
validator等能力，不能反向填进本地Humble。
[固定optimizer](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/optimizer.cpp)，
[现行validator接口](https://api.nav2.org/nav2-rolling/html/classmppi_1_1OptimalTrajectoryValidator.html)。

原装MPPI没有未来障碍输入；critic扩展点可用，已有R2动态scorer可最小适配。
R2按source状态重算CV，改消息prediction[]不会改变实际消费，所以新薄入口明确
传入stage位置数组；余下调用原生共享库函数，不重写MPPI。R4需要members/Sfc/
OSQP和角委托，R3需要corridor/worker/重锚链；本轮不引入这些额外控制变化。
共同消费者可成立，但只在受限native proxy闭环内成立。

完整STVL可安装运行且已有历史证据；本轮为低成本信息隔离，主动选择当前观测
disk直接栅格化的A_proxy，不称STVL。它省略voxel记忆、3D/frustum/遮挡、全局
搜索/BT恢复。所有组共用.5m inflation（参考A24包络一致性问题）、同一padding。
这是一项明确的P0限制：可测同一MPPI消费中的B/A'/CV/C差异，**不能测完整正式
STVL系统是否足够**；A_proxy与B差异包含消费者成本+信息，必须与A'/CV分开。

## 冻结实验与资源

详见 [运行工程](../../experiments/prediction_value/README.md) 和
[protocol](../../experiments/prediction_value/protocol.json)。P1横穿/等待释放各seed11，
5组共10次，不充正式样本；P2 empty+5类场景×seed13/29/47×5组=90次。
不得事后挑样本，碰撞/超时均入分母，参数在正式结果前提交。

场景是历史S1/S2/S3运动语义的有限代理，不复刻原GazeboSDF；持续横穿、迎面、
1.4m单通道、突然在中央停止、占据后释放以及空场；phase±.2s确定变化，源代码
就是全部场景定义。D如运行只作离线有限可行见证，不能称最优上界或实时组。

信息/控制时序：10Hz观测，100ms观测延迟，sigma=.01m完整disk质心，3.5m范围，
无遮挡；10Hz控制，3s预测；10ms物理推进，输出50ms+实测调用延迟；四全向
body-velocity、中点积分与.8/.8/2加速度/既有速度限幅。代理半尺寸(.325,.300)，
统一padding .03，不冒充新车CAD。B未来真值只进prediction接口，不给更准自车
状态或无延迟控制。C完全复用canonical KF；简化感知偏乐观，不能外推实车误差。

碰撞与安全成功分列；body clearance .05m、3mm数值容许；goal .15m/.2rad。
效率门冻结10%配对中位与≥2/3方向一致且≥2场景，无新增安全/任务退化；安全门
要求某场景B3/3安全成功而对照至少2/3失败且无其他新退化。n3只作研究信号。
100ms共同计算预算；超时/异常经同一延迟限加速度零目标退化，不认证安全停止。
到达时间只报告成功；同时保留失败时刻、路径、截断WAIT、指令变化和计算分位。
不以始终停车评最佳，不预设MPC获胜。预测刷新age/同刻评分以及真实消费以日志
检查；ROS/DDS/真实执行ACK未运行。

首次核查available约6.13GiB（后续字节测量6532554752）；新worktree约9.3MiB，
新增cache预算150MiB、证据30MiB，保护3GiB。不复制巨型worktree、不新建镜像。
只允许清理本轮可再生cache，历史数据/冷备/其他worktree全部保留。

## P1/P2/P3结果

P1 seed11 两场景×五组共10次已完成；独立代理闭环确实推进并执行原生输出。
横穿A_proxy/A'碰撞、CV/B/C到达；等待释放全部到达，B更慢。检查与首次启动
失败保留；不是正式样本，也不以该结果调整场景、参数或判据。

P2前仅修正共同观测噪声合同：每个source tick固定抽两噪声，即使目标不可见，
避免路径分叉后因range不同造成同一时刻随机序列错位。该修正不使用性能标签、
不改控制参数/运动/预算；P1旧cohort原样保留。以上是P2前登记，正式结果如下。

### P2完整运行（算法/场景/口径不变）

正式90次全部完成：空场+5动态类、每组每场景3个固定seed。10次P1单列，未
用于正式分母；一个footprint启动失败及一个补充检查放置于候选影响范围外的
失败均保留。后者只修检查输入，不修改消费者/正式样本或算法。不存在挑选
重跑/替换碰撞。正式controller源代码与协议为 `981eb7b6` 检查点，原生
consumer本身在 `0711742c` 已固定。下表压缩每场景结果，完整30行表、每seed
原值和全部压缩周期在 [P2证据](../../experiments/prediction_value/evidence/p2/)。

到达时间为成功样本中位，失败不补0；“到达/碰撞”均分母3。所有到达的
trial均满足本轮sampled硬净空门。body最小净空是动态signed几何，不是Gazebo
contact；墙净空另统计，所有组没有采样墙碰撞。

| 场景 | A_proxy到达/碰撞 | A'到达/碰撞 | CV到达/碰撞；到达s | B到达/碰撞；到达s | C到达/碰撞；到达s |
| --- | --- | --- | --- | --- | --- |
| 空场 | 3/0 | 3/0 | 3/0；8.9 | 3/0；9.0 | 3/0；9.0 |
| 横穿 | 0/3 | 0/3 | 3/0；12.6 | 3/0；13.2 | 3/0；13.1 |
| 迎面 | 0/3 | 2/1 | 3/0；11.3 | 3/0；17.6 | 3/0；18.0 |
| 窄通道 | 0/3 | 0/3 | 3/0；15.4 | 3/0；17.3 | 3/0；16.2 |
| 突然停止 | 0/2，另1timeout | 0/2，另1timeout | 3/0；15.0 | 3/0；15.6 | 3/0；17.3 |
| 等待释放 | 3/0 | 3/0 | 3/0；15.1 | 3/0；16.0 | 3/0；16.0 |

合计A_proxy6/18成功、11次碰撞、1超时；A'8/18成功、9次碰撞、1超时；
CV/B/C各18/18安全成功、0碰撞，其中每组动态15/15。这是受限proxy事实，
不能改写为正式STVL碰撞率，更不是实车表现。A_proxy缺BT/replanning/原STVL
记忆，native失败后只执行共同零目标退化，不能据此宣布生产基线很差。

| 动态场景 | CV最坏/中位body净空(m) | B最坏/中位(m) | C最坏/中位(m) |
| --- | --- | --- | --- |
| 横穿 | .299 / .394 | .359 / .401 | .253 / .290 |
| 迎面 | .174 / .186 | .159 / .184 | .147 / .209 |
| 窄通道 | .340 / .386 | .385 / .443 | .075 / .400 |
| 突然停止 | .276 / .282 | .207 / .219 | .229 / .234 |
| 等待释放 | .401 / .415 | .371 / .419 | .447 / .454 |

**按冻结门槛判读：** B相对A'在横穿、窄通道、突然停止满足安全信号（B3/3安全
到达，对照≥2/3失败，其他场景无B新增失败）。B相对简单CV的安全增益为0个
场景，效率10%门也为0个场景；不移动门槛、不拿平均净空弥补这一否定结果。
这支持“运动信息能进入已有MPPI”，不支持“越准确越好/复杂预测必然值得”。
代理安全门的阳性保留；最终Modify来自完整STVL/感知/真实执行范围缺失，而
不是事后把阳性改判失败。没有达到正式Integration或Deployment Go。

逐seed成对B/CV到达时间变化中位：横穿+2.33%、迎面+45.13%、窄通道+13.21%、
突然停止0%、等待释放0%。这些是配对相对变化的中位，与“各组时间中位相除”
不同；不能混算。B在迎面三对都更慢，KF在突然停止三对都比B慢（B/C配对
中位−12.85%）。等待释放Bseed29=18.4s，CV=14.9s、A_proxy=15.8s；这是明确
负收益样本。空场同seed各组最多.2s差来自实测调用时序反馈，不能认作预测收益。

### 预测、消费、执行和退化

三类信息分别判断：

1. **信息价值成立的范围。** A'复用同消费者且v=0；B与其差异不再包含新
   MPC/轨迹库/标准critic改变。横穿/窄通道B避免A'的碰撞，支持未来运动和
   阶段位置有用。B还包含未来位置精度；在突然停止尤其不能把oracle相对A'
   的差异全部解释为运动方向，因为位置噪声及未来停止事件也属于oracle信息。
2. **完美预测的追加价值未成立。** B与CV/C共用消费者、采样、成本、执行。
   C保留了本轮全部15个动态安全任务，简单CV同样15/15。没有正的B/CV效率
   增益分母，因此不能编一个“预测收益保留百分比”。C净空在窄通道seed47
   只有.075m而B=.385m；即使成功数相同，风险余量也不是全部保留。
3. **消费者确实利用预测，执行仍有瓶颈。** B1979、C1854、CV1901个有效
   预测周期进入评分；实际成本/command响应和startup readback证实消费。
   B有11周期存在条件安全raw候选而SG后聚合rollout按同一GT时域不安全；
   即使最终闭环全部到达，完美预测也不保证每个输出预测轨迹安全。这是局部
   消费/聚合的反例，不能唯一指定哪项标准critic或SG为根因。

预测误差（各组自身可观测周期，3个trial的均值；不是同源窗口严格成对误差）：

| 场景 | CV 1s/3s位置误差(m) | C_KF 1s/3s(m) |
| --- | --- | --- |
| 横穿 | .230 / .698 | .046 / .112 |
| 迎面 | .215 / .589 | .024 / .046 |
| 窄通道 | .208 / .586 | .023 / .046 |
| 突然停止 | .233 / .746 | .079 / .257 |
| 等待释放 | .266 / .917 | .162 / .631 |

B误差0是oracle定义检查；stage grid在代理时钟下对齐，不是DDS/TF同步认证。
KF平均更准却不总更快，CV误差更大却全部任务成功；不能用此结果把CV噪声
视为可部署探测策略。KF对停止的惯性在seed29保留：t3.6s的3s预测y≈1.831m，
真实障碍已在0；t4.5s降至≈.280m。新观测能纠正，控制也安全到达，但没有
可信度下降→降级CV/ZOH的显式policy；不能声称已实现可靠置信度退化。

存在过于乐观的完整aggregate预测周期（预测padded>.02但GT≤.02）：A'398、
CV253、C149、B0。它们是未来30步的离线风险标签，不是实际碰撞次数；闭环
每.1s重算可能避开原未来风险。反过来B“全部到达”也不是连续安全证书。
C在目标可见而prediction未ready时196周期退到同一当前占用native链，明确
记录unavailable，未虚构缺失轨迹。无消息失序/丢失或stale故障动态trial；实际
过期丢弃率为未运行，不把固定同步provider的0过期写成真实系统性能。

本轮native异常周期：A372、A'373、CV2、B0、C4，全为
`Optimizer fail to compute path`，保留原原因。共同回退只是将零速度目标排入
同一延迟/限加速度plant，绝非安全制动证书。横穿seed29 A在t3.2s首次异常时
已经接近动态障碍，当前观测(2.013,−.522)确实存在，旧平移速度仍约.5m/s；
之后发生穿透。不能把“输出零”视为立即停止或说A没看见目标。

实际控制品质：所有组速度/加速度由同一plant限幅，归一化加速度最大
1.0000000000000009（浮点误差），无模拟动力学越界；这由模型保证，不证明
实物可执行。B最大前进方向回退.615m，C最大.996m，CV最大.262m；迎面B/C
有明显后退后绕行，而CV选较直接侧向绕行。停止/启动次数、路径长度、指令
jump、每trial方向切换在summary/diagnostics保存；不忽略这些控制代价。
“无意义等待/非必要绕行”没有可靠独立最优标签，保持未定义，只报告实际
WAIT/路径/参考见证，不人为标为浪费。

### 计算预算与执行时序

| 组 | 完整native调用往返p50/p95/p99(ms) | 最大(ms) | 原生异常周期 | >100ms周期 |
| --- | --- | --- | --- | --- |
| A_proxy | 2.034 / 2.786 / 3.472 | 4.724 | 372 | 0 |
| A' | 3.448 / 5.800 / 5.877 | 39.087 | 373 | 0 |
| CV | 3.334 / 3.899 / 4.196 | 6.708 | 2 | 0 |
| B | 3.245 / 3.833 / 4.053 | 6.561 | 0 | 0 |
| C | 3.343 / 3.916 / 4.097 | 7.271 | 4 | 0 |

正式11094个消费调用，全部异常也有实际往返耗时，不以0加入延迟统计。原生
内部compute_ms另保存，以上包括地图构造、native计算和JSON管道往返，未
包括离线压缩/绘图/日志统计。感知和prediction source建模为相同10Hz同步输入/
100ms source age；Python KF/frontend处理耗时未作为独立延迟模型或planning
预算计入，不宣称整条ROS/感知端到端100ms认证。

50ms输出延迟加每次实际native往返耗时，延迟期间执行旧指令；以10ms网格
近似apply时刻。没有因oracle暂停plant或另给B自车未来状态。10Hz低速proxy
可执行，不能据这些低耗时宣称真实MID360/STVL/Nav2 executor的20/50Hz能力。
Humble原生rollout不含共同plant的加速度及实际输出delay；评分与执行模型差异
被保留，未重写MPPI。异常零目标、确认缺失和大future误差都实测，但DDS消息
老化、硬件apply ACK、故障watchdog的实际fallback尚未运行。

### D：有限离线物理见证

D在每场景/seed固定检查30个schedule：0..18s每2s等待 × 0/±.9m侧移，
同一速度/加速度/.05s输出延迟、同一geometry和10msplant，用GT只作离线
验收与选最好已找到者。无实时预算、未加native计算时延，不属于部署控制器。
18个场景/seed都找到≥1个满足.05m门的到达见证；540个有限schedule的全部
成功/失败摘要保留，不代表同伦/全局完备，也不是最短时间下界。

找到的到达时间：横穿12.2s，迎面12.5s，突然停止12.5s，等待释放16.2s；
窄通道seed13=16.2s、29/47=14.2s。迎面/停止见证快于B中位，支持B保留消费/
成本/动作选择余量；但CV已经成功且较快，不能据此要求新MPC。D有解也不
证明本轮native sampler能生成该schedule。见 [D记录](../../experiments/prediction_value/evidence/D/results.json)。

### P3裁决与问题的明确回答

**Modify，暂停等待评审。** 最小值得保留的方向是将已有R2连续时刻评分与
明确stage输入/源时间/几何合同用于一个STVL当前链内的隔离对照，再检查原生
输出在真实加速度/时延下的结果。只需要一个prediction adapter/consumer，
优先复用已有KF与MPPI。不是新tracker、新轨迹库、R5或Temporal MPC迁移；
本轮不实施下一阶段。

- 现有STVL+MPPI是否足够：历史A25 S2足够且支配R4，历史A26通道则有5/5
  contact；本轮A_proxy不能推翻或确认完整正式STVL。对全目标场景不能答已足够。
- 理想预测是否有可实现收益：本proxy对ZOH有实际安全任务收益；对已有简单
  CV没有稳定追加效率/安全任务收益。它是固定消费者实现的oracle性能参照，
  **不是信息论价值上界，也不是最优控制器上界**；更准确输入不保证启发式
  消费者输出单调改善。不能将B表中性能当预测潜能的最终天花板。
- 实际预测保留多少：本轮C保持15/15动态安全任务，但净空和时间没有统一比例；
  真实可见面/遮挡/完整体误差未覆盖，历史R2的0/261覆盖仍有效，不能用乐观
  centroid proxy替它洗掉。
- 之前失败主要在哪里：是输入几何/长时运动、候选/总排序/非凸输出和执行
  老化多层问题；部分shadow确实无行为consumer，但已运行R2/R4不是未消费。
  历史逐层反例与本轮实际消费证据都不支持“只提升预测精度就解决”。
- 是否继续投入：**可以研究最小可靠时间消费；当前不值得投入复杂预测或全面
  控制器替换。** 先保留反应式基线，完整STVL共同consumer/实车输入证据过门
  后再决定。无新授权不继续运行，接受下一阶段可能Stop。

未运行：完整STVL同消费者A/B/C新闭环、真实LiDAR/遮挡/数据关联、真实车辆
CAD/执行响应、Gazebo接触传感器、ROS时钟/TF/DDS/消息故障、取消尾段、实车。
没有将这些设计/历史测试当成本轮PASS。代表轨迹固定展示seed29全场景/全部
组；跨所有seed的成功/负例均保留，未挑最好的一次。

![全部场景的固定seed轨迹](../../experiments/prediction_value/evidence/p2/xy_all_scenes.png)
![全部场景的固定seed推进](../../experiments/prediction_value/evidence/p2/progress_all_scenes.png)

## Git、资源与恢复

完整HEAD通过 `git rev-parse experiment/prediction-value-20261013` 和本轮ignored
closeout manifest记录，避免文档自引用SHA。相对起点修改只有
`experiments/prediction_value/`、本报告及项目文档索引研究链接，不修改任何
正式源码/配置或submodule。原研究/正式refs保持，原主工作区9项WIP保留。

研究产物：P0协议、源码来源、正常/失败输入输出、P1/P2全部压缩周期、小规模
摘要/图、D有限见证；这些需保存，不是缓存。可再生部分仅本轮
`build/prediction_value_cache/`（编译/运行宿主日志）和Python字节码；可在
确认提交后按本README重建，不清理历史。唯一新运行数据已入本研究分支，
本轮不push，故其Git提交及本机磁盘是当前唯一保存位置，不能直接删除Git库。

恢复：在现有worktree查看该研究branch并按README命令build/checks；复现新
trial时使用新的evidence目录，不覆盖已提交cohort。回退：停止本入口并返回
稳定起点4541e375的现有新车worktree即可；正式导航没有启用任何本轮模块，
无实车命令或配置需要撤回。历史R1/R3/R4均从上述冻结tag恢复，未重写历史。

资源末次测量（最终提交前）：研究证据15,080,588 bytes（约14.38MiB），本轮可重建cache2,074,702 bytes（约1.98MiB），全worktree约27MiB；磁盘available6,513,459,200 bytes（约6.07GiB）。114个既有保护refs核对未变，全部90个正式输入周期压缩文件、协议SHA与结果组合核对通过。没有大规模构建或清理，始终高于3GiB保护线。最终提交后实际HEAD/clean与磁盘值另见ignored closeout manifest及交付消息。

浙大相关称谓边界：已核查的可调用时间消费资产为上述固定HWS代码及本地冻结R4/R3；未找到另一个有固定源码身份、同条件运行证据的独立“浙江大学MPC”基线。聊天中的学校称谓不作为作者身份或成功效果证据，本轮没有补造其本地实验。
