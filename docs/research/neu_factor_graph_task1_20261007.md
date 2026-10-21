# Task 1：odom + scan-to-map 离线后端

2026-10-07，Research。执行[串行路线](neu_serial_research_route_20261007.md)，仅 Task 1。固定新车源码 `bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`；FAST-LIO submodule `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321`。不改前端、正式桥或 TF owner。

**当前状态：Modify / waiting for real vehicle evidence。** 离线闭环已实现；设计噪声正控制减少误差及 correction 跳变，但已有 GICP 的合成 PCD 对照在固定权重下没有 causal 精度收益。新车输入、增量噪声与地图信息标定仍缺。**不进入 Task 2，不继续用 toy 调权重追求 Go。** 实车到位检查表、数据接口和固定 A/B 恢复入口统一见 [§8](#8-实车到位检查表与恢复入口)；§1–7 保留已有证据。

续审已闭合 covariance 的部分代数及逐段私有输入接口，确认 native publication 顺序 / 索引问题和 heading 模式的四维投影。**真实权重来源仍未闭合**，详见 §6；原5组对照保留。

完整姿态续审给出了已知单轴 / home / 启动姿态条件下的私有 full-base 重建，倾斜对照支持数学可行性；同时暴露启动重力基准、heading 定义及奇异分解的必要条件。详见 §7，**尚不能替换新车正式输入，Task 1 仍 Modify**。

## 1. 实验前定义

核心假设：在同一数值基准和 epoch 内，可信且有适当方向权重的 scan-to-map 测量，联合局部增量约束，能比每次直接替换 correction 减少噪声与长期漂移；不能因此证明全局唯一性。

Baseline：V 的桥接受已有门通过的 `S_i=T_map_base(t_i)` 后计算 `C_i=S_i*inverse(O_i)`，后续用 `C_i*O(t)`，下次接受时替换。源码 `src/rm_relocalization_bridge/src/{map_odom_from_global_pose,relocalization_math}.cpp`。本实验比较的是**接受之后的估计机制**；不声称复现未验证的新车 gate / recovery 整链。时间完全匹配、无乱序输入，不绕过真实运行的 age / nearest-odom 语义。

最小判定实验（运行前固定，不搜索阈值）：

1. 完整 3D 无噪声与 gauge 对照，检查 SE(3) 残差 / map/odom 重建。
2. 正常漂移 + 正确 basin、独立合成观测噪声；比较 causal prefix 的位置 / 姿态误差与 correction 变化。全序列 batch 另列，不能与在线 baseline 混为公平实时结果。
3. 同一组地图因子整体左乘 12 m 平移；检查代价和相对约束是否仍相同，拒绝用低代价证明全局正确。
4. 单次全局 anchor 后，纵向零信息的 corridor 观测；检查没有凭空补足纵向约束。
5. 少量独立采样的合成 PCD，调用已有固定 GICP probe，将结果变成 scan-to-map 因子。只证明文件 / 配准 / 图的软件连通，不给出赛场精度或 toy 调参结论。

停止标准：数学闭环错误先修复；正控制无收益则 Modify / Stop；即使正控制支持方法，新车实测基准 / 权重 / 地图尚缺时总体仍 Modify，不自行开始 Task 2。

## 2. FAST-LIO 输入审计

东北大学校订转录 §3 / §6 明确提出前端自建局部地图、每帧 SE(3) 后端，odom 增量保持连续、先验地图作测量。没有公开其求解库、噪声标定、边缘化或源码；本轮是本项目最小模型，不是精确复现其完整后端。[L0 原分享复原](neu_localization_controller_second_audit_20261007.md#3-东北大学方案忠实复原) 保留不确定项。

| 项目 | 源码依据 / 判断 | 本轮处理 |
| --- | --- | --- |
| Canonical pose | adapter `/odometry/lio` 为 `odom→base_link`；同 epoch 的 `inverse(O_i)*O_j` 可构造相对因子 | 位姿结构足够，不替换 FAST-LIO；外参 / home / heading 实测缺失，不能宣布新车输入已可用 |
| Cloud / odom 基准 | raw cloud 与 raw odom 同用 publish lidar time；heading fusion 将原 R 换到 B0；同名 `odom` 点值不会自动变换 | 沿用 [L2 数值基准缺口](neu_localization_l2_global_candidate_prototype_20261007.md#1-a-线真实输入闭合进度)；PCD 对照显式构造同一 O 基准 |
| Raw covariance | upstream `laserMapping_bundle.cpp:728` 的 `publish_odometry` 先 publish，再将 `kf.get_P()` 填入消息 | 不能当成本帧 P：连续调用有上一轮值、首帧默认值风险。源码审计，未实车复现、未改 upstream |
| Canonical covariance | `lio_adapter.cpp:663` 附近复制消息、转换 pose，基本沿用 covariance，heading 模式仅替换 yaw diagonal `.01` | 没有完整传播 lever arm / parent basis / heading 相关误差，不能直接当后端权重 |
| Heading 模式 RP | `canonical_odometry.cpp:87` 后 `base_rotation.setRPY(0,0,base_yaw)` | 平面 chassis 假设；坡道 / 倾斜时不能把零 RP 当可信 full SE(3) 观测，需要确认状态及未观测方向 |
| 增量 covariance | 绝对 `P_i/P_j` 没有跨时刻 `P_ij`；相邻滤波状态相关 | 不简单相加 / 相减，不把零数组当无限置信；本轮用注明来源的 nominal incremental 模型 |
| GICP 信息 | V covariance 来自 fitness clamp；T H 为 centered average point-to-plane 质量近似 | 不直接变成校准信息 / posterior covariance，不迁移 T 阈值 |
| 当前桥 | source 时刻 nearest odom，`.05 s` 容差，`S*inverse(O)` 直接替换 C | 不是插值或 smoother；公平 baseline 只比较已接受后估计公式 |

源文件：`src/rm_localization_adapters/src/{canonical_odometry,lio_adapter}.cpp`、`src/rm_gicp_relocalization/src/gicp_relocalization_node.cpp`、`src/rm_relocalization_bridge/src/{map_odom_from_global_pose,relocalization_math}.cpp`。FAST-LIO 在原始冷归档只读核验，pin 与 V gitlink 一致。本轮 `src/` 无修改。

用户已确认没有 A 实车记录。A 的实际 IP / topic / role、完整外参、clock / point-time / deskew、raw/canonical 同基准、真实 bundle 及独立建图-query episode 仍缺。继续复用 A/B CAD；机械坐标不替代 sensor / IMU 标定，不重新猜测或修改正式硬件配置。

## 3. 最小后端与 map/odom

[实验实现与复现入口](../../experiments/neu_factor_graph/README.md)：独立 JSON CLI，`COLCON_IGNORE`，无 ROS publisher；已有 SciPy 1.11.4 稀疏数值 Jacobian batch least squares，不安装 GTSAM。

```text
O_i = T_odom_base(t_i), Z_ij = inverse(O_i) * O_j
S_i = scan-to-map 的 T_map_base(t_i)
X_i = 待优化 T_map_base(t_i)
r_odom = L_odom * Log(inverse(Z_ij) * inverse(X_i) * X_j)
r_map  = L_map  * Log(inverse(S_i) * X_i)
cost   = 0.5 * sum(r.T * r)
C_i    = X_i * inverse(O_i)
```

Lie-group 相对因子残差及局部权重参照 [GTSAM BetweenFactor 官方说明](https://borglab.github.io/gtsam/betweenfactor/)。本实现右扰动、translation-first `[rho_xyz,phi_xyz]`、m / rad；旋转向量不是 Euler。GTSAM Pose3 rotation-first 和 ROS fixed-axis covariance 不能盲拷贝。`L.T*L` 为信息，允许 map L 秩亏，不强填弱维度。

首版 scan-to-map 压缩为 pose + 指定切空间信息的 unary。PCD 对照用既有固定 V GICP probe 估计 correction，再用当时 O 得到 S。没有逐点 residual 因子，也不冒充东北大学真实 Hessian。未来接 registration 信息需明确 residual / noise / 线性化点 / 转动中心与 frame，用对应 Jacobian 变换；近似质量 H 满秩并不能证明可直接使用。

第一份 full-pose 地图测量建立全局 gauge；初始化不读 scorer GT。无 map 因子时固定首节点只为消除数值 gauge，输出 relative-only，不输出 global C；相对 odom 不能确定全局原点。[GTSAM 官方教程](https://gtsam.org/tutorials/intro.html)。当前最小格式要求有地图因子时至少一个明确 full-pose anchor，不处理一般部分 anchor 的可观测性设计。

后端改 X、不重写 O；历史 C 随平滑更新可变，不能拿当前 C 给所有历史时刻重新贴 pose。causal 对照在每次地图观测到达时只重解已到达 prefix，用末节点 C 向后乘 odom，直到下一次观测；全 batch 使用未来观测，单列。正式 TF 仍只能由既有桥 owner 在未来 Integration 消费验证后的 source-time pose。本轮不写 global_pose、initialpose、map→odom 或 cmd_vel。

完整 batch 会随轨迹增长，没有 iSAM2 / fixed-lag / marginalization / 实时线程方案。[SciPy 1.11.4 官方文档](https://docs.scipy.org/doc/scipy-1.11.4/reference/generated/scipy.optimize.least_squares.html) 支持当前 API，不能由小图延迟推导比赛频率。

### 相关性及预留接口

FAST-LIO odom、deskew 后点云与 scan-to-map 可能共享误差；两个 topic 不等于独立观测。简单叠加 information 是本轮明确的模型假设，不宣称校准 posterior。增量传播还需跨时刻相关项，不能从两个绝对 pose covariance 完整得到。

重复 query ID 报错，防止同帧被反复作为独立信息；这不消除不同帧共模误差。输出 posterior covariance 为 null。下一次先建立有依据的误差尺度 / 方向与相关性，不能调 sigma 赢 toy。

私有格式预留空 `uwb_position` / `loop_between` / `global_candidates` 槽；本轮非空报错。UWB 将需 lever arm、clock、frame / covariance 及 map-UWB alignment；loop 需双 node ID / relative pose / information；SC 只输出候选集合，未来 Task 2 决定能否形成地图因子。没有实现在线融合、检索、异常门或 reset 状态机。

## 4. 固定结果

证据：[summary.json](../../experiments/neu_factor_graph/evidence/summary.json)、[代表输入](../../experiments/neu_factor_graph/evidence/drift_noise_input.json)。模型 / seed / 权重固定在实验 README。全部合成，一个噪声 realization，不是新车 field benchmark。

| 场景 | 单次 correction 位置 RMSE | causal prefix graph | 全 batch（用未来观测） | 判断 |
| --- | ---: | ---: | ---: | --- |
| 无噪声 full SE(3)，31 poses / 11 factors | <1e-12 m | <1e-12 m | <1e-12 m | 数学正控制成立 |
| 正确 basin + 设计漂移 / 噪声，31 / 11 | .13672 m | .10631 m | .09085 m | 此模型有收益，不能外推 |
| map 因子整体左移 12 m，31 / 11 | 12.01177 m | 12.03931 m | 12.00398 m | 低代价不能识别持续错误地点 |
| 首次 anchor + longitudinal 零信息，31 / 11 | .29057 m | .27533 m | .27544 m | yz 改善；纵向末端 .53198→.53334 m，未补足约束 |
| 既有 GICP 合成连通，16 / 6 | .04172 m | .05467 m | .04064 m | causal 精度变差，不能由全 batch 小改善判 Go |

设计噪声正控制：姿态 RMSE `2.83536→1.50327 deg`；接受时刻之间 C translation step RMS `.24686→.08360 m`、rotation step RMS `4.40845→1.53137 deg`。均为 source-time sample 指标，不含硬件 / 队列延迟。

PCD 对照6次 V 基础门与 probe pose gate 均通过，fitness `.01338–.01372`。probe gate 作用于 correction 的私有探针位姿，不等于正式 map/base gate 验证。map/query 为同一解析房间的不同点采样，point noise `.015 m`，没有真实遮挡 / 时钟 / 标定误差。coarse prior 明确来自合成已知起点，不是无 prior 全局定位。graph 地图 nominal sigma 固定 `.08 m / .03 rad`，不是由 fitness / H 标定。

GICP 合成 pose 很准；graph 末端位置误差 `.00085→.06031 m`、姿态 RMSE `.30770→.45152 deg`，但 C translation step RMS `.08147→.04791 m`。结果与偏置 odom 被 nominal 权重过多保留一致，但本轮没有标定实际 noise 或相关性。**更平滑不抵消精度变差；保留负面结果，不追加权重扫描。**

### 持续 12 m 错误为何不可辨识

所有 S 左乘 `G=translation(12,0,0)`，令 `X'_i=G*X_i`：

```text
inverse(X'_i)*X'_j = inverse(X_i)*X_j
inverse(G*S_i)*(G*X_i) = inverse(S_i)*X_i
```

两种残差不变。normal / alias batch cost 都为 `22.4059292148`，差 `2.63e-13`；对应左移位姿差 <`3.1e-8`。这是人工构造的持续地图因子错位，不声称新发现真实连续 12 m GICP 轨迹。L1-M / L2 已证单帧重复 basin 配准可良好；本实验说明持续错误解释不能单靠 odom + map 残差消除。经过独特几何后可能消歧，本轮未验证。

Exp / Log 最大差 `2.16e-16`、inverse identity `4.45e-16`、C 重建 X <`7e-15`。无地图不输出 global；ambiguous、同 query 重复、跨 epoch、时间错配报错，只是输入格式边界，不是 Task 2 现实异常识别。

全部 prefix / batch 按当前 termination criteria 收敛。31 poses 正常模型 prefix 中位约56 ms、16 poses PCD 约45 ms；单次、小图、只含图求解，不含 GICP / 文件 / ROS 通信，不是实时保证。没有全仓生产回归。

![合成固定对照](../../experiments/neu_factor_graph/evidence/comparison.png)

## 5. 决策与下一次入口

**Modify，停在 Task 1。** 数学与软件闭环成立，尚未证明新车从 correction 切换到图后端会更可靠：固定 GICP 对照无 causal 精度收益，真实输入又缺。不以“软件 Go”绕过串行 gate。

下一次仍只推进 Task 1：

1. 需要一段完整 session 的 canonical odom、原始 / registered cloud、heading / gimbal / home、参数快照；真实 immutable map 与独立 query。A 主研究、B 保留；不重复询问已回答“没有”的数据。
2. 先确认数值基准、clock / epoch、坡道下 full SE(3) 语义；建立 odom increment / map 信息误差模型。upstream 发布顺序、adapter covariance 在需要迁移时独立修复，本轮不暗改。
3. 固定数据和有依据的 noise 再比较 causal baseline / graph，检查位置、姿态、延迟、C 跳变、缺地图段与已知错误盆地。确认综合收益，不能只看 smoothness。

没有新输入则保存入口 / 证据，不无限合成调参；Task 2–6 未开始，L2 冻结为未来 initialization / LOST 候选入口。main、正式 profile、前端与 owner 契约不变，新资产只留本地研究分支。

## 6. Task 1 续审：信息来源与坐标传播

本轮仍为 Task 1 / Research，未重复检索已经确认缺失的实车数据。核心问题：能否从当前输出直接得到正确 frame / tangent / timestamp 的里程计信息？最小复核是固定源码、坐标传播有限差分、相同 marginals 的相关性反例，以及逐边信息的闭式控制；不更改 localization 轨迹权重或新增效果调参。

### 固定源码复核

[noise_audit.json](../../experiments/neu_factor_graph/evidence/noise_audit.json) 来自 [audit_noise_model.py](../../experiments/neu_factor_graph/audit_noise_model.py)。程序编译固定 submodule 的真实 `use-ikfom.hpp`，抽取 `publish_odometry` 的原 covariance / publish 片段，配 copied-message stub。**不是 ROS runtime / field replay**；用于独立确认状态布局与片段行为，不修改 upstream。

- native `pos.IDX=0`、`rot.IDX=3`：`use-ikfom.hpp:12` 为 pos 在前、rot 在后；pos 由 world velocity 积分，`vect` 为加法扰动；`SOn.hpp:233` 的 rot boxplus 为右乘。
- native P 的前6维是 **`[dp_world,dphi_sensor]`** 的混合坐标。publication 片段却对行列同时使用 `[3,4,5,0,1,2]`，把位置 / 旋转交换；这不是把 position-first 转成 ROS xyz/rpy 的有效转换。
- 指定测试 P diagonal `[.09,.01,.04,.0001,.0004,.0009]` 后，stub 首次发布全零；P 放大10倍后，第二次仍发布前一次的交换结果 `[.0001,.0004,.0009,.09,.01,.04]`。数值只是已知代数输入，不是实测方差。
- zero-start 把 pose 变成 `T_ZS=T_ZW*T_WS`，当前 P 填充路径没有同步父坐标处理。即使先修好顺序 / 延迟，position covariance 也需旋转：示例父基准旋转90°，位置 variance `[.09,.01,.04]→[.01,.09,.04]`。orientation 还需明确 tangent，不能只旋转 xyz 或交换数组。[ROS PoseWithCovariance 的字段约定](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/geometry_msgs/msg/PoseWithCovariance.msg)。

因此不提供“挪一下 publish 就能用”的不完整修复，也不从旧36字段自动生成可信 graph 权重。本轮给出原写的 private 数学转换，源必须是**同一时刻、明确 chart 的 native P**；修改正式消息 / adapter 要在输入及契约确定后另作 Integration。

### 可计算的部分与不可猜测的部分

本图采用 translation-first 右 SE(3) 误差。设 `R_WS` 为 sensor 到 world 的旋转、`E=T_base_sensor`、`Z=inverse(O_i)*O_j`：

```text
J_native = diag(transpose(R_WS), I)
Sigma_sensor_right = J_native * P_native_mixed * transpose(J_native)
Sigma_base_right = Ad(E) * Sigma_sensor_right * transpose(Ad(E))

A = Ad(inverse(Z))
delta_increment ≈ -A * delta_i + delta_j
Sigma_increment = A*Sigma_i*transpose(A) + Sigma_j
                  - A*Sigma_ij - transpose(Sigma_ij)*transpose(A)
```

第二行向 base 的传播仅在外参 / 当时 gimbal **已知且条件固定**时成立；真实标定 / 角度不确定性及其相关项不默认为零。跨时刻 `Sigma_ij` 在两侧各自的右误差坐标；若从 native / sensor joint covariance 转换，需要对两侧分别用相应 Jacobian / `Ad(E_i/E_j)`，不能对 cross block 只做单边变换。Adjoint / correlated relative covariance 的依据见 [GTSAM 官方推导](https://gtsam.org/2021/02/23/uncertainties-part3.html)。本代码独立实现，未复制第三方算法。

传播有限差分最大差：native covariance `5.63e-11`、sensor→base Jacobian `1.28e-9`、relative Jacobian `5.56e-10`、joint covariance `2.13e-12`；白化 identity `3.34e-16`。已知父基准变化后，正确 right-body covariance 不变（`1.39e-17`），与 legacy 未处理父坐标问题须区分：**右误差 frame invariant 不意味着旧 mixed 数组无需转换**。

相关性反例取 Z=I，两帧 marginal 均 `.01*I`：独立时 increment variance `.02*I`；合法 cross block `.0099*I` 时 `.0002*I`，相差100倍。只知道两个 P 无法辨识哪一种正确。kernel 对未知 cross-time covariance 报错；可显式声明独立近似，但不会把这种声明当作新车事实。缺外参不确定性、zero covariance、singular projected covariance 也不自动加 floor 后求逆。

即使单段 joint covariance 已知，各段 increment 之间、odom 与 map 之间还可能相关。逐段 information 不能编码这些 cross-factor 项；本图仍采用独立因子近似，输出 `cross_factor_covariance=null` 表示未建模，不能解释为零相关或校准后验。

### 当前 heading 模式是否提供完整 SE(3)

按照固定 `canonical_odometry.cpp` 公式做独立代数 mirror：传感器6维右误差 + chassis heading 1维输入，其输出 `[rho_xyz,phi_xyz]` Jacobian 秩为 **4**，roll / pitch 两行严格为0。这是软件模型投影，**不是物理 RP 被可靠观测为零，也不是实测四维 covariance**。不能把继承的非零 RP covariance、或投影后的零 covariance，直接解释为 full SE(3) odom factor information。

工程判断：FAST-LIO 的 native sensor pose 含完整旋转，保留前端仍合理；但当前新车 heading profile 的 canonical pose **不足以直接提供坡道场景的可信六维增量模型**。后续 Task 1 必须在两种有实测依据的表示中明确选择：通过时间匹配动态外参保留 full base SE(3)，或研究 sensor 状态与 base/heading 的一致转换。不能在本轮猜测安装旋转、绕开公共 owner，或为了接口形式把4维状态伪造为6维。平地限定模型可另作有明确适用范围的 Research，不能替代完整状态证据。

### 私有后端变化与结论

后端新增逐段 `odom_factors`：每段绑定 node pair、两侧源时刻、context / epoch / basis 与 information provenance。Z 仍由 canonical O 构造；不改前端、不重复输入另一个冲突 Z、不为不同采样间隔猜权重。旧通用 nominal 格式保留，等权逐段输入与旧格式31节点结果差为0，原 GICP 失败结果没有用新权重覆盖。

三节点线性特例中，两段 variance `.01/.09`、端点 variance `1e-6`，`.2 m` 总差按闭式解分配为 `.0199996/.1799964 m`；实现与闭式 pose 差 `6.96e-10 m`。只证明每段信息确实被使用，不是定位精度收益或校准方案。

**总体仍 Modify。** 已闭合坐标代数和输入表达，未闭合真实 native current-state / joint noise、标定 / gimbal uncertainty、full base state、registration 信息尺度及 odom-map 相关性。下一项仍是 Task 1 的真实输入 / 状态选择；不进入 Task 2，不以重复 toy 权重实验代替这些信息。

## 7. Task 1 续审：保留底盘完整姿态

本轮限定为输入状态的最小离线研究，不改正式 adapter / profile，不做定位权重扫描。运行前固定假设：已知 home 外参、单一 base-z yaw 轴 / 旋转中心，以及 heading 世界中启动底盘的**完整旋转**时，raw full sensor pose + 同时刻 chassis heading 是否足够恢复 full base SE(3)？CAD 平移和起始 heading 本身不满足这些条件。

Baseline 为实际固定 V 的 `compute_base_transform_from_chassis_heading`；私有候选使用解析旋转分解，并以独立生成的完整刚体链真值评分。最小对照预定为：平地含 wrap / 云台旋转的 parity；底盘 RP 与云台同时运动；倾斜起步的已知 / 错误启动基准；关节分解退化反例；局部 Jacobian 的秩及 heading 近奇异敏感性。没有真实数据效果声明。

判定：若已知模型也不能恢复位姿，停止候选；若数学成立但依赖未确认的标定 / heading 定义 / 初始重力关系，则保留 private 实现、总体 Modify，不能进入 Task 2。

### 完整姿态重建与前提

[base_state.py](../../experiments/neu_factor_graph/base_state.py) 原写解析候选；仍只在 `COLCON_IGNORE` 下，没有 ROS 发布者或对图求解权重的修改。定义：

- `E=T_B0_S(home)` 为真实 home 的完整外参；`c` 为 base 中 yaw 轴上的一点，轴向为 base +z，关节仅绕这根轴旋转。fixture 沿用仓库原 heading 测试的外参 / center，**不是 A/B 标定**。
- `A=E*inverse(T_raw_start)*T_raw_current=T_B0_S(t)`。这一步明确更换数值基准；不能只改 frame 名字。
- `H` 为 chassis heading 的固定世界 / 重力参考；`R_H_B0` 是其中启动底盘的**完整旋转**，包含 RP。heading 必须等于 `atan2(R_H_B[1,0],R_H_B[0,0])`，base 前向水平投影非零。只确认 CCW / rad 尚不足以确认倾斜时这个语义。

```text
E(gamma) = T_c * Rz(gamma) * inverse(T_c) * E
M = R_H_B0 * R(A) * transpose(R(E)) = R_H_B * Rz(gamma)
d = [cos(h), sin(h), 0], n = [-sin(h), cos(h), 0]
a = n dot M[:,0], b = n dot M[:,1]
a*cos(gamma) - b*sin(gamma) = 0
gamma = atan2(a,b) 或 gamma + pi
```

两根来自共线约束；要求 `d dot (M*Rz(-gamma))[:,0] > 0`，排除朝向反向的根。得到关节角后 `B=A*inverse(E(gamma))`，保留完整旋转和动态 lever arm。`gamma` 是相对 home 的角，不能凭空确定硬件 absolute home / 零偏。

`hypot(a,b)` 为0时 heading 不提供独立关节约束，返回 `ambiguous_joint_decomposition`，pose / gamma 为 null，不默认选0。接近0仍需考虑敏感性；当前 `1e-12` 只作浮点代数边界，**不是研究得到的实车 health / acceptance 阈值**。一般点的 `solved_conditioned` 仅表示指定条件下数学可解，`active_accept=false`、covariance=null。

这里用矩阵分解，不以 Euler RP 参数作为优化状态。fixture 的 `xyz` 旋转约定依据 [SciPy 1.11.4 官方 API](https://docs.scipy.org/doc/scipy-1.11.4/reference/generated/scipy.spatial.transform.Rotation.from_euler.html)。下述侧翻关节分解退化属于本模型的可观测性问题，须与常规 Euler 表示的 pitch gimbal lock 区分。

### 最小对照结果

[audit_base_state.py](../../experiments/neu_factor_graph/audit_base_state.py) 编译**不改动的 V canonical C++ 源文件**，通过文件探针调用实际转换函数；不是只以新 Python mirror 当 baseline，也没有启动 ROS / 驱动。C++ 与先前平面 mirror 最大矩阵差 `6.67e-16`。精选证据见 [base_state_audit.json](../../experiments/neu_factor_graph/evidence/base_state_audit.json)。

| 预定情形 | 实际平面 adapter 位置 / 姿态误差 | 私有 full-base 候选 | 结论 |
| --- | --- | --- | --- |
| 平地、云台 -1.1 rad、heading 跨 wrap | <1e-12 m / <1e-10° | 同样数值精确 | 平地 parity，既有适用模型有效 |
| 平地、heading 跨 ±pi、云台 2.8 rad | <1e-12 m / <1e-10° | 同样数值精确 | 不因角度 wrap 选错根 |
| 从水平起步，当前 RP .3 / -.25 rad、云台 -1.1 rad | .14601 m / 22.34022° | <1e-12 m / <1e-10° | 平面投影丢失姿态，也偏置位置 |
| 倾斜起步、home 静止 | <1e-12 m / <1e-10° | 同样数值精确 | 起步恒等检查不足以证明基准正确 |
| 倾斜起步后运动、已知完整 R_H_B0 | .11532 m / 19.38808° | <1e-12 m / <1e-10° | 完整启动参考可恢复 B0 中 full pose |
| 当前 roll=90°、pitch=0 | .43302 m / 90° | ambiguous，pose=null | 不可辨识的分解必须保留 |

独立已知关节角的几何控制 `B=A*inverse(E_measured)` 在全部6例也恢复真值；只是条件模型 baseline，**新车尚无已确认的独立机械角测量**，不能宣称这条输入已经可用。fixture 由 runner 构造 / 评分，候选只读 raw pose、heading 和条件外参，不读底盘真值。上述误差不是新车坡道性能。

### 反例与噪声语义

1. **倾斜起步不能简单减 yaw。** 在固定倾斜运动例中，错将 `R_H_B0` 写成纯 `Rz(h0)`，候选仍能算出 pose，但偏差 `.000686 m / 1.09035°`。home 静止时两种参考都给 I，因此起步恒等 / frame 名字相同无法证伪错误参考。`yaw(inverse(R_H_B0)*R_H_B)` 一般不等于 `h-h0`。缺完整参考时函数报错；不会暗加“水平起步”。真实重力关系可由可信完整姿态与标定建立，但本轮没有核实它已由 native 或 MCU 提供。
2. **侧翻有连续多解。** 同一 A、同一 directed heading 对应 gamma `.6/1.0 rad` 时，两种底盘姿态相差 `22.91831°`、位置相差 `.014326 m`；两条 sensor 刚体链重构矩阵差 <`4.5e-16`。这不是全局地点 alias，而是 base / joint 局部状态分解本身不唯一，不能靠更低 GICP fitness 消除。
3. **近奇异会放大输入误差。** 预定 pitch=0、roll=0° / 89° 两点，`hypot(a,b)` 分别为 `1` 和 `.0174524`，姿态对 heading 的局部响应分别为 `1` 和 `57.29869 rad/rad`，符合该 pitch=0 特例的解析 `1/cos(roll)`。没有做阈值搜索；不能由形式 rank=6 宣称所有姿态都稳健。
4. **完整状态不等于可信 covariance。** 正常倾斜例的 raw current 6维右误差 + heading 1维输入 → base 右误差 Jacobian 秩6，旧投影秩4。把“从 raw / heading 推导的关节角”当固定独立量，会得到不同 Jacobian，最大元素差 `.91098`。因此 §6 的 `Ad(E)` 公式只适用当时外参确实独立已知 / 条件固定的情况；推导关节角路径应传播完整 joint input covariance `J*Sigma(raw,heading)*J.T`，还需 startup / calibration 的不确定性与相关项。本轮不给实车 Sigma，不把 rank 当作六维信息来源。
5. **时间错配也可以数学可解。** 固定平地例，若 heading 在6 rad/s 下残留未补偿20 ms 时差，候选仍 solved，却误差 `.004324 m / 6.87549°`。正式 `lio_adapter.cpp::nearestHeading` 已按 `yaw_rate*dt` 把最近样本外推到 raw stamp，本反例不是声称当前函数原样使用旧 heading。实际 clock、heading rate 语义 / 误差、raw pose 与 deskew 时刻仍要独立核验；最大匹配时间窗口不是这些事实的证明。

### 工程选择与阶段决策

保留 FAST-LIO 完整 sensor state，继续以**full-base heading 分解作为条件明确的私有候选**，不把现有平面输出的 RP 零值直接用于六维图权重。若以后有独立机械 joint angle，可直接用时间匹配动态外参恢复 base，避免依赖本次 heading 分解；目前不能假定硬件已支持。

下一次 Task 1 的真正入口仍是实际输入，而非更多 toy 姿态 / 权重：完整 sensor / internal IMU 外参、home / axis、`H` 的 heading 定义与启动重力关系、clock / rate / deskew 基准、native 同时刻信息及跨时相关项，随后才是实际 bundle + 独立 query 的 causal graph / correction 比较。A/B CAD 平移继续保留，未写成假标定；没有替换 canonical TF owner 或正式转换。

**总体 Modify，停在 Task 1。** 本轮证明“保留前端而恢复完整 base 状态”在声明模型下数学可行，也找到了必须满足的物理条件；尚无新车数据闭合这些条件，更没有证据推翻 §4 的 GICP causal 负面结果。只保留原写候选、复现 runner 与一份小结果；生成的 C++ 探针 / 二进制在忽略的 build，不新增常设分支。Task 2–6 未开始，L2 继续冻结。

## 8. 实车到位检查表与恢复入口

**Modify / waiting for real vehicle evidence。** 本次仅核对现有接口、收口待补输入，没有重跑效果实验、调 noise / SC 参数或新增算法。恢复沿用本分支和报告中的固定源码；独立 covariance / full-base adapter 研究的后续结果须另行核对 commit / 输入合同，不能自动算成本分支能力，也不在此合并。

### 最小新车到位检查表

每项在到车时记录“证据路径 / revision / 确认者或方法”；现在均未取得实车闭合证据。复用 [L1-M §10.3](neu_localization_l1_baseline_20261007.md#103-新车输入基础已知位置与待闭合接口) 和 [L2 §1](neu_localization_l2_global_candidate_prototype_20261007.md#1-a-线真实输入闭合进度)，不重新猜 CAD。

| 到位条件 | 最少要留下的实测 / 导出内容 | 未满足时的边界 |
| --- | --- | --- |
| □ 设备及完整外参 | A 的 serial / IP / 实际 lidar、internal IMU topic 与 frame；driver left/right 角色；base 原点 / 前向；`T_base_gimbal(home)`、`T_gimbal_lidar`、`T_gimbal_imu`，方向、单位、标定来源及误差说明。B 整套参数保留 | CAD 机械原点不是 Livox / IMU 原点；占位 identity、YAML 地址不算标定；A↔B 不能只换 IP |
| □ home / axis / heading | home 的机械状态与时间；yaw 轴点、方向 / 正号、零偏；heading 的世界 H、CCW / rad、yaw-rate 语义，倾斜时是否为 base +x 的水平 azimuth；boot / reset / sequence | 只读 `/gimbal/state_derived` 不算独立机械角；本候选限单轴，不能把未知轴方向强写成 +z |
| □ startup full attitude / gravity | 融合开始的 raw IMU pose、同时刻 heading / home；可信 `R_H_B0` 及建立其重力 / yaw 关系的方法；FAST-LIO zero-start 变换与生效时刻 | h0 或启动单位 pose 不能证明完整参考正确；倾斜起步不默认 RP=0 |
| □ LiDAR / heading / MCU / ROS 时间 | LiDAR header / timebase / 点内时间单位及参考点、内部 IMU stamp；heading 的 MCU time、ROS header、接收时刻、boot/reset；clock 映射 / 误差、rate 外推及 deskew 参考时刻 | header 相同或匹配窗 .03 s 不证明同步；不能使用 receipt-now 冒充 source time，不能二次 deskew |
| □ raw FAST-LIO full state | 每帧 native `pos/rot`、发布 raw pose、源时刻 / identity；gravity、IMU↔lidar 外参与 zero-start 信息用于复核；保留完整旋转和实际 commit / launch / 参数快照 | 固定源码 `body` 是内部 IMU state 的私有别名，不是 lidar 或 base；只留投影后的 `/odometry/lio` 无法补回 RP |
| □ native current-state covariance / 信息来源 | 与 native state **同帧**的 `kf.get_P()` pose 6×6 block（可保留完整 P）、状态顺序 / chart：`[dp_world,dphi_imu(right)]`；提取位置 / 时刻和 native→raw 的确定坐标变换；增量 / 地图 information 来源 | 旧 ROS 36字段存在错序 / 延后一帧，不能直接用；native marginal 也不是 increment 或 GICP information；缺跨时相关项不得直接相加 |
| □ registered cloud / canonical odom 同数值基准 | cloud 的坐标 R、deskew / 累积窗、source stamp、raw / base pose及显式 `T_B0_R=E_home*inverse(T_R_I_start)`；以真实点 / 位姿复核换基，给 basis / epoch revision | `odom` 同名不够；`E_home` 必须是 base→**状态 IMU** 的 home 外参。Task 1 的 B0 可倾斜，L2 还需显式重力换基，不能直接声明 O 已重力对齐 |
| □ immutable map + 独立 query / 参考 | 固定 map ID / revision、PCD 和 occupancy bundle 的 manifest / origin / yaw / 单位 / 对齐证据；建图 episode 与留出 query 分离；query ID / 时间、独立位置 / 姿态参考及精度边界 | 不用 query 自己建图再证明定位；只有 merged PCD 不能重建 L2 keyframe 数据库；没有独立参考的指标记 UNKNOWN |

最小交付为：一份参数 / 标定 / 时间说明，一段建图数据与地图，一段或少量独立 query 数据，native state / current-P 的导出，以及单独的参考 / 场景标签。普通 bag 可承载已有 topic；P 若未导出，需要注明可信的 native 提取方法，**不能认为 bag 内已有正确 covariance**。本轮不建设 recorder。增量权重可来自已知 joint covariance，或独立实测标定集中的误差模型；先确定来源再锁定，不用评测 query 拟合权重。不同 odom / map 因子的共享误差仍需说明，不能伪称校准 posterior。

### 最小数据接口：沿用现有文件 / 函数

统一 `T_A_B` 将 B 点变到 A，米 / 弧度；保留 source 与接收时间，所有关联先在同一 clock / session / epoch 中完成。下面是离线交接字段，不新增公共 ROS 消息。

| 数据 | 现有消费入口 / 最小字段 |
| --- | --- |
| 真实 base / map 测量 JSON | [backend.py](../../experiments/neu_factor_graph/backend.py)：`context={session_id,odom_epoch,numerical_basis_revision,map_revision,calibration_revision,input_kind:verified_recording,frames:{map:map,odom:odom,base:base_link}}`；`samples=[{id,timestamp_ns,T_odom_base}]`；`scan_factors=[{id,query_id,node_id,timestamp_ns,context,basin_status,T_map_base,sqrt_information,noise_model_origin}]` |
| 逐段 odom information | `odom_factors=[{id,from_id,to_id,from_timestamp_ns,to_timestamp_ns,context,sqrt_information,noise_model_origin}]`，一段一份；或有依据的 common information，二选一。右切空间 `[rho_xyz,phi_xyz]`、`L.T*L=information`；当前 odom 要求满秩，map 可秩亏但至少一个 full-pose anchor |
| source / arrival 关联记录 | 同一 node / query ID 下记录 `source_timestamp_ns`、odom/cloud/heading 的接收时刻、scan 测量可用时刻及耗时；作为 sidecar，backend 不读取这些额外时间。精确关联 factor 的 timestamp 与 sample；不改旧 stamp 为 now，不混合 reset / zero-start 前后 epoch |
| full-base 数学输入 | [base_state.py](../../experiments/neu_factor_graph/base_state.py)：`from_heading(raw_start,raw_current,E_home,center,heading,R_heading_base0=...)`；这里 raw child 为选定 IMU，非投影 base。独立已知机械角才用 `from_measured_joint`；输入需先同步 / 标定，输出非 solved 不喂满秩 graph |
| covariance 数学输入 | [noise_model.py](../../experiments/neu_factor_graph/noise_model.py)：`native_mixed_to_right(T_world_imu,P_pose)`；独立已知 / 条件固定动态外参才用 `sensor_to_base_right`。推导关节角路径须传播 raw+heading 的 joint covariance 与 startup / calibration 项；`relative_right_covariance` 需要对应 cross-time block 或明示独立近似，未知不置零 |
| 留出评分数据 | 单独按 node ID / source stamp 对齐 `T_map_base_reference`、参考误差边界、建图/query episode、工况及 wrong-basin / missing-map 区间。部分参考只评分可观测维度；GT 不进求解、seed 或接收门 |

`basin_status=single_basin_assumed` 是每次指定盆地的研究假设，错误盆地压力输入也保留外部标签；不能把它写成“已证明唯一”。ambiguous 集合不能强取第一名压成因子；必要时按独立假设分别回放并保留结果。已有 `active_accept=false` 不能拿来制造“false ACCEPT 为零”的性能结论。

### 已有实验入口核对

| 资产 | 真实输入能否复用 | 本轮发现的必要限制 |
| --- | --- | --- |
| factor graph / correction A/B | `backend.solve(data)`、[run_experiment.py](../../experiments/neu_factor_graph/run_experiment.py) 的 `baseline(data)` / `causal_graph(data)` / `errors(...)` 已分离生成器与估计 / 评分；模型无需重设 | `run_experiment.py` CLI 是合成生成器，`compare` 还会算 full batch。现有 A/B helpers 要求 node ID 为连续 `0..N-1`、首节点共同 anchor、每个 node 至多一个共同 scan factor；`causal_graph` 是 **source-order prefix**，不读取 arrival sidecar。`errors` 已有 RMSE / 末端 / jump RMS，p95、arrival 延迟及工况分组留给真实数据的轻量评分 |
| covariance propagation | `noise_model.py` 接收任意真实矩阵 / 变换，可直接消费确认 chart 的测量 | `audit_noise_model.py` 是固定源码 / 数学控制，不是 bag replay。joint / cross-time 缺失需补证据，不能把审计中名义方差移给实车 |
| full-base reconstruction | `base_state.py` 接收任意确认的 raw pose / home / heading / startup；不依赖 fixture 真值 | `audit_base_state.py` CLI 固定6个控制；实车逐帧调用 kernel，未实现 bag 读取 / 标定 / 同步；近奇异及输入不确定性仍需实测 |
| L1 GICP | 私有 PCL `registration_probe` 已支持真实 target/source PCD 与 full SE(3) seed 文件 | source 先变到与 O 相同数值基准；probe 输出 correction C 后用 `S=C*O`。probe 的单位 odom/base 位姿门诊断不等于正式 map/base 门；地图 factor 集合须在 A/B 前共同确定 |
| L2 global candidate（继续冻结） | [prototype.py](../../experiments/neu_localization_l2/prototype.py) 已有真实 `build-db --manifest` / `query --database --query` 文件接口及三种 prior；无需重设描述子链 | 当前只读 ASCII XYZ PCD，二进制 PCD / bag 需先导出；synthetic runner 不作实车入口；只有 Task 1 Go 并进入 Task 2 后才恢复检索，不因数据到位自动解冻 |

L2 的 manifest 保留 `database_revision/map_revision/basis_revision/data_kind/map_bundle/keyframes`；条目需 local cloud、`keyframe_id/place_id/source_frame_id`、源 / 接收 / transform 时刻、`T_map_keyframe`、`T_odom_input/T_odom_base`。query 保留 `query_id` 和同类 metadata；`normalization_status=verified_recording` 需 `calibration_evidence` 与 point_time / deskew / home / clock_alignment / gravity_alignment 的 verified 声明。声明不是证据；建库-query 身份 / cloud 禁止泄漏。Q 是重力对齐 base 原点，`T_map_base=T_map_query*T_query_base`，不能把 Q 姿态直接当 base。

**核对结论：核心函数 / 私有 schema 可以替换真实输入复用；当前并非一条命令直接读取未处理 bag。** 到位后只需针对实际字段做轻量导出、按真实 arrival 构造已到达 prefix 并调用现有 `solve`、独立评分；不重新设计图 / 几何 / retrieval 算法。本轮不提前编写通用 converter、arrival replay 平台或状态机。

### 固定实车 A/B 与恢复顺序

1. 按上表闭合 input，固定实车 commit / profile、map、home / basis / epoch；用独立标定数据确定 noise model。保留本报告源码 pin；若需要变更输入修复版本，明确记录新旧差异，不能静默换基线。
2. 固定共同的 `O_i`、已有 GICP 的 query / seed / S / 门结果 / 到达时刻及信息来源；不让两臂产生不同测量，不用 GT 过滤坏因子。按实际 map/base 语义核对既有公共门；probe 的 correction / 单位变换门不能代替它，也不为坡道放宽既有门追求收益。初始化相同、首个可信 full-pose anchor 相同；helper 的 node 0 从该共同 anchor 开始，之前保留为未初始化时段。先分开评估当前 planar 输入及已验证 full-base 输入；**主 A/B 在同一输入表示内比较**，不能把 adapter 变化算作 graph 收益。
3. A 为 **current correction baseline**：接受同一 `S_i` 后 `C=S_i*inverse(O_i)`；B 为 **causal factor-graph backend**：仅优化当时已到达 odom / scan 因子的 prefix，末节点 `C=X_last*inverse(O_last)`，两臂此后均用 `C*O` 推进。相同测量可用事件的更新条件保持一致；历史 X 不回写已输出的实时评分序列。full batch 仅参考，不能用未来观测替代 B。
4. source-order helpers 可先用于结构 / 源时刻估计对照；**正式实车 causal 比较须按 sidecar 的真实 arrival 控制前缀**，晚到测量不能在其 source stamp 就视为可用。首次无 global anchor 只记未初始化 / relative-only，不补 identity。缺地图段两臂只按已有 C + odom 传播；epoch 分段，本轮不建 reset / recovery 状态机。
5. 预先选择下表工况、对齐独立参考，锁定评测分段 / 目标车延迟预算后重跑。保存一份输入 manifest、A/B 结果及代表失败即可，不扩展大型过程档案。

| 必评内容 | 固定比较量 / 工况 |
| --- | --- |
| position / attitude | 相同有效时刻的位置 RMSE / p95 / 末端 / 最大误差及各轴；`Log(inverse(T_ref)*X)` 的 SO(3) 角误差（deg），不能只报 yaw。无完整参考的量写 UNKNOWN，不用 LIO 自评分 |
| correction jump | 接受事件前后 `inverse(C_before)*C_after` 的平移 / SO(3) jump：RMS / p95 / 最大值；同事件、同输出时刻比较；更平滑不能抵消精度恶化 |
| latency | 相同硬件上的 GICP 公共耗时、A/B 处理 / graph solve 的 p50/p95/max、prefix node / factor 数；记录 receipt-source age 与结果可用 age。重放得到的可用延迟估计单列，不拿旧 bag stamp 减当前 wall clock 冒充在线端到端实测 |
| missing-map | 固定自然缺地图段；必要时对真实记录共同屏蔽一段观测并标为受控回放。报告缺失时长、漂移 / 姿态误差增长与恢复观测后的误差 / jump；不生成假地图因子 |
| wrong-basin | 对真实 query 保留正确与重复地点 / wrong seed 的解释、地点错误幅度 / 持续时间、GICP quality 与 graph cost；未遇到真实错误盆地就记未覆盖，不以人工平移12 m 代替现场证据。低残差不证明 global uniqueness，求解成功不算可信 ACCEPT |
| 坡道 / gimbal / high-yaw-rate | 分别标记坡道 RP、底盘 heading-rate、云台 joint-rate、静底盘转云台与组合运动；单列误差、丢帧 / 输入不可解与 latency。不得混用传感器转速与底盘转速，缺少工况就记未测 |

真实数据目录 / 文件名可不同。沿用 schema 后，文件求解入口为 `python3 experiments/neu_factor_graph/backend.py build/neu_factor_graph/real_vehicle/input.json build/neu_factor_graph/real_vehicle/batch_reference.json`；此 CLI 是 **full batch 参考**。主 A/B 复用上面的 `baseline` / `causal_graph`，真实 arrival 的调度按步骤3–4调用 `solve`，不运行合成生成器。恢复先读本节，准备共同输入与独立评分再跑；正确 basin 的收益、错误解释和实际延迟仍不足时继续 Modify / Stop，不调 toy 追 Go。

### 并行任务隔离记录

本轮出发点 `0a96ea4a`，工作区 clean；仅在 `/home/qihei/rm2027_navigation/build/neu_localization_baseline`、`experiment/neu-localization-baseline` 更新文档。另两个任务分别在 `build/full_base_heading_adapter` / `experiment/full-base-heading-adapter`、`build/fastlio_covariance_contract` / `experiment/fastlio-covariance-contract`，已通过 worktree 清单及任务实际工作路径核对；本任务不 checkout/reset/clean 它们、不修改其分支。

本 worktree FAST-LIO / Livox 均未初始化（无本地 submodule branch），gitlink 分别为 `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321` / `2a2029a6e62a2196b280be6ec00bb2418065b8e0`。原只读 FAST-LIO 参考 checkout 核对时是 e7864a63、detached HEAD、clean；本轮未修改 / 构建该共享 checkout。后续源码审计重跑应显式提供本任务独立且匹配 pin 的 `--fast-lio-source`，不借用其他任务的 dirty submodule，也不盲用旧 runner 的默认共享路径。本轮不初始化子模块、不启动 ROS，未占用 Domain；以后 build / 临时 / 数据导出留在本 worktree 自己的 build 下，ROS 测试另选未占用 Domain。

**等待条件已经收口：拿到有依据的真实输入后从本节恢复 Task 1；不进入 Task 2，不推送，不合并其他研究分支。**
