# 新车 full-base heading / gimbal pose adapter 独立研究

日期：2026-10-07（Asia/Shanghai）。结论：**数学与接口上存在有条件成立的 full-base adapter；
新车硬件尚未 Go。** 现有 planar adapter 留作 baseline。本轮没有新车数据，没有改变正式
`odom→base_link` owner、YAML、launch 或 factor graph，没有进入 Integration。

## 1. 分支、隔离与证据来源

- 实验分支：`experiment/full-base-heading-adapter`，直接从本地
  `origin/main-new-car@bd6cf689d3d5aca5c8edd430ff760ca5ceb27496` 建立；未联网更新 remote ref。
  该基线已经包含 heading fusion 数学函数、传输合同和既有测试，适合本轮审计。
- 独立 worktree：`/home/qihei/rm2027_navigation/build/full_base_heading_adapter`。
  创建前检查了所有 worktree；原目录有其他研究的 dirty 文件，因此没有在原目录修改。
  本任务所有代码、构建、探针和临时日志均在自己的 worktree 内；没有 ROS 进程或 Domain 占用。
- 本 worktree FAST-LIO submodule **未初始化**，pin
  `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321`；Livox 同样未初始化，pin
  `2a2029a6e62a2196b280be6ec00bb2418065b8e0`。
  源码审计只读取原仓库中 FAST-LIO 的固定 Git blob；检查时该 checkout 为 detached HEAD、
  同一 pin、clean。没有初始化、共享或修改 submodule 工作文件。
- 最小 cross-reference：`experiment/neu-localization-baseline@0a96ea4a5f4c32b8f99540634d2e02d87c28fe6e`
  中 `experiments/neu_factor_graph/base_state.py` 和 `evidence/base_state_audit.json`。
  只作为先前数学/反例参考；本候选重新实现通用单轴链、显式合同和状态判定，不 import
  graph 模块、不 cherry-pick、不 merge。分支父提交仍是 `main-new-car` 基线。

实现与复现入口：[实验 README](../../experiments/full_base_heading_adapter/README.md)、
[adapter.py](../../experiments/full_base_heading_adapter/adapter.py)、
[测试](../../experiments/full_base_heading_adapter/test_adapter.py)、
[固定证据](../../experiments/full_base_heading_adapter/evidence/summary.json)。
证据含 production 文件与固定 bundle 源码 SHA-256。

## 2. 实际 canonical / lio_adapter 合同

以下为固定基线的源码行为，不把 README 或 frame 名当作数值基准证明。

| 路径 | 实际输入 / 运算 | 实际输出与限制 |
|---|---|---|
| `compute_base_transform` | `T_O_S * inverse(E_t)` | full SE(3)，前提 raw 父已经是目标 O，且 E_t 是同时刻真实 `base→sensor` |
| `compute_base_transform_from_sensor_initial` | `E_t * T_S0_S * inverse(E_t)` | 单个 E 同时用于两端；固定安装成立，动态云台一般应左乘 E_0，右乘 inverse(E_t) |
| `compute_base_transform_from_chassis_heading` | `E_0 * inverse(raw0) * raw_t`；底盘取 `Rz(wrap(h_t-h_0))`；提取 joint yaw 再旋转 home lever arm | 输出 B0-based pose 与 derived gimbal yaw；显式 roll/pitch=0，joint residual 的 roll/pitch 被丢弃，无数学可辨识状态 |
| `lio_adapter/sensor_tf` | 验证 raw parent 字符串、有限 pose 和非零 quaternion（随后归一化）；按 raw 时间查 `base→input_child` TF；零 stamp / 显式 latest 时查最新 | 支持 full pose；动态 sensor-initial 的两端外参合同受上述 helper 限制；查不到 TF 可入队；placeholder fallback 按参数启用 |
| `lio_adapter/chassis_heading_fusion` | 需要 `initial_alignment_confirmed` 和 home/center；raw child 不得是 base；nearest heading 在 30 ms 内按 yaw rate 传播至 raw stamp；首个配对 raw 保存为 raw0 | 使用 planar helper；不读取当前 gimbal TF，不读取 full startup attitude 或 gravity；`raw_odom_parent_frame_mode` 在此路径不参加计算 |

审计位置：`src/rm_localization_adapters/src/canonical_odometry.cpp:63–126`、
`src/rm_localization_adapters/src/lio_adapter.cpp:287–354,505–735`。

heading 消息含 `yaw/yaw_rate`、MCU time、sequence、boot ID、reset counter、online/valid；
节点检查 sequence 和 reset identity，reset 后 latch 并要求重启/rehoming。当前 heading 路径
会接受非 base 的其他 child 名称，并依赖 home 参数与实际传感器匹配；它没有候选中的完整
frame-chain/revision 检查。这些既有 runtime 行为本轮保持不变。

canonical 输出是复制 raw `Odometry` 后改写父/子 frame 和 pose：`/odometry/lio`，默认发布
`odom→base_link`；heading 模式仅覆盖 pose covariance 的 yaw diagonal。finite-difference
在转换后的 base pose 上估算全部 6 个 twist 分量，要求非零时间，首样本、非法 dt 或 outlier
时跳过整条输出；twist covariance 使用配置的固定对角线。derived gimbal 在 odometry 成功
发布时输出，供 gimbal-state adapter / robot-state-publisher 使用。**现有 pose covariance 的复制
不是完整 sensor→base / heading 的不确定性传播；本候选不继承它。**

### FAST-LIO raw pose 到底是什么

固定 `laserMapping_bundle.cpp:693–701,728–760,1272–1307` 直接把
`state_point.rot/pos` 构成 IMU/body full SE(3)，根据 `zero_start_pose` 用保存的起始 sensor
transform 左乘归零，发布 `map_frame→body`，时间为 `publish_lidar_time`。
`body` 是滤波 IMU 状态的 origin，不自动等于 MID360 lidar 光学/点云原点。
adapter 的显式 alias 将 `body` 解释为 `lio_imu_link`，不能直接解释为 base。
FAST-LIO 内部 full state 仍保留；公开 Odometry pose 只是其中的 full sensor pose，
不携带完整的 gravity、bias、启动姿态或 raw epoch 元数据。

默认 single 配置 `mapping.zero_start_pose=true`、delay 40 scans。源码在达到 delay 后切换 raw
数值父基准，字符串 `odom` 不变，Odometry 没有 epoch 字段。候选要求上游明确稳定 epoch；
不能在第一次收到 raw 后忽略这个后续 rebase。硬件实验须等归零完成/基准稳定再建立 startup，
或使用明确固定的 backend parent；本轮未改后端和 YAML。

`lio_adapter_fast_lio_multi.yaml` 默认选 `sensor_tf` 且没有显式配置 `raw_odom_parent_frame_mode`，
节点默认 `canonical_odom`，而 single backend 启用 sensor zero-start。该组合不能仅凭父 frame
字符串宣称 raw 已是 base/目标 odom 数值基准；固定安装也需要明确的 parent alignment。
这是源码合同风险，不是本轮实车运行的失效结论。

FAST-LIO launch 隔离 backend `/tf` 和 `/tf_static`；`lio_adapter` 是 canonical owner。
这个 ownership 不改。没有运行 LIO、deskew、driver、串口或 Nav2。

## 3. Research-only full-base 接口与数值基准

`T_A_B` 把 B 坐标映射到 A。raw R 可以是任意固定父基准；S 必须是 FAST-LIO full pose
实际描述的 IMU origin。B 为物理 base；J 为固定在 base 上的关节轴 frame，J +z 是旋转轴；
G 为运动侧 frame，home 时与 J 的轴基准重合。

输入显式包含：

1. `raw_t` 和 startup `raw_0` 的完整 SE(3)、原始时间、公共 clock、raw epoch、raw parent 和 sensor frame。
2. synchronized heading：H 世界中 **base +x 水平投影的 directed azimuth**，单位 rad、右手 CCW；
   H +z 与重力相反。h0 与 h_t 的参考相同，H 的任意 yaw zero 可保留，不需要等于地图世界。
   若 MCU 实际输出是绕倾斜 chassis-local z 的积分角或另一安装轴 yaw，本公式不成立。
3. 完整 `F=T_B_J`（轴位置与方向）和 `K=T_G_S(home)`（完整 rotation 与 lever arm）、
   单 revolute 模型、home mechanical angle、frame 与 calibration revision。
   内部 lidar↔IMU 外参若 raw S 为 IMU，必须先正确转换至所选 S，不能只搬 lidar 安装坐标。
4. 同步且确认的 startup home；完整 `R_H_B0` 与归一化 gravity-down in B0、startup heading 和时间。
   二者是条件输入，可来自同一观测，不假定独立。允许用已校准且可靠的静态 gravity 加 h0
   显式构造 `R_H_B0`；不要求下位机天然提供 full chassis attitude。
5. 可选的同时刻独立机械关节角 q。仅作直接几何求解控制；不能用本 adapter derived angle
   冒充独立测量来排除多解。

标定缺失、非 SE(3)、frame/revision 不符、home 不确认、gravity 与 R0 不一致都拒绝。
输入标签只提供合同一致性，不能验证某组数值是否真的来自实车。

输出 O 的原点为 startup base 原点，轴与 H 的 gravity/heading reference 相同：

```text
D = T_O_B0 = [R_H_B0, 0]
E(q) = F * Rz(q - q_home) * K
E0 = F * K
A = E0 * inverse(raw0) * raw_t        # T_B0_S(t)
T_O_S(t) = D * A
T_O_B(t) = T_O_S(t) * inverse(E(q))
```

原始父数值基准被正确消除；动态 lever arm 与完整 rotation 都保留。
结果同时给 `T_odom_base` 与 `T_base0_base=inverse(D)*T_odom_base`。倾斜启动时 B0-based
输出 identity 不表示物理底盘水平；O-based 输出在启动时保留完整 R0。二者不能只改 frame
字符串混用。候选中的 `odom_frame` 是研究输出基准声明，没有公共 TF 发布权限。

gravity-only 构造：令 `u=-g_down_B0`，`x=(e_x-u*u_x)/norm(e_x-u*u_x)`，`y=u×x`，
`d=(cos h0,sin h0,0)`，`n=(-sin h0,cos h0,0)`，则
`R_H_B0=[d,n,e_z]*[x,y,u]^T`。base +x 垂直时 heading 未定义，拒绝。
动态加速度不是可直接替代的 gravity；若 zero-start 已丢掉重力参考，需独立校准传感器/启动
观测或可靠的 backend gravity/reference 导出。本轮不新增该导出接口。

## 4. 未知关节角的完整分解与状态

记 `Q=R_O_S*R_K^T`，`v=R_F^T*e_x`，delta 为 q−q_home。
底盘 forward vector 为：

```text
f(delta) = Q * Rz(-delta) * v
         = U*cos(delta) + V*sin(delta) + W
U = Q*[v_x,v_y,0]
V = Q*[v_y,-v_x,0]
W = Q*[0,0,v_z]
a = n·U; b = n·V; c = n·W
a*cos(delta) + b*sin(delta) + c = 0
```

解析所有根，再检查 directed `d·f>0` 及非垂直投影。轴为 base +z 时 c=0，通常两个根
相差 π，其中反向根被拒绝；轴有 tilt 时 c 可非零，**两个根可能同为正确的 heading 方向**。
无 joint encoder/另一独立观测时返回多解，不能靠 GT、默认 q=0、最近分支或启动 home 强行选。
home 仅约束 startup 时刻；同一 raw 时刻的 startup sample 可直接用已确认 home，不将它当作
以后所有时刻的关节角。关节输出为模 2π 角，不包含 turn count。

条件数诊断用 `s=abs(-a*sin(delta)+b*cos(delta))`、forward 水平投影 `l` 和
`|d(delta)/dh|=abs(d·f)/s`。这是局部 heading→joint / attitude 灵敏度，不是完整统计
covariance，也不量化所有 sensor/calibration/startup 噪声。

| 状态 | 含义 | 顶层 pose |
|---|---|---|
| `SOLVED` | 唯一可用根，输入合同及声明的局部 sensitivity gate 成立；或同步独立已知 joint 的直接解 | full SE(3) |
| `AMBIGUOUS` | continuum 或多个 directed-heading 根 | null |
| `ILL_CONDITIONED` | 唯一根接近 tangent/垂直 forward，超出演示 derivative/projection/gain gate | null |
| `INVALID_TIME` | zero/noninteger 时间、clock/epoch 不符、skew 过大、早于 startup 或同时间不同 raw pose | null |
| `INVALID_CALIBRATION` | 不完整/错误链、home、模型、revision 或 startup gravity/attitude 条件 | null |
| `INVALID_INPUT` | raw/heading 无效、frame 不符、几何无解或已知 joint 与 heading 不一致 | null |

非 `SOLVED` 的 diagnostic candidates 仅作证据，不能作为可发布 odometry。
状态是逐样本数学/输入条件，不表示恢复流程或系统健康。
默认演示 Policy：skew ≤1 ms、s ≥0.05、l ≥0.05、gain ≤20，数值 epsilon 1e−12；
**这些不是新车实测阈值**。已知 joint 不做 heading 分解 sensitivity gate，但仍检查 directed
heading/projection 与输入一致性。

候选不做同步插值、rate prediction、time relabel、连续分支选择、fallback/recovery、twist
或 covariance。所有结果 `active_accept=false`，`covariance=null`。heading 解出的关节与 raw/heading
相关，未来不确定性传播不能将它再当一份独立观测；startup 和标定的不确定性也不能默认零。

## 5. 验证结果与反例

编译实际未修改的 `canonical_odometry.cpp`，同时运行原 15 项 C++ canonical 测试；
候选 31 项测试通过，其中包含 100 个固定 seed 的完整 roll/pitch/yaw / startup / joint 控制。
没有从 graph 迁移测试执行依赖；无 ROS runtime。所有几何为已有 unit-test home/center
拆出的**合成 fixture**，不是新车 A/B 外参。

| 控制 | planar baseline | full candidate |
|---|---|---|
| 平地 +云台旋转 +heading wrap | 实际 C++ 与物理真值一致 | 与实际 C++ parity，full pose 恢复至数值精度 |
| 从水平启动后 roll=.3、pitch=−.25 | 0.146013 m / 22.340216° 误差 | `SOLVED`，位置约 1e−15 m、姿态约 1e−13° |
| tilted startup 后继续运动 | 约 0.115322 m / 19.388081° 误差 | `SOLVED`，完整 startup reference 下数值恢复 |
| 90° sideways roll | 丢失姿态；没有数学状态 | `AMBIGUOUS`，已知 joint 可直接恢复；不同解同时解释同一 sensor / heading |
| 89° roll | 无分解 gate | heading 1e−7 rad 扰动引起约 57.298689 倍姿态响应，默认 `ILL_CONDITIONED` |
| 60° tilted joint axis | planar 模型前提不成立 | 两个 directed roots，底盘相差 180° / 0.115699 m，sensor 重建差 <7e−16，`AMBIGUOUS` |
| 20 ms time mismatch | 既有 30 ms/rate propagation 合同另外保留 | 默认 `INVALID_TIME`；若错误地改成同时间，6 rad/s stale heading 可返回 `SOLVED` 却误差 7.036535° / 4.425 mm |

时差反例没有回放现有 planar rate propagation，不能解释成它在真实 20 ms 异步场景必失败。
它证明错误/隐瞒的时间戳能绕过所有形式检查，且姿态解显著偏离。

额外原生合同反例：底盘静止、joint 转 .6 rad 时，现有 `sensor_initial` helper 使用 E_t
作两端外参，虚构 base 运动 **2.131 cm / 34.377468°**；显式 E0 / E_t 两端链恢复静止。
正式 helper 本轮不修；报告明确这一移植前必须处理的合同限制。

同样，完整但错误的 level startup reference 若同时配一份内部一致的错误 gravity，形式检查
仍可 `SOLVED`。正确 home/初始 identity 不能验证真实启动重力参考。拒绝 missing startup attitude
与验证真实正确姿态是两件事，后者需要硬件证据。

## 6. 比 planar 多需要的真实信息

已有 planar 也需要真实 home/center、yaw 约定和时间。本候选进一步需要：

- **完整链和旋转轴方向**：明确 S 是 IMU origin；base→axis、home gimbal→S 的 rotation、lever arm、
  符号、home zero 和 revision 可测且刚性稳定。不是仅提供 XY offset 或只读 CAD 名称。
- **完整启动重力关系**：即便倾斜启动，也要获得 g_B0 或 R_H_B0；结合 h0 明确 O 轴与 gravity。
  raw zero-start identity 本身不能提供这项关系。
- **heading 的真实物理语义**：H 是固定 gravity-aligned 世界，测的是 base +x azimuth；
  只承诺 heading delta/sign 在平地正确不足以支持 tilted chassis。
- **统一时间与稳定 epoch**：raw/heading/home/startup reference（和可选 joint）在同一采样时间，
  有可验证 latency、clock offset、reset/rebase 信息。软件传同一 stamp 不能证明同步。
- 若实际轴 tilt / 活动范围产生多解，或需要覆盖近奇异姿态，需**独立关节角或另一独立 full-base
  attitude 观测**。已知 startup home 加未知后续旋转不足以保证每帧唯一；multi-axis 构型需要完整
  已知 joint 链或额外可观测约束，本候选明确拒绝 two-axis 模型。

## 7. 新车到位后的 Go 门槛与后续范围

先在本分支做硬件验证，保留 planar shadow 对照，研究结果单独记录，不争夺公共 TF owner：

1. **测量与校准 Go**：核对传感器 IMU/lidar origin、完整 F/K、轴方向、home 重复性与机械回差。
   每台车/每个传感器真实测量并记录 revision；当前不猜 A/B、不写假标定 YAML。
2. **参考 Go**：在水平和已知倾斜启动下，独立测量重力与完整 chassis attitude，验证 h0 / h_t
   在不同 roll/pitch 下确为 base +x azimuth。建立与 backend rebase 后的 raw epoch 的同步关系。
3. **时间 Go**：记录原始 MCU/driver/host 时间与 reset identity，量化 scan pose stamp、heading
   latency/jitter、关节角时间及 deskew；有依据地选择插值/预测方案和 skew gate，注入 delay/reset
   验证 INVALID_TIME 行为。不能复用 1 ms 或 30 ms 演示值当已标定边界。
4. **可辨识 Go**：静止、纯 chassis yaw、纯 gimbal yaw、平移、roll/pitch、斜坡、倾斜起步和组合
   运动，使用独立 mocap/测量治具/临时 encoder 等真值。覆盖活动范围，确认会否进入 continuum、
   双根或 near-singular 区域；必要时增加独立 joint 输入，再用实测误差确定 sensitivity gate。
5. **物理与后端 Go**：高角速和扫描内云台运动下确认真实 FAST-LIO/IMU/point deskew 没有破坏
   sensor pose；检查 lever arm、cloud alignment 和 base pose，而非只看 heading 数字。
6. **迁移候选 Go**：上述证据成立后再设计最小 ROS 输入桥/独立 shadow 输出，明确数值 parent、
   epoch、状态和后续 twist/covariance 合同，并保留现有 owner。决定最小实现移入 `main-new-car`
   是后续工作；本轮不进入 Integration、不接 graph、不改变 main/main-old-car、不 push。

本轮判断不把数学 `SOLVED` 解释成实车定位有效。完整状态本身仍依赖 LIO 估计质量；adapter
只能正确转换已经有物理意义的输入，不能校准未知外参、修复错误 heading 或消除时钟错误。
