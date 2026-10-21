# Full-base adapter 最后一轮输入合同收口与冻结

2026-10-08（Asia/Shanghai）。**确认有条件 Simulation Go；研究冻结，等待新车硬件验证。**
没有新的姿态求解算法，没有 Integration / Deployment Go。

本轮分支 `codex/full-base-input-contract-freeze` 直接从系统仿真
`6c5e8227a78d806bfddfe9c4e65d9742156b5c14` 建立；原算法
`experiment/full-base-heading-adapter@5733f126` 与系统验证分支均保留。
独立 worktree 为 `/home/qihei/rm2027_navigation/build/full_base_input_contract_freeze`。
开始时两个基线 worktree clean；主目录的他项 dirty 修改没有触碰。
FAST-LIO / Livox submodule 均未初始化，pin 仍为
`e7864a63a7e2a1ad62ec8fd75fcb9149db08e321` / `2a2029a6e62a2196b280be6ec00bb2418065b8e0`。
本轮没有使用或修改共享 submodule 工作文件。构建、ROS_HOME、日志、场景均在自己的目录；
使用 ROS Domain 98/99 和独立 Gazebo partition，只启动/清理本任务进程组。

最终只读审计观察到并行任务已把 `experiment/full-base-simulation-validation` 推进至
`4d7f40cbe4645267c002e4dd9cab2f9f252b1257`（其 parent 为 `6c5e8227`）。本任务未写入该
worktree 或 branch，未迁移、合并或评价其实现；本报告的实现和证据始终基于上述固定 `6c5e8227`。

[完整验证矩阵](full_base_input_contract_freeze/validation_matrix.md)、
[最终检查](full_base_input_contract_freeze/evidence/freeze_summary.json)、
[复现入口](../../experiments/full_base_simulation_validation/README.md)。

## 已解决的实际漏洞

| 项目 | 原行为 | 最小收口 |
|---|---|---|
| planar heading 路径 sensor frame | 任意非 base child 都可套用选定 home；空 child 自动补成 sensor | 只接受明确选定 sensor 或明确配置、解析后的 backend alias；空/错误 frame 不输出，不初始化 home |
| planar raw epoch 名称返回 | 拒绝 epoch2，返回 epoch1 字符串便恢复输出 | 初始化时记住 raw parent；观察到变化后复用既有 reset latch，返回旧名也不解锁 |
| full heading cache | 按 timestamp 字典覆盖；忽略 source sequence；invalid/offline 可能留下旧有效缓存 | source identity + sequence + source stamp 检查；同时间冲突/repeated sequence 新时间/倒退拒绝并清缓存；invalid/offline 清缓存 |
| full startup reference | 先建立 startup 再让数学函数拒绝错误 frame，可能污染后续 raw0 | frame/值先检查，且只有 startup 数学 `SOLVED` 才建立固定参考 |
| full epoch 与 pending | 依赖等到 heading 配对后检查 epoch | raw metadata 一到就观察 epoch；已观察到的 reset/时间回退保持既有 latch；缺 heading 也不能忽略变化 |
| 队列/缓存 | 重复 raw timestamp 可覆盖 pending；缓存靠后续 drain 才整理 | 重复 raw timestamp 明确无效；有限 pending/heading 容量及原始 stamp，不制造当前 pose |
| delay 故障注入器 | 取历史近邻值，却造 `now-delay` stamp 和当前 sequence | 显式延迟时保留实际选中源样本的原 stamp 与 sequence；刻意 `lie_stamp` 负例保留 |

实现是围绕原 solver 的传输合同，只有 cache、固定 startup 和既有不可自动解锁的 discontinuity latch。
没有恢复状态图、pose jump detector、历史根选择、速度/加速度阈值或推测固定延迟的“安全”标签。
数学 `scripts/adapter.py` 与 `5733f126` **逐字节一致**，canonical 数学和 S2 physics 没有修改。
`lio_adapter.cpp` 仅在本研究分支增加 16 行量级的输入 guard；main-new-car / 其他研究分支未改。
本轮的 planar 时间策略仍是原 30 ms nearest + rate propagation，full 策略仍为 1 ms source skew。
不能把两路的输出率差异解释为同一时间策略下的几何收益。

## 时间同步合同与 0.458° 根因

header stamp 的语义必须是 **产生该数值的源采样时刻**，经已验证的 clock 映射进入公共时间域。
它不是 ROS callback、packet receipt 或重新发布的时刻。source sequence 必须标识同一物理采样，
不是 relay 重发次数；MCU boot/reset identity 也必须对应该样本的真实估计器实例。

50 Hz 下周期 P=20 ms。如果 heading stamp 整体错移一个 P，raw(t) 可以正好匹配另一个
heading 样本的伪 t；两路 timestamp 仍相等，sequence 和 header 时间仍持续递增。
没有跨流共用、可信的采样 identity 时不能证明它们同一物理时刻。
恒定 chassis rate .4 rad/s 下，20 ms 错位产生 .008 rad，即 **.458366°**；真实仿真还叠加
预声明 startup reference 的微小误差，本轮最大 **.458453°**。未知 gimbal angle 可吸收这部分
错误 heading，给出仍满足刚体链的另一姿态；由 solver 的成功本身不能检出。

新 ROS 反例仍保留：周期 alias 和旧 heading 配合理的新 timestamp/sequence 都 `SOLVED`，
并产生约 .458366° 解析误差。这是不可观测边界的成功复现，不是正确姿态或安全时间通过。
若旧采样还保留重复/倒退 sequence，或新 sequence 复用冲突 timestamp，则当前输入确实足以
识别，新 guard 拒绝；不能据此宣称所有 hidden delay 都能发现。

明确分开五项语义：

- **采样**：消息必须说明数值所属采样时刻；消息频率、stamp 相等或 sequence 连续都不证明 clock 正确。
- **sequence**：允许 uint32 wrap、丢样本/gap；不要求每次加一。完全一致的 heading retransmission
  保留原对象，不覆盖缓存或刷新 identity；同 identity 内容/时间变化拒绝。
- **缓存**：invalid/offline/conflicting marker 之后清掉旧有效缓存；仅后续真正更新的有效采样可继续。
  不把 invalid/offline 的默认 boot/reset 值当作可证实 reboot。已经发布的较早输入不追溯改写。
- **同步**：full 最大 1 ms source skew 是研究 policy；不 rate-propagate、不 retimestamp。
  无 heading 不复用最后 pose。yaw_rate 只检查有限性，不借未声明的运动界限证明 heading 真实。
- **transport / freshness**：60 ms 是 raw 到达后的墙钟配对等待；2 s cache window 与 100 项容量
  是资源约束。它们都不是“物理观测年龄 ≤60 ms/2 s”。没有校准的源时钟到当前时钟映射，
  不能从当前 ROS 输入认证绝对 freshness 或共同固定延迟。

`source_skew/offline` 阶段保留了 1 帧 `SOLVED`：fault 标签从评分器发出请求时开始，
该帧解算使用先前已缓存、被周期偏移标成当前时间的有效 heading。新 invalid/offline marker
的输入级缓存清理由单测/实时 ROS 另行检查；不能把 fault request 边界等同于该 marker 已被
候选处理，也不能隐藏这个 .458° 的边界输出。

## Frame、epoch、startup 有效性

frame 的字符串和刚体数值必须同时有意义。输入 sensor 必须是该 F/K 描述的具体 IMU/sensor
origin；backend `body` alias 必须显式配置并解析为选定 sensor，不能当作 base。
ROS shadow 的 `sim_lidar_link` 是选定 CAD 代理 frame，不能证明真实 A/B 内部 IMU 原点。
H heading reference 必须是固定 gravity-aligned 世界中的 base +x azimuth，符号/rad/零点明确。
本 wrapper 在 startup 前检查声明的 parent/sensor/heading-reference/clock，不从 TF/truth 补齐。

raw epoch 必须代表 **不可复用的数值基准实例**，而不只是坐标系名称；startup reference、home、
calibration revision、raw0、heading0 都绑定这个实例。收到可观察的 raw parent/epoch 变化、
heading boot/reset 变化或 raw 时间回退，就拒绝旧参考，要求显式重新启动且重新确认 home/参考。
返回旧字符串、恢复较大 timestamp 不能恢复旧参考。

本仿真用 parent 名代理 raw epoch；真实 FAST-LIO Odometry 没有独立 raw epoch generation。
**若 backend 重置却继续同名、时间保持递增，且没有任何 reset metadata 被收到，当前输入不能
认证或检测这个隐蔽 rebase。** 不加位置/姿态跳变阈值冒充 epoch 检测。实车需真实 generation /
session / reset 通知，或建立能验证 backend parent 固定的启动协议；frame label 不能代替它。

完整 startup attitude/gravity、home 与标定还需要正确的物理来源及采样时间。内部一致但错误的
level reference / 外参仍可能 `SOLVED`。本仿真的 RP 是预声明 fixture，home 也是明确 fixture
条件，不是对实车的新测量。reject 首个坏输入不会证明下一帧仍在 home；硬件 startup 接受时必须
重新确认 home 和 attitude 在那个源时刻有效，不能默默用运动后的第一帧代替。
本轮没有新增可部署的 epoch、startup-attitude、clock-authentication 消息接口。

## 回归、失败记录与 truth 边界

- 原证据保持原样：19 组物理系统场景、14,454 个运动/故障样本、217 项检查。
- 全部 19 组固定输入重放；正常 motion 的 full 输出 100% `SOLVED`，与原记录矩阵差 <8e−16。
  包含 A/B、11°/15°上下坡、bank、倾斜启动、动态云台和多圈 wrap。旧 trace 不含 source sequence
  和 callback 顺序，因此用单调 proxy 做固定 pose 输入回归，**不声称重建旧 DDS freshness**。
- 原算法 31 项单测从固定 Git blob 提取到本任务 build 并通过，另有 21 项输入合同测试通过。
  原 C++ canonical 15 项及 S2 geometry CTest 通过。
- 四组原解析 ROS 边界重跑通过：`AMBIGUOUS` continuum / two roots、`ILL_CONDITIONED`
  near singular、时间回退拒绝，均无 full pose 输出。
- 九组实时 ROS 输入控制 / 47 项检查通过；包括 sensor/empty frame、raw/heading epoch return、
  repeated sequence、offline、冲突 timestamp，以及两个不可观测时间反例。
- 九组新 Gazebo 回归 / **7,000** 个运动故障评分样本；六组正常几何、多圈动态云台、倾斜启动
  满足原理想观测筛查，三组 frame/epoch/time 故障保留。140 项最终检查全部通过。
  错误 sensor 的 98 帧两路均零输出；raw epoch 名返回的 98 帧两路均继续零输出。

候选只接显式 raw / heading / fixture calibration / fixture startup；没有 truth、joint、TF 输入。
重放仅传 whitelist 的 raw/heading 给合同层，输出后才比较 scoring truth；新 Gazebo 节点图
再次确认 truth 防火墙和 shadow TF/command 边界。评分器的 q 不提供给候选，也不用于选根。
正式 TF、Nav2、cmd_vel owner 未变；runner 的 open-loop 命令仅在本任务隔离仿真域内。

首次验证脚本的目录创建遗漏及尾部停机未收齐 output 的假设错误，已修正并保留为 harness
失败记录；没有改原测量、姿态算法或 gate 来让结果通过。SIGINT 退出日志是本任务进程组清理。

## 冻结结论与实车待测

当前没有新的实质性阻塞，**有条件 Simulation Go，FROZEN_WAIT_HARDWARE**。
仅适用于声明的理想 sensor pose、真实同时间 heading、已确认完整链/home/startup reference、
稳定且有可验证 generation 的 epoch，以及非歧义/非病态姿态域。所有非 `SOLVED` 的顶层 pose
仍为 null；`SOLVED` 不授予部署、正式 TF 或定位有效性。pose carrier 的 twist/covariance 仍未验证。

新车到位后先在本冻结分支验证：

1. A/B 实测 F/K、axis/home/符号/回差、IMU↔LiDAR origin 与机械刚性；不要复制本 CAD/synthetic 参数进正式 YAML。
2. MCU yaw 是否为固定 gravity 世界的 base azimuth；倾斜启动完整 attitude/gravity 的物理来源、时间与 home 重复性。
3. 捕获 MCU sample time/sequence/boot/reset、LiDAR/IMU scan time、host receipt/publish time，校准 offset、drift、
   固定延迟和 jitter；验证 source sequence 不是中继计数，time relabel 没有掩盖历史采样。
4. backend raw zero-start/rebase/reboot 的真实 generation，实例不复用及 startup/home 绑定；注入 reset 和漏包验证拒绝。
5. 真实 FAST-LIO full sensor pose、deskew、高角速、云台运动、斜坡和启动 reference 误差。
   仿真理想观测不能证明这几项。覆盖实际姿态域；需要跨歧义/病态域持续输出时增加独立信息。

这些条件成立后才评估最小迁移与正式 pose/twist/covariance/epoch 接口；本轮不再开发新算法、
不 merge、不 push、不修改 main-new-car / main-old-car 或其他研究分支，等待实车。
