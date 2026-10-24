# Posture-Aware Motion Constraints（2027 软件合同）

Integration 候选。基于 `main@d37f8d6e`，不依赖 target arbitration feature。
适用于现有 Humble 1.1.20 MPPI + velocity_smoother 链；其他 controller 不自动适配。

## 实际姿态输入

唯一比赛输入是 Competition V2 transport 的 `/robot/posture/state`。
`RobotPosture` 与 wire enum 不重编号：

| Wire/ROS value | Native name | Navigation effective state | Profile slot |
| ---: | --- | --- | --- |
| 1 | MOVE | MOBILE | mobile |
| 0 | ATTACK | ATTACK | attack |
| 2 | DEFENSE | DEFENSE | defense |
| 5 | ENHANCED_MOVE | ENHANCED_MOBILE | enhanced_mobile |
| 3 | ENHANCED_ATTACK | ENHANCED_ATTACK | enhanced_attack |
| 4 | ENHANCED_DEFENSE | ENHANCED_DEFENSE | enhanced_defense |
| 非六态/无有效观测 | — | UNKNOWN | fallback |
| 超时/时间不可信 | — | STALE | fallback |

只使用 `posture_actual`，要求 valid、online、无 fault 且非 transitioning。
`posture_target` 是期望状态；ACK 匹配不等于完成；`completed` 还依赖当前上位机请求，
不能作为 effective profile 的前提。例如 target 仍为 ENHANCED_MOVE，actual 已恢复 MOVE，
即使 completed=false，也必须选择 mobile。导航不推测强化结束时间，不发送姿态请求。

V2 `0x81` 已能独立表达 target/actual 的全部六态，允许上游在强化结束时直接上报普通实际态。
RefereeState 当前没有姿态字段，MissionState 也不是姿态 producer。当前 mock lower 只响应
PostureRequest，不会模拟自主退出强化；测试通过合成明确的 actual 状态变化验证消费链。
实际固件主动、周期上报 measured actual 的行为仍待硬件接口验收。

ROS header 是 transport 的**主机接收时间**；有 valid/online/transitioning/fault。
wire envelope 有 per-type uint16 sequence，但当前 PostureState ROS 消息未携带它，也无
MCU posture sample time/boot。不能宣称已验证机械测量时刻或源采样防重放。
transport 的 0.5 s posture timeout 和连接/capability gate 仍有效，consumer 再独立校验
采样年龄与单调时钟 lease。重复同一 header stamp 不能延长 lease。

## 现有运动约束链与 owner

所选正式 competition 默认 YAML 是 `nav2_old_car_2026_left_stvl.yaml`：

| Layer | vx/vy/wz limit | accel/decel | Runtime mutable | Role |
| --- | --- | --- | --- | --- |
| Mission/BT | 无数值 envelope | 无 | 任务/安全门 | 决定去哪里和是否允许任务；本轮不改 |
| MPPI FollowPath | vx [-0.30,0.50]，vy ±0.50，wz ±1.20 | 此 Humble 实现无对应独立 accel/decel 参数 | 是，四个速度参数 | 预测/采样速度约束 |
| Nav2 behavior（Spin 等） | Spin max_rotational_vel=1.20 | rotational_acc_lim=2.00 | 本轮保持静态 | 独立 behavior 的既有参数，输出仍经过 smoother |
| velocity_smoother | max [0.50,0.50,1.20]；min [-0.30,-0.50,-1.20] | max_accel [0.80,0.80,2.00]；max_decel [-0.80,-0.80,-2.00] | 是，四组数组 | 主要 upper command envelope 执行者 |
| managed_motion_ingress | 无数值 clamp | 无 | 不涉及本轮 | finite/GID 来源过滤，不授予 authority |
| V2 transport | 默认 ±[0.80,0.50,6.00] | 无额外 accel clamp | 限值只启动读取 | 已有上位机防御限幅 + AUD-003 编码前最终授权；本轮不改 |
| legacy/HPM transport | 默认 ±[0.15,0.15,0.30]，可启动覆盖 | 无 | 限值只启动读取 | 互斥旧车路径，不参与姿态动态同步 |
| chassis stub | 默认 ±[1.0,1.0,2.0] | 无 | 限值只启动读取 | 仿真/stub，不是 MCU |
| MCU | 按实际姿态的 vx/vy/wz hard clamp | 下位控制/功率实现待确认 | 下位 owner | 最终物理命令限幅，不由上位机 profile 取代 |

三轴都在 MPPI、smoother 和所选 transport 限制；accel/decel 主要在 smoother。
TinyMPC 未进入当前 `src/` 运行链，本轮只确认其不在选定 owner 路径，不导入冻结研究。

代码审计依据是项目 YAML、transport/stub 的启动参数与 clamp、Nav2 1.1.20 官方源码：
[smoother](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_velocity_smoother/src/velocity_smoother.cpp)、
[MPPI optimizer](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/optimizer.cpp)、
[MPPI parameter handler](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/parameters_handler.cpp)。
两者支持 active 时动态修改；MPPI 更新会重置内部优化器，不等于取消 Nav2 action。
仅改 smoother 会留下 MPPI 的旧速度预测边界，因此最小必要同步为 **smoother 四组边界 + MPPI 四个速度参数**。
不改 planner、behavior、serial 或 MCU 的动态参数。

## 最终数据流

```text
CURRENT_EFFECTIVE_POSTURE
  -> six-state selector / freshness / 50 ms debounce
  -> seven named profile slots (six + fallback)
  -> smoother max/min velocity + accel/decel
  -> MPPI velocity bounds (same selected profile)
  -> existing cmd_vel chain / final host authorization
  -> MCU hard clamp
  -> motors
```

主 profile owner 是 `rm_posture_motion` 的 selector；smoother 是输出 envelope 执行者。
MCU 是最终 hard clamp owner，profile 不是运动授权。节点只发参数 RPC 和诊断，没有 Twist、
姿态请求、Mission 服务或 Nav2 action client，不能改变任务/目标或恢复已拒绝的授权。
已有 smoother 继续是 final `/cmd_vel` owner；受管 ingress/V2 随后执行原来的来源和授权检查。

## Production 与测试配置

production 的六种实际物理 profile 均 **HARDWARE_TBD / HARDWARE_BLOCKED**。
每个独立槽位与 fallback 暂时继承所选既有 baseline 的四组数组；没有推导或校准新值。
启动先读回 active smoother/MPPI 参数，必须与传入 Nav2 YAML 一致，否则零次写入、ERROR。
这样不会因传错 baseline 文件而静默扩大已运行的约束。

测试文件在 `src/rm_posture_motion/test/fixtures/profiles_test_only.yaml`，明确标注
TEST ONLY / NOT PHYSICAL / NOT COMPETITION CALIBRATION；只供 test source mode 使用，
不安装成 production 配置。所有测试 slot 也不超过选定 baseline。production 在未标定阶段
拒绝 concrete override；不能把测试数值改标签后混入比赛配置。

`posture_motion.launch.py` 默认关闭；启用时固定 competition source 和 production manifest，
不启动 mock/硬件/Nav2。真实源固定 `/robot/posture/state`，测试源固定 `/test/posture/state`。
可信 ROS graph 是前提，topic 隔离不是 DDS 身份认证。

## 切换、freshness 与诊断

- 首条有效输入之前、unknown enum、invalid/offline/fault/transitioning：fallback。
- 0.5 s 源时间/稳态进展 lease；stale/source lost：STALE → fallback，不保留增强态。
- 正常 profile 改变需稳定 0.05 s；失效到 fallback 不等待 debounce。
- 六个 profile 即使数值相同也保留独立 state/key/generation；只有参数 payload 的重复写入被抑制。
- 数值改变时先确认目标节点 ACTIVE，再原子更新 smoother 四组数组，然后 MPPI 四个速度参数。
  RPC 期间输入合并为最新选择，不为每条 40 Hz 姿态消息排队写参数。
- 两个节点之间不是分布式原子事务；最终 applied generation 只在两边确认同一 profile 后发布。
  短暂同步窗口中 smoother 先约束输出；速度下调遵循既有 decel，不能宣称瞬间硬限幅。
- RPC 拒绝/2 s 未确认：ERROR、applied_profile=unconfirmed，停止继续写入。不能假称 fallback
  已生效。修复服务/配置后，以已知 baseline 重启 selector；本轮不做多节点自动恢复框架。
  selector 自身退出也不会新增速度 owner 或改变 MCU authority。
- `/posture_motion/status` 复用 DiagnosticArray：effective posture、freshness、selected/applied profile、
  generation、revision、fallback reason、backend error 和写入计数。

参数服务/selector 失效不能保证新的 upper profile 已应用；MCU clamp 与最终授权仍独立生效。
这不是新的 safety controller。本轮未校准任何姿态物理性能，未建 power→velocity/accel 模型，
未实现 5 s 冷却、3 min/15 s 累计或强化生命周期管理。
