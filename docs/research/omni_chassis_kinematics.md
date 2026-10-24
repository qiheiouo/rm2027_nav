# 参数化 Omni Chassis Kinematics / Feasible Region Harness

最终状态：**FROZEN_WAIT_CHASSIS_PARAMETERS，Research，2026-10-24**。
Stage 1 / Stage 2 已完成，冻结候选等待真实底盘参数，不再自动新增算法。
本页最后的冻结复核与重启条件优先于下文保留的阶段性 Continue 判断。
新车真实轮组参数尚未确认，本工具不指定最终实车 J。
实现位于 [`experiments/omni_chassis/`](../../experiments/omni_chassis/)，由 `COLCON_IGNORE`
隔离；没有 ROS 节点、串口、控制器、launch 或主导航链改动，不作为实车控制入口。
所有结果只对应输入配置下的理想运动学，不构成 Hardware Go。

## 研究问题与最小可判定实验

假设 H1：给定完整固定轮组参数，可以自动生成一致的运动学和轮速约束多面体，
无需预先写死四轮对称 J。假设 H2：独立的 vx/vy/wz box 一般不能同时代表轮级耦合约束
和保留全部可行命令；应提供共享半空间接口，供后续优化与采样控制研究选择。

对照为两种独立轴限幅：配置给定的 box（本轮是合成值，并非现行 Nav2 配置）、
由正负纯轴极限组成的 box。另加入其最大同形内缩 box，代表可证明保守的 box 候选。
精确轮级约束为参考；不同方法使用完全相同的速度样本、种子、统计口径和坐标系。

预设范围：4 个合成配置（3/4/4/6 轮），每配置 5 个固定种子
`11,23,47,71,101`，每种子 20,000 个速度；另做 2,048 条随机方向、6 条轴方向和全部
顶点方向的边界测试。固定布局的 Monte Carlo 衡量命令分布，不解释为车辆参数后验分布。
小型核心测试另外使用固定种子的 16 个随机布局（3/4/6/8 轮），检查参数化是否隐藏了四轮假设。

判定标准在 `run.py` 中显式实现并控制退出码：

- 独立刚体点速度计算与 J 的轮速差，按驱动上下限归一化，最大值 `<1e-10`。
- `forward(inverse(v))` 按配置速度尺度归一化误差 `<1e-10`。
- Monte Carlo 中独立轮速判定与半空间判定零不一致。
- radial clip 输出的最大相对轮速越限 `<1e-9`。
- 顶点满足约束；射线边界相对残差 `<1e-9`，沿射线内移 `1e-6` 全部接受、外移
  `1e-6` 全部拒绝；内缩 box 的全部角点和接受样本无越限。

先用已知闭式多面体、零命令和小样本检查测量链，再运行上述规模。
H1 数值检查通过、H2 出现可复核反例则 **Continue：保留共享可行域候选**；
仍不进入 Integration。核心数学检查失败先排除配置/实现错误并最小复核，仍失败则
Modify/Stop。若仅某 box 恰好等于可行域，不据此宣称所有布局必须使用多面体。

## 配置、坐标与轮速单位

JSON schema 1 的唯一示例入口：

- [`synthetic_symmetric4.json`](../../experiments/omni_chassis/configs/synthetic_symmetric4.json)：
  人工构造的对称四轮，用闭式八面体验证。
- [`synthetic_asymmetric4.json`](../../experiments/omni_chassis/configs/synthetic_asymmetric4.json)：
  不同轮位、方向、半径、传动比、驱动符号与非对称速度界。
- [`synthetic_three.json`](../../experiments/omni_chassis/configs/synthetic_three.json)、
  [`synthetic_six.json`](../../experiments/omni_chassis/configs/synthetic_six.json)：改变轮数与冗余度。
- [`mcu_pending.template.json`](../../experiments/omni_chassis/configs/mcu_pending.template.json)：
  **未确认参数为 null，执行时拒绝生成结论，不回退到合成配置。** 轮列表长度可改。

这些数字均为合成 fixture，没有从 CAD 仿真 J 推断真实装配。无隐藏的默认轮位、轮序、
方向、半径、传动比或速度界。必填键和单位在加载时检查，未知键不会被静默忽略。

遵循现有 [chassis contract](../contracts/chassis_contract_2027.md) 的 body 速度语义：
在 `base_link` 原点，x 前、y 左、z 上，wz 绕 z 逆时针为正；
`v=[vx,vy,wz]` 单位分别为 `[m/s,m/s,rad/s]`。每轮必填：

| 字段 | 定义 |
| --- | --- |
| `name` | MCU 驱动 ID 的明确映射；配置数组顺序就是矩阵与反馈轮序 |
| `position_xy_m` | 从相同 body 速度参考原点到轮接触位置的平面偏移，m |
| `rolling_direction_xy` | 正向轮转动对应的单位滚动向量，**不是轮轴方向** |
| `radius_m` | 理想滚动有效半径，m，逐轮填写 |
| `drive_per_wheel_ratio` | 正的驱动端角速度/轮端角速度绝对比，例如减速 19:1 填 19 |
| `drive_sign` | 驱动端正转相对上述正向轮转的符号，+1 或 -1 |
| `speed_bounds_drive_rad_s` | 带符号的驱动端 `[lower,upper]`，rad/s，支持不对称 |

若使用电机 RPM：先乘 `2π/60` 转 rad/s；若输入已为物理轮端 rad/s，则 ratio=1，
不能再乘一次减速比。方向和 drive_sign 共同定义有符号映射，不隐含“左轮反转”等惯例。
本版要求每个速度区间有限且 `lower<0<upper`，使零速度为严格内点。
单向驱动、锁死轮、不可行或低维配置不在本轮生成域内；不静默补上底盘限幅掩盖退化。

`study.velocity_scale` 只是数值坐标尺度 `[m/s,m/s,rad/s]`，用于条件数、误差与顶点构造；
不改变物理约束。`configured_box_lower/upper` 是待比较的独立轴速度界，亦需包含零。
`parameter_status` 为 synthetic / unconfirmed / mcu_confirmed，`source` 记录参数出处。
即使标为 mcu_confirmed，导出的 `hardware_validated` 仍为 false：参数确认不等于物理验证。

## 从参数自动生成运动学

第 i 轮位置 `p_i=(x_i,y_i)`、单位滚动向量 `d_i=(dx_i,dy_i)`。
刚体接触点速度为：

```text
v_point_i = [vx - wz*y_i, vy + wz*x_i]
r_i * omega_wheel_i = dot(d_i, v_point_i)
omega_drive_i = sign_i * ratio_i * omega_wheel_i

J_i = sign_i*ratio_i/r_i * [dx_i, dy_i, x_i*dy_i - y_i*dx_i]
omega_drive = J @ v
```

inverse kinematics 为 `omega=Jv`；forward kinematics 为 `v_hat=K omega`，
`K=pinv(J)`，是**驱动 rad/s 坐标下未加权最小二乘**，不是带噪测量的最优融合器。
秩为 3 时 `KJ=I`。冗余轮组的任意 omega 未必在 J 的列空间内，故提供
`residual=omega-JKomega`；有残差的反馈不能被称为精确无滑运动。符号错、打滑、测量噪声
均可能产生残差，本工具不由残差自行诊断硬件原因。

数值秩和条件数以 `diag(1/max(-lower,upper)) J diag(velocity_scale)` 计算；
归一化最小/最大奇异值比不大于 `1e-10` 时拒绝生成三维有界域。
另检查实际 LS 逆是否能恢复三个轴，无法数值解析时明确报错。

## 可行多面体与 box

轮速约束为 `lower <= Jv <= upper`，因此：

```text
P = {v | A v <= b}
A = [ J; -J ]
b = [ upper; -lower ]
```

有限轮速、满列秩 J 保证 P 有界，严格包含零的轮速区间保证 P 三维且有内点。
秩不足时沿 null(J) 无界，应报告退化，而不是发明实车的 yaw 或平移上限。
实现先在无量纲坐标中枚举三个约束平面的交点，保留满足全部半空间的点并去重，
再由 convex hull 构造 V-representation、三角面与体积。多余半空间保留其轮 ID，
没有假设所有轮约束都构成实际面。该组合枚举适合少量车轮，不追求大规模多面体库性能。
核心测试用独立 LP 对 26 个方向的支撑值交叉检查顶点是否缺失。

**不能用 `K` 把整个独立轮速 box 映射到 chassis 后，声称那就是 P。**
冗余轮速 box 中含有不相容向量；`JK` 的投影也不保证保持轮速界。
测试中存在每个驱动都在 ±40 rad/s 内、其 LS 底盘估计重新映射却达到 60 rad/s 的反例。

报告区分：

1. configured box：直接从配置读取，不是实车已验证上限。
2. pure-axis box：分别沿 `±ex,±ey,±ew` 求最大射线长度，以轴截距组成 box。
   轴上的单独命令可行，任意轴组合未必可行；一般布局中它也未必包住整个 P。
3. homothetic inner box：对 pure-axis box 用同一个 α 缩放，取所有角点满足 Av≤b 的
   最大 α。角点可行与 convexity 共同保证整个 box 可行。它不是所有中心/宽高比中
   最大体积的轴对齐 box。
4. bounding box：从 P 顶点取各轴 min/max，只用于展示与公共采样域，不作为可行控制约束。

对任意方向 q，射线边界为
`t_max=min_{i:A_i q>0} b_i/(A_i q)`；零方向返回 infinity。
`radial_clip(v)=min(1,t_max(v))*v`，保方向收缩至可行域。
它没有混用 m/s 与 rad/s 的欧氏距离，也不声称是最近点投影或实际控制策略。

几何体积的单位为 `(m/s)^2*(rad/s)`，仅比较同一坐标单位下的可行域容量；
体积占比和 Monte Carlo 越限率均不是轨迹效果、能耗或实际命令的发生概率。

## 统一可行域接口候选

[`model.py`](../../experiments/omni_chassis/model.py) 提供纯离线候选：

| 接口 | 用途 |
| --- | --- |
| `inverse(v)`, `forward(omega)`, `residual(omega)` | 配置导出的运动学与相容性诊断 |
| `A,b`, `slack(v)`, `contains(v)` | 共享轮级半空间、驱动 rad/s slack、可行性判定 |
| `linear_constraints(M,c)` | 对 `v_body=M z+c` 返回 `C=A M, d=b-A c`，约束 `Cz<=d` |
| `ray_limit(q)`, `radial_clip(v)` | 采样边界与保方向可行化候选；支持批量 `(...,3)` |
| `export()` / `model.json` | 完整输入、J/K、轮序、A/b、顶点、三角面与 box，可由其他语言消费 |

TinyMPC 候选：若优化变量直接是 body twist，使用原 A/b；若速度位于状态某些列，
用 M 选择对应列；若研究采用 `v_next=v_now+dt*a` 抽象，显式传
`M=dt*I,c=v_now`，不能把速度约束直接当作加速度 box。世界系速度需要使用该时刻 yaw
构造 world→body 的 M，yaw 也是优化变量时涉及非线性/局部线性化，本轮不解决。
[TinyMPC 官方 Python 接口](https://github.com/TinyMPC/tinympc-python#constraints-api)
在研究时展示了线性不等式接口；未来接入仍须固定所用版本，验证 state/input 排列、求解容差
和实际约束残差。本轮没有安装、调用或验收 TinyMPC solver。仅支持 box 的版本可研究
保守内缩 box，其体积损失已量化；不宣称用原独立 box 就能表示轮约束。

MPPI 候选：对同批 body twist rollout 使用 `contains()` 作可行过滤，或研究
`radial_clip()` 约束采样；最终加权平均在固定同一凸 P、非负归一权重下仍在 P 内。
坐标旋转、时变限制和后处理必须逐时刻重查。
本轮未实现 MPPI critic、动态模型或 Nav2 adapter，也未比较闭环跟踪收益。
配置改变应重新创建模型和输出，不混用旧 J、旧多面体与新轮序。

## 复现与 MCU 参数更新

使用项目已可用的 Python、NumPy、SciPy；`--plot` 额外使用已可用的 Matplotlib。
没有新增求解器或 ROS 依赖。从仓库根目录执行：

```bash
python3 -m unittest discover -s experiments/omni_chassis -p 'test_*.py' -v
python3 experiments/omni_chassis/run.py --plot
```

默认只读取 `configs/synthetic_*.json`。输出到该工具的 `outputs/`（已忽略），每配置生成
`model.json`，可选 `slices.png`；总表 `results.json` 记录环境、base revision/dirty 状态、
种子、样本域、计数、每种子比例、汇总 Wilson 95% 区间、边界误差和判定。
所有轮级判定使用每侧轮速界 `1e-9` 的相对浮点容差，不应视为硬件安全裕量。

MCU 提供实际信息后：

1. 复制 pending 模板为自己的 JSON；按实际轮数、ID 与轮序填写全部几何与驱动参数，
   确认参考原点、滚动方向/轴、正负转、有效半径、传动比定义、端点和速度单位。
2. 在 source 记录固件版本与机械/标定来源，尚未证实的配置保留 unconfirmed；
   填写需要比较的上层 box。不要把 null 替换成 fixture 数字冒充实际信息。
3. 只传入该配置重新生成，查看条件数、J/K、A/b、box 损失及验证判定：

```bash
python3 experiments/omni_chassis/run.py \
  --config /path/to/measured_chassis.json \
  --output /tmp/omni_measured_research --plot \
  --samples-per-seed 20000 --seeds 11 23 47 71 101
```

`--config` 可重复，多配置使用不同 name 与文件名；自定义路径不依赖当前目录。
未完整填写模板时命令退出 2 且不创建部分结论。任何研究数值门失败退出 1。
运行通过只证明配置下的运动学一致；实测轮速/时间戳、MCU 饱和策略与动态约束仍需独立核对。

## 结果与决策

固定协议共 **400,000 个速度样本、8,250 个边界点**。7 个核心测试通过，含解析八面体、
独立刚体点速度、104 个 LP 支撑方向、16 个随机轮组布局、轮序/几何/半径/传动比/符号/
限速变化、仿射约束映射、冗余反馈残差及 MCU 配置替换后 CLI 重新生成。
最小关键证据见 [`evidence.json`](../../experiments/omni_chassis/evidence.json)。

“误接受”定义为该 box 接受的命令中，违反轮级约束的比例；“误拒绝”定义为
真实 P 内命令中，被该 box 拒绝的比例。表中区间为汇总条件比例的 Wilson 95% 区间。
各配置使用各自的公共包围采样域，因此不把不同布局的比例直接当作布局优劣评分。

| 合成配置 | 顶点数 | configured box 误接受 | pure-axis box 误接受，95% 区间 | pure-axis 误拒绝 | 内缩 box 体积/P |
| --- | ---: | ---: | ---: | ---: | ---: |
| symmetric4 | 6 | 56.31% | 83.25% [83.02%,83.48%] | 0% | 22.22% |
| asymmetric4 | 12 | 51.74% | 73.58% [73.27%,73.89%] | 0.61% | 17.60% |
| three | 8 | 63.94% | 62.98% [62.61%,63.36%] | 3.34% | 18.39% |
| six | 8 | 54.70% | 74.81% [74.54%,75.08%] | 0% | 25.60% |

pure-axis 误接受率在五种子间的范围依次为 symmetric4：82.87–83.54%，
asymmetric4：73.17–73.94%，three：62.34–63.74%，six：74.39–75.17%。
内缩 box 的样本误接受均为零，且全部角点满足约束；其可行性依据是凸几何保证，
不只是“随机没撞到反例”。实际保留的命令比例依赖分布，不能把几何体积比当作现场收益。

全部样本的独立轮速/多面体判定零不一致。最大归一化轮速计算差 `1.07e-15`，
最大尺度化运动学往返误差 `1.34e-15`，最大边界相对残差 `5.33e-16`，
radial clip 最大相对越限 `6.67e-16`（浮点量级）；内移均接受、外移均拒绝。

闭式 sanity case 进一步排除了“只是 MC 恰好如此”：symmetric4 满足
`|vx|+|vy|+0.5|wz| <= 2*sqrt(2)`，体积为 `60.3397786613`；
pure-axis box 体积为 P 的 6 倍，均匀采样的理论误接受率 5/6≈83.333%，位于本轮区间内。
其最大同形内缩比例 1/3，内缩体积/P=2/9≈22.222%。

**结论：H1 在上述参数域内得到支持；H2 的一般性风险由解析反例和非对称布局实验支持。
Continue，仅保留参数化研究工具及共享半空间接口候选。** 不选择最终实车 J，
不宣称 TinyMPC/MPPI 闭环效果更好，不进入导航集成或正式控制器开发。
后续有真实 MCU/机械参数时，从模板重新生成；若要研究真实执行一致性，下一项有效信息是
同一轮序/时域的受控 body 命令与实测轮速，而非继续扩大本轮软件回归规模。

原始速度样本、全部生成矩阵/图片和重复过程日志可重新生成，不进入长期版本资产。

## 明确限制

第二阶段的数学、证据和接入边界见下节；上述第一阶段结果不是控制器选择结论。

固定轮组、刚性平面、理想 omni 自由滚子、无滑动的瞬时速度模型。非 Mecanum
滚子角模型；无轮加速度/jerk、力矩、电流、功率、摩擦椭圆、负载转移、死区、延迟、
下位机内环和姿态变化。真实车限速若随电池/功率/模式改变，应更新对应配置/域；
不能由本轮静态多面体推断完整动力学可行性。没有真实新车 J，也没有实际轮速测量或闭环效果证据。

## 第二阶段：最大 box、投影与参数误差（Research）

继续同一实验分支。实现新增 [`optimization.py`](../../experiments/omni_chassis/optimization.py)、
[`uncertainty.py`](../../experiments/omni_chassis/uncertainty.py) 和
[`stage2.py`](../../experiments/omni_chassis/stage2.py)。本节是该方向唯一后续研究记录，
不新增 controller、ROS 包或实际输出 owner。所有几何、驱动参数仍来自第一阶段配置或
明确声明的合成参数生成器。

### 判据与公平比较

可复现的实验协议集中在
[`stage2_protocol.json`](../../experiments/omni_chassis/configs/stage2_protocol.json)：

- 四个固定 fixture，每配置 3 个种子、每种子 1,000 个公共输入；统一扩大后的包围域。
  比较 pure-axis naive clip、同形 conservative box clip、最大零包含 box clip、
  ray scaling、raw Euclidean projection 和 weighted projection 六种方法。
- 相同输入下报告独立轮级 violation、raw/scaled/weighted distortion、分量误差、方向误差、
  固定点可行体积、每种子结果；报告边界、零命令、幂等性和局部增益。
- runtime 分离 box 求解/投影预计算、单命令延迟及 64 命令 batch 均摊时间。
  单命令 128 次×3 轮，报告 median/p95/p99/max；轮换方法顺序，固定 BLAS 线程数。
  这是当前 Python/CPU 的测量，不是硬件实时性承诺或 C++ solver benchmark。
- 误差研究为 radius / position / angle / speed / combined 五族、三级显式预算、
  每级 16 次独立扰动、每配置每次 512 个公共输入。扰动不是新车参数先验或标定置信区间。
- 大规模参数回归为 5 种子×200 布局、每布局 64 命令，轮数 3–8，轮位/角度/半径/
  减速比/正负转和非对称轮速界均随机。配置与命令使用 `[seed,index,stream]` 独立随机流，
  防止失败或短路改变后续样本。随机范围只是压力测试域，不表示实际车辆范围。

固定数值门：投影相对越限 `<=1e-8`、顶点变分最优性残差 `<=1e-6`、最大 box 的
log-volume 对偶 gap `<=1e-5`、独立 SLSQP 投影参考的归一化 metric 距离 `<=2e-5`、
常规布局运动学往返误差 `<=1e-8`；每 25 个布局交叉核对两种投影各 4 个命令。
naive 方法预期可能越限，作为反例记录，不据此把整个实验判失败。
近奇异族单独报告接受/拒绝和条件数，不把主动拒绝退化输入计为随机常规布局成功。

Continue 条件是数学证书、共同输入比较及参数误差边界得到支持，且未掩盖失败；
若一个算法持续无法给出可验证解，记录失败域并 Modify/Stop 该候选，不抬高误差门救结果。
即使全部软件门通过，也只保留研究候选，不能自动进入导航集成。

### 最大轴对齐内接 box 的凸问题

以中心 c、半宽 h>0 表示 box：`B(c,h)={v:|v-c|<=h}`。线性函数在角点取极值，所以：

```text
B(c,h) subset P   <=>   A c + |A| h <= b
maximize log(volume) = 3 log(2) + sum_j log(h_j)
```

约束为线性、目标为凹函数，因而是凸优化问题，局部最优即全局最优。半宽唯一，中心可能
不唯一。工具分别计算 `origin: c=0`、`free: c自由`、
`contains_zero: -h<=c<=h`。最后一种用于 command box 候选，避免 clip 把零命令变为运动。
free box 可能排除零，不能仅因为体积最大就直接用作速度限幅。原点居中 box 的可选集与
此前中心略偏的非对称同形 box 不同，不要求它一定更大；零包含最优 box 则必须不小于
那个已知可行竞争者。

数值求解使用现有 SciPy SLSQP 和解析梯度，中心按 P 包围盒尺度、半宽按已知内接 box
宽度分别预条件。坐标变化不改变凸问题；避免瘦长域中 `1/h` 梯度与中心尺度失配。
初始回归的 23/1,000 例原始可行性失败定位到这个尺度问题，修正后先重放保存的五个
代表失败配置，未放宽数值门。

不只检查 solver.success。设预条件变量满足 `C c + D t <= r`，t>0；任取 λ>=0，
`q=D^T λ>0`，且可行中心位于已知包围区间 L<=c<=U，则：

```text
sum log(t) <= λ^T r + support_[L,U](-C^T λ) - sum log(q) - 3
support_[L,U](s) = sum_j max(s_j L_j, s_j U_j)
```

这是拉格朗日弱对偶界。中心站立性即使只有浮点精度，显式 support 项仍保留其影响。
工具从近活动行恢复非负乘子，检查所有角点、对偶界及 gap；体积上界为当前体积×exp(gap)。
消除浮点级越限后，对最终 box 重算证书。solver 标志、原始残差和证书均保留；
有证书而 solver 标志非成功与“求解器收敛”分开报告，证书失败不会用另一个 box 静默替代。
另有明确反例测试：将三轮合成配置的驱动正向统一、轮速界设为 `[-0.01,40]`，free 最优
box 会排除零命令；`CommandProjector` 拒绝把该 box 当作 command box 候选。

### 保留体积、失真与投影的数学差异

“retained feasible volume”定义为
`Vol({v in P: F(v)=v})/Vol(P)`，不使用 projected 点云的 convex hull 冒充体积。
三种 box clip 的集合为 `P intersect B`；naive box 用半空间交集精确计算，
两种已证明内接 box 直接用 box 体积。ray、Euclidean 和 SPD weighted projection
保持 P 内所有命令，因此均为 100%；这不意味着它们对域外命令的失真、运行时间或闭环行为相同。

| 方法 | 域外命令处理 | 保证/局限 |
| --- | --- | --- |
| naive axis clip | 每个分量独立夹到纯轴截距 | 不保证轮级可行；不能拿低失真当优势 |
| conservative box clip | 夹到同形内缩 box | 几何可行，主动改变一部分 P 内命令 |
| maximum box clip | 夹到最大零包含 box | 该 box 家族保留体积最大，不保证平均命令失真最小 |
| ray scaling | 沿原点射线收缩到 P | 保方向、保内部，不是最近点；可能改变所有速度分量 |
| Euclidean projection | 最小化 `0.5 ||v-u||²` | raw 数字坐标下失真最小；m/s 与 rad/s 混合，换单位会改变结果 |
| weighted projection | 最小化 `0.5 (v-u)^T W(v-u)` | W 必须 SPD；在明确尺度/优先级下最小，不是所有分量误差都更小 |

报告同时给 `||v-u||`、`||S^-1(v-u)||` 与 weighted norm，S 为配置 velocity_scale。
本轮 `W=S^-T diag(1,1,16) S^-1`，这是**研究用 yaw 优先级**，不代表最终控制器权重。
先前 `[1,1,4]` 与固定 fixture 的 S 恰好产生 W=0.25I，属于等价问题，不构成两种方法
有差异的对照；已修正并重新运行。结果也导出 `weighted_equivalent_to_euclidean`，自定义配置
中若再次等价，应如实解释，而非宣称算法收益。

SPD 保证投影唯一。活动约束集合 I 固定时，理论解为：

```text
v = u - W^-1 A_I^T (A_I W^-1 A_I^T)^-1 (A_I u-b_I)
lambda_I >= 0,  A v <= b
```

三维法锥可由至多三个独立活动法向表示，故枚举 1/2/3 行即足够覆盖最优解。
实现对 W 做 Cholesky 坐标变换，并用 QR 解活动面问题，不形成会平方条件数的正规方程。
候选须同时满足原始可行性与非负乘子；再在所有可行候选中取最小目标。
独立校验为所有顶点上的变分不等式
`(v-u)^T W (p-v)>=0`，并与另一种数值算法 SLSQP 交叉核对。
该枚举适合少量轮子的离线参考，复杂度随约束数增加；不是对 MPPI 大量 rollout 的优化实现。

### 边界行为与控制含义

box clipping 与固定 SPD 多面体投影为连续、分片仿射映射；活动面切换时导数可变。
metric projection 在相应 W 范数下 firmly nonexpansive，因而非扩张；不能把这个保证
无条件转为另一个单位/权重的范数。ray 映射连续且幂等，活动面 a 固定的域外导数是：

```text
F(u)=b u/(a^T u)
dF/du=b/(a^T u) [I-u a^T/(a^T u)]
```

其增益可能大于 1。零命令不变、幂等、无越限都不等于加速度/jerk 连续或闭环稳定。
边界实验用随机射线、全部顶点、零点，在 `1e-3,1e-6,1e-9` 的尺度扰动下量化局部增益、
幂等与越限；最小步长的差分受浮点误差影响，不据此声称解析导数连续。

### 误差敏感性：从 J 的导数到有限误差保证

记 `k=sign*ratio/r`、`d=(cosθ,sinθ)`、`g=[dx,dy,x*dy-y*dx]`，J_i=k g。
局部解析导数为：

```text
dJ/dr = -J/r
dJ/dx = k [0,0,dy],       dJ/dy = k [0,0,-dx]
dJ/dθ = k [-dy,dx,x*dx+y*dy]
d(A_i/b_i)/db_i = -A_i/b_i²
```

因此轮位误差主要通过 yaw lever 影响轮速，方向误差同时改变平移与旋转项；
半径与速度上限误差会直接缩放归一化轮级约束。有限误差不能只靠一阶导数保证：
设半径相对误差 <=er<1、每个位置分量误差 <=ep、角误差 <=ea<=π、
每侧轮速容量相对误差 <=es<1。令 `a=2sin(ea/2)`，则：

```text
||d_actual-d|| <= a
|lever_actual-lever| <= ||p|| a + sqrt(2) ep
|omega_actual-omega_nominal| <= E(v)
E(v) = [er |omega_nominal| + |k| (a ||v_xy|| + (||p|| a + sqrt(2) ep)|wz|)]/(1-er)
```

对上下限两侧均有充分条件
`(A_i v+E_i(v)) / [(1-es)b_i] <=1`。
它使用有限角差的界、三角不等式和最坏半径分母，不假设误差服从高斯，也不依赖“足够小”。
E 是一次齐次函数，因此可按该上界做保守的 robust ray 缩放；这只是误差预算研究对照。
预算来源若未经实车确认，保证仅属于声明的数学误差集合，不能提升为实车安全裕量。
随机扰动验证只检查实现与证明是否一致，不替代上述最坏情况推导。
还有不依赖随机数的 sanity case：若所有轮半径或轮速容量统一乘 0.99，则
`P_actual=0.99 P_nominal`，体积乘 `0.99³`；原边界点的实际利用率为 `1/0.99`，
确定性越限约 1.01%。所以“nominal 精确投影”本身不会产生参数裕量。

### 近奇异 J、配置检测与不可检测项

保留原始物理 J 和 LS 逆，不截断弱方向来伪装正常底盘。几何构造使用
`N=diag(1/drive_capacity) J S=U Σ V^T` 的白化坐标
`v=S V Σ^-1 y`，再变回物理顶点/体积；不用 Qhull jitter 改动物理域。
坐标白化可改善顶点计算，但 `||δ(S^-1 v)||` 的最坏放大仍约为
`||δ(normalized_drive)||/sigma_min(N)`。

两族退化实验分别缩小全部 yaw lever、将滚动方向逐步变为平行；
epsilon 从 1 到 1e-14，并含精确零。记录 float64 几何/逆的接受或拒绝、奇异值、
噪声放大，以及明确使用 float32 精度阈值的数值秩实验。float32 是数值对照，
不假设新车 MCU 的计算类型。不能从“轮速限制下出现巨大理论 yaw 上限”推断车辆能高速自旋。

可自动拒绝的内容包括：缺失/非数值/布尔或数字字符串、非有限数、非正半径/传动比、
无效正负转、非单位滚动向量、非法轮速区间、重复 ID、重复 JSON key、错字段/坐标系、
非法数值尺度、秩不足和超出本工具数值解析能力的布局。schema 错误与
`NumericalGeometryError.diagnostics` 分开；后者提供奇异值、条件数与逆一致性误差。

不能仅凭 JSON 自动识别“数值合理但填错”的符号、轮序映射、轴向/滚动向量混淆、
RPM 冒充 rad/s、错误参考原点或真实装配角。没有机械/固件来源时，不设置虚构的新车尺寸或
半径合理范围来声称已做物理验证；没有质量/轮宽/机构信息也不能验收机械可实现性。

### TinyMPC 与 MPPI 候选接口的接入边界

`tinympc_candidate(model, variable, mapping, offset, margin)` 生成单组未知变量的
`Cz<=d`，保留上下轮级行名和原始 drive normalizer。
`v_body=M z+c` 时 `C=A M,d=(1-margin)b-A c`。offset 拉回后的 d 可以为零或负，
故误差归一化仍用原 b，不能再除以新 d。返回 state 或 input 的逻辑参数四元组，
对应[官方线性约束 API](https://github.com/TinyMPC/tinympc-python#constraints-api)。
这是候选数据接口：本轮没有安装、调用或验收该 solver；None/空参数、矩阵存储、
horizon 广播和版本能力须在未来适配时固定并实测。

只有 yaw 为已知参数时，world→body 可作为给定 M；yaw 也是未知量时不再是固定线性约束。
如果 v 同时依赖未知 state 和未知 input，不能把同一耦合不等式拆成两组独立限制；
应在选定模型/变量后另行表达。速度、加速度、力矩或轮速输入的差别不能由适配器猜测。
solver 有可行容差，执行端还会做限幅/平滑/饱和，最终命令必须按同一配置重查。

`CommandProjector.project(commands)` 接受 `(...,3)` 的 body 速度，构造时完成 box/活动面
预计算。weighted 必须显式给 SPD W；不存在数值证书时抛异常，不静默替换成零或另一种方法。
传入预计算 box 时重新检查其有限非空、轮级可行和包含零；预计算 box 的最大性由调用方的
maximum_box 证书负责，不因接口接受它就自动获得最大性证明。
它没有 authority、deadline、watchdog、stop 或 fallback 所有权，仍是 MPPI 离线候选。

MPPI 若改变 rollout 中的命令，必须使用改变后的命令重新传播动态模型和评分。
只投影最后发出的首条命令，会让实际执行与优化的 trajectory/cost 不一致。
投影后的高斯噪声一般不再是原高斯，且会在边界产生概率质量；不能直接声称原有
importance weighting 或更新公式仍等价。固定同一 body P 内的命令非负加权平均仍可行，
但不同 yaw/frame/时刻的域不能混在一起平均。上述接口不意味着已解决 MPPI 采样分布或闭环控制。

### 第二阶段复现与结果

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 -m unittest discover -s experiments/omni_chassis -p 'test_*.py' -v
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 experiments/omni_chassis/stage2.py --plot
```

可用 `--config` 指定实测待研究文件、`--protocol` 更改显式实验协议、
`--sections comparison sensitivity conditioning validation regression` 选择独立部分；
conditioning/regression 的 synthetic_symmetric4 仅作合成构型模板，不从自定义输入猜实际车辆。
输出位于忽略的 `outputs/stage2/results.json`，每完成一个部分就保存；失败样本有 seed/index
与必要配置，不排除失败后重算漂亮指标。保存一份精简第二阶段证据，原始样本/过程日志不入版本资产。

关键证据为 [`stage2_evidence.json`](../../experiments/omni_chassis/stage2_evidence.json)。
图可由 `--plot` 重新生成到 `outputs/stage2/comparison.png`。

**最大 box 的结果**如下。表中最大值使用“中心可偏移但必须包含零”，不是仅原点居中版本。

| 合成配置 | 同形 conservative box/P | 最大零包含 box/P | 原点居中最大 box/P |
| --- | ---: | ---: | ---: |
| symmetric4 | 22.22% | 22.22% | 22.22% |
| asymmetric4 | 17.60% | 18.57% | 15.37% |
| three | 18.39% | 22.38% | 21.26% |
| six | 25.60% | 29.63% | 29.63% |

**六方法对照的代表例**为 symmetric4，3,000 个共同命令；完整四配置数据在证据中。
这一阶段的越限率是“共同原始命令经映射后的输出越限率”，与第一阶段在 box 内
条件采样的误接受率定义不同。不能直接比较两阶段百分比。

| 方法 | 输出越限率 | mean scaled distortion | mean weighted distortion | 保留可行体积 | 单命令 median / p99，μs |
| --- | ---: | ---: | ---: | ---: | ---: |
| naive axis clip | 91.33% | 0.093 | 0.194 | 100%* | 13.9 / 18.8 |
| conservative box | 0 | 1.007 | 2.231 | 22.22% | 13.9 / 18.3 |
| maximum box | 0 | 1.007 | 2.231 | 22.22% | 14.1 / 17.9 |
| ray scaling | 0 | 0.786 | 1.833 | 100% | 47.3 / 71.2 |
| Euclidean projection | 0 | 0.821 | 1.038 | 100% | 1,718 / 1,786 |
| weighted projection | 0 | 0.886 | 0.940 | 100% | 1,715 / 1,828 |

*naive box 在此布局包住 P，故保持全部原本可行命令，但它同时输出大量 P 外命令；
100% 保留体积和低失真不代表方法可行。asymmetric4/three 的 naive 保留体积也只有
99.39%/96.73%，说明截距 box 还可能误删可行命令。可行方法的最大相对越限仅在浮点量级。

weighted 将该例的平均 `|Δwz|` 从 Euclidean 的 0.646 降至 0.265 rad/s，代价是
x/y 平均误差分别从 1.101/1.077 增至 1.199/1.170 m/s；这是显式权重的取舍，不是普遍更好。
ray 的 scaled distortion 低于 raw Euclidean 也不矛盾：优化范数不同。
six 的 maximum box 虽比 conservative box 体积大，mean scaled distortion 却从
0.806 增至 0.826，说明最大几何容量不能替代当前命令分布上的失真目标。

runtime 上，box construction/QR 预计算另存 setup 指标，不计入上述 per-command。
三轮、四轮、六轮 QP 参考的单命令 median 约为 0.77、1.7、5.8 ms；
64 命令 batch 均摊约 27、49、139 μs/command。不能把后者当作单命令延迟。
当前枚举参考不具备“直接用于 MPPI 大量 horizon rollout”的性能结论。

**边界结果**：零命令、幂等性、原本可行点保持以及可行方法的越限检查均通过。
`1e-6` 尺度扰动下 ray 的最大观测局部增益在四配置中约为 2.05–3.08；
Euclidean/weighted 在各自范数中约为 1，最小步长的超出 1 部分约为浮点差分误差。
这支持前述连续但不必非扩张的数学区别，不能据此推导闭环稳定性。

**误差研究**共 960 个扰动布局。对 combined 最小预算（半径/容量 1%、每位置分量 1 mm、
角度 0.5°），四配置中 nominal weighted 输出的最坏相对越限约 2.28%–2.49%；
最大预算（10%、10 mm、5°）时约 23.3%–29.0%。这些输入一半来自 0.9 倍名义边界、
一半来自 1.5 倍名义边界后映射，不是现场命令频率。相应体积变化不足以判断边界可靠性：
小角度扰动下体积变化很小，仍可让部分原边界命令越限。
有限误差 utilization 上界没有被任何扰动反例超过，按该界生成的 robust ray 输出零越限。
鲁棒性依据仍是误差集合和推导，不是 960 次抽样的成功率，也不覆盖模型外的滑动/动力学。

**数值退化结果**：两族 epsilon=1e-6 仍可生成几何/投影，但 `sigma_min(N)` 已约
1.4e-6/1.6e-6。仅 1e-6 的归一化驱动误差即可产生约 0.71/0.64 的最坏尺度化速度误差。
epsilon=1e-8/1e-9 的 LS 逆一致性未通过，明确拒绝；1e-10 以下和精确退化按相对秩门拒绝。
float32 对照在 1e-8 已报告秩 2。该拒绝边界是本实现/精度/尺度的数值结果，不是通用硬件阈值。

**大规模回归与检测**：17 个数学/接口测试通过；1,000 个独立随机布局、64,000 个共同输入、
320 个独立 SLSQP 查询通过，随机布局条件数最大约 3.21e3。
最大相对轮级越限 `4.80e-13`、变分最优性残差 `8.07e-13`、参考 metric 距离 `1.06e-6`、
最大 box 对偶 log-volume gap `1.93e-7`，均低于预设门。
全部最大零包含 box 均不小于同形可行竞争者（浮点容差内）。仍有 1 个 box 的 solver 标志
非成功，但其角点可行与对偶 gap 通过，单独计数保留；没有把该标志改成成功。
29 个明确错误配置全部拒绝；数值合理的单位/映射错误仍需外部真值确认。

**决策**：Continue 保留最大 box、半空间和投影研究候选；raw Euclidean 作为单位依赖参考，
weighted 的尺度/优先级应随未来模型明确，不能直接当正式 controller 参数。
QP 枚举实现保留作离线 oracle，MPPI 真正采用前还需另行研究 batch/horizon 性能、
采样分布与 rollout 一致性；TinyMPC 接入前需确认实际版本/变量定义及 solver 残差。
这些都是接入边界，不在本轮自动启动 Integration。**不 merge main，不声称 Hardware Go**。

原 `/tmp` 工作区曾被环境清理；源码已从本聊天的完整 patch 记录恢复到持久目录，
第一阶段 400,000 样本和第二阶段结果均重新验证，没有把原临时目录的丢失文件当作已保留资产。

## 最终冻结复核与重启条件

**冻结判定：FROZEN_WAIT_CHASSIS_PARAMETERS。** Stage 1 / Stage 2 支持在其数学假设、
输入格式与测试域内保留这个离线候选；新车真实 J、轮速/加速度限制和执行跟踪仍未知。
本次收口仅补充状态、接口说明、重启清单和证据整理，没有新增投影/优化方法。
独立分支为 `experiment/omni-chassis-feasible-region`，代码、配置、测试、本页和两份 evidence
随同一冻结提交保留；commit SHA 由 Git 及收口报告记录，不在提交正文中自引用。

### 复核结果

| 范围 | 核对依据 | 判定 |
| --- | --- | --- |
| 参数化 J / K、轮序、符号、半径、传动比与轮速界 | 独立刚体点速度、参数变更及轮序置换测试 | 通过；通用接口没有 synthetic 轮组默认值 |
| Av<=b、顶点与体积 | 解析八面体、104 个 LP 支撑方向、Stage 1 轮级判定 | 通过；满秩、有限且包含零的轮速区间是适用条件 |
| 三种中心策略的最大 box | 角点可行、对偶上界、原点约束及 free box 排除零的反例 | 通过；不把 solver 非成功标志改成成功 |
| ray / Euclidean / weighted | 可行性、固定点、幂等、变分不等式、320 次 SLSQP 对照 | 通过；Python QP 明确是 reference/oracle |
| TinyMPC / MPPI 候选接口 | affine state/input、margin、批量形状、非法 metric/box 检查 | 通过；没有正式 solver 或导航实现接入 |
| sensitivity / degeneracy / validation | 有限误差推导、960 个扰动布局、两族退化、29 个错误/不适用配置 | 通过；不等于真实误差预算或机械校准 |
| 当前完整测试 | `unittest discover` 的全部 17 项 | 通过 |
| 文档 / 配置 / evidence | 完整协议重跑，逐项比对保留证据的确定性字段 | 一致；runtime/setup/elapsed 等计时字段会自然波动 |

固定 fixtures 是 3/4/4/6 轮，包含非对称半径、位置、方向、减速比、符号与正负容量。
小型随机测试另外覆盖 3/4/6/8 轮；完整 1,000 布局回归的实际轮数分布为
3轮171、4轮150、5轮173、6轮181、7轮166、8轮159，五种子均零失败。
未把研究配置、MCU pending 模板、浮点实验或 synthetic 回归范围写成实车事实。

冻结重跑确认：Stage 1 为 400,000 样本、8,250 边界点；Stage 2 为 12,000 个固定布局
共同输入、960 扰动布局、20 个退化配置、1,000 随机布局/64,000 个共同输入及 320 次独立
投影对照。确定性结果与收口前 evidence 完全一致，计时表更新为本次重跑记录。
保留的一个 box solver 非成功标志仍由原始可行性和对偶证书单独验收，未被忽略或删样本。

### 最小长期资产与复现

保留 `model.py`、`optimization.py`、`uncertainty.py`、两个 runner、全部 `test_*.py`、
四个 synthetic 配置、未填参数的 MCU 模板、Stage 2 协议及 `evidence.json` /
`stage2_evidence.json`。保留 `COLCON_IGNORE`，不作为 ROS package 或安装后的控制器。
`outputs/` 中的 smoke、重复完整结果、生成矩阵/图片以及 `__pycache__/` 在收口后删除，
它们均来自可复现合成输入，不含唯一实测原始数据。

验证环境为 Python 3.12.3、NumPy 1.26.4、SciPy 1.11.4，Linux x86_64、BLAS 单线程；
绘图额外使用项目已有 Matplotlib。版本来自 evidence，不声称其他依赖组合已验证。
证据是在冻结 commit 之前的工作树上生成，Stage 1 的 base_revision/dirty 字段如实保留；
不把它伪写为提交后 clean 运行。完整源代码随冻结提交提供，runtime 不是实车实时性验收。

从仓库根目录复现，无需 ROS、MCU 或 TinyMPC 安装：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 -m unittest discover -s experiments/omni_chassis -p 'test_*.py' -v
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 experiments/omni_chassis/run.py
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 experiments/omni_chassis/stage2.py
```

预期为 17 项测试 OK、四个 Stage 1 配置 PASS、Stage 2 五种子零失败和总 gate PASS。
加 `--plot` 可重建图。重新生成的输出不覆盖两份保留证据；不要要求计时百分位逐位相同。

### 真实底盘参数到位后的最小验证清单

以下是重启 Research 的最小信息与验证项，本轮不执行实车动作，不据此授予运动权限。
实际阈值和测量误差须由机械/电控及实测记录确定，不能照搬 synthetic 数字。

| 项目 | 最小确认与核验 |
| --- | --- |
| wheel geometry | 确认实际轮数、ID、相对同一 body 原点的各轮 xy 与安装角，单位 m；确认固定轴、平面理想 omni 假设适用。若滚子角/机构属于不同模型，不能强行套用本 J |
| wheel radius | 逐轮测量工作负载下有效滚动半径及误差，填 `radius_m`，不把 CAD 外径或合成半径当作标定 |
| roller / drive direction | 区分滚子自由方向、轮轴和受驱动滚动方向；确认配置单位向量与正向轮转动一致，并核对正反转，不靠左右轮惯例猜符号 |
| gear ratio | 确认 `drive_per_wheel_ratio` 是驱动角速度/轮角速度的正比值；明确反馈端点、RPM/rad/s 换算和传动比是否已由固件处理，防止重复换算 |
| MCU wheel ordering / sign | 固定固件版本、驱动 ID→配置数组的映射，逐轮核对正负方向；反馈须带单位、时间戳或采样周期、序列与有效性，检查乱序/缺轮不被当作新轮序 |
| wheel speed / acceleration limits | 记录逐轮正负速度上限、加减速度上限及电压/功率/模式依赖。速度界可填当前 schema；**轮加速度不在当前静态 polytope/schema 的保证范围**，须另存实测记录并检查时间连续执行，不能宣称本工具已验证它 |
| measured chassis command ↔ wheel speed consistency | 对齐 MCU 实际消费命令、最后一级饱和/平滑后的命令与轮反馈的时基；用零、正反纯 vx/vy/wz 及少量混合命令比对 Jv、反馈轮速、Kω 与相容性残差。若有独立 chassis 速度测量一并核对，不把 Kω 当独立真值 |

填完 pending 模板并记录固件/机械来源后，先仅重跑与该参数集有关的离线计算：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 experiments/omni_chassis/run.py --config /path/to/measured_chassis.json \
  --output experiments/omni_chassis/outputs/measured_stage1
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 experiments/omni_chassis/stage2.py --config /path/to/measured_chassis.json \
  --sections comparison sensitivity --output experiments/omni_chassis/outputs/measured_stage2
```

这些命令不导入 MCU 数据流，也不读取真实串口；若要核验测量记录，用现有 J/K/residual
接口做离线比对，实测加速度与动态执行另行判断。记录实际适用条件、条件数、误差预算和
轮级一致性后再决定是否重开 Research；导航集成、正式 TinyMPC/MPPI 与 Hardware Go 均不在本冻结判定内。
