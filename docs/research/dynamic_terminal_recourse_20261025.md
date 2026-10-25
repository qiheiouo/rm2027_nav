# Terminal recourse 固定反例研究

Research；基线为 `21b9e56076b23f17f1140b9ed68bff5edce9b5b6`。本轮从该提交建立
`codex/dynamic-terminal-recourse`，复用已冻结 harness；旧搜索 Stop 不改写。

## 定义与立项门

精确 viability 要求：从时间状态 `(x,y,vx,vy,t)` 出发，存在满足执行队列、输入约束的
后续控制，使所有未来时刻无碰撞。这通常不可低成本计算。本轮只计算一个充分条件
`R_B(s,t)`：存在有限备份库中的动作，沿途通过原 checker，最终静止，并与当前观测
的所有障碍 CV 射线保持安全间距。它是 **CV 模型条件下的可执行证书**；失败是未知，
不代表物理必撞。未知加速度、遮挡、感知误差不在证书范围。

此前“停车保持最终被撞”不证明早期停车状态已经没有安全后续动作。本轮分别报告
unsafe hold 与 backup existence，禁止为了支持 hypothesis 把两者混为一谈。
这也符合 [Inevitable Collision States 的定义](https://doi.org/10.1163/1568553042674662)：
必须对所有未来控制都无解，才是不可避免碰撞；有限候选库失败没有该含义。

| 候选近似 | 本轮取舍 |
|---|---|
| 一步 continuation / 短树 | 排除：仅把安全边界向后移动，未定义后续安全终点 |
| terminal TTC / lateral escape 时间裕度 | 排除：便宜但多障碍、墙与加速度耦合下不能给出可执行路径 |
| 应急 rollout → 刹停 → CV 安全驻留 | **选中**：固定小库、有完整执行证据，驻留段用解析最小距离检查 |
| 多层 continuation tree | 排除：同时改变搜索覆盖与预算，难以区分终端语义与更多搜索量 |

仅改变保留条件和备份证书；beam=64、2 秒时域、6000 expansion 上限及 cost 不变。
每层按原 cost 排序后，先检查 continuation，再做原有合并与 64 个节点保留。
备份为刹停或八个方向的 0.5 m/s 目标持续 1.4 s 后刹停；1.4 s 是两个最大速度斜坡
时间 `2*.5/.8` 向 0.2 s 网格上取整。该小库新增固定备份控制，不声称只改了终端评分。
所有 rollout 复用原积分、限幅和碰撞检查。驻留检查只使用本次观测位置与速度；不读取
scene、seed、未来真值或手工见证。负结果不通过调参数挽救。

门槛在执行前写入 [recourse_protocol.json](../../research/dynamic_state_time/recourse_protocol.json)。
先测试 t=2.50 s 的保存状态、排队命令和前一序列；识别 unsafe hold、保留原见证或等价动作，
并从该状态闭环执行到原任务终止规则。要求零碰撞、净空至少 .05 m、完成任务，p99≤40 ms、
max≤50 ms、无 deadline miss。一次不改参数的复核仍失败即 Stop，不展开 36 次。

只有该门通过，才执行原四场景×三相位×A/B/C 的 36 次对照。A=simple、B=旧 search、
C=search+recourse；不是 STVL A/B。完整条件、预先定义的退化阈值及未标注指标见 protocol。

## 固定反例结果与裁决

**Stop 当前“每层 beam 保留前做备份 rollout”的实现。** 固定反例两次都出现安全改善，
但两次都未通过预登记计算门。没有展开 36 次 A/B/C，也未开始新场景或调参。
这不是 terminal recourse 理念被反证，也不是当前 finite search 获得 Go。

固定 t=2.50 s 输入中，B 仍为 `viable=0, reason=reused`。新程序的 B 模式与旧提交
独立编译的程序在 command、verified、prefix、viable、expanded、schedule 上逐字段一致。
C 返回一个新序列，执行全部承诺前缀、两秒主序列、备份和刹停后，最小物理净空
**0.069342 m**，终点 `(1.63565, .66918, 0, 0)` 获得 CV 驻留证书。
原 **0.785078 m** 见证也通过驻留检查；没有把该手工序列作为消费者输入。
这是找到等价安全动作，不是保留了相同见证或相同净空。

新计算首次调用检查 1299 个 continuation，拒绝 732 个，展开 4536 个原搜索动作；
主时域、代价、宽度、动作 expansion 上限没有改变。两次该输入调用分别为
68.56 / 70.29 ms，已经超过原 50 ms 时隙。新增备份 rollout **确实增加了总计算量**，
不是免费增加终端语义；没有通过增加 CPU/时隙或放宽预算接受它。

闭环都从保存的 **绝对 t=2.50 s**、机器人状态、既有排队命令与上条序列开始。
使用原 40 s 绝对超时、到达后 3 s 观察、同一 plant/CV/碰撞检查/速度加速度限制。
`arrival` 是原场景绝对时间，减去 2.50 s 才是本次从 fixture 开始的行驶时间。

| 指标 | B 原 search，初试 / 复核 | C recourse，初试 / 复核 |
|---|---|---|
| 碰撞 | 是 / 是，均 t=6.82 s | 否 / 否 |
| 安全完成 | 否 / 否 | 是 / 是 |
| 最小物理净空 m | -.001567 / -.001567 | .068216 / .068098 |
| 到达绝对时间 s | 未到达 / 未到达 | 13.00 / 12.59 |
| 从 fixture 到达用时 s | 无 / 无 | 10.50 / 10.09 |
| 实际等待 s（含目标后观察） | 3.34 / 3.34 | 4.46 / 3.34 |
| stop count | 1 / 1 | 2 / 2 |
| 命令符号翻转 | 0 / 0 | 13 / 11 |
| 原 .5 s/.15 m trajectory churn | 0 / 0 | 0 / 0 |
| 对齐未来时刻的目标速度差 RMS，mean / max，m/s | .0061/.4357；.0061/.4357 | .0657/.3884；.0640/.3923 |
| 调用 mean / p50 / p95 / p99 / max，ms | 7.26/7.35/11.50/11.75/12.14；7.27/7.41/11.50/11.83/12.24 | 24.90/15.66/72.04/76.77/78.19；24.78/15.79/73.71/77.16/78.32 |
| deadline miss / calls | 0/87；0/87 | **43/252（17.06%）；39/244（15.98%）** |
| effective update rate Hz | 20.14 / 20.14¹ | 18.65 / 18.61 |
| 消费者 CPU 按模拟时长折算，核 | .125 / .123 | .443 / .440 |
| 消费者 peak RSS MiB | 4.26 / 4.18 | 4.38 / 4.30 |

¹ B 的最后一个执行周期被碰撞截断，cycles/duration 略大于名义 20 Hz。
`unnecessary_stop` 和 `deadlock` 仍为 null：没有逐状态反事实标签，不以等待或 null
冒充误停/死锁判断。B 因碰撞提前截断，所以它的低等待、低振荡不能作性能优势。
全部分布（含 p05）和四个逐次结果见
[summary.json](../../research/dynamic_state_time/results/recourse/summary.json) 与
[trials.csv](../../research/dynamic_state_time/results/recourse/trials.csv)。两次是相同反例的
重复，不是两个独立场景，不能用 2/2 安全完成声称稳定跨工况修复。

## 失败机制复核

- 原轨迹 t≈3.55 s 已静止在 `(1.39757,-.00552)`；驻留检查失败，但侧向备份仍可行。
  因此原策略过早放弃后续动作，不等于它此时已经进入物理必撞状态。
- t≈6.30 s，同一停车位置下障碍更近，备份库不再找到 continuation；这仍不是完整
  viability kernel 的不可行证明。时间本身必须属于状态，单看停车位置不够。
- 6 个解析驻留检查覆盖远期迎面碰撞、远离、侧向通过、相切及矩形圆角距离；均通过。
  原见证的独立 Python 执行重现 .785078 m；新候选整条执行重放也通过。
- 最坏调用为 78.32 ms，其中消费者内部 75.62 ms：展开 4698 个节点、做 2094 次
  continuation 检查、1509 次失败。首轮对应 78.19 / 75.69 ms、2104 次检查。
  计时包含同样的输入/输出路径，内部耗时已解释大部分超限，非日志写入或传输假象。
- 两轮消费者/plant 无代码或参数变化；复核只增加旧早/晚停车状态的诊断输出。
  CPU 固定一个 logical core，GCC 13.3.0 `-O3 -DNDEBUG`，已有本机 nlohmann/json，
  未安装依赖；本轮无需 Nav2，因此使用本机独立进程，A/B 均重新用相同工具链编译。
- 超时命令按旧 harness 规则**被拒绝**并排队制动；没有忽略超时仍执行安全轨迹。
  两次 C 的不同到达时间正体现真实调用耗时参与执行。这意味着当前安全改善混合了
  continuation 选择与超时制动行为，不能单独归因于 terminal certificate。

上轮计算预算不是失败原因；**本轮新增逐节点检查使计算预算成为新的失败原因**。
安全信号值得保留，当前实现却没跨过最小立项门。按照预登记停止，不缩小检查库、
降低安全裕度、调权重或扩大预算挽救结果，也不运行条件性 36 次对照。

## 保留与复现

实现：`search_consumer.cpp` 的 recourse 模式；原 simple/search 算法仍可调用。
`run.py` 仅增加 fixture 初始状态和 aligned schedule RMS 统计。
固定反例、原 summary、原 manifest 不改动。新数据集
`rm2027-terminal-recourse-fixture-20261025` 共 143552 bytes 原始证据，位于仓库外；
[manifest](../../research/dynamic_state_time/results/recourse/dataset_manifest.json) 记录大小、
SHA256、路径、生成来源、用途和所需实验。构建缓存及旧数据不复制进 Git。

```bash
export DST_DATA_ROOT=/tmp/rm2027_terminal_recourse_reproduce
bash research/dynamic_state_time/run_recourse.sh fixture_01
# 同算法/参数最小复核：
bash research/dynamic_state_time/run_recourse.sh fixture_confirmation
python3 research/dynamic_state_time/summarize_recourse.py \
  --data "$DST_DATA_ROOT" --output /tmp/rm2027_terminal_recourse_summary
```

期望 B 复现 t≈6.82 s 碰撞；C 在本机两次无碰撞，但计算门失败。
耗时依机器变化，复现时保留 deadline rejection，不能把忽略时延的重放当闭环结果。

本轮结论限定固定理想 CV 反例。CV 永久不变是驻留证书的模型假设，不是读取未来真值，
也不适用于真实 RoboMaster 突变运动。没有 STVL A/B、跨场景或硬件接受结论。

**是否继续：停止当前把 rollout 嵌入每层 beam 的路线。** recourse 的问题值得保留；
以后只有提出能在原时隙内保留并及时执行备份的新消费结构，才值得重新立项。
不自动启动该结构，也没有证据宣布所有 finite search 都不适合。

归档 branch 为 `codex/dynamic-terminal-recourse`，研究基线为 `21b9e56076b2`；
本轮轻量源码、证据与报告由本次归档提交保存。未合入 main，正式导航未修改。
