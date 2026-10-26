# Persistent Backup / Safety Reserve：固定反例初始化负结果

Research。基线为 `codex/dynamic-terminal-recourse @ 0ceb8e060f381bec81a895b9de40bf56109064c8`；
原 finite search 来自 `21b9e56076b23f17f1140b9ed68bff5edce9b5b6`。
本轮分支 `codex/dynamic-persistent-backup`，未修改 production navigation、main 或归档源码。

**Stop 当前最小 prototype，不进入 36-episode。** 固定 fixture 的实际运动初态下，
冻结的九个备份候选无法建立第一条合格 safe-rest 证书。C1 如实返回 unavailable，
未启动机器人 rollout；不能把它记为零碰撞成功，也不能声称持久备份机制已经通过或被反证。
一次实现核查和一次不改算法/参数复核后结果一致。没有扩大动作库、搜索宽度或时域。

## Hypothesis 与实现边界

执行器跨周期持有一条可执行备份。只有“不可撤销队列＋下一拍主动作＋备份＋安全驻留”
整体通过，才能替换当前承诺；规划失败/迟到不能自动丢弃仍有效的备份。

原 `search_consumer.cpp`、`world.py`、fixture、速度/加速度限制和碰撞检查均未修改。
新 `reserve_consumer.cpp` 引用归档实现中的 plant、checker 和九动作备份库，
在提交处运行，不对搜索树节点调用 recourse。所有臂直接使用独立编译的旧提交 search
作为提案源；固定输入 command、verified、prefix、viable、expanded、schedule 与当前归档
search 逐字段相等，均为 `reason=reused, viable=0`。

初始时刻为绝对 t=2.50 s，初态约 `(1.041617, .008954, .499200, -.028281)`。
保留原已排队命令，命令延迟 .10 s、控制周期 .05 s、plant 步长 .01 s、
vmax=.5 m/s、amax=.8 m/s²；原 40 s 绝对任务超时、.15 m 目标范围、到达后 3 s 观察不变。
消费者只获得当前测量状态、实际不可撤销队列和原 source-age 修正的理想 CV 输入。
场景真值仅用于碰撞/任务评价。不是正式 STVL A/B，也没有实机结论。

## 状态机与 C0/C1 的精确差异

实现的状态路径：

`INITIALIZING → HOLDING → MAIN_COMMITTED / BACKUP_EXECUTING → SAFE_REST`

任何无初始证书的路径进入 `UNAVAILABLE`；已有证书适用条件失败进入 `INVALID`。
新提案失败/迟到只拒绝该提案；C1 保留有效旧计划，按原绝对时刻选择下一条要入队的命令。
进入真实命令队列的内容不撤回。新提案只有在同一实测起点和实际队列下通过整体证书，
才替换软件层计划。接入旧备份时，保留其剩余事件的绝对时间并重新检查连接段，
不把整条旧备份向后平移。若无法接入，才尝试现有九候选。

证书实际存储：绝对起点、起始状态、初始命令、不可撤销队列及版本、完整控制事件、
备份事件、切换时刻、执行游标、参考状态、预测源时刻/版本/障碍状态、适用条件、
驻留时刻/状态及 CV 射线净空。不是单个安全布尔值。

每周期检查实际状态与参考、实际命令与队列、障碍集合及 CV 外推一致性、源年龄和地图边界。
1e-7 仅作理想模型的数值误差容差，不是实车误差包络；最大源年龄 .15 s。
证书失效不得继续声称安全。

| 臂 | 提交处检查 | 备份持有及执行 | 未通过/迟到 |
|---|---|---|---|
| B | 不新增 | 原有主序列复用，仅入队当前命令 | 原超时制动语义 |
| C0 | 与 C1 相同 checker 和动作库 | 证书用于本次主动作验收后丢弃 | 未认证制动；不承诺安全 |
| C1 | 同一 checker，可重验接入已持有备份 | 由执行器持有并消费剩余绝对时间序列 | 有效旧备份继续执行；没有初始证书则不启动 |

C0 在隔离仿真中允许未认证制动，正是“不持有备份”的消融条件；不能将这种启动方式
搬给 C1 来绕过初始化要求。C0 的旧备份为空，因此不能利用 C1 才有的旧备份接入路径。

**本轮 C1 实际仅走到 `INITIALIZING → UNAVAILABLE`。** HOLDING、替换、游标推进、
失效撤销与迟到期间执行分支已写入最小 prototype，但没有获得闭环运行验收。

## 先失败的初始化门

备份库保持上一轮配置：刹停，或八个方向的 .5 m/s 目标持续 1.4 s，再刹停，最后解析检查
静止 footprint 与当前观测的所有 CV 射线是否保持净空。先执行同一 .10 s 不可撤销前缀。

第一次预检错误地断言“应当存在初始备份”。断言失败后，核查并未调整算法：
直接调用归档程序的 `recourse_probe`，在完全相同前缀末态也返回 `backup_exists=false`。
随后把预检改为记录合法的 unavailable 结果，逐候选独立执行同一 Python plant 复核。

| 候选 | 最终 y / m | CV 驻留最近物理净空 / m | 证书 |
|---|---:|---:|---|
| brake | -.002712 | -.597288 | 不通过 |
| 负 x、负 y | -.413796 | -.186204 | 不通过 |
| 负 x | -.011542 | -.588458 | 不通过 |
| 负 x、正 y | .388615 | -.211385 | 不通过 |
| 负 y | **-.647640** | **.047640** | 不通过 |
| 正 y | .622401 | .022401 | 不通过 |
| 正 x、负 y | -.526879 | -.073121 | 不通过 |
| 正 x | .005616 | -.594384 | 不通过 |
| 正 x、正 y | .514063 | -.085937 | 不通过 |

这里的远期最近距离是从观测状态解析外推的 CV 条件结果，不读取未来真值。
最接近通过的负 y 候选，其 padded-box CV 射线净空为 .017640 m，低于原 checker 的
.024750 m 门；物理净空 .047640 m 也低于本轮 .05 m 要求。不能降低余量使其通过。

这不等于初态无安全动作：旧 .785 m 见证仍然存在，而它并非本次冻结的九候选之一。
上一轮每节点方法从两秒主序列的末端查询备份；本轮从当前运动状态加不可撤销前缀查询。
两种起点不同，前者曾能找到证书，不保证后者能初始化。

## 正常与迟到对照

预登记见 [reserve_protocol.json](../../research/dynamic_state_time/reserve_protocol.json)。
迟到条件为每六个绝对控制时隙中的第六个将响应可用时间置为至少 80 ms；
不绑定障碍位置或 3.55/6.30 s。busy planner 跳过后续时隙，三组规则一致。
C1 设计上仍由独立逻辑执行器每 .05 s 推进计划；这是离散事件模拟，不是 OS 调度证明。
注入模拟结果迟到/worker 不可用，不能把它当额外 CPU 工作。

B 在正常条件下保持旧命令行为及 t=6.82 碰撞；迟到条件使用公共的 20 Hz 时隙及 busy
跳拍规则，属于本轮延迟实验适配，不声称与旧变长循环的注入轨迹逐字节相同。
两轮各运行 B/C0 的四条固定反例轨迹，共八条；C1 两种条件各做两次初始化后停止。
这不是八个独立场景。C1 没有真正暴露于 lateness，不能宣称通过了迟到测试。

| 条件 | 臂 | 碰撞 初试/复核 | 碰撞绝对时刻 s | 最小物理净空 m | 完成 |
|---|---|---|---:|---:|---|
| normal | B | 是/是 | 6.82 / 6.82 | -.001567 / -.001567 | 否/否 |
| normal | C0 | 是/是 | 6.75 / 6.75 | -.003282 / -.003282 | 否/否 |
| normal | C1 | **未执行** | — | — | 未启动 |
| late | B | 是/是 | 6.84 / 6.84 | -.000506 / -.000506 | 否/否 |
| late | C0 | 是/是 | 6.90 / 6.90 | -.003681 / -.003681 | 否/否 |
| late | C1 | **未执行** | — | — | 未启动 |

所有 B/C0 运行均停稳一次。normal 等待 B=3.34 s、C0=3.17 s；late 为 3.38/3.46 s。
没有安全完成或可比较的成功 traversal time。unnecessary stop/deadlock 仍为 null，
不拿失败提前截断后的短等待冒充性能收益。

## proposal / committed / backup / executed 时间线

初试代表点；完整关键事件在 [timeline.json](../../research/dynamic_state_time/results/reserve/timeline.json)，
逐周期提案、入队内容、实际执行和延迟在仓库外 trace。

| 条件/臂、时刻 | proposal | accepted committed | backup | 当时实际执行/结果 |
|---|---|---|---|---|
| normal B，2.50 | 约 (.4992,-.0283) | 相同，延迟 .10 s 生效 | 无 | 仍执行不可撤销旧命令 |
| normal B，3.55 | (0,0) | (0,0) | 无 | 停稳；6.82 被撞 |
| normal C0，2.50 | 约 (.4992,-.0283) | 拒绝，安排未认证 brake | 无 | 此刻旧前缀仍执行，没有假装撤回 |
| normal C0，2.60 | 约 (.4992,-.0283) | 通过，.10 s 后生效 | 新证书生成后丢弃 | 当前实际执行前面已排队的 brake |
| normal C0，3.65 | (0,0) | 通过 | 新证书生成后丢弃 | 停稳；6.75 被撞 |
| late B/C0，2.75 | 非零前进提案 | 80 ms 迟到，拒绝 | 无 | 制动在 t=2.88 入队生效；不是瞬时停止 |
| 两种 C1，2.50 | 尚未调用 planner | 无 | 初始化 unavailable | 未执行 rollout |

“找到 continuation”没有保证 C0 随后执行它，但本轮不能进一步证明 C1 持有就一定能修复。
normal 的 B/C0 没有 deadline miss，仍碰撞；late 每条轨迹都有 14 次 timeout brake，也仍碰撞。
不存在可以归因为 timeout brake 的成功结果，更没有可宣称的 C1 安全改善。

## 延迟、资源和最坏情况

下表为真实计算与 IPC、输入/事务/命令准备的总耗时，不包含注入等待、物理评价和日志序列化。
C0 的总分布包含一次初始化测量；初始化另有独立字段。每组第二行为不改参数复核。

| 条件/臂/轮次 | mean | p50 | p95 | p99 | max（ms） |
|---|---:|---:|---:|---:|---:|
| normal B 初试 | 7.131 | 7.234 | 11.333 | 11.859 | 11.875 |
| normal B 复核 | 7.107 | 7.291 | 11.348 | 11.978 | 12.014 |
| normal C0 初试 | 7.603 | 7.619 | 12.109 | 12.708 | 12.729 |
| normal C0 复核 | 7.634 | 7.672 | 12.000 | 12.967 | **14.908** |
| late B 初试 | 7.106 | 7.120 | 11.276 | 11.496 | 11.813 |
| late B 复核 | 7.182 | 7.177 | 11.348 | 12.051 | 12.677 |
| late C0 初试 | 7.589 | 7.418 | 11.638 | 12.557 | 12.595 |
| late C0 复核 | 7.604 | 7.519 | 11.880 | 12.447 | 12.554 |

所有实际运行的 intrinsic deadline miss=0。注入后的 late 响应 p95/p99/max 均为 **80 ms**，
B 为 14/73 次 miss，C0 为 14/74 次 miss，两轮相同；不能把注入组写为零 miss。
C1 初始化耗时约 .232–.250 ms，只有单次拒绝样本，**不是 C1 完整 planning/consumption
延迟分布**。C1 完整实时门、持久执行门和迟到恢复门均 NOT_EVALUATED。

汇总保存各运行的 min/p05/mean/p50/p95/p99/max、独立 planner/checker 延迟、
CPU 时间、进程 RSS、更新率、实际命令翻转、对齐时刻的序列 RMS、加速度/jerk、
hold TTC margin，以及每条最坏净空的时刻/状态。没有只保存平均分。
计算仅固定一个 logical CPU；不安装依赖、不接 ROS、不访问硬件。

## 逃逸窗口与裁决

原 B 再次在 t≈3.55 s 停稳，至 t≈6.82 s 被撞。上一轮证据表明早期仍有侧让机会，
到 t≈6.30 小库才不再返回 continuation；这些时间没有进入控制逻辑。
C0 中途能够反复生成局部备份证书却不持有它，仍碰撞。
**C1 因初始化失败没有到达这个窗口，不能声称已经实际利用了窗口。**

本轮没有发现可以通过小范围事务修复解释的初始化错误：新检查器与归档库结果一致，
独立 plant 重放解释了净空缺口。因此不以 Modify 为由扩大 library、改变起点、
先无证书刹停、注入旧手工见证或降低安全门来启动 C1。

**Stop 当前冻结动作库下的 prototype；不进入 36-episode。**
结果否定的是“这个固定库能从指定 fixture 初态建立第一条储备”的前提。
Persistent backup 的跨周期收益仍未验证，不等于停止全部 recourse/dynamic navigation。
若以后重新立项，需要首先明确如何建立初始证书；本轮不追加该研究。

## 复现与资产

```bash
export DST_DATA_ROOT=/tmp/rm2027_persistent_backup_reproduce
bash research/dynamic_state_time/run_reserve.sh fixture_01
bash research/dynamic_state_time/run_reserve.sh fixture_confirmation
python3 research/dynamic_state_time/summarize_reserve.py \
  --data "$DST_DATA_ROOT" --output /tmp/rm2027_persistent_backup_summary
```

脚本只调度固定 fixture，不含 36-episode 入口。编译 wrapper 时归档 main 被重命名为
未调用函数，GCC 会给出该旧函数缺少 return 的警告；不影响实际入口，不改归档源码消除它。

源码：`reserve_consumer.cpp`、`reserve_run.py`、`run_reserve.sh`、`summarize_reserve.py`。
轻量结果：[summary.json](../../research/dynamic_state_time/results/reserve/summary.json)、
[trials.csv](../../research/dynamic_state_time/results/reserve/trials.csv)、timeline 和
[dataset_manifest.json](../../research/dynamic_state_time/results/reserve/dataset_manifest.json)。
manifest 记录数据集、外部路径、大小、SHA256、来源、生成性质、用途和复现入口。
raw trajectory、日志、构建产物只在仓库外；未复制历史资产。

本轮基线为 `0ceb8e060f381bec81a895b9de40bf56109064c8`，上述轻量资产由本次归档提交保存，
归档分支为 `codex/dynamic-persistent-backup`。未 push、merge 或修改 main。
Stop 仅针对从 t=2.50 s failure fixture 现场建立第一条备份的 prototype；
不等于 persistent backup hypothesis 已被反证。
