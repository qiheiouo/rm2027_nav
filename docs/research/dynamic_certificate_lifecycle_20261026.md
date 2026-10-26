# Certificate lifecycle：最小软件合同验证

**Go-to-holdout-rerun，仅允许建议下一轮重跑。** 本轮没有启动 120-case 重跑，
没有真实 tracker、STVL、production 或 Hardware Go。
旧冻结实现的 **Stop** 保持有效；本报告证明换证和 freshness 的局部软件合同，未证明完整导航的安全性或完成率。

## 归档与范围

Part A 单 commit：`f1203c3949f8d48237a9ea0125c895f7f5a8ef2c`，
“研究：归档持久备份holdout负结果”，归档分支 `codex/dynamic-preacquired-backup`。
提交后工作区 clean；15 个轻量文件，最大 468,340 bytes。
30,978,519 bytes 的原始证据仍在仓库外，未复制历史归档。

[原 holdout 报告](dynamic_preacquired_backup_holdout_20261026.md)保留：
120 个 paired case、360 次 attempt；ideal 安全完成 B/C0/C1=18/17/24（各24），
imperfect=53/52/21（各96）；C1 完成45/120，B为78/120。
75 次截断既不是安全完成，也不能推断后续必撞。
54 次 prediction_changed、12 次 obstacle_set_changed 共66次已经重建 fresh 条件证书却退出；
另9次约200ms stale输入非法重新签发并接纳。计算预算通过，已完成 C1 最差净空
0.058148m，相对5cm只余8.15mm。这些负结果没有被本轮合同测试覆盖或重新计分。

Part B 从归档建立 `codex/dynamic-certificate-lifecycle`，修复以单次轻量提交“研究：修复动态备份证书换代事务”保存。
没有改 search_consumer.cpp、world.py、动作库、beam、horizon、cost、5cm门槛或冻结 holdout protocol。
修改范围是 reserve certificate service、提交事务、输入版本元数据、退出/统计语义和最小测试。

## 正式 lifecycle contract

事务 owner 为 [CertificateLifecycle](../../research/dynamic_state_time/certificate_lifecycle.py)。
这是现有离散事件 harness 的单线程提交边界，不是跨线程/实机实时锁协议。
一次 `begin → prepare → finish` 绑定一个实测状态和不可撤销队列快照，禁止重复发布同一快照。

1. `begin` 检查当前输入版本/时钟、freshness、状态、命令队列与旧证书。
   有效时进入 `OLD_VALID`；失效时保留旧对象，但撤销其 `authorized` 权限。
   保留对象仅为事务和审计服务，不代表它仍然安全。
2. `prepare` 进入 `PREPARING_NEW`。旧证书有效才允许尝试 splice；否则基于 fresh
   实测输入和真实 queued commands，用原小型动作库构造新证书。该阶段不发布命令，也不丢弃旧对象。
3. `finish` 再验证候选，准备唯一新增的已验证 command 和更新后的队列绑定；
   证书、授权和新队列作为同一提交结果返回，executor 此前不能读取候选命令。
   成功进入 `NEW_VALID`，proposal acceptance 绑定该 committed certificate。
4. 构造/校验失败或超时：旧证书仍有效则保持并按原 absolute time dispatch；
   旧证书无效则为 `INVALID`（无旧对象为 `UNAVAILABLE`），不追加依赖该证书的命令。
   无条件 timeout brake 没有加入。不能把历史失效列表当成永久 abort latch。

证书保存 start state、initial target、绝对 controls、backup controls、reference states、
execution cursor、safe-rest certificate、plant/geometry assumptions、source timestamp/version、
receive time、obstacle-set version、frame/clock epoch 和当前绑定的 immutable queue/version。
过期队列项只能按执行时间消费；旧证书整条向后平移不被允许。

队列版本必须完全一致。唯一受控的重绑定发生在 `finish`：在旧快照检查后追加**一条已经
包含在已验证 controls 中的命令**，再验证完整新队列并与证书一起发布。
这不是允许外部任意增加版本；删掉尚未执行的旧命令、外部改版本或内容都会失败。

失效原因明确分开：

| 原因 | 行为 |
| --- | --- |
| fresh prediction_changed | 撤销旧授权；同周期可构造、验证和提交新证书 |
| fresh obstacle_set_changed | 包含新增、删除或身份集合变化；可同周期换证 |
| stale_observation | 旧授权撤销，禁止签发和接纳，等待 fresh input |
| prediction_version_regressed / obstacle_set_version_regressed | 拒绝乱序输入，保留版本高水位，不能回滚证书 |
| coordinate_reset / time_reset | 拒绝跨 frame/clock 使用，需显式建立新 session |
| state/queue/plant 不一致 | 旧证书不适用；只能从真实新快照重新验证 |

fresh observation 的高水位由事务 owner 维护，即使该输入最终未取得证书，也不能用后到旧版本覆盖。
相同 source version 的不同内容会被拒绝；同一 CV 观测按时间正常外推不是内容变化。
队列已经不可撤销的运动仍会执行；失去 certificate 时不会凭空停车。
这意味着“拒绝 stale 新命令”**不保证** stale 期间的物理安全，后续完整闭环必须实际测量。

## 统一 freshness contract

同一个 native `freshness()` 用于 generation、revalidation、replacement preparation 和最终 admission validation。
保留原阈值：`epoch − source_epoch <= 0.15 s`，原浮点容差 `1e-9 s`；没有放宽阈值。
要求时间有限且 `source_epoch <= receive_time <= epoch`，时间域必须一致。
generation 内部 `certify` 也检查；只修改外层旧证书 validator 的漏洞被封闭。

stale 输入不能维持旧授权、不能重新签发，也不能接纳新主动作。
fresh 恢复后重新验证实际状态/队列，取得并提交新证书才能恢复接纳。
trace 区分 retained invalid object 与 active backup，并记录 source time、receive time、age、
certificate source version、admission decision、执行队列和原 owner。
有效备份跨周期持有时，原 certificate source version 可以保持不变；当前 fresh 观测重新验证它的 CV 假设，不能仅凭旧 source 自称 fresh。

## Deterministic test matrix

[合同测试](../../research/dynamic_state_time/test_certificate_lifecycle.py)先于历史复放运行，
12组全部通过；其中 A–H 的 admission 检查共13次，非法接纳0。

| 门 | 检查 | 结果 |
| --- | --- | --- |
| A | fresh prediction change，准备期间旧对象未丢，原子换证并接纳 | Pass |
| B | fresh 障碍增加和减少，各自换证并接纳 | Pass |
| C | 新障碍覆盖机器人导致 construction failure，无新命令、INVALID | Pass |
| D | stale 在 generation/revalidation/replacement/admission 均被拒绝 | Pass |
| E | stale 拒绝后，仅 fresh 到达才恢复换证与接纳 | Pass |
| F | 迟到旧 observation version 不能覆盖新证书 | Pass |
| G | 旧 obstacle-set version 不能回滚 | Pass |
| H | 外部 queue version 改变、pending command 丢失、重复发布均被拒绝；合法追加绑定新版本 | Pass |
| 额外 | coordinate/time reset、deadline 保持旧绝对备份、150ms边界、timestamp顺序 | 全部 Pass |

另外通过原初始证书独立 plant replay（只测 certificate）、1200项输入因果检查、
960项与归档 adapter 的 actor/input 数值一致性检查。新增的 transport metadata 没有改变 CV 数据或扰动。

## 保存失败样例的复放

[最小 fixtures](../../research/dynamic_state_time/results/certificate_lifecycle/failure_fixtures.json)共38,700 bytes。
每例仅保存失效前证书和失效时 controller 快照/提议；用冻结 plant 重建 reference samples，
与原 safe-rest 终点核对，并验证原 post-commit queue。没有复制完整轨迹。
决策只读取保存的实测状态、队列、当时观测/CV、原提议和公共静态 grid；不读取未来 truth。

| 历史样例 | 归档行为 | 本轮行为 |
| --- | --- | --- |
| static_10001 | prediction_changed，重签但退出 | NEW_VALID，接纳，执行一拍后仍 valid |
| stop_start_10300 | prediction_changed，重签但退出 | NEW_VALID，接纳，执行一拍后仍 valid |
| occlusion_reappearance_10500 | obstacle_set_changed，重签但退出 | NEW_VALID，接纳，执行一拍后仍 valid |
| stale_observation_10801/02/03/05/06/07/09/10/11 | 200ms 输入重签并接纳 | 9/9 不签发、不接纳，INVALID，保留原不可撤销队列 |

每例重复20次用于耗时分布，共240个事务；不是240个独立导航 episode。
`accepted_without_valid_backup = 0`；`stale_certificates_issued = 0`；原9个 stale admission 全部归零。
fresh 类型只选代表例，**没有声称重放全部54+12条**。
三个 fresh 片段还实际推进50ms，再用原观测的 CV 外推重验证，备份保持绝对时刻且仍 valid。

[实际 executor 接线测试](../../research/dynamic_state_time/test_certificate_harness.py)另覆盖3组、每组3个控制周期：
prediction change、set change、stale→fresh。前两组换证后继续；stale组中间拍拒绝，fresh恢复后接纳。
三组全部运行到规定第三拍，而不是因历史失效退出；非法接纳0。仅150ms前缀，不声明任务完成或安全率。
首次该接线测试的 stale 输入同时倒退了 version，实际测到的是 version rejection；
已改为冻结同一 source，最终确认中间拍的原因精确为 stale_observation。两次证据均保留在外部。

## 延迟与范围

主机 Linux x86_64、Python 3.12.3、g++ -O3；历史复放固定 CPU affinity=[0]。
对照原归档 checker 的 validate+admit；新事务含快照/ordering、freshness、重验证、构造、
最终验证、队列绑定与发布。planner 是未改的原 finite search，command 与保存的提议逐项相同。
测量不含轨迹序列化、evaluator 或 fixture restore。

| 240个事务，ms | mean | p50 | p95 | p99 | max |
| --- | ---: | ---: | ---: | ---: | ---: |
| 新事务（含 stale 拒绝） | 0.929 | 0.216 | 3.163 | 3.206 | 3.264 |
| 原 planner + 新事务 | 11.999 | 11.311 | 14.530 | 14.693 | 14.830 |
| fresh 换证事务（60个） | 3.102 | 3.109 | 3.203 | 3.250 | 3.264 |
| fresh 相对旧 validator 的新增开销 | 1.876 | 1.882 | 1.988 | 1.999 | 2.002 |

240个事务 deadline miss=0。最坏 total 为 static_10001 第15号重复，14.829556ms。
stale 占75%的测试分布，拒绝比旧错误重签便宜；**不把总体负增量宣传为换证提速**。
9个 actual-harness 接线周期另测最大21.974301ms；样本只用于接线验证，不外推部署p99。
新 checker 累计CPU0.33s、峰值RSS3.797MiB；旧 checker0.26s/3.898MiB；planner2.48s/4.043MiB。
这些为各进程值，未把 Python evaluator 排除项伪装为整个系统资源。
最终核查还修正了 construction failure 的 reason 传递（避免 INVALID 被标成 reason=valid），
再通过全部合同，并逐个复核12个保存片段：非法接纳/stale接纳仍为0，total max15.664176ms。
这次复核单独保留，未替换或筛选原240条耗时样本。

## 裁决与复现

**Go-to-holdout-rerun。** 本地换证和四入口 freshness 合同通过，代表性 fresh 失效可以换证，
原全部9个 stale 接纳关闭，计算开销明显低于40ms预算，值得下一步验证完整闭环。
当前证据仍不足以恢复原导航路线的 Go：CV误差、消失目标、stale期间不可撤销运动、
恢复时是否还有 backup、8.15mm余量和完整任务完成率均待完整 holdout。
本轮没有扩大动作库、改阈值或新增导航算法；没有自动开始120次。

只复现本轮合同与短片段：

```bash
DST_DATA_ROOT=/tmp/rm2027_certificate_contracts_new \
  bash research/dynamic_state_time/run_certificate_contracts.sh
```

脚本在新的仓库外目录构建归档 checker、原 planner 和当前 checker，先测试 A–H，
再复放保存 fixtures，最后跑3×3控制拍的接线测试。无需旧 raw；没有 full-episode 入口。
原 source 状态可由本报告的 Git 历史定位；Git 只保存源码、测试/脚本、fixture、summary、manifest和本报告。

[summary](../../research/dynamic_state_time/results/certificate_lifecycle/summary.json)保留逐例结果、分布和最坏样例；
[manifest](../../research/dynamic_state_time/results/certificate_lifecycle/dataset_manifest.json)记录来源、大小、SHA256和外部测量位置。
Part B 原始证据位于
`/home/qihei/.codex/visualizations/2026/10/25/01a1708d-d064-7252-86b0-5e92a75657dc/dynamic_certificate_lifecycle_20261026`。

Lifecycle 归档的父提交为 Part A `f1203c3`；归档后确认 working tree clean。
没有 push、merge、修改 main 或 production navigation。后续 holdout rerun 单独记录，不改写本轮合同验证范围。
