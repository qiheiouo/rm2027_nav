# Certificate lifecycle：原120-case holdout配对重跑

**Stop 当前冻结实现；不进入 real causal tracker integration。**
换证与 freshness 事务修复消除了原75次错误提前终止，C1安全完成从45/120恢复到89/120；
但完整执行暴露28次碰撞和3次完成但全程最小净空不足5cm。碰撞跨五类非理想输入，不能归结为单一边界样本。
软件合同通过、实时预算通过，不等于条件证书覆盖了真实动态运动。
原[holdout Stop](dynamic_preacquired_backup_holdout_20261026.md)保持有效。
本裁决限于当前实现和冻结范围，不把全部 persistent backup / dynamic navigation 研究判为永久无效。

## 归档、冻结与比较口径

Lifecycle 单 commit：`e819e1002d938b5938447302d8a717ce8d49a57e`，
提交信息“研究：修复动态备份证书换代事务”，分支 `codex/dynamic-certificate-lifecycle`。
14个轻量文件，最大38,700 bytes，提交后已确认 working tree clean。
其父提交 `f1203c3949f8d48237a9ea0125c895f7f5a8ef2c` 保存原holdout负结果。
[事务合同报告](dynamic_certificate_lifecycle_20261026.md)的 Go-to-holdout-rerun 只授权本轮实验，不预设导航Go。

本轮复用原120个case、seed、初态和输入扰动：每类12例；每方法24例ideal CV、96例causal imperfect。
**B/C0/C1全部重新执行，360次rollout；没有复用旧baseline计分。**
原因是公共harness移除了错误的历史失效终止标志，C0共用checker的freshness也修复了。
旧结果仅作配对，不混入新耗时分布。120不是每方法40次，也不把360解释为360个独立场景。

冻结的 `preacquired_holdout_protocol.json` 与旧归档逐字一致；`world.py`、`search_consumer.cpp`、
`protocol.json`、`reserve_protocol.json` 均未改变。B仍从`21b9e56`编译原finite search。
plant、CV数值、动作库、5cm门槛、beam、horizon、cost、delay和加减速限制不变。
新增source/version/集合元数据只绑定事务，没有改变障碍观测数值。
原protocol中“失效即退出、没有集合元数据”的文字是旧实现描述；本轮由用户授权的事务修复覆盖，
在运行前外部 `rerun_plan.json` 显式记录，未更改任何case或验收阈值。

运行前通过1200项输入因果检查，原fixed head-on结果与归档逐项一致。
B的碰撞、完成、最小净空和arrival均与旧结果完全相同；C0仅9个stale case的净空发生变化，
完成/碰撞总数不变。完整差异保存在summary的`baseline_changes`。
决策只接收当时可用的观测/CV；truth仅用于离线评价与碰撞测量。

## 完整结果

“安全完成”要求完成原任务及goal hold、无碰撞、整个episode最小净空≥0.05m。

| 方法（各120） | 完成 | 安全完成 | 碰撞 | 完成但净空<5cm | 超时/截断 |
| --- | --- | --- | --- | --- | --- |
| B | 78 | 71 | 42 | 7 | 0 / 0 |
| C0 | 76 | 69 | 44 | 7 | 0 / 0 |
| C1 | 92 | 89 | 28 | 3 | 0 / 0 |

| 输入 | 每方法次数 | B 完成/安全/碰撞 | C0 完成/安全/碰撞 | C1 完成/安全/碰撞 |
| --- | --- | --- | --- | --- |
| ideal_cv | 24 | 18 / 18 / 6 | 17 / 17 / 7 | 24 / 24 / 0 |
| causal_imperfect | 96 | 60 / 53 / 36 | 59 / 52 / 37 | 68 / 65 / 28 |

| 场景（各方法12次） | B 完成/安全/碰撞 | C0 完成/安全/碰撞 | C1 完成/安全/碰撞 |
| --- | --- | --- | --- |
| static | 12 / 12 / 0 | 12 / 12 / 0 | 12 / 12 / 0 |
| crossing | 12 / 12 / 0 | 12 / 12 / 0 | 12 / 12 / 0 |
| head_on | 0 / 0 / 12 | 0 / 0 / 12 | 6 / 6 / 6 |
| stop_start | 2 / 2 / 10 | 2 / 2 / 10 | 2 / 2 / 10 |
| sudden_turn | 12 / 12 / 0 | 12 / 12 / 0 | 12 / 12 / 0 |
| occlusion_reappearance | 9 / 6 / 3 | 9 / 6 / 3 | 6 / 6 / 6 |
| id_switch | 9 / 9 / 3 | 9 / 9 / 3 | 9 / 9 / 3 |
| prediction_error | 10 / 6 / 2 | 10 / 6 / 2 | 9 / 6 / 3 |
| stale_observation | 0 / 0 / 12 | 0 / 0 / 12 | 12 / 12 / 0 |
| multiple | 12 / 12 / 0 | 10 / 10 / 2 | 12 / 12 / 0 |

C1相比B安全完成率增加**15个百分点**；非理想输入增加**12.5个百分点**，不是只有原fixture有效。
但非理想C1仍有28/96（29.17%）碰撞，另3例净空不合格；全部C1安全违规31/120（25.83%）。
没有任何场景类别的双方完成配对平均通行时间改善达到10%。配对范围不含仅单方完成的case。

## 原75次提前终止的逐例去向

| 旧提前终止原因 | 旧次数 | 新安全完成 | 新碰撞 | 新完成但净空不足 |
| --- | --- | --- | --- | --- |
| prediction_changed | 54 | 29 | 22 | 3 |
| obstacle_set_changed | 12 | 6 | 6 | 0 |
| stale_observation | 9 | 9 | 0 | 0 |

总计：**44 safe completion + 28 collision + 3 completion_clearance_violation = 75**。
全部75例运行超过旧终止时刻，未再按历史失效列表截断。47例最终完成，其中44例安全完成。
旧45例安全完成全部保持安全完成；旧54次prediction_changed和12次obstacle_set_changed
不再错误中止，但不能称这66例都恢复安全。
旧9个stale admission对应的case均安全完成；本轮涉及66个stale控制周期，签发/接纳均为0。

[120例完整配对](../../research/dynamic_state_time/results/certificate_holdout_rerun/paired_before_after.csv)逐例保存：old/new outcome、termination reason、
min clearance、completion、traversal/end time、换证数、构造失败数、stale拒绝数和backup实际执行时长。
[75个旧截断列表](../../research/dynamic_state_time/results/certificate_holdout_rerun/old_truncated_destinations.csv)可直接筛选三类去向，未仅保存aggregate。
旧截断前的正净空是前缀观测，不能与新完整episode净空等同解释。

## 全部失败与最坏样例

下表列出C1全部31个失败。B全部49例、C0全部51例和C1全部31例共同保存在
[131例失败清单](../../research/dynamic_state_time/results/certificate_holdout_rerun/failures.csv)，包括结局、终止原因、最坏状态、失效原因及不可用时长；
[360次trials](../../research/dynamic_state_time/results/certificate_holdout_rerun/trials.csv)保存全部成功和失败，不筛掉碰撞。

| C1 case ID | 结局 | 最小净空 m | 结束 s | 无有效证书累计 s |
| --- | --- | --- | --- | --- |
| head_on_10201 | 碰撞 | -0.001842 | 6.51 | 1.45 |
| head_on_10203 | 碰撞 | -0.003885 | 6.62 | 1.30 |
| head_on_10205 | 碰撞 | -0.000695 | 6.34 | 1.20 |
| head_on_10207 | 碰撞 | -0.001422 | 6.79 | 1.80 |
| head_on_10209 | 碰撞 | -0.003872 | 6.55 | 1.30 |
| head_on_10211 | 碰撞 | -0.001038 | 6.32 | 1.25 |
| stop_start_10300 | 碰撞 | -0.004730 | 3.57 | 0.75 |
| stop_start_10301 | 碰撞 | -0.002635 | 3.37 | 0.30 |
| stop_start_10302 | 碰撞 | -0.002428 | 4.49 | 0.90 |
| stop_start_10303 | 碰撞 | -0.005918 | 3.48 | 0.65 |
| stop_start_10305 | 碰撞 | -0.003112 | 4.31 | 0.75 |
| stop_start_10306 | 碰撞 | -0.004789 | 3.22 | 0.40 |
| stop_start_10307 | 碰撞 | -0.005549 | 3.51 | 0.45 |
| stop_start_10308 | 碰撞 | -0.004819 | 4.47 | 0.90 |
| stop_start_10310 | 碰撞 | -0.000017 | 3.96 | 0.90 |
| stop_start_10311 | 碰撞 | -0.006239 | 3.77 | 0.20 |
| occlusion_reappearance_10502 | 碰撞 | -0.000654 | 3.29 | 0.60 |
| occlusion_reappearance_10503 | 碰撞 | -0.004483 | 3.33 | 0.35 |
| occlusion_reappearance_10506 | 碰撞 | -0.004095 | 3.19 | 0.50 |
| occlusion_reappearance_10507 | 碰撞 | -0.000879 | 3.38 | 0.40 |
| occlusion_reappearance_10510 | 碰撞 | -0.006433 | 3.37 | 0.70 |
| occlusion_reappearance_10511 | 碰撞 | -0.003091 | 3.44 | 0.45 |
| id_switch_10608 | 碰撞 | -0.006175 | 2.66 | 0.00 |
| id_switch_10610 | 碰撞 | -0.000363 | 2.56 | 0.20 |
| id_switch_10611 | 碰撞 | -0.001243 | 2.67 | 0.00 |
| prediction_error_10702 | 完成但净空不足 | 0.021314 | 12.70 | 0.75 |
| prediction_error_10703 | 碰撞 | -0.002261 | 3.63 | 0.35 |
| prediction_error_10706 | 完成但净空不足 | 0.034895 | 12.95 | 1.00 |
| prediction_error_10707 | 碰撞 | -0.000713 | 3.54 | 0.35 |
| prediction_error_10710 | 完成但净空不足 | 0.038069 | 13.25 | 0.70 |
| prediction_error_10711 | 碰撞 | -0.000185 | 3.57 | 0.35 |

C1最坏净空为`occlusion_reappearance_10510_C1`的−0.006433m。
B/C0最坏均为`id_switch_10608`的−0.006175m。
负值是离散碰撞checker首次检测到重叠的程度，不能用于把较小穿透量解释成碰撞较轻。
每个失败的关键proposal、commit、backup、executed及终止事件保存在
[failure_timeline](../../research/dynamic_state_time/results/certificate_holdout_rerun/failure_timeline.json)，完整轨迹仅在外部manifest引用的位置。

### 首次失败核查与不改参数复核

`head_on_10201_C1`使用固定速度偏差+0.01m/s、观测latency 0.10s。
在排除输入/队列/执行接线和超时问题后，进行一次原样复核；没有调阈值、动作库或采样。
主运行和复核均在**6.51s碰撞**，最小净空均为**−0.0018422006321914264m**，
碰撞状态均为`[1.3098222006321911, 0.015409154803720257, 0, 0]`。
两次均accepted=100、rejected=31、replacement=100、backup dispatch=2、deadline miss=0。
主运行max latency18.895ms，复核19.107ms。复核单列，不加入360次或改善统计。

| 时刻 | 证据与行为 |
| --- | --- |
| 2.10s | 首次prediction_changed；旧授权撤销，fresh新证书提交成功，episode继续 |
| 3.55s | 机器人停稳；fresh证书包含从3.70s开始的lateral backup，当前仍提交主停车动作 |
| 5.00s | 仍停车；新证书的backup起点为5.15s |
| 5.05s | 再次换证，新backup从5.20s开始 |
| 5.10s | 已不能从真实状态和不可撤销queue构造新backup；进入INVALID，queue剩余动作均为brake |
| 后续 | 未执行旧证书的完整逃逸段，实测状态偏离旧reference；不能继续声称旧证书有效 |
| 6.51s | 停车位置被迎面障碍撞击 |

这不是未经验证把旧absolute时间整体后移；每次都是fresh输入下重新建立的条件证书。
但小速度偏差使exact-CV一致性反复失效，主停车动作不断被新条件证书准入，
备份的执行机会未转化为及时逃逸。证书换代成功不能证明闭环一直保有能承受模型偏差的安全承诺。
此非理想case到5.10s已失去新backup，不能把旧理想fixture的3.55–6.30s窗口当作通用触发时间。

另只读复核`stop_start_10300_C1`：truth在2.50s停止、2.70s重启；
停车后的第一次换证成功，但观察到重启后2.85s首次无可用backup。
队列不可撤销且没有新授权命令，最后已提交运动target保持，3.57s以约0.5m/s碰撞，净空−0.004730m。
这揭示“撤销不适用证书”不等于“物理安全停车”。没有添加新fallback来改变该结果。

## Certificate生命周期

initial acquisition **120/120**。27,338个C1控制周期：

- fresh候选构造并提交/换证26,420次；生成后未提交0次。
- 无法构造fresh候选852次。该计数包含旧有效backup仍可保留的拒绝，不等于852个不可用周期。
- 失效后同周期成功换证2,424次；拒绝主proposal但保留旧有效backup402次。
- 无有效授权516周期，累计25.8s；单episode最长连续不可用1.45s。
- episode等权backup availability：mean96.17%、p05 80%、median100%、min73.53%；
  按周期汇总98.11%。不可用集中于危险时段，均值不能替代安全证据。
- 真正运动backup执行累计20.1s，全部在当时有效授权下。
  已预排backup在当前授权失效后继续物理执行的时长为0s；这类情况单独审计，未当成当前有效证书。
- stale拒绝66次/9例；`accepted_without_valid_backup=0`、`stale_certificates_issued=0`、
  `new_dispatch_without_authority=0`、`stale_active_claims=0`、`stale_backup_execution_steps=0`。

| 失效原因 | 控制周期数 | 涉及episode数 |
| --- | --- | --- |
| prediction_changed | 2478 | 54 |
| state_deviation | 312 | 34 |
| obstacle_set_changed | 84 | 18 |
| stale_observation | 66 | 9 |

obstacle_set_changed现涉及18例：12例遮挡/重现及6例纯ID集合变化；旧终止统计12例不是同一计数口径。
所有方法的因果错误、撤回不可撤销队列、执行队列错配、旧备份保持/cursor错误和timeout brake均为0。
旧无效对象保留供审计，不被列为active backup。结果不依赖超时导致偶然制动。

## 净空、任务与鲁棒性边界

| C1样本 | n | mean m | min | p05 | p50 | p95 | p99 | max |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 所有episode最小净空 | 120 | 0.096606 | -0.006433 | -0.004790 | 0.063903 | 0.573356 | 0.744927 | 0.767315 |
| 仅完成episode最小净空 | 92 | 0.126916 | 0.021314 | 0.051921 | 0.065096 | 0.637251 | 0.747398 | 0.767315 |
| 完成episode相对5cm余量 | 92 | 0.076916 | -0.028686 | 0.001921 | 0.015096 | 0.587251 | 0.697398 | 0.717315 |

即使只看89个安全完成，最小净空也只有0.051530m，相对门槛余量**1.53mm**。
92个完成case的余量p05仅1.92mm，不能据此声称承受真实tracking误差。

| prediction_error固定偏差范数 m/s（每组3例） | 安全完成 | 碰撞 | 完成但净空不足 | 组内最小净空 m |
| --- | --- | --- | --- | --- |
| 0.014142 | 3 | 0 | 0 | 0.059871 |
| 0.042426 | 3 | 0 | 0 | 0.051530 |
| 0.084853 | 0 | 0 | 3 | 0.021314 |
| 0.169706 | 0 | 3 | 0 | -0.002261 |

这些是冻结case的条件性边界，不是通用误差容限。head-on在更小单轴偏差也能失败，
说明损失不只是单次prediction position error，还与准入后backup持续有效、执行时机和真实运动变化相关。
stop/start失败10/12；occlusion/reappearance失败6/12且比B少完成3例；ID switch失败3/12；
prediction error失败3次碰撞加3次净空违规。sudden turn在本组固定条件12/12通过，不外推任意转向。
stale拒绝后12/12完成、无stale backup执行，说明该合同在这些case有效，不保证任意丢帧时长。
multiple为12/12完成；没有证据显示本组多障碍普遍堵塞，但不能抵消其他类别物理失败。

| 方法 | 完成通行时间 s mean/p50/p95/max | 等待 s mean/p50/p95/max | TTC margin s min/p05/p50 |
| --- | --- | --- | --- |
| B | 9.772 / 10.125 / 10.966 / 11.140 | 2.197 / 2.285 / 3.411 / 3.630 | -0.090 / -0.080 / 0.650 |
| C0 | 9.729 / 10.115 / 10.787 / 11.000 | 2.179 / 2.285 / 3.400 / 4.060 | -0.090 / -0.080 / 0.645 |
| C1 | 10.568 / 10.250 / 14.123 / 14.250 | 2.212 / 2.270 / 4.211 / 4.400 | -0.090 / -0.080 / 0.640 |

未观察到40s timeout或停止超时代理；碰撞提前结束的case不能证明之后不会死锁。
通行时间只统计各方法完成样本，群体不同，不能直接以三列均值作因果收益判断。
C1等待均值2.212s，B为2.198s；本轮主要问题是安全和证书覆盖，不是普遍额外等待。
TTC为现有hold-command + 当时CV的margin诊断，非未来真值TTC、非安全证明。
按观察时长和误差的逐例值、0–3s horizon error分布均在trials/summary，短episode的样本数不等。

## 延迟与资源

Linux x86_64、Python3.12.3、g++ -O3、CPU affinity=[0]；所有方法顺序执行。
完整cycle包括planner和certificate/commit consumption；不包含离线truth evaluator、轨迹序列化与压缩。
该计时覆盖本实验决策路径，不能替代真实tracker/Nav2系统延迟。

| 方法 | 周期数 | mean ms | p50 | p95 | p99 | max | deadline miss |
| --- | --- | --- | --- | --- | --- | --- | --- |
| B | 24412 | 11.396 | 11.910 | 13.939 | 14.410 | 15.518 | 0 |
| C0 | 24007 | 11.930 | 12.486 | 14.455 | 14.892 | 15.989 | 0 |
| C1 | 27338 | 15.595 | 15.793 | 18.387 | 18.819 | 21.748 | 0 |

C1最慢周期来自`stale_observation_10801_C1`，21.748ms；p99/max和deadline门均通过。
因此安全失败不能解释为计算预算超限或timeout brake。

| 方法 | native CPU s/episode mean/p50/p95/p99/max | Python CPU s/episode mean/p50/p95/p99/max | native RSS峰值和 MiB p50/p95/max |
| --- | --- | --- | --- |
| B | 2.079 / 2.440 / 3.311 / 3.356 / 3.470 | 0.625 / 0.718 / 0.970 / 0.988 / 0.989 | 4.129 / 4.195 / 4.199 |
| C0 | 2.095 / 2.495 / 3.380 / 3.420 / 3.430 | 0.665 / 0.775 / 1.034 / 1.060 / 1.074 | 7.949 / 8.035 / 8.074 |
| C1 | 2.801 / 3.080 / 4.030 / 4.185 / 4.220 | 1.210 / 1.259 / 1.946 / 1.968 / 1.969 | 8.023 / 8.156 / 8.184 |

native CPU为planner+checker每episode累计进程CPU；Python列包含harness/evaluator。
native RSS为各子进程峰值之和，非同时总RSS。外部GNU time记录整个批处理max RSS37,988KiB（37.10MiB），
也是进程记账口径，不是所有并发进程内存之和。两种口径均远低于256MiB，但没有真实导航整栈内存结论。
批处理wall19m21.11s、user1149.21s、system8.50s、CPU99%；仿真不按实时时钟睡眠，
99%表示尽快执行实验，不能当作机器人在线CPU占用率。

## 裁决

**Stop 当前冻结实现。** lifecycle软件合同确实修好，原75次错误退出全部消除；
真实安全完成率也提高，实时门通过。但非理想输入在五类中共28次碰撞、另3次低净空，
零碰撞和完成净空门明显失败。一次不改参数复核重现首个失败，不能以增加library、调margin或oracle输入补救本轮。
这是多个常见非理想条件下的证书覆盖/持续可执行性失败，超出一个局部事务问题；不据此给Modify或Go。

**不值得直接进入real causal tracker integration。** 当前输入已经足以揭示物理安全失败，
接入真实tracker不能当作对该问题的修复。本轮不设计或实现新算法，不接STVL/Nav2/production，不宣称Hardware Go。

## 轻量资产与复现

[summary](../../research/dynamic_state_time/results/certificate_holdout_rerun/summary.json)、[trials](../../research/dynamic_state_time/results/certificate_holdout_rerun/trials.csv)、[逐例配对](../../research/dynamic_state_time/results/certificate_holdout_rerun/paired_before_after.csv)、
[失败清单](../../research/dynamic_state_time/results/certificate_holdout_rerun/failures.csv)、[manifest](../../research/dynamic_state_time/results/certificate_holdout_rerun/dataset_manifest.json)均为轻量文本。
manifest记录外部文件大小、SHA256、来源、用途和generated属性；本轮外部证据合计40,738,758bytes，
包括完整原始轨迹和日志，未复制进Git；build_cache不纳入证据清单。
原30,978,519bytes历史raw仅引用原目录，未搬运。未创建新大型fixture，精确case仍由原冻结protocol定义。

外部本轮目录：
`/home/qihei/.codex/visualizations/2026/10/25/01a1708d-d064-7252-86b0-5e92a75657dc/dynamic_certificate_holdout_rerun_20261026`。
实验于2026-10-26开始，报告于2026-10-27收尾，目录名沿用启动日。

以下仅说明复现，不在本轮再次执行。当前runtime保持在e819e10，使用新仓库外目录：

```bash
export PYTHONDONTWRITEBYTECODE=1
export DST_DATA_ROOT=/tmp/rm2027_certificate_rerun_new
bash research/dynamic_state_time/run_preacquired_holdout.sh sanity --sanity
/usr/bin/time -v -o "$DST_DATA_ROOT/process_resources.txt" \
  python3 research/dynamic_state_time/preacquired_holdout.py \
  --build "$DST_DATA_ROOT/build_cache" --output "$DST_DATA_ROOT/holdout_120"
python3 research/dynamic_state_time/preacquired_holdout.py \
  --build "$DST_DATA_ROOT/build_cache" --output "$DST_DATA_ROOT/confirm_head_on_10201" \
  --confirm head_on_10201 C1
# OLD_DATA_ROOT指向manifest中的原holdout目录；只读，不复制原raw。
python3 research/dynamic_state_time/analyze_certificate_rerun.py \
  --data "$DST_DATA_ROOT" --old-data "$OLD_DATA_ROOT" \
  --output /tmp/rm2027_certificate_rerun_summary_new \
  --decision Stop --decision-reason '28 collisions and three completed clearance violations; no tracker integration'
```

最终Git：仅Part A形成上述一个commit；新的离线分析脚本、结果和本报告作为本轮未提交WIP保留。
没有push、merge、修改main、production或旧仓。提交后clean与当前rerun产物未提交是两个时间点，
不把当前工作区误报为clean；不自动形成第二个commit。
