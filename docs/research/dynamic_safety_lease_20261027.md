# Time-bounded safety certificate / safety lease（Research）

本轮只形式化并离线评价，不改闭环算法，不运行新episode。
原mode-aware authority为Modify；原persistent-backup holdout为Stop，均不改写。

## 最终冻结记录（2026-10-27）

**FROZEN — insufficient evidence for safe non-conservative certificate issuance**

按最终收尾指令，当前dynamic-navigation研究线正式冻结。Safety Lease裁决为Stop；
不进入闭环prototype或production，不merge main，不删除分支、旧WIP或历史证据。

- R1–R4历史结论不改写：R1/R2未来occupancy写入当前costmap存在时间语义错误；
  R3约20Hz离线信号未形成稳定动态闭环；R4 HWS prediction-consumption未获production Go。
  DynamicObstacleCritic时间消费有限、Temporal Gate通用路线此前Stop的结论也保持。
- Persistent backup在理想CV下有正信号，但不能外推到非理想观测的安全性。
- Certificate lifecycle/freshness换代事务修复有效，不等于物理安全验收。
- Mode-aware authority有局部判别力，尚不足以提供安全且可执行的后续。
- Single uncertainty envelope为Stop；Safety Lease为Stop。
- Latest-commit / latest-safe-dispatch metadata作为有效的固定绝对序列排队时序研究产物保留；
  不把它当作全局latest physical escape或模型可信期限。
- 当前信息条件不足以支持安全且不过度保守的certificate issuance；形式合同不包装成已验证创新成果。
- **只有出现新的tracker、observation capability或model assumption，并形成独立可证伪命题时才重新立项。**
  增加搜索量、调阈值、延长/缩短timeout或组合既有失败机制不构成重启依据。

保留最小复现源码、配置、汇总、报告和manifest；原始证据继续按manifest存于仓库外。
STVL仍为现有稳定baseline；HWS shadow默认关闭且无production控制权。

## Formal contract

`C_t=(B_t,A_t,tau_t)`，附带issue epoch、证据source版本及适用状态/queue版本。
`B_t`仍为唯一一条绝对时间backup，含原控制序列、start state、execution cursor与safe-rest记录。
`A_t`必须列出模型/行为、目标集合覆盖、误差、frame/clock、plant、观测freshness及不可撤销queue假设。
`tau_t`是**该证据最多覆盖到的绝对时间**，不是可无限依赖的静止终点保证。

物理条件命题是：`A在[issue,tau]真实成立且机器人执行B -> 该区间净空>=0.05m`。
软件检查`issued <= now <= tau AND 未观测到A反证 AND state/queue compatible`只是必要检查，
不能直接命名为物理有效性证明。真实motion已经改变而延迟观测尚未到达时，两者会分离。
没观察到反例不等于已证明A覆盖真实未来。
如果没有物理模型界/行为覆盖或经验证的有效时间证据，则`tau`为Unknown，不能把人工数字命名为安全保证。

另有不同对象`ell(B,Q)`：保存的绝对序列尚可被新命令接入的**最晚提交时刻**。
对首个未排队、与继续当前main target不同的必要backup动作，其执行时刻为`s`，则`ell=s-command_delay`。
这是100ms不可撤销queue的因果执行期限；已经排队的动作不需重新提交，过ell后也不能假装撤回。
它不等于物理上所有可能避让轨迹的最晚开始时刻。后者在未搜索/验证其他轨迹时为Unknown。

`tau`约束“证据保证到哪里”，`ell`约束“何时来得及作决定”，不能互相替代。
把ell直接当tau，可能在backup开始执行之前就过期；到了ell并不表示应该撤销正在正确执行的backup。

### 生效、失效、续租与到期动作

1. 证书只在issue快照的实测state、原不可撤销前缀和依赖集合验证后生效；不能倒签。
2. 模型/集合/frame/clock/plant/state/queue反证可使其早于tau失效；到tau则覆盖结束，即使没有观察到新运动。
3. lease长度必须来自A的覆盖，不能单凭planner horizon、backup完成时间或“过去一直没动”。
4. 延长必须由新证据支持新的A和绝对tau，连同主承诺/backup/queue原子发布。
   改source version、重签同一CV或平移静止尾部本身不提供新的行为覆盖。
5. 保留四个既有mode，不调classifier：STABLE只是允许提出renew；CHANGE限制新CV证据；
   OCCLUDED不能刷新missing目标的source；STALE禁止renew。mode本身不能计算有物理保证的tau。
6. invalid/stale、缺失依赖覆盖、无新时间证据、queue不兼容时禁止renew，不凭truth复活旧证书。
7. 若要在未来不可撤销地执行新动作，证据必须覆盖其时间窗口及可用后续；
   最迟在ell前选择并提交仍有效backup。planner busy/failure不丢弃旧有效backup。
   若到期时backup的后续也没有覆盖，返回UNAVAILABLE；不把默认制动当安全证明。
   不能越过tau继续主计划，也不能因为名为backup就无证据执行到tau之后。

这个contract可统一信息、承诺、换代和执行义务的**语法**，但尚未给出可签发A/tau的实质安全依据。
部分时段安全不等于到期后仍有续租/逃逸能力；有限lease本身没有证明递归可行性。

## 四个候选与事先选择

| 候选 | 本轮单一定义 | 主要待证假设 |
| --- | --- | --- |
| H：prediction-horizon proxy | `tau=issue+2s`，复用原main search horizon | 2s不是已验证预测有效期；不能把planner窗口当物理保证 |
| D：absolute dispatch deadline | `ell=first necessary backup event-0.1s`；没有新必要动作则无有限ell | 只提供排队物理信息，作为tau是概念错误；本轮量化“误当lease”的后果 |
| Q：information-quality lease | STABLE取`source+0.15s`；其他mode在issue abstain | fresh不表示未来CV成立，150ms只是既有观测合同 |
| M：past-consistency lease | STABLE取`issue+min(过去连续一致source时长,2s)`；其他mode abstain | 历史平稳时长不是未来保持时间下界 |

唯一事先推荐保留的物理元数据是**D的latest-commit定义**；不预先承认它可充当tau。
其余分别作为时间覆盖对照，不组合hybrid，不按结果挑选另一个赢家。
参数与门见[协议](../../research/dynamic_state_time/safety_lease_protocol.json)，先固定再运行。
若全部只能靠短timeout或不能覆盖真实失效，则Stop并建议冻结当前研究线。

## 结果

**Stop。建议冻结当前dynamic-navigation研究线，不进入闭环prototype，不继续增加机制。**
四个候选均未通过预先写定的门：短时钟有高expiry覆盖，但切断几乎全部安全backup；
长时钟保留更多完整运动，却通常晚于物理风险，而且可被连续重签无限推后。
保存序列的latest-commit具有独立执行价值，但不能被升级为模型可信期限。

### Part A与范围

上一轮mode-aware分析已归档：`865ff70f7c2a25fe73ac69a6c4655f24ea726c06`，
标题`研究：归档信息模式权限分析`，14文件234,633 bytes，最大57,075 bytes；提交后clean。
随后从该提交建立`experiment/time-bounded-safety-lease`。
本轮新增内容保留为未提交轻量WIP，没有第二个commit、push、merge或production改动。

使用完全相同的120例C1、26,540张唯一证书（含120张初始化），没有新episode。
真安全26,308张，不安全232张。复用上一轮封存的每张证书predicted/truth完整序列标签，
重建issue/source、绝对backup controls、原验证末端时间、mode、queue ownership、执行owner及原实际结果。
mode逐拍输出原样复用，不重新训练/调整classifier。
只对232张unsafe新增计算首次物理5cm风险时间：原10ms轨迹采样，
静止尾部按分段真实运动的解析ray/bisection计算；truth仅在候选时钟确定后进入评价。
不把“保存序列反事实不安全”与“该序列实际全部执行”混为一谈。

本轮全部数据在历史中已经暴露，是retrospective exploratory比较，不是独立holdout。
4张unsafe在签发时已经处于5cm违规中，无法用签发后的expiry声称提前预防。
没有对这些失败进行排除。

### 候选总体判别

“提前”是候选到期严格早于**同一保存证书完整执行的首次5cm违规**，不是早于原episode碰撞就算成功。
“提前到期”是候选早于原记录的backup验证末端（包含刹停/保持尾段），不是已观测的任务失败率。

| 候选 | unsafe风险前expiry | safe提前expiry | safe完整运动的有限lease覆盖 |
| --- | --- | --- | --- |
| H：2s horizon | **7/232，3.02%** | 2,884/26,308，10.96% | 89.04% |
| D：把dispatch deadline误当lease | **224/232，96.55%** | 25,845/26,308，98.24% | **0%** |
| Q：freshness/mode | **228/232，98.28%** | 26,308/26,308，100% | **0%** |
| M：历史一致性 | **211/232，90.95%** | 6,817/26,308，25.91% | 74.09% |

D另有463张safe无新的必要actuation，因而**没有有限dispatch deadline**；
它们不能被计作“已取得无限安全lease”。有限覆盖率仍为0。
有限D时钟的lifetime p50/p95/p99均为50ms；26,076张在新主动作不可撤销窗口结束之前已到期。
Q全部26,540张均早于该窗口结束。它们不能仅靠“到期再执行backup”弥补覆盖，因为backup也将在expiry之后执行。

Q/M各有137张unsafe在issue立即abstain，其中133张早于风险；这部分是**既有mode拒绝**，不是新增lease收益。
正lifetime带来的提前expiry分别为Q 95张、M 78张。
D/H没有这类即时mode拒绝，不能把四个总体比率当作等价的新增机制收益。
[四方法统计](../../research/dynamic_state_time/results/safety_lease/methods.csv)。

### 分类别结果

每格为“风险前到期 / unsafe证书数”。N/A表示本组没有unsafe证书，不能推断能识别该类未来风险。

| 类别 | H | D（误作tau） | Q | M |
| --- | --- | --- | --- | --- |
| static | N/A | N/A | N/A | N/A |
| crossing | N/A | N/A | N/A | N/A |
| head-on | 0/10 | 10/10 | 10/10 | 7/10 |
| stop/start | **1/96** | 96/96 | 96/96 | 85/96 |
| sudden turn | 6/9 | 9/9 | 9/9 | 6/9 |
| occlusion/reappearance | **0/24** | 24/24 | 24/24 | 24/24 |
| ID misassociation | **0/30** | 25/30 | 27/30 | 27/30 |
| prediction error | **0/63** | 60/63 | 62/63 | 62/63 |
| stale | N/A | N/A | N/A | N/A |
| multiple | N/A | N/A | N/A | N/A |

这张表看似支持短lease，但D/Q使安全运动几乎全部提前失去覆盖，M仍大量误切。
具体边界：

- **stop/start**：长期静止的连续新观测不证明不会重启。
  H对96张unsafe仅提前1张；Q/M的好分数大量来自mode abstain或比100ms命令delay更短的lease。
  前一轮5次“旧CV依赖未被反驳但truth不安全”的问题没有被补充物理假设解决。
- **occlusion**：Q/M依靠已有OCCLUDED入口拒绝，不提供missing目标接下来在哪里的证据。
  H仍0/24；不能用新空观测包续租。短expiry之后能否执行旧backup仍缺失覆盖。
- **ID误关联**：即发即风险的例子仍存在；fresh及短lease不能修正已经错误的速度归属。
  D有5张未能在风险前到期，Q/M有3张；不把到期等于风险时刻称为提前。
- **prediction error**：H对63张unsafe全部太晚；短Q有判别表象，但100%安全运动也被提前截断。
  历史一致性时长不是误差增长界，M并未建立统计覆盖保证。
- **stale**：原合同已禁止签发stale证书，本组没有unsafe标签；
  现有old backup的source/lifetime不能靠新包或版本刷新。H仍会提前截断34.93%的本组safe运动，
  D/Q为100%，M为34.93%，没有新增已验证的安全收益。

[全部40行类别统计](../../research/dynamic_state_time/results/safety_lease/categories.csv)、
[120例×4逐例结果](../../research/dynamic_state_time/results/safety_lease/episodes.csv)。

### False-safe duration与风险提前量

定义`lead=首次反事实5cm风险-time_clock`，正值才早于风险；保留全部有限时钟，包括负值。
“false-safe tag duration”为unsafe标签证书的`max(0,time_clock-issue)`，不是实际执行时长。

| 候选 | lead mean / min / p50 / p95 / max，s | unsafe tag lifetime mean / p50 / max，s |
| --- | --- | --- |
| H | -1.253 / **-2.000** / -1.410 / -0.364 / 1.883 | 2.000 / 2.000 / 2.000 |
| D | 0.697 / **-0.050** / 0.540 / 1.586 / 3.833 | 0.050 / 0.050 / 0.050 |
| Q | 0.726 / 0.000 / 0.585 / 1.586 / 3.833 | 0.0205 / 0.000 / 0.050 |
| M | 0.530 / **-1.280** / 0.455 / 1.435 / 3.283 | 0.2166 / 0.000 / 2.000 |

时间提前并不证明可逃逸。对原29个假安全失败，取各例首张unsafe：
H到期相对原物理事件平均**晚1.122s**，D/Q/M分别平均早0.828/0.869/0.711s。
但四者均没有在该例“最后记录的可执行且truth安全旧backup”之前到期的证据（0/29）。
这个最后记录值只是已保存证据，不是全动作库的真正最后逃逸边界；不能据此宣称绝对无解。
相反，也不能把早于collision的expiry称作collision-chain interruption。
实际阻止碰撞数与新等待/完成率均为Unknown。
[原29例逐例时间比较](../../research/dynamic_state_time/results/safety_lease/known29.csv)。

### Head-on：latest-safe-commit的可确定部分

head_on_10201最后记录的truth-safe证书在5.05s签发，其必要侧逃动作在5.20s，
所以从序列和delay自动得到`ell=5.20-0.10=5.10s`。代码没有这些绝对时刻的触发参数。

| 原快照 s | ell-now s | 前100ms queue兼容 | 剩余当前CV物理净空 m | 剩余truth m | 原授权 |
| --- | --- | --- | --- | --- | --- |
| 5.05 | +0.05 | 是 | 0.081313 | 0.069230 | 是 |
| 5.10 | 0 | 是 | 0.080936 | 0.069230 | **否** |
| 5.15 | -0.05 | **否** | 0.080560 | 0.069230 | 否 |

5.05到5.10是这个保存序列可作新提交的一个控制拍；到deadline已没有额外处理时间余量。
5.15的几何仍安全不表示排队命令可以撤回；不能把旧backup整体向后平移后仍叫同一certificate。
如果5.20动作已经正确进入queue，则之后无需再提交，ell不会撤销它。
原5.10授权因`prediction_changed`丢失，lease没有提供恢复旧A的依据。

因此D正确描述了执行机会，仍没有解决“原已失效证书能否依赖”的问题。
其余head-on的相同规则诊断均保留在[时间线](../../research/dynamic_state_time/results/safety_lease/head_on_deadlines.csv)。
没有搜索平移/替代动作，**全局latest physical escape start为Unknown**。

### 安全保留、强制执行与freeze

只看STABLE的真安全证书，排除warmup，但不排除难例：

| 候选 | crossing提前expiry | multiple提前expiry | 有提前expiry的成功episode |
| --- | --- | --- | --- |
| H | 44/3078，1.43% | 82/3248，2.52% | 4/12；6/12 |
| D（误作tau） | 3056/3078，**99.29%** | 3116/3248，**95.94%** | 12/12；12/12 |
| Q | **100%** | **100%** | 12/12；12/12 |
| M | 296/3078，9.62% | 334/3248，10.28% | 12/12；12/12 |

如果没有新的合格lease，这些是“必须额外续证/进入backup或UNAVAILABLE”的机会，
不是已经测得的forced-backup执行率、额外等待或deadlock。
没有运行lease policy，不能用原state继续伪造执行结果。
H稳定对照干扰小，但它基本不识别主要unsafe；D/Q的安全覆盖表象来自切断几乎一切完整backup。
[稳定对照](../../research/dynamic_state_time/results/safety_lease/stable_controls.csv)。

### 历史依赖与renewal

原策略的26,420次replacement作为renew候选事件，原速率mean19.70Hz、p50=20Hz。
既有mode允许进入renew事务24,967次、拒绝1,453次；**这是权限，不是已证明的lease延长成功**。
候选时钟在原连续签发下均可重新变大：

| 时钟 | 向后延长次数 | 同一lineage延长次数 | 尝试发生时旧时钟已过期 |
| --- | --- | --- | --- |
| H | 26,300 | 22,421 | 0 |
| D | 25,921 | 22,062 | 181 |
| Q | 26,224 | 22,419 | 2,662 |
| M | 26,227 | 22,421 | 1,436 |

这些包含原mode不允许的事件，不能全部计作本研究可接纳renewal。
延长量中位约50ms；同lineage deadline漂移最大13.70s（成功对照multiple_10908，276张证书）。
正常通行也会合理改变backup，不能一概禁止漂移；但“每次fresh重签便重置时钟”也无法阻止危险等待链。
仅按lineage设固定年龄会干扰正常轨迹，而小模型变化新建lineage又可绕过这种人为上限。
本轮未实现该上限或新的anchoring机制。

原post-transaction owner在候选时钟之后仍被持有的记录：
H为0张/0s，D为160张/20.10s，Q为2,676张/145.83s，M为1,453张/72.58s。
H并没有发现“旧证书用太久”，因为大量危险链依靠的是不断更新的错误证书。
这些只是按候选clock回看原owner的时长，不是实际违反既有合同；旧合同没有这个tau。
原命令owner在其origin时钟后执行的时间另存CSV，但新certificate可能已经覆盖同一queue前缀，
不能把命令provenance时间简单累计成未授权运动。
[lineage明细](../../research/dynamic_state_time/results/safety_lease/lineage_deadline_drift.csv)。

### 验证、数据与计算边界

候选时钟计算26,540次：mean0.00428ms、p50=0.00357、p95=0.00478、p99=0.01088、max0.77818ms。
只计四个时间表达式和deadline提取，不含mode估计、truth重建、IO或完整在线planning cycle。
计算不构成本轮主要失败；没有宣称20Hz系统或硬件验收。

针对性测试通过：固定绝对动作减delay、没有新必要动作时不伪造finite deadline、
mode/freshness时钟、修改未来motion只改变评价标签不改变clock、stationary-ray首次风险解析例、
deadline边界的不可撤销queue语义。
正常实现核查逐张复核232个unsafe风险时间：188个原10ms轨迹首次违规、44个静止尾部ray首次进入，均通过；
其中4个在issue已经违规。没有调整参数或重跑闭环。
原mode、原scenario/seed、command delay、动作库、plant与5cm阈值保持不变。

[分析源码](../../research/dynamic_state_time/offline_safety_lease.py)、
[summary与冻结门](../../research/dynamic_state_time/results/safety_lease/summary.json)、
[manifest](../../research/dynamic_state_time/results/safety_lease/dataset_manifest.json)。
逐证书全部记录及105,680条renewal诊断在仓库外，共3,439,578 bytes；原raw evidence原地保留。
Git只保留source/config、CSV/JSON汇总、本报告及manifest，没有raw轨迹、日志、cache、binary或历史归档。

复现：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 research/dynamic_state_time/offline_safety_lease.py test
```

从原manifest来源重建使用`analyze --raw <原目录> --derived <仓库外新空目录> --output <轻量目录>`。
脚本拒绝覆盖封存逐证书数据；已有数据仅重聚合用`summarize --derived ... --output ...`，不改变clock。

### 最终立项门与统一框架判断

预先门为：unsafe提前expiry≥75%，stop/start/occlusion/prediction-error各≥50%，
safe完整运动覆盖≥95%，stable crossing/multiple提前expiry≤5%，
head-on有提交窗口，且有物理有效时间依据。四者均失败，详见summary逐项checks。
结果不依赖只差几个百分点：H漏检主要风险，D/Q截断近乎全部safe运动，M没有足够保留且无未来依据。
没有通过缩小lease、延长历史窗口、调整mode或组合hybrid挽救结论。

`信息状态 -> A覆盖什么 -> tau覆盖到哪里 -> action authority -> ell前提交backup`
可以形成统一的形式逻辑，并复用atomic replacement与persistent absolute execution。
但是**目前只有接口逻辑，没有已验证的A/tau生成与renewal证据**，不能包装成已成立的个人创新结果。
D提供的queue物理期限应作为独立执行知识归档；它不补全未来行为、遮挡集合或关联误差覆盖。

**Stop当前safety-lease方向，并建议冻结这条persistent-backup / mode-aware / lease动态导航研究线。**
不继续堆模块、调整阈值或启动prototype，不接production，不merge main。
冻结表示保留知识和最小证据，不删除任何旧WIP、原raw或历史资产；
它不等于证明所有动态导航/预测问题都不值得研究。
只有出现独立的新物理/感知覆盖依据及新的可证伪命题时，才值得另行讨论是否重启；本轮不自动追加下一机制。
