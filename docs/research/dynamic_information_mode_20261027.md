# Information-mode-aware persistent backup：Research contract

本轮只做既有证据的离线判别、以及过门后的条件性authority replay。
不是production接口合同，不赋予任何运动权限；不修改planner、CV predictor、plant、backup primitives、执行器或5cm门槛。
结果与裁决在本文件后续段落记录，不另建重复设计报告。

## 正式对象与可证伪命题

`P_t`：已接纳的主动作承诺，包含绝对执行时间、命令、queue version；
其不可撤销前缀`Q_t`由既有executor拥有，信息模式无权撤回。

`B_t`：已取得的单条backup对象，保存原有start state、绝对controls、cursor、immutable queue/version、
prediction/source版本、safe-rest及validity assumptions。
对象可以被持有而失去执行授权；`owned`不等于`valid`，`valid under exact CV`也不等于物理安全保证。

`I_t=(mode, reasons, observed-track ledger, recovery count)`：只由已交付观测prefix决定的信息状态。
`STABLE_TRACK`表示通过本轮一致性检查，不能解释为已知道未来不会启动、转向或发生新的遮挡。

假设：相对于无mode的既有lifecycle，因果信息模式可以阻止相当比例的低依据信息替换，
同时保留有证据支持的旧backup执行权限，对稳定观测的主动作准入影响很小。
若只增加拒绝、没有可执行后续，或需要长期冻结来维持表面安全，则不支持进入prototype。

令`V_t(B)`为既有状态、frame/clock、queue、cursor、覆盖与假设检查，
`Inherit(P',B,Q_t)`为新主动作后合法接回旧序列，`Draft(B')`为提交前已验证的新backup。
新主承诺仅允许在下式成立时原子发布：

```
AllowMain(I_t) AND InputFresh(t)
AND [ V_t(B_t) AND Inherit(P', B_t, Q_t) OR VerifiedDraft(P', B', Q_t, I_t) ]
```

事务同时发布主动作、对应backup及queue version；否则保持原承诺并执行仍有授权的旧序列。
planner failure/busy/lateness本身不撤销`V_t`。新的不可信输入也不能自动提供新证据。
但当新事实反驳旧假设、观测依赖过期或queue不兼容时，必须撤销执行授权，不能以“persistent”为由掩盖失效。
没有合格后续时返回`UNAVAILABLE`；不把默认制动称为已认证backup。

## 最小四模式及transition contract

当前接口没有独立association confidence/explicit track termination。
真实运动突变与速度误关联可产生相同输入，合并为`CHANGE_UNCERTAIN`，reason保留区分可观察事件。
不从数据集`kind`或truth actor index推断模式。

| mode | 纯因果入口 | 退出 |
| --- | --- | --- |
| STALE | `t-source_epoch > 150ms`，与既有freshness容差一致 | 收到fresh观测，再按其他flags判定；重复receive不算新source |
| OCCLUDED | 已看见的ID无匹配地消失；保留last state/source与missing age | 原ID重新出现，或唯一的因果空间匹配确认ID重命名；随后重新积累clean更新 |
| CHANGE_UNCERTAIN | 冷启动；ID/set变更；重现；速度跳变>0.3m/s；位置差分与报告速度残差>0.05m/s；恢复尚不足 | 连续3个不同source的clean差分，无未解决loss且输入fresh |
| STABLE_TRACK | 以上均无，且完成恢复 | 任何不可信入口立即退出 |

优先级`STALE > OCCLUDED > CHANGE_UNCERTAIN > STABLE_TRACK`，高优先级不清除其他flags。
丢失超过0.5s只标`unresolved_timeout`，不自动从证书世界删除。
ID重命名仅在消失/新增集合间存在唯一一一位置匹配（以旧CV推进到新source，误差≤0.05m）时合并观测身份；
不把旧速度可信度转给新ID，新ID从零历史重新恢复。歧义匹配继续保持missing。
这里的0.05m是ID匹配容差，不修改碰撞几何/5cm安全门槛。
无显式track termination时无法证明目标已永久离场；这是接口限制，不能用timeout伪造终止事件。

## Action authority

| mode | 新CV证书 / 主计划替换 | 已有backup |
| --- | --- | --- |
| STABLE_TRACK | 可进入原有checker与原子事务，模式不替代几何/queue检查 | 仍须`V_t`；可继承或原子替换 |
| CHANGE_UNCERTAIN | 当前精确CV证书族不支持不可信变化，拒绝新签发及新主动作；不擅自缩短horizon或改模型 | 保留对象；仅当变化未反驳其依赖且其余检查仍成立时可执行 |
| OCCLUDED | 禁止用删去missing actor的世界签发；当前证书族无lost-track覆盖，因此拒绝新签发/替换 | 可见目标更新不能刷新missing actor的source age；不满足旧依赖则unavailable |
| STALE | 禁止签发、renew、admit | 不给旧证书换时间戳；仅与stale依赖无关且仍有效的序列可执行；当前动态精确CV证书通常无法给出这个证明 |

不凭真值复活旧backup，不把“旧路径事后truth安全”当作已有授权。
本阶段不新增保守包络、行为模型、fallback或第二条backup。
初始化需要预取得backup且完成可信历史；既有证据没有episode之前的观测历史，
初始warmup可能限制主动作，这是需要报告的部署/实验接口缺口，不能补造历史。

## 预先固定的离线门与计量

见[协议](../../research/dynamic_state_time/information_mode_protocol.json)。
单一未调参设计，使用原120例，不做新的独立holdout宣称。
Phase 1：三个必要类别的可观测事件入口识别率≥90%、原首张unsafe证书时非stable覆盖≥50%；
稳定crossing/multiple恢复后stable保留≥98%；stale合同一致率100%；ideal warmup后无无依据transition。
这只证明判别力，不是导航安全门。
Phase 2：unsafe admission veto总体和必要类别≥75%、stable safe rejection≤2%、
原29链中至少50%存在“veto且旧backup仍有因果授权证据”、成功轨迹限制时间占比≤10%、连续限制≤1s、mode p99≤10ms。
不以“全部禁止”过门。实际新增等待/到达时间在离线条件下Unknown，限制时长只作proxy。

Phase 1的事件标签由既有扰动/运动事件及实际交付时刻离线构造，仅进入评价器。
首张unsafe证书、truth replay标签复用上一轮封存派生样本，不提供给mode estimator。
Phase 2按原实测state和不可撤销queue评估局部替换机会，不能把改命令后仍沿原轨迹的重放伪装成新闭环。
必须记录首个command divergence，此后统计均为条件性机会，不能称“阻止碰撞”。

## 与历史路线的关系

R1/R2的当前occupancy时间语义错误、DynamicObstacleCritic时间消费有限仍成立；本轮不向当前costmap写未来状态。
R3约20Hz离线信号没有稳定动态闭环验收；R4/HWS prediction-consumption没有production Go。
逐节点recourse的77ms尾延迟与timeout brake混杂不重试；本轮不对树节点求后续。
现有persistent lifecycle的绝对时序、单backup、不可撤销queue和原子换代继续作为前提，
新增的是显式信息状态与**替换权限合同**，不声称原persistent安全或task收益已被证明。
上一轮各向同性uncertainty筛查的Stop不改写；本轮分类不同的信息失效语义，而非再调一个连续膨胀系数。

HWS启发限于时间阶段一致的prediction consumption、跨周期warm-start/execution continuity、状态机与执行反馈。
曾迁移prediction/surface/member接口、shadow substrate不等于迁移完整闭环行为，shadow仍默认关闭。
**不声称persistent backup或information-mode authority是HWS已有机制。**
本轮formal commitment/backup/authority合同是新增研究对象；HWS原场景的运动、感知、执行假设不能直接当作RM证据。

## 结果

**Modify；Phase 1有判别力，Phase 2未过门，不进入闭环prototype。**
模式能阻止部分低依据信息替换，稳定crossing/multiple几乎没有额外干扰。
但它没有补全旧证书的未来行为覆盖，也没有凭空产生可执行reserve；单纯禁止新计划不构成安全导航方案。
没有根据结果调阈值、恢复样本数、丢失timeout、动作库、planner或safe-rest horizon。

### 归档与研究状态

上一轮uncertainty归档commit为`5ea22d61b66fac0ba741e87e0c43ec64c2948eb4`，
标题`研究：归档动态证书不确定性筛查结果`，14文件479,740 bytes，最大173,446 bytes。
提交前相关测试通过，提交后clean；随后从该提交建立当前`experiment/mode-aware-persistent-backup`。
本轮source/config/results及本报告作为未提交WIP保留；没有第二个commit、push或merge。
闭环runtime仍为`e819e10`，未修改production、默认配置、比赛行为或旧仓。
实现只有离线入口，没有production可启用路径，未宣称Research以外的Go。

### 证据口径与实现核查

读取原120个C1 episode的27,338拍观测，复用原封存的26,540张证书标签；
Phase 2只评价其中26,420次原已接纳主计划/证书换代，排除120张初始化证书。
原来未接纳的proposal未保存合格新backup，安全标签为Unknown，不为拒绝它们领取安全收益。
所有120例在历史研究中已经暴露，因此本轮是**retrospective exploratory**，不是新的独立holdout。
观察mode和局部authority决策先计算，truth只用于事后标签和保留序列审计。

完成一次正常核查及不改参数复核：

1. Phase 1初版事件评价把`source=2.6999999999999997`按epsilon当作已交付2.7s事件。
   原观测生成器严格比较，该包仍报告静止；4个restart被误记漏检。
   修正评价器交付对齐后，stop/start onset从20/24变24/24；**逐拍mode输出完全不变**。
2. Phase 2初版只核查旧证书列出的actor，漏查missing ledger是否被旧证书覆盖。
   遮挡期间原baseline新签发的空世界证书因此被错误计作可保留。
   加入已写定合同要求的missing依赖覆盖检查后，局部支持从577降到472，
   truth不安全保留从23降到5，原29链的安全reserve机会从13降到7。
   veto数、模式和所有参数不变。初版统计不得当作研究收益。

两个初版及修正说明均保留在仓库外，见[审计摘要](../../research/dynamic_state_time/results/information_mode/implementation_audit.json)与manifest。
最终覆盖测试包含“空世界旧证书不能覆盖missing actor”，没有为了降低失败数增加新机制。

### Phase 1：模式判别

| 类别 | 可观测事件入口识别 | 首张unsafe时非stable / 有unsafe的episode | 恢复后stable比例（episode均值 / 最差） |
| --- | --- | --- | --- |
| static | 6/6（位置偏差） | N/A | 99.36% / 98.70% |
| crossing | 无事件 | N/A | **100% / 100%** |
| head-on | 2/6（速度偏差） | 2/4 | 88.58% / 31.45% |
| stop/start | **24/24** | **8/11** | 92.77% / 90.32% |
| sudden turn | 12/12 | 0/1 | 98.65% / 98.64% |
| occlusion/reappearance | **12/12** | **6/6** | 84.52% / 67.19% |
| ID relabel/misassociation | **12/12** | **3/3** | 93.33% / 76.47% |
| prediction error | 6/12 | 6/6 | 67.94% / 14.89% |
| stale | 按freshness逐拍评价 | N/A | 98.36% / 95.80% |
| multiple | 无事件 | N/A | **100% / 100%** |

表中stable比例排除固定初始0.15s恢复，不删除其他非stable时间。
24/24 stop/start包括停止和启动两个事件；遮挡入口、ID入口均在首个实际交付相关观测的同拍识别，
相对delivery的检测延迟均0s。相对物理事件仍受原100ms延迟和采样边界影响，不能称提前知道事件。
小于0.05m/s一致性阈值的连续偏差可仍被判stable；不是所有prediction error都可从本规则识别。

所有拍stale与150ms freshness判定一致，与native原记录也一致，mismatch均0。
ideal-CV组在warmup后没有无依据的非stable拍；稳定crossing/multiple各12例均100%保持stable。
模式转换率（120例分布）mean0.337/s、p50=0.178/s、p95=1.253/s、max1.538/s；
任意1s内转换数p50=2、p95=4、max4，包含正常冷启动恢复与真实事件入口/退出，不等于全为chatter。
全局mode拍数：STABLE 25,649、CHANGE 1,506、OCCLUDED 117、STALE 66。

有unsafe证书的31例中6例首张未被mode限制：head_on_10203/10209、stop_start_10301/10302/10308、sudden_turn_10410。
其中sudden_turn_10410在0.70s已有反事实unsafe的无限safe-rest，而转向直到2.5s才发生；
这不是因果分类器可以提前读到的信息。stop_start_10302在3.15s已恢复stable，实际restart为3.5s。
**恢复3次一致观测不证明未来继续静止。**

按预定Phase 1门：必要三类别入口≥90%、首张unsafe非stable≥50%、
crossing/multiple≥98%、stale完全一致及ideal无误跳转，全部通过。
这只是判别门；stop/start的8/11不能升级成高覆盖安全结论。
[逐例统计](../../research/dynamic_state_time/results/information_mode/phase1_episodes.csv)、
[事件时序](../../research/dynamic_state_time/results/information_mode/phase1_events.csv)、
[全部模式转换](../../research/dynamic_state_time/results/information_mode/mode_transitions.csv)。

### Phase 2：条件性authority replay

以下是原已接纳动作在**原测量状态/queue**上的局部veto机会，
不代表拒绝后机器人仍会到达后面的原状态。
完整有限序列与无限safe-rest的原truth标签保持原口径。

| 类别 | false-safe admission veto | safe admission误拒绝 | 有旧依赖支持的保留机会 / 其中truth安全 |
| --- | --- | --- | --- |
| static | N/A | 54/2846，1.90% | 48 / 48 |
| crossing | N/A | 36/3114，1.16% | 36 / 36 |
| head-on | 7/10，70.00% | 150/2517，5.96% | 36 / 36 |
| stop/start | **13/96，13.54%** | 65/1102，5.90% | 64 / **59** |
| sudden turn | 0/9 | 72/2697，2.67% | 60 / 60 |
| occlusion | 24/24，100% | 147/1906，7.71% | 60 / 60 |
| ID relabel/misassociation | 30/30，100% | 75/2553，2.94% | 60 / 60 |
| prediction error | 63/63，100% | 645/2391，**26.98%** | 36 / 36 |
| stale | N/A | 36/3778，0.95% | 36 / 36 |
| multiple | N/A | 36/3284，1.10% | 36 / 36 |
| **全部** | **137/232，59.05%** | **1316/26188，5.03%** | **472 / 467** |

保留机会含初始warmup，且不同机会可重复同一backup，不能解释为472张新证书或472次成功执行。
所有472个支持快照还核对了原事务前queue version，mismatch=0；前100ms不可撤销命令与旧序列兼容。
旧绝对时间不平移，起点沿原plant重建；只延续之前已授权的owner，不复活先前已经invalid的对象。

**仍有95/232次unsafe admission不会被mode阻止。**
原29条失败链中24条存在unsafe veto机会；只有7条同时存在“旧依赖仍支持且事后truth安全”的局部reserve机会，
全在stop_start_10300/10303/10305/10306/10307/10310/10311，覆盖率**24.14%**，未达50%。
它们是潜在局部断点，不是已被证明中断的collision chain。
[原29例逐例结果](../../research/dynamic_state_time/results/information_mode/known29_authority.csv)。

1,689个限制快照中的原因：886次旧模型已被新观测反驳，205次此前已无授权backup，
105次missing actor不在旧证书覆盖中，12次新actor未覆盖，9次stale依赖；其余472次支持旧假设。
没有把旧路径truth安全当作恢复授权的依据。

### 关键失败、保守性与重放边界

- **stop/start，5次旧模型支持但物理不安全**：10300/10303/10305/10309/10310在2.70s。
  例如10300旧证书54剩余序列truth净空0.04349m，10309为0.03090m。
  当时source仍描述停止，CV依赖未被当前观测反驳；后续启动破坏无限safe-rest。
  因此“旧假设未被反驳”不能被命名为一般意义的安全有效backup。
- **occlusion**：入口能100%识别，24次unsafe替换可局部否决；
  但不能沿原baseline后来空世界签发的backup继续领取保留收益，missing目标的source clock也不会被新空包刷新。
  需要真实的lost-track覆盖证据，模式名本身没有提供这种证据。
- **ID误关联**：能在首个速度不一致包进入CHANGE并否决30次unsafe admission；
  旧模型被速度反转/归属变化反驳，不能因为想保留reserve就忽略反证。
  纯ID relabel与速度误关联必须区别，mode合并只表示当前接口无法总是区分物理原因。
- **持续偏差的冻结**：prediction_error_10702/10706/10710连续限制10.60/10.85/11.15s，
  占原轨迹84.65%/84.94%/85.28%；head_on_10205/10211连续4.25s，占69.29%。
  这些episode有的原本完成但净空违规，不能只看89个安全成功对照来隐藏它们。
- **head_on_10201**在5.05/5.10/5.15仍为STABLE；小连续偏差没有触发mode。
  原5.10仍可执行但被撤销的逃逸时机问题依然独立存在，本轮层没有恢复授权，也不声称已修复。

稳定crossing/multiple在warmup之后的**6,326次safe admission零误拒绝**；
表中该两类36次误拒绝各来自12例×3拍warmup。
89个原安全完成episode的限制时间占比：mean2.01%、p50=1.28%、p95=4.54%、max5.06%；
最长连续限制p50=0.15s、p95=0.45s、max0.70s。
这表明规则并非普遍拒绝一切，但特定持续偏差会长期冻结。
实际新增等待、任务到达时间、deadlock和碰撞变化均为**Unknown**；上述是原轨迹上的限制proxy。

120例全部在t=0出现第一个不同dispatch：新mode处于cold-start，先持有/派发已有初始backup，
原baseline已经接纳主动作。之后原state不能作为新策略的真实state。
因此本重放逐拍重新以原owner/queue为条件，不维护一条伪造的counterfactual物理轨迹。
**没有报告“减少了多少碰撞”或“增加了多少实际等待”。**
初始化可信历史需要真实取得，不能把episode后的观测倒填给t=0。

### 计算、可复现资产与验收

mode计算27,338拍：mean0.02812ms、p50=0.02624、p95=0.03541、p99=0.05347、max0.81530ms。
该计时包括因果输入拷贝、ledger与模式估计，不含离线projection/IO；未测试完整在线周期CPU/RSS或executor实时性。
阶段没有进入prototype，不据此声称通过20Hz系统验收。

[source](../../research/dynamic_state_time/offline_information_mode.py)、
[Phase 1 summary](../../research/dynamic_state_time/results/information_mode/phase1_summary.json)、
[Phase 2 summary](../../research/dynamic_state_time/results/information_mode/phase2_summary.json)、
[逐例authority](../../research/dynamic_state_time/results/information_mode/authority_episodes.csv)、
[代表性时间线](../../research/dynamic_state_time/results/information_mode/representative_authority.json)、
[数据manifest](../../research/dynamic_state_time/results/information_mode/dataset_manifest.json)。
所有逐拍mode/authority明细和初版核查数据保留在仓库外；没有搬运历史raw archive。

最小验证：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 research/dynamic_state_time/offline_information_mode.py test
```

新空数据目录的重建顺序：`phase1 --raw <原manifest来源> --derived <仓库外新目录> --output <轻量输出目录>`；
仅Phase 1过门后运行同参数的`phase2`。
脚本拒绝覆盖封存的mode与authority明细；不要用重跑覆盖初版负结果。
验证覆盖因果prefix/隐藏元数据隔离、重复source不能恢复、missing持有/timeout、
ID重命名不继承速度可信度、旧模型反证、不复活invalid、missing时钟不刷新及空世界覆盖拒绝。
复核确认原120 scenario/seed/plant/观察扰动/证书标签未改变，协议SHA保持一致；
Phase 1评价修正前后的逐拍mode完全相同，Phase 2修正只移除不满足既定覆盖合同的保留机会。

### 决定

Phase 1通过；Phase 2总体unsafe veto59.05%（门75%）、stop/start13.54%（门75%）、
已知链有局部安全reserve机会24.14%（门50%）均失败。
还存在5次原CV假设仍成立但truth不安全的保留，且持续偏差会引起长时间限制。
稳定场景干扰、stale一致性与单mode计算成本则有正信号。

**Modify，当前方案不进入prototype。**
值得保留的是信息模式与action authority必须区分的形式化，以及分类不等于backup validity的反证。
下一立项门需要先解决：当前可见信息一致时，旧certificate究竟承诺哪段时间、哪些未观测行为与哪些目标集合；
无法给出覆盖证据就必须unavailable，不能靠延长quarantine、放宽阈值或默许stale来救结果。
本轮不实现新的certificate族，不扩大状态机，不跑闭环episode。

后续若另获研究Go，仍依次需要Software Go、common simulation、real causal tracker、目标机器/实车验证和main merge decision；
本轮没有授权这些阶段，不自动接STVL/Nav2或production。
