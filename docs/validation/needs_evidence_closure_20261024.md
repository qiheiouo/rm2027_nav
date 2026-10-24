# Needs-Evidence Closure Campaign（2026-10-24）

阶段 Integration；只处理 AUD-008/016/031/040/052/054/056 的证据缺口。
**本轮没有一项足以无条件升级为 Resolved**：4 Partial、1 Feature-gated、
1 Hardware-blocked、1 Still needs evidence。新增可复用软件证据，但没有修复第三方算法。
分类变化不等于缺陷消失，也不减少需要整改的已知软件问题。

## 基准与输入

- 当前 `main == origin/main == d37f8d6e5ebe53533b7fc62aca8bb84392bee31b`；本轮未 fetch，
  此处是本地 ref 身份。最新 software-debt-closure 为 `fed2cff`，是该 main 的后继。
- 本轮独立分支 `codex/needs-evidence-closure-20261024` 从 `fed2cff` 继续，
  保留上轮已验结果，不重写 Mission、策略、仲裁或定位实现。main 没有变化。
- 原始定义、工程处置和软件审计收口分别读取自
  `/home/qihei/Documents/Codex/2026-10-09/rm2027-navigation-rm2027-navigation-rm2027-agents/outputs/`
  下 `audit_findings_20261008.md`、`audit_engineering_disposition_20261009.md`、
  `audit_software_closeout_20261009.md`，并实际打开相关历史 JSON/夹具/源码。
- 最新 reconciliation 为另一工作区尚未提交的
  `/home/qihei/.codex/worktrees/historical-debt-round2/rm2027_navigation_clean/docs/validation/historical_debt_round2_20261024.md`；
  本轮读取快照 SHA256 `0dbfee7ea37cae755388a66836b0766619e52a547f270f35309187d321d9cec9`。
  该工作区的 034/044 等唯一修改没有迁入，也没有修改或提交它们。
- 继承结果：[software_debt_closure_20261024.md](software_debt_closure_20261024.md)。
  本文的 61 项总表只是在上述对账上应用上轮已提交结果和本轮 7 项裁决，**不是重新全仓审计**。

当前 pin：FAST-LIO `9aa8c79`、ikd-Tree `0438b0d`、Livox `ffb5c72`；
均未修改。新 worktree 未初始化子模块，只读使用主工作区同 pin 的 clean 源码。
实际 Nav2 包仍为 Humble 1.1.20；没有因为旧报告说有问题便假定当前版本仍相同。

## 逐项重新裁决

分类编号：①现有证据足以判定；②新增 software-only 验证；③缩小到明确 feature 条件；
④需要真实 bag/设备/目标机数据。一个 AUD 可以同时存在软件和硬件子边界。

### AUD-008：Partial（②，余项按③保留）

- **原始 claim**：已注册测试未覆盖关键节点退出、授权末端与旧串口背压；
  旧 colcon 通过项不能证明这些故障链已覆盖。它是测试资产缺口，不是一个独立运行 bug。
- **当前实际实现**：已有定位有效性、AUD-003 V2 最终编码守卫、Livox lifecycle/并发、
  Mission lease/取消终态等定向证据；`tests/localization_validity` 和
  `tests/aud003_final_motion` 是显式手动入口，不能全算默认包级注册测试。
  legacy/HPM `writeAll` 仍存在无界忙重试，没有相应背压整改/默认回归。
- **原来为什么 Needs evidence / Partial**：旧测试统计和当前真实启用链不能一一对应；
  历史人工 probe 没有全部进入可复用注册入口，新增测试数量也不能代表末端覆盖。
- **本轮新增证据**：在 `rm_relocalization_bridge` 默认 CTest 注册实际 gate/bridge 回归：
  健康正对照 → SIGKILL gate、raw/odom 继续 → bridge 失效 → 重放过期 status 不续租
  → 新实例无 pose 仍失效 → 新鲜 pose 才恢复。连续 3 次通过；最后一次失效约 0.978s，
  清理退出码 `[0,-9,0]`，无额外强杀。另注册 8 个显式依赖诊断，见后文。
- **仍缺什么**：legacy/HPM 背压、所有未选 profile、完整真实末端退出/物理停止；
  V2 byte-level 证据不替代 UART/MCU。故只关闭“定位 gate 退出缺默认回归”子项，整体 Partial。
  不在此任务中处理 AUD-004 或重做已闭合的 001/003/020/050。

### AUD-016：Partial（①＋②；仍有软件缺口）

- **原始 claim**：Nav2 costmap 非 current 时等待阻塞取消；恢复后 controller 先计算旧目标速度，
  planner 的首图等待也可能在取消 ACK 后返回 SUCCEEDED。
- **当前实际实现**：固定 Nav2 1.1.20 没有本地等待修复。
  上轮 Mission 屏障只管理它直接持有的外层 action；统一 smoother 路由也没有修改
  planner/controller 内部取消语义。默认 Voxel 不等于“所有 costmap 等待均不可达”。
- **原来为什么 Needs evidence**：主机缺陷已有充分反例，但是否需要 fork Nav2 取决于
  所选恢复 SLA、撤权/隔离和末端影响；旧 4 个 smoother/V2 调度及默认 BT 联合观察均未见
  非零末端，既不证明安全，也不证明车会复动。
- **本轮新增证据**：当前原样 `nav2_phase2f_deployment.yaml`、实际 planner/Navfn/global costmap，
  首图暂缺、当前合成 TF；cancel ACK 成功后继续等待约 0.803s 仍无终态。
  地图恢复后旧请求 SUCCEEDED、18 poses；健康下一请求也 SUCCEEDED。
  该反例成为显式 CTest 诊断。没有重跑旧 4 个末端时序，也没有修改 current/时效阈值。
- **仍缺什么**：可取消等待或经过验证的受管隔离/重启合同、内部 action 未知时禁止重入、
  明确恢复 SLA。当前没有该完整软件闭环，不能把问题全部转成 Hardware-blocked。
  本次不含 controller、速度、串口或 MCU；旧 raw 脉冲未穿透的结论保持其原范围。

### AUD-031：Still needs evidence（②；资源接受门未满足）

- **原始 claim**：FAST-LIO 一路失联时其他应用输入队列无容量/年龄上限。
- **当前实际实现**：bundle 的实际 callbacks 预处理后持续 push，sync 在另一输入缺失时返回 false；
  首次 LiDAR 前 IMU 保护、rollback 闭锁和可选 IMU gap 门均不能限制“LiDAR 停、IMU 持续”的队列。
  配置没有已验证的故障隔离期限/资源预算；DDS depth 不限制已经被 callback 消费的 deque。
- **原来为什么 Needs evidence**：旧方法夹具已证明计数增长，但使用小型 message/preprocess stub；
  未给出目标机真实 cadence、内存预算、故障窗口或恢复接受合同，因此不能凭计数宣布 OOM，
  也不能凭未发生 OOM 接受它。
- **本轮新增证据**：重新编译完整当前 bundle callbacks/sync，使用真实 ROS 消息、PCL、
  固定生产预处理库；每次循环都调用实际 sync。健康单路正对照先成功，随后：

  | 故障 | 有界注入 | 实际保留 |
  | --- | --- | --- |
  | LiDAR 静默、IMU 连续 | 20,000 个 IMU、500Hz 的 40s **逻辑源时间** | IMU 20,001；sync 一直 false；高水位 RSS 约增加 7.5MiB |
  | IMU 静默、LiDAR 连续 | 200 云、10Hz 的 20s **逻辑源时间** | 200 云 / 59,800 个预处理后点，时间队列一致 |
  | dual 第二路缺失 | 200 左云及 IMU | 201 云 / 60,099 点 / 202 IMU；无 epoch 闭锁 |

  这些是加速 callback 注入，**不是 20/40s 实时 ROS 流、目标机负载、OOM 或完整估计恢复测试**。
  3 个进程正常退出。没有为了清空队列而改同步/质量门。
- **仍缺什么**：目标 cadence/点规模、允许失联时长与内存预算，或已实现并验证的外部限时停止。
  软件缺少上限已确认；“能否接受、何时必须整改”仍不能判定，继续保留 Still needs evidence。
  async/adaptive 未选入口没有扩展验证。

### AUD-040：Feature-gated（①＋③，有明确条件）

- **原始 claim**：ikd 后台空有效点重建跳过重新加锁，却执行未持锁 unlock。
- **当前实际实现**：嵌套 ikd pin 未变，空/非空分支锁状态差异仍在。
  当前 bundle 的 FOV 入口只有在距 local cube 某边不超过 `1.5 * det_range` 才调用盒删除。
  single/old-car/dual YAML 均为 cube=1000m、det_range=100m；本轮动态验证只用 single。
- **原来为什么 Needs evidence**：完整库公开 API 的受控调度反例已充分，缺实际 FOV 是否能清空
  子树并命中该调度的证据；人工强删除不是正式 FOV 输入，不值得再重复相同库反例。
- **本轮新增证据**：执行当前完整 `lasermap_fov_segment()`，初始化估计 LiDAR 原点后，
  六个正负轴方向在 349.99m 不产生删除盒，350m 各产生 1 个盒，与源码分支一致。
  精确排除条件为：**源码计算并转换为 float 的六个窗口边距均严格大于 150m**。
  此时 FOV 不调用删除，不能触发本项的删除→空重建链。350m 是零原点窗口的名义位移边界；
  不把理想实数的“位移严格小于350m”当成浮点舍入/任意初始化原点下的保证。
- **最终 gate**：缩为“启用 FOV 裁剪/其他盒删除 consumer，或失去上述估计位移条件之前”
  必须单独接受库修复或提供可达性/并发证据。`use_lio_backend=false` 的入口也不执行该库。
  **这是条件裁决，没有新增运行时边界 fence，也没有证明实际场地/漂移始终满足它**。
  位移是估计坐标，不是车辆物理走程；不能用“小场地”常识授权所有 FAST-LIO 场景。
  库错误没有 Resolved，跨界/漂移/改 cube、range/API consumer 即重新开启此 gate。

### AUD-052：Partial（②；确定软件错误仍未修复）

- **原始 claim**：反向去畸变遗漏零 offset 首点，或到 begin 后只跳出内循环而重复补偿首点。
- **当前实际实现**：共享 `ImuProcess` 中 strict `>` 和内层 begin/break 仍在；
  038/059 初始化修复没有改这些遍历边界，三个 executable 仍共享该 header。
- **原来为什么 Needs evidence**：旧 7-case 使用原完整方法、PCL/Eigen、理想 predict 载体；
  真实预处理后最早有效 offset 分布和定位误差预算未知，不能从单点毫米差推出整车影响。
- **本轮新增证据**：改用真实当前 `ImuProcess` 和 ESKF，显式正确初始化重力、零杆臂/偏置、
  constant velocity 的独立解析 oracle。single/dual、静止/0.1m/s、首点 0/5/25/50/75/100ms，
  共 24 个数值场景：18 个对照符合 oracle；6 个已知错误场景分别为两种 lidar 模式的
  0ms（10mm）、50ms（5mm）、75ms（2.5mm）。50ms 处 source time 转 double 后略低于
  0.05s，strict `>` 让 begin 再次进入早期区间，是本轮补出的边界证据。
- **测试有效性修正**：初次夹具把 manifold 默认重力方向误当成已初始化方向，frame-end 状态
  正对照失败；显式设为竖直重力后通过。随后 50ms 被误列为正对照，解析逐点差确认其为
  另一个同根因边界；保留并断言具体误差，没有放宽误差阈值或修改生产算法。
- **仍缺什么**：若要 Resolved，需要遍历修复及其回归，超出本轮只补测试的实现范围；
  真实 bag 的 offset/运动分布及影响预算用于判断优先级，不能抹去已经确认的软件错误。
  不把 synthetic oracle 当作真实估计精度或 Hardware Go。

### AUD-054：Hardware-blocked（①＋②＋④，仅真实发生性/模式接受门）

- **原始 claim**：Livox 同步聚包必须精确命中毫秒 modulo 相位，未命中不发布，点持续保留。
- **当前实际实现**：020/050 的退出/并发修复没有改 `PubHandler::CheckTimer()` 的相位条件。
  时间模式由实际 packet `time_type` 选择；配置 time_sync enable/method 不能证明 firmware
  实际产出的 packet phase。非同步 wall-time 路径存在，但本轮没有接受它作为硬件替代方案。
- **原来为什么 Needs evidence**：旧 4-case/raw 线程已证明合法合成 PTP 错相位停发；
  真实 firmware/PTP/source cadence/分组与发布进展没有数据。
- **本轮新增证据**：编译完整当前 `pub_handler.cpp`，仅以 SDK observer 注册载体喂合法合成包。
  10/50Hz × 首包 phase 0～9ms，200 包/每包2点、10ms 源步长：2 个对齐正对照有发布；
  18 个错相位组合均 0 callback，随后对齐包一次释放全部 402 点，无点丢失。
  原 raw worker 与当前生命周期退出都实际执行，未初始化或链接真实 SDK 数据入口。
- **仍缺什么**：真实 packet capture 或带来源/固件/PTP 元数据的合法 bag，关联
  callback→packet grouping→ROS publication/backlog；覆盖同步/重连/模式变化。
  已知软件算法缺口仍保留，Hardware-blocked 只标识“是否在所选设备模式触发、是否可接受”
  的剩余门，**不表示必须等硬件才能修相位算法**。本轮不凭合成相位修改传感器时间语义。

### AUD-056：Partial（①＋②；仍有软件缺口）

- **原始 claim**：Nav2 BT 接受超时后丢 future，迟到目标没有被 halt 取消。
- **当前实际实现**：安装的 1.1.20 `BtActionNode` 超时 reset future；halt 只有已取得的
  goal_handle 才能取消。当前 YAML 的 default_server_timeout 仍为 20ms。
  项目 Mission 的迟到接受/终态屏障没有修改 Nav2 内层 BT，也没有完整 pipeline 管理重启。
- **原来为什么 Needs evidence**：真实 ROS client 的 121ms 延迟反例和默认树重试/UUID 输出隔离
  已充分；未知的是目标 CPU 的接受分布、完整隔离/恢复 SLA 与终端后果。
- **本轮新增证据**：当前安装 header + 真实 ROS action client，0/80/240ms 接受延迟。
  0ms 正对照 halt 收到 1 次 cancel；80/240ms 均在 BT FAILURE/halt 后才接受，
  分别仍完成旧请求、0 cancel。显式 CTest 注册；服务端调度/Spin 执行是无运动 fixture，
  不冒充实际 Nav2 SimpleActionServer、行为执行或多个真实 goal 同时 active。
- **仍缺什么**：迟到 future reconcile，或未知 owner 时完整阻断并管理重启 pipeline 的
  软件接受证据；真实延迟/物理停车另属目标机/MCU 门。不能通过单纯提高 timeout、
  外层 Mission CANCELED 或一次 V2 禁行来宣称 Nav2 BT 已修复。

## 新增验证与可复现边界

- `rm_relocalization_bridge` 受影响包单独构建通过；新增默认注册退出回归连续 3 次通过。
- `tests/needs_evidence_closure` 独立 CMake 的 8/8 CTest 最终通过：3 queue、1 FOV、
  1 deskew（24 数值场景）、1 Livox phase（20 组合）、1 BT late accept（3 延迟）、1 actual planner。
  **这 8 项是已知行为/缺陷刻画，不是 8 项修复验收**；用 COLCON_IGNORE 与默认包测试隔离。
- 环境复用 `rm2027_navigation:humble@81b325bebf2f`，GCC11/PCL1.12/Nav2 1.1.20；
  禁网、私有 IPC、无设备、非 root、cap-drop ALL、no-new-privileges、2 CPU/2GiB。
  只读源码；复用 10-21 同 pin 的消息依赖和预处理库，重新编译实际被测方法。
- 没有新 bag 回放；当前仓库 artifacts 未发现可用于本轮 source phase/真实 offset 接受的 bag。
  历史 JSON 与脚本用于核实原证据，不将历史通过数重写成本轮实测。
- 初次原生构建缺少 source include 目录，补正夹具 include 后构建通过；未改依赖源码。
  deskew 两次夹具判定修正见上文。未使用失败运行宣称测试通过。
- 有限进程正常回收，producer 的 SIGKILL 是预定注入，清理无额外 SIGKILL；
  planner 正常退出 0。没有共享 host ROS domain、启动 driver、UART 或硬件运动。
- 运行证据留在 `/home/qihei/projects/rm2027_navigation_clean/build/needs_evidence_validation/`，
  包级 JSON 在 `producer_tmp`；原生数值在 `native/Testing/Temporary/LastTest.log`，
  planner JSON 在 `native/planner/result.json`。不提交 build、XML、重复日志或点云。
  [测试入口与准确命令](../../tests/needs_evidence_closure/README.md)。

## 全部 61 项状态重计

每行一个主状态，**针对本分支**；“main/继承基线”列保留未被 main 接受的事实。
Resolved 仅指写明的软件范围；Partial 即仍未关闭，不能折算成 Resolved。
其余条目只继承最新对账/上轮收口，没有重新审计或整改。

| AUD | 历史 claim 摘要 | main / 继承基线事实 | 本分支最终主状态 |
| --- | --- | --- | --- |
| 001 | gate 消失后桥仍健康 | main 软件已闭合；继承 | Resolved |
| 002 | Mission 缓存安全状态/目标 | main Partial；上轮 lease/原子源已闭合 | Resolved |
| 003 | 最终串口缺运动授权 | V2 dry-run 已有；真实 owner/profile、legacy 仍缺 | Partial |
| 004 | 旧串口背压忙重试 | legacy/HPM 启用前保留 | Feature-gated |
| 005 | binary PCD 弱检查 | 外部 PCD/GICP 接受前保留 | Feature-gated |
| 006 | cap 停止未通知 sampler | managed mapping 未闭合 | Feature-gated |
| 007 | 未来 pose/过期 pending | main 软件已闭合；继承 | Resolved |
| 008 | 关键退出/末端测试覆盖 | 本轮注册 gate 退出；整体覆盖仍不完整 | Partial |
| 009 | 文档称 referee parser 缺失 | 当前文档事实已取代旧表 | Superseded |
| 010 | reset ACK 前清地图 | managed mapping 未闭合 | Feature-gated |
| 011 | 取消先丢 owner | main 未改；上轮终态屏障已闭合 | Resolved |
| 012 | canonical covariance 未换基 | 消费/表达传播前保留 | Feature-gated |
| 013 | FAST covariance 发布顺序/排列 | 独立研究未接受 | Feature-gated |
| 014 | readiness 接收旧 odom 当新鲜 | main 未改；上轮 odom 软件范围闭合 | Resolved |
| 015 | covariance 非对称/非 PSD | main 未改；上轮数学合法性闭合 | Resolved |
| 016 | Nav2 costmap 等待取消 | 本轮确认当前依赖仍有缺口 | Partial |
| 017 | seeded GICP 不证明地点 | seeded/独立 coarse prior 技术局限 | Accepted |
| 018 | 狗洞 stale latest TF | 仿真候选启用前保留 | Feature-gated |
| 019 | 狗洞迟到 goal 未取消 | 独立 owner 未修复 | Feature-gated |
| 020 | Livox poll/join 不退出 | main 软件已闭合；继承 | Resolved |
| 021 | 双云混用车体时刻 | dual obstacle 未闭合 | Feature-gated |
| 022 | STVL 旧 cloud 仍 current | 默认 Voxel 不消除 STVL 问题 | Feature-gated |
| 023 | STVL 可选依赖未声明 | 可选 profile 安装前保留 | Feature-gated |
| 024 | T 增强健康门不失效 | main 无该研究增强门 | Not applicable |
| 025 | organized cloud 行填充 | 紧凑 MID360 输入条件保持 | Accepted |
| 026 | 重复顶点退化边 | 当前配置条件保持 | Accepted |
| 027 | 未观测角当清障射线 | 实验 local_scan 条件保持 | Feature-gated |
| 028 | T 坡道模型不过期 | main 无该研究模块 | Not applicable |
| 029 | license TODO/Proprietary | 权利人决定；非证据测试可代替 | Open |
| 030 | FAST rollback 混 epoch | bundle 闭合，async/adaptive 未覆盖 | Partial |
| 031 | 输入等待队列无界 | 缺资源/失联/恢复接受预算 | Still needs evidence |
| 032 | dual 拼帧 offset 未重基 | dual-LIO 启用前保留 | Feature-gated |
| 033 | PCD/栅格无共同切点 | managed mapping 未闭合 | Feature-gated |
| 034 | 正常信号退出报异常 | main Open；原工作区唯一 candidate 未采用 | Open |
| 035 | 重叠区朝向约束放宽 | main 无 path_annotations | Not applicable |
| 036 | DDA 零穿行邻格自由 | 被拒研究路线未恢复 | Not applicable |
| 037 | 旧自由证据删新障碍 | 被拒研究路线未恢复 | Not applicable |
| 038 | 首次 IMU 读未初始化状态 | main 软件已闭合；继承 | Resolved |
| 039 | 动态准入旧 pose/当前 ETA | main 无该 consumer | Not applicable |
| 040 | IKD 空重建未持锁解锁 | 本轮给出 FOV 不调用删除的严格条件 | Feature-gated |
| 041 | 第40帧静默重锚 | 上轮 default single 闭合；old/dual 仍 gated | Partial |
| 042 | 离散净空非连续保证 | main 无旧准入 consumer | Not applicable |
| 043 | T-DT SfcSquare 错误 | 仅隔离冻结快照 | Not applicable |
| 044 | 地图阈值等值分类错误 | main Open；原工作区唯一 candidate 未采用 | Open |
| 045 | Spin 状态被 holding 覆盖 | main 未改；上轮实际状态回归闭合 | Resolved |
| 046 | clear 后越界写向量 | bundle 闭合，async/adaptive 未覆盖 | Partial |
| 047 | IsPathValid 无 pose 仍 true | 正式树无 consumer 条件保持 | Accepted |
| 048 | heading 仿真绕过扰动 | 仿真能力限制保持 | Accepted |
| 049 | 无净空观测仍 success | 狗洞研究接受前保留 | Feature-gated |
| 050 | Livox volatile SPSC 索引 | main 软件已闭合；继承 | Resolved |
| 051 | 多设备共用 IMU 旋转 | 零 SDK 外参/单主 IMU 条件保持 | Accepted |
| 052 | 首点遗漏/重复 deskew | 本轮真实 ESKF 确认错误，未改算法 | Partial |
| 053 | IKD ties 精确删除漏点 | 当前 FOV 不用精确点删除 API 条件保持 | Accepted |
| 054 | 同步聚包错相位停发 | 真实设备 phase/cadence/模式接受未验 | Hardware-blocked |
| 055 | behavior 绕 smoother | 上轮 generic/legacy Gazebo 闭合；其他组合未验 | Partial |
| 056 | BT 迟到接受丢 future | 本轮当前 header/ROS client 确认未修复 | Partial |
| 057 | sim 刷新旧速度 | 仿真故障模型限制保持 | Accepted |
| 058 | shadow 速率告警仍 GOOD | main 无该 integrity 模块 | Not applicable |
| 059 | 非法 IMU 完成初始化 | main 软件已闭合；继承 | Resolved |
| 060 | derived YAML 被 launch 覆盖 | main 未改；上轮 YAML/CLI 软件闭合 | Resolved |
| 061 | GICP cloud producer 默认关闭 | 上轮 generic single 闭合；其他 producer 另验 | Partial |

| 主状态 | 数量 |
| --- | ---: |
| Resolved | 12 |
| Partial | 10 |
| Feature-gated | 16 |
| Hardware-blocked | 1 |
| Still needs evidence | 1 |
| Open | 3 |
| Accepted | 8 |
| Not applicable | 9 |
| Superseded | 1 |
| **合计** | **61** |

一行一个主状态不代表只有 AUD-054 有硬件依赖：003 的 MCU/物理停车、真实 radar、
source 时钟、机械标定、prior/query、地图/感知/导航及目标机负载等跨项硬件门全部继承。
本文没有将这些门重复计入 AUD 总数，也没有修改原 Accepted/Not applicable/Superseded 条件。

## 交付与限制

相对 `fed2cff` 只新增测试/诊断、必要测试依赖和本报告/索引，没有生产行为或比赛策略修改，
没有新功能、没有依赖 pin 变化、没有 merge main 或 push。
本轮可审查资产为该分支的干净提交；原工作区和其他候选工作区保持原状。
未完成的门已逐项写明；当前没有 Deployment Go，不自动进入下一开发阶段。
