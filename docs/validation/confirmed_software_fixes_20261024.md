# 三项已确认软件缺陷修复（2026-10-24）

Integration；从 `main d37f8d6` 开始，依据 Needs-Evidence Closure `82bc206`，
严格按 016 → 052 → 056 独立提交。收尾时只读查询远端 main 仍为 d37f8d6。不重新审计，不修改 Mission/比赛策略。
运行环境复用 Humble/Nav2 1.1.20 镜像，禁网、私有 IPC、无设备、非 root、
cap-drop ALL/no-new-privileges、2 CPU/2 GiB。软件证据不能作为 Hardware Go。

## AUD-016

正确合同：costmap 不 current 时可被 cancel、server 停用或 context 退出打断；
取消在等待结束前被观察到后，旧请求不得继续计算路径/速度；终态不依赖地图恢复。
不承诺抢占一个已进入执行的 controller/planner plugin，不以 action ACK 冒充物理停止。

根因：planner 等待循环仅检查 currentness，调用方不再检查 cancel；controller 内层等待
也绕过外层取消检查，恢复 currentness 后先执行一次旧速度计算。

最小复现：真实安装 planner/Navfn + 原 `nav2_phase2f_deployment.yaml`，首图缺失，
cancel ACK 成功，0.804s 后仍无终态；地图恢复后旧 goal SUCCEEDED、18 poses。
同一正确性回归在未修复 Nav2 上两种规划 action 均失败（取消等待未完成）。

修改：固定源码 overlay 中为两类 planner 等待传入本 action server；等待中检查取消、
server/context；controller 等待循环及其返回边界检查同样条件并先发零再终结。
保留原 100Hz 轮询频率，使用 WallRate，避免依赖暂停的 ROS 时间；未加入取消超时。
[补丁来源/加载/回退](../../dependencies/nav2_humble/README.md)。系统包本身未改。

回归入口：[真实服务器回归](../../tests/confirmed_software_fixes/README.md)。
planner 覆盖 ToPose/ThroughPoses 的缺图取消和新请求恢复；controller 使用显式启用的
currentness layer、setPlan 事件和计算计数。fixture 初版未设 enabled_，层被忽略；
该次非零不计作有效缺陷证据，修正后重新跑 baseline/修复版本。

结果：修复后 3/3 CTest 通过（4.72s）；两类 planner 的取消终态分别约 8.88/9.07ms，
controller 约 10.43ms，均在地图/currentness 未恢复时完成。controller 计算 0 次、非零 0 次，
恢复后新 goal 计算 1 次并可取消；三次服务器清理退出码均 0。这是主机观测，非 SLA。
修正 fixture 的启用标志和更新区域后，未修复 controller 也稳定失败于取消仍阻塞。

兼容性：对本地 main 与全部既有候选 ref 逐个临时 index 应用检查。
main、software-debt/needs-evidence、mission-layer、intent-arbitration、posture、
localization-acquisition/z-integration/upstream-parity、omni 及治理 ref 均无补丁冲突。
dynamic-surface / hws-shadow 的代码补丁无冲突，但仍使用重构前 docs/runtime_profiles.md，
完整提交迁入需将文档说明映射到该路径；不为本任务改动这些分支。
未改比赛配置/目标选择/动作参数；未声称执行所有候选分支的完整联合运行。
建议进入 main 的软件修复候选，前提是构建并加载对应 overlay；仍无 Hardware Go。

Hardware gate 保留：目标机所选地图/传感器更新与负载下的取消/恢复延迟、原速度链、
真实串口/MCU 停车和重新授权验收。本次未运行 smoother/串口/MCU。

## AUD-052

正确合同：有效 IMU pose 区间覆盖下，非负 offset 的每个点必须恰好补偿一次到共同帧末，
包含 offset=0、等于 IMU 区间端点和仅有一个点的扫描；保留排序及已有位姿公式。

根因：strict `>` 漏过零 offset；首点补偿后只 break 内层循环，游标仍指向 begin，
外层早期 IMU 区间可再次补偿它。50ms 时间转 double 的边界也是该游标耗尽错误的实例。

最小复现：原始真实 ESKF 正确性 oracle 在 single、0.1m/s、首点 0ms 上得到 10mm
误差并失败；继承 Closure 24 场景中的其他确认反例。没有以末端定位精度预算否认软件错误。

修改：仅共享 `IMU_Processing.hpp` 的 single / dual1 / dual2 三处反向游标，使用
rbegin/rend 表示已消费完毕并使区间边界包含零点。不重构模型/外参/噪声/预处理，
不修 dual 拼帧时间重基（AUD-032），不解锁 async/adaptive 的既有 gate。

新增回归为完整方法 + 真实 ESKF，解析匀速平移/匀角速 yaw oracle，包含原24格、
第二单雷达入口、singleton、重复/乱序、空云和旋转，共51场景。
完整三维夹具初次发现 S2 重力模长固定9.809而 IMU 归一化9.81，导致与本缺陷无关的
2.5µm竖直差；显式设置已知竖直bias抵消该差后重新验证，未改生产常量或放宽1µm阈值。

结果：51/51 数值场景通过，最大三维误差5.73e-8m（门槛1e-6m），帧末位置/旋转
正对照通过；默认 `laserMapping_bundle` 编译通过。第二时间入口按实际方法仍消费主
点云载体的合同构造 oracle，不在该测试中假设其直接读取 lidar2。
依赖 commit 在 [FAST-LIO 记录](../external/fast_lio_multi_ros2.md) 单点维护；只改该 header。
main、needs-evidence/software-debt、mission、intent、posture、Z-localization 相关 ref
当前均 pin 同一9aa8c79；无依赖覆盖冲突。未扩展到候选各自所有运行配置。
建议进入 main 的软件修复候选；入 main 前必须先发布该本地依赖 commit，保证 gitlink
可由新 checkout 获取。本任务没有 push，原子模块 checkout 未改变。

Hardware gate 保留：真实 bag 的 offset/运动分布、实测外参、真实估计精度和负载下时序；
本次合成解析 oracle 只闭合遍历算法，不是 Hardware Go。

## AUD-056

正确合同：接受超时、halt、对象销毁不能丢失请求归属；迟到接受后要取消，旧反馈/结果
不能写回 BT 输出；同一 BT node 在明确拒绝/终态前不得重发。取消 ACK 不是终态。
不通过增加 server_timeout、sleep 或外层 Mission 终态来关闭本项。

根因：接受超时 reset future，而 halt 只处理已知 handle，回调组在 halt 后又无人推进；
迟到接受的目标无取消 owner。原回调直接捕获 this，保留迟到处理时还必须防止
跨失败/销毁生命周期访问。

最小复现：真实当前 ROS client + 安装 BtActionNode，事件闸门扣住接受响应，BT 失败并
halt 后才释放；服务端接受旧请求但0次cancel，正确性回归失败。

修改：在固定 Nav2 overlay 的共享模板内保留 pending future/已有 handle，停止其输出
回调；正常已接受目标保持原有有界 halt 等待，未闭合请求转交局部清理任务并持续处理
回调，迟到接受即取消，拒绝或真实终态才解除本 node 的失败屏障。
清理仅持有 ROS client/node/group 和失效回调凭据，不访问已销毁 BT；context
pre-shutdown 先中断并收束清理任务、释放 DDS 资源，再允许 ROS 退出。
不更改比赛树、策略、action名、默认20ms参数、重试选择或最终速度 owner。

验证中正常 halt 的取消钩子被检查为仅接收有效取消结果；新夹具补齐服务器 handle
建立事件，避免把 goal response ready 误认为服务器侧 handle 已存入夹具。定向 ASan 发现本次修复初版独立清理线程与 ROS/FastDDS 退出重叠，已按上述
pre-shutdown 合同修复；不把该失败版本列为通过。
初次包编译的构建工具自动开多编译任务触及2GiB限制，改为 `cmake --build -j1`；
最终 BT 包使用 -O0 编译以控制验证资源，不用该结果声明实时性能。

结果：9类事件回归均通过，定向 ASan（关闭 leak 检查，仅验证非法访问/生命周期）
每类连续3次通过；没有用此前失败运行宣称通过。真实 `nav2_behavior_tree` 包及全部
action plugins 重编/安装通过；显式指定安装包的 CMake package_DIR，确认 header
来自修复安装目录（第一次手动安装后下游仍误选 apt，已纠正），没有用源 header 覆盖。
实际加载重建的 Spin plugin，迟到接受/取消、终态前阻断和新请求成功均通过。

最终组合 CTest **14/14通过，6.13s**：AUD-016的3项、AUD-052的51场景数值项、
AUD-056的9种事件调度和1项实际插件。planner/controller与BT修复共同加载，
受测服务器均正常退出，没有真实 driver/UART/MCU、最终物理速度或额外运动owner。
证据保留在主工作区 `build/confirmed_software_validation/`；准确构建/运行入口见
[回归说明](../../tests/confirmed_software_fixes/README.md)。生成日志/build未入Git。

兼容性：按当时所有本地候选 ref 的临时 index 检查累计补丁，main与主要候选可直接
应用，dynamic-surface/hws-shadow仅有旧runtime文档路径差异。BT模板改变C++派生类
布局，必须重建本包action插件，不能混入旧apt模板派生插件；消息/action ABI未变。
Needs-Evidence `82bc206` 的三项历史 characterization 明确断言“缺陷仍存在”，修复后
不能原样作为验收：迁入时停用旧016/052/056注册，改跑本次正确性regression；其余
queue/FOV/Livox刻画不在本任务重审。该分支未进行完整联合回归，不声称其原8/8仍成立。
建议进入main的软件修复候选，需部署同一Nav2 overlay；保留下面的硬件接受门。

Hardware gate 保留：目标 CPU 的接受/取消/恢复分布，Nav2内部执行到最终速度撤权、
UART/MCU及物理停车。迟到接受到cancel之间可能已经执行；本软件补丁不提供物理SLA。
服务端始终无响应/不终结时保持本 node 的失败屏障；没有添加自动超时解锁或pipeline重启。
