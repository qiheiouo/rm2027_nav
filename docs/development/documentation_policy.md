# 文档策略

优先更新已有权威文档。每份文档只承担一个主要职责，同一事实只保留一个维护入口。
执行优先级见根目录 [AGENTS.md](../../AGENTS.md)；查找入口见[文档地图](../README.md)。

## 三类文档

| 类别 | 职责 | 更新方式 |
| --- | --- | --- |
| Current | 当前分支的 architecture、contracts、runtime、operations | 随相关代码修改同步；明确车型和实验开关 |
| Research / Validation | 一次研究、实验或验证的条件、结果与限制 | 可带日期；不自动代表当前系统状态 |
| Archive | 历史过程、Phase 里程碑、冻结阅读副本 | 保留知识；不得作为当前接口或运行方式权威 |

当前事实按 branch/profile 理解，不能把旧车或研究配置写成全项目统一事实。
发现文档与源码冲突，说明差异；不要依据历史报告推断新的运行协议或验收结论。
公共接口变化更新对应 contract；启动/配置行为变化更新 operations；局部用法更新 package README。
只改受影响的权威文档，不要求每次同时修改索引、报告和所有 README。

## 不制造文档负担

- 不为每次 Codex 任务或微小改动新建 Markdown。
- 不重复生成 summary + report + final report + freeze report + closeout report。
- 不要求所有任务填写模板、checklist、报告或同一测试矩阵。
- 研究里程碑或需要未来复现的结论才值得单独记录；优先追加到已有路线文档。
- 新目录保持浅层；普通入口约 2～3 层，历史 Phase 分组和原始证据布局允许更深。
- 移动文档用 `git mv`，修复活动文档和 README 的相对链接及正文路径引用。

## 新主仓分类与保留例外

`phase1*`、部分 `phase2*` 和 `phase3*` 的历史验收过程进入 `archive/phases/`。
仍承担当前职责的内容直接迁为正式文档：地图 bundle 到 contracts，任务结构到 architecture，
地图部署、建图、重定位、串口 dry-run、双雷达及比赛 bringup 到 operations。
这些页保留原 Phase 来源和历史验收章节，不重复生成一套历史副本。
旧车早期计划/行为完成表进入 `archive/old_car/`，带日期的现场证据集中在 `validation/old_car/`。

根目录保留两处选择性迁入的研究目录作为路径例外：

- `dynamic_obstacle_critic/`：精选 critic 研究结论；不含旧仓全部 evidence 或未提交 WIP，也不属于正式运行入口。
- `dynamic_navigation/`：精选研究总结、冻结记录和少量必要索引；历史原文与大型证据按迁移 provenance 从旧仓恢复。

选择性迁移的冻结文档可能引用未迁入的原始上下文，应按 provenance 中的固定来源恢复，不伪造资产。
`research_snapshots/` 默认由 `COLCON_IGNORE` 隔离，不能作为正式 package 或 runtime owner。
未来新研究使用 research/ 或 validation/，不继续扩展根目录例外。

## 外部依赖与开源模块

仅对真正引入系统的模块，在 [external](../external/) 记录：upstream URL、branch/tag/commit、license、
引入范围、本地修改、adapter boundary、引入理由、替换或移除方式。
版本信息在该记录维护一次；不要在多份报告重复维护同一 pin。
只读调研记录在 research，不因调研就 vendor、添加 submodule、fork 或重复制作 upstream snapshot。
保留现有许可证、来源和本地修改记录；不为“架构完整”引入外部模块。

## 新旧仓职责与同步

- 新 GitHub 仓库是后续开发、集成、实车验证和正式主线的唯一入口。
- 旧 Gitee 仓库及其 worktree 保留完整历史和未迁入研究资产，只作为冷归档与固定来源。
- 旧仓新产生的有价值提交不得整体 merge 两套不相关历史；应从新仓最新 `main` 建立分支，选择性迁移提交或文件并重新验证。
- 新功能、文档和验证结果只推送新 GitHub 对应分支；不得把旧仓全部 refs 或 evidence 批量导入。
- 老车、新车和研究配置仍通过 profile、合同和独立功能分支表达，不能把某个历史分支当作第二条长期主线。

本目录四份短规范替代原先重叠的长版流程文档。旧英文历史和冻结报告保持原意，不为统一风格改写。
