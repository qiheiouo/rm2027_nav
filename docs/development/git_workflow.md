# Git 工作流与研究资产

本页承接原 development_workflow 的 Git 规则和 main 的研究资产原则。
实现取舍见[日常开发原则](development_principles.md)，实验与验证见[研究与验证](research_and_validation.md)，
文档/开源引入规则见[文档策略](documentation_policy.md)。

## 正式开发入口

当前工作站以 `/home/qihei/projects/rm2027_navigation_clean` 为正式开发目录，
`origin` 必须指向 `git@github.com:qiheiouo/rm2027_nav.git`。旧 Gitee checkout 仅作历史冷归档，
不得继续作为新功能分支的默认创建和推送入口。

日常开发从新主仓最新 `main` 开始：

```bash
cd /home/qihei/projects/rm2027_navigation_clean
git switch main
git pull --ff-only
git switch -c feature/<topic>
```

Codex 创建的分支使用 `codex/<topic>`；研究和集成分别使用 `experiment/<topic>`、
`integration/<topic>`。首次发布功能分支时只推送当前分支：

```bash
git push -u origin HEAD
```

旧仓已有成果只按固定 SHA 选择性迁移到从新 `main` 创建的分支，保留来源记录并重新验证；
不要使用 `--allow-unrelated-histories` 合并两套根历史，也不要把旧仓 `origin` 直接改成新主仓。

## 实车验证后进入主线

功能分支完成软件、仿真和所需实车验证后：

1. 在原分支更新已有 validation / contract，明确车型、配置、环境、结果和 Hardware Go/Modify/Stop。
2. 合入最新 `origin/main` 并解决冲突；已共享分支不通过 force push 改写历史。
3. 按功能风险重跑受影响构建、核心回归和必要实车确认，确保默认关闭、安全 owner 与回退路径未变。
4. 将该分支单独提交 GitHub PR；一次只合入一个可解释功能，不批量合并旧研究历史。
5. PR 通过后用普通合并进入 `main`，再删除临时分支与 worktree 前确认没有唯一数据。

旧仓分支不能直接 merge 新主仓；先把经过实车验证、确有价值的最终实现选择性迁入新仓集成分支。

## 分支与提交

- 操作前确认 branch、worktree 和 status；保留无关用户修改。
- main 保持可构建，保护已接受功能；新旧车事实按各自长期分支维护。
- 通常按目的使用 feature/*、fix/*、experiment/*、integration/*、docs/*；Codex 工具新建分支默认用 codex/ 前缀，用户指定优先。
- 一个研究方向一般只维护少量重要实验分支，中间 audit/test/evidence 状态用 commit，不永久创建远端 branch。
- 一次 commit 一个有意义的逻辑变化，提交前看近期 log、staged diff 和 status。
- 默认一行简短中文标题，不用长篇 body 或大量 commit 制造工作量。
- 只提交任务相关内容，不混入无关格式化、generated logs、build/install/log 或 cache。

示例：

```text
文档：整理开发规范与文档结构
修复：统一地图到里程计变换所有权
实验：验证TinyMPC急转弯性能
```

## 历史与上传

不默认 force push、rebase 已共享历史、rewrite old history、为统一语言修改旧 commit、
大量创建 tag，或把所有本地分支上传远端；禁止默认执行 `git push --all`。
不自动 push/merge；获得明确授权后只操作确认的目标引用。

有长期价值的代码和唯一证据应形成可恢复 checkpoint，并提出备份需求；这不是自动上传授权。
冻结路线保留少量代表状态，tag 仅用于重要里程碑；不要求每个实验都建 tag 或新的远端入口。

## Worktree 与研究结束

Worktree 必须有明确用途，不充当无限增长的归档入口。
研究成功时只提取真正需要的修改进入干净集成分支，不整体 merge 长期失败实验和 evidence 历史。
失败或暂停时，更新已有路线总结，保留关键代码状态和必要证据，不继续无目的扩充回归。

长期最低资产为：**总结 + 关键代码状态 + 最小关键证据**。
保留解释结论、恢复关键实现和复核决定性结果的信息，不保留全部中间过程。
既有冷备份保持可恢复，不因每次小实验重做完整归档。

清理无价值临时 worktree 前，确认没有唯一 dirty/untracked 数据且有明确授权。
不自动删除用户 branch/worktree，不为“Git 好看”清理或改写历史。

## 大文件

bag、大量日志、build/install/log、大型仿真输出、原始点云、临时录屏和重复数据集不得直接进入普通 Git。
长期必要的大资产按需求用 artifact store、Release、Git LFS 或冷备份，仓库只放索引和必要元数据。
只有小型代表性证据且确有长期价值时才随源码保存。
已有原始证据不因本规范自动删除；删除前确认可恢复性与任务授权。
