# 开发日志索引

`.dsh/logs/` 是**面向开发者的过程记录**：为什么这么做、试过哪些死路、根因分析。

- 每条日志是**当时的过程快照**，发现写错了**不回改正文**，在当天或新日志里说明（见 [`../agents/writing.md`](../agents/writing.md)）
- 现行事实与设计在 [`../agents/`](../agents/)；本目录只解释「怎么变成现在这样的」
- 与 `.dsh/agents/` 不同，本索引是**手工维护**的（本仓库没有生成脚本）：新增日志后回来加一行

| 日期 | 日志 | 一句话 |
| --- | --- | --- |
| 2026-09-25 | [agents-system-bootstrap](2026-09-25-agents-system-bootstrap.md) | 从赛题 PDF 与参考项目搭建披露式 agent 知识体系，并把方法论蒸馏成 `dsh-agent-init` 技能 |
| 2026-09-25 | [refactor-perception](2026-09-25-refactor-perception.md) | 逆向参考项目算法，落地感知层（config/arena/vision）与 28 个行为测试 |
