# 开发日志索引

`.dsh/logs/` 是**面向开发者的过程记录**：为什么这么做、试过哪些死路、根因分析。

- 每条日志是**当时的过程快照**，发现写错了**不回改正文**，在当天或新日志里说明（见 [`../agents/writing.md`](../agents/writing.md)）
- 现行事实与设计在 [`../agents/`](../agents/)；本目录只解释「怎么变成现在这样的」
- 与 `.dsh/agents/` 不同，本索引是**手工维护**的（本仓库没有生成脚本）：新增日志后回来加一行

| 日期 | 日志 | 一句话 |
| --- | --- | --- |
| 2026-09-25 | [agents-system-bootstrap](2026-09-25-agents-system-bootstrap.md) | 从赛题 PDF 与参考项目搭建披露式 agent 知识体系，并把方法论蒸馏成 `dsh-agent-init` 技能 |
| 2026-09-25 | [refactor-perception](2026-09-25-refactor-perception.md) | 逆向参考项目算法，落地感知层（config/arena/vision）与 28 个行为测试 |
| 2026-09-25 | [refactor-control-reporting-fsm](2026-09-25-refactor-control-reporting-fsm.md) | 重构控制、上报、状态机与平台层，闭环仿真跑通；弯角仍未通过（可执行的已知失败） |
| 2026-09-25 | [corner-heading-sign](2026-09-25-corner-heading-sign.md) | 修掉内环航向正反馈等四个结构性缺陷；接入航向源按角度终止；偏左起始可跑完全程 |
| 2026-09-25 | [day-summary](2026-09-25-day-summary.md) | **收尾**：工具链、模块边界与十条耦合、实测结论；并写明今天的代码是参考实现、后面可能大改 |
| 2026-09-25 | [skill-repo](2026-09-25-skill-repo.md) | 给 `dsh-agent-init` 技能建独立仓库；仓库根为何选在技能目录（附发现规则的证据） |
| 2026-09-26 | [branch-gate](2026-09-26-branch-gate.md) | 新增开发者 README；把一次提交误落到 `main` 的事故与根因（检查没当闸门用）；`AGENT.md` 为何不能用 |
| 2026-10-02 | [control-concurrency](2026-10-02-control-concurrency.md) | 评估「两个线程互抢控制权」的设想并否掉（`aim()` 的 6 次发送会交错成「急转 + 全速」）；定下按时间尺度切线程、唯一写者 + 固定节拍 + 看门狗；查实 RC override 会超时 |
| 2026-10-02 | [package-and-entry-point](2026-10-02-package-and-entry-point.md) | 查证参考项目确有 4 个业务子包（我原先判断错了）；决定我方保持扁平包并留下拆分触发器；补 `pyproject.toml` + 命令行入口 + `--trace`；新增运行顺序文档 |
| 2026-10-02 | [hardware-facts-and-code-notes](2026-10-02-hardware-facts-and-code-notes.md) | 队内五条实物事实逐条核对（U 形与「⊐」一致、白色圆柱管道 vs 原文「LED 灯带」冲突）；`heave` 不再下发；新增 `darkness` 判别器（含 `val_band` 默认值的一次中途改正）；11 个模块加中文定位块 |
| 2026-10-02 | [vehicle-startup](2026-10-02-vehicle-startup.md) | 查证「丢到树莓派怎么启动」：列出 7 处空缺（**`--camera` 实际不碰硬件**最危险）；补 `run_armed`（解锁 + 必上锁）、`--vehicle`、`pyserial`、systemd 单元；修 `connect()` 没兑现的「返回失败而非抛异常」契约；**启动时机作为规则问题留待决定** |
| 2026-10-02 | [linear-rewrite](2026-10-02-linear-rewrite.md) | 线性重写：`src/` 4517 行 11 模块 → 1280 行 7 文件；主循环 `main.py` 从上往下读；仿真搬到 `sim/` 不入包；抓出上报合并、误判未启动、脱线计时三个 bug |
| 2026-10-02 | [doc-alignment-and-sectioning](2026-10-02-doc-alignment-and-sectioning.md) | 五份带横幅的文档正文对齐到 v2（设计理由保留、模块名换掉）；删掉一个死导入；**拆函数 vs 分节的取舍**（判断标准是「有没有私有状态」，不是行数）；分节用 AST 证明代码未变，但如实记下文件变长了 |
| 2026-10-11 | [doc-debt-and-param-tags](2026-10-11-doc-debt-and-param-tags.md) | 给 `params.py` 的 54 个待实测字段打上 `【待验证·实机】`（用字段级对比证明只改了注释）；按 `src/` 逐条清文档债（AGENTS 红线 1/7、README 安装、backlog 重写、四份带横幅的文档删横幅、补三条「设计在、代码不在」的 CAUTION）；记下四个未修的代码缺口 |
| 2026-10-13 | [tail-fin-implementation](2026-10-13-tail-fin-implementation.md) | **单关节尾鳍接口落地 + 两轮自测**：下位机只收一路 PWM，所以合成（`drive.synthesize_tail_fin`）与波形（`fin.py`，独立线程 100Hz）都在上位机；修掉三处真 bug（串口打不开仍报成功、坏参数静默杀死波形线程、预算约束从不触发的空实现），撤回一处自己的误判（「量级缩水 20 倍」不成立）；记下**耦合打开后仿真过不了弯角**的实测数据，据此默认关闭预算并加用例守住；提交前复审又查出一处只在真机出现的 TypeError（`tail_fin_budget=None` 被 `:g` 格式化），并验证了新用例真能抓住它。提交 `cda9218` |
