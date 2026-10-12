# 2026-10-11 · 文档债清理 + `params.py` 的「实机不确定」标记

**触发**：审阅者读完知识库与源码后问两件事——① 开测前要做什么（含部署到上位机）；② 哪些参数要重调、看什么来调。核对过程中发现**文档与 v2 代码有大量不一致**，于是分两步：先给 `params.py` 的待实测字段加统一标记，再清文档债。

**范围**：只动注释与 Markdown。**代码行为一行没动**（下面有 AST / 字段级的证明），**代码缺口一个没修**（审阅者明确要求先别动），**没有提交**。

---

## 一、`params.py`：54 个字段打上 `【待验证·实机】`

审阅者提的格式是「在每个变量上面加一句注释」，选定的写法是复用仓库既有的标记体系：

```python
    # 【待验证·实机】管道直径：卡尺实测
    strip_width_mm: float = 24.0
```

- **为什么不用一句散话**：`.dsh/agents/` 里已经有 `【事实】/【设计】/【待验证】` 三态约定，`【待验证·实机】` 是它在**代码里**的专用形式。好处有两条：`grep -n "【待验证·实机】" src/bionic_fish/params.py` 就是一份待标定清单；而且**「标定完必须把标记摘掉」这条纪律自动适用**（[writing.md](../agents/writing.md) 已补进约定表）
- **范围是 54 个，不是全部 76 个**。76 个字段分三档：6 个是赛题硬性数字（`lost_timeout_s` / `wall_hit_limit` / `start_timeout_s` / `run_timeout_s` / `max_reports` / `marker_diameter_mm`）；约 16 个是算法自身的选择（`roi_*_frac` / `lookahead_frac` / `min_rows` / `open_ksize` / `fit_residual_px` / `aspect_max` / `corner_refractory_s` / `k_lateral` …），它们由测试与现场调试定，**不随硬件改变**；剩下 54 个才是「值必须来自实测或真机联调」
- **图例写进了模块 docstring**，含那句 grep 与「没有标记的只有两种」的判据。没有图例的话，54 行光秃秃的标记没人知道定义

**中途改正过两处**（按本目录的约定要写下来）：

1. 我的第一版范围估计是「约 45 个」。真去数才发现按同一条判据数出来是 **54 个**——`pwm_min` / `pwm_max`（要对着实际执行器验）、三个非默认分割分支的阈值（`v_min` / `s_max` / `r_minus_b_min`）都算在内。**估计与清单不该混着用**，最后按判据出清单。
2. 第一版脚本按 `^    name\s*:` 匹配字段，**把模块 docstring 里的 YAML 示例（`ch_yaw: 4`）也匹配上了**，还没写完就自己触发了「已标记」断言。脚本改成只在 `class Params:` 与第一个 `@property` 之间匹配。

**验证**（改了注释就不该有任何行为变化，所以用字段级对比而不是「跑一遍看看」）：

| 检查 | 结果 |
| --- | --- |
| 与 `git show HEAD:src/bionic_fish/params.py` 逐字段比 (`dataclasses.fields` 的名字 + 默认值) | 76 → 76，**逐字段一致** |
| 测试 | 18 条通过 |
| 仿真 | `到达终点` / 8 个 / 25.45s / 5000mm，与改前逐位相同 |
| `compileall` | 通过 |

## 二、文档债：逐条对着 `src/` 核

原则沿用 v2 收尾那一轮：**设计理由保留，模块名与「代码里有没有」改到实况**。改完删掉 `architecture.md` / `vision.md` / `control.md` / `hardware.md` 顶部那四张「本文件描述的是 v0.1」横幅——它们的正文已经不需要免责声明了。

| 文件 | 改了什么 |
| --- | --- |
| [`AGENTS.md`](../../AGENTS.md) | 红线 1（`p>3/8` 判决在 v2 不存在，实际只剩「上限 8」，默认配置实测 8 次只有 5 次命中真点）与红线 7（去重是布尔 `in_dot` + `report_row`，不是轨迹身份）改到 v2 现状；IMPORTANT 第 4 条删掉「灯带颜色仍未确认」（早已确认为白色圆柱管道） |
| [`README.md`](../../README.md) | §五 拆开「开发机 `-e .`」与「上位机 `-e ".[hardware,rpi]"`」，并记下本机缺 `pyserial` / `RPi.GPIO` / `onnxruntime`；§七 改成三种起始条件都跑完全程，补第 4 件卡住的事（落盘与可视化）与 54 个标记的 grep |
| [`backlog.md`](../../backlog.md) | 全文件按 v2 核对：v0.1 类型名全部改掉；`artefact_dir` 条目重写成「落盘与可视化尚未实现」；**新增两条真代码缺口**（`bionic-fish` 脚本指向不存在的函数、相机分辨率没有任何处理）；TTS 条目补上「`__main__` 不传 `speaker` ⇒ 真机永不发声」 |
| [`invariants.md`](../agents/invariants.md) | 第 2 / 4 / 5 条各补一段「v2 没有实现 / 实现形式变了」，并给出可执行的 `grep` 自查 |
| [`troubleshooting.md`](../agents/troubleshooting.md) | 开头加 CAUTION：没有三道闸、没有闸门日志、没有落盘、没有可视化——**证据清单今天一项也产不出来**，所以「先补落盘再用手册」；§3 的逐闸排查改成 v2 只有两处能拒；§4/§6/§7/§8 换成现行名字 |
| [`vision.md`](../agents/vision.md) | 判别特征表的「代码现状」列改成实况（`hue`/`pattern`/`position` **未实现**，只有 `dot_val_max`）；`val_band` 全部改称 `dot_val_max`；删掉不存在的测试名；标定表产物列换成 v2 字段名；补 2026-10-11 复测（默认配置 8 次上报只有 5 次命中真点） |
| [`control.md`](../agents/control.md) | 实测表改为三种起始条件都跑完全程（25.4s / 8 上报）、删掉不存在的 `test_closed_loop.py`；弯向改为「写死的正号，没有 `corner_turn_sign`」；**补三条「设计在、代码不在」的 CAUTION**（`ALIGN` 起步对齐、四级丢线恢复、转弯的「转够角度即停」）；速度档与参数整定表换成现行字段名 |
| [`operations.md`](../agents/operations.md) | 「尚无」表补落盘/可视化与语音发声两行；`src/`、`tests/` 现状改准（占位动力学在 `sim/`、18 条、无 expectedFailure）；标定清单换成 `mm_per_px_override` / `report_row` / `strip_width_mm` / `tape_*` |
| [`architecture.md`](../agents/architecture.md) | 通道布局改为只有 `ch_yaw`/`ch_forward`（通道号未确认）；8.2/8.3 的模块数与行数改准（7 文件约 1330 行、`vision.py` 343 行、`params.py` 约 290 行）；记下控制台脚本的问题 |
| [`reporting.md`](../agents/reporting.md) · [`reference-pipeline.md`](../agents/reference-pipeline.md) · [`hardware.md`](../agents/hardware.md) · [`competition.md`](../agents/competition.md) · [`writing.md`](../agents/writing.md) | 前两份只改残留的类型名；hardware / competition / writing 同时补上 `【待验证·实机】` 的约定与 2026-10-11 变更行 |

## 三、验证

| 项 | 结果 |
| --- | --- |
| 链接校验 | 30 个文档、376 条相对链接与锚点**全部可达** |
| 残留 v0.1 名字 | 全仓 `--include=*.md` 扫过，剩下的每一处**都明确写着「v0.1 …已删除 / 没有了」**或位于「变更记录」表里 |
| 代码 | `params.py` 除注释外与 HEAD 一致；18 条测试通过；仿真与改前逐位相同 |

## 四、没做的事（如实记）

1. **四个代码缺口一个没修**：控制台脚本、相机分辨率、TTS 接线、落盘与可视化。审阅者明确要求先别动，已全部记进 [backlog](../../backlog.md)。
2. **没有提交、没有合并**。当前在 `dsh` 分支；是否提交由审阅者决定。
3. **`reporting.md` 的正文没重写**：它的顶部已有 CAUTION 说明「这套判决在 v2 里没有了」，本轮只改残留名。要恢复置信度判决时，那份文档仍是起点。
4. **`strategy.md` 没动**：它的内容是推导，不依赖具体模块名。但它引用的 `时序确认` / `三道闸` 说法同样属于 v0.1，真要落地时得按 v2 重写。

---

相关：[writing.md](../agents/writing.md)（新增 `【待验证·实机】` 约定）· [`params.py`](../../src/bionic_fish/params.py)（54 个标记）· [backlog.md](../../backlog.md)（新增两条代码缺口）· [2026-10-02-linear-rewrite](2026-10-02-linear-rewrite.md)（这些债的来源）
