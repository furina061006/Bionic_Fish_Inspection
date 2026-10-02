# 水中机器人巡检任务挑战赛 — 项目索引（智能体入口）

全球校园人工智能算法精英大赛 · 算法应用赛 · 水中机器人巡检任务挑战赛（PDF 元数据 `JSAI2025`）。
本仓库**只负责比赛的视觉与控制部分算法**：灯带巡线、漏油点识别、检测上报决策、巡线控制。
机器鱼本体结构、推进机构、电源、外壳由统一平台与另一部分工作负责，本仓库只消费它提供的接口。

赛题原文（唯一权威）：[`docs/16水中机器人巡检任务挑战赛(6)(1).pdf`](docs/16水中机器人巡检任务挑战赛%286%29%281%29.pdf)
场地图：[`docs/images/arena-top-view.png`](docs/images/arena-top-view.png)、[`docs/images/arena-3d-view.png`](docs/images/arena-3d-view.png)

> [!IMPORTANT]
> 1. **本文件只是索引 + 红线 + 路由表**。分主题的项目事实在 `.dsh/agents/` 下——动手写代码前按下面的路由表读对应文档，写完回来更新它。
> 2. **唯一需求权威是 `docs/` 里的赛题 PDF，以及组委会 / 平台方的官方材料。** 其它项目——队内招新题、往届作品、邻近比赛、开发机上恰好装了哪些库——**一律不构成需求依据**，引用它们前必须先向组委会确认。
> 3. **每句话都必须能归类**：`【事实】`= 赛题文件 / 官方规则 / 实测结果；`【设计】`= 本队已决定的方案；`【待验证】`= 假设，未验证前不能作为依据。**没标 `【事实】` 的数字不许写进代码常量。**
> 4. **工具链由用户指定为「与 `reference/pipe_inspector_raspberry` 一致」**：树莓派 + Python + OpenCV + MAVLink `RC_CHANNELS_OVERRIDE` + onnxruntime + `RPi.GPIO`。但**通道数、相机、灯带颜色仍未确认**，见 [`reference-pipeline.md`](.dsh/agents/reference-pipeline.md)。参考项目是**工具链依据，不是需求依据**，其算法是要重构的起点。

## 工作方式：git 分支纪律（**先读这条**）

1. **所有开发都在 `dsh` 分支上进行。** `main` 保持现状，不直接在其上改代码。
2. **合并到 `main` 必须经用户明确确认**（例如「可以合了」）。不得自行合并，也不得因为「改动很小」就跳过确认。
3. **提交前把分支检查当闸门用，不是打印一行就继续。** 先跑 `git branch --show-current`，**不在 `dsh` 就中止**。工作区与另一个终端共享，HEAD 会被切走——2026-09-26 就因此把一次提交落到了 `main` 上：当时别人在 `main` 上做完合并后没切回来，而我那条命令是 `git add && echo "BRANCH: $(...)" && git commit`，**检查印出了 `main`，`&&` 链却照样提交**。正确的写法是先判断再动作，例如：

   ```sh
   [ "$(git branch --show-current)" = dsh ] || { echo "不在 dsh，中止"; exit 1; }
   ```
4. 分支只是防线，**产物才是交付**：合并前先把 `.dsh/agents/` 与 `.dsh/logs/` 更新到与代码一致，否则过了确认的 `main` 里文档与代码互相矛盾。

## 红线（不翻文档也必须记住）

1. **误报比漏检贵，而且贵得不对称**：正确上报一个漏油点 `+5`，误报一个 `−3`。一个候选点只有置信度 `p > 3/8 = 0.375` 时才值得上报；已确认 8 个之后继续上报的收益为 0、风险仍为 −3。→ [不变量 2](.dsh/agents/invariants.md)
2. **不知道误识别点长什么样**：赛题只说 8 个漏油点是 ⌀3cm 黑色圆、5 个误识别点也是 ⌀3cm 圆，**没说区分特征**。这是当前最大未知项，只能在调试窗口标定。→ [视觉](.dsh/agents/vision.md) · [排查](.dsh/agents/troubleshooting.md)
3. **禁止任何外部辅助定位**：声呐信标、光学标记等一律禁止；自带外部视觉标记还会让「技术加分 5 分」直接归零。巡线只能用本体传感器。→ [硬件](.dsh/agents/hardware.md)
4. **三条硬终止条件必须实现为保护，不是优化项**：完全脱离路线 >10 秒终止；触底 / 触壁满 3 次本轮结束；裁判宣布开始后 30 秒未启动结束。任何「激进」的丢线恢复动作都按这三条设计。→ [控制](.dsh/agents/control.md)
5. **时间分梯度陡到改变设计**：≤10 秒 20 分、≤30 秒 12 分、>150 秒 0 分。速度与稳定性的取舍是第一性问题，不是后期调参。→ [策略](.dsh/agents/strategy.md)
6. **语音播报是独立得分项，且必须与上报一一对应**：每上报一个播报一次，漏播一个 `−2`（扣完 10 分为止）。上报和播报必须共用同一个事件，不许各写一套触发。→ [上报](.dsh/agents/reporting.md)
7. **上报去重按「候选轨迹身份」，既不是纯时间也不是纯距离**：13 个标记平均间距约 460mm，**小于鱼体 600mm 长度**，所以距离窗口会把相邻两个真实漏油点合并（漏检），而纯冷却时间会在慢速时误抑制、在快速时重复上报（重复上报按误报扣 `−3`）。→ [不变量 4](.dsh/agents/invariants.md)
8. **两张图都不是按比例画的**：按 500mm 内缩推算直线段只有约 5000mm，与文字「路径总长约 6m」不一致。不许从图上量尺寸当依据。→ [赛题事实](.dsh/agents/competition.md)
9. **阈值必须来自标定，不许硬编码魔数**：水下光照、水池材质、灯带亮度每场都变，「3cm 圆对应多少像素」必须先量。→ [操作](.dsh/agents/operations.md)

## 路由表：要做什么 → 先读什么

| 我要做的事 | 先读 |
| --- | --- |
| 查参考项目的工具链、算法与已知缺陷 | [`.dsh/agents/reference-pipeline.md`](.dsh/agents/reference-pipeline.md) |
| 查场地尺寸、道具、判罚、评分表的原始规定 | [`.dsh/agents/competition.md`](.dsh/agents/competition.md) |
| 选型相机 / 灯 / 传感器，或核对尺寸、防水、禁用项 | [`.dsh/agents/hardware.md`](.dsh/agents/hardware.md) |
| 动模块划分、数据流、状态机、模块间接口 | [`.dsh/agents/architecture.md`](.dsh/agents/architecture.md) |
| 想知道**代码每帧按什么顺序跑**、该从哪一行开始读 | [`.dsh/agents/runtime-flow.md`](.dsh/agents/runtime-flow.md) |
| 改图像增强、灯带提取、中心线、暗斑检测 | [`.dsh/agents/vision.md`](.dsh/agents/vision.md) |
| 改巡线控制、90° 转弯、起终点、丢线恢复 | [`.dsh/agents/control.md`](.dsh/agents/control.md) |
| 改上报时机、去重、防误报、声光与语音 | [`.dsh/agents/reporting.md`](.dsh/agents/reporting.md) |
| 决定「先做哪个、值多少分、要不要冒险」 | [`.dsh/agents/strategy.md`](.dsh/agents/strategy.md) |
| 提交前逐条核对红线 | [`.dsh/agents/invariants.md`](.dsh/agents/invariants.md) |
| 排查「跑偏 / 丢线 / 漏检 / 误报 / 弯角过不去」 | [`.dsh/agents/troubleshooting.md`](.dsh/agents/troubleshooting.md) |
| 搭环境、标定、仿真、比赛日流程与清单 | [`.dsh/agents/operations.md`](.dsh/agents/operations.md) |
| 写 / 改 Markdown 文档（加粗、标记约定等写法） | [`.dsh/agents/writing.md`](.dsh/agents/writing.md) |
| 回答「项目怎么跑、该改哪个文件」（给人看的口径） | [`README.md`](README.md) |
| 想知道「现在卡在哪、下一步要问谁」 | [`backlog.md`](backlog.md) |
| 想知道「为什么这么定」「踩过什么坑」 | [`.dsh/logs/README.md`](.dsh/logs/README.md) |

## 仓库结构（**只记目录，不记目录里的文件**）

| 路径 | 职责 |
| --- | --- |
| `AGENTS.md` | 本文件：索引 + 工作方式 + 红线 + 路由表 + 目录结构（**给智能体**） |
| `README.md` | 开发者入口：架构总览、`.dsh/` 与 `AGENTS.md` 的用法、`src/` 各模块职责、怎么跑（**给人**） |
| `backlog.md` | 已知但未完成的事，按「卡住什么」分组 |
| `pyproject.toml` | 打包元数据：包名、依赖、可选硬件 extras、命令行入口——**依赖与版本的唯一住所** |
| `requirements.txt` | 部署快捷方式，只含 `-e .` 指向 `pyproject.toml`；**不要再抄一份版本号** |
| `.gitignore` | 什么不进版本控制，以及为什么 |
| `.dsh/agents/` | 分主题的项目事实、设计决定、不变量、排查流程 |
| `.dsh/logs/` | 面向开发者的过程记录（当时快照，不改写正文） |
| `.dsh/skills/` | 项目内技能入口（软链接到用户级源文件，本身不入库） |
| `.dsh/tools/` | 校验脚本 |
| `src/` | 实现代码（**只有实机代码**） |
| `sim/` | 仿真壳：合成场地、占位动力学、假相机/假飞控。**不入包** |
| `tests/` | 测试 |
| `deploy/` | 部署：systemd 单元。**启动方式与时机见 [`.dsh/agents/operations.md`](.dsh/agents/operations.md)** |
| `docs/` | 权威输入与面向使用者的资料（赛题 PDF、场地图） |
| `reference/` | 第三方参考存档，只读，**不入库**（见 `.gitignore`） |

**为什么只记目录**：目录清单一旦列到文件，就是文件系统的第二份副本，下一次提交就过期。目录的职责很少变，写下来才守得住。想知道有哪些模块，代码本身才是权威。同理，本表不写「某个目录下有几个文件」。

## 知识管理约定（每次会话遵守）

- **`AGENTS.md`（本文件）**——索引 + 工作方式 + 红线 + 路由表 + 目录结构。**保持精简**：细节写进 `.dsh/agents/`，过程写进 `.dsh/logs/`
- **`backlog.md`（仓库根目录）**——已知但未完成的事。每条写清「缺什么、找谁、解开后能做什么」；**做完就删掉**，不留痕（历史在 `.dsh/logs/`）
- **`.gitignore`（仓库根目录）**——什么不进版本控制，以及**为什么**。判断依据是「谁拥有它、该不该进历史」：凭据、生成物、大二进制、第三方树、本机配置一律排除
- **`.dsh/agents/`**——分主题的项目事实、设计决定、不变量、排查流程（本文件的展开）。改代码前按路由表读，**改完要回来更新**
- **`.dsh/logs/`**——面向开发者的过程记录：为什么这么做、试过哪些死路、根因分析。每条日志是**当时的过程快照，不改写正文**；其 `README.md` 是索引
- **`.dsh/skills/`**——项目内技能入口。本项目的 `dsh-agent-init` 是指向 `~/.dsh/skills/dsh-agent-init/` 的**软链接**；技能源文件在用户级目录、**有独立的 git 仓库**，不在本仓库
- **`.dsh/tools/`**——校验脚本。改完文档跑 `python3 .dsh/tools/check-links.py .`，验证所有相对链接与锚点可达
- **`docs/`**——面向使用者的资料与赛题原始文件（PDF、场地图）
- **git 分支**——开发在 `dsh`，经用户确认后才并入 `main`（见上面的「工作方式」）
- **每次重大技术变化后**：先更新 `.dsh/agents/` 对应文档 → 再写 `.dsh/logs/` 开发日志 → 更新 `.dsh/logs/README.md` 索引

**为什么本文件必须在仓库根目录**：DSH 只自动加载 `AGENTS.md` / `CLAUDE.md` 这两个**文件名**，而**位置决定作用域**——根目录那份 = 项目级指令（每轮都在上下文里），放在任何子目录（例如 `.dsh/`）就只对那个目录生效。所以索引留在根目录，细节才敢放进 `.dsh/agents/`。

## 相关文档

- [`README.md`](README.md)——**给人**的开发者入口：架构总览、`.dsh/` 与 `AGENTS.md` 怎么用、`src/` 各模块职责、怎么跑
- `.dsh/agents/`——赛题事实、硬件约束、架构、**代码运行顺序**、视觉、控制、上报、策略、不变量、排查、操作、写法
- `.dsh/logs/README.md`——开发日志索引
- `.dsh/skills/dsh-agent-init/SKILL.md`——**本知识体系自身的方法论**，用同样的方式给新项目初始化 `.dsh/`
