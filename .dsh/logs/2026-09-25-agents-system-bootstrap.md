# 2026-09-25 — 搭建披露式 agent 知识体系

## 目标

为「水中机器人巡检任务挑战赛」项目（本仓库只负责**视觉与控制算法**）搭建 agent 知识体系，方法论来自两个参考：

1. `/home/furina061006/AAA_Project_libraries/MOOC_reminder/AGENTS.md` + 其 `.dsh/`（索引 + 红线 + 路由表 / `.dsh/agents/` 分主题事实 / `.dsh/logs/` 过程记录）
2. `/home/furina061006/AAA_Software_libraries/Agents/deepseek-harness/`（`AGENTS.md` + `.agents/skills/<name>/SKILL.md` 的披露式技能体系）

并把方法论蒸馏成可复用技能 `dsh-agent-init`。

## 做了什么

1. **提取赛题事实**：用 `pymupdf` 抽取 `docs/16水中机器人巡检任务挑战赛(6)(1).pdf` 全文（6 页）+ 页内插图；用 300 dpi 重新渲染场地俯视图与立体图到 `docs/images/`（原嵌图只有 630×437，看不清标注）
2. **确定目录布局**：把用户建的 `dsh/` 改名为 `.dsh/`，`AGENT.md` 改名为 `AGENTS.md`
3. **写 `.dsh/agents/` 11 份主题文档**：`competition` / `hardware` / `architecture` / `vision` / `control` / `reporting` / `strategy` / `invariants` / `troubleshooting` / `operations` / `writing`
4. **建立本日志目录**与索引
5. **蒸馏技能** → `~/.dsh/skills/dsh-agent-init/`，项目内 `.dsh/skills/dsh-agent-init` 做软链接

## 关键决策与依据

### 目录名必须是 `.dsh`，不能是 `dsh`

DSH 的 `skill-filesystem` 提供者按固定根目录表扫描技能（`packages/skill/skill-filesystem/src/index.ts`）：

| Rank | source | 路径 |
| --- | --- | --- |
| 100 | `project-dsh` | `<projectRoot>/.dsh/skills` |
| 200 | `project-agents` | `<projectRoot>/.agents/skills` |
| 300 | `custom` | `Config.customSkillDirs` |
| 400 | `user-dsh` | `<dshHome>/skills`（`$DSH_HOME` 或 `~/.dsh`） |
| 500 | `user-agents` | `<agentsHome>/skills`（`~/.agents`） |

项目根 = **最近的含 `.git` 的祖先目录**，没有 `.git` 时用当前工作目录。本仓库没有 `.git`，所以 `projectRoot` 就是仓库目录本身。

### `AGENT.md` 不会被自动加载

`packages/context/agent-instructions/src/config.ts`：

```ts
const DEFAULT_INSTRUCTION_FILE_CANDIDATES = ['AGENTS.md', 'CLAUDE.md'] as const
```

只有这两个文件名会被加载。原文件 `AGENT.md` 是 0 字节，直接改名不影响内容。

### 技能名不能带下划线

`packages/skill/skill/src/index.ts`：

```ts
const SKILL_NAME = /^[a-z0-9]+(?:-[a-z0-9]+)*$/
```

所以用户口述的 `dsh-agent_init_skill` **不是合法技能名**，落成 `dsh-agent-init`。

### 软链接是本项目选定的安装方式

`skill-filesystem` 的 `nodeEntryKind` 对符号链接做 `stat` 并跟随（目录 → `directory`，文件 → `file`），因此 `root/<name>` → 用户级目录的软链接能被正常发现。这满足「源放 `~/.dsh/skills`、项目内软链接」的要求，且只有一份源文件。

### 用 `pymupdf` 而不是 `pdftotext`

系统没有 `pdftotext` / `pdftoppm`，也没有 `pdfplumber` / `PyPDF2`。`pip install pymupdf` 装的是预编译 wheel（约 26 MB），能同时做文本抽取、区块坐标抽取和按 dpi 重渲染，一次满足三个需求。

## 从赛题里读出的、值得记的过程结论

### 两张场地示意图都不是按比例画的

把「上 / 下 / 右灯带各距池壁 500mm」与 3000×2000 场地一起算：两段横向直道各约 2000mm、竖向约 1000mm，合计 **约 5000mm**，而原文文字写「路径总长约 6m」。

补充证据：图中「上灯带距上池壁」与「下灯带距下池壁」的像素距离明显不相等，而两者按文字都应是 500mm。

结论：**不许从图上量尺寸**。这条写进了 `AGENTS.md` 红线 8 与 `competition.md` 的 `【待验证】`。

### 评分表决定了整个技术路线

从评分表读出两个不对称，它们直接改写了算法设计：

1. **`+5` vs `−3`**：候选点上报的期望收益是 `8p − 3`，阈值 `p > 0.375`。这就是 `reporting.md` 三道闸的理由，也是「看到黑点就报」被列为红线的原因
2. **时间分梯度 vs 其它项的失分**：30s → 20s 只多赚 3 分，而偏离一次掉 5 分、漏检一个掉 5 分、误报一次掉 3 分。所以 `strategy.md` 的结论是**不去抢时间分的 20 分**，先把「不偏离 + 不漏 + 不误报」稳住

### 最大的未知项：误识别点长什么样

赛题只说 8 个漏油点是 ⌀3cm 黑色圆、5 个误识别点也是 ⌀3cm 圆，**完全没说两者怎么区分**。而且比赛既然设了误识别点并对误报扣分，设计上必然可区分——判据只是没写在赛题文件里。

这条被列为关键路径（`operations.md` 未决问题 #1），并给出两个动作：向负责人索取物料说明；准备可在 10 分钟调试窗口内跑完的多特征标定脚本。

### 相机视场是被水深卡死的

水面高度 260 ± 20mm，相机向下看池底，距离量级只有 0.15–0.26m。按 `W ≈ 2·h·tan(HFOV/2)`：

| h | HFOV 60° | HFOV 90° | HFOV 120° |
| --- | --- | --- | --- |
| 0.15 m | 0.17 m | 0.30 m | 0.52 m |
| 0.26 m | 0.30 m | 0.52 m | 0.90 m |

60° 镜头下前方只有 17–30cm 可视，基本不可用。结论：**广角（≥90°）镜头是必需的**，而且控制律的前瞻距离上限由视场决定，不是自由参数。这条把 `hardware.md`、`vision.md`、`control.md` 三份文档串了起来。

### 标记间距小于鱼体长度

8 + 5 = 13 个标记分布在约 6m 路径上，平均间距约 460mm，**小于鱼体 600mm 长度**。因此：

- 不能用距离窗口做上报去重（会把相邻两个真实漏油点合并）→ 必须用**候选轨迹身份**去重
- 也不能用纯时间冷却（快慢变化下既误抑制又重复触发）

这条写进了 `invariants.md` 第 4 条。

## 中途改正过的地方

- 一开始打算把可复用技能同时放项目内与 `~/.agents/skills/`（复制两份）。用户选择「源放 `~/.dsh/skills`、项目内软链接」，改为软链接方案，避免两份漂移
- 一开始误以为 `AGENT.md` / `dsh/` 是用户有意命名的；查 DSH 源码确认这两个名字都**不会生效**，遂改名（用户确认 `.dsh/`）
- 第一版文档写完后跑链接校验，发现 7 处问题，全部修掉：
  1. **赛题 PDF 的链接被括号截断**——文件名含 `(6)(1)`，Markdown 会把第一个 `)` 当链接结尾。改为百分号转义 `%28` / `%29`。新增约定记入 `writing.md`
  2. **中文标题锚点不匹配**——原先的锚点生成只剥 ASCII 标点，而 GitHub 会连全角括号一起丢掉。校验脚本改为「只保留字母 / 数字 / 组合标记与 `-` `_`」，`#联系方式原文-p6` 这类锚点才开始可达
- 同一轮校验里发现 2 处**违反自己写的「一个事实一个家」**：`strategy.md` 完整复述了时间分档表、`hardware.md` 复述了水面高度。前者改为引用 + 派生表，后者标注为「`competition.md` 的派生视图」；同时在 `writing.md` 补上「红线允许行内写阈值」「派生视图的判定标准」两条例外

## 验证结果

| 检查 | 方法 | 结果 |
| --- | --- | --- |
| 相对链接与锚点全部可达 | `python3 .dsh/tools/check-links.py . ~/.dsh/skills/dsh-agent-init` | 17 个 md、152 条相对链接，全部通过 |
| 技能可被发现 | 按 `skill-filesystem` 的 `discoverRoot` / `nodeEntryKind` 规则复刻脚本，逐个根目录检查 | `project-dsh` 与 `user-dsh` 两个根都解析出 `dsh-agent-init`，软链接被识别为 `directory`，`name` 匹配 `^[a-z0-9]+(?:-[a-z0-9]+)*$` |
| 技能进入会话目录 | 会话技能目录 | 创建后无需重启即出现在可用技能列表中 |
| 没有残留旧命名 | `ls` | 无 `dsh/`、无 `AGENT.md` |

## 未做（明确留给后续）

- 没有写任何算法代码（用户要求「先做这么多」）
- 没有建 git 仓库；`.dsh/` 的项目根判定因此退化为工作目录（`skill-filesystem` 的 `projectRoot` 取最近含 `.git` 的祖先，没有则用 cwd）
- `competition.md` 里 8 条 `【待验证】` 都还没有结论，需要向组委会 / 平台方确认（清单见 `.dsh/agents/operations.md` 未决问题表）

---

## 同日补充：调查环境时走的一次弯路（**记下来防止重犯**）

用户问「要不要了解一下当前的环境情况，比如 conda 环境里已经装了哪些库」。调查完毕，得到的事实是：

| 查到的 | 结论 |
| --- | --- |
| conda 环境 `aquacore`（Python 3.10.21） | 装了 ROS 2 Humble 全套、OpenCV 5.0.0、PyTorch 2.8.0+cu129、ultralytics 8.4.144、pymavlink |
| 硬件 | RTX 5070 8GB / CUDA 12.9 / WSL2；**`/dev/video*` 为空**（无相机接入） |
| `AAA_Project_libraries/neu-aquacore-pretests` | 队内**招新题**：OpenCV 四道识别题（引导线 / 管道 / 门框 / 球）、YOLO 训练、ArduSub SITL 四关 |

**我当时的错误推理**：从招新题里用了 ArduSub SITL 和 `vectored` 模型，就推断本项目的控制栈是 ArduSub + MAVLink；从 `aquacore` 环境装了 PyTorch/CUDA，就暗示部署环境有 GPU。我甚至把这些结论写进了给用户的建议里。

**用户当场纠正**：招新题和本项目**没有任何关系**，里面的数据不一定对，工具链也没定，**一切根据 `docs/` 的 PDF**。

**核对 PDF 后的正确结论**：赛题原文对运行环境**只字未提**——操作系统、语言、中间件、是否用 ROS、算力放在哪，全都没有规定。所以「运行环境未定」这个最初写在 `operations.md` 里的判断**本来就是对的**，是招新题把我带偏了。这正是 `writing.md` 里「`【事实】` 必须能指到出处」这条规则要防的事，而我违反了它。

**顺手从 PDF 里读出来的、之前漏掉的推论**（原文 p2）：

- 「机器鱼需**搭载**独立电源、**自主控制系统**及传感器模块」
- 「漏油点检测模块需**集成于机器鱼本体**」
- 「必须完全自主运行，**禁止远程操控**或预编程固定路径」

→ 【推论】把感知放在池边计算机、只把指令送回鱼体，与上面三条的字面要求相冲突；**默认应按「全部算力在鱼上」设计**。这反过来说明：开发机上的 GPU 与深度学习框架**不构成「比赛时可用 GPU」的证据**。

→ 还漏了两条硬性义务：「传感器可使用组委会提供的标准套件或自主设计，但**需提前提交技术方案并通过审核**」（p2），以及传感器是「提供」还是「自研」会改变整个视觉方案的工作量。这两条已补进 `operations.md` 的未决问题表（新增 #2、#3、#5）。

**因此本轮对知识库的修正**（不改历史正文，只做追加与订正）：

1. `AGENTS.md`：IMPORTANT 块新增一条——唯一权威是 `docs/` 的 PDF 与官方材料，**其它项目不构成依据**
2. `operations.md`：现状表把「运行环境未定」写清为「PDF 对工具链只字未提，选型自负后果」；未决问题从 8 条扩到 11 条，新增传感器来源（#2）、技术方案审核流程（#3）、算力是否必须机载（#5）；补上「算力必须机载」的推论块与「开发环境 ≠ 部署环境」的警告
3. `architecture.md`：进程与代码组织一节改为「PDF 无规定 + 默认算力机载 + 不要从开发机倒推部署架构」
4. `writing.md`：在状态标记规则里补上「依据来源的合法性」——其它项目只能产生 `【待验证】` 假设

**教训**：手上有一个相邻领域的真实代码库时，最容易犯的错就是把它当成需求来源。**代码是「别人怎么做的」，PDF 是「我们必须做什么」**，两者不能互相顶替。
