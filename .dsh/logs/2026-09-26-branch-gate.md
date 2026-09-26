# 2026-09-26 — 分支检查必须是闸门，不是打印

## 目标

用户要求：写一份开发者 README（讲架构、`.dsh/` 与 `AGENTS.md` 的用法、`src/` 各模块），并回答「`agents.md` 是不是该改名叫 `agent.md`」。

两件事都做完了，但过程中**我把一次提交落到了 `main` 上**。这份日志记那个事故——它不是代码问题，是流程问题，而且正是本项目分支纪律想防的那一类。

## 事故

### 发生了什么

`README.md`、`AGENTS.md`、`.gitignore` 三处改动提交后，`git log` 显示：

```
53bf403 (HEAD -> main) docs: 新增开发者 README，并回答 AGENTS.md 能不能改名
ee22472 (dsh)           docs: 技能源文件现在有独立仓库
```

**提交落在 `main` 上**，而按约定开发应在 `dsh`。

### 为什么——先查 reflog，不猜

```
ee22472 HEAD@{Sat Sep 26 12:35:42 2026}: merge dsh: Fast-forward
06b02b1 HEAD@{Sat Sep 26 12:35:35 2026}: checkout: moving from dsh to main
53bf403 HEAD@{Sat Sep 26 12:48:47 2026}: commit: docs: 新增开发者 README...
```

所以顺序是：**用户在 12:35 检出 `main` 并做了 `git merge dsh`（fast-forward）**——那是用户给出的确认，是正当操作；之后 HEAD 留在 `main` 上；我在 12:48 提交，就落在 `main` 上。

**这一步很重要**：我先看 reflog 才动手。如果直接「发现 main 多了提交就 reset」，就会去改用户刚刚有意做出的合并状态。

### 我违反的到底是什么

纪律第 3 条原文是「每次提交前先跑 `git branch --show-current` 确认自己在 `dsh`」。我那条命令是：

```sh
git add -A && echo "BRANCH: $(git branch --show-current)" && git status --short && git commit -q -F - <<'MSG' ...
```

它**确实跑了检查，也印出了 `main`**——但 `&&` 不关心 `echo` 说了什么，链子照样走到 `commit`。

> **检查的结果没有改变下一步动作，那就不算检查。**

这是和「`cv2.circle` 是赋值不是叠加」同一类的错误：形式上有那个动作，语义上没起作用。

## 怎么修的

提交内容本身没问题，只是放错了分支；而且 `53bf403` 是 `dsh` 的直接后代，所以搬运是**无损快进**：

```sh
git branch -f dsh 53bf403      # 把提交纳入 dsh
git switch dsh                 # HEAD 回到工作分支
git branch -f main ee22472     # main 回到用户确认的那个合并点
```

结果：`main` = `ee22472`（用户确认的状态），`dsh` = `53bf403`（含新提交），工作区干净，没有内容丢失。动手前先确认了工作区干净、且 `git merge-base --is-ancestor dsh 53bf403` 成立。

## 顺手核出来的一件事：`AGENT.md` 不能用

用户问「`agents.md` 是不是应该改名叫 `agent.md`」。去读了源码而不是凭记忆：

```ts
// packages/context/agent-instructions/src/config.ts
const DEFAULT_INSTRUCTION_FILE_CANDIDATES = ['AGENTS.md', 'CLAUDE.md'] as const
const DEFAULT_LOCAL_INSTRUCTION_FILE_CANDIDATES = ['AGENTS.local.md', 'CLAUDE.local.md'] as const
```

**答案是不应该**：只有 `AGENTS.md` / `CLAUDE.md`（及 `.local` 覆盖层）会被加载，`AGENT.md`（单数）在整个代码库里**没有任何引用**——改名会让索引**静默失效**（不报错，只是再也不进上下文）。

顺带发现**`.local` 覆盖层**这一条我之前漏了：那是「给智能体的本机指令」，属于本机配置，必须 gitignore（提交一份等于把某个开发者的机器专属指令推给所有人）。已补进本仓库 `.gitignore`、技能的事实文件与技能模板。

## 改了哪些地方（防止重犯）

| 位置 | 改法 |
| --- | --- |
| `AGENTS.md` 工作方式第 3 条 | 从「先跑 `git branch --show-current` 确认」改成「**当闸门用，不在 `dsh` 就中止**」，并贴上这次事故的具体形态与正确写法 `[ "$(git branch --show-current)" = dsh ] \|\| exit 1` |
| `README.md` 动手前的约定第 1 条 | 同上，附日志链接 |
| 技能 `SKILL.md` 的「protect the default branch」 | 第 3 条改成「check must be a gate」，并把 `git add && echo && git commit` 这个反例写进去；**新增第 4 条**：「发现自己站在默认分支上，也可能是用户有意为之（比如他正在做合并）——先读 reflog 再下结论」，免得下次把用户正当的操作当事故「修掉」 |
| 技能 `references/dsh-layout-facts.md` | 补 `.local` 覆盖层；把「改名成 `agent.md` 会静默失效」写显 |

## 未做

- 没有把 `dsh` 合并到 `main`，也没有退回任何已提交内容。`main` 停在用户确认的 `ee22472`，等用户明确说合再合。
- 技能仓库的这次改动在它自己的 `dsh` 分支（`a7ae906`），按其纪律同样等确认后再并入 `main`。
