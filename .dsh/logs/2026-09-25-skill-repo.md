# 2026-09-25 — 给技能目录建 git 仓库

## 目标

`dsh-agent-init` 技能此前只存在于文件系统（`~/.dsh/skills/dsh-agent-init/`），**不在任何 git 仓库里**。用户说以后要用，要求给它建仓库。

## 做了什么

1. 在 `~/.dsh/skills/dsh-agent-init/` 建仓库（`main` 分支），基线提交 `f530a89`
2. 补 `.gitignore`（纯文档技能，只防「以后加了 `scripts/`」的情况）
3. 补 `README.md`：这是什么、**为什么仓库根在这一层**、怎么装进项目、目录里有什么
4. 验证技能发现未被破坏；更新本项目 `AGENTS.md` 里描述技能位置的句子

## 关键决策：仓库根选在技能目录，而不是 `~/.dsh/skills/`

一开始想放在技能根 `~/.dsh/skills/`（「一个仓库管所有本地技能」），但去读了提供者的发现规则
（`packages/skill/skill-filesystem/src/index.ts` 的 `discoverRoot`）：

```ts
const locator = entry.type === 'directory'
  ? { path: join(entry.path, 'SKILL.md'), directory: entry.path }
  : entry.type === 'file' && entry.name.endsWith('.md')
    ? { path: entry.path, directory: root.path }      // 顶层 .md 当作「扁平技能」
    : undefined
```

- 顶层**目录**条目 → 只读 `<dir>/SKILL.md`
- 顶层**`.md` 文件** → 当作扁平技能解析（没有合法 frontmatter 就被跳过并打警告）

所以 `~/.dsh/skills/README.md` 会被当成一个**技能候选**。而仓库是需要 README 的。
放在 `dsh-agent-init/README.md` 则完全不会被扫描——目录条目只读 `SKILL.md`。

**结论：仓库根必须在技能目录这一层。**顺带一个好处：`~/.dsh/skills/` 保持「DSH 的技能命名空间」这一单一职责，不会混进 `.git` / `.gitignore`。

## 验证结果

| 检查 | 结果 |
| --- | --- |
| 两个根都能发现技能 | `project-dsh`（项目软链接）与 `user-dsh`（仓库本体）都解析出 `dsh-agent-init`，`name` 合法 |
| 新 `.git` 没造成幻影条目 | `~/.dsh/skills/` 顶层条目仍只有 `dsh-agent-init`，无多出的技能 |
| 项目软链接仍可达 | `.dsh/skills/dsh-agent-init -> ~/.dsh/skills/dsh-agent-init` 可解析 |
| 仓库状态 | 基线提交 `f530a89`，工作区干净 |

## 中途改正过的地方

README 初稿里写了 `[DSH](https://github.com/)`——一个占位 URL。凭空指一个链接等于伪造出处，已改成纯文本「DeepSeek Harness (DSH)」。

## 未做 / 留给后续

- **没有远端**：这台机器上没装 `gh`（`which gh` 无输出），所以只建了本地仓库。要备份/分享需用户先在托管平台建空仓库，再把地址给我。
- **分支纪律**：这次是用户明确要求的基线导入，所以直接落在 `main`。技能本身教的是「开发在工作分支、合并到 `main` 需明确确认」，以后改技能应按同一规矩办。
