# 代码运行顺序：一帧怎么走完全链路

> 入口索引与工作约定在仓库根目录 [`AGENTS.md`](../../AGENTS.md)。
> 本文件回答「**每帧实际按什么顺序调、数据怎么流、跨帧记忆住在哪**」。
> 「模块边界为什么这么切」在 [`architecture.md`](architecture.md)；「参考项目怎么写的」在 [`reference-pipeline.md`](reference-pipeline.md)。

**本文件状态：以 `【事实】` 为主。** 内容由 `src/` 现行代码读出，行号对应当前 `dsh` 分支。标 `【设计】` 的是「为什么这么排」。**行号会漂——动手前先 grep 确认，不要相信任何抄下来的行号。**

## 一、为什么需要这一份

参考项目的运行顺序是**代码自明**的：[`pipe2025_func.py`](../../reference/pipe_inspector_raspberry-main/main_process/pipe2025_func.py) 从上往下读就是顺序，`_pipe1 → _turn_first → _pipe2 → _turn_second → _pipe3`，一个函数执行完就不再执行。

本项目的顺序**在任何单个模块里都看不见**。每个模块只提供 `observe()` / `step()` 这类「被调一次」的方法；谁调它、什么时候调、按什么顺序调，只出现在 [`runtime.py`](../../src/bionic_fish/runtime.py) 的 `Runner._iterate` 里。

- 【事实】这是「每帧把各部件转一圈」的代价：读单个模块永远看不出顺序
- 【设计】这是刻意的。模块之间互不引用（第四节），所以每个模块都能单独测、单独换；代价是顺序成了**只有一处**才知道的知识
- 【事实】这一页以前不存在，2026-10-02 补上。见 [logs/2026-10-02-package-and-entry-point](../logs/2026-10-02-package-and-entry-point.md)

## 二、入口：三个，最后都汇到一个循环

| 入口 | 怎么用 | 状态 |
| --- | --- | --- |
| 命令行 | `python -m bionic_fish` | 【事实】[`__main__.py`](../../src/bionic_fish/__main__.py)，2026-10-02 加入 |
| 合成闭环 | `Runner(cfg, arena).run_synthetic(...)` | 【事实】当前**唯一**能跑完整链路的路径（没有相机） |
| 录像 / 相机 | `Runner.run_frames(...)` / `Runner.run_camera(...)` | 【待验证】真机路径，未在真机上跑过 |

【事实】三个入口最终都调 `Runner._iterate`（`runtime.py:154`）。

**所以：读运行顺序 = 读 `_iterate`。** 其余 4000 行都在回答「这一步具体怎么算」。

## 三、三层结构

```
第 0 层  组装          Runner.__init__          造出 8 个部件，互相不引用
第 1 层  每帧一圈      Runner._iterate          13 步，顺序的唯一住所
第 2 层  部件内部顺序  各 observe() / step()    每个部件自己的判断次序
```

看代码时的顺序必须是 **0 → 1 → 2**。反过来读（先钻 `vision.py`）会迷路，因为那份代码里没有任何东西告诉你「它什么时候被调、拿到的 frame 是哪来的」。

## 四、第 0 层：组装（`runtime.py:128-152`）

【事实】`Runner.__init__` 造 8 个部件，全部只依赖 `Config`，**它们之间互不引用**：

| 部件 | 构造自 | 职责 |
| --- | --- | --- |
| `StripTracker` | `cfg.strip` + `cfg.calibration` | 灯带中心线 |
| `MarkerDetector` | `cfg.marker` + `cfg.calibration` | 暗斑候选 |
| `CrossLineDetector` | `cfg.marker` + `cfg.calibration` | 起终点黑带 |
| `Reporter` | `cfg.report` + `cfg.marker` + `cfg.calibration` | 上报判决 |
| `LineController` | `cfg.control` + `cfg.channels` | 打舵 + 前进 |
| `StateMachine` | `cfg.fsm` + `cfg.safety` + `cfg.control` + `cfg.channels` | 状态 + 四条硬终止 |
| `link` | 默认 `SimLink` | 硬件出口 |
| `annunciator` | `cfg.annunciator` + `LoggingSpeaker` | 声光 + 播报 |

- 【事实】**`StripTracker` 不认识 `Reporter`，`Reporter` 不认识 `StateMachine`。** 所有连接都发生在 `_iterate` 里，而且是每帧现连一次
- 【事实】这条纪律的收益：三份 agents 文档里说的「感知不许决定该不该上报」，在代码里就是**没有那根引用**
- 【事实】`reset()`（`runtime.py:154`）重置三个有跨帧记忆的部件（`reporter` / `controller` / `machine`）；感知层不用重置，因为它没有记忆（第八节）

## 五、第 1 层：每帧的 13 步（**核心**）

【事实】`runtime.py:174-240`，一个 `for`，13 步：

| # | 行 | 做什么 | 失败的表示 |
| --- | --- | --- | --- |
| 1 | 175-177 | 取一帧 `(now_s, frame)`；`dt = now_s - last_now` | 超过 `max_steps` 就停 |
| 2 | 180 | **感知①** `tracker.observe(frame)` → `line` | `line.valid = False` |
| 3 | 181 | **感知②** `markers.observe(frame, line)` → `cands` | 空列表 |
| 4 | 182 | **感知③** `cross.observe(frame, line)` → `cross` | `cross.detected = False` |
| 5 | 184-186 | **状态** `machine.step(line, cross, now_s, last_forward_us, link.yaw_rad())` → `decision` | `decision.terminal` |
| 6 | 187 | **上报** `reporter.update(cands, now_s, cross)` → `events` | 空列表 + 闸计数器 |
| 7 | 189-190 | 每个 event：存日志 + `annunciator.on_report()` 亮灯/蜂鸣/播报 | — |
| 8 | 191 | `annunciator.tick(now_s)` 到点关灯关蜂鸣 | — |
| 9 | 193-200 | **控制** `controller.step(line, decision.target_forward_us, now_s, dt, corner=..., yaw_override_us=...)` → `actuator` | 保持上一帧舵量 |
| 10 | 201 | **发出** `link.send(actuator)` | — |
| 11 | 202 | `last_forward_us = actuator.forward_us`（留给下一帧的状态机） | — |
| 12 | 204-205 | 合成模式才有：`plant.step(actuator, dt)` 让鱼动一步 | — |
| 13 | 208-231 | 记账（帧数、状态、横向误差）；`decision.terminal` 就 `break` | — |

**一句话记忆**：

```
感知 → 判断 → 上报 → 响 → 控制 → 发 →（鱼动）→ 记账
```

### 五个不看代码想不到的顺序事实

1. 【事实】**上报发生在控制之前**（187 早于 193）。这一帧就算上报了，方向盘也已经在同一帧算完了——**上报不改变当前动作**。
2. 【事实】**`link.yaw_rad()` 是在状态机内部被调的**（184）。`StateMachine` 不持有 `link`；航向是**从外面喂进去**的。所以没有航向源时，状态机看到的是 `None`（不是 0）——`_turned_rad()` 会把 `None` 当「未知」，绝不当作「还没转」。
3. 【事实】**`last_forward_us` 是上一帧的输出**（202 → 下一帧的 184），而且刻意用**实际发出的值**而不是状态机想要的目标值。"状态机要求动"不等于"执行器动了"，而「30 秒未启动」判的是后者。
4. 【事实】**感知每帧无条件全跑**（180-182），不管状态机在 `IDLE`、`DONE` 还是 `ABORT`。感知层完全不知道策略存在。
5. 【事实】**`dt` 来自帧时间戳**（177），不是循环自己数的。所以录像快放慢放时，PID 的微分项自动跟着变。`Pid.step` 另外把落在 `[1e-3, 0.5]` s 之外的 `dt` 当抖动丢掉（`control.py:109`）。

> 【事实】2026-10-02 顺手删掉了 `_iterate` 里一个**从未被使用的 `dt_s` 参数**：三个调用方都传了它，函数体里却一律用帧时间戳算 `dt`。合成模式下两者恰好相等（帧由 `step * dt_s` 生成），所以一直没暴露。

## 六、第 2 层：各部件内部的顺序

### 6.1 三个观察者是串联的（`vision.py:268 / 389 / 526`）

```
frame ──▶ StripTracker.observe(frame)          ──▶ LineObs
              │
              ├──▶ MarkerDetector.observe(frame, line)   ──▶ list[MarkerCand]
              └──▶ CrossLineDetector.observe(frame, line) ──▶ CrossObs
```

- 【事实】**只有第一个吃原始帧**；后两个都要 `line`。暗斑要算「离灯带中心多远」（`lateral_offset_px`），黑带要确认「灯带在不在」——横穿灯带这个事件没有灯带就没有定义
- 【事实】这个依赖顺序是硬性的，180 → 181 → 182 **不能换**

### 6.2 `StateMachine.step` 的判断次序（`fsm.py:331-442`）

```
1. 是终态吗？→ 是就直接返回                    fsm.py:350
2. safety.update() → 四个硬终止，按此序查：     fsm.py:353
     脱线 > 10s → 撞壁/触底 3 次 → 30s 没动 → 150s 到
3. IDLE   → 舵回中、不走                        :364
4. ALIGN  → 对齐了 → FOLLOW ／ 超时了也 FOLLOW ／ 否则微速前进  :367
5. FOLLOW / CORNER / LINE_LOST：                 :381
     - 丢线   → LINE_LOST（但弯角里丢线不清除转弯）  :382
     - 重获   → FOLLOW                            :390
     - FOLLOW → 弯角线索连续确认 N 帧 → CORNER + 起转  :395
6. 弯角推进 _advance_bend()（**独立于状态**）      :409
7. 终点黑带？需「已跑够时间」才认（起点黑带长得一样）  :419
8. FINISH_APPR → 蠕行 finish_creep_s → DONE       :423
9. 都不是 → _forward_target() + _yaw_override() 组 Decision  :435
```

- 【设计】第 6 步**独立于状态**：灯带在弯中途必然消失，若丢线顺手把弯角逻辑也关了，**唯一能终止转向的机制会在最需要它的时候失效**
- 【设计】转弯的终止判据按可信度排序（`fsm.py:443-498` 的 `_advance_bend`）：**按角度**（需航向源，唯一不依赖灯带的判据）> **按重新看见**（须先转过 `corner_min_commit_s`）> **反向搜一次** > **超时交给恢复**

### 6.3 `Reporter.update` 的七道闸（`reporting.py:239-339`）

【事实】先给所有轨迹 `misses += 1`，再对每个候选按固定次序过闸，**每道不过就 `continue` 并记一个计数器**：

| 序 | 判据 | 不过时计入 | 行 |
| --- | --- | --- | --- |
| 1 | `accepts_marker()`：形态 / 尺寸 / 判别特征 | `rejected_discriminator` | 255 |
| 2 | `_in_tape_zone()`：撞上起终点黑带 | `rejected_tape_zone` | 258 |
| 3 | `candidate_score()`：最弱一环得分为 0 | `rejected_shape` | 263 |
| 4 | `_associate()`：找不到旧轨迹就新建一条 | `tracks_created` | 267 |
| 5 | 确认帧数够 + 正在逼近 | `rejected_unconfirmed` | 296 |
| 6 | 这条轨迹是否已经报过 | `rejected_duplicate` | 301 |
| 7 | 行号到触发行了吗 | `rejected_not_yet_coincident` | 305 |
| 8 | 置信度过阈值吗 | `rejected_confidence` | 310 |

- 【设计】数这 10 个计数器是因为**漏检和「看见了被否决」是排查的第一个分叉**；没有它们，两种情况的日志长得一样（[troubleshooting](troubleshooting.md)）
- 【事实】报过的轨迹**永久保留**（`reporting.py:333-338`），它的身份就是「同一个标记不会报两次」的实现；没报过的连续 `max_misses` 帧没见就删

### 6.4 `LineController.step` 的三条分支（`control.py:229-289`）

```
1. 有 yaw_override？ → 直接用固定舵量（飞弯角），并把 PID 历史 hold 掉   control.py:253
2. 灯带 valid？      → 正常串级：heading_error = desired_heading() + e_theta_rad  :260
3. 都不行？          → 保持上一帧舵量；两个 PID 都 hold，**绝不当成「居中」**  :263

然后：forward_us() 限斜率逼近目标 → 各自 clamp → 按 ChannelMap 组装字典  :270-283
```

- 【事实】只有第 2 条分支会更新 PID；第 1、3 条都调 `hold(now_s)`，把误差历史清空——否则恢复控制时会对一个几帧前的误差做微分，产生尖峰
- 【设计】切换弯角增益组时会 `self.yaw.reset()`（`control.py:197`，在 `steer_offset` 里），同一个理由：不在不连续点两侧做微分

## 七、一场比赛的完整生命历程（状态视角）

【事实】把第五节和 6.2 串起来，一条能跑通的合成路径（`--lateral-mm -30`）是这样走的：

| 阶段 | 状态 | 进入后发生什么 |
| --- | --- | --- |
| 发令 | `IDLE` → `ALIGN` | `machine.start(0.0)`；安全计时开始（`ALIGN` 是 `start()` 里进的，不是 `step()`） |
| 起步 | `ALIGN` | 灯带够正 → `FOLLOW`；超时也硬着头皮 `FOLLOW`（因为「30s 未启动」是硬终止） |
| 直道 | `FOLLOW` | 每帧：`e_y_px` → 期望航向 → 内环 PID → 舵量；定高环盯灯带宽度 |
| 过弯 | `CORNER` | `confidence` 连续塌陷 N 帧 → 起转，`yaw_override_us` 接管舵轴 |
| 弯中丢线 | `LINE_LOST` | 但 `_lost_from_corner=True`，**舵量照旧压着**，不松手 |
| 转完 | `FOLLOW` | 按角度测到 90° → 结束转弯；或超时 → `LINE_LOST` 走恢复 |
| 见终点 | `FINISH_APPR` | 黑带 + 已跑够时间 → 蠕行，让整条鱼压过黑带 |
| 收工 | `DONE` | `decision.terminal=True` → 循环 `break` |

- 【事实】**上报（`Reporter`）是与状态机完全平行的一条线**：它不读状态，只吃 `cands` 和 `cross`
- 【事实】因此 `IDLE` 在 `Runner` 的两次运行里**都不会出现**——两个入口都在进循环之前调了 `machine.start()`。`IDLE` 分支只在直接单测状态机时可达

## 八、跨帧记忆住在哪（顺序的另一半）

【事实】「每帧转一圈」要能工作，靠的是各部件把记忆存在自己的实例字段里。**这张表就是顺序的另一半**：

| 部件 | 跨帧字段 | 丢了会怎样 |
| --- | --- | --- |
| `StripTracker` | **无**（只有 params / calibration / 预计算核） | — |
| `MarkerDetector` | **无** | — |
| `CrossLineDetector` | **无** | — |
| `Reporter` | `tracks` / `stats` / `_next_track_id` / `_reports_emitted` | 同一个标记报两次（重复上报按误报 −3） |
| `StateMachine` | `state` / `_corner_hits` / `_bend_active` / `_turn_start_yaw` / `_bend_finished_at` … | 每帧都当第一帧：弯角永远确认不了，或永远转不完 |
| `SafetyMonitor` | `_armed_at` / `_off_track_since` / `_wall_hits` … | 四条硬终止全部失效 |
| `LineController` | `_forward_us` / `steer_offset_us` / `_last_mode_corner` | 前进指令跳变；增益切换时微分尖峰 |
| `Pid`（×2） | `_integral` / `_previous_error` / `_previous_time` | 积分清零、微分失效 |
| `Annunciator` | `light_off_at` / `buzzer_off_at` / `announcements` | 灯不灭；播报与上报对不上（漏播 −2） |

- 【事实】**感知层完全没有跨帧记忆**。这是它能被逐帧单独测试、也能被并行化的原因（[architecture](architecture.md) 第七节）
- 【事实】`Runner.reset()` 只重置 `reporter` / `controller` / `machine` 三处——正好覆盖上表里真正有记忆的那些

## 九、把顺序「看出来」而不是「读出来」

【事实】2026-10-02 起，`Runner(trace=True)` 会逐帧记一条 `FrameTrace`（`runtime.py:47`），命令行用 `--trace` 打出来：

```sh
python -m bionic_fish --steps 40 --trace --trace-limit 40
```

```
      t state       line   e_y_px    e_th  width  conf cand rep target   yaw   fwd  note
   0.00 ALIGN        yes     23.1  -0.004   20.0  1.00    0   0   1512  1520  1500  start command received
   0.05 ALIGN        yes     23.1  -0.004   20.0  1.00    0   0   1512  1520  1503  start command received
   ...
   1.60 FOLLOW       yes     15.5  -0.082   20.0  0.95    1   0   1560  1514  1515  aligned on the strip
```

列的排序**就是第五节的执行顺序**：感知（`line` / `e_y_px` / `e_th` / `width` / `conf`）→ 上报闸（`cand` / `rep`）→ 状态机（`state` / `target`）→ 控制（`yaw` / `fwd`）。

- 【事实】它直接把顺序变成可观察的数据：`target` 是状态机要的，`yaw`/`fwd` 是真正发出去的，两者不同就是「限斜率」在起作用（看 `t=0.00`：target 1512、fwd 1500）
- 【设计】这是补充，不是替代——顺序的唯一权威仍然是 `_iterate` 那 13 步

## 十、给读代码的人的建议顺序

**有两条路，按目的选，不要混着走。**

### 路径 A：想知道「代码怎么跑起来」（含感知与仿真）

1. `runtime.py:174-240`（**67 行**）——把「每帧一圈」彻底看懂，这是全部
2. `runtime.py:128-152`（组装）——知道谁被造出来、谁不认识谁
3. `fsm.py:331-442` 的 `step()`——判断次序，本文件 6.2 那张表
4. `control.py:229-289` 的 `step()`——三条分支
5. 然后才是感知：`vision.py:268 / 389 / 526` 三个 `observe()`（[vision](vision.md)）
6. 最后 `reporting.py:239-339`——七道闸

### 路径 B：只想读懂**逻辑链**（不碰感知实现、不碰仿真）

**前提**：逻辑链**不依赖感知的实现**，也不依赖仿真。它只依赖**跨模块传递的那 6 个数据结构**——把这 6 个看懂，逻辑链就通了：

| # | 类型 | 谁产生 | 谁消费 | 关键字段 |
| --- | --- | --- | --- | --- |
| 1 | `LineObs` | `StripTracker` | `fsm` + `control` | `valid` / `e_y_px` / `e_theta_rad` |
| 2 | `MarkerCand` | `MarkerDetector` | `reporting` | `u` / `v` / `val` / `circularity` |
| 3 | `CrossObs` | `CrossLineDetector` | `fsm` + `reporting` | `detected` / `row_lo` / `row_hi` |
| 4 | `ReportEvent` | `Reporter` | `Annunciator` + 日志 | `sequence`（播报序号） |
| 5 | `Decision` | `StateMachine` | `control` | `target_forward_us` / `corner` / `yaw_override_us` |
| 6 | `Actuator` | `LineController` | `Link` | `channels` / `yaw_us` / `forward_us` |

读的顺序（**约 90 分钟，全程不需要相机、不需要仿真、不需要 `config.py`**）：

| 步 | 读哪里 | 你应该能回答 |
| --- | --- | --- |
| 1 | `runtime.py:174-240` | 一帧里各部件按什么次序被调 |
| 2 | `vision.py:52-147`——**只读三个观测结构的字段与注释，跳过实现** | 每个下游模块能拿到什么、拿不到什么 |
| 3 | `reporting.py:239-339` | 一个候选要过哪 8 道闸才变成一次上报 |
| 4 | `fsm.py:331-442` | 什么条件进什么状态；四条硬终止在哪一步查 |
| 5 | `control.py:229-289` | 舵量与前进量各自怎么算出来 |
| 6 | `platform.py:51-73` 的 `Link` 协议 + `:257-265` 的 `send` | **实物上唯一要换的是什么** |
| 7 | `tests/test_control_reporting_fsm.py`——**49 个用例名** | 每条规则的**可执行规格** |

**为什么第 7 步是测试**：那 49 个用例名就是一份规则清单——`test_needs_temporal_confirmation`、`test_never_reports_the_same_track_twice`、`test_a_stationary_candidate_is_not_an_approach`、`test_off_track_timeout`、`test_corner_commits_to_a_bounded_turn`、`test_terminals_are_sticky`……**读名字**比读实现快得多，而且它们同时就是改逻辑时的回归网。整个文件 624 行，但它是**逻辑的规格**，不是逻辑的副本。

> [!IMPORTANT]
> **要上实物时，这条链上只有第 6 步会变。** `platform.Link` 是一个协议，`SimLink`（开发机）与 `MavlinkLink`（真机）是它的两个实现；换后端只需要在构造 `Runner` 时注入，**逻辑链一行都不用改**。这也正是它被设计成协议的原因——见 [architecture](architecture.md) 第 8.2 节。

### 什么**不要**先读

| 不要读 | 为什么 |
| --- | --- |
| `config.py`（758 行） | 全是参数声明，没有控制流。它是用来**查**的，不是用来**读**的 |
| `arena.py`（487 行） | 合成场地。不跑仿真就不需要它 |
| `plant.py`（113 行） | 占位动力学，**已知错**（[architecture](architecture.md) 耦合 12），换真机时会被换掉 |
| `vision.py` 的实现部分 | 它是最不稳定的一层（阈值全待标定），而逻辑链不依赖它 |

---

## 变更记录

| 日期 | 变更 |
| --- | --- |
| 2026-10-02 | 首次建立：三层结构、每帧 13 步、四个部件内部次序、状态生命历程、跨帧记忆表、`--trace` 用法；记录删除 `_iterate` 死参数 `dt_s` |
| 2026-10-02 | **全文行号重新 grep 校正**（此前是按增量推算的，错了）：`_iterate` 174-240、`fsm.step` 331-442、`reporting.update` 239-339、`control.step` 229-289、三个 `observe` 268/389/526。第十节拆成**两条路**：路径 A（看它怎么跑，含感知与仿真）与**路径 B（只读逻辑链，不碰感知实现与仿真）**，并补上「跨模块传递的 6 个数据结构」表与「上实物时这条链上只有 `Link` 会变」 |
