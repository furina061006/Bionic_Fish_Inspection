# 单关节尾鳍：接口落地，以及第一轮自测查出的六处纰漏

**日期**：2026-10-13
**背景**：下位机只接收**一路** PWM（一个舵机，串口透传，1500µs = 50% 占空比），
所以上位机必须把 `yaw` + `forward` 合成成**一个**关节指令，并且本地生成摆动波形
（主循环 15–30 fps，做不出 2–5 Hz 的平滑尾摆）。

> [!IMPORTANT]
> 这条日志记的是**当时的过程快照**。现行事实在
> [`../agents/hardware.md`](../agents/hardware.md) 的「执行器」一节。
> 下面第 4 节有一处**我自己先前的误判被撤回**，照约定不回改正文，在这里写明。

---

## 一、落地的东西

| 文件 | 内容 |
| --- | --- |
| [`fin.py`](../../src/bionic_fish/fin.py) | `TailFinWaveform`：波形线程 + 串口发送；`open_serial`；`RecordingSerial`（测试用假串口） |
| [`drive.py`](../../src/bionic_fish/drive.py) | `synthesize_tail_fin(yaw, forward, p) → (amplitude, bias)`，纯函数 |
| [`link.py`](../../src/bionic_fish/link.py) | `TailFinLink`：与 `MavlinkLink` 同形状（`connect`/`arm`/`disarm`/`close`），但对外只有 `send_tail_fin` |
| [`main.py`](../../src/bionic_fish/main.py) | 第 6 步按 `hasattr(link, "send_tail_fin")` 分叉，**合成只在这一处发生** |
| [`__main__.py`](../../src/bionic_fish/__main__.py) | `--tail-fin`（默认）/ `--mavlink` 选后端；`--device` 覆写串口 |
| [`params.py`](../../src/bionic_fish/params.py) | 尾鳍参数 8 个 + 两个派生量 `tail_fin_bias_scale` / `tail_fin_amp_scale`，并进 `check()` |
| [`tests/test_logic.py`](../../tests/test_logic.py) | 新增 `TailFinTest` **16** 条；全套 18 → **34** 条 |

改动量（`git show --stat cda9218`）：14 个文件 +1032 / −43 行。其中 `fin.py` 172 行
（新文件）、5 个源文件、`tests/test_logic.py` +284 行（`TailFinTest` 16 条，
全套 18 → 34）、6 份 Markdown 文档。

数据流：

```
main.py（15–30 fps）  yaw / forward
      ↓ synthesize_tail_fin()          ← 纯函数，drive.py
   (amplitude, bias)  归一化
      ↓ link.send_tail_fin()
   fin.TailFinWaveform  ── 线程 100 Hz ──> pwm(t) = neutral
      │                                      + bias  * bias_scale
      │                                      + ampl  * amp_scale * sin(2πft)
      ↓ 串口文本（每行一个 µs 值 + \n）
   下位机 → 舵机 PWM
```

**为什么不把波形塞进主循环**：帧率一掉到 15 fps，3 Hz 的摆动只剩 5 个采样点/周期，
尾摆会抖成锯齿。主循环只管更新目标（慢变量），相位推进在独立线程里。

---

## 二、自测查出的六处纰漏

写完先跑了一遍探针（不是「确认它能跑」，是主动找它什么时候会静默出错），
六条里三条是真 bug。

### 1. 串口打不开，`connect()` 照样返回 True（真 bug，安全性）

原实现 `open_serial()` 吞掉所有异常、返回一个 `_RecordingSerial`，于是
`TailFinLink.connect()` **永远成功**。后果：真机上串口接错、或者 `pyserial` 没装
（`pip install -e .` 不带 extras），程序会打印一行友好的提示、跑完全程、四条硬终止
全部正常、`Result` 漂漂亮亮，而**舵机一帧都没收到**。这正是 `link.py` 开头为
`MavlinkLink` 写下的那条教训（「不解锁就跑等于白跑」）换了个形式重犯。

**改法**：`open_serial()` 失败就抛，`connect()` 转成 `last_error` + `False`；
假串口改名 `RecordingSerial` 且**只由测试 / 仿真显式注入**，实机路径没有兜底。
`__main__.py` 的失败提示也改成「这里不会退化成『假装发送』」。

### 2. 坏参数静默杀死波形线程（真 bug）

`Params.check()` 原来一个尾鳍参数都不校验。实测 `tail_fin_wave_rate_hz = 0.0`：

```
p.check() 放过了 tail_fin_wave_rate_hz=0.0
波形线程还活着吗: False        发出了几帧: 0
ZeroDivisionError: float division by zero     ← 在子线程里，主循环收不到通知
```

**改法**：`check()` 补 8 条（含一条**混叠**检查：`freq * 2 <= wave_rate`）；
另外 `TailFinWaveform.start()` 自己也会 `check()` 一遍，参数非法就**拒绝启动**
并返回 `False`（而不是起一个必然死掉的线程）。

### 3. 预算约束从不触发 —— 耦合 12 是空实现（真 bug）

原实现把 `yaw` 和 `forward` **都**按 `pwm_max - neutral = 400` 归一化，于是：

```
峰值需求 0.300 (budget=1.0)     预算触发帧数 0/273
```

一整趟**一次都没触发**。写进文档、标成「实现耦合 12」的预算分配是死代码：两个量
各自从同一根刻度独立映射，谁也没和谁争，「大舵量吃掉推力」根本没发生。

**改法**：两套量纲分开归一化 —— 偏置除以 `tail_fin_bias_scale`（满舵 = 1.0），
振幅除以 `tail_fin_amp_scale`（满推力 = 1.0）；`fin.py` 也改在**微秒域**里合成，
把两个基准分别乘回去。

这条修正**不改变**未触发预算时的物理输出：旧式子代数上等于
`pwm = yaw + (forward - neutral)·sin`（中心 = yaw、摆幅 = ±(forward − neutral)），
新写法在同样条件下给出同一个值 —— 有专门一条用例钉住它
（`test_unscaled_synthesis_reproduces_the_joint_command`）。

### 4. 撤回：先前「旧参数被重新解释、量级缩水 20 倍」的说法是错的

我先前把「满舵 20µs 只占舵机 ±400µs 行程的 5%」说成「量级缩水 20 倍」，**这个框架
不成立**：旧式子的代数结果就是 `中心 = yaw`、`摆幅 = ±(forward − neutral)`，
20µs 的满舵被正确地映射成 20µs，没有缩水。`clamp_us = 20` 本来就是舵量的实际上限
（`params.py` 原话「舵量相对中位的上限」）。

真正成立的只有第 3 条（预算不触发）。而「一整趟只用到舵机 28% 的机械行程」
是个**硬件 / 整定问题**（要么舵机选得量程过大、要么摆幅偏小），不是代码缺陷。

### 5. 新代码零回归测试（已补）

`tests/test_logic.py` 当时仍是 18 条，没有一条碰合成或波形 —— 而耦合 12 是有文档的
设计不变量。已补 `TailFinTest`（最终 16 条，第 16 条出自第四节的复审）：耦合会削推力、
默认关闭、预算不越界、恒等映射、波形中心与摆幅、停摆回中位、坏参数拒绝启动、
串口失败必须响、**`TailFinLink` 不许有第二条合成路径**、**走完整入口的端到端**。

### 6. 文档日期是编的（已改）

初版日志与 `hardware.md` 的变更行写成 `2026-01-XX`，而当天是 **2026-10-13**；
日志里的「新增 447 行」与 `git diff` 也对不上。本文件即为重写版。

---

## 三、一个必须记下来的后果：耦合打开后，仿真过不了弯角

修正第 3 条之后预算真的会触发了，于是同一份合成场地闭环出现下面的分歧
（用反解适配器把 `(amplitude, bias)` 还原成 `(yaw, forward)` 喂给占位动力学，
三种起始横向偏置）：

| `tail_fin_budget` | 结果 | 用时 |
| --- | --- | --- |
| `1.0` | **完全脱离路线超过 10 秒**（硬终止 ①） | 20.1 / 21.4 / 20.9 s |
| `1.5` | **完全脱离路线超过 10 秒**（硬终止 ①） | 22.4 / 19.9 / 19.6 s |
| `2.5`（≈ 关） | 到达终点，8 上报 | 27.2 / 27.2 / 27.3 s |
| `None`（默认） | 到达终点，8 上报 | 27.2 / 27.2 / 27.3 s |

原因很清楚：`corner_turn_us = 20` **就是满舵**，而弯角状态必然是「满舵 + 减速」，
一进预算就被按比例削，占位动力学需要那个舵量才转得动 90°；转不过去 → 丢线 → 10 秒
硬终止。**这是耦合第一次真实生效的后果，不是回归。**

于是默认值定为 **`tail_fin_budget = None`（关闭）**，理由：

1. `sim/plant.py` 的动力学**已知是错的**（它把推力与转向当两个独立轴），
   拿它整定等于拟合虚构（项目红线）。
2. 但也不能假装耦合不存在 —— 机器实现了、用例钉住了，缺的只是
   **实测的「转向 + 推力」权衡曲线**。

`tests/test_logic.py::test_the_budget_is_off_by_default_and_that_is_deliberate`
就是防「有人悄悄把默认值打开」的闸门：一打开，闭环就从「到达终点」变成硬终止。

---

## 四、提交前复审又查出一处（第七处）：默认配置下实机入口必崩

**提交前把所有改动重读了一遍**（不是重跑测试 —— 测试已经全绿了），查出一处
**前面两轮自测都漏掉的真 bug**：

```python
# src/bionic_fish/__main__.py（修复前）
print(f"[尾鳍模式] 摆动 ... 预算 {p.tail_fin_budget:g}（转向权重 ...）")
```

`tail_fin_budget` 默认是 **`None`**，而 `f"{None:g}"` 抛
`TypeError: unsupported format string passed to NoneType.__format__`。

**为什么两轮自测都没抓到**：崩点在**串口连上之后**的打印那一行，而

- `--help` 在参数解析阶段就返回了，走不到那里；
- 开发机上 `serial` 模块根本没装，`connect()` 在更早处返回 `False` → `SystemExit`
  —— **「连不上就退出」这条路径恰好绕过了 bug**；
- 单元测试一次都没调过 `__main__.cli()`，只测了 `TailFinLink.connect()`。

也就是说：**这个 bug 只在真机上出现，而真机恰恰是唯一还没验证过的环境**。
这正是「开发机的失败路径掩盖了真机的成功路径」这一类错误。

**修法与防线**：

1. 摘要行抽成 `_tail_fin_summary(p)`，`None` 打印成「关闭」；
2. 补一条**走完整入口**的用例 `test_the_vehicle_entry_point_survives_the_default_config`
   —— 用 `mock.patch` 换掉 `TailFinLink`（注入假串口）与 `Camera`，其余全走真代码：
   参数加载 → 后端选择 → 摘要 → `run()` → `finally` 里的 `disarm`/`close`；
3. **验证了这条用例真的能抓住它**：把 `f"{...:g}"` 那段临时放回去，用例 FAIL；
   还原后 OK。不这么试一遍，就不知道防线是不是空的。

复审同时清掉的小问题：

| 问题 | 处理 |
| --- | --- |
| `fin.py:start()` 用整个 `p.check()` 当闸门 | 改成只挡会让线程崩的两条。否则一个无关字段（如 `camera_index`）出错会报成「波形参数非法」，把人引向错方向 |
| `main.py` 的 Protocol 叫 `TailFinLink`，与 `link.TailFinLink` **完全同名** | 改名 `TailFin`，并在 docstring 里写明「运行时是 `link.TailFinLink`」 |
| `params.py` 注释写「这四条」而实际有 8 条校验 | 改成「这几条」 |
| `--tail-fin` 解析了却从不被读 | 加注释说明它只用于互斥报错（`--tail-fin --mavlink` 必须报错而不是静默二选一） |
| 测试里混叠用例塞在坏参数循环里、带一个 `if` 特例 | 拆成独立的 `test_an_aliasing_wave_rate_is_rejected`；顺手清掉一处格式错乱 |
| `backlog` 的 `[#4]` 标题还写「后端选型」 | 选型已定，改为「尾鳍参数标定与实物定型」 |

**留下的教训**：`None` 是个会到处漏的默认值 —— 凡是 `X | None` 的字段，任何
`f"{x:...}"` 都是一颗雷。复审时用 `grep` 把所有 `:g` / `:f` / `:d` 的格式化点过了一遍。

---

## 五、提交

`cda9218`（分支 `dsh`，**未合并到 `main`**，按纪律要用户明确确认才合）。

提交前跑过的四道闸门：

```
全量测试     34 条 OK（18 旧 + 16 尾鳍）
文档链接     416 个相对链接全部可达
仿真基线     到达终点 25.45s / 8 上报 / 5000mm  ← 与本次改动前的文档基线逐字一致
编译         src/ sim/ tests/ 全部通过
```

---

## 六、还没解决的

- **仿真壳覆盖不到尾鳍路径**：`SimLink` 只有 `send(yaw, forward)`，所以 `sim/`
  走的仍是旧分叉（本次是靠日志外的反解适配器才测到闭环）。要真正覆盖，得先按
  文档要求重建 `plant.py`（单关节动力学），否则反解是有损的。
  → [`backlog.md`](../../backlog.md)
- **`tail_fin_budget` / `tail_fin_yaw_weight` / `tail_fin_freq_hz` 全部未标定**。
- **下位机串口协议**（文本 / µs + `\n`）是**我们单方面定的**，下位机固件还没对过。
- **树莓派上的线程抖动**未实测：100 Hz 的 `time.sleep` 在负载高时会漂，
  实测不够就降到 50–80 Hz 或换硬件 PWM。
- **舵机只用到 28% 机械行程**（见第二节第 4 条撤回段）：要不要放大摆幅是硬件问题。
- **波形线程没有看门狗**：`start()` 现在挡住了已知会崩的参数，但**未知异常仍会把线程
  打死而不留痕迹** —— 主循环不查 `link` 是否还活着，四条硬终止也不管执行器。
  真要堵死，得让 `main.py` 有一个「执行器还活着吗」的判据（第三个接口契约），
  并按硬终止处理。本次没做，因为它是**新契约**，不该顺手塞进这一版。
  ⚠️ 这条是**已知残留风险**，不是待办清单上的普通一项。
