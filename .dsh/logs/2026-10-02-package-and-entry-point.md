# 2026-10-02 · 打包、入口文件与「代码运行顺序」文档

**触发**：审阅者读完 `_iterate` 之后说「这个项目比参考项目乱很多」「参考项目把核心代码和包分开存储，我们这个项目没有包吗」。

**结论**：包结构**保持扁平不拆**；补上真正缺的两样——打包元数据和命令行入口；并补一份此前不存在的文档 [runtime-flow](../agents/runtime-flow.md)。

---

## 一、先取证：参考项目到底有没有包结构

我最初的判断是「参考项目的 `packaged_part/` 是假分层，靠 namespace package 侥幸能跑」。**这个判断错了**，是查证之后改的：

```sh
find reference/pipe_inspector_raspberry-main -name "__init__.py" | wc -l   # 15
ls packaged_part/*/                                                       # 4 个业务子包 + 1 个 vendored
```

- 4 个业务子包 `visual` / `move` / `pid_catalog_first` / `camera_gimbal_control` **各自都有 `__init__.py`**，都是真包
- 主程序 `main_process/pipe2025_func.py:5-11` 从 5 个子包各拉一次，**全部是 `import *`**

所以准确的说法不是「它没有包」，而是：

> 它的分层依据是**用途 / 来源**（视觉、运动、PID、云台灯），粘合方式是 `import *` 拉进同一个全局命名空间。

**中途改正过的地方**（按本目录的约定要写下来）：我先有结论再去查，差点把「它是 namespace package」写进文档。行号和 `__init__.py` 这类事实必须 grep / ls 出来，不能从「看起来像」推。

## 二、我方现状（取证）

```sh
find . -name "__init__.py" -not -path "./reference/*"    # 只有 src/bionic_fish/__init__.py
ls pyproject.toml setup.py setup.cfg requirements.txt     # 四个都没有
python -c "import bionic_fish"                            # ModuleNotFoundError
```

- 【事实】`bionic_fish` **是**一个规范的包（有 `__init__.py`），**是平的**：11 个模块、约 4140 行、没有子包
- 【事实】但**项目本身不是可安装的包**：没有 `pyproject.toml` / `setup.py` / `requirements.txt`，所以每一步都要 `PYTHONPATH=src`
- 【事实】也没有任何入口：`__main__` 只出现在三个测试文件里，跑一次系统必须从测试文件里抄 `Runner(cfg, arena).run_synthetic(...)`

审阅者的感受有一个准确的落点：**「没有包」说的其实不是 `bionic_fish` 不是包，而是这个项目不能作为一个包被安装和使用。**

## 三、决定：不拆子包，理由与触发器写进架构文档

不拆的四条理由（已写进 [architecture 8.2/8.3](../agents/architecture.md)）：

1. 11 个模块是扁平包的正常规模；子包要到二三十个模块才有意义
2. 加深度**不带来新边界**。参考项目的 4 个子包之间是 `import *` 双向耦合，就是「加了目录没加边界」的实证
3. 我们的模块**就是**分层图本身；按层切已经做到，按人/按来源切才是要避免的
4. **最直接的一条**：运行顺序只存在于 `runtime.py` 的 `_iterate` 里（43 行）。拆更多目录只会让那 43 行更难被找到——而「顺序看不见」正是这次困惑的根源

留下两个**可执行的触发器**而不是模糊的原则：

| 对象 | 决定 | 触发条件 |
| --- | --- | --- |
| `vision.py`（599 行） | 现在不拆 | >800 行 / 加第 4 个检测器 / `MarkerDetector` 需独立标定 |
| `config.py`（688 行） | **明确不拆** | —（它的体积来自「阈值唯一住所」这条红线，拆开正是红线要防的） |

## 四、补的东西

### 4.1 打包元数据

- `pyproject.toml`：src-layout，`bionic_fish` 为唯一包；依赖 3 个（`numpy` / `opencv-python` / `PyYAML`），真机额外依赖拆成 extras `hardware`（pymavlink）/ `rpi`（RPi.GPIO）——**因为这两个在开发机上装不了**，不能写进必装依赖
- 版本用 `dynamic = {attr = "bionic_fish.__version__"}` 从 `__init__.py` 读，**不在这里再写一份**，两处不会漂
- `requirements.txt` 只含 `-e .` 加一段注释：依赖的唯一权威是 `pyproject.toml`。**刻意不抄版本号**——两份清单必然漂移，然后两份都不被相信
- 验证方式不是「装上去看看」，而是 `pip wheel . --no-deps --no-build-isolation`：**不污染环境**就能证明配置有效（构建出 `bionic_fish-0.1.0-py3-none-any.whl`，11 个模块齐全，`entry_points.txt` 正确）

### 4.2 入口文件

`src/bionic_fish/__main__.py`（259 行）——三种帧源（合成 / 录像 / 相机），五种打印（摘要 / 上报 / 闸门计数 / 评分 / 状态占比）。

实跑（`--lateral-mm -30`，已知能跑通的那条）：

```
frames 587   elapsed 29.30 s   final state DONE   finished yes
strip visible 0.927   lateral error mean 29.2 px  max 270.3 px
reports 8/8（第 1..8 点，t=3.35s..21.65s）
score: detection +40.0   false_report -0.0   speech +10.0   time +12.0   subtotal +62.0
states: FOLLOW 69.7%  LINE_LOST 13.1%  CORNER 7.3%  ALIGN 5.5%  FINISH_APPR 4.3%  DONE 0.2%
```

### 4.3 `--trace`：把运行顺序变成数据

`Runner(trace=True)` 逐帧记一条 `FrameTrace`（`runtime.py:47`），**字段顺序就是执行顺序**：感知 → 上报闸 → 状态机 → 控制。

```
      t state       line   e_y_px    e_th  width  conf cand rep target   yaw   fwd  note
   0.00 ALIGN        yes     23.1  -0.004   20.0  1.00    0   0   1512  1520  1500  start command received
   1.60 FOLLOW       yes     15.5  -0.082   20.0  0.95    1   0   1560  1514  1515  aligned on the strip
```

选它的理由是它能一眼回答「顺序问题」：`target` 是状态机要的，`yaw`/`fwd` 是真正发出去的，两者不同就是限斜率在起作用（上面第一行：target 1512 / fwd 1500）。

### 4.4 顺手删掉的死参数

`_iterate` 有一个 `dt_s` 参数，三个调用方全都传了，**函数体里一次都没用**（内部一律 `dt = now_s - last_now`）。合成模式下帧由 `step * dt_s` 生成，两者恰好相等，所以一直没暴露。已删。

## 五、这一轮的教训

1. **「结构」不等于「目录层数」。** 判据是「加了这一层，某条边界是不是变成不可能违反」。参考项目有 4 个子包，代价是 `import *` 把 5 个子包的全局变量混成一个命名空间——未定义全局 `pwm`、重名 `PID`、四条 MAVLink 连接都是它的后果
2. **「看不见」是文档问题，不总是代码问题。** 审阅者的困惑有一半是真的缺陷（没有入口、没有打包），另一半是这一页文档从来不存在。**先分清是哪一半，再动代码**
3. **先取证再下结论。** 第一节那次判断反转就是例子
4. **验证不需要副作用。** `pip wheel` 能证明打包配置有效，不必先装进环境

## 六、留下的口子

- `pip install -e .` **没有实际执行**（只验证了 wheel 能构建）。真正装上之后 `PYTHONPATH=src` 可以从文档里去掉
- `--camera` / `--video` 两条路径仍未在真机 / 真实录像上跑过
- `test_no_reports_before_the_start_command` **没跑循环**（只断言构造函数里计数为 0），名字比它做的事大——已记下，尚未改

---

相关：[runtime-flow](../agents/runtime-flow.md)（本轮新增的运行顺序文档）· [architecture 第八节](../agents/architecture.md)（包结构与拆分触发器）· [reference-pipeline 附录](../agents/reference-pipeline.md)（行号基准修正）
