# 参考项目逆向：`pipe_inspector_raspberry`

> 入口索引与工作约定在仓库根目录 [`AGENTS.md`](../../AGENTS.md)；本文件是 `.dsh/agents/` 知识库的一部分。
> 需求依据是 [`competition.md`](competition.md)（赛题 PDF）；本文件只负责说明「参考项目是怎么做的」以及「我们哪些保留、哪些重做」。

## 这份文档的定位

- 【事实】用户把参考项目放在 [`reference/pipe_inspector_raspberry-main/`](../../reference/pipe_inspector_raspberry-main/)，并指定：**本项目的工具链应与之一致**
- 因此参考项目是**工具链依据**（语言、框架、硬件接口、通信协议），但**不是需求依据**
- 它的**算法只是起点**：要按 [competition](competition.md) 的要求重构，不能照搬（两个任务的目标、判罚、时间约束都不同）
- 参考项目自述为「已淘汰项目的存档代码」（[README](../../reference/pipe_inspector_raspberry-main/README.md)），代码里有大量真实缺陷，**缺陷清单见下文第 6 节**——这是本文最有价值的部分

---

## 1. 工具链（本项目的既定基线）

| 层 | 参考项目的选择 | 出处 |
| --- | --- | --- |
| 主控 | 树莓派（Raspberry Pi），Python | README 硬件要求 |
| 相机 | USB 摄像头 `/dev/video0`，`cv2.VideoCapture(0)` | `pipe2025_func.py:79` |
| 飞控通信 | MAVLink over 串口 `/dev/ttyACM0` @ 115200 | `pipe2025_func.py:68`、`move.py:9` |
| 控制指令 | **`RC_CHANNELS_OVERRIDE`**，直接覆盖 RC 通道 PWM（µs） | `move.py:33-36` |
| 通道映射 | ch3=上浮下潜 / ch4=左拐右拐 / ch5=前进后退 / ch1,2,6 置中 / ch9=灯 | `move.py:47-54`、`init.py:11` |
| 解锁 | `MAV_CMD_COMPONENT_ARM_DISARM` command_long | `arm_disarm.py:18-23` |
| 检测 | **ONNX Runtime** 跑 YOLOv5-Lite（`best-4.21.onnx`，2.9 MB） | `pipe2025_func.py:76`、`detectpipeline.py:12` |
| 图像处理 | OpenCV（阈值、形态学、轮廓、minAreaRect） | `visual/pipeline.py` |
| 外设 | `RPi.GPIO`：舵机 PWM 50 Hz / GPIO18，LED | `pipe2025_func.py:43-47`、`move_camera.py:19` |
| 并发 | `threading` + `queue`（LifoQueue） | `pipeline.py:40-41`、`pipe2025_func.py:126` |

- 【事实】通道映射 ch3/4/5 与 [ArduSub 的默认 RC 输入布局](https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs)一致（1=pitch, 2=roll, 3=throttle/heave, 4=yaw, 5=forward），代码注释里也直接给了该链接（`move.py:16`）
- 【推论】所以参考平台是**ArduSub + 6 自由度 ROV 布局**。**但本项目的机器鱼是不是同款、有没有垂直通道，未确认** → 见第 7 节问题 1

---

## 2. 算法总览

```
                       ┌──────────────── 每帧都跑（便宜） ────────────────┐
camera.read() ─→ resize 320×320 ─→ patrol(frame) ─→ q_center (质心 x,y)
                                          └────────→ q_width  (近场填充率)
                       └──────────────────────────────────────────────────┘
                       ┌──────── 每 3 帧才跑（贵） ───────────────────────┐
                       └→ yolov5_lite.detect() ─→ angle / cube / cube_name
                       └──────────────────────────────────────────────────┘
                                          ↓
                       主循环：读队列 → PID → aim(pwm1,pwm2,pwm3) → MAVLink
```

- 【设计】这是**双速架构**：经典 CV 全帧率负责「连续量」（管道在哪），YOLO 降频负责「离散量」（拐角、方块在不在）。在树莓派算力下这是必然选择
- 【事实】降频方式是 `count = (count + 1) % 3`，只有 `count == 0` 才调 `net.detect`（`pipe2025_func.py:170-173`）；非检测帧沿用上一帧结果（`:176-179`）
- 【事实】主循环不含任何超时或退出条件：`while 1`（`pipe2025_func.py:156`、`298`、`402`）

---

## 3. 视觉：`patrol()` 逆向

主入口 `FollowMode_GetBottomTarget(src)`（[`visual/pipeline.py:106-162`](../../reference/pipe_inspector_raspberry-main/packaged_part/visual/pipeline.py)）：

| 步骤 | 做法 | 行 |
| --- | --- | --- |
| 1. 颜色分割 | `rImg - bImg` → `cv2.threshold(..., 10, 150, BINARY)`。**用「红减蓝」找偏红的管道**在偏蓝的水里 | 108-110 |
| 2. 视野屏蔽 | 顶部 150 行清零、左右各 20 列清零 → 只看**近场中央 280×170** | 111-113 |
| 3. 形态学 | `erode(4×4, 1)` 再 `dilate(4×4, 3)` → 去碎点 + 连断口 | 117-118 |
| 4. 找质心 | `findsmallcontour`：`minAreaRect` 面积 > 2500 的候选里选一个，返回其外接框中心 `(x,y)` | 49-85 |
| 5. 横向信号 | 质心 x 存入 `q_center` | 121 |
| 6. 近场统计 | `bin_part = bin[150:300, 20:300]` → 最大轮廓 → `minAreaRect` → 排序四角 → 算**边缘斜率 `m`** | 122-154 |
| 7. 定高信号 | `means = cv2.meanStdDev(bin_part)` 的均值 ×150 → **近场窗口里管道的填充比例**，存入 `q_width` | 133、162、193 |

关键结论：

- 【事实】**横向控制量是「最近处管道轮廓的质心 x」**，归一化后为 `center = tube_center[0] / 320`（`pipe2025_func.py:194`）
- 【事实】**定高量是「近场窗口的填充比例」**，用表观宽度代替深度计做视觉伺服（`pipe2025_func.py:196`）
- 【事实】**航向信息算出来了却被丢弃**：斜率 `m` 一路随元组传递（`pipe2025_func.py:181`、`:321`、`:425`），**从未参与任何控制计算**。控制器只有位置环，没有航向环
- 【事实】**没有前瞻**：顶部 150 行被主动清零（`pipeline.py:111`），控制器纯粹反应式，只看脚下
- 【事实】`patrol()` 内部直接调 `cv2.imshow` + `cv2.waitKey(1)`（`pipeline.py:191-192`）——GUI 调用在控制环里
- 【事实】`patrol` 的注释写「进行多线程同时计算」（`pipe2025_func.py:180`），但实际是**同步调用**，没有线程

---

## 4. 视觉：YOLO 后处理里的几何门控

`postprocess`（[`detectpipeline.py:51-113`](../../reference/pipe_inspector_raspberry-main/packaged_part/YOLOv5Litemaster/detectpipeline.py)）在 NMS 之后还有一层**图像空间门控**：

```python
# 方块类：面积 > 500，中心 y ∈ (cube_num, 290)，中心 x 距画面中心 < 110
if (self.classes[classIds[i]] in list1) and area > 500 \
        and 290 > (top + height/2) > cube_num and abs(left + width/2 - 160) < 110:
# 拐角类：中心 y ∈ (angle_num, 280)，中心 x 距画面中心 < 100
elif self.classes[classIds[i]] == "angle" and 280 > (top+height/2) > angle_num \
        and abs(left + width/2 - 160) < 100:
```

- 【事实】`cube_num=50`、`angle_num=120` 由主循环传入（`pipe2025_func.py:173`）
- 【设计·值得保留】**这正是本项目「几何门控」闸门的现成实现**：目标必须落在画面中央的横向窗口内、且纵向落在有效带内，否则不算数。它把「颜色/置信度只负责出候选」落实到代码里
- 【事实】纵向带的**下沿（y=290）就是「重合」的判据**：目标中心越过 y=290 即移出有效带 → 视为「已经到达目标下方」

---

## 5. 控制层

### PID（`pid_catalog_first/sheer_pid.py:3-31`）

```python
error = pv - setPoint
output = middle + Kp*error + Ki*integral + Kd*derivative   # 手工抗饱和：直接 clamp
```

| 实例 | dt | clamp | Kp | Kd | Ki | middle | 用途 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `pipe1_fast` | 0.1 | 1480–1520 | 200 | 20 | 0 | 1495 | 浅水直行，未见拐角 |
| `pipe1_slow` | 0.1 | 1490–1510 | 20 | 5 | 0 | 1495 | 浅水直行，**已见拐角** |
| `pipe3` | 0.1 | 1463–1523 | 150 | 20 | 0 | 1499 | 深水直行 |
| `ud_pid` | 0.1 | 1390–1430 | 5 | 0 | 0 | 1410 | 定高（视觉伺服） |

- 【事实】全部 `Ki = 0` → 实际都是 **PD 控制器**
- 【事实】`dt` 硬编码 0.1 s，但真实环周期 = 相机帧率 + 每 3 帧一次 YOLO 推理，**在树莓派上大概率远大于 0.1 s** → 微分项的实际增益与设计值不符
- 【推导】`pipe1_fast`：`Kp=200`、clamp ±20 µs、误差归一化到 ±0.5 → 只要 `|error| > 0.125`（画面宽度的 12.5%）就**打满**。实际是**近似的 bang-bang 控制**
- 【推导】`pipe1_slow`：`Kp=20` → 误差 ±0.5 时输出仅 ±10 µs，**永远打不满**，非常温和。设计意图是「看到拐角时松开偏航，别和转弯抢」，但拐角恰恰是需要转向的时候
- 【推导】`ud_pid`：`Kp=5`、clamp ±20 µs、目标填充率 34/150 → 线性区只有 `mean ∈ [30, 38]`，**带宽极窄，实际是带小死区的 bang-bang**。深度主要靠飞控自身保持，这个 PID 只做慢速微调

### 输出（`move/move.py:47-54`）

```python
def aim(pwm1, pwm2, pwm3):
    set_rc_channel_pwm(3, pwm1)   # 上浮下潜
    set_rc_channel_pwm(4, pwm2)   # 左拐右拐
    set_rc_channel_pwm(5, pwm3)   # 前进后退
    set_rc_channel_pwm(6, 1500); set_rc_channel_pwm(1, 1500); set_rc_channel_pwm(2, 1500)
```

- 【事实】`set_rc_channel_pwm` 用 `rc_channels_override_send`，未使用通道填 `65535`（MAVLink 语义 = 不覆盖），每次只发一条消息（`move.py:31-36`）
- 【事实】**前进速度是开环常数**，只按「有没有检测到目标/拐角」切换三档：1500 附近 1490 / 1550 / 1590（`pipe2025_func.py:149-151`、`:297-299`）

---

## 6. 参考项目的缺陷清单（重构时必须修掉）

按严重程度排序。每条都给了位置，便于复核。

| # | 缺陷 | 位置 | 后果 |
| --- | --- | --- | --- |
| 1 | **`q_center.get()` 死锁**：它默认阻塞，而 `patrol`（唯一 put 来源）在 `if ret:` 内部 | `pipe2025_func.py:181` vs `:187` | 相机读失败一帧 → 程序**永久卡死** |
| 2 | **状态用 23 元组在函数间传递** | `pipe2025_func.py:261`、`:266`、`:459-464` | 新增一个状态要改 5 处签名；**23 个值里有 6 个从未被读过**：`time_start`、`time_over`、`flag`、`m`、`angle_center`、`angle_last_center` |
| 3 | **转弯是开环定时**：`sleep(1)` + `sleep(2)` 配固定 PWM | `pipe2025_func.py:268-273`、`:385-390` | 无法适应电池电压、水流、负载变化。本项目浅水窄池下会直接撞壁（触壁 3 次即本轮结束） |
| 4 | **没有任何终止/超时保护** | `while 1`，`pipe2025_func.py:156`、`298`、`402` | 本项目有 4 条硬终止条件（10 s 脱离 / 触壁 3 次 / 30 s 未启动 / 150 s），**一条都没实现** |
| 5 | **动态目标选择 bug**：`dis_center` 先初始化为 320，随后被赋成 `center_test[0]`（质心 x），**而不是到画面中心的距离** | `pipeline.py:51`、`:66-68` | 实际选的是**最靠左**的大轮廓，不是最靠中间的。管道偏右时必然选错 |
| 6 | **`top_num` 永不更新**：`if top > top_num:` 里没写 `top_num = top` | `detectpipeline.py:104-105` | `cube_name` 取的是循环里最后一个候选，不是最远/最近的那个；灯色与后续动作可能对应错目标 |
| 7 | **线程参数传错**：`threading.Thread(target=..., args=cube_name_last)`，`args` 收到的是**字符串**（会按字符拆开），而目标函数要一个参数 | `pipe2025_func.py:133-134`、`:113` | 线程一启动就 `TypeError`；亮灯静默失效 |
| 8 | **解锁不校验回执**：`arm()` 把 `motors_armed_wait()` 注释掉，无条件打印 `Armed!` | `arm_disarm.py:27-28` | 解锁失败也照常往下跑，空中解体式失败 |
| 9 | **`patrol` 的输入帧不一致**：`_pipe1` 传 YOLO 画过框的 `frame2`，`_pipe2`/`_pipe3` 传原始 `frame` | `pipe2025_func.py:181` vs `:321`、`:425` | 红框/绿字会污染颜色分割阈值，三个阶段的视觉特性不一致 |
| 10 | **`patrol` 调用位置不一致**：`_pipe3` 里在 `if ret:` 之外 | `pipe2025_func.py:425` | 读帧失败时用上一帧的陈旧画面继续控制 |
| 11 | **`cv2.imshow` 在控制环内** | `pipeline.py:191-192` | 无显示器的树莓派上抛异常；有显示器时拖慢环周期 |
| 12 | **四条独立 MAVLink 连接**并存（`move.py:9` 模块级、`arm_disarm.py:9`、`arm_disarm.py:32`、`pipe2025_func.py:68`） | 同上 | 串口资源与心跳混乱；`target_system` 可能在 `wait_heartbeat` 之前被读取；`_begin_init` 开的那条连上后**从未被使用** |
| 13 | **硬编码魔数**：`1495/1499/1410/1650/1590/1550/34/40/2500/1000/110/100/120/50/0.25/0.3`，以及绝对路径 `/home/pi/pipe_new` | 全项目 | 换一台机器、换一个场地就要改代码；无法标定 |
| 14 | **`from sympy import false`**，随后与 Python `True` 混用 | `pipe2025_func.py:3`、`:154`、`:234` | 为一个小写常量引入 sympy 重依赖；真假语义混乱 |
| 15 | **`LED._light_blink` 里 `args` 与 `target` 不匹配**（同 7）；`_target_catch_thing` 先 Close 再 Open | `pipe2025_func.py:106-109` | 抓取动作语义可疑（关-开等于没抓） |
| 16 | **`findsmallcontour` 自我修改掩膜**：`frame_copy` 与 `mask` 是同一对象 | `pipeline.py:119` | 调试绘制写进了二值掩膜 |
| 17 | **无日志结构**：靠 `sys.stdout` 重定向到 `dayYYYY_MM_DD.log` | `pipeline.py:6-38` | 每个决策没有结构化记录，事后无法回放「为什么这次上报了」 |

### 6.1 复核补充（2026-10-02）

对 `pipe2025_func.py`、`move/move.py`、`camera_gimbal_control/LED.py`、`pid_catalog_first/*` 逐行复核后新增的缺陷。**行号以 2026-10-02 的代码为准**（该文件的 19 元组在第 141 行被拆成 141–146 行，本文上面那些 ≥ 142 的引用已随之 +5）。

| # | 缺陷 | 位置 | 后果 |
| --- | --- | --- | --- |
| 18 | **LED 引脚与代码不符**：模块 setup 的是 16 / 17 / 6，而 `green_on` / `blue_on` 实际写的是 **27 / 22**；`GPIO.setup(16, GPIO.HIGH)` 的第二参用了 `HIGH`（在 RPi.GPIO 里 `HIGH`=1=`IN`） | `LED.py:4-10`、`:18-28` | 绿、蓝灯首次输出抛 `RuntimeError`【推论】，只有红、紫可用；注释（`#RED IS 16` 等）与代码也全不对应 |
| 19 | **夹爪线程被塞进 `light_control_thread`**，`catch_control_thread` 声明后从未使用 | `pipe2025_func.py:17`、`:127` | 变量名与内容不符，排查时误导 |
| 20 | **`aim()` 每帧发 6 条消息**，未用通道填 `65535`（语义 = 不覆盖） | `move.py:31-36`、`:47-54` | 下一帧就把 `init()` 设的 `ch9=1100` **释放**掉【推论】，「灯通道」形同虚设 |
| 21 | **`count` 一变量两用**：帧分频 `(count+1)%3` 与转弯去抖 `count += 1` | `pipe2025_func.py:170`、`:235` | 「连续 3 帧」的语义不严格，实际行为取决于重置相位 |
| 22 | **两个阶段的转弯触发不一致**：`_pipe1` 有 3 帧去抖，`_pipe2` 下降沿当帧即转 | `:233` vs `:361` | 一次单帧误检就能触发转弯 |
| 23 | **PID 的 `dt` 硬编码 0.1**，而真实环周期是它的数倍 | `sheer_pid.py:33-42`、`set_deepth.py:30` | `dt` 恒定时 D 项退化成「第二个 P 项」，实际增益与设计值不符 |
| 24 | **斜率 `m` 与拐角坐标 `angle_center` 算了不用** | `pipeline.py:151`、`detectpipeline.py:109` | 控制器只有位置环、没有航向环 |
| 25 | **`m` 的零分母保护判 `box`、除法却用 `rect`**（两组不同的点） | `pipeline.py:148-151` | 保护失效；`float32` 除零得到 `inf/nan` 而非异常【推论】 |
| 26 | **`patrol(img, False)` 会抛 `UnboundLocalError`**（`m`/`means` 未赋值） | `pipeline.py:188-194` | 目前恒传 `True`，属隐患 |
| 27 | **`q_width` 的 put/get 不平衡**：中心为 `None` 时只 put 不 get | `pipe2025_func.py:192` | LIFO 下取值仍新鲜，但队列会无限增长 |
| 28 | **模型路径是相对 CWD，与 README 的启动方式冲突** | `:76` vs `README.md:69` | 按 README 的 `cd main_process` 启动则模型加载失败；改用项目根启动则日志落到项目外（`:66` 的 `path='../'`）。两种启动方式必有一种错位 |
| 29 | **模块级 `set_servo_angle` / `_claw_close` / `_claw_open` 引用了未定义的全局 `pwm`** | `:21-37` | 一旦调用即 `NameError`；好在主流程未调用 |
| 30 | **两个 `PID` 类同名且连续 `import *`** | `:9-10` | 模块命名空间里的 `PID` 指向 `sheer_pid.PID`；已有实例不受影响，但极易埋坑 |
| 31 | **`_turn_first` 与 `_turn_second` 逐字重复** | `:265-279`、`:380-395` | 只差状态名与末尾一句 `aim`，改一处必须记得改两处 |

**两处正文修正**（已在上表对应行内就地改正）：

1. 原第 2 条写「`time_start`/`time_over` 未定义就被打包进返回值」——**不成立**。`_pipe1` 在 `:141-146` 从 `_begin_init` 的 19 元组里解出了它们（值为 0），不会 `NameError`；它们只是**死变量**
2. 原第 12 条写「三条 MAVLink 连接」——实际是**四条**（漏了 `_begin_init` 自己开的那条）

---

## 7. 参考算法 → 本项目需求 的差异

| 维度 | 参考项目 | 本项目（[competition](competition.md)） |
| --- | --- | --- |
| 跟踪目标 | 池底红管（红减蓝分割） | **LED 灯带**（自发光，颜色待确认 → 问题 5） |
| 跟踪信号 | 最近处轮廓质心 x | 灯带**中心线**横向偏差 + **航向偏差**（`m` 必须真正用起来） |
| 前瞻 | 无，顶部 150 行清零 | **需要**：6 m 路径 + 2 个 90° 弯角，纯反应式跟不上 |
| 定高 | 管道表观填充率恒定（目标 34/150） | 相机对池底距离恒定（可用灯带宽度/标记像素尺寸，水深仅 260±20 mm） |
| 离散目标 | YOLO 5 类：angle / cube / triangle / circular / circle | **⌀3 cm 黑圆**：8 个真漏油点 + 5 个**误识别点**（外观未知） |
| 目标动作 | 检测消失（y>290）→ 亮灯 + 抓取 | **重合时立即上报**：声光 + 无线回传 + 语音播报（10 分项） |
| 动作去重 | 无 | **必须有**：误报 `−3`、正确 `+5`，判决阈值 `p > 3/8` |
| 误报概念 | 无 | **核心风险**：误识别点 + 起终点黑带都是必然误报源 |
| 转弯 | 开环定时 2 s | 必须闭环 + 时限保护（触壁 3 次即结束） |
| 终止条件 | 无 | 4 条硬终止必须实现 |
| 时间预算 | 无 | 150 s 上限，时间分 7 档 |
| 状态传递 | 23 元组 | 状态对象 |
| 参数 | 硬编码魔数 | 标定产物 + 版本化配置 |
| 评分意识 | 无 | 有（[strategy](strategy.md)） |

---

## 8. 保留 / 重做 清单

**保留（参考项目已验证在树莓派上可行）**

1. 双速架构：经典 CV 全帧率 + 检测网络降频（本项目可先只上经典 CV）
2. 处理分辨率 320×320 —— 匹配 Pi 算力，且 ⌀3 cm 在标定后仍占足够像素（见 [hardware](hardware.md) 的分辨率推算）
3. **几何门控**：横向窗口 + 纵向有效带（对应本项目 [reporting](reporting.md) 闸 1）
4. **「重合」用纵向行阈值判定**（y > 290 的思想）—— 比用里程预测更简单，且不依赖里程计
5. **消失沿（falling edge）触发**动作，但要加防重复上报
6. MAVLink `RC_CHANNELS_OVERRIDE` + ArduSub 通道布局
7. PID 输出形式 `middle + PD` 后 clamp（限幅即抗饱和）
8. 视觉伺服定高（用表观尺寸代替深度计）
9. 队列传递视觉结果、非检测帧沿用上一帧结果
10. 目标丢失时用上一个有效值继续（`tube_center_last`）→ 对应本项目丢线恢复

**重做**

1. 状态机对象化，替换 23 元组与 5 处签名
2. 弯角：开环定时 → 闭环 + 时限 + 丢失保护
3. 补齐 4 条硬终止条件与可视化余量
4. 上报：三道闸 + 轨迹身份去重 + `p > 3/8` 判决 + 声光/语音共用一个事件
5. 误识别点与起终点黑带的排除规则
6. `dt` 用实测环周期，不再硬编码 0.1
7. 参数与标定外置、版本化
8. 结构化日志：每个上报决策的三道闸逐条可回放
9. 修掉第 6 节全部 17 条缺陷
10. 控制加航向环（把参考项目丢弃的斜率 `m` 用起来）

---

## 9. 不明数据（需向用户确认）

| # | 问题 | 为什么阻塞 |
| --- | --- | --- |
| 1 | **本项目机器鱼有几个推进通道？** 参考是 ROV 三通道（ch3 垂推 / ch4 偏航 / ch5 前进）；尾鳍仿生鱼通常只有「尾摆频率 + 偏转」两个 | **已定（2026-10-02）**：**单关节尾鳍** ⇒ 两个标量（转向偏置 + 推力/摆频），**没有深度轴**；`ChannelMap.heave` 已置 `None`。见 [hardware](hardware.md) |
| 2 | 飞控型号与连接方式是否与参考一致（`/dev/ttyACM0` @115200 + ArduSub RC override）？ | **已定（2026-10-02）**：接口由**我们自己设计**，不沿用参考的飞控假设。若最终由树莓派直驱舵机，`platform` 需新增一个后端（`Link` 协议已为此准备） |
| 3 | 相机型号 / 分辨率 / 安装朝向与高度？ | **部分已定**：相机在**水下**（因此无折射问题）；型号 / 分辨率 / 视场角 / 高度仍未知，且高度要量到**管顶**而不是池底 |
| 4 | **误识别点外观**（与 ⌀3 cm 黑圆靠什么区分） | [vision](vision.md) 的核心未知项，未变 |
| 5 | **LED 灯带是什么颜色/亮度？** 参考用「红减蓝」找偏红管道；若灯带发蓝光，该特征在湖蓝池底上会失效 | **已回答（2026-10-02）**：实物**不是 LED 灯带**，是**白色圆柱管道**（被动反光，不是自发光）。所以「红减蓝」不适用；分割特征必须在真实白管上重选，见 [vision](vision.md) |
| 6 | 灯带实物宽度（mm）？ | **部分已定**：它是**管道直径**；数值仍未知。像素标定与宽度先验都要用它 |
| 7 | **是否复用 `best-4.21.onnx`？** 它的 5 类（angle/cube/triangle/circular/circle）与本项目目标**完全不对口** | 决定要不要重新标注+训练，还是纯经典 CV |
| 8 | 有没有任何真实水下图像/视频可用于验证？（开发机无 `/dev/video0`，reference 内也无图片） | 决定能否离线验证重构结果 |
| 9 | **摄像头的安装俯仰角**：画面里哪一行对应「鱼的正下方」？参考项目把动作触发线写死在 y=290，而「正下方」在图像中的位置是**安装角度的函数** | 决定「重合」的行阈值（[architecture](architecture.md) 第三节的耦合 2 与耦合 7）。【推算】相机垂直朝下时正下方是 y=160；前倾 θ 时该行下移，约 `160 × (1 + θ/(VFOV/2))`，θ ≈ 0.81 × VFOV/2 时才落在 y≈290（VFOV=60° 时 θ≈24°，是常见安装角）。**参考项目把它写死成魔数，本项目必须实测标定** |

---

## 变更记录

| 日期 | 变更 |
| --- | --- |
| 2026-09-25 | 首次建立：完整逆向参考项目工具链、双速视觉、`patrol()`、PID、控制输出、状态机；列出 17 条缺陷、保留/重做清单、8 项待确认数据 |
| 2026-10-02 | 6.1 复核补充：新增 14 条缺陷（LED 引脚、`aim()` 释放 ch9、`count` 两用、PID 的 `dt`、模型路径与 CWD 冲突等）与 2 条正文修正；行号随 `pipe2025_func.py` 重新格式化整体 +5；新增待确认项「摄像头俯仰角 ↔ 正下方行」 |
