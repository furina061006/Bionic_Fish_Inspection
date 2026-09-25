# 系统架构、数据流与状态机

> 入口索引与工作约定在仓库根目录 [`AGENTS.md`](../../AGENTS.md)；本文件是 `.dsh/agents/` 知识库的一部分。
> 场地与规则见 [`competition.md`](competition.md)；硬件输入输出见 [`hardware.md`](hardware.md)。

**本文件状态：全文 `【设计】`**——仓库当前没有代码，这是已决定的分层与接口约定。改动接口要先改本文件，再改代码。

## 分层

```
感知层  vision        图像 → 灯带中心线观测 + 标记候选 + 起终点线观测
   ↓
融合层  estimator     观测 + 姿态 + 里程 → 沿路径里程 s、横向偏差 e_y、航向偏差 e_θ
   ↓
决策层  fsm           状态机：在哪一段、该快还是该慢、该不该上报
   ↓        └→ reporting   上报三道闸 + 声光 / 语音触发
控制层  control       目标 (e_y, e_θ, v) → 转向 / 推力指令
   ↓
平台接口              steer / thrust → 执行器
```

**为什么这样切**：`vision` 只做「这一帧看到了什么」，不做「该不该转弯」；`fsm` 只做「现在是什么状态、目标是什么」，不做像素运算；`control` 只做「给定目标怎么打舵」。任何一层越界都会让排查时分不清是「看错了」还是「判错了」还是「控不住」——见 [troubleshooting](troubleshooting.md)。

## 数据流

```
相机帧 (frame, ts)
  ↓ vision.enhance()          水下增强，输出可视化用的增强帧（仅调试，不喂下游）
  ↓ vision.line()             灯带分割 → 骨架 / 中心线 → 拟合
  │     LineObs { valid, e_y, e_theta, curvature, confidence, ts }
  ↓ vision.markers()          暗斑检测 → 候选
  │     MarkerCand { u, v, px_area, polarity, features, ts }
  ↓ vision.gate()             起终点黑色胶带 → 横穿灯带的暗带
  │     CrossObs { detected, distance_ahead, ts }
  ↓ estimator.update()        结合 attitude / odom
  │     Pose { s, e_y, e_theta, v, ts }
  ↓ fsm.step()                状态迁移 + 速度档位目标
  │     Command { state, v_target, report_req }
  ├→ reporting.decide()       对每个 MarkerCand 走三道闸
  │     ReportEvent { id, s_at_report, ts }   → 声光 + 语音（共用同一事件）
  ↓ control.step()            Pose + Command → 指令
        Actuator { steer, thrust }
```

## 接口约定

| 名称 | 生产者 → 消费者 | 字段 | 约定 |
| --- | --- | --- | --- |
| `LineObs` | vision → estimator | `valid: bool`、`e_y: float`（灯带中心相对图像中心的横向偏差，**需定义单位**）、`e_theta: float`（灯带方向相对机体前向的夹角）、`curvature`、`confidence`、`ts` | `valid=false` 时其余字段无意义；**不许用 0 冒充「没看到」** |
| `MarkerCand` | vision → reporting | `u, v`、`px_area`、`polarity`、`features`、`ts` | `features` 是「能用来区分漏油点与误识别点」的特征集合，内容随标定结果变（[vision](vision.md)） |
| `CrossObs` | vision → fsm | `detected`、`distance_ahead` | 起终点黑带；`distance_ahead` 用于决定何时开始制动 |
| `Pose` | estimator → fsm/control | `s`（沿路径里程）、`e_y`、`e_theta`、`v` | `s` 是上报去重的唯一键，必须是单调增的（[不变量 4](invariants.md)） |
| `ReportEvent` | reporting → 声光 / 语音 | `id`、`s_at_report`、`ts` | **声光与语音必须订阅同一个事件**，不许各自判条件（[不变量 5](invariants.md)） |
| `Actuator` | control → 平台 | `steer`、`thrust`，归一化 | 正负方向与单位待与平台对齐（[hardware](hardware.md)） |

【待验证】全部字段的**单位、坐标系、正负号**尚未与平台方确认。在确认前先在本文件补齐表格，再写代码。

## 状态机

```
        ┌──────────┐
        │   IDLE   │  等待「比赛开始」，电机禁用
        └────┬─────┘
             ▼
        ┌──────────┐
        │  ALIGN   │  起点区内对齐灯带：原地微调航向直到 LineObs.valid 且 |e_theta| 小
        └────┬─────┘
             ▼
   ┌─────────────────┐   持续丢线 ──→ ┌──────────────┐
   │     FOLLOW      │◀─────────────│  LINE_LOST   │
   │ 沿灯带巡线，跑  │──────────────▶│ 分级搜索     │
   │ 漏油点检测      │   重新捕获     └──────┬───────┘
   └───┬─────────┬───┘                      │ 超过 10s（硬终止）
       │         │                          ▼
  弯角 │         │ 检测到起终点黑带    ┌──────────┐
       ▼         ▼                     │  ABORT   │ 停机上电保持
  ┌────────┐ ┌────────────┐            └──────────┘
  │ CORNER │ │ FINISH_APPR│
  │ 减速   │ │ 制动 + 全身 │
  │ 大转向 │ │ 进入终点区  │
  └───┬────┘ └─────┬──────┘
      │ 重捕直线    ▼
      └──▶ FOLLOW  ┌──────────┐
                  │   DONE   │  停机，等待裁判
                  └──────────┘
```

- 【设计】`ALIGN` 单独成状态而不是塞进 `FOLLOW`：起点区内空间只有约 500mm，初始航向误差大，用巡线控制律去纠会直接撞分界线
- 【设计】`CORNER` 的判据是 `LineObs.curvature` 超阈值或中心线在图像中呈现拐折；不靠「跑了多少米」的里程猜测
- 【设计】`FINISH_APPR` 要满足「**全身进入**终点区」：看到黑带后不是立刻停，而是按已知的鱼体长度继续前进一段再停。**具体距离待实测鱼体长度后填入**
- 【设计】`ABORT` 覆盖所有硬终止条件的自查：丢线 > 10 秒、触壁计数达 3（若平台提供碰撞信号）、收到停止指令
- 【待验证】`LINE_LOST` 的分级搜索动作（保持 → 减速直行 → 扫掠）在浅水窄池里是否安全，见 [control](control.md)

## 时序与算力预算

- 【待验证】控制环频率：目标 **≥20 Hz**（欠驱动 + 大滞后，环率低于此无法稳定）
- 【待验证】视觉频率：灯带提取目标 ≥30 Hz；标记检测可降频（例如 10 Hz）以省算力，但**上报延迟会增加**，需与速度档位一起评估
- 【设计】相机采集与视觉处理若在同一进程，必须把「最新帧覆盖」而非「排队」，否则延迟会随算力抖动累积
- 【待验证】端到端延迟（曝光 → 执行器动作）是控制参数整定的前提，必须实测，见 [operations](operations.md)

## 进程与代码组织（待定）

- 【事实】赛题 PDF 对运行环境**没有任何规定**——操作系统、语言、中间件、是否用 ROS、进程边界、算力放在鱼上还是池边，全部自由
- 【推论】但 PDF 要求「搭载……自主控制系统」「检测模块需集成于机器鱼本体」「禁止远程操控」，因此**默认按「全部算力在鱼上」设计**，板外算力只当不可用的优化手段（见 [operations](operations.md) 的推论）
- 【待验证】在上述两条答案落地前，目录结构、消息中间件、日志方案一律不定；**不要因为开发机上装了什么就倒推部署架构**
- 【设计】无论选哪种，都必须满足两条：**原始帧可落盘**（排查的唯一证据）、**每个上报决策可回放**（三个闸各自的输入输出都要能打出来）

---

## 变更记录

| 日期 | 变更 |
| --- | --- |
| 2026-09-25 | 首次建立：分层、数据流、接口表、状态机、时序预算 |
