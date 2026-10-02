# 2026-10-02 · 上真机怎么启动：查证、补齐、以及一个没定的设计问题

**触发**：审阅者问「如果把这个项目丢到实物的树莓派里，怎么启动该固件」。

先纠正一个说法：**这不是固件**。没有烧录、没有 bootloader —— 它是一个 Python 包，启动就是启动一个进程。这个措辞上的差异有实际后果：问「怎么烧录」会去找工具链，而真正的问题是「谁来启动进程、什么时候启动」。

---

## 一、先查证：现在丢到树莓派上会发生什么

查完的结论是：**跑不起来，而且最危险的是它看起来能跑。**

| # | 发现 | 证据 |
| --- | --- | --- |
| 1 | **从没解锁过** | `src/` 里 `.arm()` 出现 **0 次**（只有测试调过一次）。未解锁的 ArduSub 电机不转 |
| 2 | **从没构造过真链路** | `MavlinkLink(` 在 `src/` 里出现 0 次，只有定义 |
| 3 | **从没接过真 GPIO** | `Annunciator.for_host(` 出现 0 次 |
| 4 | **`--camera` 不碰任何硬件** | `__main__` 构造 `Runner(cfg, arena, trace=...)`，**不传 link** → 落回 `SimLink` → 只记录不发送 |
| 5 | `artefact_dir` 是空壳 | 声明在 `config.py:647`，**没有任何代码读它** |
| 6 | `mavlink_device` / `mavlink_baud` 同样是空壳 | 声明了，没人读 |
| 7 | 没有部署方式 | 没有 systemd 单元、没有 `scripts/`、没有任何启动脚本 |

第 4 条是最坏的：`--camera 0` 的语义看起来是「上真机跑」，实际是「开着相机空转」。**一个会说话的空壳比一个明显的缺失更贵** —— 它会让人在池边花掉半小时才意识到指令根本没发出去。

## 二、补的东西

### 2.1 `Runner.run_armed()`：解锁与**必上锁**

真机入口。两个刻意的决定：

- **解锁被拒 = 直接抛错，不降级继续。** 未解锁的 ArduSub 仍然接受 `RC_CHANNELS_OVERRIDE`、仍然报航向，所以程序会跑完、日志会很漂亮、鱼一动不动。安静地失败是最贵的失败方式
- **`disarm` 放在 `finally`。** 这是安全要求，不是整洁：终端状态、异常、Ctrl-C —— 任何一条退出路径漏掉解锁，飞控就保持最后一根杆位，鱼在没人看的时候继续游。比赛里这就是攒够 3 次撞壁的方式

### 2.2 `--vehicle`，并把 `--camera` 的语义写清楚

```
--camera INDEX   台架：整条链路照常跑，一条指令都不发
--vehicle        实物：MAVLink + 真 GPIO + 相机 + 解锁，跑完自动上锁
```

`--camera` 的 help 现在明说 "transmits nothing"。构造真硬件的地方**只有一处**（`_build_vehicle_io`），因为接口还在设计、后端可能还要换。

### 2.3 一个真 bug：`connect()` 没有兑现自己的承诺

`MavlinkLink.connect()` 的 docstring 写着「A missing autopilot must be a **returned failure, not an exception**」—— 但实现让驱动异常直接逃逸。在开发机上试 `--vehicle`，得到的是二十行 traceback 包着唯一有用的那句：

```
ModuleNotFoundError: No module named 'serial'
```

三处修：① `connect()` 包 try/except 并把原因存进 `last_error`；② `__main__` 打印可操作的一行加四条排查提示；③ **`pyproject.toml` 的 `hardware` extra 补上 `pyserial`** —— `pymavlink` 不依赖它，但串口端点走 `mavutil.mavserial`，它要 `serial`。这是个容易漏的依赖。

### 2.4 systemd 单元：`deploy/bionic-fish.service`

注释里写清了每个非显然设置的理由，其中两条是安全相关的：

- **`Restart=no`**：重启会重新解锁并开一趟新巡检。`Restart=always` 会把「中途崩溃」变成「鱼自己在池子里又出发一次」
- **`TimeoutStopSec=10`**：`systemctl stop` 发 SIGTERM → Python 抛 KeyboardInterrupt → 走 `finally` 解锁。给它时间走完，否则 SIGKILL 会留下一个保持着杆位的飞控 —— 正是 `finally` 想防的那件事

并**故意不写 `[Install]` 段**，即默认不开机自启。理由见下。

### 2.5 测试

新增 4 条（`test_control_reporting_fsm.py`）：解锁在循环之前、拒绝解锁时不发任何指令、**Ctrl-C 也必须解锁**、连接失败要返回而不是抛异常。这些是在没有硬件的前提下唯一能约束硬件路径的东西。

## 三、**没定**的问题：谁来启动，什么时候启动

这是查证过程中浮出来的真问题，不是代码问题：

- 规则**禁止远程操控与人工干预**，而鱼在 **10 分钟调试窗口内就已经在池子里**了
- 所以「服务启动了」**不能**等于「比赛开始了」—— 否则鱼在队伍还在标定的时候就自己出发了
- 而「裁判宣布开始后 30 秒未启动即结束」这条计时**从宣布开始算，不是从上电算**

三个候选，一个都没定：物理开关 / 固定延时 + 宣布时上电 / 入水检测。

**在定下来之前不写任何假设代码**（写了就是在赌，而赌错的表现是鱼在调试窗口里跑掉）。已写进 [operations](../agents/operations.md) 与 [backlog](../../backlog.md)。

## 四、验证到什么程度

**已实测**（用假设备）：

- `--vehicle --device /dev/ttyNOPE` → 一行可操作的报错，退出码 1，不再有 traceback
- 解锁被拒 → 抛错且**零发送**
- 循环抛 `KeyboardInterrupt` → 仍然解锁
- `--help` 正常，92 → 测试全绿

**未验证**（没有硬件，标记为 `【待验证】`）：

- 真实串口、真实飞控、真实 GPIO、真实相机 —— 四段路径全是空白
- `pyserial` 装不装得上、飞控能不能接受 `COMMAND_ACK`、GPIO 引脚号对不对

**仍然缺**：`artefact_dir`（原始帧落盘）。架构 8.1 把它列为硬要求，而配置项是个空壳。已进 backlog，并在 `config.py` 里把注释从「`None` disables frame dumping」改成「NOT IMPLEMENTED YET」——**原来的注释在说谎**。

## 五、教训

1. **「看起来能跑」比「明显不能跑」贵。** 第 4 条那个 `--camera` 空转问题，如果只是缺功能倒还好；它的问题是语义骗人
2. **docstring 承诺了但实现没做的，要当成 bug 修。** `connect()` 那句 "returned failure, not an exception" 是当初写下的正确契约，只是没人兑现
3. **依赖清单要按「实际用到的模块」核。** `pymavlink` → `pyserial` 这层间接依赖，不实际连一次串口是不会暴露的
4. **不是所有问题都该用代码回答。** 启动时机是规则问题，写代码解决它只会把风险藏起来

---

相关：[operations](../agents/operations.md)（启动与部署一节）· [hardware](../agents/hardware.md)（车体与接口）· [architecture](../agents/architecture.md)（8.2 包结构与 `Link` 注入）· [`deploy/bionic-fish.service`](../../deploy/bionic-fish.service)
