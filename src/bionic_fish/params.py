"""全部可调参数，就在这一个文件里。

默认值**照参考项目**（`reference/pipe_inspector_raspberry-main/`），现场再慢慢调。
想改而不动代码，就写一个 YAML：

    load()                     # 全用下面的默认值
    load("tank.yaml")          # 只覆写文件里出现的键

YAML 是**平的**，键名就是下面的字段名：

    cruise: 1590
    report_row: 290
    ch_yaw: 4

字段上标 `# 【待验证·实机】` 的是**值必须来自实测或真机联调**的参数：默认值照参考
项目，没有在任何实物上量过。它们一次性列出来靠的就是这个标记：

    grep -n "【待验证·实机】" src/bionic_fish/params.py

标定 / 联调完成之后，把那一行的标记改成 `【实测】` 并写清依据（标记约定见
`.dsh/agents/writing.md`）。**没有这个标记的字段只有两种**：赛题硬性数字（150 秒上限、
8 个漏油点、⌀30mm 标记），以及算法自身的选择（由测试与现场调试定，不随硬件改变）。

为什么要一个 `Params` 对象而不是散落各处的模块常量：仿真壳与实机壳要能各带一套
参数跑同一个 `main()`（见 `sim/harness.py`），而且测试要能造一个改两个字段的副本
（`dataclasses.replace`）。
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Params:
    # ======================= 通道与 PWM（照参考项目 move.py）=======================
    # 【待验证·实机】通道映射：平台未定，用 send_channel 静态验通道
    ch_yaw: int = 4
    # 【待验证·实机】通道映射：平台未定，用 send_channel 静态验通道
    ch_forward: int = 5
    # 【待验证·实机】中位 PWM：按实际执行器的停转点实测
    neutral: int = 1500
    # 【待验证·实机】量程下限：按实际执行器实测
    pwm_min: int = 1100
    # 【待验证·实机】量程上限：按实际执行器实测
    pwm_max: int = 1900
    #: 参考项目 `_pipe1` 的巡航值。
    # 【待验证·实机】巡航 PWM：要实测「前进量 → 速度」的关系
    cruise: int = 1590
    #: 转弯时的前进量。参考项目在 `_turn_first` 前把速度压到中位附近；
    #: 压得太低会**转不动**（欠驱动：转向权限正比于前进速度）。
    # 【待验证·实机】弯角速度档：受实测转向权限约束（几乎停住就转不动）
    turn_slow: int = 1525
    #: 丢线后的降级速度。参考项目 `_pipe1` 的丢线档。
    # 【待验证·实机】丢线速度档：受实测转向权限约束
    lost_slow: int = 1550
    #: `|forward - neutral|` 超过这个值就算「鱼动了」（30 秒未启动判据）。
    #:
    #: **必须远小于最小速度档与中位之差。** 丢线档是 `lost_slow - neutral = 50`，
    #: 若这里取 60，一条丢线后正在游动的鱼会被判成「从未启动」，30 秒后被
    #: 「未启动」误终止 —— 而且日志会指向错误的方向。取 20，宁可判成「动了」。
    # 【待验证·实机】起步门限：随实测速度档重算，须远小于最小档与中位之差
    started_forward_us: int = 20

    # ============================ 巡线控制（照参考项目 sheer_pid.py）============================
    #: 外环增益，1.0 = 几何上正确的纯追踪修正量。
    k_lateral: float = 1.0
    #: 期望航向的上限，弧度。
    max_heading_cmd_rad: float = 0.6
    # 【待验证·实机】内环比例增益：先量 steer → 航向角速率的阶跃响应
    kp: float = 200.0
    # 【待验证·实机】内环微分增益：同上，按实测死时间与时间常数定
    kd: float = 20.0
    #: 舵量相对中位的上限，微秒。
    # 【待验证·实机】舵量上限：受实测转向权限约束
    clamp_us: float = 20.0
    #: 前进量的斜率限制，微秒/秒。防止指令阶跃。
    # 【待验证·实机】前进量斜率：按实测推力响应重定
    forward_slew_us_per_s: float = 600.0

    # ============================== 转弯 ==============================
    #: 转弯时的固定舵量（相对中位），微秒。
    # 【待验证·实机】弯角舵量：必须实测最小转弯半径与转向角速率
    corner_turn_us: float = 20.0
    #: 弯角判据一：行扫描拟合出的曲率超过这个值（弧度/行）。
    # 【待验证·实机】弯角曲率阈值：需真实弯角图像才能定
    corner_curve_max: float = 0.02
    #: 弯角判据二：可用行占比低于这个值。白管在画面里变成 L 形时它会塌下来。
    # 【待验证·实机】可用行占比阈值：需真实弯角图像才能定
    corner_confidence_min: float = 0.55
    #: 起转到允许「重新看见管道就结束」之间的最短时间。
    #: 参考项目没有这一条，代价是实测**只转了 4.4°** 就宣布弯角结束、径直冲过拐角。
    # 【待验证·实机】最短转弯时间：随实测转向角速率定
    corner_min_commit_s: float = 1.0
    #: 转弯的硬上限；超了就交给丢线恢复。
    # 【待验证·实机】转弯时限：随实测转向角速率定
    corner_timeout_s: float = 4.0
    #: 刚转完的不应期：灯带横向出画这个判据在转完后仍成立，没有不应期会立刻
    #: 识别出第二个弯、再转 90°。
    corner_refractory_s: float = 1.5

    # ==================== 四条硬终止（赛题 PDF 的事实，不许改小）====================
    #: 完全脱离路线超过这个秒数 → 本轮终止。
    lost_timeout_s: float = 10.0
    #: 触底 / 触壁满这个次数 → 本轮结束。
    wall_hit_limit: int = 3
    #: 撞壁没有传感器时的推断：横向偏差持续超过这个比例…
    # 【待验证·实机】撞壁推断阈值：平台没有碰撞信号时才用，需实测
    wall_hit_error_frac: float = 0.75
    #: …持续这么久算**一次**接触。一次持续饱和只算一次。
    # 【待验证·实机】撞壁推断时长：同上
    wall_hit_infer_s: float = 1.5
    #: 裁判宣布开始后这个秒数内没动 → 结束。
    start_timeout_s: float = 30.0
    #: 总时长上限；超过则计时分 0。
    run_timeout_s: float = 150.0

    # ============================== 上报 ==============================
    #: 图像行号，候选经过这一行就上报（「已到鱼的正下方」）。参考项目写死 y=290。
    # 【待验证·实机】「重合」行号：随相机安装俯仰变，必须现场重标
    report_row: float = 290.0
    #: 检测项满分对应的上报数；到顶后再报没有上行收益、风险仍是 −3。
    max_reports: int = 8
    #: 起点黑带长得和终点一样，所以要跑够这么久之后才认终点。
    finish_min_run_s: float = 20.0
    #: 认了终点之后继续往前挪，让整条鱼压过黑带。
    # 【待验证·实机】终点续行时长：按实测鱼体长度 + 制动距离定
    finish_creep_s: float = 1.5

    # ========================= 相机与像素标定 =========================
    camera_index: int = 0
    # 【待验证·实机】帧宽：必须等于相机真实输出（当前代码不做 resize）
    frame_w: int = 320
    # 【待验证·实机】帧高：必须等于相机真实输出（当前代码不做 resize）
    frame_h: int = 320
    #: 镜头到**管顶**的距离（不是池底——标记贴在管顶，管顶近一个管径）。
    # 【待验证·实机】镜头到管顶的距离：实物测量（不是池底）
    camera_height_m: float = 0.20
    # 【待验证·实机】水平视场角：按镜头型号或实测
    hfov_deg: float = 90.0
    #: 实测优先：量一个 ⌀30mm 的圆在画面里占几个像素，填这里，绕过上面的模型。
    # 【待验证·实机】像素尺度：拍 ⌀30mm 实物量像素直径，优先用它
    mm_per_px_override: float | None = None
    #: 白管的**直径**（俯视看圆柱，轮廓宽度就是直径）。
    # 【待验证·实机】管道直径：卡尺实测
    strip_width_mm: float = 24.0
    #: 漏油点 / 误识别点的直径（赛题事实：3cm）。
    marker_diameter_mm: float = 30.0

    # ========================= 白管分割 =========================
    #: "tophat"（亮局部结构，默认）/ "bright_low_sat" / "red_minus_blue"。
    # 【待验证·实机】分割方法：必须用真实白管照片在三种方法里重选
    strip_method: str = "tophat"
    #: top-hat 核必须**大于**管道像素宽度，否则管道内部会被自己抹掉。
    # 【待验证·实机】核尺寸：必须大于管道的像素宽度
    tophat_ksize: int = 31
    # 【待验证·实机】亮局部结构阈值：按真实水下光照标定
    tophat_threshold: int = 35
    # 【待验证·实机】亮度门限：仅 bright_low_sat 分支使用，需真实图像
    v_min: int = 200
    # 【待验证·实机】饱和度门限：仅 bright_low_sat 分支使用，需真实图像
    s_max: int = 100
    # 【待验证·实机】红减蓝门限：仅 red_minus_blue 分支使用，需真实图像
    r_minus_b_min: int = 10
    #: 搜索区，画面比例。上面留一点，前瞻点才有地方放。
    roi_top_frac: float = 0.05
    roi_bottom_frac: float = 0.98
    roi_side_frac: float = 0.03
    open_ksize: int = 3
    #: 闭运算核；0 = 自动取一个能跨过漏油点在管道上打的洞的尺寸。
    close_ksize: int = 0
    #: 可接受的游程宽度，按管道标定宽度的倍数。窗口窄才挡得住镜面高光带
    #: （它比管壁更亮、更窄）。
    # 【待验证·实机】游程窗口下界：挡管顶高光带的关键，按真实管径标定
    run_ratio_lo: float = 0.7
    # 【待验证·实机】游程窗口上界：同上
    run_ratio_hi: float = 1.6
    #: 行间连续性：与上一行中心的跳变超过这个像素数就丢弃。
    continuity_window_px: float = 40.0
    #: 拟合成立所需的最少行数。
    min_rows: int = 12
    #: 前瞻点所在的行（画面比例，越小越远）。
    lookahead_frac: float = 0.22
    #: 鲁棒拟合的残差上限，像素。
    fit_residual_px: float = 8.0

    # ========================= 漏油点检测 =========================
    # 【待验证·实机】核尺寸：须不小于标记的像素直径
    blackhat_ksize: int = 31
    # 【待验证·实机】相对暗度阈值：用真实物料标定
    blackhat_threshold: int = 40
    #: 候选中心离管道中心线的最远距离，画面宽度比例。
    # 【待验证·实机】横向门控：按「标记在路线上」的先验实测
    max_lateral_offset_frac: float = 0.25
    #: 面积窗口，按标定直径对应的面积倍数。
    # 【待验证·实机】面积窗口下界：随像素尺度重标
    area_ratio_lo: float = 0.35
    # 【待验证·实机】面积窗口上界：随像素尺度重标
    area_ratio_hi: float = 3.0
    # 【待验证·实机】圆度：0.55 是按平面推的，标记贴在圆柱面上必须重测
    circularity_min: float = 0.55
    aspect_max: float = 1.8
    #: 「有多黑」的上限（OpenCV HSV 的 V，0-255）。漏油点是**纯黑**，误识别点未知，
    #: 所以默认全收（255）；标定出两者的值以后再往下压。
    # 【待验证·实机】第一风险：纯黑 vs 误识别点，必须用真实物料标定（默认全收）
    dot_val_max: float = 255.0

    # ========================= 起终点黑带 =========================
    #: 灰度不超过这个值算黑带。用绝对暗度而不是黑帽响应：黑帽在亮结构的**边缘**
    #: 也会响应，会把管道自己的边界报成一条黑带。
    # 【待验证·实机】黑带灰度门限：按真实胶带与现场光照标定
    tape_max_grey: int = 90
    # 【待验证·实机】黑带宽度下界：随像素尺度重标
    tape_width_px_lo: float = 60.0
    # 【待验证·实机】黑带宽度上界：随像素尺度重标
    tape_width_px_hi: float = 400.0
    #: 黑带沿行进方向的厚度上限。池壁外的暗区会远超这个值。
    # 【待验证·实机】黑带厚度上限：随像素尺度重标
    tape_thickness_px_max: float = 40.0

    # ========================= 声光与播报 =========================
    # 【待验证·实机】GPIO 引脚：按实际接线
    light_pin: int = 16
    # 【待验证·实机】GPIO 引脚：按实际接线
    buzzer_pin: int = 17
    # 【待验证·实机】亮灯时长：须按「录像里能辨别」现场验证
    light_hold_s: float = 0.6
    # 【待验证·实机】蜂鸣时长：须按「录像里能辨别」现场验证
    buzzer_hold_s: float = 0.25
    speech_enabled: bool = True

    # ========================= 飞控连接 =========================
    #: 参考项目用 `/dev/ttyACM0` @115200。
    # 【待验证·实机】串口设备节点：按实际飞控
    mavlink_device: str = "/dev/ttyACM0"
    # 【待验证·实机】波特率：按实际飞控
    mavlink_baud: int = 115200
    #: 解锁要等 `COMMAND_ACK` 的秒数。参考项目把这一步注释掉、无条件打印 Armed!。
    # 【待验证·实机】等 ACK 的时长：按实测飞控回执延迟（参考项目没测过）
    ack_timeout_s: float = 5.0

    # ===================== 单关节尾鳍合成 =====================
    #
    # 合成把 `yaw`（转向）与 `forward`（推力）压成一个关节指令。两路**量纲不同**，
    # 所以各有各的满量程基准 —— 这是这套参数最容易搞错的地方：
    #
    #     偏置（转向）= (yaw - neutral) / tail_fin_bias_full_us     满舵 → 1.0
    #     振幅（推力）= (forward - neutral) / tail_fin_amp_full_us  巡航 → 1.0
    #
    # 两个基准**不能合成一个**。用同一个基准（例如都用 `pwm_max - neutral = 400`）
    # 会让满舵只占 20/400 = 5% 的量纲，于是预算约束永远不触发、
    # 「大舵量吃掉推力」这个耦合等于没实现（2026-10-13 实测：一整趟触发 0 帧）。
    #
    # 预算约束（耦合 12）：
    #
    #     |偏置| * yaw_weight + |振幅| <= budget      （budget 非 None 时）
    #
    # 超了就按比例一起缩。三个区间（以 `yaw_weight = 1.5` 为例）：
    #
    #     budget = 1.0   满推力独占预算 → **一打舵就削推力**（5µs 的小修正也要削 27%）
    #     budget = 1.5   小修正不削；满推力 + 满舵削到 0.6
    #     budget >= 2.5  永不触发 → 与 `None` 等价
    #
    # **默认 `None`（关闭）是刻意的，不是偷懒。** 2026-10-13 实测：把 budget 设成
    # 1.0 / 1.5 让耦合真正生效之后，合成场地的闭环在弯角处**转不过去**，20 秒左右
    # 就触发第一条硬终止（完全脱离路线 >10 秒）；只有关掉耦合才跑得完全程。
    # 原因清楚：`corner_turn_us` 就是满舵，弯角又是「满舵 + 减速」，一进预算就被
    # 按比例削，而占位动力学需要那个舵量才转得动。
    # 但 `sim/plant.py` 那份动力学**已知是错的**（它把推力与转向当两个独立轴），
    # 所以不能拿它当整定依据（红线：不许对着占位数字调参）。于是：
    # **机器保留、默认关闭、等实测「转向 + 推力」权衡曲线之后再定这个数。**
    #: 预算上限。`None` = 关闭耦合；正数才启用。
    # 【待验证·实机】预算上限：按实测「转向+推力」权衡曲线定
    # （关闭的理由与实测数据见 `.dsh/logs/2026-10-13-tail-fin-implementation.md`）
    tail_fin_budget: float | None = None
    #: 尾鳍摆动频率，Hz。
    # 【待验证·实机】摆动频率：按实测推力响应与效率定
    tail_fin_freq_hz: float = 3.0
    #: 转向在预算里的权重（相对推力）。越大越保转向、越牺牲推力。
    # 【待验证·实机】转向权重：按实测偏航响应 vs 推力需求定
    tail_fin_yaw_weight: float = 1.5
    #: 满舵对应的舵量偏移，微秒。`None` = 取 `clamp_us`（舵量的实际上限）。
    # 【待验证·实机】满舵基准：按实测舵机-转向力矩曲线定
    tail_fin_bias_full_us: float | None = None
    #: 满推力对应的前进量偏移，微秒。`None` = 取 `cruise - neutral`。
    # 【待验证·实机】满推力基准：按实测「振幅 → 推力」曲线定
    tail_fin_amp_full_us: float | None = None
    #: 波形生成线程的更新频率，Hz。必须 > 2 * tail_fin_freq_hz，否则波形混叠。
    tail_fin_wave_rate_hz: float = 100.0
    #: 下位机串口设备节点。
    # 【待验证·实机】下位机串口：按实际接线
    tail_fin_serial_device: str = "/dev/ttyUSB0"
    #: 下位机串口波特率。
    # 【待验证·实机】串口波特率：按下位机固件
    tail_fin_serial_baud: int = 115200

    # ====================== 由上面的量算出来的 ======================

    @property
    def mm_per_px(self) -> float:
        """池底每像素多少毫米。实测值优先。"""
        if self.mm_per_px_override is not None:
            return self.mm_per_px_override
        footprint_mm = (
            2.0 * self.camera_height_m * math.tan(math.radians(self.hfov_deg) / 2.0) * 1000.0
        )
        return footprint_mm / float(self.frame_w)

    @property
    def strip_width_px(self) -> float:
        """白管在画面里应有的宽度，像素。游程窗口就建在它上面。"""
        return self.strip_width_mm / self.mm_per_px

    @property
    def marker_diameter_px(self) -> float:
        return self.marker_diameter_mm / self.mm_per_px

    @property
    def marker_area_px(self) -> tuple[float, float]:
        nominal = math.pi * (self.marker_diameter_px / 2.0) ** 2
        return nominal * self.area_ratio_lo, nominal * self.area_ratio_hi

    def clamp(self, pwm: float) -> int:
        return int(max(self.pwm_min, min(self.pwm_max, round(pwm))))

    @property
    def tail_fin_bias_scale(self) -> float:
        """满舵对应的舵量偏移，微秒。合成的分母之一。"""
        if self.tail_fin_bias_full_us is not None:
            return float(self.tail_fin_bias_full_us)
        return float(self.clamp_us)

    @property
    def tail_fin_amp_scale(self) -> float:
        """满推力对应的前进量偏移，微秒。合成的另一个分母。"""
        if self.tail_fin_amp_full_us is not None:
            return float(self.tail_fin_amp_full_us)
        return float(self.cruise - self.neutral)

    def check(self) -> None:
        """自相矛盾的配置在加载期就报错，而不是跑到一半行为怪。"""
        if self.ch_yaw == self.ch_forward:
            raise ValueError("ch_yaw and ch_forward must differ")
        if not self.pwm_min < self.neutral < self.pwm_max:
            raise ValueError("need pwm_min < neutral < pwm_max")
        if self.camera_index < 0:
            raise ValueError("camera_index must not be negative")
        if self.lost_timeout_s >= self.run_timeout_s:
            raise ValueError("lost_timeout_s must be below run_timeout_s")

        # 尾鳍。这几条不是洁癖：波形线程里一个除零或负频率只会把线程悄悄打死，
        # 主循环照样跑完全程、四条硬终止照样成立，而舵机一动不动。
        if self.tail_fin_bias_scale <= 0.0:
            raise ValueError("tail_fin_bias_scale must be positive")
        if self.tail_fin_amp_scale <= 0.0:
            raise ValueError("tail_fin_amp_scale must be positive")
        if self.tail_fin_budget is not None and self.tail_fin_budget <= 0.0:
            raise ValueError("tail_fin_budget must be positive or None (= off)")
        if self.tail_fin_yaw_weight < 0.0:
            raise ValueError("tail_fin_yaw_weight must not be negative")
        if self.tail_fin_freq_hz <= 0.0:
            raise ValueError("tail_fin_freq_hz must be positive")
        if self.tail_fin_wave_rate_hz <= 0.0:
            raise ValueError("tail_fin_wave_rate_hz must be positive")
        if self.tail_fin_freq_hz * 2.0 > self.tail_fin_wave_rate_hz:
            raise ValueError(
                f"tail_fin_freq_hz={self.tail_fin_freq_hz:g} needs "
                f"tail_fin_wave_rate_hz > {self.tail_fin_freq_hz * 2:g} "
                "(otherwise the waveform aliases)"
            )
        if self.tail_fin_serial_baud <= 0:
            raise ValueError("tail_fin_serial_baud must be positive")


def load(path: str | Path | None = None) -> Params:
    """Build the parameters, optionally overriding fields from a flat YAML file."""
    p = Params()
    if path is not None:
        import yaml

        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict):
            raise ValueError(f"{path}: expected a flat mapping of field: value")
        known = {f.name for f in dataclasses.fields(p)}
        for key, value in data.items():
            if key not in known:
                raise ValueError(f"{path}: unknown parameter {key!r}")
            setattr(p, key, value)
    p.check()
    return p
