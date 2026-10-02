"""全部可调参数，就在这一个文件里。

默认值**照参考项目**（`reference/pipe_inspector_raspberry-main/`），现场再慢慢调。
想改而不动代码，就写一个 YAML：

    load()                     # 全用下面的默认值
    load("tank.yaml")          # 只覆写文件里出现的键

YAML 是**平的**，键名就是下面的字段名：

    cruise: 1560
    report_row: 280
    ch_yaw: 4

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
    ch_yaw: int = 4
    ch_forward: int = 5
    neutral: int = 1500
    pwm_min: int = 1100
    pwm_max: int = 1900
    #: 参考项目 `_pipe1` 的巡航值。
    cruise: int = 1590
    #: 转弯时的前进量。参考项目在 `_turn_first` 前把速度压到中位附近；
    #: 压得太低会**转不动**（欠驱动：转向权限正比于前进速度）。
    turn_slow: int = 1525
    #: 丢线后的降级速度。参考项目 `_pipe1` 的丢线档。
    lost_slow: int = 1550
    #: `|forward - neutral|` 超过这个值就算「鱼动了」（30 秒未启动判据）。
    #:
    #: **必须远小于最小速度档与中位之差。** 丢线档是 `lost_slow - neutral = 50`，
    #: 若这里取 60，一条丢线后正在游动的鱼会被判成「从未启动」，30 秒后被
    #: 「未启动」误终止 —— 而且日志会指向错误的方向。取 20，宁可判成「动了」。
    started_forward_us: int = 20

    # ============================ 巡线控制（照参考项目 sheer_pid.py）============================
    #: 外环增益，1.0 = 几何上正确的纯追踪修正量。
    k_lateral: float = 1.0
    #: 期望航向的上限，弧度。
    max_heading_cmd_rad: float = 0.6
    kp: float = 200.0
    kd: float = 20.0
    #: 舵量相对中位的上限，微秒。
    clamp_us: float = 20.0
    #: 前进量的斜率限制，微秒/秒。防止指令阶跃。
    forward_slew_us_per_s: float = 600.0

    # ============================== 转弯 ==============================
    #: 转弯时的固定舵量（相对中位），微秒。
    corner_turn_us: float = 20.0
    #: 弯角判据一：行扫描拟合出的曲率超过这个值（弧度/行）。
    corner_curve_max: float = 0.02
    #: 弯角判据二：可用行占比低于这个值。白管在画面里变成 L 形时它会塌下来。
    corner_confidence_min: float = 0.55
    #: 起转到允许「重新看见管道就结束」之间的最短时间。
    #: 参考项目没有这一条，代价是实测**只转了 4.4°** 就宣布弯角结束、径直冲过拐角。
    corner_min_commit_s: float = 1.0
    #: 转弯的硬上限；超了就交给丢线恢复。
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
    wall_hit_error_frac: float = 0.75
    #: …持续这么久算**一次**接触。一次持续饱和只算一次。
    wall_hit_infer_s: float = 1.5
    #: 裁判宣布开始后这个秒数内没动 → 结束。
    start_timeout_s: float = 30.0
    #: 总时长上限；超过则计时分 0。
    run_timeout_s: float = 150.0

    # ============================== 上报 ==============================
    #: 图像行号，候选经过这一行就上报（「已到鱼的正下方」）。参考项目写死 y=290。
    report_row: float = 290.0
    #: 检测项满分对应的上报数；到顶后再报没有上行收益、风险仍是 −3。
    max_reports: int = 8
    #: 起点黑带长得和终点一样，所以要跑够这么久之后才认终点。
    finish_min_run_s: float = 20.0
    #: 认了终点之后继续往前挪，让整条鱼压过黑带。
    finish_creep_s: float = 1.5

    # ========================= 相机与像素标定 =========================
    camera_index: int = 0
    frame_w: int = 320
    frame_h: int = 320
    #: 镜头到**管顶**的距离（不是池底——标记贴在管顶，管顶近一个管径）。
    camera_height_m: float = 0.20
    hfov_deg: float = 90.0
    #: 实测优先：量一个 ⌀30mm 的圆在画面里占几个像素，填这里，绕过上面的模型。
    mm_per_px_override: float | None = None
    #: 白管的**直径**（俯视看圆柱，轮廓宽度就是直径）。
    strip_width_mm: float = 24.0
    #: 漏油点 / 误识别点的直径（赛题事实：3cm）。
    marker_diameter_mm: float = 30.0

    # ========================= 白管分割 =========================
    #: "tophat"（亮局部结构，默认）/ "bright_low_sat" / "red_minus_blue"。
    strip_method: str = "tophat"
    #: top-hat 核必须**大于**管道像素宽度，否则管道内部会被自己抹掉。
    tophat_ksize: int = 31
    tophat_threshold: int = 35
    v_min: int = 200
    s_max: int = 100
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
    run_ratio_lo: float = 0.7
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
    blackhat_ksize: int = 31
    blackhat_threshold: int = 40
    #: 候选中心离管道中心线的最远距离，画面宽度比例。
    max_lateral_offset_frac: float = 0.25
    #: 面积窗口，按标定直径对应的面积倍数。
    area_ratio_lo: float = 0.35
    area_ratio_hi: float = 3.0
    circularity_min: float = 0.55
    aspect_max: float = 1.8
    #: 「有多黑」的上限（OpenCV HSV 的 V，0-255）。漏油点是**纯黑**，误识别点未知，
    #: 所以默认全收（255）；标定出两者的值以后再往下压。
    dot_val_max: float = 255.0

    # ========================= 起终点黑带 =========================
    #: 灰度不超过这个值算黑带。用绝对暗度而不是黑帽响应：黑帽在亮结构的**边缘**
    #: 也会响应，会把管道自己的边界报成一条黑带。
    tape_max_grey: int = 90
    tape_width_px_lo: float = 60.0
    tape_width_px_hi: float = 400.0
    #: 黑带沿行进方向的厚度上限。池壁外的暗区会远超这个值。
    tape_thickness_px_max: float = 40.0

    # ========================= 声光与播报 =========================
    light_pin: int = 16
    buzzer_pin: int = 17
    light_hold_s: float = 0.6
    buzzer_hold_s: float = 0.25
    speech_enabled: bool = True

    # ========================= 飞控连接 =========================
    #: 参考项目用 `/dev/ttyACM0` @115200。
    mavlink_device: str = "/dev/ttyACM0"
    mavlink_baud: int = 115200
    #: 解锁要等 `COMMAND_ACK` 的秒数。参考项目把这一步注释掉、无条件打印 Armed!。
    ack_timeout_s: float = 5.0

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
