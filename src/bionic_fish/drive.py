"""判方向、打舵：给定期望，算出舵量和前进量。

两级串级：外环把横向偏差变成**期望航向**，内环把实际航向开到期望航向上。参考项目
直接把横向偏差喂给偏航 PID，丢掉了它已经算出来的管道斜率 —— 那是比例环在和自己的
延迟打架。
"""

from __future__ import annotations

import math

from .params import Params
from .vision import LineObs

#: 超出这个范围的 dt 当成相机抖动，不当成真的过了这么久。
DT_MIN_S = 1e-3
DT_MAX_S = 0.5


class Pid:
    """PD（可选 I）控制器，用**实测**的时间步长。

    输出是**相对中位的偏移**（微秒），钳位在 `±clamp_us`。钳偏移而不是钳绝对 PWM，
    这样限幅与通道的中位值无关。

    `dt` 实测而不是写死：参考项目给每个 PID 都构造了 `dt = 0.1`，而真实的环周期是
    相机周期加一次检测，在树莓派上远不是 0.1 秒。`dt` 错了，微分项就被悄悄重新标定。
    """

    def __init__(self, p: Params) -> None:
        self.p = p
        self._integral = 0.0
        self._last_error: float | None = None
        self._last_time: float | None = None

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = None
        self._last_time = None

    def hold(self, now_s: float) -> None:
        """只推进时钟，不产生输出。丢线时用。

        清掉误差历史，否则恢复控制的第一帧会对一个几帧前的误差做微分，产生尖峰。
        """
        self._last_time = now_s
        self._last_error = None

    def step(self, error: float, now_s: float) -> float:
        p = self.p
        if self._last_time is None:
            dt = 0.0
        else:
            dt = now_s - self._last_time
            if not (DT_MIN_S <= dt <= DT_MAX_S):
                dt = 0.0

        derivative = 0.0
        if dt > 0.0 and self._last_error is not None:
            derivative = p.kd * (error - self._last_error) / dt

        unclamped = p.kp * error + derivative
        clamped = max(-p.clamp_us, min(p.clamp_us, unclamped))
        self._last_error = error
        self._last_time = now_s
        return clamped


def desired_heading(pipe: LineObs, p: Params) -> float:
    """把横向偏差变成「应该朝哪」的修正量，弧度。

    `atan2(横向, 前瞻距离)` 正好是到前瞻点的角度，所以增益的单位是「几何正确修正量的
    倍数」，而不是一个凭空的缩放系数。

    注意这**不是**绝对航向，而是相对管道的修正量 —— 和 `e_theta_rad` 怎么合起来，
    见 `heading_error`。
    """
    geometric = math.atan2(pipe.e_y_px, max(1.0, pipe.lookahead_px))
    return max(-p.max_heading_cmd_rad, min(p.max_heading_cmd_rad, p.k_lateral * geometric))


def heading_error(pipe: LineObs, p: Params) -> float:
    """内环误差，弧度。

    `e_theta_rad` 是**管道方向减车体航向**，也就是一个「管道相对车体」的角度。
    车要到达的航向是（管道方向 + 纯追踪修正量），所以要消掉的误差是：

        (管道方向 + 修正量) - 车体航向  ==  e_theta + 修正量

    **写成两者相减——那个看起来很自然的形式——会让航向环变成正反馈**：一辆朝管道
    右侧偏的车会被命令继续往右。这个错误在「直道 + 横向偏置」上看不出来，因为那里
    `e_theta` 恒等于 0，两种写法结果一样 —— 它就是这样活过了第一轮测试的。
    """
    return desired_heading(pipe, p) + pipe.e_theta_rad


def steer_offset(pipe: LineObs, p: Params, pid: Pid, now_s: float) -> float:
    """巡线时相对中位的舵量，微秒。"""
    return pid.step(heading_error(pipe, p), now_s)


def slew_forward(current_us: float, target_us: float, p: Params, dt_s: float) -> float:
    """限制前进量的变化率，免得指令阶跃。

    参考项目在阶段之间直接给一个新 PWM；欠驱动 + 大滞后的车会对阶跃回一个很大的
    超调。
    """
    rate = p.forward_slew_us_per_s * max(0.0, dt_s)
    delta = float(target_us) - current_us
    if abs(delta) > rate:
        delta = math.copysign(rate, delta)
    return current_us + delta
