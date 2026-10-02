"""A minimal vehicle model, so the control loop can be closed offline.

There is no vehicle on the bench and none of its dynamics have been measured, so
**every number in :class:`PlantParams` is a placeholder**.  They were chosen so
that the loop is stable and the time budget is self-consistent - a 60 microsecond
forward offset gives roughly the 200 mm/s that a 30-second run of a 5 m route
implies - not because anything was observed.

The model exists so that the controller, the state machine and the report gate
can be exercised end to end before any hardware exists, and so that the gain
sweep has something to run against.  It must be replaced by measured dynamics
before any of its conclusions are trusted; see `.dsh/agents/control.md`.

Two features are deliberate, because they are what make this vehicle hard:

* **dead time plus a first-order lag** on both axes;
* **steering requires way**, so the yaw rate is scaled by forward speed.  A
  stationary fish cannot turn, which is why the recovery logic must keep some
  forward command alive.

在链路里的位置
    每帧第 12 步（共 13 步，见 runtime.Runner._iterate），**只在合成模式存在**
    上游：main.py 发出去的两个数（yaw_us / forward_us）
    下游：新位姿 → arena.view() 生成下一帧（闭环就是这样合上的）
    参数全部是占位值。**对着它调参等于拟合虚构**（.dsh/agents/control.md）
    已知错：它把偏航与前进当成两个独立轴（只把偏航力矩按速度缩放），而实车是
            **单关节尾鳍**——两者出自同一个执行器，大舵量会直接吃掉推力。
            换真机前必须重新建模，见 .dsh/agents/architecture.md 耦合 12
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field



@dataclass
class PlantParams:
    """Vehicle response.  ALL VALUES UNVERIFIED."""

    #: Yaw rate per microsecond of yaw offset, at full way, rad/s per us.
    yaw_rate_per_us: float = 0.017
    #: First-order lag time constant for yaw rate, seconds.
    yaw_tau_s: float = 0.30
    #: Forward speed per microsecond of forward offset, mm/s per us.
    speed_per_us: float = 3.3
    #: First-order lag time constant for speed, seconds.
    speed_tau_s: float = 0.40
    #: Transport delay applied to commands, seconds.
    dead_time_s: float = 0.12
    #: Forward offset, in microseconds, at which steering authority is full.
    full_authority_us: float = 40.0


class VehiclePlant:
    """Integrate a vehicle pose from one yaw/forward command per step.

    State is the arena-frame pose, so the plant and the synthetic camera share
    one coordinate system and a rendered frame is always consistent with the
    pose that produced it.
    """

    def __init__(
        self,
        params: PlantParams,
        neutral: int,
        x_mm: float,
        y_mm: float,
        yaw_rad: float,
    ) -> None:
        self.params = params
        self.neutral = neutral
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.yaw_rad = yaw_rad
        self.speed_mm_per_s = 0.0
        self.yaw_rate_rad_per_s = 0.0
        self._delay: deque[tuple[int, int]] = deque()
        self._delay_steps = 0

    def set_dt(self, dt_s: float) -> None:
        """Set the delay line length for a given control period."""
        steps = max(0, int(round(self.params.dead_time_s / max(1e-6, dt_s))))
        if steps != self._delay_steps:
            self._delay_steps = steps
            self._delay = deque(maxlen=steps + 1 if steps else 1)

    def step(self, yaw_us: float, forward_us: float, dt_s: float) -> tuple[float, float, float]:
        """Advance the pose by ``dt_s`` under one yaw/forward command."""
        p = self.params
        yaw_offset = yaw_us - self.neutral
        forward_offset = forward_us - self.neutral

        # Transport delay: commands queue up and emerge later.
        self._delay.append((yaw_offset, forward_offset))
        if len(self._delay) > self._delay_steps:
            applied_yaw, applied_forward = self._delay[0]
        else:
            applied_yaw, applied_forward = 0, 0

        target_speed = p.speed_per_us * applied_forward
        alpha_speed = min(1.0, dt_s / max(1e-6, p.speed_tau_s))
        self.speed_mm_per_s += (target_speed - self.speed_mm_per_s) * alpha_speed

        # Steering authority scales with way: a stopped vehicle cannot turn.
        authority = min(1.0, abs(self.speed_mm_per_s) / max(1e-6, p.speed_per_us * p.full_authority_us))
        target_yaw_rate = p.yaw_rate_per_us * applied_yaw * authority
        alpha_yaw = min(1.0, dt_s / max(1e-6, p.yaw_tau_s))
        self.yaw_rate_rad_per_s += (target_yaw_rate - self.yaw_rate_rad_per_s) * alpha_yaw

        self.yaw_rad += self.yaw_rate_rad_per_s * dt_s
        self.x_mm += math.cos(self.yaw_rad) * self.speed_mm_per_s * dt_s
        self.y_mm += math.sin(self.yaw_rad) * self.speed_mm_per_s * dt_s
        return self.x_mm, self.y_mm, self.yaw_rad

    @property
    def pose(self) -> tuple[float, float, float]:
        """The current ``(x_mm, y_mm, yaw_rad)``."""
        return self.x_mm, self.y_mm, self.yaw_rad
