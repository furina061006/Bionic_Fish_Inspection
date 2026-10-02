"""Control: cascaded lateral control, visual-servo heave, speed schedule.

This is the refactor of the reference project's control layer
(``packaged_part/pid_catalog_first/sheer_pid.py`` and
``packaged_part/move/move.py``).  What changed, and why:

* **A heading loop exists.**  The reference drove its yaw PID straight from the
  lateral error and discarded the strip slope it had already computed, which
  makes a proportional-only loop fight its own delay.  Here the outer loop turns
  the look-ahead lateral error into a *desired heading*, and the inner loop
  drives the measured strip heading to it.  That is pure pursuit with the strip
  as the path.
* **``dt`` is measured, not hardcoded.**  The reference constructed every PID
  with ``dt = 0.1`` while the real loop period is the camera period plus a
  detection, which on a Raspberry Pi is far from 0.1 s.  A wrong ``dt`` silently
  rescales the derivative term.
* **Anti-windup is explicit.**  Integrators accumulate only while the output is
  unsaturated, so an integral term cannot build up against a clamped actuator.
* **The channel assignment is data.**  :class:`~bionic_fish.config.ChannelMap`
  says which channel carries which axis, so the same controller drives the
  reference's three-channel ROV layout or a two-channel tail-fin fish with no
  code change.

在链路里的位置
    每帧第 9 步（共 13 步，见 runtime.Runner._iterate），由 runtime 调用
    上游：vision.LineObs + fsm.Decision（目标前进量 / 弯角标志 / 转向覆盖）
    下游：Actuator → platform.Link.send（第 10 步，唯一的执行器写点）
    跨帧状态：有。两个 Pid 的积分与微分历史、_forward_us、steer_offset_us、
              _last_mode_corner。切换增益组或丢线时会 hold() / reset()，
              目的是**不在不连续点两侧做微分**
    概念：runtime-flow.md 第 6.4 节（三条分支）、.dsh/agents/control.md
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .config import ChannelMap, ControlParams, GainSet
from .vision import LineObs

#: Loop periods outside this range are treated as glitches rather than as time
#: actually elapsed.  A camera hiccup must not produce a huge derivative spike.
DT_MIN_S = 1e-3
DT_MAX_S = 0.5


@dataclass
class Actuator:
    """One control frame, expressed both as channels and as named axes.

    ``channels`` is the sole source of truth for what is transmitted; the named
    fields are kept so that logs and the report gate can talk about "yaw" rather
    than about "channel 4", which matters when the channel map changes.
    """

    channels: dict[int, int] = field(default_factory=dict)
    yaw_us: int = 0
    heave_us: int = 0
    forward_us: int = 0
    #: Yaw command offset from neutral, microseconds.  This is the saturated
    #: controller output and the quantity the wall-hit inference watches.
    yaw_offset_us: float = 0.0

    def pwm(self, channel: int | None) -> int:
        """PWM for a channel, or the neutral value when the axis is unmapped."""
        if channel is None:
            return 0
        return self.channels.get(channel, 0)


class Pid:
    """A PD (optionally PID) controller with a measured time step.

    Output is an offset from neutral in microseconds, clamped to
    ``+/- gains.clamp_us``.  Clamping the offset rather than the absolute PWM
    keeps the limit independent of the channel's neutral value.
    """

    def __init__(self, gains: GainSet) -> None:
        self.gains = gains
        self._integral = 0.0
        self._previous_error: float | None = None
        self._previous_time: float | None = None

    def reset(self) -> None:
        """Clear the integrator and derivative history."""
        self._integral = 0.0
        self._previous_error = None
        self._previous_time = None

    def hold(self, now_s: float) -> None:
        """Advance the clock without producing output.

        Used while the measurement is invalid.  The caller keeps the previous
        command, and clearing the error history stops the next valid step from
        differentiating across the gap, which would otherwise spike.
        """
        self._previous_time = now_s
        self._previous_error = None

    def step(self, error: float, now_s: float) -> float:
        """Advance the controller by one measured time step."""
        gains = self.gains
        if self._previous_time is None:
            dt = 0.0
        else:
            dt = now_s - self._previous_time
            if not (DT_MIN_S <= dt <= DT_MAX_S):
                dt = 0.0

        proportional = gains.kp * error

        derivative = 0.0
        if dt > 0.0 and self._previous_error is not None:
            derivative = gains.kd * (error - self._previous_error) / dt

        unclamped = proportional + derivative + gains.ki * self._integral
        clamped = max(-gains.clamp_us, min(gains.clamp_us, unclamped))

        # Integrate only when the output is not saturated, so the integrator
        # cannot wind up against a clamped actuator.
        if gains.ki != 0.0 and dt > 0.0 and abs(unclamped) <= gains.clamp_us:
            self._integral += error * dt

        self._previous_error = error
        self._previous_time = now_s
        return clamped


class LineController:
    """Turn perception observations into actuator commands."""

    def __init__(self, params: ControlParams, channels: ChannelMap) -> None:
        self.params = params
        self.channels = channels
        self.yaw = Pid(params.yaw_follow)
        self.heave = Pid(params.heave_gain)
        self._forward_us = float(channels.neutral)
        self._last_mode_corner = False
        self.steer_offset_us = 0.0

    def reset(self) -> None:
        """Return to the pre-start state."""
        self.yaw.reset()
        self.heave.reset()
        self._forward_us = float(self.channels.neutral)
        self._last_mode_corner = False
        self.steer_offset_us = 0.0

    # ------------------------------------------------------------------ lateral

    def desired_heading(self, line: LineObs) -> float:
        """Pure-pursuit correction that would bring the vehicle onto the strip.

        The sign follows the project-wide convention: positive means "to the
        vehicle's right".  ``atan2(lateral, lookahead)`` is the exact angle to
        the look-ahead point, so the gain has units of "fraction of the
        geometrically correct correction" rather than an arbitrary scale factor.

        Note what this is *not*: it is a correction **relative to the strip**, not
        an absolute heading.  Combining it with ``e_theta`` correctly is the
        subject of :meth:`heading_error`.
        """
        geometric = math.atan2(line.e_y_px, max(1.0, line.lookahead_px))
        commanded = self.params.k_lateral * geometric
        limit = self.params.max_heading_cmd_rad
        return max(-limit, min(limit, commanded))

    def heading_error(self, line: LineObs) -> float:
        """Yaw error whose sign means "rotate this way", radians.

        ``LineObs.e_theta_rad`` is the strip direction **minus** the vehicle
        heading, so it is a strip-relative-to-vehicle angle: positive means the
        strip heads to the vehicle's right.  The heading the vehicle must reach
        is therefore (strip direction + the pure-pursuit correction), and the
        error to drive out is::

            (psi_strip + e_theta_d) - psi_vehicle  ==  e_theta + e_theta_d

        Subtracting the two, which is the obvious-looking form, gives
        ``e_theta_d - e_theta`` and makes the heading loop *positive feedback*:
        a vehicle pointing to the right of the strip would be commanded further
        right.  That inversion is invisible on a straight run with a lateral
        offset, because there ``e_theta`` is zero and both forms agree - which is
        exactly how it survived the first round of testing.
        """
        return self.desired_heading(line) + line.e_theta_rad

    def steer_offset(self, line: LineObs, now_s: float, corner: bool) -> float:
        """Yaw offset from neutral, microseconds."""
        if corner != self._last_mode_corner:
            # Swapping gain sets mid-turn would differentiate across the
            # discontinuity, so reset the derivative history on the transition.
            self._last_mode_corner = corner
            self.yaw.gains = self.params.yaw_corner if corner else self.params.yaw_follow
            self.yaw.reset()

        return self.yaw.step(self.heading_error(line), now_s)

    # -------------------------------------------------------------------- heave

    def heave_offset(self, line: LineObs, now_s: float) -> float:
        """Visual-servo heave offset from neutral, microseconds.

        The controlled quantity is the strip's apparent width, which a nadir
        camera over a flat floor ties directly to the lens-to-floor distance.
        Keeping it constant therefore keeps the camera at a fixed height without
        any depth sensor.  The reference project did the same thing with the
        pipe's fill ratio; using the strip width is the same idea with a
        measurement that is already available and calibrated.
        """
        error = line.strip_width_px - self.params.target_strip_width_px
        return self.params.heave_trim_us + self.heave.step(error, now_s)

    # ------------------------------------------------------------------ forward

    def forward_us(self, target_us: int, now_s: float, dt_s: float) -> int:
        """Slew-limited forward command."""
        rate = self.params.forward_slew_us_per_s * max(0.0, dt_s)
        delta = float(target_us) - self._forward_us
        if abs(delta) > rate:
            delta = math.copysign(rate, delta)
        self._forward_us += delta
        return self.channels.clamp(self._forward_us)

    # -------------------------------------------------------------------- frame

    def step(
        self,
        line: LineObs,
        target_forward_us: int,
        now_s: float,
        dt_s: float,
        corner: bool = False,
        yaw_override_us: float | None = None,
    ) -> Actuator:
        """Produce one :class:`Actuator` frame.

        ``yaw_override_us`` commits the yaw axis to a fixed offset, bypassing the
        strip-following loop.  It is used to fly a bend, where the strip has left
        the frame and no heading reference remains; the loop's history is held so
        that it does not differentiate across the gap when control resumes.

        With an invalid strip observation and no override, the previous yaw
        command is held and the forward command slews to its target; the caller
        chooses that target from the state machine's recovery level.  The
        controller never treats "no strip" as "centred".
        """
        channels = self.channels
        neutral = channels.neutral

        if yaw_override_us is not None:
            self.steer_offset_us = max(
                -self.params.yaw_corner.clamp_us,
                min(self.params.yaw_corner.clamp_us, yaw_override_us),
            )
            self.yaw.hold(now_s)
            heave_offset = self.params.heave_trim_us
        elif line.valid:
            self.steer_offset_us = self.steer_offset(line, now_s, corner)
            heave_offset = self.heave_offset(line, now_s)
        else:
            # Hold the last yaw command; freeze the heave trim.  Do not feed a
            # fabricated zero error into either loop.
            self.yaw.hold(now_s)
            self.heave.hold(now_s)
            heave_offset = self.params.heave_trim_us

        forward = self.forward_us(target_forward_us, now_s, dt_s)

        yaw_us = channels.clamp(neutral + self.steer_offset_us)
        heave_us = channels.clamp(neutral + heave_offset)

        mapping: dict[int, int] = {}
        if channels.yaw is not None:
            mapping[channels.yaw] = yaw_us
        if channels.heave is not None:
            mapping[channels.heave] = heave_us
        if channels.forward is not None:
            mapping[channels.forward] = forward

        return Actuator(
            channels=mapping,
            yaw_us=yaw_us,
            heave_us=heave_us,
            forward_us=forward,
            yaw_offset_us=self.steer_offset_us,
        )
