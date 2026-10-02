"""Platform interface: the MAVLink link and the outward annunciator.

This wraps the parts of the reference project that touched hardware
(``packaged_part/move/move.py``, ``arm_disarm.py``, ``init.py`` and
``camera_gimbal_control/LED.py``), with three deliberate changes:

* **One connection.**  The reference opened a separate ``mavutil`` connection
  inside ``move.py``, ``arm_disarm.arm()`` and ``arm_disarm.disarm()`` - three
  live links to one serial port, each doing its own ``wait_heartbeat`` and each
  reading ``target_system`` possibly before the heartbeat arrived.
* **Arming is verified.**  The reference commented out ``motors_armed_wait()``
  and printed ``Armed!`` unconditionally, so an arming failure proceeded
  silently.  Here arming waits for a ``COMMAND_ACK`` and reports failure.
* **Nothing blocks the control loop.**  The reference's servo helpers called
  ``sleep(0.3)`` inline.  The annunciator here is driven by ``tick(now)`` so
  that holding a light on never stalls perception or control.

``RPi.GPIO`` is imported lazily.  Every backend here has a software fallback so
that the full stack runs - and is tested - on a development machine with no
autopilot and no GPIO.

在链路里的位置
    每帧第 7、8、10 步（共 13 步，见 runtime.Runner._iterate）
    第 10 步 link.send(actuator) 是**全仓库唯一的执行器写点**
    上游：control.Actuator（控制）+ reporting.ReportEvent（声光与播报）
    下游：MAVLink RC_CHANNELS_OVERRIDE / RPi.GPIO / 语音 sink
    这是唯一的硬件出口，所以其余部分能在没有硬件的机器上跑完
    跨帧状态：Annunciator 有（light_off_at / buzzer_off_at / announcements）——
              灯要能被关掉、播报要能与上报一一对应
    待定：车体是自研的。若最终由树莓派直接驱动舵机、没有飞控，这里需要加第三个
          后端；Link 协议就是为这种替换准备的，后端经 Runner 的 link 参数注入，
          其余代码一行都不用改
    概念：runtime-flow.md 第 5 节；.dsh/agents/reporting.md（声光与语音的评分）
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Protocol

from .config import AnnunciatorParams, ChannelMap
from .control import Actuator
from .reporting import ReportEvent

#: MAVLink's "do not override this channel" value for RC_CHANNELS_OVERRIDE.
RC_CHANNEL_IGNORE = 65535
#: RC_CHANNELS_OVERRIDE carries up to 18 channels; the message itself carries 18.
RC_CHANNEL_COUNT = 18


class Link(Protocol):
    """The control output path."""

    def arm(self) -> bool:
        """Arm the vehicle, returning whether it was accepted."""
        ...

    def disarm(self) -> None:
        """Disarm the vehicle."""
        ...

    def send(self, actuator: Actuator) -> None:
        """Transmit one control frame."""
        ...

    def yaw_rad(self) -> float | None:
        """Current heading in radians, or ``None`` when unavailable."""
        ...

    def close(self) -> None:
        """Release the connection."""
        ...


class SimLink:
    """A link that records commands instead of transmitting them.

    Used for offline runs and tests.  It is the only backend that can be used
    when no autopilot is attached, which is the normal state on the development
    machine.

    ``heading_fn`` optionally supplies a heading, which lets the closed loop
    exercise the angle-based bend termination without an autopilot.  That does
    not make the vehicle model real - it only removes the *heading source* from
    the list of things that must exist before the bend logic can be tested.
    """

    def __init__(self, heading_fn: "Callable[[], float] | None" = None) -> None:
        self.sent: list[Actuator] = []
        self.armed = False
        self.closed = False
        self._heading_fn = heading_fn

    def arm(self) -> bool:
        """Record the arm request."""
        self.armed = True
        return True

    def disarm(self) -> None:
        """Record the disarm request."""
        self.armed = False

    def send(self, actuator: Actuator) -> None:
        """Record one control frame."""
        self.sent.append(actuator)

    def yaw_rad(self) -> float | None:
        """Return the injected heading, or ``None``."""
        return None if self._heading_fn is None else float(self._heading_fn())

    def set_heading_source(self, heading_fn: "Callable[[], float] | None") -> None:
        """Inject a heading source, for bench runs and tests."""
        self._heading_fn = heading_fn

    def close(self) -> None:
        """Mark the link closed."""
        self.closed = True

    @property
    def last(self) -> Actuator | None:
        """The most recent frame, or ``None``."""
        return self.sent[-1] if self.sent else None


class MavlinkLink:
    """A single MAVLink connection to an ArduSub autopilot.

    Control is sent as ``RC_CHANNELS_OVERRIDE`` rather than as a position or
    attitude target, matching the reference platform: the autopilot keeps
    stabilisation and depth hold, and this stack supplies stick positions.
    """

    def __init__(
        self,
        device: str,
        baud: int,
        channels: ChannelMap,
        *,
        ack_timeout_s: float = 5.0,
        attitude_timeout_s: float = 0.5,
    ) -> None:
        self.device = device
        self.baud = baud
        self.channels = channels
        self.ack_timeout_s = ack_timeout_s
        self.attitude_timeout_s = attitude_timeout_s
        self._master = None
        self._mavlink = None
        self._yaw: float | None = None
        self._yaw_at: float | None = None
        #: Why the last :meth:`connect` failed, or ``None`` when it succeeded.
        self.last_error: str | None = None

    # ------------------------------------------------------------- connection

    def connect(self, *, heartbeat_timeout_s: float = 30.0) -> bool:
        """Open the link and wait for a heartbeat.

        Returns whether the autopilot was found.  A missing autopilot, an absent
        serial device, and a missing driver are all **returned failures rather
        than exceptions**: the most likely case on a bench is simply that nothing
        is plugged in, and that is not a defect in this program.  Turning it into
        a traceback would bury the one actionable line in twenty frames of stack.

        :attr:`last_error` carries the reason when this returns ``False``.
        """
        from pymavlink import mavutil  # imported lazily: absent on a bare host

        self._mavlink = mavutil.mavlink
        try:
            self._master = mavutil.mavlink_connection(self.device, baud=self.baud)
            heartbeat = self._master.wait_heartbeat(timeout=heartbeat_timeout_s)
        except Exception as exc:  # noqa: BLE001 - any driver failure is a reported failure
            # Covers FileNotFoundError (no such device), ModuleNotFoundError
            # (pymavlink without pyserial), and serial.SerialException (in use).
            self._master = None
            self.last_error = f"{type(exc).__name__}: {exc}"
            return False

        if heartbeat is None:
            self.last_error = f"no heartbeat within {heartbeat_timeout_s:g}s"
            return False
        self.last_error = None
        return True

    @property
    def connected(self) -> bool:
        """Whether a heartbeat has been seen."""
        return self._master is not None and self._master.target_system != 0

    # ------------------------------------------------------------------ arming

    def _wait_command_ack(self, command: int) -> bool:
        """Wait for the COMMAND_ACK of a specific command.

        The command number is checked: an unrelated acknowledgement must not be
        accepted as confirmation of this one.
        """
        import time

        if self._master is None:
            return False
        accepted = int(getattr(self._mavlink, "MAV_RESULT_ACCEPTED", 0))
        deadline = time.monotonic() + self.ack_timeout_s
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            ack = self._master.recv_match(type="COMMAND_ACK", blocking=True, timeout=remaining)
            if ack is None or int(getattr(ack, "command", -1)) != command:
                continue
            return int(getattr(ack, "result", -1)) == accepted
        return False

    def arm(self) -> bool:
        """Arm the vehicle and verify the acknowledgement."""
        if self._master is None:
            return False
        command = int(
            getattr(self._mavlink, "MAV_CMD_COMPONENT_ARM_DISARM", 400)
        )
        self._master.mav.command_long_send(
            self._master.target_system,
            self._master.target_component,
            command,
            0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        return self._wait_command_ack(command)

    def disarm(self) -> None:
        """Disarm the vehicle without waiting for confirmation."""
        if self._master is None:
            return
        command = int(getattr(self._mavlink, "MAV_CMD_COMPONENT_ARM_DISARM", 400))
        self._master.mav.command_long_send(
            self._master.target_system,
            self._master.target_component,
            command,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    # ---------------------------------------------------------------- commands

    def _override(self, values: dict[int, int]) -> None:
        """Send one RC_CHANNELS_OVERRIDE with the given channels set.

        Unlisted channels are filled with :data:`RC_CHANNEL_IGNORE` so that the
        autopilot keeps its own value for them; this is the behaviour the
        reference project's zero-filled list did not achieve.
        """
        if self._master is None:
            return
        payload = [RC_CHANNEL_IGNORE] * RC_CHANNEL_COUNT
        for channel, pwm in values.items():
            if 1 <= channel <= RC_CHANNEL_COUNT:
                payload[channel - 1] = int(pwm)
        self._master.mav.rc_channels_override_send(
            self._master.target_system,
            self._master.target_component,
            *payload,
        )

    def send(self, actuator: Actuator) -> None:
        """Transmit one control frame, centring any unmapped axis."""
        values = dict(actuator.channels)
        # Axes with no channel present in the frame are explicitly centred so a
        # stale command cannot persist after a state change.
        for channel in (self.channels.yaw, self.channels.heave, self.channels.forward):
            if channel is not None and channel not in values:
                values[channel] = self.channels.neutral
        self._override(values)

    def send_channel(self, channel: int, pwm: int) -> None:
        """Drive a single channel; used by the calibration procedure."""
        self._override({channel: self.channels.clamp(pwm)})

    def set_lights(self, on: bool) -> None:
        """Switch the illumination channel, if one is mapped."""
        if self.channels.lights is None:
            return
        self._override(
            {self.channels.lights: self.channels.lights_on if on else self.channels.lights_off}
        )

    def yaw_rad(self) -> float | None:
        """Most recent heading from ``ATTITUDE``, or ``None`` if stale.

        The same connection carries control output and telemetry on purpose: a
        MAVLink stream has exactly one reader, so the reference project's habit of
        opening a second connection for telemetry would have raced this one for
        messages.

        A stale heading returns ``None`` rather than the last value, so the state
        machine falls back to its time-bounded turn instead of turning on a
        frozen measurement.
        """
        import time

        if self._master is None:
            return None
        # Drain everything queued so the newest attitude wins; a control loop
        # must never act on a backlog.
        while True:
            message = self._master.recv_match(type="ATTITUDE", blocking=False)
            if message is None:
                break
            self._yaw = float(getattr(message, "yaw", 0.0))
            self._yaw_at = time.monotonic()
        if self._yaw is None or self._yaw_at is None:
            return None
        if time.monotonic() - self._yaw_at > self.attitude_timeout_s:
            return None
        return self._yaw

    def close(self) -> None:
        """Close the serial connection if it was opened."""
        if self._master is not None:
            self._master.close()
            self._master = None


# --------------------------------------------------------------------------- #
# Annunciator
# --------------------------------------------------------------------------- #


class GpioBackend(Protocol):
    """Minimal GPIO surface, so the real one can be swapped for a recorder."""

    def setup_output(self, pin: int) -> None:
        """Configure a pin as an output."""
        ...

    def write(self, pin: int, value: bool) -> None:
        """Drive a pin high or low."""
        ...


class RecordingGpio:
    """A GPIO backend that records transitions instead of driving pins."""

    def __init__(self) -> None:
        self.events: list[tuple[float, int, bool]] = []
        self._clock = 0.0

    def set_clock(self, now_s: float) -> None:
        """Advance the recorded clock."""
        self._clock = now_s

    def setup_output(self, pin: int) -> None:
        """Accept a pin configuration."""

    def write(self, pin: int, value: bool) -> None:
        """Record a pin transition."""
        self.events.append((self._clock, pin, value))


class _RpiGpio:
    """``RPi.GPIO`` adapter, imported lazily so the host can run without it."""

    def __init__(self) -> None:
        import RPi.GPIO as GPIO  # type: ignore[import-not-found]

        self._gpio = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

    def setup_output(self, pin: int) -> None:
        """Configure a pin as an output."""
        self._gpio.setup(pin, self._gpio.OUT, initial=False)

    def write(self, pin: int, value: bool) -> None:
        """Drive a pin high or low."""
        self._gpio.output(pin, bool(value))


class Annunciator:
    """Light, buzzer and speech, all driven by one :class:`ReportEvent`.

    The competition scores the spoken announcement separately and docks 2 points
    for each leak that goes unannounced, so the announcement must correspond
    one-to-one with what was reported.  That is only guaranteed if both are
    driven by the same event, which is why the reporter emits a single
    :class:`~bionic_fish.reporting.ReportEvent` and this class subscribes to it
    rather than deciding for itself when to speak (invariant 5).
    """

    def __init__(
        self,
        params: AnnunciatorParams,
        *,
        gpio: GpioBackend | None = None,
        speaker: "Speaker | None" = None,
    ) -> None:
        self.params = params
        self.gpio = gpio
        self.speaker = speaker
        self.light_off_at: float | None = None
        self.buzzer_off_at: float | None = None
        self.announcements: list[str] = []
        if self.gpio is not None and params.light_enabled:
            self.gpio.setup_output(params.light_pin)
        if self.gpio is not None and params.buzzer_enabled:
            self.gpio.setup_output(params.buzzer_pin)

    @classmethod
    def for_host(cls, params: AnnunciatorParams, *, speaker: "Speaker | None" = None) -> "Annunciator":
        """Build an annunciator using real GPIO when available, else a recorder.

        A missing ``RPi.GPIO`` is expected on the development machine and is not
        an error; the stack must stay runnable and testable there.
        """
        try:
            gpio: GpioBackend = _RpiGpio()
        except Exception:  # noqa: BLE001 - any import or setup failure is a host limitation
            gpio = RecordingGpio()
        return cls(params, gpio=gpio, speaker=speaker)

    def on_report(self, event: ReportEvent, now_s: float) -> None:
        """React to a report: light, buzzer, and one spoken announcement."""
        if self.params.light_enabled and self.gpio is not None:
            self._set_clock(now_s)
            self.gpio.write(self.params.light_pin, True)
            self.light_off_at = now_s + self.params.light_hold_s
        if self.params.buzzer_enabled and self.gpio is not None:
            self._set_clock(now_s)
            self.gpio.write(self.params.buzzer_pin, True)
            self.buzzer_off_at = now_s + self.params.buzzer_hold_s
        if self.params.speech_enabled:
            text = event.announcement
            self.announcements.append(text)
            if self.speaker is not None:
                self.speaker.say(text)

    def tick(self, now_s: float) -> None:
        """Turn the light and buzzer off once their hold has elapsed.

        Called from the control loop, so holding a signal never blocks it.
        """
        if self.gpio is None:
            return
        self._set_clock(now_s)
        if self.light_off_at is not None and now_s >= self.light_off_at:
            self.gpio.write(self.params.light_pin, False)
            self.light_off_at = None
        if self.buzzer_off_at is not None and now_s >= self.buzzer_off_at:
            self.gpio.write(self.params.buzzer_pin, False)
            self.buzzer_off_at = None

    def _set_clock(self, now_s: float) -> None:
        if isinstance(self.gpio, RecordingGpio):
            self.gpio.set_clock(now_s)


class Speaker(Protocol):
    """A text-to-speech sink."""

    def say(self, text: str) -> None:
        """Speak one short phrase."""
        ...


@dataclass
class LoggingSpeaker:
    """A speaker that records phrases instead of vocalising them.

    The development machine has no audio path and no chosen TTS engine, so this
    is the default.  Which engine the vehicle will use is UNVERIFIED; the
    competition only requires that the announcement be intelligible in the
    recorded video, which must be checked on site.
    """

    spoken: list[str] = field(default_factory=list)

    def say(self, text: str) -> None:
        """Record one phrase."""
        self.spoken.append(text)
