"""出：把两个数字发给飞控，以及亮灯 / 响蜂鸣 / 播报。

这是通向外界的**两个口之一**（另一个是 `camera.py`）。核心只要求对象有
`send(yaw_us, forward_us)`，所以 `sim/` 能塞一个记录器进来而不改任何逻辑。

两条从参考项目继承时必须改掉的习惯：

1. **只开一条 MAVLink 连接。** 参考项目在 `move.py`、`arm_disarm.arm()`、
   `disarm()` 里各开一条，三个对象抢同一串口的心跳。MAVLink 流只有一个读者。
2. **解锁要校验回执。** 参考项目把 `motors_armed_wait()` 注释掉、**无条件打印
   `Armed!`**，解锁失败会静默继续。
"""

from __future__ import annotations

from typing import Protocol

from .params import Params

#: RC_CHANNELS_OVERRIDE 里「不覆盖这个通道」的取值。
IGNORE = 65535
#: 该消息带 18 个通道。
CHANNELS = 18


class Speaker(Protocol):
    def say(self, text: str) -> None: ...


class LoggingSpeaker:
    """默认播报器：只记下来。真机要换成 TTS 引擎。"""

    def __init__(self) -> None:
        self.spoken: list[str] = []

    def say(self, text: str) -> None:
        self.spoken.append(text)


class MavlinkLink:
    """和一台 ArduSub 飞控的单条 MAVLink 连接。

    控制指令发的是 **`RC_CHANNELS_OVERRIDE`**（遥控杆位），不是位置或姿态目标，
    与参考项目一致：稳定与定深留给飞控，我们只给杆位。
    """

    def __init__(self, p: Params) -> None:
        self.p = p
        self._master = None
        self._mavlink = None
        #: 上一次 `connect()` 为什么失败；成功时为 None。
        self.last_error: str | None = None

    # ------------------------------------------------------------- 连接与解锁

    def connect(self, *, heartbeat_timeout_s: float = 30.0) -> bool:
        """开串口并等心跳。

        设备不存在、驱动缺失、串口被占 —— 全部返回 `False` 而不是抛异常：台架上最
        常见的情况就是什么都没插，那不是程序的缺陷，把它变成一段 traceback 会把唯一
        有用的那句话说掉。失败原因在 `last_error` 里。
        """
        from pymavlink import mavutil  # 延迟导入：裸机上没有

        self._mavlink = mavutil.mavlink
        try:
            self._master = mavutil.mavlink_connection(self.p.mavlink_device, baud=self.p.mavlink_baud)
            heartbeat = self._master.wait_heartbeat(timeout=heartbeat_timeout_s)
        except Exception as exc:  # noqa: BLE001 - 任何驱动失败都算「报告型的失败」
            self._master = None
            self.last_error = f"{type(exc).__name__}: {exc}"
            return False
        if heartbeat is None:
            self.last_error = f"no heartbeat within {heartbeat_timeout_s:g}s"
            return False
        self.last_error = None
        return True

    def _wait_ack(self, command: int) -> bool:
        """等某个命令的 COMMAND_ACK。要核对命令号 —— 别的 ACK 不算数。"""
        import time

        if self._master is None:
            return False
        accepted = int(getattr(self._mavlink, "MAV_RESULT_ACCEPTED", 0))
        deadline = time.monotonic() + self.p.ack_timeout_s
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            ack = self._master.recv_match(type="COMMAND_ACK", blocking=True, timeout=remaining)
            if ack is None or int(getattr(ack, "command", -1)) != command:
                continue
            return int(getattr(ack, "result", -1)) == accepted
        return False

    def arm(self) -> bool:
        """解锁，并校验回执。"""
        if self._master is None:
            return False
        command = int(getattr(self._mavlink, "MAV_CMD_COMPONENT_ARM_DISARM", 400))
        self._master.mav.command_long_send(
            self._master.target_system, self._master.target_component,
            command, 0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )
        return self._wait_ack(command)

    def disarm(self) -> None:
        """上锁，不等回执。"""
        if self._master is None:
            return
        command = int(getattr(self._mavlink, "MAV_CMD_COMPONENT_ARM_DISARM", 400))
        self._master.mav.command_long_send(
            self._master.target_system, self._master.target_component,
            command, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )

    # ------------------------------------------------------------------ 发指令

    def send(self, yaw_us: float, forward_us: float) -> None:
        """发一帧。没映射的通道填「不覆盖」。"""
        if self._master is None:
            return
        values = {self.p.ch_yaw: self.p.clamp(yaw_us), self.p.ch_forward: self.p.clamp(forward_us)}
        payload = [IGNORE] * CHANNELS
        for channel, pwm in values.items():
            if 1 <= channel <= CHANNELS:
                payload[channel - 1] = int(pwm)
        self._master.mav.rc_channels_override_send(
            self._master.target_system, self._master.target_component, *payload
        )

    def send_channel(self, channel: int, pwm: int) -> None:
        """单独驱动一个通道。标定与「静态验接口」用。"""
        if self._master is None:
            return
        payload = [IGNORE] * CHANNELS
        if 1 <= channel <= CHANNELS:
            payload[channel - 1] = self.p.clamp(pwm)
        self._master.mav.rc_channels_override_send(
            self._master.target_system, self._master.target_component, *payload
        )

    def close(self) -> None:
        if self._master is not None:
            self._master.close()
            self._master = None


# --------------------------------------------------------------------------- #
# 声光与播报
# --------------------------------------------------------------------------- #


class Gpio(Protocol):
    def setup_output(self, pin: int) -> None: ...
    def write(self, pin: int, value: bool) -> None: ...


class RecordingGpio:
    """没有 `RPi.GPIO` 时的替代品：记录引脚变化，不碰硬件。"""

    def __init__(self) -> None:
        self.events: list[tuple[str, int, bool]] = []

    def setup_output(self, pin: int) -> None:
        self.events.append(("setup", pin, False))

    def write(self, pin: int, value: bool) -> None:
        self.events.append(("write", pin, value))


class _RpiGpio:
    def __init__(self) -> None:
        import RPi.GPIO as GPIO  # 延迟导入：开发机上没有

        self._gpio = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

    def setup_output(self, pin: int) -> None:
        self._gpio.setup(pin, self._gpio.OUT)

    def write(self, pin: int, value: bool) -> None:
        self._gpio.output(pin, bool(value))


class Annunciator:
    """灯、蜂鸣、播报，全部由**同一个**上报触发。

    语音是独立得分项，漏播一个 `−2`，所以播报不能自己决定什么时候说话 —— 它必须和
    上报共用一次触发，否则两套条件一定会漂移成「报了不播」。
    """

    def __init__(self, p: Params, gpio: Gpio | None = None, speaker: Speaker | None = None) -> None:
        self.p = p
        self.gpio = gpio
        self.speaker = speaker
        self._light_off_at: float | None = None
        self._buzzer_off_at: float | None = None
        self.spoken: list[str] = []
        if gpio is not None:
            gpio.setup_output(p.light_pin)
            gpio.setup_output(p.buzzer_pin)

    @classmethod
    def for_host(cls, p: Params, speaker: Speaker | None = None) -> "Annunciator":
        """有真 GPIO 就用真的，没有就用记录器。开发机上没有是正常的，不是错误。"""
        try:
            gpio: Gpio = _RpiGpio()
        except Exception:  # noqa: BLE001 - 任何导入/初始化失败都算本机限制
            gpio = RecordingGpio()
        return cls(p, gpio=gpio, speaker=speaker)

    def on_report(self, n: int, now_s: float) -> None:
        """第 n 个漏油点：亮灯、响一声、播报一次。"""
        if self.gpio is not None:
            self.gpio.write(self.p.light_pin, True)
            self.gpio.write(self.p.buzzer_pin, True)
        self._light_off_at = now_s + self.p.light_hold_s
        self._buzzer_off_at = now_s + self.p.buzzer_hold_s
        text = f"第 {n} 点"
        self.spoken.append(text)
        if self.p.speech_enabled and self.speaker is not None:
            self.speaker.say(text)

    def tick(self, now_s: float) -> None:
        """到点关灯关蜂鸣。由主循环调用，所以按住信号不会阻塞控制。"""
        if self.gpio is None:
            return
        if self._light_off_at is not None and now_s >= self._light_off_at:
            self.gpio.write(self.p.light_pin, False)
            self._light_off_at = None
        if self._buzzer_off_at is not None and now_s >= self._buzzer_off_at:
            self.gpio.write(self.p.buzzer_pin, False)
            self._buzzer_off_at = None
