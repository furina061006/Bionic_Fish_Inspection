"""单关节尾鳍波形生成器：独立线程按 ~100Hz 生成摆动波形，通过串口发给下位机。

这个模块解决一个物理约束：主循环在跑视觉 + 控制（15–30 fps），无法直接生成
2–5 Hz 的平滑尾摆波形。所以：

- 主循环（慢）：算出期望的**振幅**与**偏置**，调 `update_target()`
- 波形线程（快）：按当前目标和相位算出瞬时关节角，发给下位机

**合成在微秒域里做**：`yaw` 与 `forward` 两个量纲的满量程基准不同
（见 `params.py` 的 `tail_fin_bias_scale` / `tail_fin_amp_scale`），
所以不能把归一化后的两个数加在一起再乘同一根刻度 —— 那样偏置会被放大成
振幅的量级。这里分别乘回各自的基准：

    pwm(t) = neutral + bias * bias_scale + amplitude * amp_scale * sin(2πft)

线程安全：`amplitude` / `bias` 由 `threading.Lock` 保护（主循环写、波形线程读）。
"""

from __future__ import annotations

import math
import threading
import time
from typing import Protocol

from .params import Params


class SerialPort(Protocol):
    """串口抽象，让测试能换成假的。"""

    def write(self, data: bytes) -> None: ...
    def close(self) -> None: ...


class TailFinWaveform:
    """单关节尾鳍波形生成器 + 串口发送器。

    波形：

        pwm(t) = neutral + bias * bias_scale + amplitude * amp_scale * sin(2πft)

    其中 `bias` / `amplitude` 是 `drive.synthesize_tail_fin()` 的输出
    （归一化，满舵 / 满推力为 1.0），两个 `*_scale` 是各自的微秒基准。

    于是不触发预算时，实际发出去的正是「中心 = `yaw`、摆幅 = `forward - neutral`」。
    """

    def __init__(self, p: Params, serial_port: SerialPort | None = None) -> None:
        self.p = p
        self._serial = serial_port

        # 当前目标（主循环写，波形线程读）
        self._lock = threading.Lock()
        self._amplitude = 0.0
        self._bias = 0.0

        # 波形生成器状态
        self._t0 = time.monotonic()
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> bool:
        """启动波形生成线程。参数非法就**拒绝启动**并返回 `False`。

        拒绝而不是「启动一个必然死掉的线程」：除零发生在子线程里，主循环收不到
        任何通知 —— 鱼会一动不动地跑完全程，日志上一切正常。

        这里只挡**会让线程崩掉**的两条（摆动频率与更新率必须为正）。完整校验在
        `Params.check()`（`load()` 会调它），**不在这里重复**：否则一个无关的字段
        （比如 `camera_index`）出错，会在这里报成「波形参数非法」，把人引向错方向。
        """
        if self._running:
            return True
        if self.p.tail_fin_freq_hz <= 0.0 or self.p.tail_fin_wave_rate_hz <= 0.0:
            return False
        self._running = True
        self._t0 = time.monotonic()
        self._thread = threading.Thread(target=self._wave_loop, daemon=True, name="TailFinWave")
        self._thread.start()
        return True

    def stop(self) -> None:
        """停止波形生成线程，并把关节送回中位。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        # 停摆的舵机不该停在一个偏置上，否则鱼会一直往一边画圈。
        if self._serial is not None:
            self._send_pwm(self.p.neutral)

    def update_target(self, amplitude: float, bias: float) -> None:
        """更新目标（主循环调用）。

        Args:
            amplitude: 摆幅，归一化，1.0 = 满推力
            bias: 摆动中心，归一化，1.0 = 满舵
        """
        with self._lock:
            self._amplitude = max(-1.0, min(1.0, amplitude))
            self._bias = max(-1.0, min(1.0, bias))

    def pwm_at(self, t_s: float) -> int:
        """`t_s` 时刻应发的 PWM。纯函数，单独可测。"""
        with self._lock:
            amplitude = self._amplitude
            bias = self._bias
        angle_us = (
            bias * self.p.tail_fin_bias_scale
            + amplitude * self.p.tail_fin_amp_scale * math.sin(2.0 * math.pi * self.p.tail_fin_freq_hz * t_s)
        )
        return self.p.clamp(self.p.neutral + angle_us)

    def _wave_loop(self) -> None:
        """波形生成线程主循环。"""
        dt = 1.0 / self.p.tail_fin_wave_rate_hz
        while self._running:
            t_start = time.monotonic()
            self._send_pwm(self.pwm_at(t_start - self._t0))
            sleep_time = dt - (time.monotonic() - t_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    def _send_pwm(self, pwm: int) -> None:
        """把一帧 PWM 发给下位机。

        协议：文本，每行一个十进制微秒值 + `\\n`。下位机固件按这个格式解析。
        换协议只改这里。
        """
        if self._serial is None:
            return
        try:
            self._serial.write(f"{pwm}\n".encode("ascii"))
        except Exception:  # noqa: BLE001 - 一帧写失败不该打死波形线程
            pass

    def close(self) -> None:
        """停止波形生成并释放串口。"""
        self.stop()
        if self._serial is not None:
            self._serial.close()


def open_serial(p: Params) -> SerialPort:
    """打开下位机串口。**打不开就抛异常**，由调用方决定这是不是致命错误。

    这里**不做**「打不开就退回记录模式」的兜底：那会让真机上串口接错
    （或没装 `pyserial`）变成「跑完全程、日志漂亮、舵机一动不动」，
    而且没有任何征兆。实机路径必须失败得响。
    """
    import serial  # 延迟导入：裸机上没有

    return serial.Serial(p.tail_fin_serial_device, p.tail_fin_serial_baud, timeout=0.1)


class RecordingSerial:
    """假串口：记录写进来的内容，不碰硬件。**只给测试与仿真用。**

    实机路径（`__main__.py`）不会自动退回它 —— 要显式传进来。
    """

    def __init__(self) -> None:
        self.written: list[bytes] = []

    def write(self, data: bytes) -> None:
        self.written.append(data)

    def close(self) -> None:
        pass

    def pwms(self) -> list[int]:
        return [int(b.decode("ascii").strip()) for b in self.written]
