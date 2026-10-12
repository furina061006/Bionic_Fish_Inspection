"""主循环的逻辑测试。

这里测的是**纯函数与循环行为**，不测「鱼开得好不好」—— 动力学是占位值，对着它调参
等于拟合虚构。

每个硬终止都单独有一条用例，因为它们是赛题判罚条款：一条没生效就是丢分。
"""

from __future__ import annotations

import dataclasses
import unittest
from unittest import mock

import numpy as np

from bionic_fish import params
from bionic_fish.main import main as run
from bionic_fish.vision import Dot, LineObs

DT = 0.05


class FixedCamera:
    """每帧都返回同一张图；时间用虚拟时钟，所以测试是确定性的、而且跑得飞快。"""

    def __init__(self, frame: np.ndarray | None = None, max_frames: int = 100_000) -> None:
        self.frame = frame if frame is not None else np.zeros((320, 320, 3), np.uint8)
        self.n = 0
        self.max_frames = max_frames

    def read(self):
        if self.n >= self.max_frames:
            return None
        t = self.n * DT
        self.n += 1
        return t, self.frame


class RecordingLink:
    def __init__(self) -> None:
        self.sent: list[tuple[float, float]] = []

    def send(self, yaw_us: float, forward_us: float) -> None:
        self.sent.append((float(yaw_us), float(forward_us)))


class LoopCase(unittest.TestCase):
    def setUp(self) -> None:
        self.p = params.load()
        self.camera = FixedCamera()
        self.link = RecordingLink()

    def go(self, **overrides):
        for key, value in overrides.items():
            setattr(self.p, key, value)
        return run(self.camera, self.link, self.p)


# --------------------------------------------------------------------------- #
# 四条硬终止（赛题 PDF 的事实）
# --------------------------------------------------------------------------- #


class HardTerminationTest(LoopCase):
    def test_stops_when_the_vehicle_never_moves(self) -> None:
        """裁判宣布开始后 30 秒未启动 → 结束。

        要让「没动」成立，三档速度都得等于中位 —— 否则上一条指令就把它算成动了。
        """
        result = self.go(cruise=self.p.neutral, lost_slow=self.p.neutral, turn_slow=self.p.neutral)
        self.assertIn("未启动", result.reason)
        self.assertLess(result.elapsed_s, self.p.start_timeout_s + 1.0)

    def test_stops_after_losing_the_line_for_ten_seconds(self) -> None:
        """完全脱离路线超过 10 秒 → 终止。空画面就是彻底丢线。"""
        result = self.go(start_timeout_s=100.0)
        self.assertIn("脱离路线", result.reason)
        self.assertAlmostEqual(result.elapsed_s, self.p.lost_timeout_s, delta=0.5)

    def test_stops_at_the_wall_hit_limit(self) -> None:
        """触壁 / 触底满 3 次 → 本轮结束。

        **一次持续饱和只算一次接触** —— 所以要有三段**分开的**饱和才会到 3 次。
        每段 0.6 秒（超过 0.5 秒的判定时间），中间用 0.2 秒的正常偏差隔开。
        """
        saturated = LineObs(valid=True, confidence=1.0, e_y_px=200.0, lookahead_px=100.0)
        clear = LineObs(valid=True, confidence=1.0, e_y_px=0.0, lookahead_px=100.0)
        pattern = [saturated] * 12 + [clear] * 4
        frames = iter(pattern * 3 + [clear] * 500)

        with mock.patch("bionic_fish.main.find_white_pipe", side_effect=lambda *_: next(frames)):
            result = self.go(wall_hit_error_frac=0.5, wall_hit_infer_s=0.5, start_timeout_s=100.0)
        self.assertIn("池底/池壁", result.reason)
        self.assertLess(result.elapsed_s, 10.0, "三段饱和之后就该结束，不必等到 150 秒")

    def test_one_continuous_saturation_counts_once(self) -> None:
        """一直饱和到底只算**一次**接触，不会每 0.5 秒加一次。

        否则一次失误（比如贴着一侧池壁跑）就会把三次机会用完，本轮直接结束。
        """
        saturated = LineObs(valid=True, confidence=1.0, e_y_px=200.0, lookahead_px=100.0)
        with mock.patch("bionic_fish.main.find_white_pipe", return_value=saturated):
            result = self.go(
                wall_hit_error_frac=0.5, wall_hit_infer_s=0.5, start_timeout_s=100.0, run_timeout_s=8.0
            )
        self.assertIn("超过", result.reason, "只有一次接触，不该因撞壁结束")
        self.assertAlmostEqual(result.elapsed_s, 8.0, delta=0.5)

    def test_stops_at_the_run_timeout(self) -> None:
        """超过 150 秒 → 计时分 0。"""
        with mock.patch("bionic_fish.main.find_white_pipe", return_value=self._line()):
            result = self.go(run_timeout_s=30.0, lost_timeout_s=5.0, wall_hit_error_frac=2.0)
        self.assertIn("超过", result.reason)
        self.assertAlmostEqual(result.elapsed_s, 30.0, delta=0.5)

    @staticmethod
    def _line() -> LineObs:
        return LineObs(valid=True, confidence=1.0, lookahead_px=100.0)


# --------------------------------------------------------------------------- #
# 上报
# --------------------------------------------------------------------------- #


class ReportingTest(LoopCase):
    def _run_with_dots(self, rows: list[float]) -> int:
        """喂一串行号，每次调用 `find_black_dot` 取下一个；用尽后返回 None。"""
        dots = iter([Dot(u=160.0, v=row, diameter_px=24.0, val=50.0) for row in rows])

        def next_dot(*_args, **_kwargs):
            return next(dots, None)

        with mock.patch("bionic_fish.main.find_white_pipe", return_value=LineObs(valid=True)), \
             mock.patch("bionic_fish.main.find_black_dot", side_effect=next_dot), \
             mock.patch("bionic_fish.main.is_tape", return_value=False):
            return self.go(start_timeout_s=1000.0, lost_timeout_s=1000.0).reports

    def test_a_marker_that_stays_in_view_is_reported_once(self) -> None:
        """同一块黑点从远处走到近处，只算一个点。

        参考项目靠下降沿触发，同一块黑点在画面里停十几帧就会重复亮灯 ——
        重复上报按误报扣 3 分。
        """
        rows = [100, 150, 200, 250, 300, 310, 300, 290, 280]
        self.assertEqual(self._run_with_dots(rows), 1)

    def test_two_markers_seen_one_after_the_other_are_both_reported(self) -> None:
        """相邻两个真漏油点必须各报一次。

        赛题 13 个标记平均间距约 460mm，**小于相机视场约 400mm**，所以画面里常常
        同时有两个点。复位判据若写成「黑点离开画面」，这两个点会被合并成一个。
        """
        rows = [100, 300, 310, 120, 200, 300, 310]     # 第一个报过之后，新的又从前方案出现
        self.assertEqual(self._run_with_dots(rows), 2)

    def test_nothing_is_reported_before_the_trigger_row(self) -> None:
        """候选必须走到「已经到车下方」才上报；只是远远看见不算。"""
        self.assertEqual(self._run_with_dots([100, 150, 200, 250, 260, 270]), 0)

    def test_the_report_cap_is_respected(self) -> None:
        """到满分数之后继续报没有上行收益、风险仍是 −3。"""
        rows: list[float] = []
        for _ in range(20):
            rows += [120, 300, 310]
        self.assertEqual(self._run_with_dots(rows), self.p.max_reports)


# --------------------------------------------------------------------------- #
# 打舵与前进
# --------------------------------------------------------------------------- #


class DriveTest(unittest.TestCase):
    def test_forward_is_slew_limited(self) -> None:
        """前进量不能阶跃 —— 欠驱动 + 大滞后的车会对阶跃回一个很大的超调。"""
        from bionic_fish.drive import slew_forward

        p = params.load()
        p.forward_slew_us_per_s = 100.0
        first = slew_forward(p.neutral, p.cruise, p, DT)      # dt 0.05 → 最多动 5
        self.assertAlmostEqual(first, p.neutral + 5.0, delta=1e-6)
        second = slew_forward(first, p.cruise, p, DT)
        self.assertAlmostEqual(second, p.neutral + 10.0, delta=1e-6)

    def test_heading_error_sign_is_negative_feedback(self) -> None:
        """内环误差是 `e_theta + 修正量`，**不是差**。

        写成差会让航向环变成正反馈：一辆朝管道右侧偏的车会被命令继续往右。
        这个错误在「直道 + 横向偏置」上看不出来，因为那里 `e_theta` 恒为 0。
        """
        from bionic_fish.drive import heading_error

        p = params.load()
        # 管道在右边（正 e_y），而且管道朝右（正 e_theta）→ 误差必须为正，也就是
        # 「要往右转」。写成差的话这里会是负的。
        pipe = LineObs(valid=True, e_y_px=40.0, e_theta_rad=0.2, lookahead_px=100.0)
        self.assertGreater(heading_error(pipe, p), 0.0)

    def test_a_lost_line_does_not_reset_the_rudder(self) -> None:
        """丢线时保持上一帧舵量。

        **绝不把「没看到」当成「已经对准」** —— 那会让鱼直直冲向池壁，而「撞壁
        3 次」是硬终止。`Pid.hold()` 只推进时钟、清掉误差历史，不产生新输出。
        """
        from bionic_fish.drive import Pid, steer_offset

        p = params.load()
        pid = Pid(p)
        yaw = p.neutral + steer_offset(
            LineObs(valid=True, e_y_px=60.0, lookahead_px=100.0), p, pid, 0.0
        )
        banked = yaw
        self.assertGreater(abs(banked - p.neutral), 1.0, "巡线时应当确实打了个舵")
        pid.hold(DT)                    # 丢线：主循环不重新赋值 yaw
        self.assertEqual(yaw, banked)
        self.assertIsNone(pid._last_error, "误差历史必须清掉，免得恢复时微分出尖峰")


# --------------------------------------------------------------------------- #
# 感知（合成场地）
# --------------------------------------------------------------------------- #


class VisionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        from sim.arena import Arena

        cls.p = params.load()
        cls.arena = Arena(calibration=cls.p)

    def test_the_pipe_is_found_on_a_straight(self) -> None:
        from bionic_fish.vision import find_white_pipe

        (x0, y0), heading = self.arena.spec.point_at(350.0)
        pipe = find_white_pipe(self.arena.view(x0 + 300.0, y0, heading), self.p)
        self.assertTrue(pipe.valid)
        self.assertLess(abs(pipe.e_y_px), 5.0, "在管子正上方时横向偏差应当接近 0")

    def test_a_blank_frame_is_not_a_pipe(self) -> None:
        from bionic_fish.vision import find_white_pipe

        pipe = find_white_pipe(np.zeros((320, 320, 3), np.uint8), self.p)
        self.assertFalse(pipe.valid)

    def test_the_marker_is_found_and_reads_dark(self) -> None:
        from bionic_fish.vision import find_black_dot, find_white_pipe

        kind, mx, my = next(m for m in self.arena.markers if m[0] == "leak" and abs(m[2] - 500.0) < 1.0)
        del kind
        frame = self.arena.view(mx - 100.0, my, 0.0)
        pipe = find_white_pipe(frame, self.p)
        dot = find_black_dot(frame, pipe, self.p)
        self.assertIsNotNone(dot)
        self.assertAlmostEqual(dot.u, 160.0, delta=4.0)
        self.assertLess(dot.val, 90.0, "漏油点是纯黑，实测明度应当明显偏低")

    def test_the_distractor_is_not_reported_when_the_band_is_calibrated(self) -> None:
        """把 `dot_val_max` 压到 90，渲染器里的干扰点（灰圆）就被挡掉。

        这个 90 是**渲染器**的值 —— 它的干扰点外观是猜的。真机必须用真实物料重量。
        """
        from bionic_fish.vision import find_black_dot, find_white_pipe

        marker = next((m for m in self.arena.markers if m[0] == "distractor" and abs(m[2] - 500.0) < 1.0), None)
        if marker is None:
            self.skipTest("上直道上没有干扰点")
        _, mx, my = marker
        frame = self.arena.view(mx - 100.0, my, 0.0)
        pipe = find_white_pipe(frame, self.p)
        dot = find_black_dot(frame, pipe, self.p)
        self.assertIsNotNone(dot, "检测本身应当找得到它")
        self.assertGreater(dot.val, 90.0, "它是灰的，明度应当明显高于漏油点")
        strict = dataclasses.replace(self.p, dot_val_max=90.0)
        self.assertIsNone(find_black_dot(frame, pipe, strict))

    def test_the_start_tape_is_recognised(self) -> None:
        from bionic_fish.vision import find_white_pipe, is_tape

        (x0, y0), heading = self.arena.spec.point_at(0.0)
        frame = self.arena.view(x0, y0, heading)
        pipe = find_white_pipe(frame, self.p)
        if not pipe.valid:
            self.skipTest("起点处管道不可见")
        self.assertTrue(is_tape(frame, pipe, self.p))

    def test_a_bare_straight_is_not_a_tape(self) -> None:
        from bionic_fish.vision import find_white_pipe, is_tape

        (x0, y0), heading = self.arena.spec.point_at(1200.0)
        frame = self.arena.view(x0, y0, heading)
        pipe = find_white_pipe(frame, self.p)
        self.assertTrue(pipe.valid)
        self.assertFalse(is_tape(frame, pipe, self.p))


# --------------------------------------------------------------------------- #

class TailFinTest(unittest.TestCase):
    """单关节尾鳍：合成（耦合 12）与波形。

    耦合 12 是**同一个执行器被推力和转向争**，所以这里钉的不是「鱼开得好不好」，
    而是「预算真的会触发」和「不触发时物理输出不变」两条结构性质。
    """

    def setUp(self) -> None:
        self.p = params.load()

    # ------------------------------------------------------------ 合成
    def test_full_cruise_straight_does_not_touch_the_budget(self) -> None:
        """巡航 + 不打舵：正好用满预算，不该被削。

        这是整套参数的标定点 —— 它若被削，就等于「直着走也要减速」。
        """
        from bionic_fish.drive import synthesize_tail_fin

        amp, bias = synthesize_tail_fin(self.p.neutral, self.p.cruise, self.p)
        self.assertAlmostEqual(amp, 1.0, places=9)
        self.assertAlmostEqual(bias, 0.0, places=9)

    def test_rudder_costs_thrust(self) -> None:
        """**耦合 12**：偏置越大，可用于推进的振幅越小。

        这是「大舵量直接吃掉推力」。两个量纲若共用同一个分母（都取
        `pwm_max - neutral`），满舵只占 5% 的量纲，预算永远不触发 —— 这条就会
        变成一句空话（2026-10-13 实测：一整趟触发 0 帧）。

        **必须显式打开预算**才测得到：默认是关的（见下面那条与 `params.py`）。
        """
        from bionic_fish.drive import synthesize_tail_fin

        p = dataclasses.replace(self.p, tail_fin_budget=1.5)
        straight, _ = synthesize_tail_fin(p.neutral, p.cruise, p)
        turning, _ = synthesize_tail_fin(p.neutral + p.clamp_us, p.cruise, p)
        self.assertLess(turning, straight, "打了满舵就必须少摆一点")

    def test_the_budget_is_off_by_default_and_that_is_deliberate(self) -> None:
        """默认**不削**任何指令。关掉是刻意的，不是没实现。

        2026-10-13 实测：耦合真正生效（budget=1.0 / 1.5）之后，合成场地的闭环在
        弯角转不过去、20 秒左右触发「完全脱离路线 >10 秒」。原因清楚
        （`corner_turn_us` 就是满舵，弯角必然是满舵 + 减速，一进预算就被削），
        但 `sim/plant.py` 的动力学**已知是错的**（推力与转向当两个独立轴），
        所以不能拿它整定 —— 于是机器留着、默认关着，等实测。

        这条用例的作用是**防止有人悄悄把默认值打开**：一旦打开，闭环就从
        「到达终点」变成硬终止。
        """
        from bionic_fish.drive import synthesize_tail_fin

        p = self.p
        self.assertIsNone(p.tail_fin_budget, "默认必须是关闭")
        amp, bias = synthesize_tail_fin(p.neutral + p.clamp_us, p.cruise, p)
        self.assertAlmostEqual(bias, 1.0, places=9, msg="满舵不许被削")
        self.assertAlmostEqual(amp, 1.0, places=9, msg="满推力不许被削")

    def test_the_budget_is_never_exceeded(self) -> None:
        """打开预算后，任何输入（含远超量程的）都不许超预算。"""
        from bionic_fish.drive import synthesize_tail_fin

        p = dataclasses.replace(self.p, tail_fin_budget=1.5)
        for yaw in (1100, 1400, 1500, 1520, 1700, 1900):
            for fwd in (1100, 1500, 1590, 1800, 1900):
                amp, bias = synthesize_tail_fin(yaw, fwd, p)
                demand = abs(bias) * p.tail_fin_yaw_weight + abs(amp)
                self.assertLessEqual(
                    demand, p.tail_fin_budget + 1e-9, f"yaw={yaw} fwd={fwd}"
                )
                self.assertLessEqual(abs(bias), 1.0 + 1e-9)
                self.assertLessEqual(abs(amp), 1.0 + 1e-9)

    def test_unscaled_synthesis_reproduces_the_joint_command(self) -> None:
        """不触发预算时，关节指令必须正好是「中心 = `yaw`、摆幅 = `forward - neutral`」。

        这条钉住的是**两套量纲的分母必须分别乘回去**：把归一化后的两个数加在一起
        再乘同一根 400µs 的刻度，偏置会被放大到振幅的量级（满舵变成 400µs 而不是
        20µs），舵机会被指令打到行程边缘。
        """
        from bionic_fish.drive import synthesize_tail_fin
        from bionic_fish.fin import RecordingSerial, TailFinWaveform

        p = self.p
        yaw = p.neutral + 5.0                      # 小舵量，不触发预算
        amp, bias = synthesize_tail_fin(yaw, p.cruise, p)

        wave = TailFinWaveform(p, RecordingSerial())
        wave.update_target(amp, bias)

        # sin = 0 处就是摆动中心 → 必须正好是 yaw
        self.assertEqual(wave.pwm_at(0.0), int(round(yaw)))
        # sin = 1 处 → yaw + (forward - neutral)
        quarter = 1.0 / (4.0 * p.tail_fin_freq_hz)
        self.assertEqual(wave.pwm_at(quarter), p.neutral + 5 + (p.cruise - p.neutral))

    def test_the_budget_binds_at_full_thrust_plus_full_rudder(self) -> None:
        """打开预算时，满推力 + 满舵必须**正好**把预算用满（而不是还剩 70%）。"""
        from bionic_fish.drive import synthesize_tail_fin

        p = dataclasses.replace(self.p, tail_fin_budget=1.5)
        amp, bias = synthesize_tail_fin(p.neutral + p.clamp_us, p.cruise, p)
        demand = abs(bias) * p.tail_fin_yaw_weight + abs(amp)
        self.assertAlmostEqual(demand, p.tail_fin_budget, places=9)
        self.assertLess(amp, 1.0)

    # ------------------------------------------------------------ 波形
    def test_the_waveform_swings_around_the_bias(self) -> None:
        """波形真的在摆，而且摆幅/中心与给出的目标一致。"""
        import time

        from bionic_fish.fin import RecordingSerial, TailFinWaveform

        p = dataclasses.replace(
            self.p, tail_fin_freq_hz=10.0, tail_fin_wave_rate_hz=200.0
        )
        serial = RecordingSerial()
        wave = TailFinWaveform(p, serial)
        wave.update_target(amplitude=1.0, bias=0.0)
        self.assertTrue(wave.start())
        time.sleep(0.25)                     # 10 Hz → 约 2.5 个周期
        wave.stop()

        pwms = serial.pwms()
        self.assertGreater(len(pwms), 20, "波形线程必须真的在发")
        self.assertGreaterEqual(min(pwms), p.pwm_min)
        self.assertLessEqual(max(pwms), p.pwm_max)
        # 中心 = neutral + bias*scale = 1500；摆幅 = ±amp_scale = ±90
        self.assertAlmostEqual((min(pwms) + max(pwms)) / 2, p.neutral, delta=6)
        self.assertAlmostEqual(max(pwms) - min(pwms), 2 * p.tail_fin_amp_scale, delta=12)

    def test_stopping_returns_the_joint_to_neutral(self) -> None:
        """停摆时舵机不许停在一个偏置上 —— 那会让鱼一直往一边画圈。"""
        import time

        from bionic_fish.fin import RecordingSerial, TailFinWaveform

        p = dataclasses.replace(
            self.p, tail_fin_freq_hz=10.0, tail_fin_wave_rate_hz=200.0
        )
        serial = RecordingSerial()
        wave = TailFinWaveform(p, serial)
        wave.update_target(amplitude=1.0, bias=0.5)     # 明确偏离中位
        wave.start()
        time.sleep(0.1)
        wave.stop()
        self.assertEqual(serial.pwms()[-1], p.neutral)

    def test_the_waveform_refuses_to_start_on_a_bad_rate(self) -> None:
        """坏参数必须**拒绝启动**，而不是起一个必然除零死掉的线程。

        线程里除零的后果特别隐蔽：主循环照样跑完、四条硬终止照样成立、`Result`
        漂漂亮亮，而舵机一帧都没收到。
        """
        from bionic_fish.fin import RecordingSerial, TailFinWaveform

        p = dataclasses.replace(self.p, tail_fin_wave_rate_hz=0.0)
        wave = TailFinWaveform(p, RecordingSerial())
        self.assertFalse(wave.start())
        self.assertIsNone(wave._thread)

    # ------------------------------------------------------ 串口失败必须响
    def test_a_missing_serial_is_reported_not_swallowed(self) -> None:
        """串口打不开时 `connect()` 必须返回 `False`。

        **不许退回「假装发送」**：那会让真机上串口接错变成「跑完全程、日志漂亮、
        舵机一动不动」，而且没有任何征兆 —— 与「不解锁就跑等于白跑」是同一条。
        """
        from bionic_fish.link import TailFinLink

        p = dataclasses.replace(
            self.p, tail_fin_serial_device="/dev/definitely-not-a-tail-fin"
        )
        link = TailFinLink(p)
        try:
            self.assertFalse(link.connect())
            self.assertIsNotNone(link.last_error, "失败必须留下原因")
        finally:
            link.close()

    def test_an_injected_serial_connects_without_touching_the_hardware(self) -> None:
        """显式注入串口（测试 / 仿真）时不碰真硬件，但仍然要起波形线程。"""
        import time

        from bionic_fish.fin import RecordingSerial
        from bionic_fish.link import TailFinLink

        serial = RecordingSerial()
        link = TailFinLink(self.p, serial_port=serial)
        try:
            self.assertTrue(link.connect())
            self.assertIsNone(link.last_error)
            link.send_tail_fin(amplitude=1.0, bias=0.0)
            time.sleep(0.1)
            self.assertGreater(len(serial.written), 0)
        finally:
            link.close()
        self.assertEqual(serial.pwms()[-1], self.p.neutral)

    def test_tail_fin_link_has_no_second_synthesis_path(self) -> None:
        """`TailFinLink` 故意**不**提供 `send(yaw, forward)`。

        有两条合成路径，就会有两套行为，迟早漂移成「主循环以为发了 A、后端发了 B」。
        合成只允许发生在 `main.py` 一处。
        """
        from bionic_fish.link import TailFinLink

        self.assertFalse(hasattr(TailFinLink, "send"))

    # ------------------------------------------------------------ 参数校验
    def test_bad_tail_fin_parameters_are_rejected(self) -> None:
        """坏参数必须在加载期报错，而不是跑到一半行为怪。"""
        for field, bad in [
            ("tail_fin_budget", 0.0),
            ("tail_fin_budget", -1.0),
            ("tail_fin_yaw_weight", -0.5),
            ("tail_fin_freq_hz", 0.0),
            ("tail_fin_freq_hz", -3.0),
            ("tail_fin_wave_rate_hz", 0.0),
            ("tail_fin_serial_baud", 0),
            ("tail_fin_bias_full_us", 0.0),
            ("tail_fin_amp_full_us", -1.0),
        ]:
            p = dataclasses.replace(self.p, **{field: bad})
            with self.assertRaises(ValueError, msg=f"{field}={bad} 被放过了"):
                p.check()

    def test_an_aliasing_wave_rate_is_rejected(self) -> None:
        """波形更新率不足两倍摆动频率时拒绝 —— 那出来的不是正弦，是混叠。"""
        p = dataclasses.replace(self.p, tail_fin_freq_hz=60.0, tail_fin_wave_rate_hz=100.0)
        with self.assertRaises(ValueError, msg="60Hz 摆动配 100Hz 更新率被放过了"):
            p.check()

    def test_the_default_config_is_self_consistent(self) -> None:
        """默认值必须过 `check()`，而且两个满量程基准取到该取的字段。"""
        p = params.load()
        self.assertEqual(p.tail_fin_bias_scale, p.clamp_us)
        self.assertEqual(p.tail_fin_amp_scale, float(p.cruise - p.neutral))

    # -------------------------------------------------- 实机入口的成功路径
    def test_the_vehicle_entry_point_survives_the_default_config(self) -> None:
        """`python -m bionic_fish` 必须能走通到启动摘要那一行。

        这条是**踩出来的**：`tail_fin_budget` 默认是 `None`，而摘要行最初写成
        `f"{p.tail_fin_budget:g}"` → `TypeError: unsupported format string passed
        to NoneType.__format__`。崩点在**串口连上之后**，所以「连不上就退出」与
        `--help` 两条路径都测不到它 —— 而真机上串口是通的，一跑就崩。

        所以这里必须走**完整入口**：换掉串口、换掉相机，其余（参数加载、后端选择、
        摘要、`run()`、`finally` 里的 disarm/close）全用真代码。
        """
        import contextlib
        import io

        import bionic_fish.__main__ as entry

        from bionic_fish.fin import RecordingSerial
        from bionic_fish.link import TailFinLink

        serial = RecordingSerial()

        class InjectedSerial(TailFinLink):
            def __init__(self, p):
                super().__init__(p, serial_port=serial)

        class InjectedCamera(FixedCamera):
            """真 `Camera` 有 `close()`，入口的 `finally` 会调它。"""

            def close(self) -> None:
                pass

        with mock.patch.object(entry, "TailFinLink", InjectedSerial), \
                mock.patch.object(entry, "Camera", lambda index: InjectedCamera()), \
                contextlib.redirect_stdout(io.StringIO()) as out:
            code = entry.cli([])

        self.assertEqual(code, 0)
        self.assertIn("预算 关闭", out.getvalue(), "默认预算应打印成「关闭」而不是崩掉")
        self.assertGreater(len(serial.written), 0, "波形线程必须真的发过帧")
        self.assertEqual(serial.pwms()[-1], params.load().neutral, "退出时必须回中位")


if __name__ == "__main__":
    unittest.main()
