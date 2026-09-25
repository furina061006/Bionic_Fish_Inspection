"""Unit tests for control, the report gate, the state machine and signalling.

These use hand-built observations rather than the synthetic arena, so they test
the decision logic independently of perception and independently of any gain
tuning.  That matters: the closed-loop behaviour still depends on placeholder
vehicle dynamics, but the rules these tests encode come straight from the
competition document and must hold regardless of tuning.

Run with::

    PYTHONPATH=src python -m unittest discover -s tests
"""

from __future__ import annotations

import dataclasses
import math
import unittest

from bionic_fish.config import (
    AnnunciatorParams,
    Config,
    ControlParams,
    FsmParams,
    GainSet,
    MarkerParams,
    ReportParams,
    SafetyParams,
)
from bionic_fish.control import Actuator, LineController, Pid
from bionic_fish.fsm import AbortReason, SafetyMonitor, State, StateMachine
from bionic_fish.platform import Annunciator, LoggingSpeaker, RecordingGpio, SimLink
from bionic_fish.reporting import Reporter, candidate_score
from bionic_fish.vision import CrossObs, LineObs, MarkerCand


def make_line(**overrides) -> LineObs:
    """A valid, centred strip observation."""
    base = dict(
        valid=True,
        lookahead_px=89.6,
        e_y_px=0.0,
        e_y_norm=0.0,
        e_theta_rad=0.0,
        curvature_rad_per_row=0.0,
        strip_width_px=19.2,
        support_rows=140,
        confidence=0.93,
        fit=(0.0, 160.0),
    )
    base.update(overrides)
    return LineObs(**base)


def make_cand(**overrides) -> MarkerCand:
    """A clean, well-formed marker candidate on the strip centre line."""
    base = dict(
        u=160.0,
        v=100.0,
        area_px=452.0,
        circularity=0.92,
        equiv_diameter_px=24.0,
        hue_deg=100.0,
        sat=60.0,
        val=45.0,
        fill_ratio=0.97,
        lateral_offset_px=0.0,
        lateral_reliable=True,
        contrast=140.0,
    )
    base.update(overrides)
    return MarkerCand(**base)


# --------------------------------------------------------------------------- #
# Control
# --------------------------------------------------------------------------- #


class PidTest(unittest.TestCase):
    def test_output_is_clamped_to_the_gain_set(self) -> None:
        pid = Pid(GainSet(kp=1000.0, kd=0.0, ki=0.0, clamp_us=20.0))
        self.assertAlmostEqual(pid.step(0.5, 0.0), 20.0)
        pid = Pid(GainSet(kp=1000.0, kd=0.0, ki=0.0, clamp_us=20.0))
        self.assertAlmostEqual(pid.step(-0.5, 0.0), -20.0)

    def test_derivative_uses_measured_time(self) -> None:
        """A wrong dt silently rescales the D term; it must come from the clock."""
        gains = GainSet(kp=0.0, kd=100.0, ki=0.0, clamp_us=1000.0)
        slow = Pid(gains)
        slow.step(0.0, 0.0)
        slow_out = slow.step(1.0, 0.10)
        fast = Pid(gains)
        fast.step(0.0, 0.0)
        fast_out = fast.step(1.0, 0.01)
        self.assertAlmostEqual(slow_out, 1000.0)  # 100 * 1.0 / 0.10, then clamped
        self.assertAlmostEqual(fast_out, 1000.0)  # 100 * 1.0 / 0.01, also clamped
        # Use small errors so neither clamps, and compare.
        slow = Pid(gains)
        slow.step(0.0, 0.0)
        self.assertAlmostEqual(slow.step(0.001, 0.10), 1.0, places=6)
        fast = Pid(gains)
        fast.step(0.0, 0.0)
        self.assertAlmostEqual(fast.step(0.001, 0.01), 10.0, places=6)

    def test_time_glitch_is_ignored(self) -> None:
        """A camera hiccup must not produce a huge derivative spike."""
        pid = Pid(GainSet(kp=0.0, kd=100.0, ki=0.0, clamp_us=1000.0))
        pid.step(0.0, 0.0)
        self.assertAlmostEqual(pid.step(0.001, 5.0), 0.0)

    def test_hold_clears_derivative_history(self) -> None:
        pid = Pid(GainSet(kp=0.0, kd=100.0, ki=0.0, clamp_us=1000.0))
        pid.step(0.0, 0.0)
        pid.hold(0.05)
        self.assertAlmostEqual(pid.step(1.0, 0.10), 0.0)


class LineControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = Config()
        self.controller = LineController(self.cfg.control, self.cfg.channels)

    def test_pure_pursuit_heading_matches_geometry(self) -> None:
        line = make_line(e_y_px=45.0, lookahead_px=90.0)
        expected = math.atan2(45.0, 90.0)
        self.assertAlmostEqual(self.controller.desired_heading(line), expected, places=6)

    def test_desired_heading_is_bounded(self) -> None:
        line = make_line(e_y_px=500.0, lookahead_px=10.0)
        self.assertAlmostEqual(
            abs(self.controller.desired_heading(line)), self.cfg.control.max_heading_cmd_rad, places=6
        )

    def test_steering_sign_follows_the_strip(self) -> None:
        """A strip to the vehicle's right must produce a right turn command."""
        left_of_centre = make_line(e_y_px=-40.0)
        right_of_centre = make_line(e_y_px=+40.0)
        a = self.controller.step(left_of_centre, 1500, 0.1, 0.1)
        self.controller.reset()
        b = self.controller.step(right_of_centre, 1500, 0.1, 0.1)
        self.assertLess(a.yaw_us, self.cfg.channels.neutral)
        self.assertGreater(b.yaw_us, self.cfg.channels.neutral)

    def test_heading_error_sign_is_negative_feedback(self) -> None:
        """Regression: e_theta is strip-relative, so the loop must not invert.

        A strip that heads to the vehicle's right (positive ``e_theta``) means the
        vehicle is pointing left of the strip and must turn right.  The
        obvious-looking ``e_theta_d - e_theta`` inverts this and turns the heading
        loop into positive feedback; it is invisible when ``e_theta`` is zero,
        which is why the lateral-offset test above does not catch it.
        """
        strip_heads_right = make_line(e_y_px=0.0, e_theta_rad=+0.20)
        strip_heads_left = make_line(e_y_px=0.0, e_theta_rad=-0.20)
        self.assertGreater(self.controller.heading_error(strip_heads_right), 0.0)
        self.assertLess(self.controller.heading_error(strip_heads_left), 0.0)

        self.controller.reset()
        a = self.controller.step(strip_heads_right, 1500, 0.1, 0.1)
        self.controller.reset()
        b = self.controller.step(strip_heads_left, 1500, 0.1, 0.1)
        self.assertGreater(a.yaw_us, self.cfg.channels.neutral)
        self.assertLess(b.yaw_us, self.cfg.channels.neutral)

    def test_heading_error_combines_offset_and_heading(self) -> None:
        """Both cues must push the same way when they agree."""
        both_right = make_line(e_y_px=+40.0, e_theta_rad=+0.10)
        offset_only = make_line(e_y_px=+40.0, e_theta_rad=0.0)
        self.assertGreater(
            self.controller.heading_error(both_right), self.controller.heading_error(offset_only)
        )

    def test_invalid_line_never_reads_as_centred(self) -> None:
        """e_y = 0 on an invalid observation must not look like perfect alignment."""
        # Establish a strong correction first.
        self.controller.step(make_line(e_y_px=60.0), 1500, 0.1, 0.1)
        committed = self.controller.steer_offset_us
        self.assertGreater(committed, 0.0)
        actuator = self.controller.step(make_line(valid=False, e_y_px=0.0), 1500, 0.2, 0.1)
        self.assertAlmostEqual(actuator.yaw_offset_us, committed, places=6)

    def test_forward_is_slew_limited(self) -> None:
        cfg = Config()
        cfg.control.forward_slew_us_per_s = 60.0
        cfg.control.forward_align = 1500
        controller = LineController(cfg.control, cfg.channels)
        first = controller.step(make_line(), 1600, 0.1, 0.1)
        self.assertAlmostEqual(first.forward_us, 1506, delta=1)
        # Enough time and it reaches the target.
        now = 0.1
        for _ in range(200):
            now += 0.1
            last = controller.step(make_line(), 1600, now, 0.1)
        self.assertEqual(last.forward_us, 1600)

    def test_yaw_override_bypasses_the_loop_and_is_bounded(self) -> None:
        cfg = Config()
        cfg.control.corner_turn_us = 1000.0  # far beyond the corner clamp
        controller = LineController(cfg.control, cfg.channels)
        actuator = controller.step(make_line(e_y_px=-99.0), 1500, 0.1, 0.1, corner=True, yaw_override_us=1000.0)
        self.assertAlmostEqual(actuator.yaw_offset_us, cfg.control.yaw_corner.clamp_us, places=6)

    def test_channel_map_can_omit_heave(self) -> None:
        """A two-channel tail-fin vehicle must work with heave unmapped."""
        cfg = Config()
        cfg.channels.heave = None
        controller = LineController(cfg.control, cfg.channels)
        actuator = controller.step(make_line(), 1560, 0.1, 0.1)
        self.assertNotIn(3, actuator.channels)
        self.assertIn(cfg.channels.yaw, actuator.channels)
        self.assertIn(cfg.channels.forward, actuator.channels)

    def test_output_stays_inside_the_pwm_range(self) -> None:
        cfg = Config()
        cfg.channels.neutral = 1095  # absurdly close to the lower bound
        controller = LineController(cfg.control, cfg.channels)
        actuator = controller.step(make_line(e_y_px=-200.0), 1095, 0.1, 0.1)
        for pwm in actuator.channels.values():
            self.assertGreaterEqual(pwm, cfg.channels.pwm_min)
            self.assertLessEqual(pwm, cfg.channels.pwm_max)


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #


class CandidateScoreTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cal = Config().calibration

    def test_a_clean_marker_scores_high(self) -> None:
        self.assertGreater(candidate_score(make_cand(), self.cal), 0.9)

    def test_an_irregular_shape_is_pulled_down(self) -> None:
        self.assertLess(candidate_score(make_cand(circularity=0.56), self.cal), 0.3)

    def test_a_wrong_size_is_pulled_down(self) -> None:
        self.assertLess(candidate_score(make_cand(equiv_diameter_px=90.0), self.cal), 0.5)

    def test_unreliable_lateral_is_not_treated_as_centred(self) -> None:
        centred = candidate_score(make_cand(lateral_offset_px=0.0, lateral_reliable=True), self.cal)
        unknown = candidate_score(make_cand(lateral_offset_px=0.0, lateral_reliable=False), self.cal)
        self.assertLess(unknown, centred)
        self.assertAlmostEqual(unknown, 0.5)


class ReporterTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = Config()
        self.reporter = Reporter(self.cfg.report, self.cfg.marker, self.cfg.calibration)
        self.trigger_row = self.cfg.calibration.y_trigger_row

    def _feed(self, rows: list[float], now0: float = 0.0, dt: float = 0.05, **cand_overrides):
        events = []
        for index, row in enumerate(rows):
            cand = make_cand(v=row, **cand_overrides)
            events.extend(self.reporter.update([cand], now0 + index * dt))
        return events

    def test_needs_temporal_confirmation(self) -> None:
        cfg = Config()
        cfg.report.confirm_frames = 5
        reporter = Reporter(cfg.report, cfg.marker, cfg.calibration)
        rows = [self.trigger_row - 6 + i for i in range(3)]  # only 3 sightings, all before the row
        events = []
        for index, row in enumerate(rows):
            events.extend(reporter.update([make_cand(v=row)], index * 0.05))
        self.assertEqual(events, [])

    def test_reports_once_the_marker_reaches_the_trigger_row(self) -> None:
        rows = [self.trigger_row - 12 + i * 2 for i in range(10)]
        events = self._feed(rows)
        self.assertEqual(len(events), 1)
        self.assertGreaterEqual(events[0].row, self.trigger_row - 1e-6)
        self.assertEqual(events[0].sequence, 1)

    def test_never_reports_the_same_track_twice(self) -> None:
        rows = [self.trigger_row - 10 + i for i in range(30)]
        events = self._feed(rows)
        self.assertEqual(len(events), 1)
        self.assertGreater(self.reporter.stats.rejected_duplicate, 0)

    def test_a_stationary_candidate_is_not_an_approach(self) -> None:
        """A blob that never moves toward the vehicle is not a marker passing under."""
        events = self._feed([self.trigger_row] * 10)
        self.assertEqual(events, [])

    def test_tape_zone_suppresses_candidates(self) -> None:
        cross = CrossObs(detected=True, row=self.trigger_row, width_px=320.0, row_lo=self.trigger_row - 8, row_hi=self.trigger_row + 8)
        rows = [self.trigger_row - 12 + i * 2 for i in range(10)]
        events = []
        for index, row in enumerate(rows):
            events.extend(self.reporter.update([make_cand(v=row)], index * 0.05, cross))
        self.assertEqual(events, [])
        self.assertGreater(self.reporter.stats.rejected_tape_zone, 0)

    def test_discriminator_can_reject_everything(self) -> None:
        cfg = Config()
        cfg.marker = MarkerParams(discriminator="hue", hue_band_deg=(0.0, 1.0))
        reporter = Reporter(cfg.report, cfg.marker, cfg.calibration)
        rows = [self.trigger_row - 12 + i * 2 for i in range(10)]
        events = []
        for index, row in enumerate(rows):
            events.extend(reporter.update([make_cand(v=row, hue_deg=100.0)], index * 0.05))
        self.assertEqual(events, [])
        self.assertGreater(reporter.stats.rejected_discriminator, 0)

    def test_threshold_rises_after_the_detection_cap(self) -> None:
        """Past the detection cap the 40-point ceiling is spent, so the bar must go up.

        ``max_reports`` is lowered here so the rule can be tested without
        fighting the association radius: two markers placed far apart in the
        frame must become two tracks.
        """
        cfg = Config()
        cfg.report.max_reports = 2
        reporter = Reporter(cfg.report, cfg.marker, cfg.calibration)
        self.assertAlmostEqual(reporter.threshold, cfg.report.min_confidence)
        for index, u in enumerate((60.0, 260.0)):
            rows = [self.trigger_row - 6 + i for i in range(8)]
            for step, row in enumerate(rows):
                reporter.update([make_cand(u=u, v=row)], step * 0.05)
            self.assertEqual(reporter.reports_emitted, index + 1)
        self.assertEqual(reporter.reports_emitted, cfg.report.max_reports)
        self.assertAlmostEqual(reporter.threshold, cfg.report.saturated_confidence)

    def test_a_new_marker_still_forms_its_own_track_after_a_report(self) -> None:
        """Reported tracks must not swallow later markers that appear far away."""
        rows = [self.trigger_row - 6 + i for i in range(8)]
        for step, row in enumerate(rows):
            self.reporter.update([make_cand(u=160.0, v=row)], step * 0.05)
        self.assertEqual(self.reporter.reports_emitted, 1)
        # A new marker enters at the top of the frame, far from the reported one.
        for step, row in enumerate(range(5, 40, 5)):
            self.reporter.update([make_cand(u=160.0, v=float(row))], 1.0 + step * 0.05)
        self.assertGreater(self.reporter.stats.tracks_created, 1)

    def test_stats_expose_where_candidates_were_rejected(self) -> None:
        """A missed leak must be distinguishable from a leak that was rejected."""
        # Enough sightings to pass confirmation, all short of the trigger row.
        self._feed([self.trigger_row - 10 + i for i in range(6)])
        stats = self.reporter.stats.as_dict()
        self.assertGreater(stats["candidates"], 0)
        self.assertGreater(stats["rejected_not_yet_coincident"], 0)
        self.assertEqual(stats["reports"], 0)


# --------------------------------------------------------------------------- #
# State machine and terminations
# --------------------------------------------------------------------------- #


class SafetyMonitorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = Config()
        self.monitor = SafetyMonitor(self.cfg.safety, self.cfg.channels)

    def test_nothing_fires_before_the_start_command(self) -> None:
        self.assertIsNone(
            self.monitor.update(line_valid=False, lateral_norm=0.0, commanded_forward_us=1600, now_s=999.0)
        )

    def test_off_track_timeout(self) -> None:
        self.monitor.arm(0.0)
        reason = None
        for step in range(1000):
            now = step * 0.1
            reason = self.monitor.update(
                line_valid=False, lateral_norm=0.0, commanded_forward_us=1560, now_s=now
            )
            if reason:
                break
        self.assertEqual(reason, AbortReason.OFF_TRACK_TIMEOUT)
        self.assertAlmostEqual(now, self.cfg.safety.off_track_timeout_s, delta=0.2)

    def test_start_timeout_when_never_moving(self) -> None:
        self.monitor.arm(0.0)
        reason = None
        for step in range(1000):
            now = step * 0.1
            reason = self.monitor.update(
                line_valid=True, lateral_norm=0.0, commanded_forward_us=self.cfg.channels.neutral, now_s=now
            )
            if reason:
                break
        self.assertEqual(reason, AbortReason.START_TIMEOUT)

    def test_moving_clears_the_start_timeout(self) -> None:
        self.monitor.arm(0.0)
        for step in range(100):
            self.monitor.update(
                line_valid=True, lateral_norm=0.0, commanded_forward_us=1560, now_s=step * 0.1
            )
        self.assertTrue(self.monitor.started)

    def test_run_timeout(self) -> None:
        self.monitor.arm(0.0)
        reason = self.monitor.update(
            line_valid=True, lateral_norm=0.0, commanded_forward_us=1560, now_s=self.cfg.safety.run_timeout_s
        )
        self.assertEqual(reason, AbortReason.RUN_TIMEOUT)

    def test_wall_hits_count_episodes_not_frames(self) -> None:
        """One sustained saturation is one contact, not one per frame."""
        self.monitor.arm(0.0)
        now = 0.0
        # Three episodes of sustained saturation, separated by recovery.
        for _ in range(3):
            for _ in range(20):
                now += 0.1
                self.monitor.update(
                    line_valid=True,
                    lateral_norm=0.9,
                    commanded_forward_us=1560,
                    now_s=now,
                )
            for _ in range(5):
                now += 0.1
                reason = self.monitor.update(
                    line_valid=True,
                    lateral_norm=0.0,
                    commanded_forward_us=1560,
                    now_s=now,
                )
                if reason:
                    break
            if reason:
                break
        self.assertEqual(reason, AbortReason.WALL_HITS)
        self.assertEqual(self.monitor.wall_hits, self.cfg.safety.wall_hit_limit)


class StateMachineTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = Config()
        self.machine = StateMachine(self.cfg.fsm, self.cfg.safety, self.cfg.control, self.cfg.channels)

    def test_starts_in_idle_and_needs_a_start_command(self) -> None:
        decision = self.machine.step(make_line(), None, 0.0)
        self.assertIs(decision.state, State.IDLE)
        self.machine.start(0.0)
        # An unaligned strip keeps it in ALIGN; an aligned one proceeds at once.
        unaligned = self.machine.step(make_line(e_theta_rad=0.5), None, 0.0)
        self.assertIs(unaligned.state, State.ALIGN)

    def test_align_completes_when_the_strip_is_centred(self) -> None:
        self.machine.start(0.0)
        self.assertIs(self.machine.step(make_line(), None, 0.0).state, State.FOLLOW)

    def test_align_gives_up_rather_than_missing_the_start_window(self) -> None:
        self.machine.start(0.0)
        decision = self.machine.step(
            make_line(e_theta_rad=0.5), None, self.cfg.fsm.align_timeout_s
        )
        self.assertIs(decision.state, State.FOLLOW)

    def test_corner_needs_confirmation(self) -> None:
        self.machine.start(0.0)
        self.machine.step(make_line(), None, 0.0)
        bendy = make_line(confidence=0.2)
        for step in range(1, self.cfg.fsm.corner_confirm_frames):
            decision = self.machine.step(bendy, None, step * 0.05)
            self.assertIs(decision.state, State.FOLLOW)
        decision = self.machine.step(bendy, None, self.cfg.fsm.corner_confirm_frames * 0.05)
        self.assertIs(decision.state, State.CORNER)

    def test_corner_commits_to_a_bounded_turn(self) -> None:
        self.machine.start(0.0)
        self.machine.step(make_line(), None, 0.0)
        bendy = make_line(confidence=0.2)
        for step in range(1, self.cfg.fsm.corner_confirm_frames + 1):
            decision = self.machine.step(bendy, None, step * 0.05)
        self.assertIs(decision.state, State.CORNER)
        self.assertIsNotNone(decision.yaw_override_us)
        self.assertGreater(decision.yaw_override_us, 0.0)
        self.assertEqual(decision.target_forward_us, self.cfg.control.forward_corner)

    def test_corner_escalates_to_recovery_and_keeps_turning(self) -> None:
        """With no heading source the bend searches, reverses, then hands over.

        The committed turn must survive the hand-over: dropping it when the state
        changes would leave the vehicle stopped part way round a corner and
        driving at the wall.
        """
        self.machine.start(0.0)
        self.machine.step(make_line(), None, 0.0)
        # A bend that stays un-reacquired: confidence low and a large heading
        # error, so the re-sighting cue cannot end it.
        bendy = make_line(confidence=0.2, e_theta_rad=0.9)
        now = 0.0
        decision = None
        for _ in range(self.cfg.fsm.corner_confirm_frames + 1):
            now += 0.05
            decision = self.machine.step(bendy, None, now)
        self.assertIs(decision.state, State.CORNER)

        saw_reversal = False
        for _ in range(400):
            now += 0.05
            decision = self.machine.step(bendy, None, now)
            if "reversed" in decision.note:
                saw_reversal = True
            if decision.state is State.LINE_LOST:
                break
        self.assertTrue(saw_reversal, "the search direction was never reversed")
        self.assertIs(decision.state, State.LINE_LOST)
        self.assertIsNotNone(decision.yaw_override_us)

    def test_losing_the_strip_enters_recovery(self) -> None:
        self.machine.start(0.0)
        self.machine.step(make_line(), None, 0.0)
        decision = self.machine.step(make_line(valid=False), None, 0.1)
        self.assertIs(decision.state, State.LINE_LOST)
        self.assertIsNone(decision.yaw_override_us)  # not from a corner

    def test_reacquiring_the_strip_returns_to_following(self) -> None:
        self.machine.start(0.0)
        self.machine.step(make_line(), None, 0.0)
        self.machine.step(make_line(valid=False), None, 0.1)
        self.assertIs(self.machine.step(make_line(), None, 0.2).state, State.FOLLOW)

    def test_a_start_tape_is_not_accepted_as_the_finish(self) -> None:
        """Invariant 6: the finish line needs state context, not just a sighting."""
        self.machine.start(0.0)
        self.machine.step(make_line(), None, 0.0)
        cross = CrossObs(detected=True, row=100.0, width_px=320.0, row_lo=95.0, row_hi=105.0)
        decision = self.machine.step(make_line(), cross, 0.5)
        self.assertIs(decision.state, State.FOLLOW)

    def test_finish_creep_then_done(self) -> None:
        self.machine.start(0.0)
        self.machine.step(make_line(), None, 0.0)
        cross = CrossObs(detected=True, row=100.0, width_px=320.0, row_lo=95.0, row_hi=105.0)
        later = self.cfg.fsm.min_run_time_for_finish_s + 0.1
        decision = self.machine.step(make_line(), cross, later)
        self.assertIs(decision.state, State.FINISH_APPR)
        self.assertFalse(decision.terminal)
        decision = self.machine.step(make_line(), cross, later + self.cfg.fsm.finish_creep_s + 0.01)
        self.assertIs(decision.state, State.DONE)
        self.assertTrue(decision.terminal)

    def test_terminal_states_are_sticky(self) -> None:
        self.machine.start(0.0)
        self.machine.step(make_line(), None, 0.0)
        self.machine.step(make_line(valid=False), None, 0.1)
        decision = self.machine.step(
            make_line(valid=False), None, self.cfg.safety.off_track_timeout_s + 1.0
        )
        self.assertIs(decision.state, State.ABORT)
        self.assertEqual(decision.abort, AbortReason.OFF_TRACK_TIMEOUT)
        after = self.machine.step(make_line(), None, self.cfg.safety.off_track_timeout_s + 5.0)
        self.assertIs(after.state, State.ABORT)

    def test_abort_command_is_neutral(self) -> None:
        self.machine.start(0.0)
        self.machine.step(make_line(), None, 0.0)
        self.machine.step(make_line(valid=False), None, 0.1)
        decision = self.machine.step(
            make_line(valid=False), None, self.cfg.safety.off_track_timeout_s + 1.0
        )
        self.assertEqual(decision.target_forward_us, self.cfg.channels.neutral)


# --------------------------------------------------------------------------- #
# Signalling
# --------------------------------------------------------------------------- #


class AnnunciatorTest(unittest.TestCase):
    def _annunciator(self, **overrides):
        params = AnnunciatorParams(**overrides)
        gpio = RecordingGpio()
        speaker = LoggingSpeaker()
        return Annunciator(params, gpio=gpio, speaker=speaker), gpio, speaker

    def test_light_goes_on_then_off_without_blocking(self) -> None:
        from bionic_fish.reporting import ReportEvent

        ann, gpio, _ = self._annunciator(light_hold_s=0.5, buzzer_enabled=False)
        ann.on_report(ReportEvent(track_id=1, sequence=1, row=290.0, confidence=0.9, timestamp_s=0.0), 0.0)
        self.assertTrue(any(pin == 23 and on for _, pin, on in gpio.events))
        # Still on before the hold expires.
        ann.tick(0.4)
        self.assertFalse(any(pin == 23 and not on for _, pin, on in gpio.events))
        ann.tick(0.6)
        self.assertTrue(any(pin == 23 and not on for _, pin, on in gpio.events))

    def test_one_announcement_per_report(self) -> None:
        from bionic_fish.reporting import ReportEvent

        ann, _, speaker = self._annunciator()
        for index in range(3):
            ann.on_report(
                ReportEvent(track_id=index, sequence=index + 1, row=290.0, confidence=0.9, timestamp_s=0.0),
                float(index),
            )
        self.assertEqual(speaker.spoken, ["第 1 点", "第 2 点", "第 3 点"])

    def test_speech_can_be_disabled_without_disabling_the_light(self) -> None:
        from bionic_fish.reporting import ReportEvent

        ann, gpio, speaker = self._annunciator(speech_enabled=False)
        ann.on_report(ReportEvent(track_id=1, sequence=1, row=290.0, confidence=0.9, timestamp_s=0.0), 0.0)
        self.assertEqual(speaker.spoken, [])
        self.assertTrue(any(on for _, _, on in gpio.events))


class SimLinkTest(unittest.TestCase):
    def test_records_frames_and_arming(self) -> None:
        link = SimLink()
        self.assertTrue(link.arm())
        self.assertTrue(link.armed)
        link.send(Actuator(channels={4: 1520}, yaw_us=1520))
        self.assertEqual(link.last.channels, {4: 1520})
        link.disarm()
        self.assertFalse(link.armed)
        link.close()
        self.assertTrue(link.closed)


if __name__ == "__main__":
    unittest.main()
