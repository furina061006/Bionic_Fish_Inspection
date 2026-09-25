"""Behavioural tests for the refactored perception stack.

Run with::

    PYTHONPATH=src python -m unittest discover -s tests -v

There is no camera on the development machine, so every test drives the
synthetic arena in :mod:`bionic_fish.arena`.  The arena's floor is drawn at a
known millimetre scale and its camera is an exact nadir projection, so the
expected pixel position of any feature follows analytically from the calibration
rather than from a recorded expectation.  That is what makes these tests
meaningful without real footage.

Three of these tests are regressions for defects found while building the
simulator and the tracker:

* ``test_specular_highlights_are_bright`` - ``cv2.circle`` assigns rather than
  adds, so the highlights were originally stamped in as dark discs, which is the
  same signature as a leak marker;
* ``test_strip_centre_is_unbiased`` - ``StripTracker`` cropped the row a second
  time inside ``_row_run``, shifting every centre by the ROI offset;
* ``test_config_validate_rejects_*`` - misconfiguration must fail at load time.
"""

from __future__ import annotations

import dataclasses
import math
import unittest

import cv2
import numpy as np

from bionic_fish.arena import (
    Arena,
    ArenaSpec,
    DegradeSpec,
    MARKER_DIAMETER_MM,
    ground_truth_offset_px,
)
from bionic_fish.config import Config, StripParams, load_config
from bionic_fish.vision import CrossLineDetector, MarkerDetector, StripTracker


class ArenaGeometryTest(unittest.TestCase):
    """The synthetic arena must match the competition document's geometry."""

    def test_path_is_the_inset_polyline(self) -> None:
        spec = ArenaSpec()
        pts = spec.path_points()
        self.assertEqual(pts, [(500.0, 500.0), (2500.0, 500.0), (2500.0, 1500.0), (500.0, 1500.0)])
        # 2000 + 1000 + 2000.  The document's prose says "about 6 m"; the two
        # figures disagree and the diagrams are not to scale, so the geometry is
        # what the renderer follows.
        self.assertEqual(spec.path_length_mm, 5000.0)

    def test_marker_counts_and_spacing(self) -> None:
        spec = ArenaSpec()
        layout = spec.marker_layout()
        kinds = [k for k, _, _ in layout]
        self.assertEqual(kinds.count("leak"), 8)
        self.assertEqual(kinds.count("distractor"), 5)

        arc = sorted(s for _, s in spec.marker_arc_lengths())
        gaps = np.diff(arc)
        self.assertGreaterEqual(float(gaps.min()), spec.min_marker_spacing_mm)
        # Markers must also stay inside the path, clear of either end.
        self.assertGreaterEqual(arc[0], 0.0)
        self.assertLessEqual(arc[-1], spec.path_length_mm)

    def test_layout_is_deterministic(self) -> None:
        self.assertEqual(ArenaSpec().marker_layout(), ArenaSpec().marker_layout())

    def test_layout_raises_when_spacing_is_impossible(self) -> None:
        with self.assertRaises(RuntimeError):
            ArenaSpec(min_marker_spacing_mm=500.0).marker_layout()


class ArenaCameraTest(unittest.TestCase):
    """The nadir projection must place features where geometry says they are."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.cfg = load_config()
        cls.cal = cls.cfg.calibration
        cls.clean = Arena(calibration=cls.cal, degrade=DegradeSpec(enabled=False))

    def test_tonal_footprint_and_pixel_scale(self) -> None:
        self.assertAlmostEqual(self.cal.mm_per_px, 1.25, places=6)
        self.assertAlmostEqual(self.cal.marker_diameter_px, 24.0, places=6)
        w, h = self.cal.footprint_mm
        self.assertAlmostEqual(w, 400.0, places=6)
        self.assertAlmostEqual(h, 400.0, places=6)

    def test_strip_appears_at_the_predicted_column(self) -> None:
        """A strip parallel to the heading must be a vertical band at the centre."""
        for dy in (0.0, 30.0, -30.0, 60.0):
            frame = self.clean.view(1400.0, 500.0 + dy, 0.0)
            grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            bright = np.where(grey.mean(axis=0) > 200)[0]
            self.assertGreater(bright.size, 0, f"no strip found for dy={dy}")
            centre = (bright.min() + bright.max()) / 2.0
            self.assertAlmostEqual(centre, 160.0 - dy / self.cal.mm_per_px, delta=1.0)

    def test_marker_pixel_diameter_follows_scale(self) -> None:
        kind, mx, my = next(m for m in self.clean.markers if m[0] == "leak")
        frame = self.clean.view(mx, my, 0.0)
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dark = (grey < grey.mean() - 40).astype(np.uint8)
        n, _, stats, _ = cv2.connectedComponentsWithStats(dark, 8)
        areas = [stats[i, cv2.CC_STAT_AREA] for i in range(1, n)]
        self.assertTrue(areas, "the marker was not rendered")
        area = max(areas)
        diameter = math.sqrt(4.0 * area / math.pi)
        expected = MARKER_DIAMETER_MM / self.cal.mm_per_px
        self.assertAlmostEqual(diameter, expected, delta=expected * 0.35)

    def test_specular_highlights_only_add_light(self) -> None:
        """Regression: highlights must add light, not assign a dark disc.

        The comparison is between two renders that differ *only* in the specular
        layer, so the assertion holds regardless of what else is in frame
        (markers are legitimately dark).
        """
        without = Arena(calibration=self.cal, degrade=DegradeSpec(specular_count=0))
        with_spec = Arena(calibration=self.cal, degrade=DegradeSpec(specular_count=2))
        f0 = without.view(1400.0, 500.0, 0.0).astype(np.int32)
        f1 = with_spec.view(1400.0, 500.0, 0.0).astype(np.int32)
        # ``cv2.circle`` assigns, so accidentally drawing the glow onto the
        # scene would darken pixels; that is what this guards against.
        self.assertGreaterEqual(int((f0 - f1).max()), -2, "the specular layer darkened the scene")
        self.assertGreater(int((f1 - f0).sum()), 0, "no light was added")

    def test_ground_truth_offset_sign(self) -> None:
        (x, y), heading = ArenaSpec().point_at(1000.0)
        # Offsetting the vehicle to its right must put the strip to its left.
        right = 30.0
        ox, oy = -math.sin(heading) * right, math.cos(heading) * right
        self.assertLess(ground_truth_offset_px(self.clean, x + ox, y + oy, heading), 0.0)


class StripTrackerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.cfg = load_config()
        cls.arena = Arena(calibration=cls.cfg.calibration)
        cls.tracker = StripTracker(cls.cfg.strip, cls.cfg.calibration)

    def test_centre_is_unbiased_across_lateral_offsets(self) -> None:
        """Regression: the ROI offset must not leak into the reported centre."""
        (x, y), heading = self.arena.spec.point_at(1000.0)
        errors = []
        for lateral in (-40.0, -20.0, -10.0, 0.0, 10.0, 20.0, 40.0):
            ox, oy = -math.sin(heading) * lateral, math.cos(heading) * lateral
            obs = self.tracker.observe(self.arena.view(x + ox, y + oy, heading))
            self.assertTrue(obs.valid, f"strip lost at lateral={lateral}")
            truth = ground_truth_offset_px(self.arena, x + ox, y + oy, heading)
            errors.append(obs.e_y_px - truth)
        self.assertLess(abs(float(np.mean(errors))), 1.5, f"bias {np.mean(errors):.2f} px")
        self.assertLess(float(np.max(np.abs(errors))), 3.0, f"errors {errors}")

    def test_heading_error_sign_and_magnitude(self) -> None:
        (x, y), heading = self.arena.spec.point_at(1000.0)
        for deg in (-12.0, -5.0, 0.0, 5.0, 12.0):
            obs = self.tracker.observe(self.arena.view(x, y, heading + math.radians(deg)))
            self.assertTrue(obs.valid)
            # Positive yaw swings the vehicle clockwise in arena coordinates,
            # which points the strip to its left, so e_theta is -deg.
            self.assertAlmostEqual(math.degrees(obs.e_theta_rad), -deg, delta=1.0)

    def test_never_invalid_on_a_clean_straight(self) -> None:
        for s in range(600, 1900, 200):
            (px, py), hd = self.arena.spec.point_at(float(s))
            obs = self.tracker.observe(self.arena.view(px, py, hd))
            self.assertTrue(obs.valid, f"strip reported invalid at s={s}")
            self.assertEqual(obs.support_rows > 50, True, f"weak support at s={s}")

    def test_strip_width_matches_calibration(self) -> None:
        obs = self.tracker.observe(self.arena.view(1400.0, 500.0, 0.0))
        expected = self.cfg.calibration.strip_width_px
        self.assertAlmostEqual(obs.strip_width_px, expected, delta=expected * 0.3)

    def test_corner_shows_a_cue(self) -> None:
        """Approaching a 90 degree bend, confidence must fall or curvature rise."""
        straight = self.tracker.observe(self.arena.view(1400.0, 500.0, 0.0))
        near_corner = self.tracker.observe(self.arena.view(2480.0, 500.0, 0.0))
        curvature_cue = abs(near_corner.curvature_rad_per_row) > abs(straight.curvature_rad_per_row) * 5.0
        confidence_cue = near_corner.confidence < straight.confidence * 0.75
        self.assertTrue(
            curvature_cue or confidence_cue,
            f"no corner cue: straight conf={straight.confidence:.2f} curv={straight.curvature_rad_per_row:.5f}, "
            f"corner conf={near_corner.confidence:.2f} curv={near_corner.curvature_rad_per_row:.5f}",
        )


class MarkerDetectorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.cfg = load_config()
        cls.arena = Arena(calibration=cls.cfg.calibration)
        cls.tracker = StripTracker(cls.cfg.strip, cls.cfg.calibration)
        cls.detector = MarkerDetector(cls.cfg.marker, cls.cfg.calibration)

    def test_finds_the_marker_at_its_true_position_and_size(self) -> None:
        kind, mx, my = next(m for m in self.arena.markers if m[0] == "leak")
        # Place the vehicle 100 mm before the marker, on the top straight.
        frame = self.arena.view(mx - 100.0, my, 0.0)
        line = self.tracker.observe(frame)
        cands = self.detector.observe(frame, line)
        self.assertEqual(len(cands), 1, f"expected exactly one marker, got {cands}")
        cand = cands[0]
        self.assertAlmostEqual(cand.u, 160.0, delta=3.0)
        self.assertAlmostEqual(cand.v, 160.0 - 100.0 / self.cfg.calibration.mm_per_px, delta=3.0)
        self.assertAlmostEqual(
            cand.equiv_diameter_px, self.cfg.calibration.marker_diameter_px, delta=6.0
        )
        self.assertGreater(cand.circularity, 0.75)
        self.assertGreater(cand.fill_ratio, 0.85)
        self.assertLess(abs(cand.lateral_offset_px), 3.0)

    def _clear_strip_position(self) -> float:
        """An x on the top straight with no marker inside the camera footprint.

        Chosen from the arena rather than hard-coded, so the test does not
        silently start looking at a marker when the layout seed changes.
        """
        half = self.cfg.calibration.footprint_mm[0] / 2.0 + 40.0
        marker_xs = [x for kind, x, y in self.arena.markers if abs(y - 500.0) < 1.0]
        for x in np.arange(600.0, 2400.0, 20.0):
            if all(abs(x - mx) > half for mx in marker_xs):
                return float(x)
        self.skipTest("no marker-free stretch on the top straight")

    def test_no_candidates_on_bare_strip(self) -> None:
        frame = self.arena.view(self._clear_strip_position(), 500.0, 0.0)
        line = self.tracker.observe(frame)
        self.assertTrue(line.valid)
        self.assertEqual(self.detector.observe(frame, line), [])

    def test_discriminator_none_accepts_everything(self) -> None:
        frame = self.arena.view(839.0, 500.0, 0.0)
        line = self.tracker.observe(frame)
        cands = self.detector.observe(frame, line)
        self.assertTrue(all(self.detector.accepts(c) for c in cands))

    def test_hue_discriminator_can_reject(self) -> None:
        params = dataclasses.replace(self.cfg.marker, discriminator="hue", hue_band_deg=(0.0, 1.0))
        detector = MarkerDetector(params, self.cfg.calibration)
        frame = self.arena.view(839.0, 500.0, 0.0)
        line = self.tracker.observe(frame)
        cands = detector.observe(frame, line)
        self.assertTrue(cands, "detection must still produce candidates")
        self.assertFalse(any(detector.accepts(c) for c in cands))


class CrossLineDetectorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.cfg = load_config()
        cls.arena = Arena(calibration=cls.cfg.calibration)
        cls.tracker = StripTracker(cls.cfg.strip, cls.cfg.calibration)
        cls.detector = CrossLineDetector(cls.cfg.marker, cls.cfg.calibration)

    def _observe(self, x: float, y: float, yaw: float):
        frame = self.arena.view(x, y, yaw)
        return self.detector.observe(frame, self.tracker.observe(frame))

    def test_tape_is_seen_while_still_ahead(self) -> None:
        """The finish tape must be detected before the vehicle reaches it."""
        obs = self._observe(640.0, 1500.0, math.pi)
        self.assertTrue(obs.detected, "finish tape missed 140 mm ahead")
        self.assertLess(obs.row, self.cfg.calibration.frame_h / 2.0)

    def test_tape_is_seen_under_the_vehicle(self) -> None:
        obs = self._observe(500.0, 1500.0, math.pi)
        self.assertTrue(obs.detected)
        self.assertAlmostEqual(obs.row, self.cfg.calibration.frame_h / 2.0, delta=4.0)

    def test_no_false_tape_on_the_top_straight(self) -> None:
        for x in (1200.0, 1600.0, 2000.0):
            self.assertFalse(self._observe(x, 500.0, 0.0).detected, f"false tape at x={x}")


class ConfigValidationTest(unittest.TestCase):
    def test_defaults_are_valid(self) -> None:
        Config().validate()

    def test_rejects_duplicate_channels(self) -> None:
        cfg = Config()
        cfg.channels.yaw = cfg.channels.forward
        with self.assertRaises(ValueError):
            cfg.validate()

    def test_rejects_out_of_range_channel(self) -> None:
        cfg = Config()
        cfg.channels.yaw = 19
        with self.assertRaises(ValueError):
            cfg.validate()

    def test_rejects_inverted_pwm_range(self) -> None:
        cfg = Config()
        cfg.channels.pwm_min = 1600
        with self.assertRaises(ValueError):
            cfg.validate()

    def test_rejects_unknown_strip_method(self) -> None:
        cfg = Config()
        cfg.strip = StripParams(method="nonsense")
        with self.assertRaises(ValueError):
            cfg.validate()

    def test_rejects_align_timeout_at_or_above_start_timeout(self) -> None:
        cfg = Config()
        cfg.fsm.align_timeout_s = cfg.safety.start_timeout_s
        with self.assertRaises(ValueError):
            cfg.validate()

    def test_rejects_unknown_key_in_yaml(self) -> None:
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.yaml"
            path.write_text("strip:\n  not_a_field: 3\n", encoding="utf-8")
            with self.assertRaises(KeyError):
                load_config(path)


if __name__ == "__main__":
    unittest.main()
