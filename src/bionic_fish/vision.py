"""Perception: strip centre line, leak-marker candidates, start/finish tape.

This module answers four questions and nothing else:

1. Where is the LED strip, and which way does it run?  -> :class:`LineObs`
2. Is there a dark disc on or beside it?               -> :class:`MarkerCand`
3. Is the dark band ahead the start/finish tape?       -> :class:`CrossObs`
4. How big is a marker in pixels?                      -> carried on the config

It makes no decision about turning or reporting.  Keeping perception free of
policy is what lets :mod:`bionic_fish.troubleshooting` style diagnosis separate
"did not see it" from "saw it and rejected it".

Departures from the reference project, all deliberate:

* the entire frame height is searched, so a look-ahead row exists; the reference
  zeroed the top 47% of the frame and had no preview at all;
* the strip heading is returned and used, where the reference computed a slope
  and discarded it;
* ``valid`` is explicit; the reference used a centroid of ``None`` and a bare
  ``else`` branch, which made "no strip" and "strip in the centre" easy to
  confuse.

在链路里的位置
    每帧第 2~4 步（共 13 步，见 runtime.Runner._iterate），由 runtime 调用
    上游：相机帧（真机）或 arena 合成帧（开发机）
    下游：reporting.Reporter 吃 MarkerCand；fsm.StateMachine 吃 LineObs / CrossObs
    调用次序有硬依赖：StripTracker 必须先跑，另外两个都要它的 LineObs
    跨帧状态：**无**。三个检测器只持有参数与预计算的形态学核
              —— 所以它们能逐帧单独测试，也能被并行化
    概念：.dsh/agents/vision.md（设计）、runtime-flow.md 第 6.1 节（调用次序）
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np

from .config import Calibration, MarkerParams, StripParams


# --------------------------------------------------------------------------- #
# Observations
# --------------------------------------------------------------------------- #


@dataclass
class LineObs:
    """Where the LED strip is, relative to the vehicle.

    Sign convention for every angular and lateral quantity: **positive means the
    strip lies to the vehicle's right**.  A controller can therefore steer with
    a positive gain without sign gymnastics.
    """

    valid: bool
    #: Look-ahead row used for ``e_y_px``, in frame pixels.
    lookahead_px: float = 0.0
    #: Strip centre x minus image centre x at the look-ahead row, in pixels.
    e_y_px: float = 0.0
    #: ``e_y_px`` normalised by half the frame width, for gain portability.
    e_y_norm: float = 0.0
    #: Angle between the strip's forward direction and the vehicle's forward
    #: direction, radians; positive means the strip heads to the right.
    e_theta_rad: float = 0.0
    #: Rate of change of strip heading with image row, radians per row.
    curvature_rad_per_row: float = 0.0
    #: Median strip width in pixels; feeds the visual-servo heave loop.
    strip_width_px: float = 0.0
    #: Number of rows that contributed a centre point.
    support_rows: int = 0
    #: Fraction of sampled rows that produced a usable run.
    confidence: float = 0.0
    #: Fitted centre line as ``(slope, intercept)`` with ``x = slope*row + intercept``.
    fit: tuple[float, float] = (0.0, 0.0)
    #: Deviation of the strip's far end from the straight-line fit, pixels,
    #: measured over the forward quarter of the ROI using rows that the robust
    #: fit discarded.  Positive means the strip branches to the vehicle's right,
    #: which is the turn direction at a bend.  Zero when there is no bend.
    branch_px: float = 0.0
    #: Sampled centre points ``(row, x)`` for debugging and replay.
    points: tuple[tuple[float, float], ...] = ()

    def strip_x_at(self, row: float) -> float:
        """Fitted strip centre x at a given frame row."""
        slope, intercept = self.fit
        return slope * row + intercept


@dataclass
class MarkerCand:
    """One dark disc-shaped candidate, with the features a rule may use.

    The discriminating feature between the 8 leak markers and the 5 distractor
    markers is unknown, so every plausible feature is carried rather than
    discarded at detection time.
    """

    #: Centroid in frame pixels.
    u: float
    v: float
    area_px: float
    #: ``4*pi*A/P^2``; 1.0 for a perfect disc.
    circularity: float
    #: Equivalent diameter from the area, pixels.
    equiv_diameter_px: float
    #: Mean hue of the disc interior, degrees on OpenCV's 0-180 scale.
    hue_deg: float
    #: Mean saturation and value of the disc interior, 0-255.
    sat: float
    val: float
    #: Fraction of the fitted disc that is actually dark.  A solid marker is
    #: near 1.0; a ring or an outline is lower.
    fill_ratio: float
    #: Signed lateral offset from the fitted strip centre at this row, pixels.
    lateral_offset_px: float
    #: False when the strip was not located, so ``lateral_offset_px`` carries no
    #: information.  A downstream score must not read "no strip" as "centred".
    lateral_reliable: bool
    #: Black-hat response at the centroid, 0-255.
    contrast: float


@dataclass
class CrossObs:
    """Start/finish tape observation."""

    detected: bool
    #: Frame row of the tape centre.
    row: float = 0.0
    #: Width of the dark band across the strip, in pixels.
    width_px: float = 0.0
    #: Inclusive row span occupied by the tape, so that marker candidates
    #: coinciding with it can be rejected as tape rather than reported.
    row_lo: float = 0.0
    row_hi: float = 0.0


# --------------------------------------------------------------------------- #
# Strip tracking
# --------------------------------------------------------------------------- #


class StripTracker:
    """Extract the LED strip centre line from a downward camera frame."""

    def __init__(self, params: StripParams, calibration: Calibration) -> None:
        self.params = params
        self.calibration = calibration
        self._open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (params.open_ksize, params.open_ksize)
        )
        self._close_kernel = self._make_close_kernel()

    def _make_close_kernel(self) -> np.ndarray:
        """Kernel that bridges the gap a marker punches in the strip.

        Leak markers sit *on* the strip, so each one removes a hole roughly one
        marker diameter tall.  Leaving the hole unbridged does not merely lose
        rows: the runs at the hole's edges are truncated, and a truncated run's
        centre is offset toward the surviving side, which biases the fitted
        centre line.  Bridging the hole is therefore a correctness requirement,
        not cosmetic smoothing.
        """
        size = self.params.close_ksize
        if size <= 0:
            size = int(round(self.calibration.marker_diameter_px)) + 3
        size = max(3, size | 1)  # force odd so the kernel has a centre
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    # ------------------------------------------------------------- segmentation

    def segment(self, frame: np.ndarray) -> np.ndarray:
        """Return a binary mask of strip pixels over the whole frame."""
        p = self.params
        if p.method == "tophat":
            grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (p.tophat_ksize, p.tophat_ksize)
            )
            response = cv2.morphologyEx(grey, cv2.MORPH_TOPHAT, kernel)
            mask = (response >= p.tophat_threshold).astype(np.uint8) * 255
        elif p.method == "bright_low_sat":
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h, s, v = cv2.split(hsv)
            mask = ((v >= p.v_min) & (s <= p.s_max)).astype(np.uint8) * 255
        elif p.method == "red_minus_blue":
            # The reference project's cue, kept for a red-emitting strip.
            b, g, r = cv2.split(frame.astype(np.int16))
            mask = ((r - b) >= p.r_minus_b_min).astype(np.uint8) * 255
        else:  # pragma: no cover - guarded by Config.validate
            raise ValueError(f"unknown strip method {p.method!r}")

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._close_kernel)
        return mask

    # --------------------------------------------------------------- centre line

    def run_window(self) -> tuple[float, float]:
        """Accepted strip run width in pixels.

        Derived from the strip's physical width unless an explicit pixel window
        is configured, so the threshold traces back to a measurement rather than
        to a magic number.
        """
        p = self.params
        if p.run_px_lo is not None and p.run_px_hi is not None:
            return p.run_px_lo, p.run_px_hi
        expected = self.calibration.strip_width_px
        lo_ratio, hi_ratio = p.run_width_ratio
        return max(2.0, expected * lo_ratio), max(3.0, expected * hi_ratio)

    @staticmethod
    def _row_run(
        row: np.ndarray,
        x0: int,
        lo: float,
        hi: float,
        hint_x: float | None = None,
    ) -> tuple[float, float] | None:
        """Pick the run of set pixels in one ROI row that is most likely the strip.

        ``row`` must already be cropped to the ROI; ``x0`` is the absolute frame
        column of ``row[0]`` and is added back to report positions in frame
        coordinates.  Cropping here as well as at the caller would shift every
        centre by ``x0``.

        Runs narrower than ``lo`` are noise; runs wider than ``hi`` are a
        specular sheet or a corner elbow seen end-on.  Among the survivors the
        run nearest ``hint_x`` - the centre found one step closer to the vehicle
        - wins, so tracking stays continuous when a second feature of similar
        width is also in view.  Without a hint the longest run wins.
        """
        if not row.any():
            return None
        padded = np.concatenate(([0], row, [0]))
        edges = np.flatnonzero(np.diff(padded.astype(np.int8)))
        if edges.size < 2:
            return None
        starts, ends = edges[0::2], edges[1::2]
        lengths = (ends - starts).astype(np.float64)
        keep = (lengths >= lo) & (lengths <= hi)
        if not keep.any():
            return None
        centres = x0 + (starts + ends - 1) / 2.0
        if hint_x is None:
            best = int(np.argmax(np.where(keep, lengths, -1.0)))
        else:
            distance = np.where(keep, np.abs(centres - hint_x), np.inf)
            best = int(np.argmin(distance))
        return float(centres[best]), float(lengths[best])

    def observe(self, frame: np.ndarray) -> LineObs:
        """Locate the strip centre line and return a :class:`LineObs`."""
        p = self.params
        h, w = frame.shape[:2]
        x0, y0, x1, y1 = p.roi(w, h)
        mask = self.segment(frame)
        sub = mask[y0:y1, x0:x1]
        w_lo, w_hi = self.run_window()

        rows: list[float] = []
        xs: list[float] = []
        widths: list[float] = []
        step = 2
        hint: float | None = None
        for r in range(sub.shape[0] - 1, -1, -step):
            found = self._row_run(sub[r], x0, w_lo, w_hi, hint)
            if found is None:
                continue
            cx, length = found
            # Refuse a jump: keeping the previous centre is better than locking
            # onto a distractor of similar width.
            if hint is not None and abs(cx - hint) > p.continuity_window_px:
                continue
            hint = cx
            rows.append(float(y0 + r))
            xs.append(cx)
            widths.append(length)

        sampled = max(1, len(range(sub.shape[0] - 1, -1, -step)))
        if len(rows) < p.min_rows:
            return LineObs(valid=False, confidence=len(rows) / sampled)

        row_arr = np.asarray(rows)
        x_arr = np.asarray(xs)

        # Robust straight fit with one rejection pass.  A single line is the
        # right model: at a corner the fit is rejected by the curvature check
        # rather than by bending the line.
        slope, intercept = self._fit_line(row_arr, x_arr)
        residual = np.abs(x_arr - (slope * row_arr + intercept))
        keep = residual <= p.fit_residual_px
        if keep.sum() < p.min_rows:
            return LineObs(valid=False, confidence=len(rows) / sampled)
        slope, intercept = self._fit_line(row_arr[keep], x_arr[keep])
        width_px = float(np.median(np.asarray(widths)[keep]))

        # Look-ahead point: a row near the top of the ROI, i.e. ahead of the
        # vehicle.  This is the pure-pursuit target.
        lookahead_row = p.lookahead_frac * h
        strip_x = slope * lookahead_row + intercept
        e_y_px = strip_x - w / 2.0
        e_theta = -math.atan(slope)  # slope is dx/drow; row grows toward the vehicle

        # Curvature from the difference in heading between the far and near
        # thirds.  A 90 degree corner shows up as a large value here.
        near = row_arr[keep] > (y0 + y1) / 2.0
        curvature = 0.0
        if near.any() and (~near).any():
            s_near, _ = self._fit_line(row_arr[keep][near], x_arr[keep][near])
            s_far, _ = self._fit_line(row_arr[keep][~near], x_arr[keep][~near])
            gap = float(abs(np.median(row_arr[keep][~near]) - np.median(row_arr[keep][near])))
            if gap > 1.0:
                curvature = (math.atan(s_far) - math.atan(s_near)) / gap

        # Branch cue: how far the strip's far end departs from the straight-line
        # fit, using every row found rather than only the rows the robust fit
        # kept.  A bend is exactly the case the robust fit throws away, so the
        # discarded rows are the signal.
        branch_px = 0.0
        far_limit = y0 + 0.25 * (y1 - y0)
        far = row_arr <= far_limit
        if far.sum() >= 3:
            mean_row = float(row_arr[far].mean())
            branch_px = float(x_arr[far].mean() - (slope * mean_row + intercept))

        return LineObs(
            valid=True,
            # Forward distance from the vehicle (the image centre row) to the
            # look-ahead point, in pixels.  This is the pure-pursuit horizon.
            lookahead_px=max(1.0, h / 2.0 - lookahead_row),
            e_y_px=float(e_y_px),
            e_y_norm=float(e_y_px / (w / 2.0)),
            e_theta_rad=float(e_theta),
            curvature_rad_per_row=float(curvature),
            strip_width_px=width_px,
            support_rows=int(keep.sum()),
            confidence=float(keep.sum() / sampled),
            fit=(float(slope), float(intercept)),
            branch_px=branch_px,
            points=tuple(zip(row_arr[keep].tolist(), x_arr[keep].tolist())),
        )

    @staticmethod
    def _fit_line(rows: np.ndarray, xs: np.ndarray) -> tuple[float, float]:
        """Least-squares fit of ``x = slope*row + intercept``."""
        if rows.size < 2 or float(np.ptp(rows)) == 0.0:
            return 0.0, float(xs.mean()) if xs.size else 0.0
        slope, intercept = np.polyfit(rows, xs, 1)
        return float(slope), float(intercept)


# --------------------------------------------------------------------------- #
# Marker detection
# --------------------------------------------------------------------------- #


class MarkerDetector:
    """Find dark disc-shaped candidates and expose their discriminating features."""

    def __init__(self, params: MarkerParams, calibration: Calibration) -> None:
        self.params = params
        self.calibration = calibration
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (params.blackhat_ksize, params.blackhat_ksize)
        )

    def blackhat(self, frame: np.ndarray) -> np.ndarray:
        """Dark-blob response: bright features smaller than the kernel vanish."""
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.morphologyEx(grey, cv2.MORPH_BLACKHAT, self._kernel)

    def observe(self, frame: np.ndarray, line: LineObs) -> list[MarkerCand]:
        """Return geometrically valid candidates, gated against the strip line."""
        p = self.params
        cal = self.calibration
        response = self.blackhat(frame)
        binary = (response >= p.blackhat_threshold).astype(np.uint8)
        n, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, 8)
        if n <= 1:
            return []

        area_lo, area_hi = cal.marker_area_px
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        max_offset = p.max_lateral_offset_frac * cal.frame_w
        out: list[MarkerCand] = []

        for i in range(1, n):
            area = float(stats[i, cv2.CC_STAT_AREA])
            if not (area_lo <= area <= area_hi):
                continue

            component = (labels == i).astype(np.uint8)
            contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0.0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < cal.marker_circularity_min:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if bh == 0 or bw == 0:
                continue
            aspect = max(bw, bh) / float(min(bw, bh))
            if aspect > cal.marker_aspect_max:
                continue

            u, v = float(centroids[i][0]), float(centroids[i][1])

            # Gate 1: the candidate must lie near the strip centre line.
            lateral = u - line.strip_x_at(v) if line.valid else 0.0
            if line.valid and abs(lateral) > max_offset:
                continue
            equiv = math.sqrt(4.0 * area / math.pi)
            radius = max(1, int(round(equiv / 2.0)))
            disc = np.zeros_like(component)
            cv2.circle(disc, (int(round(u)), int(round(v))), radius, 1, -1)
            disc_px = disc.astype(bool)
            fill_ratio = float(component[disc_px].mean()) if disc_px.any() else 0.0

            # Interior colour, eroded so the disc edge does not dominate.
            inner = cv2.erode(disc, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
            inner_px = inner.astype(bool)
            if inner_px.any():
                hue = float(hsv[:, :, 0][inner_px].mean())
                sat = float(hsv[:, :, 1][inner_px].mean())
                val = float(hsv[:, :, 2][inner_px].mean())
            else:
                hue = sat = val = 0.0

            out.append(
                MarkerCand(
                    u=u,
                    v=v,
                    area_px=area,
                    circularity=float(circularity),
                    equiv_diameter_px=float(equiv),
                    hue_deg=hue,
                    sat=sat,
                    val=val,
                    fill_ratio=fill_ratio,
                    lateral_offset_px=float(lateral),
                    lateral_reliable=bool(line.valid),
                    contrast=float(response[int(round(v)), int(round(u))]),
                )
            )
        return out

    def accepts(self, cand: MarkerCand) -> bool:
        """Apply the configured leak/distractor rule."""
        return accepts_marker(cand, self.params, self.calibration)


def accepts_marker(cand: MarkerCand, params: MarkerParams, calibration: Calibration) -> bool:
    """Decide whether a candidate is a leak marker rather than a distractor.

    The separating feature is unknown, so the rule is a configuration choice
    rather than an assumption baked into detection.  With
    ``discriminator == "none"`` nothing is rejected here and the report gate
    carries the whole burden.
    """
    if params.discriminator == "none":
        return True
    if params.discriminator == "hue":
        lo, hi = params.hue_band_deg
        return lo <= cand.hue_deg <= hi
    if params.discriminator == "pattern":
        lo, hi = params.fill_ratio_band
        return lo <= cand.fill_ratio <= hi
    if params.discriminator == "position":
        lo, hi = params.offset_band_frac
        if not cand.lateral_reliable:
            return False
        frac = abs(cand.lateral_offset_px) / calibration.frame_w
        return lo <= frac <= hi
    if params.discriminator == "darkness":
        lo, hi = params.val_band
        return lo <= cand.val <= hi
    return True  # pragma: no cover - guarded by Config.validate


# --------------------------------------------------------------------------- #
# Start / finish tape
# --------------------------------------------------------------------------- #


class CrossLineDetector:
    """Detect the black tape that crosses the strip at the start and finish.

    The tape is the highest-risk false marker in the whole task: it is black,
    wider than a marker, and guaranteed to appear.  It is therefore detected
    explicitly and reported with its own observation type, so the report gate can
    give it priority (invariant 6) instead of relying on a size window.

    Detection uses absolute darkness, not a black-hat response.  A black-hat
    response fires on the edges of bright structures as well as on dark blobs,
    so beside the strip it reports the strip's own border as a wide dark band;
    at a corner that artefact is indistinguishable from a tape crossing.  A grey
    threshold avoids it, because the tape is much darker than both the bright
    strip and the painted floor.
    """

    def __init__(self, params: MarkerParams, calibration: Calibration) -> None:
        self.params = params
        self.calibration = calibration

    def observe(self, frame: np.ndarray, line: LineObs) -> CrossObs:
        """Return the tape crossing nearest the vehicle.

        The whole frame is scanned, not just the near field: the finish tape must
        be seen while it is still *ahead*, which puts it in the upper half of a
        downward view.  The returned ``row`` therefore also tells the caller
        whether the tape is still ahead (``row < frame_h / 2``) or already under
        the vehicle.
        """
        cal = self.calibration
        if not line.valid:
            # A crossing is defined relative to the strip, so without the strip
            # there is nothing to cross.  Falling back to the frame centre makes
            # any dark region that happens to straddle the middle a "tape", which
            # is exactly how the out-of-pool region produced a false finish when
            # the vehicle drifted near a wall with the strip out of view.
            return CrossObs(detected=False)

        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dark = grey <= cal.tape_max_grey

        best: tuple[float, float] | None = None
        best_distance = float("inf")
        passing: dict[int, float] = {}
        for row in range(cal.frame_h):
            centre_x = line.strip_x_at(row)
            col = int(min(max(round(centre_x), 0), cal.frame_w - 1))
            if not dark[row, col]:
                continue
            left = col
            while left > 0 and dark[row, left - 1]:
                left -= 1
            right = col
            while right < cal.frame_w - 1 and dark[row, right + 1]:
                right += 1
            width = float(right - left + 1)
            if not (cal.crossline_width_px_lo <= width <= cal.crossline_width_px_hi):
                continue
            # The tape must straddle the strip on both sides, not sit on one
            # side of it.  A leak marker is dark too, but it is a disc about one
            # diameter wide and fails this and the width test.
            margin = cal.strip_width_px
            if not (left <= centre_x - margin and right >= centre_x + margin):
                continue
            passing[row] = width
            distance = abs(row - cal.frame_h / 2.0)
            if distance < best_distance:
                best_distance = distance
                best = (float(row), width)
        if best is None:
            return CrossObs(detected=False)

        # Expand around the nearest crossing to report the tape's full row span,
        # which the reporter uses to reject marker candidates that are really
        # tape.  A bare row index would leave the tape's own thickness exposed.
        centre_row = int(best[0])
        row_lo = centre_row
        while (row_lo - 1) in passing:
            row_lo -= 1
        row_hi = centre_row
        while (row_hi + 1) in passing:
            row_hi += 1

        # A tape is an isolated band a few pixels thick.  A dark *area* - the
        # region beyond the pool wall, which the vehicle reaches whenever it
        # overshoots a bend - spans far more rows, and must not read as tape.
        if (row_hi - row_lo + 1) > cal.crossline_thickness_px_max:
            return CrossObs(detected=False)

        # The band must also be interior.  A dark region clipped by the frame edge
        # can be only a few rows tall while still being an area, not a band, and
        # that is how the out-of-pool region produced a false finish: driving near
        # the wall, the top rows of the frame are outside the pool.  Beyond a real
        # tape there is always floor or strip on both sides along the direction of
        # travel, so a genuine crossing never touches the border.
        if row_lo <= 0 or row_hi >= cal.frame_h - 1:
            return CrossObs(detected=False)

        return CrossObs(
            detected=True,
            row=best[0],
            width_px=best[1],
            row_lo=float(row_lo),
            row_hi=float(row_hi),
        )


def summarise_candidates(cands: Sequence[MarkerCand]) -> str:
    """Compact one-line description for logs."""
    if not cands:
        return "no candidates"
    return ", ".join(
        f"(u={c.u:.0f},v={c.v:.0f},d={c.equiv_diameter_px:.1f},fill={c.fill_ratio:.2f},"
        f"hue={c.hue_deg:.0f},lat={c.lateral_offset_px:+.0f})"
        for c in cands
    )
