"""Deployment-varying configuration for the bionic-fish inspection stack.

Design rule (see `.dsh/agents/invariants.md` #7): no algorithm module may carry a
tunable threshold.  Every number that depends on the pool, the camera, or the
vehicle is declared here and supplied by a YAML file produced by calibration.

Evidence markers used in this file and across the repository:

``FACT``        fixed by the competition document or by a measurement
``DESIGN``      a decision this repository made
``UNVERIFIED``  an assumption that must be measured before the competition
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any

import yaml

# --------------------------------------------------------------------------- #
# Control output mapping
# --------------------------------------------------------------------------- #


@dataclass
class ChannelMap:
    """Which RC channel carries which control axis.

    The defaults reproduce the reference platform's ArduSub layout
    (3=heave, 4=yaw, 5=forward, 9=lights).  A tail-fin fish with only two
    effective axes sets ``heave = None``; nothing else changes.

    UNVERIFIED: the competition vehicle's channel count and assignment are not
    confirmed.  See `.dsh/agents/reference-pipeline.md` section 9.
    """

    heave: int | None = 3
    yaw: int | None = 4
    forward: int | None = 5
    lights: int | None = 9
    #: PWM that commands no motion on any axis.
    neutral: int = 1500
    pwm_min: int = 1100
    pwm_max: int = 1900
    #: Raw values for the illumination channel, if one exists.
    lights_on: int = 1100
    lights_off: int = 1500

    def clamp(self, pwm: float) -> int:
        """Clamp a PWM command into the channel's legal range."""
        return int(max(self.pwm_min, min(self.pwm_max, round(pwm))))


# --------------------------------------------------------------------------- #
# Camera / pixel calibration
# --------------------------------------------------------------------------- #


@dataclass
class Calibration:
    """Camera geometry and the pixel windows derived from it.

    ``mm_per_px`` is computed from the physical mount rather than stored
    directly, so that a measured ``camera_height_m`` propagates into every
    size-based threshold.  Set ``mm_per_px_override`` after a real measurement
    to bypass the model.
    """

    #: Height of the lens above the pool floor.  UNVERIFIED - measure it.
    #: Bounded above by the 260 +/- 20 mm water depth (FACT).
    camera_height_m: float = 0.20
    #: Horizontal field of view.  UNVERIFIED; >= 75 deg is required by the
    #: footprint argument in `.dsh/agents/hardware.md`.
    hfov_deg: float = 90.0
    #: Vertical field of view.  UNVERIFIED.
    vfov_deg: float = 90.0
    frame_w: int = 320
    frame_h: int = 320
    #: Radial distortion coefficient, 0 disables undistortion.
    k1: float = 0.0
    #: Measured scale; ``None`` means "derive from the model above".
    mm_per_px_override: float | None = None

    #: Diameter of both the leak markers and the distractor markers (FACT, p2).
    marker_diameter_mm: float = 30.0
    #: Physical width of the LED strip.  UNVERIFIED - must be measured on site.
    #: Used as a strong shape prior: with a nadir camera over a flat floor the
    #: strip's pixel width is constant across the frame, which is what separates
    #: it from a specular reflection that happens to be bright and unsaturated.
    strip_width_mm: float = 24.0
    #: Valid marker area window, as multiples of the nominal area.
    marker_area_lo_ratio: float = 0.35
    marker_area_hi_ratio: float = 3.0
    #: Valid marker circularity (4*pi*A/P^2) window.
    marker_circularity_min: float = 0.55
    #: Valid marker aspect-ratio ceiling (long side / short side).
    marker_aspect_max: float = 1.8

    #: Frame row, as a fraction of height, at which a tracked marker counts as
    #: coincident with the vehicle and the report fires.  DESIGN: the reference
    #: project uses the same idea (a fixed row threshold), which avoids needing
    #: any odometry.  UNVERIFIED value.
    y_trigger_frac: float = 0.90

    #: Width of the start/finish tape band, in pixels.  UNVERIFIED.
    crossline_width_px_lo: float = 60.0
    crossline_width_px_hi: float = 400.0

    @property
    def mm_per_px(self) -> float:
        """Millimetres of floor per pixel at the image centre."""
        if self.mm_per_px_override is not None:
            return self.mm_per_px_override
        footprint_mm = 2.0 * self.camera_height_m * math.tan(math.radians(self.hfov_deg) / 2.0) * 1000.0
        return footprint_mm / float(self.frame_w)

    @property
    def footprint_mm(self) -> tuple[float, float]:
        """Ground footprint ``(width, height)`` in millimetres."""
        w = 2.0 * self.camera_height_m * math.tan(math.radians(self.hfov_deg) / 2.0) * 1000.0
        h = 2.0 * self.camera_height_m * math.tan(math.radians(self.vfov_deg) / 2.0) * 1000.0
        return w, h

    @property
    def marker_diameter_px(self) -> float:
        """Expected marker diameter in pixels at the floor plane."""
        return self.marker_diameter_mm / self.mm_per_px

    @property
    def strip_width_px(self) -> float:
        """Expected strip width in pixels.

        Constant across the frame: the camera is nadir over a flat floor, so
        there is no perspective foreshortening of the floor plane.
        """
        return self.strip_width_mm / self.mm_per_px

    @property
    def marker_area_px(self) -> tuple[float, float]:
        """Valid marker area window derived from the calibrated scale."""
        nominal = math.pi / 4.0 * self.marker_diameter_px**2
        return nominal * self.marker_area_lo_ratio, nominal * self.marker_area_hi_ratio

    @property
    def y_trigger_row(self) -> float:
        """Frame row at which a marker is considered coincident."""
        return self.y_trigger_frac * self.frame_h


# --------------------------------------------------------------------------- #
# Perception parameters
# --------------------------------------------------------------------------- #


@dataclass
class StripParams:
    """LED-strip segmentation and centre-line extraction.

    Three segmentation methods are implemented because the strip's actual
    appearance is UNVERIFIED:

    ``tophat``          bright *local* structure (default).  Water attenuation
                        tints everything toward one hue and the illumination
                        gradient changes with position, so an absolute
                        brightness/hue test has only a thin margin: in the
                        synthetic arena the strip's saturation rises to ~80
                        under water tint while the floor sits at ~125.  A
                        top-hat compares each pixel with its own neighbourhood,
                        which removes both the tint and the gradient.
    ``bright_low_sat``  absolute high-value, low-saturation test.  Works when
                        the exposure is controlled, but its thresholds must be
                        calibrated on real footage.
    ``red_minus_blue``  the reference project's cue for a red-emitting strip.
    """

    method: str = "tophat"
    #: top-hat kernel size, pixels.  Must exceed the strip width so the whole
    #: strip is suppressed by the opening, and stay below the scale over which
    #: the floor's illumination varies.
    tophat_ksize: int = 31
    #: top-hat response threshold, 0-255.  UNVERIFIED, but far less sensitive
    #: than an absolute threshold because the response is a local difference.
    tophat_threshold: int = 35
    #: bright_low_sat thresholds, on OpenCV's 0-255 HSV scale.  UNVERIFIED.
    v_min: int = 200
    s_max: int = 100
    #: red_minus_blue threshold.  UNVERIFIED.
    r_minus_b_min: int = 10
    #: Region of interest, as frame fractions.  The top band is kept small so
    #: that a look-ahead point exists; the reference project discarded the top
    #: 47% of the frame, which removed all preview.
    roi_top_frac: float = 0.05
    roi_bottom_frac: float = 0.98
    roi_side_frac: float = 0.03
    #: Morphology kernel sizes, in pixels.  ``close_ksize <= 0`` derives a size
    #: that can bridge the hole a marker punches in the strip, because a
    #: partially occluded run has its centre pulled off the true centre line.
    open_ksize: int = 3
    close_ksize: int = 0
    #: Accepted run width, as a multiple of the calibrated strip width.  A tight
    #: window is what rejects a bright specular sheet that merged with the strip:
    #: such a merge produces an over-wide run, and accepting it would bias the
    #: run centre silently.  The lower bound rejects a run truncated by a marker
    #: or by a segmentation dropout, which is narrower than the true strip.  The
    #: floor plane has no perspective, so the true width does not vary with row
    #: and the window can be narrow.
    run_width_ratio: tuple[float, float] = (0.7, 1.6)
    #: Absolute run-width override in pixels; when set it replaces the ratio.
    #: Use only when the strip's physical width is unknown and a measured pixel
    #: width is available instead.
    run_px_lo: float | None = None
    run_px_hi: float | None = None
    #: Prefer, among accepted runs, the one nearest this many pixels from the
    #: previous row's centre.  Keeps the tracked run continuous when a distractor
    #: of similar width is also in view.
    continuity_window_px: float = 40.0
    #: Minimum number of valid rows before a centre line is accepted.
    min_rows: int = 12
    #: Row (frame fraction) at which the look-ahead point is sampled.
    lookahead_frac: float = 0.22
    #: Maximum residual, in pixels, for a row to survive the robust fit.
    fit_residual_px: float = 8.0

    def roi(self, w: int, h: int) -> tuple[int, int, int, int]:
        """Return the ROI as ``(x0, y0, x1, y1)``."""
        x0 = int(round(self.roi_side_frac * w))
        x1 = int(round((1.0 - self.roi_side_frac) * w))
        y0 = int(round(self.roi_top_frac * h))
        y1 = int(round(self.roi_bottom_frac * h))
        return x0, y0, x1, y1


@dataclass
class MarkerParams:
    """Dark-blob detection and the pluggable leak/distractor discrimination.

    The feature that separates the 8 leak markers from the 5 distractor markers
    is **unknown** (see `.dsh/agents/vision.md`).  This class therefore exposes
    the candidate features as configuration rather than assuming one, and
    ``discriminator`` selects the rule:

    ``none``        accept every geometrically valid dark blob (default until
                    calibration says otherwise; expects the report gate to be
                    the only defence)
    ``hue``         accept blobs whose mean hue falls inside a band
    ``pattern``     accept blobs by interior fill ratio
    ``position``    accept blobs by their offset from the strip centre line
    """

    discriminator: str = "none"
    #: Morphological black-hat kernel; should exceed the marker diameter so the
    #: whole disc is suppressed by the closing.
    blackhat_ksize: int = 31
    #: Black-hat response threshold, 0-255.  UNVERIFIED.
    blackhat_threshold: int = 40
    #: Lateral gate: candidate centres must lie within this fraction of the
    #: frame width from the strip centre line.  DESIGN (gate 1).
    max_lateral_offset_frac: float = 0.25
    #: hue band, degrees, for ``discriminator == "hue"``.  UNVERIFIED.
    hue_band_deg: tuple[float, float] = (0.0, 180.0)
    #: interior fill-ratio band for ``discriminator == "pattern"``.  UNVERIFIED.
    fill_ratio_band: tuple[float, float] = (0.0, 1.0)
    #: offset band, as a fraction of frame width, for ``discriminator ==
    #: "position"``.  UNVERIFIED.
    offset_band_frac: tuple[float, float] = (0.0, 1.0)
    #: Minimum vertical speed, in px per frame, for a track to count as
    #: approaching rather than noise.  Gate 2.  UNVERIFIED.
    min_approach_px_per_frame: float = 0.4
    #: A track is dropped after this many frames without an association.
    max_misses: int = 3


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #


@dataclass
class ReportParams:
    """Reporting decision parameters.

    ``min_confidence`` defaults to the break-even probability derived from the
    published scoring: reporting a true leak earns +5 and a false one costs -3,
    so the expected gain ``8p - 3`` is positive only for ``p > 3/8``.
    """

    #: Gate 2: consecutive confirmations required before a track may report.
    confirm_frames: int = 4
    #: Gate 3: a reported track is suppressed for this long.  Deliberately short;
    #: identity, not time, is what prevents double reporting.
    cooldown_s: float = 0.6
    #: Decision threshold on accumulated confidence.
    min_confidence: float = 0.375
    #: Threshold applied once ``max_reports`` reports have been emitted; the
    #: 40-point detection cap means further reports have no upside.
    #: FACT: 8 leak markers, 5 points each, 40 point cap.
    saturated_confidence: float = 0.95
    #: Number of reports after which the saturated threshold applies.
    max_reports: int = 8
    #: How much a single confirming observation adds to a track's confidence.
    confidence_step: float = 0.2
    #: Starting confidence of a newly created track.
    confidence_initial: float = 0.3
    #: Lead compensation applied to the trigger row, in pixels, to cover the
    #: exposure-to-actuation delay.  UNVERIFIED.
    trigger_lead_px: float = 0.0


# --------------------------------------------------------------------------- #
# Control
# --------------------------------------------------------------------------- #


@dataclass
class GainSet:
    """One PD gain set, matching the reference project's structure."""

    kp: float = 200.0
    kd: float = 20.0
    ki: float = 0.0
    #: Half-width of the output clamp, in PWM microseconds.
    clamp_us: float = 20.0


@dataclass
class ControlParams:
    """Cascaded lateral control plus the speed schedule.

    Outer loop: lateral error -> desired heading error.
    Inner loop: heading error -> yaw PWM.

    The reference project drove the yaw PID from the lateral error alone and
    discarded the heading it had already measured; the cascade uses it.
    """

    #: Outer-loop proportional gain, PWM-independent (pure angle command).
    k_lateral: float = 2.0
    #: Ceiling on the desired heading error, radians.
    max_heading_cmd_rad: float = 0.6
    #: Inner loop gains while tracking a straight section.
    yaw_follow: GainSet = field(default_factory=lambda: GainSet(200.0, 20.0, 0.0, 20.0))
    #: Inner loop gains while a corner is being negotiated: deliberately
    #: narrower, as in the reference project, to avoid fighting the turn.
    yaw_corner: GainSet = field(default_factory=lambda: GainSet(120.0, 10.0, 0.0, 45.0))
    #: Heave trim from the visual-servo scale error.
    heave_gain: GainSet = field(default_factory=lambda: GainSet(5.0, 0.0, 0.0, 20.0))
    #: Neutral offset for the heave channel; the reference project sat here.
    heave_trim_us: float = -90.0
    #: Target strip width in pixels for the visual-servo heave loop.  UNVERIFIED.
    target_strip_width_px: float = 34.0

    #: Forward PWM per speed state.  Open loop, as in the reference project;
    #: the values are UNVERIFIED.
    forward_align: int = 1500
    forward_cruise: int = 1560
    forward_corner: int = 1495
    forward_search: int = 1555
    forward_finish: int = 1530
    #: Maximum rate of change of the forward command, in microseconds per
    #: second, so that speed changes are ramps rather than steps.
    forward_slew_us_per_s: float = 60.0

    #: Forward speed assumed when dead-reckoning distance, in mm/s.  UNVERIFIED
    #: and used for logging only; no decision depends on it.
    assumed_speed_mm_per_s: float = 200.0


# --------------------------------------------------------------------------- #
# Safety and the state machine
# --------------------------------------------------------------------------- #


@dataclass
class SafetyParams:
    """Hard termination conditions.

    Every value is a FACT from the competition document (p2, p6).
    """

    #: Total run limit, seconds.
    run_timeout_s: float = 150.0
    #: Being fully off the track for this long terminates the run.
    off_track_timeout_s: float = 10.0
    #: The vehicle must start moving within this long of the start command.
    start_timeout_s: float = 30.0
    #: Touching the floor or a wall this many times ends the round.
    wall_hit_limit: int = 3
    #: A wall hit is inferred when the lateral error stays saturated this long.
    wall_hit_infer_s: float = 0.8
    #: Largest lateral error, as a fraction of frame width, tolerated before it
    #: counts as saturation for the wall-hit inference.
    wall_hit_error_frac: float = 0.42
    #: Speed component, microseconds above neutral, above which the vehicle
    #: counts as having started.
    started_forward_us: int = 5


@dataclass
class FsmParams:
    """State-machine thresholds."""

    #: Heading error, radians, under which ALIGN is considered complete.
    align_heading_tol_rad: float = 0.12
    #: Lateral error, frame fraction, under which ALIGN is considered complete.
    align_lateral_tol: float = 0.10
    #: ALIGN is abandoned and the run starts anyway after this long.  Must be
    #: well below ``SafetyParams.start_timeout_s``.  DESIGN.
    align_timeout_s: float = 6.0
    #: Curvature, radians per row, above which a corner is declared.
    corner_curvature_rad_per_row: float = 0.004
    #: Strip-line confidence below which a corner is suspected.  This is the
    #: primary cue: as the vehicle reaches a 90 degree bend the strip leaves the
    #: frame sideways, so the share of rows yielding a run collapses before the
    #: bend is reached.  Measured on the synthetic arena, confidence falls
    #: monotonically 0.94 -> 0.43 over the last 400 mm before the corner while
    #: curvature spikes roughly 160x.  Using both avoids relying on either alone.
    #: UNVERIFIED against a real pool.
    corner_confidence_min: float = 0.60
    #: Consecutive frames of corner evidence required (the reference project's
    #: three-strike debounce, made explicit).
    corner_confirm_frames: int = 3
    #: Corner negotiation is bounded in time; if the strip is not reacquired by
    #: then, the run enters LINE_LOST.  DESIGN - the reference project had no
    #: bound here.
    corner_timeout_s: float = 4.0
    #: Lost-strip escalation thresholds, seconds.
    lost_degrade_s: float = 0.3
    lost_scan_s: float = 2.0
    #: A cross line is only accepted as the finish after this much of the run.
    min_run_time_for_finish_s: float = 8.0
    #: After the finish line, keep going this far to satisfy "fully inside the
    #: finish zone".  UNVERIFIED - needs the real body length.
    finish_creep_s: float = 1.2
    #: Start/finish zone half-extent as a fraction of frame width; candidate
    #: markers inside it are treated as tape, not leaks (invariant 6).
    cross_zone_lateral_frac: float = 0.5


# --------------------------------------------------------------------------- #
# Root
# --------------------------------------------------------------------------- #


@dataclass
class Config:
    """Root configuration object."""

    channels: ChannelMap = field(default_factory=ChannelMap)
    calibration: Calibration = field(default_factory=Calibration)
    strip: StripParams = field(default_factory=StripParams)
    marker: MarkerParams = field(default_factory=MarkerParams)
    report: ReportParams = field(default_factory=ReportParams)
    control: ControlParams = field(default_factory=ControlParams)
    safety: SafetyParams = field(default_factory=SafetyParams)
    fsm: FsmParams = field(default_factory=FsmParams)

    #: MAVLink connection string, e.g. ``/dev/ttyACM0`` or ``udpin:0.0.0.0:14550``.
    #: FACT: the reference platform uses ``/dev/ttyACM0`` at 115200.
    mavlink_device: str = "/dev/ttyACM0"
    mavlink_baud: int = 115200
    #: Directory for run artefacts.  ``None`` disables frame dumping.
    artefact_dir: str | None = None

    def validate(self) -> None:
        """Reject a self-inconsistent configuration.

        Misconfiguration fails here, at load time, rather than silently
        degrading behaviour in the middle of a run.
        """
        assigned = {
            "heave": self.channels.heave,
            "yaw": self.channels.yaw,
            "forward": self.channels.forward,
            "lights": self.channels.lights,
        }
        used: dict[int, str] = {}
        for axis, channel in assigned.items():
            if channel is None:
                continue
            if not 1 <= channel <= 18:
                raise ValueError(f"channel {axis}={channel} is outside MAVLink's 1..18 range")
            if channel in used:
                raise ValueError(
                    f"channels {used[channel]!r} and {axis!r} both use channel {channel}"
                )
            used[channel] = axis
        if self.channels.yaw is None and self.channels.forward is None:
            raise ValueError("at least one of yaw/forward must be mapped")

        ch = self.channels
        if not ch.pwm_min < ch.neutral < ch.pwm_max:
            raise ValueError(
                f"need pwm_min < neutral < pwm_max, got {ch.pwm_min}/{ch.neutral}/{ch.pwm_max}"
            )

        cal = self.calibration
        if cal.camera_height_m <= 0.0:
            raise ValueError("calibration.camera_height_m must be positive")
        if not 0.0 < cal.hfov_deg < 180.0 or not 0.0 < cal.vfov_deg < 180.0:
            raise ValueError("field of view must be within (0, 180) degrees")
        if cal.frame_w < 16 or cal.frame_h < 16:
            raise ValueError("frame size must be at least 16x16")
        if cal.mm_per_px <= 0.0:
            raise ValueError("derived mm_per_px must be positive")
        if not 0.0 < cal.y_trigger_frac <= 1.0:
            raise ValueError("calibration.y_trigger_frac must be within (0, 1]")

        if self.strip.method not in ("tophat", "bright_low_sat", "red_minus_blue"):
            raise ValueError(f"unknown strip.method {self.strip.method!r}")
        if self.marker.discriminator not in ("none", "hue", "pattern", "position"):
            raise ValueError(f"unknown marker.discriminator {self.marker.discriminator!r}")
        if self.strip.lookahead_frac <= self.strip.roi_top_frac:
            raise ValueError(
                "strip.lookahead_frac must lie inside the ROI "
                f"(roi_top_frac={self.strip.roi_top_frac}, lookahead={self.strip.lookahead_frac})"
            )

        for name in ("min_confidence", "saturated_confidence"):
            value = getattr(self.report, name)
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"report.{name} must be within [0, 1], got {value}")
        if self.report.saturated_confidence < self.report.min_confidence:
            raise ValueError(
                "report.saturated_confidence must not be below report.min_confidence"
            )
        if self.report.confirm_frames < 1:
            raise ValueError("report.confirm_frames must be at least 1")

        if self.safety.wall_hit_limit < 1:
            raise ValueError("safety.wall_hit_limit must be at least 1")
        if self.fsm.align_timeout_s >= self.safety.start_timeout_s:
            raise ValueError(
                "fsm.align_timeout_s must be below safety.start_timeout_s "
                "so the vehicle starts before the start timeout expires"
            )
        if self.fsm.lost_scan_s >= self.safety.off_track_timeout_s:
            raise ValueError(
                "fsm.lost_scan_s must be below safety.off_track_timeout_s "
                "so the search finishes before the off-track timeout"
            )


def _apply(target: Any, data: dict[str, Any], path: str = "") -> None:
    """Recursively assign mapping values onto a dataclass instance."""
    known = {f.name: f for f in fields(target)}
    for key, value in data.items():
        if key not in known:
            raise KeyError(f"unknown configuration key {path}{key!r}")
        current = getattr(target, key)
        if is_dataclass(current) and isinstance(value, dict):
            _apply(current, value, f"{path}{key}.")
        elif isinstance(value, list) and isinstance(current, (list, tuple)):
            setattr(target, key, tuple(value))
        else:
            setattr(target, key, value)


def load_config(path: str | Path | None = None) -> Config:
    """Build a :class:`Config`, optionally overlaying a YAML file."""
    cfg = Config()
    if path is not None:
        text = Path(path).read_text(encoding="utf-8")
        data = yaml.safe_load(text) or {}
        if not isinstance(data, dict):
            raise TypeError(f"{path}: top level must be a mapping")
        _apply(cfg, data)
    cfg.validate()
    return cfg


def save_config(cfg: Config, path: str | Path) -> None:
    """Write a configuration to YAML.

    Missing directories raise; configuration files are not created implicitly.
    """
    Path(path).write_text(
        yaml.safe_dump(asdict(cfg), sort_keys=True, allow_unicode=True),
        encoding="utf-8",
    )
