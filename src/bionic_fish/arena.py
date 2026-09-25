"""Synthetic arena: a geometrically faithful stand-in for the competition pool.

There is no camera on the development machine and no recorded underwater video,
so every perception and control test in this repository runs against a rendered
arena.  The renderer is deliberately physical rather than decorative: the floor
is drawn at a known millimetre scale, markers are drawn at their true 30 mm
diameter, and the camera view is produced by warping the floor through the
nadir-camera geometry.  As a result the pixel size of a marker in a synthetic
frame follows from the same numbers a real camera would obey, which is what
makes threshold tests meaningful.

Every arena dimension is a FACT from the competition document (p3) unless marked
otherwise.  Model limitations are listed on :class:`Arena`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterator

import cv2
import numpy as np

from .config import Calibration

# --------------------------------------------------------------------------- #
# Arena geometry (FACT, competition document p3)
# --------------------------------------------------------------------------- #

POOL_LENGTH_MM = 3000.0
POOL_WIDTH_MM = 2000.0
POOL_HEIGHT_MM = 600.0
WATER_DEPTH_MM = 260.0
STRIP_INSET_MM = 500.0
MARKER_DIAMETER_MM = 30.0
LEAK_COUNT = 8
DISTRACTOR_COUNT = 5


@dataclass
class ArenaSpec:
    """Geometry and appearance of the synthetic arena.

    The strip inset and marker diameter are FACT.  Colours, the strip width, the
    tape width, and the marker placement seed are DESIGN choices for the
    renderer; the real values are UNVERIFIED.
    """

    pool_length_mm: float = POOL_LENGTH_MM
    pool_width_mm: float = POOL_WIDTH_MM
    strip_inset_mm: float = STRIP_INSET_MM
    #: Width of the LED strip itself.  UNVERIFIED; the real strip is a physical
    #: object whose width must be measured on site.
    strip_width_mm: float = 24.0
    marker_diameter_mm: float = MARKER_DIAMETER_MM
    tape_width_mm: float = 18.0
    n_leak: int = LEAK_COUNT
    n_distractor: int = DISTRACTOR_COUNT
    #: Lateral scatter of marker centres about the strip centre line, in mm.
    #: UNVERIFIED; zero keeps them centred as the arena diagram shows.
    marker_lateral_mm: float = 0.0
    #: Minimum spacing between adjacent markers along the path, in mm.  FACT:
    #: the competition does not state one, so the renderer enforces a spacing
    #: that keeps the test cases unambiguous.
    min_marker_spacing_mm: float = 260.0
    seed: int = 7

    # Appearance.  BGR order throughout.
    floor_bgr: tuple[int, int, int] = (200, 175, 130)
    strip_bgr: tuple[int, int, int] = (250, 250, 245)
    leak_bgr: tuple[int, int, int] = (18, 18, 20)
    #: Distractor appearance is UNKNOWN.  The default renders a mid-grey disc
    #: with a lighter ring; change it to exercise a different hypothesis
    #: without touching any detector code.
    distractor_bgr: tuple[int, int, int] = (90, 95, 100)
    distractor_ring_bgr: tuple[int, int, int] = (225, 225, 225)
    tape_bgr: tuple[int, int, int] = (30, 30, 32)
    outside_bgr: tuple[int, int, int] = (70, 60, 50)

    def path_points(self) -> list[tuple[float, float]]:
        """Return the strip centre line as a polyline in arena millimetres.

        Arena coordinates put the origin at the pool corner, x along the 3000 mm
        side, y along the 2000 mm side, so the layout matches the top view in
        the competition document.
        """
        near = self.strip_inset_mm
        far_x = self.pool_length_mm - self.strip_inset_mm
        far_y = self.pool_width_mm - self.strip_inset_mm
        return [(near, near), (far_x, near), (far_x, far_y), (near, far_y)]

    def segments(self) -> list[tuple[tuple[float, float], tuple[float, float], float]]:
        """Return ``(start, end, length_mm)`` for each straight run."""
        pts = self.path_points()
        out = []
        for a, b in zip(pts, pts[1:]):
            out.append((a, b, math.dist(a, b)))
        return out

    @property
    def path_length_mm(self) -> float:
        """Geometric path length.

        The competition document states the path is about 6 m.  The polyline
        implied by the 500 mm inset is 5000 mm; the two figures disagree and the
        document's diagrams are not to scale, so this property reports the
        geometry, not the prose.  See the discrepancy note in
        `.dsh/agents/competition.md`.
        """
        return sum(seg[2] for seg in self.segments())

    def point_at(self, s_mm: float) -> tuple[tuple[float, float], float]:
        """Return ``((x, y), heading_rad)`` at arc length ``s_mm`` along the path."""
        remaining = max(0.0, s_mm)
        for (a, b, length) in self.segments():
            if remaining <= length or length == 0.0:
                t = 0.0 if length == 0.0 else remaining / length
                x = a[0] + (b[0] - a[0]) * t
                y = a[1] + (b[1] - a[1]) * t
                heading = math.atan2(b[1] - a[1], b[0] - a[0])
                return (x, y), heading
            remaining -= length
        last = self.path_points()[-1]
        a, b, _ = self.segments()[-1]
        return last, math.atan2(b[1] - a[1], b[0] - a[0])

    def start_zone(self) -> tuple[float, float, float, float]:
        """Start zone as ``(x0, y0, x1, y1)``, the upper-left 500 mm square."""
        return (0.0, 0.0, self.strip_inset_mm, self.pool_width_mm / 2.0)

    def finish_zone(self) -> tuple[float, float, float, float]:
        """Finish zone as ``(x0, y0, x1, y1)``, the lower-left 500 mm square."""
        return (0.0, self.pool_width_mm / 2.0, self.strip_inset_mm, self.pool_width_mm)

    def arc_length_of(self, x_mm: float, y_mm: float) -> float:
        """Arc length of the path point nearest to ``(x_mm, y_mm)``.

        Used to sort markers along the route and to record where a report
        happened.  This is geometry, not an odometry estimate: the vehicle does
        not know its own arc length, only the arena does.
        """
        acc = 0.0
        best_s = 0.0
        best_d = float("inf")
        for (a, b, length) in self.segments():
            ax, ay = a
            dx, dy = b[0] - ax, b[1] - ay
            denom = dx * dx + dy * dy
            t = 0.0 if denom == 0.0 else ((x_mm - ax) * dx + (y_mm - ay) * dy) / denom
            t = min(1.0, max(0.0, t))
            px, py = ax + t * dx, ay + t * dy
            distance = math.dist((x_mm, y_mm), (px, py))
            if distance < best_d:
                best_d = distance
                best_s = acc + t * length
            acc += length
        return best_s

    def marker_arc_lengths(self) -> list[tuple[str, float]]:
        """Return ``[(kind, arc_length_mm)]`` sorted along the route."""
        return sorted(
            ((kind, self.arc_length_of(x, y)) for kind, x, y in self.marker_layout()),
            key=lambda item: item[1],
        )

    def marker_layout(self) -> list[tuple[str, float, float]]:
        """Return ``[(kind, x_mm, y_mm)]`` for every marker, deterministically.

        ``kind`` is ``"leak"`` or ``"distractor"``.  Positions are drawn along
        the path with the configured minimum spacing, using a seeded generator so
        that a test failure is reproducible.
        """
        total = self.path_length_mm
        rng = np.random.default_rng(self.seed)
        count = self.n_leak + self.n_distractor
        # Reserve a margin at both ends so no marker sits on a start/finish tape.
        margin = self.strip_inset_mm * 0.8
        usable = total - 2.0 * margin
        slot = usable / count
        if slot < self.min_marker_spacing_mm:
            raise RuntimeError(
                f"cannot place {count} markers at least {self.min_marker_spacing_mm:.0f} mm apart "
                f"on a {total:.0f} mm path: usable {usable:.0f} mm gives {slot:.0f} mm per slot"
            )
        # One marker per equal slot, jittered within the slack.  Giving each
        # marker its own slot makes the spacing guarantee structural rather than
        # probabilistic: for any draw, adjacent centres are at least
        # ``slot - slack == min_marker_spacing_mm`` apart.
        slack = slot - self.min_marker_spacing_mm
        jitter = rng.uniform(0.0, slack, size=count) if slack > 0.0 else np.zeros(count)
        offsets = margin + np.arange(count) * slot + jitter

        kinds = ["leak"] * self.n_leak + ["distractor"] * self.n_distractor
        rng.shuffle(kinds)

        out: list[tuple[str, float, float]] = []
        for kind, s in zip(kinds, offsets):
            (x, y), heading = self.point_at(float(s))
            if self.marker_lateral_mm:
                lateral = float(rng.uniform(-1.0, 1.0)) * self.marker_lateral_mm
                x += -math.sin(heading) * lateral
                y += math.cos(heading) * lateral
            out.append((kind, float(x), float(y)))
        return out


@dataclass
class DegradeSpec:
    """Sensor degradation applied to a rendered view.

    These parameters exist so that a detector can be tested against something
    worse than a clean drawing.  None of them is calibrated to the real pool;
    they are DESIGN placeholders until real footage exists.
    """

    blur_sigma: float = 0.8
    #: Multiplicative per-channel gain modelling wavelength-dependent
    #: absorption.  BGR order; blue survives, red is attenuated.
    water_gain_bgr: tuple[float, float, float] = (1.0, 0.88, 0.72)
    #: Additive veiling light, modelling backscatter lifting the black level.
    veiling_bgr: tuple[float, float, float] = (26.0, 24.0, 20.0)
    #: Peak-to-peak amplitude of a smooth illumination gradient.
    illumination_amplitude: float = 0.22
    #: Gaussian read noise standard deviation, in 8-bit levels.
    noise_sigma: float = 3.0
    #: Number of specular highlight blobs drawn on the surface.
    specular_count: int = 2
    specular_radius_px: int = 14
    specular_gain: float = 70.0
    enabled: bool = True


class Arena:
    """A rendered pool with a camera that can be placed anywhere on it.

    Model limitations, all of which must be revisited before trusting a
    synthetic result as evidence about the real pool:

    * the floor is treated as a plane and the camera as exactly nadir, so the
      floor-to-image map is affine; real tilt, lens distortion, and the water
      surface are not modelled;
    * water is modelled as a per-channel gain plus veiling light, with no
      scattering point-spread, no refraction, and no caustics;
    * marker appearance, strip colour, and strip width are guesses.
    """

    #: Millimetres of floor per pixel in the rendered floor image.  Chosen finer
    #: than any expected camera scale so the camera view is a genuine
    #: downsampling rather than an upsample.
    FLOOR_MM_PER_PX = 1.0

    def __init__(
        self,
        spec: ArenaSpec | None = None,
        calibration: Calibration | None = None,
        degrade: DegradeSpec | None = None,
    ) -> None:
        self.spec = spec or ArenaSpec()
        self.calibration = calibration or Calibration()
        self.degrade = degrade or DegradeSpec()
        self._floor: np.ndarray | None = None
        self._markers = self.spec.marker_layout()

    # ---------------------------------------------------------------- rendering

    @property
    def markers(self) -> list[tuple[str, float, float]]:
        """The placed markers as ``[(kind, x_mm, y_mm)]``."""
        return list(self._markers)

    @property
    def floor_px_per_mm(self) -> float:
        return 1.0 / self.FLOOR_MM_PER_PX

    def render_floor(self) -> np.ndarray:
        """Render the top-down floor once and cache it.

        Returns a BGR image whose pixel ``(i, j)`` corresponds to arena
        millimetres ``(j * FLOOR_MM_PER_PX, i * FLOOR_MM_PER_PX)``.
        """
        if self._floor is not None:
            return self._floor

        spec = self.spec
        w = int(round(spec.pool_length_mm / self.FLOOR_MM_PER_PX))
        h = int(round(spec.pool_width_mm / self.FLOOR_MM_PER_PX))
        floor = np.zeros((h, w, 3), np.uint8)
        floor[:] = spec.floor_bgr

        px = self.floor_px_per_mm

        # LED strip: a polyline of constant physical width.
        pts = np.array(
            [[int(round(x * px)), int(round(y * px))] for x, y in spec.path_points()],
            np.int32,
        )
        cv2.polylines(
            floor,
            [pts],
            False,
            spec.strip_bgr,
            max(1, int(round(spec.strip_width_mm * px))),
            cv2.LINE_8,
        )

        # Start/finish tape (FACT: black tape marks the zone boundary).  The
        # vertical tape separates both zones from the track; the horizontal tape
        # separates the start zone from the finish zone.
        tape = max(1, int(round(spec.tape_width_mm * px)))
        x_tape = int(round(spec.strip_inset_mm * px))
        cv2.line(floor, (x_tape, 0), (x_tape, h - 1), spec.tape_bgr, tape)
        y_tape = int(round(spec.pool_width_mm / 2.0 * px))
        cv2.line(floor, (0, y_tape), (x_tape, y_tape), spec.tape_bgr, tape)

        # Markers.
        radius = max(1, int(round(spec.marker_diameter_mm / 2.0 * px)))
        for kind, x_mm, y_mm in self._markers:
            centre = (int(round(x_mm * px)), int(round(y_mm * px)))
            if kind == "leak":
                cv2.circle(floor, centre, radius, spec.leak_bgr, -1, cv2.LINE_AA)
            else:
                cv2.circle(floor, centre, radius, spec.distractor_bgr, -1, cv2.LINE_AA)
                ring = max(1, radius // 3)
                cv2.circle(floor, centre, ring, spec.distractor_ring_bgr, -1, cv2.LINE_AA)

        self._floor = floor
        return floor

    # ------------------------------------------------------------------ camera

    def floor_to_image_affine(self, x_mm: float, y_mm: float, yaw_rad: float) -> np.ndarray:
        """Return the affine map from camera pixels to floor-image pixels.

        The camera is nadir at ``calibration.camera_height_m`` above the floor,
        with the image's top edge pointing along the vehicle's heading.  For a
        flat floor and a nadir view this map is exactly affine, so no
        perspective term is dropped.
        """
        cal = self.calibration
        mpp = cal.mm_per_px
        fppm = self.floor_px_per_mm
        w, h = cal.frame_w, cal.frame_h
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)

        m = np.zeros((2, 3), np.float64)
        m[0, 0] = -fppm * mpp * s
        m[0, 1] = -fppm * mpp * c
        m[1, 0] = fppm * mpp * c
        m[1, 1] = -fppm * mpp * s
        m[0, 2] = fppm * (x_mm + (h / 2.0) * mpp * c + (w / 2.0) * mpp * s)
        m[1, 2] = fppm * (y_mm + (h / 2.0) * mpp * s - (w / 2.0) * mpp * c)
        return m

    def view(self, x_mm: float, y_mm: float, yaw_rad: float) -> np.ndarray:
        """Render the downward camera view for a vehicle pose.

        ``yaw_rad`` is the heading in arena coordinates, measured from the +x
        axis with +y downward, matching the top view.
        """
        cal = self.calibration
        floor = self.render_floor()
        m = self.floor_to_image_affine(x_mm, y_mm, yaw_rad)
        # ``floor_to_image_affine`` returns the *sampling* map, dst pixel ->
        # source pixel.  ``cv2.warpAffine`` inverts its matrix by default, so the
        # inverse-map flag is required to use the map in the direction it was
        # built.  Without it the frame is sampled through the inverse transform.
        frame = cv2.warpAffine(
            floor,
            m,
            (cal.frame_w, cal.frame_h),
            flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=self.spec.outside_bgr,
        )
        return self._degrade(frame)

    def _degrade(self, frame: np.ndarray) -> np.ndarray:
        """Apply the configured sensor degradation."""
        deg = self.degrade
        if not deg.enabled:
            return frame
        out = frame.astype(np.float32)

        out *= np.array(deg.water_gain_bgr, np.float32).reshape(1, 1, 3)
        out += np.array(deg.veiling_bgr, np.float32).reshape(1, 1, 3)

        if deg.illumination_amplitude > 0.0:
            h, w = out.shape[:2]
            yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
            radius = np.sqrt(((xx - w / 2.0) / (w / 2.0)) ** 2 + ((yy - h / 2.0) / (h / 2.0)) ** 2)
            falloff = 1.0 - deg.illumination_amplitude * np.clip(radius, 0.0, 1.5)
            out *= falloff[:, :, None]

        if deg.specular_count > 0:
            rng = np.random.default_rng(self.spec.seed + 991)
            h, w = out.shape[:2]
            # Highlights must be *added* to the scene, not assigned into it.
            # ``cv2.circle`` assigns, so drawing straight onto ``out`` would
            # stamp dark discs onto the image - the same signature as a leak
            # marker - and every detector test would be measuring that artefact.
            # Draw into a separate layer and add a blurred copy.
            glow = np.zeros_like(out)
            for _ in range(deg.specular_count):
                cx = int(rng.uniform(0, w))
                cy = int(rng.uniform(0, h))
                cv2.circle(glow, (cx, cy), deg.specular_radius_px, (deg.specular_gain,) * 3, -1)
            glow = cv2.GaussianBlur(glow, (0, 0), max(1.0, deg.specular_radius_px / 2.0))
            out += glow

        if deg.blur_sigma > 0.0:
            out = cv2.GaussianBlur(out, (0, 0), deg.blur_sigma)

        if deg.noise_sigma > 0.0:
            rng = np.random.default_rng(self.spec.seed + 17)
            out += rng.normal(0.0, deg.noise_sigma, out.shape).astype(np.float32)

        return np.clip(out, 0, 255).astype(np.uint8)

    # -------------------------------------------------------------- trajectory

    def trajectory(
        self,
        speed_mm_per_s: float,
        dt_s: float,
        lateral_mm: float = 0.0,
        start_s_mm: float = 0.0,
    ) -> Iterator[tuple[float, float, float, float]]:
        """Yield ``(time_s, x_mm, y_mm, yaw_rad)`` along the path.

        ``lateral_mm`` offsets the vehicle to its right, which is how a lateral
        tracking error is injected.  The heading stays parallel to the path, so
        the offset is a pure position error.
        """
        total = self.spec.path_length_mm
        s = start_s_mm
        t = 0.0
        while s <= total:
            (x, y), heading = self.spec.point_at(s)
            ox = -math.sin(heading) * lateral_mm
            oy = math.cos(heading) * lateral_mm
            yield t, x + ox, y + oy, heading
            s += speed_mm_per_s * dt_s
            t += dt_s


def ground_truth_offset_px(arena: Arena, x_mm: float, y_mm: float, yaw_rad: float) -> float:
    """Signed lateral offset of the strip under the vehicle, in camera pixels.

    Positive means the strip centre lies to the vehicle's right in the image,
    which is the sign convention :class:`~bionic_fish.vision.LineObs` uses.
    """
    spec = arena.spec
    pts = spec.path_points()
    best: tuple[float, float] | None = None
    best_d = float("inf")
    for a, b in zip(pts, pts[1:]):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        if length2 == 0.0:
            continue
        t = ((x_mm - ax) * dx + (y_mm - ay) * dy) / length2
        t = min(1.0, max(0.0, t))
        px, py = ax + t * dx, ay + t * dy
        d = math.dist((x_mm, y_mm), (px, py))
        if d < best_d:
            best_d = d
            best = (px, py)
    if best is None:  # pragma: no cover - the spec always has segments
        return 0.0
    # Vector from vehicle to the nearest strip point, projected onto the
    # vehicle's right-hand direction.
    vx, vy = best[0] - x_mm, best[1] - y_mm
    rx, ry = -math.sin(yaw_rad), math.cos(yaw_rad)
    lateral_mm = vx * rx + vy * ry
    return lateral_mm / arena.calibration.mm_per_px
