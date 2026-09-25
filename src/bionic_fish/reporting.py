"""Reporting: track association, the three gates, and the scoring decision.

The reference project fired its target action on the falling edge of a
detection (``cube == False and cube_last == True``) and had no notion of a false
positive: every disappearance produced a light change and a claw motion.  Under
this competition's rules that design is unusable, because a wrong report costs
``-3`` while a correct one earns ``+5`` and the penalty accumulates to ``-24``.

This module therefore separates three concerns the reference merged:

1. **Gate 1, geometry** - applied in :mod:`bionic_fish.vision`: a candidate must
   be disc-shaped, near the calibrated size, and close to the strip centre line.
   The start/finish tape is removed here as well, using the tape's row span
   rather than a size test, because the tape is black and always present
   (invariant 6).
2. **Gate 2, temporal confirmation** - a candidate must be seen on several
   consecutive frames, must be approaching (its image row grows toward the
   vehicle), and must be growing.
3. **Gate 3, identity** - a *track* reports at most once.  Distance windows and
   cooldowns are both wrong here: 13 markers on a 5 m route average about
   460 mm apart, less than the 600 mm vehicle, so a distance window would merge
   two real leaks, while a pure cooldown both suppresses and duplicates as the
   speed varies (invariant 4).

The decision itself is a probability comparison.  With a true-report gain of +5
and a false-report loss of -3, reporting has positive expectation only when
``p > 3/8``.  Confidence is a weakest-link score over the measurable features,
which is deliberately conservative: one disqualifying feature is enough to hold
a candidate back.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .config import Calibration, MarkerParams, ReportParams
from .vision import CrossObs, MarkerCand, accepts_marker


@dataclass
class ReportEvent:
    """The single event that drives every outward signal.

    Light, buzzer, speech and telemetry all subscribe to this one object.  The
    competition scores speech separately and docks 2 points per missing
    announcement, so letting speech decide for itself when to speak would
    eventually drift from what was actually reported (invariant 5).
    """

    track_id: int
    #: 1-based ordinal, used for the spoken index ("point N").
    sequence: int
    row: float
    confidence: float
    timestamp_s: float
    kind: str = "leak"

    @property
    def announcement(self) -> str:
        """Text for the speech channel."""
        return f"第 {self.sequence} 点"


@dataclass
class Track:
    """One continuously observed candidate."""

    track_id: int
    first_row: float
    last_row: float
    u: float
    v: float
    hits: int
    misses: int = 0
    confidence: float = 0.0
    reported: bool = False
    reported_at_s: float | None = None

    @property
    def approaching(self) -> bool:
        """True when the candidate has moved toward the vehicle overall."""
        return self.last_row > self.first_row


@dataclass
class GateStats:
    """Counters for every rejection path.

    Without these, a missed leak is indistinguishable from a leak that was seen
    and then rejected, and that distinction is the first branch of any diagnosis
    (see `.dsh/agents/troubleshooting.md`).
    """

    candidates: int = 0
    rejected_discriminator: int = 0
    rejected_tape_zone: int = 0
    rejected_shape: int = 0
    tracks_created: int = 0
    rejected_unconfirmed: int = 0
    rejected_confidence: int = 0
    rejected_duplicate: int = 0
    rejected_not_yet_coincident: int = 0
    reports: int = 0

    def as_dict(self) -> dict[str, int]:
        """Plain mapping for logging."""
        return {
            "candidates": self.candidates,
            "rejected_discriminator": self.rejected_discriminator,
            "rejected_tape_zone": self.rejected_tape_zone,
            "rejected_shape": self.rejected_shape,
            "tracks_created": self.tracks_created,
            "rejected_unconfirmed": self.rejected_unconfirmed,
            "rejected_confidence": self.rejected_confidence,
            "rejected_duplicate": self.rejected_duplicate,
            "rejected_not_yet_coincident": self.rejected_not_yet_coincident,
            "reports": self.reports,
        }


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def candidate_score(cand: MarkerCand, calibration: Calibration) -> float:
    """Weakest-link evidence score in ``[0, 1]`` for one observation.

    Every term is a feature that a real marker should satisfy.  The minimum is
    taken rather than the mean so that a single disqualifying feature - a
    shape that is not a disc, a size far from the calibrated diameter, a
    position far off the route - pulls the whole score down.  That bias is
    intentional: the scoring penalises a false report more than it rewards an
    extra true one.

    A candidate whose lateral offset is unreliable (the strip was not located)
    is scored at a fixed middle value rather than as perfectly centred, so that
    "we do not know where the route is" cannot masquerade as strong evidence.
    """
    cal = calibration

    circularity = _clamp01(
        (cand.circularity - cal.marker_circularity_min) / max(1e-6, 0.95 - cal.marker_circularity_min)
    )
    fill = _clamp01((cand.fill_ratio - 0.60) / 0.35)

    expected = max(1e-6, cal.marker_diameter_px)
    ratio = max(1e-6, cand.equiv_diameter_px / expected)
    size = _clamp01(1.0 - abs(math.log(ratio)) / math.log(2.0))

    lateral = 0.5
    if cand.lateral_reliable:
        lateral = _clamp01(1.0 - abs(cand.lateral_offset_px) / max(1.0, cal.frame_w * 0.25))

    return min(circularity, fill, size, lateral)


class Reporter:
    """Associate candidates into tracks and decide what to report."""

    #: Association gate as a multiple of the calibrated marker diameter.  A
    #: candidate must land this close to a track's last position to continue it.
    ASSOCIATION_RADIUS_RATIO = 2.0

    def __init__(
        self,
        report: ReportParams,
        marker: MarkerParams,
        calibration: Calibration,
    ) -> None:
        self.params = report
        self.marker_params = marker
        self.calibration = calibration
        self.tracks: list[Track] = []
        self.stats = GateStats()
        self._next_track_id = 1
        self._reports_emitted = 0

    def reset(self) -> None:
        """Clear all tracks and counters for a new run."""
        self.tracks.clear()
        self.stats = GateStats()
        self._next_track_id = 1
        self._reports_emitted = 0

    @property
    def reports_emitted(self) -> int:
        """How many reports have been emitted this run."""
        return self._reports_emitted

    @property
    def threshold(self) -> float:
        """Current confidence threshold.

        After ``max_reports`` reports the 40-point detection cap is exhausted, so
        further reports have no upside while a false one still costs -3.  The
        threshold is raised accordingly rather than continuing to report at the
        break-even probability.
        """
        if self._reports_emitted >= self.params.max_reports:
            return self.params.saturated_confidence
        return self.params.min_confidence

    # ------------------------------------------------------------- association

    def _associate(self, cand: MarkerCand) -> Track | None:
        radius = self.ASSOCIATION_RADIUS_RATIO * self.calibration.marker_diameter_px
        best: Track | None = None
        best_distance = float("inf")
        for track in self.tracks:
            distance = math.hypot(cand.u - track.u, cand.v - track.v)
            if distance <= radius and distance < best_distance:
                best = track
                best_distance = distance
        return best

    def _in_tape_zone(self, cand: MarkerCand, cross: CrossObs | None) -> bool:
        """Whether a candidate coincides with the start/finish tape.

        The tape is black, spans the frame, and is guaranteed to appear, which
        makes it the most dangerous false marker in the task.
        """
        if cross is None or not cross.detected:
            return False
        margin = self.calibration.marker_diameter_px
        return (cross.row_lo - margin) <= cand.v <= (cross.row_hi + margin)

    # ------------------------------------------------------------------- update

    def update(
        self,
        cands: list[MarkerCand],
        now_s: float,
        cross: CrossObs | None = None,
    ) -> list[ReportEvent]:
        """Consume one frame of candidates and return any report events."""
        cfg = self.params
        events: list[ReportEvent] = []

        for track in self.tracks:
            track.misses += 1

        for cand in cands:
            self.stats.candidates += 1

            if not accepts_marker(cand, self.marker_params, self.calibration):
                self.stats.rejected_discriminator += 1
                continue
            if self._in_tape_zone(cand, cross):
                self.stats.rejected_tape_zone += 1
                continue

            score = candidate_score(cand, self.calibration)
            if score <= 0.0:
                self.stats.rejected_shape += 1
                continue

            track = self._associate(cand)
            if track is None:
                track = Track(
                    track_id=self._next_track_id,
                    first_row=cand.v,
                    last_row=cand.v,
                    u=cand.u,
                    v=cand.v,
                    hits=1,
                    confidence=score,
                )
                self._next_track_id += 1
                self.tracks.append(track)
                self.stats.tracks_created += 1
            else:
                # A candidate that jumped backwards is not the same object
                # approaching the vehicle; treat it as a new sighting.
                if cand.v + 1.0 < track.last_row:
                    track.first_row = cand.v
                track.u = cand.u
                track.v = cand.v
                track.last_row = max(track.last_row, cand.v)
                track.hits += 1
                track.misses = 0
                # Exponential mean so recent observations dominate, without
                # requiring the same frame count to converge.
                track.confidence = 0.5 * track.confidence + 0.5 * score

            # Gate 2: confirmation, approach, and growth.
            confirmed = track.hits >= cfg.confirm_frames and track.approaching
            if not confirmed:
                self.stats.rejected_unconfirmed += 1
                continue

            if track.reported:
                self.stats.rejected_duplicate += 1
                continue

            trigger_row = self.calibration.y_trigger_row - cfg.trigger_lead_px
            if cand.v < trigger_row:
                self.stats.rejected_not_yet_coincident += 1
                continue

            if track.confidence < self.threshold:
                self.stats.rejected_confidence += 1
                continue

            track.reported = True
            track.reported_at_s = now_s
            self._reports_emitted += 1
            self.stats.reports += 1
            events.append(
                ReportEvent(
                    track_id=track.track_id,
                    sequence=self._reports_emitted,
                    row=cand.v,
                    confidence=track.confidence,
                    timestamp_s=now_s,
                )
            )

        # Retire tracks only after they have been missing for a while.  A track
        # that has already reported is kept for the rest of the run: its
        # identity is what prevents the same marker being reported twice, and a
        # run contains at most a handful of markers, so retaining them costs
        # nothing.
        self.tracks = [
            track
            for track in self.tracks
            if track.reported or track.misses <= self.marker_params.max_misses
        ]

        return events
