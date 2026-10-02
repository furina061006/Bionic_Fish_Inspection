"""Assembly and the main loop.

Wires perception, the report gate, the state machine, control and the platform
link into one run, and provides the two ways to drive it:

* :meth:`Runner.run_synthetic` closes the loop against the synthetic arena and
  the placeholder plant.  This is the only way to exercise the whole stack
  today, because there is no camera, no water and no vehicle.
* :meth:`Runner.run_frames` replays an iterable of ``(timestamp, frame)``.  It
  is the entry point that will be used once real footage or a live camera
  exists, and it needs no changes to any other module.

The loop deliberately processes one frame per iteration and never sleeps.  The
reference project called ``cv2.imshow`` and ``cv2.waitKey`` inside its control
loop and used blocking ``sleep`` calls for timed turns, neither of which can
coexist with a 20 Hz control requirement.

The order that loop imposes on the stack is invisible in the stages themselves,
because each stage is called once per frame and knows nothing about the others.
It is recorded, one row per frame, when the runner is built with ``trace=True``
(see :class:`FrameTrace`), and printed by the command line front end in
:mod:`bionic_fish.__main__` (``python -m bionic_fish --trace``).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Iterable, Iterator

import numpy as np

from .arena import Arena
from .config import Config
from .control import Actuator, LineController
from .fsm import Decision, State, StateMachine
from .plant import PlantParams, VehiclePlant
from .platform import Annunciator, Link, LoggingSpeaker, SimLink
from .reporting import GateStats, ReportEvent, Reporter
from .vision import CrossLineDetector, MarkerDetector, StripTracker

#: One frame of the run, as the loop sees it.
FrameSource = Iterable[tuple[float, np.ndarray]]


@dataclass
class FrameTrace:
    """One row of the per-frame trace.

    The order in which a frame moves through the stack is not visible in any
    single module: every stage is called once per frame by :meth:`Runner._iterate`
    and none of them knows about the others.  This is the instrument that makes
    that order observable, so the fields are ordered the way the stages run.
    """

    t_s: float
    state: str
    line_valid: bool
    e_y_px: float
    e_theta_rad: float
    strip_width_px: float
    confidence: float
    #: Candidates found this frame, and reports emitted from them.
    candidates: int
    reports: int
    #: What the state machine asked for, then what control actually sent.
    target_forward_us: int
    yaw_us: int
    forward_us: int
    note: str


@dataclass
class RunLog:
    """Everything a run produced, in a form that can be asserted on."""

    frames: int = 0
    elapsed_s: float = 0.0
    #: State of every frame, in order.
    states: list[str] = field(default_factory=list)
    #: One entry per report, in order.
    reports: list[ReportEvent] = field(default_factory=list)
    #: Phrases the annunciator actually spoke.  Must match ``reports`` one to
    #: one; a mismatch is a scoring defect, not a cosmetic one.
    announcements: list[str] = field(default_factory=list)
    #: Gate rejection counters.
    gate_stats: dict[str, int] = field(default_factory=dict)
    #: Why the run ended, if it ended early.
    abort: str | None = None
    #: Final pose, for synthetic runs.
    final_pose: tuple[float, float, float] | None = None
    #: Signed lateral error in image pixels for each valid frame.
    lateral_errors_px: list[float] = field(default_factory=list)
    #: Fraction of frames on which the strip was located.
    strip_valid_ratio: float = 0.0
    #: True when the run reached the finish state.
    finished: bool = False
    #: Per-frame trace, one row per frame.  Empty unless the runner was built
    #: with ``trace=True``.
    trace: list[FrameTrace] = field(default_factory=list)

    @property
    def mean_abs_lateral_px(self) -> float:
        """Mean absolute lateral error over frames where the strip was visible."""
        if not self.lateral_errors_px:
            return float("nan")
        return float(np.mean(np.abs(self.lateral_errors_px)))

    @property
    def max_abs_lateral_px(self) -> float:
        """Worst lateral error over frames where the strip was visible."""
        if not self.lateral_errors_px:
            return float("nan")
        return float(np.max(np.abs(self.lateral_errors_px)))


class Runner:
    """Owns one run's components and executes its loop."""

    def __init__(
        self,
        cfg: Config,
        arena: Arena,
        *,
        link: Link | None = None,
        annunciator: Annunciator | None = None,
        trace: bool = False,
    ) -> None:
        self.cfg = cfg
        self.arena = arena
        #: Record one :class:`FrameTrace` per frame.  Off by default: a long run
        #: holds one row per frame, which is the point when reading the loop back
        #: and pure overhead when not.
        self.trace = trace
        self.tracker = StripTracker(cfg.strip, cfg.calibration)
        self.markers = MarkerDetector(cfg.marker, cfg.calibration)
        self.cross = CrossLineDetector(cfg.marker, cfg.calibration)
        self.reporter = Reporter(cfg.report, cfg.marker, cfg.calibration)
        self.controller = LineController(cfg.control, cfg.channels)
        self.machine = StateMachine(cfg.fsm, cfg.safety, cfg.control, cfg.channels)
        self.link: Link = link if link is not None else SimLink()
        self.annunciator = annunciator if annunciator is not None else Annunciator(
            cfg.annunciator, speaker=LoggingSpeaker()
        )

    def reset(self) -> None:
        """Return every component to its pre-start state."""
        self.reporter.reset()
        self.controller.reset()
        self.machine.reset()

    # ------------------------------------------------------------------ the loop

    def _iterate(
        self,
        frames: FrameSource,
        *,
        plant: VehiclePlant | None,
        max_steps: int,
    ) -> RunLog:
        log = RunLog()
        valid_frames = 0
        last_forward_us: int | None = None
        last_now = 0.0

        for now_s, frame in frames:
            if log.frames >= max_steps:
                break
            dt = max(1e-6, now_s - last_now)
            last_now = now_s

            line = self.tracker.observe(frame)
            cands = self.markers.observe(frame, line)
            cross = self.cross.observe(frame, line)

            decision = self.machine.step(
                line, cross, now_s, last_forward_us, self.link.yaw_rad()
            )
            events = self.reporter.update(cands, now_s, cross)
            for event in events:
                log.reports.append(event)
                self.annunciator.on_report(event, now_s)
            self.annunciator.tick(now_s)

            actuator = self.controller.step(
                line,
                decision.target_forward_us,
                now_s,
                dt,
                corner=decision.corner,
                yaw_override_us=decision.yaw_override_us,
            )
            self.link.send(actuator)
            last_forward_us = actuator.forward_us

            if plant is not None:
                plant.step(actuator, dt)
                log.final_pose = plant.pose

            log.frames += 1
            log.states.append(decision.state.value)
            if line.valid:
                valid_frames += 1
                log.lateral_errors_px.append(line.e_y_px)
            if self.trace:
                log.trace.append(
                    FrameTrace(
                        t_s=now_s,
                        state=decision.state.value,
                        line_valid=line.valid,
                        e_y_px=line.e_y_px,
                        e_theta_rad=line.e_theta_rad,
                        strip_width_px=line.strip_width_px,
                        confidence=line.confidence,
                        candidates=len(cands),
                        reports=len(events),
                        target_forward_us=decision.target_forward_us,
                        yaw_us=actuator.yaw_us,
                        forward_us=actuator.forward_us,
                        note=decision.note,
                    )
                )
            if decision.terminal:
                log.abort = decision.abort.value if decision.abort else None
                log.finished = decision.state is State.DONE
                break

        log.elapsed_s = last_now
        log.strip_valid_ratio = valid_frames / max(1, log.frames)
        log.gate_stats = self.reporter.stats.as_dict()
        log.announcements = list(getattr(self.annunciator.speaker, "spoken", []) or self.annunciator.announcements)
        return log

    # ---------------------------------------------------------------- synthetic

    def run_synthetic(
        self,
        *,
        dt_s: float = 0.05,
        max_steps: int = 4000,
        start_s_mm: float = 350.0,
        lateral_mm: float = 0.0,
        yaw_error_rad: float = 0.0,
        plant_params: PlantParams | None = None,
    ) -> RunLog:
        """Close the loop against the synthetic arena.

        The vehicle starts on the first straight, offset laterally and rotated if
        asked, so that the run also exercises the recovery path rather than
        starting perfectly aligned.
        """
        spec = self.arena.spec
        (x0, y0), heading0 = spec.point_at(start_s_mm)
        x0 += -np.sin(heading0) * lateral_mm
        y0 += np.cos(heading0) * lateral_mm

        plant = VehiclePlant(
            plant_params or PlantParams(),
            self.cfg.channels.neutral,
            float(x0),
            float(y0),
            float(heading0 + yaw_error_rad),
        )
        plant.set_dt(dt_s)
        self.machine.start(0.0)

        # On the bench the plant's own pose is the only heading that exists, so
        # offer it to the link.  This makes the angle-based bend termination
        # testable before an autopilot is available; it does not make the vehicle
        # model real.
        if isinstance(self.link, SimLink):
            self.link.set_heading_source(lambda: plant.yaw_rad)

        def frames() -> Iterator[tuple[float, np.ndarray]]:
            step = 0
            while True:
                x, y, yaw = plant.pose
                yield step * dt_s, self.arena.view(x, y, yaw)
                step += 1

        return self._iterate(frames(), plant=plant, max_steps=max_steps)

    # ------------------------------------------------------------------- replay

    def run_frames(
        self,
        frames: FrameSource,
        *,
        dt_s: float = 0.05,
        max_steps: int = 100_000,
        start_at_s: float = 0.0,
    ) -> RunLog:
        """Replay timestamps and frames without a plant.

        The entry point for recorded footage and for a live camera: the caller
        supplies ``(timestamp_s, frame)`` pairs, so the source can be a video
        file, a directory of images, or a camera, and nothing else changes.
        """
        self.machine.start(start_at_s)
        return self._iterate(frames, plant=None, max_steps=max_steps)

    # ------------------------------------------------------------------- camera

    def run_camera(self, device: int = 0, *, dt_s: float = 0.05, max_steps: int = 4000) -> RunLog:
        """Run against a live camera.

        Requires OpenCV with video capture and a camera on the host; neither is
        available on the development machine, so this path is UNVERIFIED.
        """
        import cv2

        capture = cv2.VideoCapture(device)
        if not capture.isOpened():
            raise RuntimeError(f"camera {device} did not open")

        def frames() -> Iterator[tuple[float, np.ndarray]]:
            started = time.monotonic()
            while True:
                ok, frame = capture.read()
                if not ok:
                    return
                yield time.monotonic() - started, frame

        try:
            return self.run_frames(frames(), dt_s=dt_s, max_steps=max_steps)
        finally:
            capture.release()


def evaluate_scoring(log: RunLog, *, true_leaks: int = 8) -> dict[str, float]:
    """Score a run against the published table.

    Only the terms a run log can determine are computed.  Completion score,
    innovation bonus and the time band need the judges' measurements, so they are
    left out rather than guessed; the time band is reported as the band the run
    would fall into.

    A report is assumed true: the log cannot know whether a report was correct,
    because the distractor's appearance is unknown.  Treat the false-positive
    term as an input, not a result.
    """
    reports = sum(1 for _ in log.reports)
    detection = min(reports, true_leaks) * 5.0
    false_reports = max(0, reports - true_leaks)
    penalty = min(false_reports * 3.0, 24.0)
    speech = max(0.0, 10.0 - 2.0 * max(0, len(log.reports) - len(log.announcements)))

    elapsed = log.elapsed_s
    if elapsed <= 10.0:
        time_score = 20.0
    elif elapsed <= 20.0:
        time_score = 15.0
    elif elapsed <= 30.0:
        time_score = 12.0
    elif elapsed <= 60.0:
        time_score = 10.0
    elif elapsed <= 100.0:
        time_score = 8.0
    elif elapsed <= 150.0:
        time_score = 5.0
    else:
        time_score = 0.0

    return {
        "reports": float(reports),
        "detection_points": detection,
        "false_report_penalty": -penalty,
        "speech_points": speech,
        "time_points": time_score,
        "subtotal": detection - penalty + speech + time_score,
    }
