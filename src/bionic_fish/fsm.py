"""State machine and the four hard termination conditions.

The reference project had no state machine worth the name and no termination
logic at all: its main loops were ``while 1``, its phase transitions were
``break`` statements, turning was a pair of ``sleep`` calls, and the whole run
state travelled between phases as a 23-element tuple.

Under this competition that is not viable, because four conditions end a run
outright and each one is a FACT from the document (p2, p6):

============================  ==========================================
Condition                     Consequence
============================  ==========================================
off the route > 10 s          run terminated
3 wall or floor contacts      round ends
not moving 30 s after start   the round ends
150 s elapsed                 time score is zero
============================  ==========================================

This module therefore owns two things that must not be spread elsewhere: the
state transitions, and the counters that can end the run.  Every
:class:`Decision` carries those counters so an operator can see the remaining
margins without reading a log.

在链路里的位置
    每帧第 5 步（共 13 步，见 runtime.Runner._iterate），由 runtime 调用
    上游：vision.LineObs / vision.CrossObs、上一帧**实际发出**的前进量、当前航向或 None
    下游：Decision → control.LineController.step
          它只读三个字段：target_forward_us / corner / yaw_override_us
          其余字段（elapsed_s / off_track_s / wall_hits / note）是给人看的诊断量
    跨帧状态：有。state、_corner_hits、_bend_active、_turn_start_yaw、
              _bend_finished_at 等；Runner.reset() 会调 reset()
    概念：runtime-flow.md 第 6.2 节（判断次序）、第 7 节（状态生命历程）
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from .config import ChannelMap, ControlParams, FsmParams, SafetyParams
from .vision import CrossObs, LineObs


def wrap_pi(angle_rad: float) -> float:
    """Wrap an angle into ``(-pi, pi]``."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


class State(str, Enum):
    """Every state the run can be in."""

    IDLE = "IDLE"
    ALIGN = "ALIGN"
    FOLLOW = "FOLLOW"
    CORNER = "CORNER"
    LINE_LOST = "LINE_LOST"
    FINISH_APPR = "FINISH_APPR"
    DONE = "DONE"
    ABORT = "ABORT"


class AbortReason(str, Enum):
    """Why a run ended early.  Each maps to one FACT condition."""

    OFF_TRACK_TIMEOUT = "off_track_timeout"
    WALL_HITS = "wall_hits"
    START_TIMEOUT = "start_timeout"
    RUN_TIMEOUT = "run_timeout"


#: States from which no further transition happens.
TERMINAL_STATES = frozenset({State.DONE, State.ABORT})


@dataclass
class Decision:
    """One state-machine output frame."""

    state: State
    #: Forward PWM the controller should slew toward.
    target_forward_us: int
    #: Whether the inner yaw loop should use the corner gain set.
    corner: bool
    terminal: bool
    abort: AbortReason | None
    elapsed_s: float
    off_track_s: float
    wall_hits: int
    #: Human-readable reason for the current state, for logs and the debug view.
    note: str = ""
    #: When set, the controller must command this yaw offset directly instead of
    #: running the strip-following loop.  Used to commit to a bend, where no
    #: heading reference exists.
    yaw_override_us: float | None = None


class SafetyMonitor:
    """Counters for the four termination conditions.

    Kept separate from the transition logic on purpose: these are the conditions
    that must keep working even when the state machine is confused.  If they
    lived inside the transitions, a bug in a transition could disable a
    protection as well.
    """

    def __init__(self, params: SafetyParams, channels: ChannelMap) -> None:
        self.params = params
        self.channels = channels
        self.reset()

    def reset(self) -> None:
        """Clear every counter."""
        self._armed_at: float | None = None
        self._off_track_since: float | None = None
        self._saturated_since: float | None = None
        self._saturation_counted = False
        self._wall_hits = 0
        self._started = False

    def arm(self, now_s: float) -> None:
        """Record the moment the start command was given."""
        if self._armed_at is None:
            self._armed_at = now_s

    @property
    def armed(self) -> bool:
        """Whether the start command has been given."""
        return self._armed_at is not None

    @property
    def started(self) -> bool:
        """Whether the vehicle has been observed to move."""
        return self._started

    @property
    def wall_hits(self) -> int:
        """Wall/floor contacts inferred so far."""
        return self._wall_hits

    def elapsed_s(self, now_s: float) -> float:
        """Seconds since the start command, or 0 before it."""
        if self._armed_at is None:
            return 0.0
        return max(0.0, now_s - self._armed_at)

    def off_track_s(self, now_s: float) -> float:
        """Seconds the strip has been missing, or 0 while it is visible."""
        if self._off_track_since is None:
            return 0.0
        return max(0.0, now_s - self._off_track_since)

    def update(
        self,
        *,
        line_valid: bool,
        lateral_norm: float,
        commanded_forward_us: int,
        now_s: float,
    ) -> AbortReason | None:
        """Advance the counters and return a termination reason if one fired.

        ``lateral_norm`` is the strip's lateral error as a fraction of half the
        frame width, so the thresholds do not depend on frame size.
        ``commanded_forward_us`` is what was actually sent last frame, not the
        target: "the vehicle started" must mean the actuator moved, not that the
        state machine asked it to.
        """
        p = self.params
        if not self.armed:
            return None

        if abs(commanded_forward_us - self.channels.neutral) > p.started_forward_us:
            self._started = True

        if line_valid:
            self._off_track_since = None
        elif self._off_track_since is None:
            self._off_track_since = now_s

        # Wall contact, inferred.  The platform may or may not expose a
        # collision signal; without one, a lateral error pinned near the frame
        # edge for a sustained period is the available proxy.  One continuous
        # excursion counts as one contact: counting again every
        # ``wall_hit_infer_s`` while the error stays pinned would end the round
        # from a single mistake and would understate how much margin remains.
        if abs(lateral_norm) >= p.wall_hit_error_frac:
            if self._saturated_since is None:
                self._saturated_since = now_s
                self._saturation_counted = False
            elif (
                not self._saturation_counted
                and now_s - self._saturated_since >= p.wall_hit_infer_s
            ):
                self._wall_hits += 1
                self._saturation_counted = True
        else:
            self._saturated_since = None
            self._saturation_counted = False

        if self.off_track_s(now_s) >= p.off_track_timeout_s:
            return AbortReason.OFF_TRACK_TIMEOUT
        if self._wall_hits >= p.wall_hit_limit:
            return AbortReason.WALL_HITS
        if not self._started and self.elapsed_s(now_s) >= p.start_timeout_s:
            return AbortReason.START_TIMEOUT
        if self.elapsed_s(now_s) >= p.run_timeout_s:
            return AbortReason.RUN_TIMEOUT
        return None


class StateMachine:
    """Drive the run through its states and enforce the terminations."""

    def __init__(
        self,
        fsm: FsmParams,
        safety: SafetyParams,
        control: ControlParams,
        channels: ChannelMap,
    ) -> None:
        self.params = fsm
        self.control = control
        self.channels = channels
        self.safety = SafetyMonitor(safety, channels)
        self.reset()

    def reset(self) -> None:
        """Return to the pre-start state."""
        self.state = State.IDLE
        self.safety.reset()
        self.abort_reason: AbortReason | None = None
        self._corner_hits = 0
        self._state_since: float | None = None
        self._finish_since: float | None = None
        self._lost_from_corner = False
        self._turn_direction = self.params.corner_turn_sign
        self._turn_start_yaw: float | None = None
        self._turn_phase_since: float | None = None
        self._turn_reversed = False
        self._bend_active = False
        self._bend_since: float | None = None
        self._bend_finished_at: float | None = None
        self._note = "waiting for the start command"

    # --------------------------------------------------------------------- turn

    def _begin_turn(self, yaw_rad: float | None, now_s: float) -> None:
        """Start a bend: pick a direction and remember the heading it began at.

        With a heading source the bend can be ended by angle, which is the only
        termination that does not depend on the strip being visible.  Without
        one, the direction is searched and the time bound is the only limit.
        """
        self._turn_direction = self.params.corner_turn_sign
        self._turn_start_yaw = yaw_rad
        self._turn_phase_since = now_s
        self._turn_reversed = False
        self._bend_active = True
        self._bend_since = now_s

    def _finish_bend(self, line: LineObs, now_s: float, note: str) -> None:
        """End the committed turn and hand back to strip following."""
        self._bend_active = False
        self._lost_from_corner = False
        self._corner_hits = 0
        self._turn_start_yaw = None
        self._bend_finished_at = now_s
        if line.valid:
            self._enter(State.FOLLOW, now_s, note)
        else:
            self._enter(State.LINE_LOST, now_s, f"{note}; strip not visible")

    def _in_bend_refractory(self, now_s: float) -> bool:
        """Whether a just-completed bend still suppresses the corner cue."""
        if self._bend_finished_at is None:
            return False
        return now_s - self._bend_finished_at < self.params.corner_refractory_s

    def _turned_rad(self, yaw_rad: float | None) -> float | None:
        """Heading change since the bend began, signed into the turn direction.

        ``None`` when no heading source is available, which the caller must treat
        as "unknown", never as "no turn yet made".
        """
        if yaw_rad is None or self._turn_start_yaw is None:
            return None
        return wrap_pi(yaw_rad - self._turn_start_yaw) * self._turn_direction

    # ------------------------------------------------------------------ control

    def start(self, now_s: float) -> None:
        """Accept the start command and begin the run."""
        if self.state is not State.IDLE:
            return
        self.safety.arm(now_s)
        self._enter(State.ALIGN, now_s, "start command received")

    def _enter(self, state: State, now_s: float, note: str) -> None:
        self.state = state
        self._state_since = now_s
        self._note = note
        if state is State.FINISH_APPR and self._finish_since is None:
            self._finish_since = now_s

    def _time_in_state(self, now_s: float) -> float:
        if self._state_since is None:
            return 0.0
        return max(0.0, now_s - self._state_since)

    # ------------------------------------------------------------------- corner

    def _corner_cue(self, line: LineObs) -> bool:
        """Whether this frame looks like a 90-degree bend.

        Measured on the synthetic arena: approaching the bend the strip leaves
        the frame sideways, so the share of rows yielding a run collapses
        monotonically (0.94 -> 0.43), while curvature spikes roughly 160x but
        only within the last few centimetres.  Confidence is therefore the
        primary cue and curvature the confirmation.  Both thresholds are
        UNVERIFIED against a real pool, where a lateral offset also lowers
        confidence.
        """
        weak_support = line.confidence < self.params.corner_confidence_min
        tight_bend = abs(line.curvature_rad_per_row) > self.params.corner_curvature_rad_per_row
        return weak_support or tight_bend

    # --------------------------------------------------------------------- step

    def step(
        self,
        line: LineObs,
        cross: CrossObs | None,
        now_s: float,
        commanded_forward_us: int | None = None,
        yaw_rad: float | None = None,
    ) -> Decision:
        """Advance the state machine by one frame.

        ``commanded_forward_us`` is the actuator value actually sent on the
        previous frame, used only to decide whether the vehicle has started.
        ``yaw_rad`` is the current heading when a source exists; without one the
        bend falls back to a time-bounded two-way search.
        """
        c = self.control
        neutral = self.channels.neutral
        commanded = neutral if commanded_forward_us is None else commanded_forward_us

        if self.state in TERMINAL_STATES:
            return self._decision(self.state, neutral, now_s, corner=False)

        abort = self.safety.update(
            line_valid=line.valid,
            lateral_norm=line.e_y_norm if line.valid else 0.0,
            commanded_forward_us=commanded,
            now_s=now_s,
        )
        if abort is not None:
            self.abort_reason = abort
            self._enter(State.ABORT, now_s, f"aborted: {abort.value}")
            return self._decision(State.ABORT, neutral, now_s, corner=False, abort=abort)

        if self.state is State.IDLE:
            return self._decision(State.IDLE, neutral, now_s, corner=False)

        if self.state is State.ALIGN:
            aligned = line.valid and (
                abs(line.e_theta_rad) <= self.params.align_heading_tol_rad
                and abs(line.e_y_norm) <= self.params.align_lateral_tol
            )
            if aligned:
                self._enter(State.FOLLOW, now_s, "aligned on the strip")
            elif self._time_in_state(now_s) >= self.params.align_timeout_s:
                # The start timeout ends the round, so give up aligning and move
                # rather than waiting for a perfect pose.
                self._enter(State.FOLLOW, now_s, "align timeout: starting anyway")
            else:
                return self._decision(State.ALIGN, c.forward_align, now_s, corner=False)

        if self.state in (State.FOLLOW, State.CORNER, State.LINE_LOST):
            if not line.valid:
                if self.state is not State.LINE_LOST:
                    # A bend keeps running across the loss.  The strip always
                    # disappears part way round a 90 degree corner, and if that
                    # cleared the bend the committed turn would never be ended.
                    self._lost_from_corner = self._bend_active
                    self._corner_hits = 0
                    self._enter(State.LINE_LOST, now_s, "strip lost")
            elif self.state is State.LINE_LOST:
                self._corner_hits = 0
                if not self._bend_active:
                    self._lost_from_corner = False
                    self._enter(State.FOLLOW, now_s, "strip reacquired")
            elif self.state is State.FOLLOW:
                if self._in_bend_refractory(now_s):
                    self._corner_hits = 0
                elif self._corner_cue(line):
                    self._corner_hits += 1
                    if self._corner_hits >= self.params.corner_confirm_frames:
                        self._enter(State.CORNER, now_s, "bend confirmed")
                        self._begin_turn(yaw_rad, now_s)
                else:
                    self._corner_hits = 0

        # Advancing the bend is deliberately independent of the state, so that a
        # strip loss mid-corner cannot disable the termination that stops the
        # turn.
        if self._bend_active:
            self._advance_bend(line, yaw_rad, now_s)

        # Finish acceptance needs context: the start tape looks identical, so a
        # cross line only counts once the vehicle has been running long enough to
        # have reached the far end (invariant 6).
        if (
            self.state in (State.FOLLOW, State.CORNER, State.LINE_LOST)
            and cross is not None
            and cross.detected
            and self.safety.elapsed_s(now_s) >= self.params.min_run_time_for_finish_s
        ):
            self._enter(State.FINISH_APPR, now_s, "finish tape detected")

        if self.state is State.FINISH_APPR:
            since = self._finish_since if self._finish_since is not None else now_s
            if now_s - since >= self.params.finish_creep_s:
                self._enter(State.DONE, now_s, "finish zone entered")
                return self._decision(State.DONE, neutral, now_s, corner=False)
            # Keep creeping so the whole body clears the tape, and keep the
            # actuator live so the vehicle does not coast to a stop early.
            return self._decision(State.FINISH_APPR, c.forward_finish, now_s, corner=False)

        if self.state is State.DONE:
            return self._decision(State.DONE, neutral, now_s, corner=False)

        return self._decision(
            self.state,
            self._forward_target(now_s),
            now_s,
            corner=self.state is State.CORNER,
            yaw_override_us=self._yaw_override(),
        )

    def _advance_bend(self, line: LineObs, yaw_rad: float | None, now_s: float) -> None:
        """Decide whether the bend is finished, reversed, or out of time.

        Termination is tried in order of trustworthiness:

        1. **by angle**, when a heading source exists: the document states the
           corners are 90 degrees, so a measured 90 degree change ends the bend
           regardless of what the camera sees;
        2. **by re-sighting**, but only after a minimum commitment, because on the
           straight approach the strip is re-seen immediately and ending there
           means driving straight through the corner;
        3. **by reversing the search** once, when there is no heading source;
        4. **by the time bound**, which hands over to the recovery state.
        """
        turned = self._turned_rad(yaw_rad)
        committed = self._time_in_state(now_s) >= self.params.corner_min_commit_s
        bend_elapsed = now_s - (self._bend_since if self._bend_since is not None else now_s)

        angle_done = (
            turned is not None
            and turned
            >= self.params.corner_target_angle_rad - self.params.corner_angle_tolerance_rad
        )
        if angle_done:
            self._finish_bend(line, now_s, "bend complete by angle")
            return

        if turned is None:
            # No heading reference, so fall back to less trustworthy cues.
            #
            # With a heading source this branch is skipped on purpose.  Near a
            # corner the strip is an L in the frame, and the row-scan fit can
            # latch onto the outgoing straight from the previous leg, reporting a
            # small ``e_theta`` while the vehicle has turned only a few degrees.
            # Accepting that as "bend cleared" ends the turn almost immediately
            # and the vehicle drives straight past the corner - the failure
            # observed in the closed loop.
            if committed and abs(line.e_theta_rad) <= self.params.align_heading_tol_rad:
                self._finish_bend(line, now_s, "bend cleared")
                return

            if (
                not self._turn_reversed
                and self._turn_phase_since is not None
                and now_s - self._turn_phase_since >= self.params.corner_search_s
            ):
                self._turn_direction = -self._turn_direction
                self._turn_reversed = True
                self._turn_phase_since = now_s
                self._note = "bend: reversed the search direction"
                return

        if bend_elapsed >= self.params.corner_timeout_s and self.state is State.CORNER:
            self._lost_from_corner = True
            self._enter(State.LINE_LOST, now_s, "bend not completed in time")

    def _yaw_override(self) -> float | None:
        """The committed turn command, when a bend must be flown blind.

        A bend is flown open-loop in *direction* but closed-loop in *duration*:
        the command is fixed, and the state machine ends it as soon as the strip
        is reacquired or the bound expires.  Extending it past a corner loss is
        deliberate - the strip usually disappears part way round, and dropping
        the turn at that moment would strand the vehicle pointing at the wall.
        """
        c = self.control
        if self.state is State.CORNER:
            return self._turn_direction * c.corner_turn_us
        if self.state is State.LINE_LOST and self._lost_from_corner:
            return self._turn_direction * c.corner_turn_us
        return None

    def _forward_target(self, now_s: float) -> int:
        c = self.control
        if self.state is State.CORNER:
            return c.forward_corner
        if self.state is State.LINE_LOST:
            # Escalate through the recovery levels rather than issuing a single
            # emergency command; the off-track timer is the hard bound.
            lost = self.safety.off_track_s(now_s)
            if lost < self.params.lost_degrade_s:
                return c.forward_cruise
            if lost < self.params.lost_scan_s:
                return c.forward_search
            return c.forward_align
        if self.state is State.FOLLOW:
            return c.forward_cruise
        return c.forward_align

    def _decision(
        self,
        state: State,
        target_forward_us: int,
        now_s: float,
        *,
        corner: bool = False,
        abort: AbortReason | None = None,
        yaw_override_us: float | None = None,
    ) -> Decision:
        return Decision(
            state=state,
            target_forward_us=target_forward_us,
            corner=corner,
            terminal=state in TERMINAL_STATES,
            abort=abort,
            elapsed_s=self.safety.elapsed_s(now_s),
            off_track_s=self.safety.off_track_s(now_s),
            wall_hits=self.safety.wall_hits,
            note=self._note,
            yaw_override_us=yaw_override_us,
        )
