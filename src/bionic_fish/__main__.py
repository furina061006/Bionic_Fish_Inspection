"""Command-line entry point: ``python -m bionic_fish``.

Why this module exists
----------------------
Every other module in this package is a *stage*; none of them runs anything.  The
only way to execute the stack used to be to import
:class:`~bionic_fish.runtime.Runner` from a test file, which turned "how do I run
this?" into a question about the test suite.  This module is the single answer::

    python -m bionic_fish                     # synthetic closed loop, one run
    python -m bionic_fish --trace             # plus one row per frame
    python -m bionic_fish --lateral-mm 0      # a start offset that does not complete
    python -m bionic_fish --video run.mp4     # replay recorded footage
    python -m bionic_fish --camera 0          # live camera (needs one)

It is a *debugging* front end, not the competition program: it runs one episode,
prints what happened and exits.  The competition entry will need the same
:class:`~bionic_fish.runtime.Runner` plus an arm/disarm sequence around it.

``--trace`` is the answer to "what order does this code run in?".  The order is
real but invisible: no single module shows it, because every stage is called once
per frame by the loop in :meth:`~bionic_fish.runtime.Runner._iterate`.  The trace
prints that loop as data, one row per frame.

在链路里的位置
    它是**用户入口**，不是链路里的一环：唯一职责是把 Runner 跑起来并把 RunLog
    打印成人能读的东西。它不含任何算法、不做任何决策
    三种帧源：合成场地（默认）/ 录像（--video）/ 相机（--camera）
    读代码建议：想知道「怎么跑」，读本文件；想知道「跑起来之后发生什么」，
              读 runtime._iterate。两者不要混着读
    概念：.dsh/agents/runtime-flow.md 第 2 节（三个入口）、第 9 节（--trace 用法）
"""

from __future__ import annotations

import argparse
import math
from collections import Counter
from pathlib import Path
from typing import Iterator

import numpy as np

from .arena import Arena
from .config import Config, load_config
from .runtime import RunLog, Runner, evaluate_scoring

#: Synthetic start distance along the route, millimetres.  The default camera and
#: route are calibrated with this row.
DEFAULT_START_MM = 350.0

#: Known-good start offset.  From 30 mm to the left of the centre line the
#: synthetic run completes the whole route and reports all 8 leaks; from the
#: centre or to the right it does not (see tests/test_closed_loop.py).
DEFAULT_LATERAL_MM = -30.0


# --------------------------------------------------------------------------- sources


def _video_frames(path: Path) -> Iterator[tuple[float, np.ndarray]]:
    """Yield ``(timestamp_s, frame)`` from a video file.

    The timestamp is reconstructed from the file's frame rate, so replay timing
    matches recording timing and the controller sees the same ``dt`` a live
    camera would give it.
    """
    import cv2  # local import: the synthetic path must not need a video backend

    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise SystemExit(f"could not open video: {path}")
    fps = capture.get(cv2.CAP_PROP_FPS) or 20.0
    index = 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                return
            yield index / fps, frame
            index += 1
    finally:
        capture.release()


# -------------------------------------------------------------------------- printing


def _print_summary(log: RunLog) -> None:
    print("=== run summary ===")
    print(f"  {'frames':22} {log.frames}")
    print(f"  {'elapsed':22} {log.elapsed_s:.2f} s")
    print(f"  {'final state':22} {log.states[-1] if log.states else '-'}")
    print(f"  {'finished':22} {'yes' if log.finished else 'no'}")
    if log.abort:
        print(f"  {'abort reason':22} {log.abort}")
    print(f"  {'strip visible':22} {log.strip_valid_ratio:.3f} of frames")
    print(
        f"  {'lateral error':22} mean {log.mean_abs_lateral_px:.1f} px"
        f"   max {log.max_abs_lateral_px:.1f} px"
    )


def _print_reports(log: RunLog) -> None:
    print(f"\n=== reports ({len(log.reports)}) ===")
    for event in log.reports:
        print(
            f"  #{event.sequence:<3} t={event.timestamp_s:7.2f}s"
            f"  row={event.row:6.1f}  p={event.confidence:.3f}"
            f"  track={event.track_id:<3} {event.announcement}"
        )
    missing = len(log.reports) - len(log.announcements)
    if missing:
        print(f"  !! {missing} report(s) without an announcement (speech scores -2 each)")


def _print_gates(log: RunLog) -> None:
    print("\n=== gate counters (why a candidate did or did not become a report) ===")
    for key, value in log.gate_stats.items():
        print(f"  {key:32} {value}")


def _print_score(log: RunLog) -> None:
    scores = evaluate_scoring(log)
    print("\n=== score (published table; only the terms a run log can determine) ===")
    print(f"  {'reports':24} {int(scores['reports']):>8}")
    for key in (
        "detection_points",
        "false_report_penalty",
        "speech_points",
        "time_points",
        "subtotal",
    ):
        print(f"  {key:24} {scores[key]:+8.1f}")
    print("  (completion, innovation bonus and the judges' measurements are not inferred)")


def _print_states(log: RunLog) -> None:
    print("\n=== state occupancy ===")
    for state, count in Counter(log.states).most_common():
        print(f"  {state:12} {count:6}  {100.0 * count / max(1, log.frames):5.1f}%")


_TRACE_HEADER = (
    f"{'t':>7} {'state':<11} {'line':>4} {'e_y_px':>8} {'e_th':>7} {'width':>6} "
    f"{'conf':>5} {'cand':>4} {'rep':>3} {'target':>6} {'yaw':>5} {'fwd':>5}  note"
)


def _print_trace(log: RunLog, every: int, limit: int) -> None:
    """Print the per-frame loop as a table.

    Columns are the quantities that cross a module boundary, in the order the
    stages produce them: perception (``line``/``e_y_px``/``e_th``/``width``/
    ``conf``), the report gate (``cand``/``rep``), the state machine (``state``/
    ``target``), then control (``yaw``/``fwd``).
    """
    print(f"\n=== per-frame trace (every {every} frame(s), at most {limit} rows) ===")
    if not log.trace:
        print("  (no trace recorded - pass --trace to Runner)")
        return
    print(_TRACE_HEADER)
    shown = 0
    for index, row in enumerate(log.trace):
        if index % every:
            continue
        if shown >= limit:
            print(f"  ... {len(log.trace) - index} further frame(s) not shown")
            break
        print(
            f"{row.t_s:7.2f} {row.state:<11} {'yes' if row.line_valid else 'no':>4}"
            f" {row.e_y_px:8.1f} {row.e_theta_rad:7.3f} {row.strip_width_px:6.1f}"
            f" {row.confidence:5.2f} {row.candidates:4} {row.reports:3}"
            f" {row.target_forward_us:6} {row.yaw_us:5} {row.forward_us:5}  {row.note}"
        )
        shown += 1


# ------------------------------------------------------------------------------ cli


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m bionic_fish",
        description="Run the inspection stack once and print what it did.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="YAML overlay on top of the built-in defaults (see bionic_fish.config)",
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument(
        "--video", type=Path, help="replay a video file instead of the synthetic arena"
    )
    source.add_argument(
        "--camera",
        type=int,
        metavar="INDEX",
        help="run against a live camera; UNVERIFIED on real hardware",
    )
    parser.add_argument("--steps", type=int, default=3000, help="maximum frames [3000]")
    parser.add_argument("--dt", type=float, default=0.05, help="synthetic period, s [0.05]")
    parser.add_argument(
        "--start-mm",
        type=float,
        default=DEFAULT_START_MM,
        help=f"synthetic start distance along the route, mm [{DEFAULT_START_MM:g}]",
    )
    parser.add_argument(
        "--lateral-mm",
        type=float,
        default=DEFAULT_LATERAL_MM,
        help=f"synthetic start offset, positive to the right, mm [{DEFAULT_LATERAL_MM:g}]",
    )
    parser.add_argument(
        "--yaw-error-deg", type=float, default=0.0, help="synthetic initial heading error [0]"
    )
    parser.add_argument(
        "--trace", action="store_true", help="print the per-frame loop as a table"
    )
    parser.add_argument(
        "--trace-every", type=int, default=1, metavar="N", help="with --trace, every Nth frame [1]"
    )
    parser.add_argument(
        "--trace-limit", type=int, default=200, metavar="N", help="with --trace, at most N rows [200]"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run one episode and print a report.  Returns a process exit code."""
    args = _parser().parse_args(argv)

    cfg: Config = load_config(args.config)
    # Constructing the arena is cheap - the floor is rendered lazily on the first
    # camera view - so it is built even for replay, where it is unused.
    arena = Arena(calibration=cfg.calibration)
    runner = Runner(cfg, arena, trace=args.trace)

    steps = max(1, args.steps)
    if args.video is not None:
        log = runner.run_frames(_video_frames(args.video), dt_s=args.dt, max_steps=steps)
    elif args.camera is not None:
        log = runner.run_camera(args.camera, dt_s=args.dt, max_steps=steps)
    else:
        log = runner.run_synthetic(
            dt_s=args.dt,
            max_steps=steps,
            start_s_mm=args.start_mm,
            lateral_mm=args.lateral_mm,
            yaw_error_rad=math.radians(args.yaw_error_deg),
        )

    _print_summary(log)
    _print_reports(log)
    _print_gates(log)
    _print_score(log)
    _print_states(log)
    if args.trace:
        _print_trace(log, max(1, args.trace_every), max(0, args.trace_limit))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
