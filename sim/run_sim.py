"""仿真入口。

    python -m sim.run_sim
    python -m sim.run_sim --lateral-mm -30
    python -m sim.run_sim --config tank.yaml --trace

它做的事和实机入口一模一样：装好两个口，调 `bionic_fish.main`，打印结果。区别只在
塞进去的是渲染器和记录器，而不是相机和飞控。
"""

from __future__ import annotations

import argparse

from bionic_fish import params
from bionic_fish.link import Annunciator, RecordingGpio
from bionic_fish.main import main as run

from .harness import build


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m sim.run_sim", description="跑一趟巡检（仿真）")
    parser.add_argument("--config", default=None, help="平的 YAML 参数覆写，见 params.py")
    parser.add_argument("--steps", type=int, default=4000, help="最多渲染多少帧 [4000]")
    parser.add_argument("--dt", type=float, default=0.05, help="模拟步长，秒 [0.05]")
    parser.add_argument("--start-mm", type=float, default=350.0, help="起始里程，毫米 [350]")
    parser.add_argument("--lateral-mm", type=float, default=0.0, help="起始横向偏置，毫米 [0]")
    return parser


def cli(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    p = params.load(args.config)

    camera, link, plant, arena = build(
        p, dt_s=args.dt, start_mm=args.start_mm, lateral_mm=args.lateral_mm, max_frames=args.steps
    )
    gpio = RecordingGpio()
    result = run(camera, link, p, Annunciator(p, gpio=gpio))

    lit = sum(1 for kind, _, value in gpio.events if kind == "write" and value)
    arc = arena.spec.arc_length_of(plant.pose[0], plant.pose[1])
    print("=== 结束 ===")
    print(f"  原因   {result.reason}")
    print(f"  上报   {result.reports} 个")
    print(f"  用时   {result.elapsed_s:.2f} s（{camera.index} 帧）")
    print(f"  走完   {arc:.0f} / {arena.spec.path_length_mm:.0f} mm")
    print(f"  灯亮   {lit} 次")
    return 0


if __name__ == "__main__":
    raise SystemExit(cli())
