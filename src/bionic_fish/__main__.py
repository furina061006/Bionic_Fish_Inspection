"""实机入口：连飞控 → 开相机 → 解锁 → 跑 → 上锁 → 打印。

    python -m bionic_fish
    python -m bionic_fish --config tank.yaml
    python -m bionic_fish --device /dev/ttyACM0 --camera 0

这是**实机**路径。仿真在 `sim/`，跑 `python -m sim.run_sim`。
"""

from __future__ import annotations

import argparse

from . import params
from .camera import Camera
from .link import Annunciator, MavlinkLink
from .main import main as run


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m bionic_fish", description="跑一趟巡检（实机）"
    )
    parser.add_argument("--config", default=None, help="平的 YAML 参数覆写，见 params.py")
    parser.add_argument("--device", default=None, help="飞控串口，覆盖 params.mavlink_device")
    parser.add_argument("--camera", type=int, default=None, help="相机编号，覆盖 params.camera_index")
    return parser


def cli(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    p = params.load(args.config)
    if args.device is not None:
        p.mavlink_device = args.device
    if args.camera is not None:
        p.camera_index = args.camera

    link = MavlinkLink(p)
    if not link.connect():
        raise SystemExit(
            f"飞控连不上（{p.mavlink_device}）：{link.last_error}\n"
            "  - 飞控上电了吗、线插好了吗？\n"
            "  - 设备路径对吗（--device）？\n"
            "  - 串口被别的进程占着吗？\n"
            "  - 台架上没有飞控的话，用 `python -m sim.run_sim` 跑仿真。"
        )

    # 先开相机再解锁：相机起不来就不该解锁，否则鱼会在没有视觉的情况下被解锁。
    camera = Camera(p.camera_index)
    if not link.arm():
        camera.close()
        link.close()
        raise SystemExit(
            "飞控拒绝解锁。**不解锁就跑等于白跑**：未解锁的飞控仍然接受杆位指令、"
            "仍然报姿态，程序会跑完、日志会很漂亮、鱼一动不动。"
        )

    try:
        result = run(camera, link, p, Annunciator.for_host(p))
    finally:
        # 上锁放在 finally 里是**安全要求**，不是整洁：终端状态、异常、Ctrl-C 任何
        # 一条退出路径漏掉解锁，飞控都会保持最后一根杆位，鱼在没人看的时候继续游。
        link.disarm()
        camera.close()
        link.close()

    print("=== 结束 ===")
    print(f"  原因   {result.reason}")
    print(f"  上报   {result.reports} 个")
    print(f"  用时   {result.elapsed_s:.2f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(cli())
