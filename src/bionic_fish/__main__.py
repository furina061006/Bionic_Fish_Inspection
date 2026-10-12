"""实机入口：连飞控/尾鳍 → 开相机 → 解锁 → 跑 → 上锁 → 打印。

    python -m bionic_fish                          # 默认：单关节尾鳍模式
    python -m bionic_fish --tail-fin               # 显式指定尾鳍模式
    python -m bionic_fish --mavlink                # MAVLink 飞控模式（保留）
    python -m bionic_fish --config tank.yaml
    python -m bionic_fish --camera 0

这是**实机**路径。仿真在 `sim/`，跑 `python -m sim.run_sim`。
"""

from __future__ import annotations

import argparse

from . import params
from .camera import Camera
from .link import Annunciator, MavlinkLink, TailFinLink
from .main import main as run


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m bionic_fish", description="跑一趟巡检（实机）"
    )
    parser.add_argument("--config", default=None, help="平的 YAML 参数覆写，见 params.py")
    parser.add_argument("--camera", type=int, default=None, help="相机编号，覆盖 params.camera_index")

    # 后端选择。尾鳍是默认，所以两个开关都默认 False，判据写成 `not args.mavlink` ——
    # 给 `--tail-fin` 一个 `default=True` 会让「默认值」和「用户显式选」混在一起。
    backend = parser.add_mutually_exclusive_group()
    backend.add_argument(
        "--tail-fin",
        action="store_true",
        help="单关节尾鳍模式（默认）：串口直驱舵机 + 波形生成器",
    )
    backend.add_argument(
        "--mavlink",
        action="store_true",
        help="MAVLink 飞控模式：通过 RC_CHANNELS_OVERRIDE 发两根杆位",
    )

    # 设备覆写：两个后端都只有这一个「往哪根线发」的旋钮，含义按后端不同。
    parser.add_argument(
        "--device",
        default=None,
        help="设备路径：尾鳍模式是下位机串口（/dev/ttyUSB0），MAVLink 模式是飞控串口（/dev/ttyACM0）",
    )

    return parser


def _tail_fin_summary(p: params.Params) -> str:
    """尾鳍后端启动时打印的那一行。

    **单独成函数是为了能测**：它格式化 `tail_fin_budget`，而那个字段默认是
    `None` —— 写成 `f"{p.tail_fin_budget:g}"` 会抛
    `TypeError: unsupported format string passed to NoneType.__format__`。
    崩点在**串口连上之后**，所以「连不上就退出」和 `--help` 两条路径都测不到它。
    """
    budget = "关闭" if p.tail_fin_budget is None else f"{p.tail_fin_budget:g}"
    return (
        f"摆动 {p.tail_fin_freq_hz:g} Hz，"
        f"波形更新 {p.tail_fin_wave_rate_hz:g} Hz，"
        f"预算 {budget}（转向权重 {p.tail_fin_yaw_weight:g}）"
    )


def cli(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    p = params.load(args.config)
    if args.camera is not None:
        p.camera_index = args.camera

    # 后端选择：尾鳍是默认，所以只有 `--mavlink` 会翻这个开关
    # （`--tail-fin` 的作用是让 `--tail-fin --mavlink` 这类自相矛盾的命令行报错，
    #  而不是静默地二选一）。
    use_tail_fin = not args.mavlink

    if use_tail_fin:
        # === 单关节尾鳍模式 ===
        if args.device is not None:
            p.tail_fin_serial_device = args.device

        link = TailFinLink(p)
        if not link.connect():
            raise SystemExit(
                f"尾鳍串口连不上（{p.tail_fin_serial_device} @{p.tail_fin_serial_baud}）："
                f"{link.last_error}\n"
                "  - 下位机上电了吗、串口线插好了吗？\n"
                "  - 设备路径对吗（--device）？\n"
                "  - 串口被别的进程占着吗（`lsof /dev/ttyUSB0`）？\n"
                '  - 装了 pyserial 吗？`pip install -e ".[hardware]"`（不带 extras 装不上）。\n'
                "  **这里不会退化成「假装发送」** —— 串口没通就是舵机一帧都收不到，\n"
                "  跑完一整趟也只是白跑。"
            )

        print(f"[尾鳍模式] 串口: {p.tail_fin_serial_device} @{p.tail_fin_serial_baud}")
        print(f"[尾鳍模式] {_tail_fin_summary(p)}")
    else:
        # === MAVLink 飞控模式 ===
        if args.device is not None:
            p.mavlink_device = args.device
        
        link = MavlinkLink(p)
        if not link.connect():
            raise SystemExit(
                f"飞控连不上（{p.mavlink_device}）：{link.last_error}\n"
                "  - 飞控上电了吗、线插好了吗？\n"
                "  - 设备路径对吗（--device）？\n"
                "  - 串口被别的进程占着吗？\n"
                "  - 台架上没有飞控的话，用 `python -m sim.run_sim` 跑仿真。"
            )
        
        print(f"[MAVLink 模式] 飞控: {p.mavlink_device} @{p.mavlink_baud}")

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
        # 一条退出路径漏掉，执行器都会保持最后状态，鱼在没人看的时候继续游。
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
