"""假相机 + 假飞控。闭环就是把这两个口接起来：

    link.send(yaw, forward)  →  鱼往前动一步
    camera.read()            →  渲染出它现在的位姿

核心（`bionic_fish.main`）不知道这中间没有硬件。
"""

from __future__ import annotations

import numpy as np

from bionic_fish.params import Params

from .arena import Arena
from .plant import PlantParams, VehiclePlant


class FakeCamera:
    """从合成场地渲染帧。时间用虚拟时钟，所以跑得比实时快、而且可复现。"""

    def __init__(self, arena: Arena, plant: VehiclePlant, dt_s: float, max_frames: int) -> None:
        self.arena = arena
        self.plant = plant
        self.dt_s = dt_s
        self.max_frames = max_frames
        self.index = 0

    def read(self) -> tuple[float, np.ndarray] | None:
        if self.index >= self.max_frames:
            return None
        x, y, yaw = self.plant.pose
        frame = self.arena.view(x, y, yaw)
        t = self.index * self.dt_s
        self.index += 1
        return t, frame


class SimLink:
    """假飞控：不发任何东西，只把指令喂回动力学，并记下来。"""

    def __init__(self, plant: VehiclePlant, dt_s: float) -> None:
        self.plant = plant
        self.dt_s = dt_s
        self.sent: list[tuple[float, float]] = []

    def send(self, yaw_us: float, forward_us: float) -> None:
        self.sent.append((float(yaw_us), float(forward_us)))
        self.plant.step(float(yaw_us), float(forward_us), self.dt_s)


def build(p: Params, *, dt_s: float, start_mm: float, lateral_mm: float, max_frames: int):
    """按给定的起始条件装好一整条闭环，返回 `(camera, link, plant, arena)`。"""
    arena = Arena(calibration=p)
    (x0, y0), heading = arena.spec.point_at(start_mm)
    # 横向偏置沿行进方向的法线方向加。
    x0 += -np.sin(heading) * lateral_mm
    y0 += np.cos(heading) * lateral_mm
    plant = VehiclePlant(PlantParams(), p.neutral, float(x0), float(y0), float(heading))
    plant.set_dt(dt_s)
    return FakeCamera(arena, plant, dt_s, max_frames), SimLink(plant, dt_s), plant, arena
