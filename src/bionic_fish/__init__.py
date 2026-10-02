"""水中巡检机器鱼的视觉与控制。

先跑起来，再读循环::

    python -m sim.run_sim    # 仿真，假相机 + 假飞控，不用硬件
    python -m bionic_fish    # 实机（需要真相机与真飞控）

然后**只读 `main.py`** —— 它是一个循环，从上往下读就是鱼在池子里做的事。

模块地图（按你需要的顺序）::

    main.py      主循环。鱼的行为全在这里
    params.py    全部可调参数，默认值照参考项目
    vision.py    看：白管、黑点、起终点黑带
    drive.py     判方向、打舵
    link.py      出：MAVLink + 灯 + 蜂鸣 + 播报
    camera.py    进：相机
    __main__.py  实机入口：连接、解锁、调 main()、上锁

`main(camera, link, p)` **不知道自己在哪跑** —— 它只要求 `camera.read()` 和
`link.send(yaw, forward)`。实机与仿真就是这两个口各塞一套实现，逻辑一行不改。
"""

from __future__ import annotations

__all__ = ["__version__"]

__version__ = "0.2.0"
