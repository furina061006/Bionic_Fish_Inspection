"""Vision and control stack for the underwater pipe-inspection challenge.

The package is a refactor of the reference platform in ``reference/``
(see ``.dsh/agents/reference-pipeline.md``), retargeted from red-pipe tracking
and object retrieval to white-pipe tracking and leak-marker reporting under the
rules in ``docs/``.

先把它跑起来，再读循环
----------------------
::

    python -m bionic_fish                # 跑一场合成闭环，打印摘要
    python -m bionic_fish --trace        # 同上，外加每帧一行表

本包里几乎每个模块都在回答「这一步怎么算」，**没有一个模块回答「这些步按什么
次序跑」**。那个次序只存在于一个地方：:meth:`bionic_fish.runtime.Runner._iterate`。
所以先读它，其余都当作细节。写下来的版本是 ``.dsh/agents/runtime-flow.md``，
其中有「跨帧记忆住在哪」一节，解释了为什么一个每帧循环能工作。

模块地图（按循环碰到它们的次序）::

    config.py    全部随部署变化的数字。**最后再读**：里面没有控制流
    arena.py     合成场地与相机 —— 我们目前唯一的帧来源
    vision.py    管道中心线、暗斑候选、起终点黑带
                 -> LineObs / MarkerCand / CrossObs。无策略、无跨帧记忆
    reporting.py 轨迹关联、三道闸、p > 3/8 判决
                 -> ReportEvent，一切对外信号的唯一触发源
    fsm.py       状态机与四条硬终止
                 -> Decision，其中 control 只读三个字段
    control.py   串级横向控制与速度档位
                 -> Actuator，唯一能到硬件的东西
    platform.py  MAVLink 链路、声光与语音（全仓库唯一的硬件出口）
    plant.py     占位动力学，只在合成模式存在。**不要对着它调参**
    runtime.py   组装与主循环 —— **次序就在这里**
    __main__.py  命令行前端（python -m bionic_fish）

本包的三条约定，都是刻意的
--------------------------
1. **每个模块的 docstring 以「在链路里的位置」收尾**：写清调用点（第几步）、
   上游与下游的类型、以及它有没有跨帧状态。那一段用中文，因为它是给本项目的人
   看的导航；周围的技术细节仍用英文，与代码一致。
2. **可调数字一律不出现在 ``config.py`` 之外**（不变量 7）。
3. **没有哪个模块替别的模块做决定**：感知从不决定该不该上报，上报也从不读状态机。
   这一条是「一次漏报能分清是没看见还是被否决」的前提。

命名提醒：代码里沿用了参考项目的 strip 一词（``StripTracker``、``StripParams``），
而实物是**白色管道**。改名要动很多处，暂时只把语义记在
``.dsh/agents/vision.md`` 与 ``config.py`` 的注释里。
"""

from __future__ import annotations

__all__ = ["__version__"]

__version__ = "0.1.0"
