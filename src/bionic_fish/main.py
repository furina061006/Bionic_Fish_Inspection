"""鱼在池子里做的事。**从上往下读，就是这个顺序。**

折叠起来应该只剩这副骨架::

    def main(camera, link, p, annunciator=None) -> Result:
        # ========== 准备 ==========
        while True:
            # ========== 1. 四条硬终止 ==========
            # ========== 2. 看 ==========
            # ========== 3. 撞壁计数 ==========
            # ========== 4. 上报 ==========
            # ========== 5. 判 ==========
            # ========== 6. 打舵 + 发 ==========
            # ========== 7. 读下一帧 ==========

展开每一节就是它的理由。**每节的说明集中写在节标题下面**，所以折叠是干净的。

这个函数**不知道自己在哪跑**：`camera` 只要求有 `read()`，`link` 只要求有
`send(yaw, forward)`。实机塞的是真相机和真飞控，`sim/` 塞的是渲染器和记录器。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .drive import Pid, slew_forward, steer_offset, synthesize_tail_fin
from .link import Annunciator
from .params import Params
from .vision import find_black_dot, find_white_pipe, is_tape


class Camera(Protocol):
    def read(self) -> tuple[float, np.ndarray] | None: ...


class Link(Protocol):
    def send(self, yaw_us: float, forward_us: float) -> None: ...


class TailFin(Protocol):
    """单关节尾鳍后端（运行时是 `link.TailFinLink`）。`main.py` 用 `hasattr` 认它。

    故意**不叫** `TailFinLink` —— 那会和 `link.TailFinLink` 同名，读代码时分不清
    哪个是契约、哪个是实现。

    没有 `send(yaw, forward)`：合成只发生在 `main.py` 一处。
    """

    def send_tail_fin(self, amplitude: float, bias: float) -> None: ...


@dataclass
class Result:
    """为什么停下来，报了几个点，用了多久。两个壳各自打印它。"""

    reason: str
    reports: int
    elapsed_s: float


#: 巡线时这个状态的一切正常。
FOLLOW = "巡线"
TURN = "转弯"
SEARCH = "找线"
FINISH = "终点"


def main(camera: Camera, link: Link, p: Params, annunciator: Annunciator | None = None) -> Result:
    # ======================================================================
    # 准备
    # ======================================================================
    # 下面两个有状态，所以建在循环外面。不传 annunciator 时给的是一个 gpio=None
    # 的默认实例 —— tick() 直接返回、on_report() 什么都不写，测试不会真去点灯。
    ann = annunciator if annunciator is not None else Annunciator(p)
    pid = Pid(p)

    # 先读一帧。「相机没有送出任何一帧」是唯一一条**还没开始就结束**的路径。
    first = camera.read()
    if first is None:
        return Result("相机没有送出任何一帧", 0, 0.0)
    now, frame = first
    t0 = now                            # 所有计时从这里算起
    last_now = now                      # 下一帧的 dt 要减它

    # 跨帧记忆 —— 这一趟鱼的全部状态就是下面这些。它们**故意留在这里**而不是藏进
    # 别的对象：一个「每帧转一圈」的循环能读懂，靠的就是能一眼看全它们。
    state = FOLLOW
    forward = float(p.neutral)          # 实际发出去的前进量，受斜率限制
    yaw = float(p.neutral)              # 实际发出去的舵量；丢线时保持不变
    lost_since: float | None = None     # 从哪一刻开始看不见管道
    in_dot = False                      # 正压在一个黑点上（一次「看见-经过」只报一次）
    walls = 0                           # 推断出的撞壁次数
    sat_since: float | None = None      # 横向偏差从哪一刻开始饱和
    sat_counted = False                 # 这段饱和是不是已经算过一次
    started = False                     # 鱼有没有真的动过
    reports = 0
    turn_started: float | None = None   # 本次转弯什么时候开始的
    bend_done_at: float | None = None   # 上一次转弯什么时候结束的（不应期用）
    finish_seen_at: float | None = None # 第一次看到终点黑带

    while True:
        dt = max(1e-6, now - last_now)
        last_now = now
        elapsed = now - t0

        # ==================================================================
        # 1. 四条硬终止
        # ==================================================================
        # 放在最前面，任何状态逻辑都绕不过去。四条全部来自赛题 PDF，
        # **参考项目一条都没有** —— 它的主循环是 while 1。
        #
        # 次序与条件都有意义：「未启动」排在「脱离路线」之前，而且脱线**只在鱼动过
        # 之后才算**。没动过的鱼谈不上「脱离路线」，可空画面会让两条同时成立 ——
        # 若不这样，一条从没启动的鱼会被记成「跑丢了」，读日志的人会往错的方向查。
        if elapsed > p.run_timeout_s:
            return Result(f"超过 {p.run_timeout_s:g} 秒上限", reports, elapsed)
        if walls >= p.wall_hit_limit:
            return Result(f"触碰池底/池壁满 {p.wall_hit_limit} 次", reports, elapsed)
        if not started and elapsed > p.start_timeout_s:
            return Result(f"宣布开始后 {p.start_timeout_s:g} 秒未启动", reports, elapsed)
        if started and lost_since is not None and now - lost_since > p.lost_timeout_s:
            return Result(f"完全脱离路线超过 {p.lost_timeout_s:g} 秒", reports, elapsed)

        # ==================================================================
        # 2. 看
        # ==================================================================
        # 三个纯函数，没有跨帧状态，所以能逐帧单独测。`find_black_dot` 吃 `pipe` ——
        # 候选必须落在管道附近，所以管道必须先找到。
        pipe = find_white_pipe(frame, p)
        dot = find_black_dot(frame, pipe, p)
        tape = is_tape(frame, pipe, p)

        # ==================================================================
        # 3. 撞壁计数
        # ==================================================================
        # 没有碰撞传感器，用「横向偏差持续饱和」推断。管道看不见时 abs() 没意义，
        # 所以先要求 valid。
        #
        # **一次持续饱和只算一次接触**（靠 sat_counted）：每过一段时间就加一次，
        # 会让一次失误直接结束本轮。
        if pipe.valid and abs(pipe.e_y_px) / (p.frame_w / 2.0) >= p.wall_hit_error_frac:
            sat_since = now if sat_since is None else sat_since
            if not sat_counted and now - sat_since >= p.wall_hit_infer_s:
                walls += 1
                sat_counted = True
        else:
            sat_since = None
            sat_counted = False

        # ==================================================================
        # 4. 上报
        # ==================================================================
        # 一次「看见 → 经过」算一个点；压在黑点上的时候只报一次。参考项目靠下降沿
        # 触发，同一块黑点停十几帧就会重复亮灯 —— 重复上报按误报扣 3 分。
        #
        # 复位判据（`dot.v < 画面高的一半`）**不能**写成「黑点离开画面」：赛题 13 个
        # 标记平均间距约 460mm，而相机视场只有约 400mm，画面里常常同时有两个点 ——
        # 等它完全离开再复位，会把相邻的真漏油点**合并成一个**（实测：上直道 4 个
        # 真点只报了 1 个）。
        #
        # 也**不能**写成「比上一帧小一点」：那太松，一次瞬时误检就能把**同一个点**
        # 报第二遍（实测出现过相隔 14mm 的两次上报）。所以判据取「黑点回到了车体
        # 前方」—— 报过的点已经在车后（行号 > 290），新车前点必然在画面上半。
        if dot is None:
            in_dot = False
        else:
            if dot.v < p.frame_h * 0.5:
                in_dot = False
            if dot.v > p.report_row and not in_dot and reports < p.max_reports:
                reports += 1
                ann.on_report(reports, now)
                in_dot = True
        ann.tick(now)               # 到点关灯；每帧都调，所以按住信号不阻塞控制

        # ==================================================================
        # 5. 判
        # ==================================================================
        # 一整个 if/elif 链，**顺序就是优先级**：
        #     终点 > 已在终点 > 不应期 > 转弯 > 丢线 > 弯角线索 > 巡线
        #
        # 其中两条顺序有语义：
        #   · 终点要「已经跑够时间」才认 —— 起点黑带长得和终点一模一样
        #   · **转弯排在丢线之前** —— 管道在弯中途必然消失；若丢线分支在前面，
        #     唯一能结束转向的机制会在最需要它的时候失效
        if tape and elapsed > p.finish_min_run_s:
            finish_seen_at = now if finish_seen_at is None else finish_seen_at
            state = FINISH
        elif state is FINISH:
            pass                    # 已经在终点区里蠕行，不再改判
        elif pipe.valid and state not in (TURN, SEARCH) and bend_done_at is not None \
                and now - bend_done_at < p.corner_refractory_s:
            pass                    # 不应期：刚转完时「管道横向出画」这个判据仍成立
        elif state is TURN:
            # 转过最短时间之后，重新看见管道就结束。
            # 「最短时间」是踩坑来的：弯角处管道在画面里是个 L 形，行扫描拟合会锁到
            # 上一段直道的残余、报出很小的航向误差。就此判定「弯转完了」，实测
            # **只转了 4.4°** 就会径直冲过拐角。参考项目没有这一条。
            if turn_started is not None and now - turn_started >= p.corner_min_commit_s and pipe.valid:
                state = FOLLOW
                bend_done_at = now
                turn_started = None
            elif turn_started is not None and now - turn_started > p.corner_timeout_s:
                lost_since = lost_since if lost_since is not None else now
                state = SEARCH
                turn_started = None
        elif not pipe.valid:
            lost_since = lost_since if lost_since is not None else now
            state = SEARCH
        elif (
            pipe.confidence < p.corner_confidence_min or abs(pipe.curvature) > p.corner_curve_max
        ):
            state = TURN
            turn_started = now
            lost_since = None
        else:
            state = FOLLOW
            lost_since = None

        # ==================================================================
        # 6. 打舵 + 发
        # ==================================================================
        # 前进量按状态选一档：巡航 / 转弯慢速 / 丢线慢速 / 终点巡航。
        #
        # 舵量有三条分支，第三条是硬终止的防线：**丢线时 `yaw` 不重新赋值**，于是它
        # 自然保持上一帧。绝不把「没看到」当成「已经对准」—— 那会让鱼直直冲向池壁，
        # 而「撞壁 3 次」就是结束。
        #
        # 最后 `started` 用的是**实际发出的**前进量，不是目标值 ——
        # 「30 秒未启动」判的是执行器动了，不是状态机想要动。
        if state is FINISH:
            target_forward = p.cruise
        elif state is TURN:
            target_forward = p.turn_slow
        elif state is SEARCH:
            target_forward = p.lost_slow
        else:
            target_forward = p.cruise

        if state is TURN:
            # 转弯是**开环方向 + 闭环时长**：舵量固定，什么时候结束由第 5 步决定。
            yaw = p.neutral + p.corner_turn_us
            pid.hold(now)
        elif pipe.valid:
            yaw = p.neutral + steer_offset(pipe, p, pid, now)
        else:
            pid.hold(now)           # 保持上一帧的 yaw，只推进 PID 的时钟

        # 从当前值向目标值限速逼近，再发出去。全循环只有这一次对外。
        #
        # 两个后端，**只有这一处分叉**：
        #   · 尾鳍：先把 yaw/forward 合成成一个关节指令 (摆幅, 偏置) —— 单关节
        #     尾鳍的推力和转向出自同一个执行器，必须一起决策（耦合 12）
        #   · 飞控 / 仿真：照旧发两根杆位
        forward = slew_forward(forward, target_forward, p, dt)

        if hasattr(link, "send_tail_fin"):
            amplitude, bias = synthesize_tail_fin(yaw, forward, p)
            link.send_tail_fin(amplitude, bias)
        else:
            link.send(yaw, forward)

        if abs(forward - p.neutral) > p.started_forward_us:
            started = True

        # 发完之后：终点区里再挪一段，让整条鱼压过黑带。
        if state is FINISH and finish_seen_at is not None and now - finish_seen_at >= p.finish_creep_s:
            return Result("到达终点", reports, elapsed)

        # ==================================================================
        # 7. 读下一帧
        # ==================================================================
        # 返回 None = 相机断了。回到循环开头，第 1 步会重新查四条硬终止。
        nxt = camera.read()
        if nxt is None:
            return Result("相机断了", reports, elapsed)
        now, frame = nxt
