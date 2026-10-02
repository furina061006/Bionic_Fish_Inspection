"""鱼在池子里做的事。**从上往下读，就是这个顺序。**

    连接（在 __main__.py 里）→ 解锁 → ┐
                                      │  每一帧：
                                      │    1. 先查四条硬终止
                                      │    2. 看：白管 / 黑点 / 黑带
                                      │    3. 撞壁计数
                                      │    4. 看到漏油点就报（亮灯 + 播报）
                                      │    5. 判：巡线 / 转弯 / 找线 / 终点
                                      │    6. 打舵 + 发出去
                                      └─ 7. 读下一帧
    → 上锁（在 __main__.py 里）

这个函数**不知道自己在哪跑**：`camera` 只要求有 `read()`，`link` 只要求有
`send(yaw, forward)`。实机塞的是真相机和真飞控，`sim/` 塞的是渲染器和记录器。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .drive import Pid, slew_forward, steer_offset
from .link import Annunciator
from .params import Params
from .vision import find_black_dot, find_white_pipe, is_tape


class Camera(Protocol):
    def read(self) -> tuple[float, np.ndarray] | None: ...


class Link(Protocol):
    def send(self, yaw_us: float, forward_us: float) -> None: ...


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
    ann = annunciator if annunciator is not None else Annunciator(p)
    pid = Pid(p)

    first = camera.read()
    if first is None:
        return Result("相机没有送出任何一帧", 0, 0.0)
    now, frame = first
    t0 = now
    last_now = now

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

        # ================= 1. 四条硬终止 =================
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

        # ================= 2. 看 =================
        pipe = find_white_pipe(frame, p)
        dot = find_black_dot(frame, pipe, p)
        tape = is_tape(frame, pipe, p)

        # ================= 3. 撞壁计数 =================
        # 没有碰撞传感器，用「横向偏差持续饱和」推断。**一次持续饱和只算一次接触** ——
        # 每过一段时间就加一次，会让一次失误直接结束本轮。
        if pipe.valid and abs(pipe.e_y_px) / (p.frame_w / 2.0) >= p.wall_hit_error_frac:
            sat_since = now if sat_since is None else sat_since
            if not sat_counted and now - sat_since >= p.wall_hit_infer_s:
                walls += 1
                sat_counted = True
        else:
            sat_since = None
            sat_counted = False

        # ================= 4. 上报 =================
        # 一次「看见 → 经过」算一个点，压在黑点上的时候只报一次。参考项目靠下降沿
        # 触发，同一块黑点停十几帧就会重复亮灯 —— 重复上报按误报扣 3 分。
        #
        # **复位判据不能写成「黑点离开画面」**：赛题 13 个标记平均间距约 460mm，而
        # 相机视场只有约 400mm，画面里常常同时有两个点。等它完全离开再复位，会把
        # 相邻的真漏油点**合并成一个**（实测：上直道 4 个真点只报了 1 个）。
        #
        # 也不能写成「比上一帧小一点」：那太松，一次瞬时误检就能把**同一个点**报第二遍
        # （实测出现过相隔 14mm 的两次上报）。判据取「黑点回到了车体**前方**」——
        # 报过的点已经在车后（行号 > 290），新车前点必然在画面上半（行号 < 160）。
        if dot is None:
            in_dot = False
        else:
            if dot.v < p.frame_h * 0.5:
                in_dot = False
            if dot.v > p.report_row and not in_dot and reports < p.max_reports:
                reports += 1
                ann.on_report(reports, now)
                in_dot = True
        ann.tick(now)

        # ================= 5. 判 =================
        if tape and elapsed > p.finish_min_run_s:
            # 起点黑带长得和终点一模一样，所以必须「已经跑够久」才认它。
            finish_seen_at = now if finish_seen_at is None else finish_seen_at
            state = FINISH
        elif state is FINISH:
            pass                              # 已经在终点区里蠕行，不再改判
        elif pipe.valid and state not in (TURN, SEARCH) and bend_done_at is not None \
                and now - bend_done_at < p.corner_refractory_s:
            pass                              # 刚转完的不应期：灯带横向出画这个判据仍成立
        elif state is TURN:
            # 转弯的终止判据：转过最短时间之后，重新看见管道就结束。
            #
            # 「最短时间」这一条是踩坑来的：弯角处管道在画面里是个 L 形，行扫描拟合
            # 会锁到上一段直道的残余，报出很小的航向误差。就此判定「弯转完了」，
            # 实测**只转了 4.4°** 就会径直冲过拐角。参考项目没有这一条。
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

        # ================= 6. 打舵 + 发 =================
        if state is FINISH:
            target_forward = p.cruise
        elif state is TURN:
            target_forward = p.turn_slow
        elif state is SEARCH:
            target_forward = p.lost_slow
        else:
            target_forward = p.cruise

        if state is TURN:
            # 转弯是**开环方向 + 闭环时长**：舵量固定，什么时候结束由上面那段判据决定。
            yaw = p.neutral + p.corner_turn_us
            pid.hold(now)
        elif pipe.valid:
            yaw = p.neutral + steer_offset(pipe, p, pid, now)
        else:
            # 保持上一帧的舵量（`yaw` 不重新赋值）。**绝不把「没看到」当成「已经对准」**
            # —— 那会让鱼直直冲向池壁，而「撞壁 3 次」是硬终止。
            pid.hold(now)

        forward = slew_forward(forward, target_forward, p, dt)
        link.send(yaw, forward)
        if abs(forward - p.neutral) > p.started_forward_us:
            started = True

        # 终点区里再挪一段，让整条鱼压过黑带。
        if state is FINISH and finish_seen_at is not None and now - finish_seen_at >= p.finish_creep_s:
            return Result("到达终点", reports, elapsed)

        # ================= 7. 读下一帧 =================
        nxt = camera.read()
        if nxt is None:
            return Result("相机断了", reports, elapsed)
        now, frame = nxt
