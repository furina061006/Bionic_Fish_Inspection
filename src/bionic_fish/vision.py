"""看：白管在哪、有没有黑色小圆、前面是不是起终点黑带。

三个函数，各回答一个问题。**它们不做任何决定** —— 不上报、不转向。看不出东西时
返回 `None` 或 `valid=False`，绝不返回一个「看起来像 0」的值。

实物是**白色圆柱形管道**（原文 p3 写作「池底 LED 灯带」，指同一样东西）。注意它是
**被动反光**而不是自发光，所以：亮度取决于照明，而且管顶会有一条**比管壁更亮、更窄**
的镜面高光带 —— 那条高光带是这里最需要防的假目标，靠「游程宽度窗口」挡掉。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache

import cv2
import numpy as np

from .params import Params


@dataclass
class LineObs:
    """白管相对车体的位置。所有横向/角度量：**正 = 在车的右边**。"""

    valid: bool
    #: 前瞻点的横向偏差：管道中心 x 减画面中心 x，像素。
    e_y_px: float = 0.0
    #: 管道方向与车体前进方向的夹角，弧度。正 = 管道朝右。
    e_theta_rad: float = 0.0
    #: 管道的弯曲程度，弧度/行。直角处会突然变大。
    curvature: float = 0.0
    #: 管道在画面里的中位宽度，像素。
    width_px: float = 0.0
    #: 有贡献的行占采样行的比例。灯带转成 L 形时它会塌下来。
    confidence: float = 0.0
    #: 前瞻点到车体的前向距离，像素。纯追踪的视界。
    lookahead_px: float = 1.0
    #: 拟合直线 `x = slope*row + intercept`。
    fit: tuple[float, float] = (0.0, 0.0)

    def x_at(self, row: float) -> float:
        """拟合出的管道中心在某一行的 x。"""
        slope, intercept = self.fit
        return slope * row + intercept


@dataclass
class Dot:
    """一个通过几何筛选的黑色小圆。"""

    u: float
    v: float
    diameter_px: float
    #: 圆内部的平均明度（OpenCV HSV 的 V，0-255）。漏油点应当是**很黑**的值。
    val: float


# --------------------------------------------------------------------------- #
# 白管
# --------------------------------------------------------------------------- #


@lru_cache(maxsize=8)
def _kernels(open_ksize: int, close_ksize: int, tophat_ksize: int, bh_ksize: int):
    """形态学核。缓存，因为每帧重建一个 31×31 椭圆是纯浪费。"""
    ellipse = cv2.MORPH_ELLIPSE
    return (
        cv2.getStructuringElement(ellipse, (open_ksize, open_ksize)),
        cv2.getStructuringElement(ellipse, (close_ksize, close_ksize)),
        cv2.getStructuringElement(ellipse, (tophat_ksize, tophat_ksize)),
        cv2.getStructuringElement(ellipse, (bh_ksize, bh_ksize)),
    )


def _close_ksize(p: Params) -> int:
    """闭运算核。

    必须**大于漏油点的直径**：标记就贴在管子上，会在管子的掩膜里打一个洞，而一个
    被截断的游程，其中心会偏向存活的那一侧 —— 直接把中心线拉偏。这不是平滑，是
    正确性要求。
    """
    if p.close_ksize > 0:
        return p.close_ksize
    return int(max(9, round(p.marker_diameter_px * 1.6)))


def _mask(frame: np.ndarray, p: Params) -> np.ndarray:
    """白管的二值掩膜。"""
    _, close_kernel, tophat_kernel, _ = _kernels(
        p.open_ksize, _close_ksize(p), p.tophat_ksize, p.blackhat_ksize
    )
    if p.strip_method == "tophat":
        # 「亮**局部**结构」：把每个像素和它自己的邻域比。水下衰减会给整幅图染色、
        # 光照还会随位置渐变，所以绝对阈值只有很薄的一点余量。
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        response = cv2.morphologyEx(grey, cv2.MORPH_TOPHAT, tophat_kernel)
        mask = (response >= p.tophat_threshold).astype(np.uint8) * 255
    elif p.strip_method == "bright_low_sat":
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        _, s, v = cv2.split(hsv)
        mask = ((v >= p.v_min) & (s <= p.s_max)).astype(np.uint8) * 255
    else:  # red_minus_blue —— 参考项目找偏红管道的判据，留作对照
        b, _, r = cv2.split(frame.astype(np.int16))
        mask = ((r - b) >= p.r_minus_b_min).astype(np.uint8) * 255

    open_kernel, _, _, _ = _kernels(p.open_ksize, _close_ksize(p), p.tophat_ksize, p.blackhat_ksize)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)


def _run_window(p: Params) -> tuple[float, float]:
    return max(2.0, p.strip_width_px * p.run_ratio_lo), max(3.0, p.strip_width_px * p.run_ratio_hi)


def _row_run(row: np.ndarray, x0: int, lo: float, hi: float, hint: float | None) -> tuple[float, float] | None:
    """在一行里挑出最像白管的那个游程。

    比 `lo` 窄的是噪声；比 `hi` 宽的是镜面反光片或弯角处横过来的肘部。活下来的
    里面，取**离 hint 最近**的那个（hint = 更靠近车的那一行找到的中心），这样画面里
    同时出现两个宽度相近的东西时，跟踪不会跳。
    """
    if not row.any():
        return None
    padded = np.concatenate(([0], row, [0]))
    edges = np.flatnonzero(np.diff(padded.astype(np.int8)))
    if edges.size < 2:
        return None
    starts, ends = edges[0::2], edges[1::2]
    lengths = (ends - starts).astype(np.float64)
    keep = (lengths >= lo) & (lengths <= hi)
    if not keep.any():
        return None
    centres = x0 + (starts + ends - 1) / 2.0
    if hint is None:
        best = int(np.argmax(np.where(keep, lengths, -1.0)))
    else:
        best = int(np.argmin(np.where(keep, np.abs(centres - hint), np.inf)))
    return float(centres[best]), float(lengths[best])


def _fit(rows: np.ndarray, xs: np.ndarray) -> tuple[float, float]:
    """最小二乘拟合 `x = slope*row + intercept`。"""
    if rows.size < 2 or float(np.ptp(rows)) == 0.0:
        return 0.0, float(xs.mean()) if xs.size else 0.0
    slope, intercept = np.polyfit(rows, xs, 1)
    return float(slope), float(intercept)


def find_white_pipe(frame: np.ndarray, p: Params) -> LineObs:
    """找到白管中心线。看不见就返回 `valid=False`。"""
    h, w = frame.shape[:2]
    x1 = int(round((1.0 - p.roi_side_frac) * w))
    x0 = int(round(p.roi_side_frac * w))
    y1 = int(round(p.roi_bottom_frac * h))
    y0 = int(round(p.roi_top_frac * h))
    sub = _mask(frame, p)[y0:y1, x0:x1]
    lo, hi = _run_window(p)

    rows: list[float] = []
    xs: list[float] = []
    widths: list[float] = []
    step = 2
    hint: float | None = None
    for r in range(sub.shape[0] - 1, -1, -step):
        found = _row_run(sub[r], x0, lo, hi, hint)
        if found is None:
            continue
        cx, length = found
        if hint is not None and abs(cx - hint) > p.continuity_window_px:
            # 拒绝跳变：留着一个错的东西比留着上一个中心更糟。
            continue
        hint = cx
        rows.append(float(y0 + r))
        xs.append(cx)
        widths.append(length)

    sampled = max(1, len(range(sub.shape[0] - 1, -1, -step)))
    if len(rows) < p.min_rows:
        return LineObs(valid=False, confidence=len(rows) / sampled)

    row_arr = np.asarray(rows)
    x_arr = np.asarray(xs)

    # 一次剔除的鲁棒直线拟合。**只用一条直线是对的**：弯角处应当由曲率判据拒绝，
    # 而不是靠把直线掰弯去迁就它。
    slope, intercept = _fit(row_arr, x_arr)
    keep = np.abs(x_arr - (slope * row_arr + intercept)) <= p.fit_residual_px
    if keep.sum() < p.min_rows:
        return LineObs(valid=False, confidence=len(rows) / sampled)
    slope, intercept = _fit(row_arr[keep], x_arr[keep])
    width_px = float(np.median(np.asarray(widths)[keep]))

    # 前瞻点：ROI 上方的一行，也就是车**前方**。这就是纯追踪的目标点。
    lookahead_row = p.lookahead_frac * h
    e_y_px = slope * lookahead_row + intercept - w / 2.0
    # slope 是 dx/drow，而行号朝车体方向增大，所以取负。
    e_theta = -math.atan(slope)

    # 曲率 = 远端三分之一与近端三分之一的朝向差。直角在这里会变成一个很大的值。
    near = row_arr[keep] > (y0 + y1) / 2.0
    curvature = 0.0
    if near.any() and (~near).any():
        s_near, _ = _fit(row_arr[keep][near], x_arr[keep][near])
        s_far, _ = _fit(row_arr[keep][~near], x_arr[keep][~near])
        gap = float(abs(np.median(row_arr[keep][~near]) - np.median(row_arr[keep][near])))
        if gap > 1.0:
            curvature = (math.atan(s_far) - math.atan(s_near)) / gap

    return LineObs(
        valid=True,
        e_y_px=float(e_y_px),
        e_theta_rad=float(e_theta),
        curvature=float(curvature),
        width_px=width_px,
        confidence=float(keep.sum() / sampled),
        lookahead_px=max(1.0, h / 2.0 - lookahead_row),
        fit=(float(slope), float(intercept)),
    )


# --------------------------------------------------------------------------- #
# 漏油点
# --------------------------------------------------------------------------- #


def find_black_dot(frame: np.ndarray, pipe: LineObs, p: Params) -> Dot | None:
    """找**最靠近车**（画面最下方）的那个黑色小圆。

    用黑帽变换找比邻域暗的团块，再过几何闸：面积、圆度、长宽比、以及「必须落在
    管道中心线附近」。最后按内部明度过滤 —— 见 `params.dot_val_max`。
    """
    _, _, _, bh_kernel = _kernels(
        p.open_ksize, _close_ksize(p), p.tophat_ksize, p.blackhat_ksize
    )
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    response = cv2.morphologyEx(grey, cv2.MORPH_BLACKHAT, bh_kernel)
    binary = (response >= p.blackhat_threshold).astype(np.uint8)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, 8)
    if count <= 1:
        return None

    area_lo, area_hi = p.marker_area_px
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    max_offset = p.max_lateral_offset_frac * p.frame_w
    best: Dot | None = None

    for i in range(1, count):
        area = float(stats[i, cv2.CC_STAT_AREA])
        if not (area_lo <= area <= area_hi):
            continue
        component = (labels == i).astype(np.uint8)
        contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0.0:
            continue
        if 4.0 * math.pi * area / (perimeter * perimeter) < p.circularity_min:
            continue
        bw, bh = cv2.boundingRect(contour)[2:]
        if bw == 0 or bh == 0 or max(bw, bh) / float(min(bw, bh)) > p.aspect_max:
            continue

        u, v = float(centroids[i][0]), float(centroids[i][1])
        # 候选必须落在管道附近：标记是沿路线布置的。
        if pipe.valid and abs(u - pipe.x_at(v)) > max_offset:
            continue

        diameter = math.sqrt(4.0 * area / math.pi)
        radius = max(1, int(round(diameter / 2.0)))
        disc = np.zeros_like(component)
        cv2.circle(disc, (int(round(u)), int(round(v))), radius, 1, -1)
        inner = cv2.erode(disc, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))).astype(bool)
        val = float(hsv[:, :, 2][inner].mean()) if inner.any() else 0.0
        if val > p.dot_val_max:
            continue

        # 取最靠下的那个（离车最近，也就是马上要经过的）。
        if best is None or v > best.v:
            best = Dot(u=u, v=v, diameter_px=float(diameter), val=val)
    return best


# --------------------------------------------------------------------------- #
# 起终点黑带
# --------------------------------------------------------------------------- #


def is_tape(frame: np.ndarray, pipe: LineObs, p: Params) -> bool:
    """画面里有没有一条横穿管道的黑色胶带。

    黑带是全场最危险的假漏油点：它也是黑的、比标记宽、而且必然出现。用**绝对暗度**
    而不是黑帽响应 —— 黑帽在亮结构的边缘也会响应，于是紧贴管道处会把管道自己的边界
    报成一条宽黑带，而在弯角那里这个假象和真黑带无法区分。

    四条物理约束缺一不可：宽度像带子、**必须横跨管道两侧**、沿行进方向足够薄、
    而且不能贴着画面边缘（池壁外的暗区会同时满足前三条）。
    """
    if not pipe.valid:
        # 「横穿管道」这个事件没有管道就没有定义。退化成拿画面中心去找暗区，会让
        # 任何恰好压在中间的暗块变成「终点」——池壁外的暗区就是这样骗过它的。
        return False

    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dark = grey <= p.tape_max_grey
    passing: dict[int, float] = {}
    for row in range(p.frame_h):
        centre_x = pipe.x_at(row)
        col = int(min(max(round(centre_x), 0), p.frame_w - 1))
        if not dark[row, col]:
            continue
        left = col
        while left > 0 and dark[row, left - 1]:
            left -= 1
        right = col
        while right < p.frame_w - 1 and dark[row, right + 1]:
            right += 1
        width = float(right - left + 1)
        if not (p.tape_width_px_lo <= width <= p.tape_width_px_hi):
            continue
        # 必须横跨管道两侧。漏油点也是黑的，但它只有一个直径宽，过不了这一关。
        margin = p.strip_width_px
        if left <= centre_x - margin and right >= centre_x + margin:
            passing[row] = width

    if not passing:
        return False
    # 取离画面中心（= 车体正下方）最近的那条。
    centre_row = min(passing, key=lambda r: abs(r - p.frame_h / 2.0))
    row_lo, row_hi = centre_row, centre_row
    while (row_lo - 1) in passing:
        row_lo -= 1
    while (row_hi + 1) in passing:
        row_hi += 1

    if (row_hi - row_lo + 1) > p.tape_thickness_px_max:
        return False  # 一大片暗区，不是带子
    if row_lo <= 0 or row_hi >= p.frame_h - 1:
        return False  # 被画面边缘切掉了，是池壁外，不是带子
    return True
