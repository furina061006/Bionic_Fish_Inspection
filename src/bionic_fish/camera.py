"""进：从相机拿一帧。

这是通向外界的**两个口之一**（另一个是 `link.py`）。核心的其余部分只要求对象有
`read()`，所以 `sim/` 能塞一个渲染器进来而不改任何逻辑。
"""

from __future__ import annotations

import time

import numpy as np


class Camera:
    """A live camera, read one frame at a time.

    `read()` returns `(timestamp_s, frame)` or `None` when the stream ends.  The
    timestamp is `time.monotonic` — the same clock the loop uses to derive `dt`.
    A wall-clock timestamp would make every derivative term depend on how the
    system clock happened to be set.
    """

    def __init__(self, index: int = 0) -> None:
        import cv2

        self.index = index
        self._capture = cv2.VideoCapture(index)
        if not self._capture.isOpened():
            raise RuntimeError(
                f"camera {index} did not open - check /dev/video{index}, that nothing "
                "else holds it, and that the user is in the video group"
            )
        self._started = time.monotonic()

    def read(self) -> tuple[float, np.ndarray] | None:
        ok, frame = self._capture.read()
        if not ok:
            return None
        return time.monotonic() - self._started, frame

    def close(self) -> None:
        self._capture.release()
