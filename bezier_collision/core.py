from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np


@dataclass
class RectObstacle:
    xmin: float
    xmax: float
    ymin: float
    ymax: float

    def contains(self, x: float, y: float) -> bool:
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax


@dataclass
class CircleObstacle:
    cx: float
    cy: float
    r: float

    def contains(self, x: float, y: float) -> bool:
        return (x - self.cx) ** 2 + (y - self.cy) ** 2 <= self.r ** 2


def bezier_point(ctrl: Sequence[Sequence[float]], t: float) -> np.ndarray:
    """Evaluate a Bézier curve at parameter t using De Casteljau."""
    ctrl = np.asarray(ctrl, dtype=float)
    n = len(ctrl) - 1
    points = ctrl.copy()
    for r in range(1, n + 1):
        points[: n - r + 1] = (1 - t) * points[: n - r + 1] + t * points[1 : n - r + 2]
    return points[0]


def world_to_grid(x: float, y: float, origin_x: float, origin_y: float, resolution: float) -> Tuple[int, int]:
    ix = int((x - origin_x) / resolution)
    iy = int((y - origin_y) / resolution)
    return ix, iy


def is_colliding_point_grid(
    x: float,
    y: float,
    grid: np.ndarray,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    resolution: float = 1.0,
) -> bool:
    ix, iy = world_to_grid(x, y, origin_x, origin_y, resolution)
    if ix < 0 or iy < 0 or ix >= grid.shape[1] or iy >= grid.shape[0]:
        return False
    return grid[iy, ix] != 0


def is_colliding_point_shapes(x: float, y: float, obstacles: Sequence[object]) -> bool:
    for obs in obstacles:
        if hasattr(obs, "contains") and obs.contains(x, y):
            return True
    return False


def find_collision_intervals(
    ctrl: Sequence[Sequence[float]],
    obstacles: Sequence[object],
    n_samples: int = 200,
    use_arc_length: bool = False,
    use_grid: bool = False,
    grid: np.ndarray | None = None,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    resolution: float = 1.0,
) -> Tuple[
    List[Tuple[float, float]],
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    List[Tuple[int, int]],
]:
    t_vals = np.linspace(0.0, 1.0, n_samples)
    pts = np.array([bezier_point(ctrl, t) for t in t_vals])

    collisions = []
    for x, y in pts:
        if use_grid and grid is not None:
            c = is_colliding_point_grid(x, y, grid, origin_x, origin_y, resolution)
        else:
            c = is_colliding_point_shapes(x, y, obstacles)
        collisions.append(c)
    collisions = np.array(collisions, dtype=bool)

    if use_arc_length:
        seg_len = np.linalg.norm(pts[1:] - pts[:-1], axis=1)
        cum_len = np.concatenate([[0.0], np.cumsum(seg_len)])
        tau_vals = np.zeros_like(t_vals) if cum_len[-1] <= 0 else cum_len / cum_len[-1]
    else:
        tau_vals = t_vals

    intervals: List[Tuple[float, float]] = []
    interval_indices: List[Tuple[int, int]] = []
    in_block = False
    start_idx = 0
    for i, flag in enumerate(collisions):
        if flag and not in_block:
            in_block = True
            start_idx = i
        if (not flag) and in_block:
            in_block = False
            end_idx = i - 1
            intervals.append((float(tau_vals[start_idx]), float(tau_vals[end_idx])))
            interval_indices.append((start_idx, end_idx))
    if in_block:
        intervals.append((float(tau_vals[start_idx]), float(tau_vals[-1])))
        interval_indices.append((start_idx, len(tau_vals) - 1))

    return intervals, t_vals, pts, collisions, tau_vals, interval_indices


def collision_segment_endpoints(
    ctrl: Sequence[Sequence[float]],
    t_vals: np.ndarray,
    interval_indices: List[Tuple[int, int]],
    intervals_tau: List[Tuple[float, float]],
) -> List[Tuple[Tuple[float, float], np.ndarray, np.ndarray]]:
    """Return tau interval with its start/end points; uses true t for evaluation."""
    segments: List[Tuple[Tuple[float, float], np.ndarray, np.ndarray]] = []
    for (idx_s, idx_e), (tau_s, tau_e) in zip(interval_indices, intervals_tau):
        t_s = float(t_vals[idx_s])
        t_e = float(t_vals[idx_e])
        p_start = bezier_point(ctrl, t_s)
        p_end = bezier_point(ctrl, t_e)
        segments.append(((tau_s, tau_e), p_start, p_end))
    return segments


def circle_radius_to_obstacles(center: np.ndarray, obstacles: Sequence[object]) -> float:
    """Compute smallest radius from center to first obstacle corner/boundary contact."""
    cx, cy = center
    radii = []
    for obs in obstacles:
        if isinstance(obs, RectObstacle):
            corners = [
                (obs.xmin, obs.ymin),
                (obs.xmin, obs.ymax),
                (obs.xmax, obs.ymin),
                (obs.xmax, obs.ymax),
            ]
            for vx, vy in corners:
                radii.append(float(np.hypot(vx - cx, vy - cy)))
        elif isinstance(obs, CircleObstacle):
            dist_center = np.hypot(obs.cx - cx, obs.cy - cy)
            radii.append(max(float(dist_center - obs.r), 0.0))
    if not radii:
        return 0.0
    return min(radii)


__all__ = [
    "RectObstacle",
    "CircleObstacle",
    "bezier_point",
    "world_to_grid",
    "is_colliding_point_grid",
    "is_colliding_point_shapes",
    "find_collision_intervals",
    "collision_segment_endpoints",
    "circle_radius_to_obstacles",
]
