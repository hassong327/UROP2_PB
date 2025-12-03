"""
ROS2-facing utilities that wrap the shared Bezier core with costmap helpers.

- Core geometry/QP logic lives in `bezier_core`.
- This file adds OccupancyGrid sampling, mixed collision checks, and
  convenience functions for planning from start/goal.
"""

import os
from typing import List, Optional, Sequence, Tuple

from bezier_core import (  # type: ignore
    Obstacle,
    Point,
    bezier_eval,
    clamp_step,
    find_intervals_bezier_hits,
    recursive_push_away as _core_recursive_push_away,  # re-export convenience
    solve_qp,
    vecP,
    visualize_curve_and_hits,
    unvecP,
)


# ---- OccupancyGrid-based collision helpers ----
def world_to_pixel(x, y, og, y_flip=False):
    res = og.info.resolution
    W = og.info.width
    H = og.info.height
    ox = og.info.origin.position.x
    oy = og.info.origin.position.y
    px = int((x - ox) / res - 0.5)
    py = int((H - 1) - ((y - oy) / res - 0.5)) if y_flip else int((y - oy) / res - 0.5)
    return px, py


def value_at(x, y, og, y_flip=False):
    px, py = world_to_pixel(x, y, og, y_flip)
    if 0 <= px < og.info.width and 0 <= py < og.info.height:
        return og.data[py * og.info.width + px]
    return -999  # 맵 밖


def is_occupied_xy(x, y, og, occ_th=65, treat_unknown_as_occ=True, y_flip=False):
    px, py = world_to_pixel(x, y, og, y_flip=y_flip)
    if px < 0 or py < 0 or px >= og.info.width or py >= og.info.height:
        return True  # 맵 밖은 막힌 것으로
    v = og.data[py * og.info.width + px]
    if v < 0:
        return treat_unknown_as_occ
    return v >= occ_th


def sample_curve_hits_pixel(ctrl, og, occ_th=65, treat_unknown_as_occ=True, y_flip=False, S=2000):
    """곡선을 조밀 샘플링하여 점유셀을 밟는 [t0,t1] 구간 반환"""
    hit_ts = []
    for i in range(S + 1):
        t = i / float(S)
        x, y = bezier_eval(ctrl, t)
        if is_occupied_xy(x, y, og, occ_th=occ_th, treat_unknown_as_occ=treat_unknown_as_occ, y_flip=y_flip):
            hit_ts.append(t)
    if not hit_ts:
        return []
    hit_ts.sort()
    intervals = []
    a = hit_ts[0]
    prev = a
    step = 1.0 / float(S)
    tol = 2.5 * step
    for tt in hit_ts[1:]:
        if tt - prev <= tol:
            prev = tt
        else:
            intervals.append((a, prev))
            a = prev = tt
    intervals.append((a, prev))
    return intervals


def find_intervals_bezier_hits_mixed(
    ctrl,
    obstacles,
    og=None,
    flat_eps=1e-2,
    max_depth=30,
    occ_th=65,
    treat_unknown_as_occ=True,
    y_flip=False,
):
    ints_poly = find_intervals_bezier_hits(ctrl, obstacles, flat_eps=flat_eps, max_depth=max_depth)
    ints_pix = []
    if og is not None:
        ints_pix = sample_curve_hits_pixel(ctrl, og, occ_th=occ_th, treat_unknown_as_occ=treat_unknown_as_occ, y_flip=y_flip, S=2000)
    ints = sorted(ints_poly + ints_pix)
    if not ints:
        return []
    merged = [ints[0]]
    for s, e in ints[1:]:
        ps, pe = merged[-1]
        if s <= pe + 1e-6:
            merged[-1] = (ps, max(pe, e))
        else:
            merged.append((s, e))
    return merged


# ===== 재귀형 충돌 해소 (OccupancyGrid 포함) =====
def _total_interval_len(intervals):
    return float(sum(max(0.0, b - a) for (a, b) in intervals))


def recursive_push_away(
    ctrl_init,
    obstacles,
    *,
    og=None,  # OccupancyGrid 래퍼(/map)
    occ_th=65,
    treat_unknown_as_occ=True,
    y_flip=False,
    flat_eps=5e-3,
    max_depth=28,
    rho_init=0.05,
    lambda_smooth=0.5,
    tol_improve=1e-4,
    rho_growth=1.6,
    rho_max=1.0,
    max_calls=12,
    verbose=True,
    plot=True,
    save_dir=None,
    samples=800,
    max_step: float = 0.3,
):
    """
    OccupancyGrid + 기하학 장애물을 모두 고려한 push-away.
    """
    ctrl = list(ctrl_init)

    if plot:
        intervals0 = find_intervals_bezier_hits_mixed(
            ctrl,
            obstacles,
            og=og,
            flat_eps=flat_eps,
            max_depth=max_depth,
            occ_th=occ_th,
            treat_unknown_as_occ=treat_unknown_as_occ,
            y_flip=y_flip,
        )
        visualize_curve_and_hits(
            ctrl,
            obstacles,
            title="Step 0 (initial)",
            intervals=intervals0,
            samples=samples,
            flat_eps=flat_eps,
            max_depth=max_depth,
            show_now=True,
        )

    def _rec(ctrl_cur, step, rho, last_len):
        intervals = find_intervals_bezier_hits_mixed(
            ctrl_cur,
            obstacles,
            og=og,
            flat_eps=flat_eps,
            max_depth=max_depth,
            occ_th=occ_th,
            treat_unknown_as_occ=treat_unknown_as_occ,
            y_flip=y_flip,
        )
        L = _total_interval_len(intervals)
        if verbose:
            print(f"[step {step}] rho={rho:.4f}  hit_len={L:.6f}  intervals={[(round(a,6),round(b,6)) for (a,b) in intervals]}")

        if not intervals:
            return ctrl_cur, [], {"steps": step, "status": "resolved"}

        if step >= max_calls:
            return ctrl_cur, intervals, {"steps": step, "status": "max_calls_reached"}

        Pvec = vecP(ctrl_cur)
        res = solve_qp(Pvec, ctrl_cur, intervals, obstacles, rho=rho, lambda_smooth=lambda_smooth)
        d = res.x
        ctrl2_raw = unvecP(Pvec + d)
        ctrl2 = clamp_step(ctrl_cur, ctrl2_raw, max_step=max_step)

        intervals2 = find_intervals_bezier_hits_mixed(
            ctrl2,
            obstacles,
            og=og,
            flat_eps=flat_eps,
            max_depth=max_depth,
            occ_th=occ_th,
            treat_unknown_as_occ=treat_unknown_as_occ,
            y_flip=y_flip,
        )
        L2 = _total_interval_len(intervals2)
        if verbose:
            print(f"         -> after step {step}: hit_len={L2:.6f}")

        if plot:
            sp = None
            if save_dir:
                os.makedirs(save_dir, exist_ok=True)
                sp = os.path.join(save_dir, f"step_{step+1:02d}.png")
            visualize_curve_and_hits(
                ctrl2,
                obstacles,
                title=f"Step {step+1}",
                intervals=intervals2,
                samples=samples,
                flat_eps=flat_eps,
                max_depth=max_depth,
                show_now=True,
                save_path=sp,
            )

        if not intervals2:
            return ctrl2, [], {"steps": step + 1, "status": "resolved"}

        improved = L2 < L - tol_improve
        if improved:
            return _rec(ctrl2, step + 1, rho, L2)

        new_rho = min(rho_max, rho * rho_growth)
        if new_rho > rho + 1e-12:
            if verbose:
                print(f"         (stalled) increasing rho -> {new_rho:.4f}")
            return _rec(ctrl2, step + 1, new_rho, L2)

        return ctrl2, intervals2, {"steps": step + 1, "status": "stalled"}

    return _rec(ctrl, step=0, rho=rho_init, last_len=float("inf"))


# ======================================================================
#  start / goal 로부터 초기 제어점 생성 + QP 기반 곡선 계획
# ======================================================================
def plan_bezier_from_start_goal(
    *,
    start: Point,
    goal: Point,
    degree: int,
    obstacles: Optional[List[Obstacle]] = None,
    og=None,
    occ_th: int = 65,
    treat_unknown_as_occ: bool = True,
    y_flip: bool = False,
    flat_eps: float = 5e-3,
    max_depth: int = 28,
    rho_init: float = 0.03,
    lambda_smooth: float = 2.0,
    tol_improve: float = 1e-4,
    rho_growth: float = 1.5,
    rho_max: float = 0.4,
    max_calls: int = 8,
    verbose: bool = False,
    plot: bool = False,
    save_dir: Optional[str] = None,
    samples: int = 800,
):
    if obstacles is None:
        obstacles = []

    ctrl0 = init_ctrl_from_start_goal(start, goal, degree)

    if og is not None and len(obstacles) == 0:
        obstacles_from_og = build_circle_obstacles_from_og_along_curve(
            ctrl=ctrl0,
            og=og,
            occ_th=occ_th,
            treat_unknown_as_occ=treat_unknown_as_occ,
            y_flip=y_flip,
            S=400,
            radius_scale=0.0,
        )
        obstacles = obstacles_from_og

    ctrl_final, intervals, info = recursive_push_away(
        ctrl_init=ctrl0,
        obstacles=obstacles,
        og=og,
        occ_th=occ_th,
        treat_unknown_as_occ=treat_unknown_as_occ,
        y_flip=y_flip,
        flat_eps=flat_eps,
        max_depth=max_depth,
        rho_init=rho_init,
        lambda_smooth=lambda_smooth,
        tol_improve=tol_improve,
        rho_growth=rho_growth,
        rho_max=rho_max,
        max_calls=max_calls,
        verbose=verbose,
        plot=plot,
        save_dir=save_dir,
        samples=samples,
    )

    return ctrl_final, intervals, info


def init_ctrl_from_start_goal(start: Point, goal: Point, degree: int) -> List[Point]:
    """
    start~goal 직선을 degree 등분해서 제어점 생성.
    - degree=3 이면 제어점 4개 (P0=start, P3=goal)
    """
    degree = max(1, int(degree))
    sx, sy = start
    gx, gy = goal

    ctrl: List[Point] = []
    for i in range(degree + 1):
        t = i / float(degree)
        x = sx + t * (gx - sx)
        y = sy + t * (gy - sy)
        ctrl.append((x, y))
    return ctrl


def build_circle_obstacles_from_og_along_curve(
    ctrl: Sequence[Point],
    og,
    occ_th: int = 65,
    treat_unknown_as_occ: bool = True,
    y_flip: bool = False,
    S: int = 400,
    radius_scale: float = 0.7,
) -> List[Obstacle]:
    """
    주어진 베지어 곡선(ctrl)을 따라 샘플링하면서,
    점유(또는 unknown) 셀을 만난 위치에 원형 장애물을 생성해서 리턴.
    """
    if og is None:
        return []

    res = og.info.resolution
    obstacles: List[Obstacle] = []
    seen_cells = set()

    for i in range(S + 1):
        t = i / float(S)
        x, y = bezier_eval(ctrl, t)
        px, py = world_to_pixel(x, y, og, y_flip=y_flip)
        if px < 0 or py < 0 or px >= og.info.width or py >= og.info.height:
            continue

        cell_idx = (px, py)
        if cell_idx in seen_cells:
            continue
        seen_cells.add(cell_idx)

        v = og.data[py * og.info.width + px]
        occ = treat_unknown_as_occ if v < 0 else (v >= occ_th)
        if not occ:
            continue

        r = res * radius_scale
        obstacles.append(Obstacle(circle=((x, y), r)))

    return obstacles


# Export a few core helpers for convenience
__all__ = [
    "Obstacle",
    "Point",
    "bezier_eval",
    "find_intervals_bezier_hits",
    "solve_qp",
    "vecP",
    "unvecP",
    "clamp_step",
    "world_to_pixel",
    "value_at",
    "is_occupied_xy",
    "sample_curve_hits_pixel",
    "find_intervals_bezier_hits_mixed",
    "recursive_push_away",
    "plan_bezier_from_start_goal",
    "init_ctrl_from_start_goal",
    "build_circle_obstacles_from_og_along_curve",
    "visualize_curve_and_hits",
    "_core_recursive_push_away",
]
