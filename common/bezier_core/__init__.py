"""
Shared Bezier path-planning core.

This module contains the geometry, collision checks, and QP-based
push-away solver that both the standalone demo and the ROS2 package use.
Keep ROS/PGM/map I/O out of here so this remains a lightweight dependency.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon
from scipy.optimize import LinearConstraint, minimize


Point = Tuple[float, float]


# ========= Geometry + Bezier helpers =========
def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def lerp2(p: Point, q: Point, t: float) -> Point:
    return (lerp(p[0], q[0], t), lerp(p[1], q[1], t))


def de_casteljau_split(ctrl: Sequence[Point], t: float):
    work = [list(ctrl)]
    n = len(ctrl) - 1
    for _ in range(1, n + 1):
        prev = work[-1]
        cur = [lerp2(prev[i], prev[i + 1], t) for i in range(len(prev) - 1)]
        work.append(cur)
    left = [work[i][0] for i in range(n + 1)]
    right = [work[n - i][i] for i in range(n + 1)]
    return left, right


def bezier_eval(ctrl: Sequence[Point], t: float) -> Point:
    pts = list(ctrl)
    n = len(pts) - 1
    for _ in range(1, n + 1):
        pts = [lerp2(pts[i], pts[i + 1], t) for i in range(len(pts) - 1)]
    return pts[0]


def bezier_flatness(ctrl: Sequence[Point]) -> float:
    p0, pn = ctrl[0], ctrl[-1]
    x0, y0 = p0
    x1, y1 = pn
    dx, dy = x1 - x0, y1 - y0
    denom = math.hypot(dx, dy)
    if denom == 0.0:
        return 0.0
    mx = 0.0
    for p in ctrl[1:-1]:
        num = abs(dy * (p[0] - x0) - dx * (p[1] - y0))
        d = num / denom
        if d > mx:
            mx = d
    return mx


def aabb_of_points(pts: Sequence[Point]):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def aabb_overlap(a, b) -> bool:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    return not (ax1 < bx0 or bx1 < ax0 or ay1 < by0 or by1 < ay0)


# ---------- intersection helpers ----------
def orient(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def on_segment(a: Point, b: Point, p: Point) -> bool:
    return (
        min(a[0], b[0]) - 1e-12 <= p[0] <= max(a[0], b[0]) + 1e-12
        and min(a[1], b[1]) - 1e-12 <= p[1] <= max(a[1], b[1]) + 1e-12
        and abs(orient(a, b, p)) <= 1e-12
    )


def segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)
    if (o1 == 0 and on_segment(a, b, c)) or (o2 == 0 and on_segment(a, b, d)) or (o3 == 0 and on_segment(c, d, a)) or (o4 == 0 and on_segment(c, d, b)):
        return True
    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)


def point_in_polygon(pt: Point, poly: Sequence[Point]) -> bool:
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-18) + x1):
            inside = not inside
    return inside


def segment_polygon_intersect(a: Point, b: Point, poly: Sequence[Point]) -> bool:
    if point_in_polygon(a, poly) or point_in_polygon(b, poly):
        return True
    n = len(poly)
    for i in range(n):
        c = poly[i]
        d = poly[(i + 1) % n]
        if segments_intersect(a, b, c, d):
            return True
    return False


def segment_circle_intersect(a: Point, b: Point, center: Point, r: float) -> bool:
    (x1, y1), (x2, y2) = a, b
    (cx, cy) = center
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(cx - x1, cy - y1) <= r
    t = ((cx - x1) * dx + (cy - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    px, py = x1 + t * dx, y1 + t * dy
    return math.hypot(px - cx, py - cy) <= r + 1e-12


# ---------- obstacles ----------
@dataclass
class Obstacle:
    poly: Optional[Sequence[Point]] = None
    circle: Optional[Tuple[Point, float]] = None

    def __post_init__(self):
        assert (self.poly is not None) ^ (self.circle is not None), "poly 또는 circle 중 하나만 지정"
        if self.poly is not None:
            self.aabb = aabb_of_points(self.poly)
        else:
            (c, r) = self.circle
            cx, cy = c
            self.aabb = (cx - r, cy - r, cx + r, cy + r)

    def aabb_overlap(self, aabb) -> bool:
        return aabb_overlap(self.aabb, aabb)

    def seg_hit(self, a: Point, b: Point) -> bool:
        if self.poly is not None:
            return segment_polygon_intersect(a, b, self.poly)
        (c, r) = self.circle
        return segment_circle_intersect(a, b, c, r)


# ---------- collision interval search ----------
def find_intervals_bezier_hits(
    ctrl: Sequence[Point],
    obstacles: List[Obstacle],
    t0: float = 0.0,
    t1: float = 1.0,
    flat_eps: float = 1e-2,
    max_depth: int = 30,
) -> List[Tuple[float, float]]:
    hits: List[Tuple[float, float]] = []

    def recurse(ctrl_local, a, b, depth):
        aabb = aabb_of_points(ctrl_local)
        if not any(obs.aabb_overlap(aabb) for obs in obstacles):
            return
        if (depth >= max_depth) or (bezier_flatness(ctrl_local) <= flat_eps):
            p0, p1 = ctrl_local[0], ctrl_local[-1]
            if any(obs.seg_hit(p0, p1) for obs in obstacles):
                hits.append((a, b))
            return
        left, right = de_casteljau_split(ctrl_local, 0.5)
        mid = 0.5 * (a + b)
        recurse(left, a, mid, depth + 1)
        recurse(right, mid, b, depth + 1)

    recurse(ctrl, t0, t1, 0)
    if not hits:
        return []
    hits.sort()
    merged = [hits[0]]
    for s, e in hits[1:]:
        ps, pe = merged[-1]
        if s <= pe + 1e-6:
            merged[-1] = (ps, max(pe, e))
        else:
            merged.append((s, e))
    return merged


# ---------- De Casteljau split matrices ----------
def split_matrices(n: int, t: float):
    """De Casteljau 분할의 '왼쪽/오른쪽 제어점'을 원 제어점의 선형결합으로 나타내는 행렬 L,R."""
    prev = [np.eye(n + 1)[i] for i in range(n + 1)]  # 각 원소는 (n+1,) row vector
    work = [prev]
    for _ in range(1, n + 1):
        prev = work[-1]
        cur = [(1 - t) * prev[i] + t * prev[i + 1] for i in range(len(prev) - 1)]
        work.append(cur)
    left_rows = [work[i][0] for i in range(n + 1)]
    right_rows = [work[n - i][i] for i in range(n + 1)]
    L = np.vstack(left_rows)
    R = np.vstack(right_rows)
    return L, R


def subcurve_matrix(n: int, u: float, v: float):
    """서브구간 [u,v]의 제어다각형 Q[u,v] = M @ P (성분별 적용)."""
    assert 0.0 <= u < v <= 1.0 + 1e-15
    L1, R1 = split_matrices(n, u)
    if u == 1.0:
        return L1 * 0.0  # degenerate
    s = (v - u) / (1 - u)
    L2, _ = split_matrices(n, s)
    M = L2 @ R1
    return M  # (n+1, n+1)


# ---------- halfspace builders ----------
def halfspace_circle(center: Point, r: float, a: float, b: float, ctrl: Sequence[Point], rho: float):
    A = np.array(bezier_eval(ctrl, a))
    B = np.array(bezier_eval(ctrl, b))
    xref = 0.5 * (A + B)
    c = np.array(center)
    v = xref - c
    nrm = np.linalg.norm(v)
    if nrm < 1e-12:
        # 중심과 같은 점이면 끝점 방향으로라도 정의
        v = (B - A)
        nrm = np.linalg.norm(v) + 1e-12
    n = v / nrm
    beta = n @ (c + (r + rho) * n)
    return n, beta  # n^T x >= beta


def polygon_outward_normals(poly: Sequence[Point]):
    """에지들에 대한 (outward n, beta) 후보들을 리턴."""
    m = len(poly)
    cent = np.mean(np.array(poly), axis=0)
    outs = []
    for i in range(m):
        a = np.array(poly[i])
        b = np.array(poly[(i + 1) % m])
        e = b - a
        n = np.array([e[1], -e[0]])  # left-hand normal
        nn = np.linalg.norm(n)
        if nn < 1e-12:
            continue
        n = n / nn
        beta = n @ a
        # 방향 보정: 다각형 중심이 '안'에 있으므로 n^T centroid <= beta 여야 함.
        if n @ cent > beta:
            n = -n
            beta = n @ a
        outs.append((n, beta))
    return outs


def halfspace_polygon(poly: Sequence[Point], q_points: np.ndarray):
    """
    q_points: (k,2) 서브커브의 제어점들 (끝점 제외 권장).
    가장 '위반(=n^T x - beta가 최소)'이 큰 변의 반평면을 선택.
    """
    cands = polygon_outward_normals(poly)  # list of (n, beta)
    worst = None
    worst_margin = +1e18
    for (n, beta) in cands:
        margins = q_points @ n - beta
        m = np.min(margins)  # 가장 안쪽으로 파고든 정도(음수가 크면 위반 큼)
        if m < worst_margin:
            worst_margin = m
            worst = (n, beta)
    # worst가 None이면 폴리곤이 퇴화한 경우
    if worst is None:
        # fallback: 아무거나
        a = np.array(poly[0])
        b = np.array(poly[1])
        e = b - a
        n = np.array([e[1], -e[0]])
        n = n / (np.linalg.norm(n) + 1e-12)
        beta = n @ a
        return n, beta
    return worst  # n, beta


# ---------- QP assembly ----------
def second_diff_matrix(n_pts: int):
    """길이 n_pts에 대한 2차 차분 행렬 D (shape: (n_pts-2, n_pts))."""
    if n_pts < 3:
        return np.zeros((0, n_pts))
    D = np.zeros((n_pts - 2, n_pts))
    for i in range(n_pts - 2):
        D[i, i] = 1.0
        D[i, i + 1] = -2.0
        D[i, i + 2] = 1.0
    return D


def vecP(ctrl: Sequence[Point]) -> np.ndarray:
    xs = np.array([p[0] for p in ctrl])
    ys = np.array([p[1] for p in ctrl])
    return np.concatenate([xs, ys], axis=0)


def unvecP(v: np.ndarray) -> List[Point]:
    m = v.shape[0] // 2
    xs = v[:m]
    ys = v[m:]
    return [(float(xs[i]), float(ys[i])) for i in range(m)]


def clamp_step(ctrl_old: Sequence[Point], ctrl_new: Sequence[Point], max_step: float) -> List[Point]:
    """
    한 스텝에서 각 제어점이 움직일 수 있는 최대 거리를 max_step으로 제한.
    너무 멀리 튀는 것을 방지하기 위한 안전장치.
    """
    out: List[Point] = []
    for (x0, y0), (x1, y1) in zip(ctrl_old, ctrl_new):
        dx = x1 - x0
        dy = y1 - y0
        dist = math.hypot(dx, dy)
        if dist > max_step:
            s = max_step / (dist + 1e-12)
            out.append((x0 + dx * s, y0 + dy * s))
        else:
            out.append((x1, y1))
    return out


def build_constraints(
    P: np.ndarray,  # vec(P) (2*(n+1),)
    ctrl: Sequence[Point],
    intervals: List[Tuple[float, float]],
    obstacles: List[Obstacle],
    rho: float,
):
    """
    A_ineq @ d >= b_ineq   (d는 Δ=vec(ΔP))
    A_eq   @ d = 0         (끝점 고정)
    """
    n = len(ctrl) - 1
    Aeq_rows = []
    beq = []

    def row_for_idx(idx, is_x=True):
        r = np.zeros(2 * (n + 1))
        if is_x:
            r[idx] = 1.0
        else:
            r[(n + 1) + idx] = 1.0
        return r

    for idx in [0, n]:
        Aeq_rows.append(row_for_idx(idx, True))
        beq.append(0.0)
        Aeq_rows.append(row_for_idx(idx, False))
        beq.append(0.0)
    A_eq = np.vstack(Aeq_rows) if Aeq_rows else np.zeros((0, 2 * (n + 1)))
    b_eq = np.array(beq) if beq else np.zeros((0,))

    # 부등식
    A_rows = []
    b_rows = []

    Px = P[: n + 1]
    Py = P[n + 1 :]

    for (a, b) in intervals:
        if not (0.0 <= a < b <= 1.0):
            continue
        M = subcurve_matrix(n, a, b)  # (n+1, n+1)
        qx = M @ Px
        qy = M @ Py
        Q = np.stack([qx, qy], axis=1)  # (n+1,2)
        Qaabb = aabb_of_points(Q.tolist())

        # 이 구간과 AABB가 겹치는 장애물만 고려
        related = [obs for obs in obstacles if obs.aabb_overlap(Qaabb)]
        if not related:
            related = obstacles  # 혹시 모두 빠지면 보수적으로 전부

        inner_idx = list(range(1, n)) if n >= 2 else []

        for obs in related:
            if obs.circle is not None:
                (c, r) = obs.circle
                nvec, beta = halfspace_circle(c, r, a, b, ctrl, rho)
            else:
                nvec, beta = halfspace_polygon(obs.poly, Q[inner_idx] if inner_idx else Q)

            nx, ny = float(nvec[0]), float(nvec[1])
            for j in inner_idx:
                mj = M[j, :]
                w = np.zeros(2 * (n + 1))
                w[: n + 1] = nx * mj
                w[(n + 1) :] = ny * mj
                rhs = beta - (nx * (mj @ Px) + ny * (mj @ Py))
                A_rows.append(w)
                b_rows.append(rhs)

    if A_rows:
        A_ineq = np.vstack(A_rows)
        b_ineq = np.array(b_rows)
    else:
        A_ineq = np.zeros((0, 2 * (n + 1)))
        b_ineq = np.zeros((0,))

    return A_eq, b_eq, A_ineq, b_ineq


def solve_qp(
    P_init_vec: np.ndarray,
    ctrl_init: Sequence[Point],
    intervals: List[Tuple[float, float]],
    obstacles: List[Obstacle],
    rho: float = 0.05,
    lambda_smooth: float = 1.0,
):
    """
    Δ를 구해 P_new = P + Δ. 끝점은 고정, 내부는 최소이동+매끄러움으로 반평면 제약을 만족.
    """
    n = len(ctrl_init) - 1
    m = 2 * (n + 1)

    A_eq, b_eq, A_ineq, b_ineq = build_constraints(P_init_vec, ctrl_init, intervals, obstacles, rho)

    I = np.eye(m)
    D = second_diff_matrix(n + 1)  # (n-1, n+1)
    if D.shape[0] == 0:
        L = np.zeros((0, m))
    else:
        Z = np.zeros_like(D)
        L = np.vstack([np.hstack([D, Z]), np.hstack([Z, D])])
    H = I + lambda_smooth * (L.T @ L)

    def fun(d):
        return 0.5 * d @ (H @ d)

    def jac(d):
        return H @ d

    def hess(_):
        return H

    cons = []
    if A_eq.shape[0] > 0:
        cons.append(LinearConstraint(A_eq, b_eq, b_eq))
    if A_ineq.shape[0] > 0:
        lb = b_ineq
        ub = np.full_like(b_ineq, np.inf, dtype=float)
        cons.append(LinearConstraint(A_ineq, lb, ub))

    d0 = np.zeros_like(P_init_vec)
    res = minimize(fun, d0, method="trust-constr", jac=jac, hess=hess, constraints=cons, options={"maxiter": 500, "verbose": 0, "gtol": 1e-8})
    return res


# ---------- visualization ----------
def visualize_curve_and_hits(
    ctrl,
    obstacles,
    title="",
    intervals=None,
    samples: int = 800,
    flat_eps: float = 5e-3,
    max_depth: int = 28,
    show_now: bool = True,
    save_path: str = None,
):
    # intervals 미지정 시 내부 계산
    if intervals is None:
        intervals = find_intervals_bezier_hits(ctrl, obstacles, flat_eps=flat_eps, max_depth=max_depth)

    ts = np.linspace(0, 1, samples)
    curve = [bezier_eval(ctrl, float(t)) for t in ts]
    xs = [p[0] for p in curve]
    ys = [p[1] for p in curve]

    plt.figure(figsize=(7, 6))
    ax = plt.gca()
    for obs in obstacles:
        if obs.poly is not None:
            ax.add_patch(Polygon(obs.poly, fill=False))
        else:
            (c, r) = obs.circle
            ax.add_patch(Circle(c, r, fill=False))

    plt.plot(xs, ys, linewidth=2, label="curve")
    cx = [p[0] for p in ctrl]
    cy = [p[1] for p in ctrl]
    plt.plot(cx, cy, linestyle="--", marker="o", label="control polygon")

    if intervals:
        for (a, b) in intervals:
            i0 = max(0, int(a * (samples - 1)))
            i1 = min(samples - 1, int(b * (samples - 1)))
            if i1 > i0:
                plt.plot(xs[i0 : i1 + 1], ys[i0 : i1 + 1], linewidth=4, alpha=0.5, label="hit segment")

    if intervals:
        title = f"{title}  |  Hit intervals: {[(round(a,6), round(b,6)) for (a,b) in intervals]}"
    if title:
        plt.title(title)
    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")

    if show_now:
        plt.show(block=False)
        plt.pause(10)
    else:
        plt.close()


# ---------- push-away pipelines ----------
def push_away_with_halfspaces(
    ctrl_init: Sequence[Point],
    obstacles: List[Obstacle],
    flat_eps=5e-3,
    max_depth=28,
    rho=0.05,
    lambda_smooth=1.0,
    max_passes=5,
):
    ctrl = list(ctrl_init)
    for it in range(max_passes):
        intervals = find_intervals_bezier_hits(ctrl, obstacles, flat_eps=flat_eps, max_depth=max_depth)
        visualize_curve_and_hits(ctrl, obstacles, title=f"Pass {it}: before", intervals=intervals)
        if not intervals:
            return ctrl, intervals, {"passes": it, "status": "already_safe"}

        Pvec = vecP(ctrl)
        res = solve_qp(Pvec, ctrl, intervals, obstacles, rho=rho, lambda_smooth=lambda_smooth)
        d = res.x

        newP = Pvec + d
        ctrl = unvecP(newP)

        intervals2 = find_intervals_bezier_hits(ctrl, obstacles, flat_eps=flat_eps, max_depth=max_depth)
        visualize_curve_and_hits(ctrl, obstacles, title=f"Pass {it}: after", intervals=intervals2)

        if not intervals2:
            return ctrl, intervals2, {"passes": it + 1, "status": "resolved"}
    final_intervals = find_intervals_bezier_hits(ctrl, obstacles, flat_eps=flat_eps, max_depth=max_depth)
    return ctrl, final_intervals, {"passes": max_passes, "status": "unfinished"}


def _total_interval_len(intervals):
    return float(sum(max(0.0, b - a) for (a, b) in intervals))


def recursive_push_away(
    ctrl_init,
    obstacles,
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
):
    """
    초기 곡선 1회 출력 후, 매 스텝의 '수정된 결과'만 출력.
    충돌 없어지면 즉시 종료.
    """
    import os

    ctrl = list(ctrl_init)

    # --- 초기 1회 출력 ---
    if plot:
        intervals0 = find_intervals_bezier_hits(ctrl, obstacles, flat_eps=flat_eps, max_depth=max_depth)
        save_path0 = os.path.join(save_dir, "step_00_initial.png") if save_dir else None
        visualize_curve_and_hits(ctrl, obstacles, title="Step 0 (initial)", intervals=intervals0, samples=samples, flat_eps=flat_eps, max_depth=max_depth, show_now=True, save_path=save_path0)

    def _rec(ctrl_cur, step, rho, last_len):
        intervals = find_intervals_bezier_hits(ctrl_cur, obstacles, flat_eps=flat_eps, max_depth=max_depth)
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
        ctrl_next = unvecP(Pvec + d)

        intervals2 = find_intervals_bezier_hits(ctrl_next, obstacles, flat_eps=flat_eps, max_depth=max_depth)
        L2 = _total_interval_len(intervals2)
        if verbose:
            print(f"         -> after step {step}: hit_len={L2:.6f}")

        if plot:
            sp = None
            if save_dir:
                os.makedirs(save_dir, exist_ok=True)
                sp = os.path.join(save_dir, f"step_{step+1:02d}.png")
            visualize_curve_and_hits(ctrl_next, obstacles, title=f"Step {step+1}", intervals=intervals2, samples=samples, flat_eps=flat_eps, max_depth=max_depth, show_now=True, save_path=sp)

        if not intervals2:
            return ctrl_next, [], {"steps": step + 1, "status": "resolved"}

        improved = L2 < L - tol_improve
        if improved:
            return _rec(ctrl_next, step + 1, rho, L2)

        new_rho = min(rho_max, rho * rho_growth)
        if new_rho > rho + 1e-12:
            if verbose:
                print(f"         (stalled) increasing rho -> {new_rho:.4f}")
            return _rec(ctrl_next, step + 1, new_rho, L2)

        return ctrl_next, intervals2, {"steps": step + 1, "status": "stalled"}

    return _rec(ctrl, step=0, rho=rho_init, last_len=float("inf"))
