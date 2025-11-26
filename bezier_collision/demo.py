import matplotlib.pyplot as plt
import numpy as np

from .core import (
    CircleObstacle,
    RectObstacle,
    circle_radius_to_obstacles,
    collision_segment_endpoints,
    find_collision_intervals,
)


def demo_plot(use_grid: bool = False) -> None:
    """Run the collision sampling demo and show the plot."""
    obstacles = [
        RectObstacle(1.5, 3.0, 1.0, 3.5),
        RectObstacle(5.5, 7.0, 4.0, 5.5),
        CircleObstacle(6.0, 2.5, 0.9),
    ]

    resolution = 0.1
    grid = np.zeros((120, 120), dtype=np.uint8)
    origin_x = 0.0
    origin_y = 0.0
    grid[10:35, 15:30] = 1
    grid[40:55, 55:70] = 1
    yy, xx = np.ogrid[:grid.shape[0], :grid.shape[1]]
    cx, cy, r = (6.0 - origin_x) / resolution, (2.5 - origin_y) / resolution, 0.9 / resolution
    circle_mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= r ** 2
    grid[circle_mask] = 1

    ctrl = [(0.5, 0.5), (3.0, 6.0), (6.5, -1.0), (9.0, 6.0)]

    intervals, t_vals, pts, collisions, _, interval_indices = find_collision_intervals(
        ctrl,
        obstacles,
        n_samples=500,
        use_arc_length=True,
        use_grid=use_grid,
        grid=grid,
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=resolution,
    )
    segments = collision_segment_endpoints(ctrl, t_vals, interval_indices, intervals)
    circles = []
    for (_, p_start, p_end) in segments:
        mid = 0.5 * (p_start + p_end)
        r_circle = circle_radius_to_obstacles(mid, obstacles)
        circles.append((mid, r_circle))

    fig, ax = plt.subplots(figsize=(8, 6))
    for obs in obstacles:
        if isinstance(obs, RectObstacle):
            ax.add_patch(
                plt.Rectangle(
                    (obs.xmin, obs.ymin),
                    obs.xmax - obs.xmin,
                    obs.ymax - obs.ymin,
                    color="tab:red",
                    alpha=0.25,
                )
            )
        elif isinstance(obs, CircleObstacle):
            ax.add_patch(plt.Circle((obs.cx, obs.cy), obs.r, color="tab:red", alpha=0.25))

    ax.plot(pts[:, 0], pts[:, 1], color="steelblue", label="Bézier curve")
    coll_pts = pts[collisions]
    if len(coll_pts):
        ax.scatter(coll_pts[:, 0], coll_pts[:, 1], color="orange", s=14, label="Collision")
        for idx, (_, p_start, p_end) in enumerate(segments):
            label = "Start/End" if idx == 0 else None
            ax.scatter(p_start[0], p_start[1], color="red", s=30, marker="s", label=label)
            ax.scatter(p_end[0], p_end[1], color="red", s=30, marker="s")
        for idx, (mid, r_circle) in enumerate(circles):
            label = "Mid circle" if idx == 0 else None
            ax.add_patch(plt.Circle(mid, r_circle, fill=False, color="black", linestyle="-", linewidth=1.5, label=label))

    ax.set_aspect("equal", "box")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.legend()
    ax.set_title("Bézier collision intervals (tau)")

    print("Collision intervals (tau):", intervals)
    if segments:
        print("Collision segment endpoints (De Casteljau):")
        for i, ((tau_s, tau_e), p_s, p_e) in enumerate(segments, start=1):
            print(
                f"  Segment {i}: tau=({tau_s:.4f}, {tau_e:.4f}) "
                f"start=({p_s[0]:.3f}, {p_s[1]:.3f}) "
                f"end=({p_e[0]:.3f}, {p_e[1]:.3f})"
            )
        print("Midpoint circles (center, radius to first obstacle contact):")
        for i, (mid, r_circle) in enumerate(circles, start=1):
            print(f"  Segment {i}: center=({mid[0]:.3f}, {mid[1]:.3f}), radius={r_circle:.3f}")
    plt.show()


if __name__ == "__main__":
    demo_plot()
