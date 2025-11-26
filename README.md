# bezier_with_matplotlib

Small demo that samples a cubic Bézier curve against obstacles, finds collision intervals, and visualizes the result with matplotlib. Obstacles can be defined analytically (rectangles/circles) or via a costmap-style grid.

## Layout
- `bezier_collision/core.py`: core functions and obstacle helpers (importable).
- `bezier_collision/demo.py`: plotting/demo logic.
- `bezier_collision.py`: thin runner that calls the demo.

## What it does
- Samples the curve, detects where it intersects obstacles, and reports the tau intervals (uniform or arc-length parameterization).
- Shows collision points, interval endpoints, and helper circles centered at segment midpoints with radius to the nearest obstacle contact.
- Prints the intervals and segment endpoints to stdout, and displays a plot of the scene.

## Run the demo
```bash
pip install numpy matplotlib
python bezier_collision.py
```
The demo uses hardcoded obstacles/control points. Toggle `use_grid` in `demo_plot()` to switch between analytic shapes and the grid map representation.

## Use as a module
```python
from bezier_collision import (
    RectObstacle,
    CircleObstacle,
    find_collision_intervals,
    collision_segment_endpoints,
)

ctrl = [(0.5, 0.5), (3.0, 6.0), (6.5, -1.0), (9.0, 6.0)]
obstacles = [RectObstacle(1.5, 3.0, 1.0, 3.5), CircleObstacle(6.0, 2.5, 0.9)]
intervals, t_vals, pts, collisions, tau_vals, interval_indices = find_collision_intervals(
    ctrl, obstacles, n_samples=200, use_arc_length=True
)
segments = collision_segment_endpoints(ctrl, t_vals, interval_indices, intervals)
```
