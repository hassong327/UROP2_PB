# bezier_with_matplotlib

Small demo that samples a cubic Bézier curve against obstacles, finds collision intervals, and visualizes the result with matplotlib. Obstacles can be defined analytically (rectangles/circles) or via a costmap-style grid.

## What it does
- Samples the curve, detects where it intersects obstacles, and reports the tau intervals (uniform or arc-length parameterization).
- Shows collision points, interval endpoints, and helper circles centered at segment midpoints with radius to the nearest obstacle contact.
- Prints the intervals and segment endpoints to stdout, and displays a plot of the scene.

## Run
```bash
pip install numpy matplotlib
python bezier_collision.py
```
The demo uses the hardcoded obstacles/control points in `bezier_collision.py`. Toggle `use_grid` in `demo_plot()` to switch between analytic shapes and the grid map representation.
