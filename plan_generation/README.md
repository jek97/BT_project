# Interactive OccupancyGrid A* + Spline Planner

`occupancy_grid_planner.py` shows a nav_msgs/OccupancyGrid-style 2D map
(obstacles black, free space white, unknown gray), lets you click a start
point (green dot) and a goal point (red dot), runs A* between them, fits a
B-spline to the resulting path, and writes the spline's characteristics
(knots, control points, waypoints, and a dense sampling of the curve) to a
`.txt` file next to the map.

## Install

```bash
pip install numpy scipy matplotlib pyyaml pillow
```

## Run

If you have a standard ROS `map_server` map (a `.yaml` + its paired
`.pgm`/`.png`, e.g. from `map_saver`):

```bash
python3 occupancy_grid_planner.py --map path/to/map.yaml
```

If instead you have the OccupancyGrid data as a raw numpy array (0-100 =
occupancy, -1 = unknown, row 0 = origin y � the same layout as
`msg.data` reshaped to `(height, width)`):

```bash
python3 occupancy_grid_planner.py --npy path/to/grid.npy --resolution 0.05 --origin 0 0 0
```

Then:
1. A window opens with the map.
2. Left-click once for **start** (green dot).
3. Left-click again for **goal** (red dot) � planning + spline fitting run
   automatically.
4. Press `r` at any time to reset and pick a new start/goal.

The spline characteristics are saved to `<map_name>_spline.txt` next to the
map (or wherever `--out` points).

## Useful options

| flag | meaning |
|---|---|
| `--occ-thresh N` | occupancy value (0-100) at/above which a cell is an obstacle (default 50) |
| `--connectivity 4\|8` | A* neighbor connectivity (default 8) |
| `--inflate R` | grow obstacles by R meters before planning, e.g. for robot radius |
| `--spline-degree K` | B-spline degree (default 3, cubic) |
| `--spline-smooth S` | spline smoothing factor (default 0 = passes exactly through the A* waypoints) |
| `--samples N` | number of points sampled along the spline for the output file/plot |
| `--out FILE.txt` | override the output text file path |

## Getting the map from a live ROS system instead of a file

If you're pulling the OccupancyGrid straight from a topic rather than a
saved file, build an `OccupancyGridMap` directly instead of using
`load_map_yaml`/`load_map_npy`:

```python
from occupancy_grid_planner import OccupancyGridMap
import numpy as np

def from_ros_msg(msg):
    data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    origin = [msg.info.origin.position.x, msg.info.origin.position.y, 0.0]
    return OccupancyGridMap(data, msg.info.resolution, origin,
                             msg.info.width, msg.info.height)
```

## Output file format

The generated `*_spline.txt` contains, in order: start/goal world
coordinates, spline degree, the A* waypoints used to fit the spline, the
knot vector, the x/y control points, the spline parameter `u` of each
waypoint, and a dense `(x, y)` sampling of the curve � everything needed to
reconstruct or re-evaluate the spline (e.g. with `scipy.interpolate.splev`)
without rerunning the planner.