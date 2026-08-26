#!/usr/bin/env python3
"""
occupancy_grid_planner.py
==========================

Interactive planner for a nav_msgs/OccupancyGrid-style 2D map.

Workflow
--------
1. The map is loaded and displayed: obstacles in black, free space in white,
   unknown cells in gray.
2. Left-click once on the map to set the START point   -> marked with a green dot.
3. Left-click a second time to set the GOAL point       -> marked with a red dot.
4. As soon as both points are set, obstacles are inflated by 0.5m (by
   default -- see --inflate) to keep the finite-size robot's centerline
   away from walls, and A* is run on this INFLATED grid to find a path.
5. The resulting path is fitted with a B-spline (scipy splprep/splev) and
   drawn on top of the A* path.
6. The B-spline is converted EXACTLY into a chain of cubic Bezier segments
   (via full knot insertion) and written out as ProbLog/Prolog facts
   (start/2, goal/2, control_points/1) directly consultable by
   moveto_continuous.pl -- see write_prolog_plan() below.

Press 'r' at any time to reset the start/goal selection and try again.

Map input
---------
By default this script always loads the map from a FIXED location relative
to the current working directory:

    ./../environments/maps/map.yaml

i.e. it expects to be run from a directory whose parent contains
environments/maps/map.yaml (the standard map_server yaml + image pair, the
usual output of `map_saver` / the map_generation scripts in this project).
Pass --map to point at a different yaml file, or --npy to load a raw
OccupancyGrid array instead (see below), if you need to override this.

  1) --map path/to/map.yaml   (default: ./../environments/maps/map.yaml)
     A standard ROS `map_server` yaml + image pair.

  2) --npy path/to/grid.npy  --resolution 0.05 --origin 0 0 0
     A raw 2D numpy array saved with np.save, using the same value
     convention as nav_msgs/OccupancyGrid.data reshaped to (height, width):
     0..100 = probability of occupancy, -1 = unknown, row 0 = y = origin_y.

Output
------
Two files are always written into a FIXED output folder relative to the
current working directory:

    ./plan/

(created automatically if it doesn't exist):

  - ./plan/<map_name>_plan.pl  -- a descriptive, per-map record of this
    specific plan (so multiple maps/runs don't overwrite each other)
  - ./plan/current_plan.pl     -- ALWAYS overwritten with the same content
    as the file above; this is the FIXED path moveto_continuous.pl expects
    to consult (see its own header comment for the expected relative
    location: ./plan_generation/plan/current_plan.pl)

Both are plain ProbLog/Prolog fact files:

    start(X0, Y0).
    goal(Xn, Yn).
    control_points([point(X0,Y0), point(X1,Y1), ..., point(Xn,Yn)]).

control_points/1 has length 3k+1 for k cubic Bezier segments (segment i
uses control points 3i..3i+3, consecutive segments sharing their boundary
point) -- this is EXACTLY the format moveto_continuous.pl's spline_point/4
expects. It is obtained from the fitted B-spline by full knot insertion
(each interior knot raised to multiplicity == degree), which is a lossless,
exact conversion -- not an approximation -- from the general B-spline
scipy fits into the specific chained-cubic-Bezier representation the
action theory uses. See bspline_to_bezier_chain() below.

Pass --out to override the per-map output file path if needed (the fixed
current_plan.pl in the same folder is still written either way).

If you are getting the map directly from a running ROS system instead of a
file, subscribe to the OccupancyGrid topic and build an OccupancyGridMap
object directly from msg.data / msg.info (see the OccupancyGridMap
docstring below) instead of using load_map_yaml/load_map_npy.

Usage examples
--------------
    python3 occupancy_grid_planner.py
    python3 occupancy_grid_planner.py --map some/other/map.yaml
    python3 occupancy_grid_planner.py --npy maps/grid.npy --resolution 0.05 --origin 0 0 0
    python3 occupancy_grid_planner.py --inflate 0.3 --connectivity 8

Dependencies
------------
    pip install numpy scipy matplotlib pyyaml pillow
"""

import argparse
import heapq
import math
import os

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev, insert
from scipy.ndimage import binary_dilation


# --------------------------------------------------------------------------
# Fixed locations
# --------------------------------------------------------------------------

# Default map input, relative to the current working directory.
DEFAULT_MAP_PATH = os.path.join(".", "..", "environments", "maps", "map.yaml")

# Fixed output folder, relative to the current working directory.
OUTPUT_DIR = os.path.join(os.getcwd(), "plan")

# Fixed filename moveto_continuous.pl always consults (see its header).
CURRENT_PLAN_FILENAME = "current_plan.pl"


# --------------------------------------------------------------------------
# Map representation
# --------------------------------------------------------------------------

class OccupancyGridMap:
    """
    Minimal stand-in for the information carried by a nav_msgs/OccupancyGrid
    message.

    data       : 2D int array, shape (height, width). data[row, col] is the
                 occupancy of the cell whose world position is
                 (origin_x + col*resolution, origin_y + row*resolution).
                 Values: 0..100 = occupancy probability, -1 = unknown.
                 (This is exactly msg.data reshaped to (msg.info.height,
                 msg.info.width), i.e. row-major with row 0 = min y.)
    resolution : meters per cell (msg.info.resolution)
    origin     : [x, y, yaw] world pose of cell (0,0)'s corner (msg.info.origin)
    width      : number of columns  (msg.info.width)
    height     : number of rows     (msg.info.height)
    """

    def __init__(self, data, resolution, origin, width, height):
        self.data = data
        self.resolution = float(resolution)
        self.origin = list(origin)
        self.width = int(width)
        self.height = int(height)

    def world_to_grid(self, x, y):
        col = int(math.floor((x - self.origin[0]) / self.resolution))
        row = int(math.floor((y - self.origin[1]) / self.resolution))
        return row, col

    def grid_to_world(self, row, col):
        x = self.origin[0] + (col + 0.5) * self.resolution
        y = self.origin[1] + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def is_free(self, row, col, occ_thresh=50, unknown_is_occupied=True):
        if not self.in_bounds(row, col):
            return False
        v = self.data[row, col]
        if v == -1:
            return not unknown_is_occupied
        return v < occ_thresh

    def copy_with_data(self, new_data):
        return OccupancyGridMap(new_data, self.resolution, self.origin,
                                 self.width, self.height)


def load_map_yaml(yaml_path):
    """Load a standard ROS map_server yaml + image pair."""
    import yaml
    from PIL import Image

    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    image_path = meta["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    resolution = float(meta["resolution"])
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    negate = int(meta.get("negate", 0))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))

    img = Image.open(image_path)
    if img.mode != "L":
        img = img.convert("L")
    img_arr = np.array(img, dtype=np.float64)  # row 0 = top of image

    if negate:
        occ = img_arr / 255.0
    else:
        occ = (255.0 - img_arr) / 255.0

    grid = np.full(occ.shape, -1, dtype=np.int8)
    grid[occ > occupied_thresh] = 100
    grid[occ < free_thresh] = 0
    # cells in between stay -1 (unknown)

    # Image row 0 is the top of the picture (max y). OccupancyGrid row 0 is
    # y = origin_y (min y). Flip vertically to match the OccupancyGrid convention.
    grid = np.flipud(grid).copy()

    height, width = grid.shape
    return OccupancyGridMap(grid, resolution, origin, width, height)


def load_map_npy(npy_path, resolution, origin):
    data = np.load(npy_path)
    if data.ndim != 2:
        raise ValueError("--npy file must contain a 2D array")
    height, width = data.shape
    return OccupancyGridMap(data.astype(np.int8), resolution, origin, width, height)


def inflate_obstacles(grid_map, radius_m):
    """Return a copy of grid_map where obstacles (and unknown cells) have
    been grown by radius_m (a simple circular Minkowski dilation), useful
    for keeping a finite-size robot away from walls."""
    if radius_m <= 0:
        return grid_map
    radius_cells = max(1, int(round(radius_m / grid_map.resolution)))
    occ_mask = (grid_map.data == 100) | (grid_map.data == -1)

    yy, xx = np.ogrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
    disk = (xx ** 2 + yy ** 2) <= radius_cells ** 2

    inflated_mask = binary_dilation(occ_mask, structure=disk)
    new_data = grid_map.data.copy()
    new_data[inflated_mask] = 100
    return grid_map.copy_with_data(new_data)


# --------------------------------------------------------------------------
# A*
# --------------------------------------------------------------------------

def astar(grid_map, start_rc, goal_rc, occ_thresh=50, connectivity=8,
          unknown_is_occupied=True):
    """8- or 4-connected A* over grid_map. Returns a list of (row, col)
    cells from start to goal (inclusive), or None if no path exists."""

    if not grid_map.is_free(*start_rc, occ_thresh, unknown_is_occupied):
        return None
    if not grid_map.is_free(*goal_rc, occ_thresh, unknown_is_occupied):
        return None

    if connectivity == 8:
        steps = [(-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)),
                 (0, -1, 1.0), (0, 1, 1.0),
                 (1, -1, math.sqrt(2)), (1, 0, 1.0), (1, 1, math.sqrt(2))]
    else:
        steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]

    def heuristic(rc):
        return math.hypot(rc[0] - goal_rc[0], rc[1] - goal_rc[1])

    open_heap = [(heuristic(start_rc), 0.0, start_rc)]
    came_from = {}
    g_score = {start_rc: 0.0}
    closed = set()

    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal_rc:
            return _reconstruct_path(came_from, current)
        closed.add(current)

        for dr, dc, step_cost in steps:
            nr, nc = current[0] + dr, current[1] + dc
            neighbor = (nr, nc)
            if not grid_map.is_free(nr, nc, occ_thresh, unknown_is_occupied):
                continue
            # don't let the path cut diagonally through a corner of two obstacles
            if dr != 0 and dc != 0:
                if not grid_map.is_free(current[0] + dr, current[1], occ_thresh, unknown_is_occupied):
                    continue
                if not grid_map.is_free(current[0], current[1] + dc, occ_thresh, unknown_is_occupied):
                    continue

            tentative_g = g + step_cost
            if tentative_g < g_score.get(neighbor, float("inf")):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + heuristic(neighbor)
                heapq.heappush(open_heap, (f_score, tentative_g, neighbor))

    return None


def _reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# --------------------------------------------------------------------------
# Spline fitting
# --------------------------------------------------------------------------

def fit_spline(path_xy, degree=3, smoothing=0.0):
    """Fit a parametric B-spline through the (x, y) waypoints.
    Returns (tck, u) as produced by scipy.interpolate.splprep, where
    tck = (knots t, coefficients c=[cx, cy], degree k)."""
    path_xy = np.asarray(path_xy, dtype=float)
    x, y = path_xy[:, 0], path_xy[:, 1]
    n = len(x)
    if n < 2:
        raise ValueError("Need at least 2 waypoints to fit a spline")
    k = max(1, min(degree, n - 1))
    tck, u = splprep([x, y], k=k, s=smoothing)
    return tck, u


def bspline_to_bezier_chain(tck):
    """
    Convert a scipy parametric B-spline tck=(t,c,k) into an EXACT chain of
    degree-k Bezier segments, via full knot insertion: every interior knot
    is raised to multiplicity == k (the degree), at which point consecutive
    groups of (k+1) control points ARE the Bezier control points of one
    segment, with consecutive segments sharing their boundary point --
    i.e. exactly the "length 3k+1 for k cubic segments" format
    moveto_continuous.pl expects (for the standard, and only currently
    supported, case k=3).

    This is a LOSSLESS conversion, not a resampling/approximation -- the
    resulting Bezier chain reproduces the original B-spline curve exactly
    (verified numerically to floating-point precision). It says nothing
    about the ORIGINAL knot vector's spacing being preserved as segment
    boundaries in moveto_continuous.pl's own u-parametrization: that file
    always treats each segment as an equal 1/k share of its own u in [0,1]
    (see its spline_point/4), using arc-length integration -- NOT this
    B-spline's own (possibly non-uniform) knot spacing -- to determine
    timing. That's fine: only the CURVE SHAPE needs to transfer exactly,
    which it does; timing/speed along it is independently handled by
    moveto_continuous.pl's own arc-length machinery either way.

    Returns (control_points, degree) where control_points is a list of
    (x,y) tuples of length 3*num_segments + 1 for degree k=3.
    Raises ValueError if the fitted spline's degree isn't 3, since that's
    the only degree moveto_continuous.pl's Bezier evaluator supports.
    """
    t, c, k = tck
    if k != 3:
        raise ValueError(
            f"bspline_to_bezier_chain: got degree k={k}, but "
            f"moveto_continuous.pl only supports CUBIC (k=3) Bezier "
            f"segments. Re-fit with --spline-degree 3.")

    t = list(t)
    c = [list(comp) for comp in c]
    tck2 = (t, c, k)

    # interior knots = all knots strictly between the (k+1)-fold repeated
    # boundary knots at each end
    interior_knots = sorted(set(t[k + 1: len(t) - (k + 1)]))
    for knot in interior_knots:
        current_mult = t.count(knot)
        needed = k - current_mult
        if needed > 0:
            tck2 = insert(knot, tck2, m=needed, per=0)
            t = list(tck2[0])

    t_final, c_final, k_final = tck2
    n_ctrl = len(t_final) - k_final - 1
    cx, cy = c_final[0][:n_ctrl], c_final[1][:n_ctrl]
    control_points = list(zip(cx, cy))

    if (len(control_points) - 1) % k_final != 0:
        raise ValueError(
            "bspline_to_bezier_chain: extraction did not yield a clean "
            "3k+1-length control point list -- this shouldn't happen for "
            "a clamped cubic B-spline; check the input spline's knot "
            "vector.")

    return control_points, k_final


def write_prolog_plan(out_path, control_points, start_world, goal_world):
    """Write start/2, goal/2, control_points/1 as ProbLog/Prolog facts,
    directly consultable by moveto_continuous.pl -- no separate parsing
    step needed on the Prolog side."""
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)

    with open(out_path, "w") as f:
        f.write("% AUTO-GENERATED by occupancy_grid_planner.py -- do not hand-edit.\n")
        f.write("% A* path -> B-spline fit -> exact Bezier-chain extraction.\n")
        f.write("% control_points/1 has length 3k+1 for k cubic Bezier segments\n")
        f.write("% (segment i uses control points 3i..3i+3, consecutive segments\n")
        f.write("% sharing their boundary point) -- consult directly from\n")
        f.write("% moveto_continuous.pl.\n\n")

        f.write(f"start({start_world[0]:.6f}, {start_world[1]:.6f}).\n")
        f.write(f"goal({goal_world[0]:.6f}, {goal_world[1]:.6f}).\n\n")

        pts_str = ",\n    ".join(f"point({x:.6f},{y:.6f})" for x, y in control_points)
        f.write(f"control_points([\n    {pts_str}\n]).\n")


# --------------------------------------------------------------------------
# Interactive GUI
# --------------------------------------------------------------------------

class InteractivePlanner:
    def __init__(self, grid_map, args):
        self.grid_map = grid_map
        self.args = args
        self.start_rc = None
        self.goal_rc = None

        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self._draw_map()
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.ax.set_title("Click the START point")
        plt.show()

    def _draw_map(self):
        h, w = self.grid_map.height, self.grid_map.width
        rgb = np.ones((h, w, 3))                      # free -> white
        rgb[self.grid_map.data == 100] = [0, 0, 0]     # obstacle -> black
        rgb[self.grid_map.data == -1] = [0.6, 0.6, 0.6]  # unknown -> gray

        extent = [
            self.grid_map.origin[0],
            self.grid_map.origin[0] + w * self.grid_map.resolution,
            self.grid_map.origin[1],
            self.grid_map.origin[1] + h * self.grid_map.resolution,
        ]
        self.ax.imshow(rgb, origin="lower", extent=extent, interpolation="nearest")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.set_aspect("equal")

    def on_key(self, event):
        if event.key == "r":
            self.reset()

    def reset(self):
        self.start_rc = None
        self.goal_rc = None
        self.ax.cla()
        self._draw_map()
        self.ax.set_title("Click the START point")
        self.fig.canvas.draw()

    def on_click(self, event):
        if event.inaxes != self.ax or event.button != 1:
            return
        if event.xdata is None or event.ydata is None:
            return

        x, y = event.xdata, event.ydata
        row, col = self.grid_map.world_to_grid(x, y)
        if not self.grid_map.is_free(row, col, self.args.occ_thresh, unknown_is_occupied=True):
            print(f"({x:.2f}, {y:.2f}) is occupied or unknown, click a free cell instead")
            return

        if self.start_rc is None:
            self.start_rc = (row, col)
            self.ax.plot(x, y, "o", color="green", markersize=10, zorder=5)
            self.ax.set_title("Click the GOAL point")
            self.fig.canvas.draw()
        elif self.goal_rc is None:
            self.goal_rc = (row, col)
            self.ax.plot(x, y, "o", color="red", markersize=10, zorder=5)
            self.fig.canvas.draw()
            self.run_planning()
        else:
            print("Start and goal are already set. Press 'r' to reset and pick new ones.")

    def run_planning(self):
        self.ax.set_title("Planning...")
        self.fig.canvas.draw()
        plt.pause(0.01)

        planning_map = self.grid_map
        if self.args.inflate > 0:
            planning_map = inflate_obstacles(self.grid_map, self.args.inflate)

        path_rc = astar(
            planning_map, self.start_rc, self.goal_rc,
            occ_thresh=self.args.occ_thresh,
            connectivity=self.args.connectivity,
            unknown_is_occupied=True,
        )

        if path_rc is None:
            self.ax.set_title("No path found. Press 'r' to reset and try again.")
            self.fig.canvas.draw()
            print("No path found between start and goal.")
            return

        path_xy = np.array([self.grid_map.grid_to_world(r, c) for r, c in path_rc])
        self.ax.plot(path_xy[:, 0], path_xy[:, 1], "-", color="blue",
                     linewidth=1.5, label="A* path")

        tck, u = fit_spline(path_xy, degree=self.args.spline_degree,
                             smoothing=self.args.spline_smooth)

        try:
            control_points, degree = bspline_to_bezier_chain(tck)
        except ValueError as e:
            self.ax.set_title("Bezier extraction failed (see console).")
            self.fig.canvas.draw()
            print(f"Error: {e}")
            return

        u_fine = np.linspace(0.0, 1.0, self.args.samples)
        x_fine, y_fine = splev(u_fine, tck)
        self.ax.plot(x_fine, y_fine, "-", color="magenta", linewidth=2, label="Spline")
        self.ax.legend(loc="upper right")
        self.ax.set_title("Path found - spline fitted (press 'r' to plan again)")
        self.fig.canvas.draw()

        start_world = self.grid_map.grid_to_world(*self.start_rc)
        goal_world = self.grid_map.grid_to_world(*self.goal_rc)

        write_prolog_plan(self.args.out, control_points, start_world, goal_world)
        print(f"Plan (Prolog facts) written to: {self.args.out}")

        current_plan_path = os.path.join(OUTPUT_DIR, CURRENT_PLAN_FILENAME)
        write_prolog_plan(current_plan_path, control_points, start_world, goal_world)
        print(f"Also updated fixed path consulted by moveto_continuous.pl: {current_plan_path}")


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Click a start/goal on an OccupancyGrid map, plan with A*, "
                    "fit a spline to the path, convert it exactly to a cubic "
                    "Bezier chain, and export it as ProbLog/Prolog facts "
                    "(start/2, goal/2, control_points/1) directly consultable "
                    "by moveto_continuous.pl. By default the map is always "
                    "read from ./../environments/maps/map.yaml and the result "
                    "is always written into ./plan/ (both a per-map file and "
                    "the fixed ./plan/current_plan.pl).")

    src = parser.add_mutually_exclusive_group(required=False)
    src.add_argument("--map", type=str, default=None,
                      help="Path to a ROS map_server .yaml file (with its paired pgm/png "
                           f"image). Default: {DEFAULT_MAP_PATH}")
    src.add_argument("--npy", type=str,
                      help="Path to a .npy file holding a raw OccupancyGrid data array. "
                           "Overrides the fixed default map location.")

    parser.add_argument("--resolution", type=float, default=1.0,
                         help="Meters/cell, used only with --npy (default: 1.0)")
    parser.add_argument("--origin", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                         metavar=("X", "Y", "YAW"),
                         help="Map origin, used only with --npy (default: 0 0 0)")

    parser.add_argument("--occ-thresh", type=int, default=50,
                         help="Occupancy value (0-100) at/above which a cell counts as an obstacle (default: 50)")
    parser.add_argument("--connectivity", type=int, choices=[4, 8], default=8,
                         help="A* neighbor connectivity (default: 8)")
    parser.add_argument("--inflate", type=float, default=0.5,
                         help="Inflate obstacles by this radius in meters before "
                              "planning with A* (default: 0.5m -- pass --inflate 0 "
                              "to disable and plan over the raw map instead)")

    parser.add_argument("--spline-degree", type=int, default=3,
                         help="B-spline degree (default: 3, cubic -- REQUIRED to be 3 "
                              "for moveto_continuous.pl compatibility)")
    parser.add_argument("--spline-smooth", type=float, default=0.0,
                         help="Spline smoothing factor s (0 = interpolate exactly through waypoints)")
    parser.add_argument("--samples", type=int, default=200,
                         help="Number of points to sample along the fitted spline for the on-screen plot")

    parser.add_argument("--out", type=str, default=None,
                         help="Output .pl path for the per-map plan record "
                              "(default: ./plan/<map_name>_plan.pl). The fixed "
                              f"./plan/{CURRENT_PLAN_FILENAME} is always ALSO "
                              "written, regardless of this option.")
    return parser


def main():
    args = build_arg_parser().parse_args()

    if args.npy:
        grid_map = load_map_npy(args.npy, args.resolution, args.origin)
        map_name = os.path.splitext(os.path.basename(args.npy))[0]
    else:
        map_path = args.map if args.map else DEFAULT_MAP_PATH
        grid_map = load_map_yaml(map_path)
        map_name = os.path.splitext(os.path.basename(map_path))[0]

    if args.out is None:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        args.out = os.path.join(OUTPUT_DIR, f"{map_name}_plan.pl")

    InteractivePlanner(grid_map, args)


if __name__ == "__main__":
    main()