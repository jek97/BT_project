#!/usr/bin/env python3
"""
module/translators/occgrid_to_problog.py

Convert a ROS nav_msgs/OccupancyGrid map, given in its standard on-disk
map_server form (a .pgm image + a .yaml metadata file), into polygonal
obstacle facts for the continuous-space Golog/ProbLog action theory in
basic_action_theory.pl.

Pipeline:
  1. load the .yaml (resolution, origin, negate, thresholds) + .pgm
  2. threshold to an occupied/free binary mask using the SAME convention
     map_server itself uses
  3. INFLATE that mask outward by clearance_m (a robot-clearance
     Minkowski dilation with a disk structuring element -- the SAME
     technique module/theory/planners.py's own inflate_obstacles used
     to use for its own, now-removed, SEPARATE A*-planning inflation)
     -- see inflate_mask below.
  4. find each connected occupied region (of the INFLATED mask) and
     extract its boundary as a polygon (cv2.findContours +
     cv2.approxPolyDP for simplification)
  5. convert every vertex from pixel coordinates to metric map-frame
     coordinates using the resolution/origin from the yaml
  6. write ONE obstacle_polygon(Id, [point(X,Y),...]) fact per region

This is a deterministic, OFFLINE preprocessing step -- it has nothing to
do with ProbLog's probabilistic machinery and does not affect
world-count/grounding cost at all.

ONE OBSTACLE REPRESENTATION, USED EVERYWHERE: obstacle_polygon/2 is
inflated by clearance_m, i.e. safety_margin/1 (robot_radius+
safety_buffer) -- the ONLY obstacle geometry this project generates,
and every consumer (collision_geometry.py's own collision/
obstacle_in_bound/obstacle_on_path geometry, AND module/theory/
planners.py's plan_astar/plan_voronoi/follow_boarder) reads THIS SAME
already-inflated polygon set, rather than each owning its own separate
inflation policy:
  - collision_geometry.py: a bare contact/containment test against it
    already means "within the robot's own full physical safety
    clearance" -- see basic_action_theory.pl's own safety_margin/1
    note (first_collision_time/7 calls the generic crossing-time
    machinery at Threshold=0.0 for exactly this reason).
  - plan_astar/plan_voronoi: route through it directly, with NO
    additional inflation of their own -- see planners.py's own module
    docstring for why a SEPARATE, lesser "planning-only" inflation
    amount was tried and then dropped in favour of this single,
    uniform one.
  - follow_boarder(ObstacleId, Offset): also reads this SAME polygon
    (it has to -- ObstacleId always came FROM collision detection, so
    it must resolve against the exact polygon set that Id was reported
    against), but SUBTRACTS safety_margin back out of its own Offset
    argument before applying it as an additional per-edge push -- see
    planners.py's own _follow_boarder_control_points note for the
    "already-inflated boundary + (Offset-safety_margin)" arithmetic.

generate(yaml_path, output_path, epsilon_m, min_area_m2, clearance_m)
is the importable core (steps 1-6 above); main.py calls it directly,
before every run, exactly like it already does for module/translators/
config_to_prolog.py's own generate() and module/translators/
bt_to_prolog.py's generate_plan_pl() -- so a problem's map.yaml is
translated fresh every run, same as config.yaml/behavior_tree.xml, with
no separate manual step to remember. main.py passes clearance_m =
robot.radius + robot.safety_buffer, read from THIS SAME problem's own
config.yaml, so the inflation amount can never drift out of sync with
safety_margin/1 on the Prolog side.

Usage (the CLI wrapper, for standalone/one-off use):
    python3 module/translators/occgrid_to_problog.py map.yaml
    python3 module/translators/occgrid_to_problog.py map.yaml \
        --out obstacles_generated.pl --epsilon 0.05 --min-area 0.02 \
        --clearance 0.3

By default, all generated output is written into a fixed ./maps/
directory relative to the current working directory (i.e. wherever you
invoke the script from), not next to the script or the input yaml. Pass
--out with your own path if you want to override this (main.py always
passes an explicit --out of <problem>/obstacles_generated.pl).
--clearance defaults to 0.0 (no inflation) for the standalone CLI,
since it has no problem directory / config.yaml of its own to read a
robot clearance from; main.py's own pipeline call always passes the
real value explicitly.
"""
import argparse
import os
import sys

import numpy as np
import yaml
import cv2
from scipy.ndimage import binary_dilation

# Fixed output directory, relative to the current working directory.
OUTPUT_DIR = os.path.join(os.getcwd(), "maps")


# ---------------------------------------------------------------------
# 1. Load map.yaml + the referenced .pgm
# ---------------------------------------------------------------------
def load_map(yaml_path):
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    image_path = meta["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                                   image_path)

    resolution = float(meta["resolution"])          # metres / pixel
    origin = meta["origin"]                          # [x, y, theta] of pixel (0,H-1)
    negate = int(meta.get("negate", 0))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not read map image: {image_path}")

    return img, resolution, origin, negate, occupied_thresh, free_thresh


# ---------------------------------------------------------------------
# 2. Threshold to an occupied binary mask, using map_server's own
#    convention so this matches what the robot's real costmap sees.
# ---------------------------------------------------------------------
def occupied_mask(img, negate, occupied_thresh):
    """
    map_server convention:
      norm       = pixel / 255                (pixel "white-ness")
      occ_prob   = norm            if negate == 1
                 = 1 - norm        if negate == 0
      cell is OCCUPIED  if occ_prob > occupied_thresh
    """
    norm = img.astype(np.float64) / 255.0
    occ_prob = norm if negate else (1.0 - norm)
    mask = (occ_prob > occupied_thresh).astype(np.uint8) * 255
    return mask


# ---------------------------------------------------------------------
# 3. Inflate the occupied mask outward by clearance_m -- a robot-
#    clearance Minkowski dilation with a disk structuring element.
#    Applied to the MASK (pixels), not to the polygon vertices AFTER
#    extraction, so the later contour/approxPolyDP step (step 4 below)
#    still produces clean, simplified polygon boundaries around the
#    ALREADY-INFLATED region, including correctly rounded-then-
#    simplified outer corners -- exact polygon offsetting (Minkowski
#    sum on the vertex list directly) would need separate, more
#    involved handling for concave regions and self-intersection; this
#    reuses the already-working raster pipeline instead.
# ---------------------------------------------------------------------
def inflate_mask(mask, resolution, clearance_m):
    """mask: the {0,255} occupied mask from occupied_mask() above.
    Returns a copy grown by clearance_m in every direction (a no-op
    copy if clearance_m <= 0)."""
    if clearance_m <= 0:
        return mask
    radius_px = max(1, int(round(clearance_m / resolution)))
    yy, xx = np.ogrid[-radius_px:radius_px + 1, -radius_px:radius_px + 1]
    disk = (xx ** 2 + yy ** 2) <= radius_px ** 2
    inflated = binary_dilation(mask > 0, structure=disk)
    return inflated.astype(np.uint8) * 255


# ---------------------------------------------------------------------
# 4+5. Extract simplified polygon contours and convert to map-frame
#      metres. PGM row 0 is the TOP of the image; map_server's origin
#      is the metric position of the BOTTOM-LEFT pixel, so row index
#      must be flipped when converting to the y coordinate.
# ---------------------------------------------------------------------
def pixel_to_map(row, col, height, resolution, origin):
    ox, oy = origin[0], origin[1]
    x = ox + col * resolution
    y = oy + (height - 1 - row) * resolution
    return x, y


def extract_polygons(mask, resolution, origin, epsilon_m, min_area_m2):
    height = mask.shape[0]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    polygons = []
    for cnt in contours:
        area_m2 = cv2.contourArea(cnt) * (resolution ** 2)
        if area_m2 < min_area_m2:
            continue  # discard single-pixel / sensor-noise specks

        epsilon_px = max(epsilon_m / resolution, 0.5)
        approx = cv2.approxPolyDP(cnt, epsilon_px, closed=True)

        pts = []
        for p in approx.reshape(-1, 2):
            col, row = int(p[0]), int(p[1])
            x, y = pixel_to_map(row, col, height, resolution, origin)
            pts.append((x, y))

        if len(pts) >= 3:
            polygons.append(pts)

    return polygons


# ---------------------------------------------------------------------
# 6. Write the ProbLog facts
# ---------------------------------------------------------------------
def generate(yaml_path, output_path, epsilon_m=0.05, min_area_m2=0.02, clearance_m=0.0):
    """Regenerate output_path (obstacle_polygon/2 facts) from yaml_path (a
    map_server map.yaml) -- the importable core main() itself calls, same
    "importable function + thin CLI wrapper" shape as
    config_to_prolog.py's generate() and bt_to_prolog.py's
    generate_plan_pl() (both siblings of this file). main.py calls this
    directly before every run, exactly like it already does for those
    other two -- the map pipeline used to be the one manual,
    easy-to-forget step; it no longer is.

    clearance_m inflates the occupied mask (see inflate_mask above)
    BEFORE polygon extraction, so obstacle_polygon/2 already encodes
    the robot's own safety clearance -- main.py passes robot.radius +
    robot.safety_buffer (this problem's own config.yaml, matching
    safety_margin/1 in basic_action_theory.pl exactly); 0.0 (the
    default here) reproduces the old, uninflated behaviour. This is
    the ONLY obstacle geometry this generator produces -- see this
    module's own docstring for why every consumer (collision detection
    AND every planner) reads this SAME already-inflated set instead of
    each owning a separate copy at a separate inflation amount."""
    img, resolution, origin, negate, occ_thresh, free_thresh = load_map(yaml_path)
    mask = occupied_mask(img, negate, occ_thresh)
    mask = inflate_mask(mask, resolution, clearance_m)
    polygons = extract_polygons(mask, resolution, origin, epsilon_m, min_area_m2)
    write_problog_facts(polygons, output_path, yaml_path, clearance_m)
    return output_path


def write_problog_facts(polygons, out_path, source_yaml, clearance_m=0.0):
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        f.write("% AUTO-GENERATED by occgrid_to_problog.py -- do not hand-edit.\n")
        f.write(f"% Source map: {source_yaml}\n")
        f.write("% obstacle_polygon(Id, [point(X,Y), ...]) -- vertices in metres,\n")
        f.write("% map frame, consistent with the source OccupancyGrid's origin.\n")
        if clearance_m > 0:
            f.write(f"% Pre-inflated by {clearance_m:.4f}m (robot_radius+safety_buffer,\n")
            f.write("% this problem's own config.yaml -- see basic_action_theory.pl's\n")
            f.write("% own safety_margin/1 note): every polygon below is the ROBOT'S\n")
            f.write("% OWN safety-inflated obstacle boundary, not the map's raw occupied\n")
            f.write("% cells. This is the ONLY obstacle geometry generated for this\n")
            f.write("% problem -- collision_geometry.py AND every planner in\n")
            f.write("% module/theory/planners.py (plan_astar, plan_voronoi,\n")
            f.write("% follow_boarder) all read this SAME already-inflated set; see\n")
            f.write("% planners.py's own module docstring for how follow_boarder\n")
            f.write("% corrects its own Offset argument for the inflation already\n")
            f.write("% baked in here.\n")
        else:
            f.write("% NOT inflated (clearance_m=0) -- these are the map's raw occupied\n")
            f.write("% cells, with no robot safety clearance baked in.\n")
        f.write(f"% {len(polygons)} obstacle region(s) extracted.\n\n")
        for i, poly in enumerate(polygons, start=1):
            pts_str = ", ".join(f"point({x:.4f},{y:.4f})" for x, y in poly)
            f.write(f"obstacle_polygon(obs{i}, [{pts_str}]).\n")
    print(f"Wrote {len(polygons)} obstacle polygon(s) to {out_path}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("yaml_path", help="Path to the map .yaml (map_server format)")
    ap.add_argument("--out", default=None,
                     help="Output ProbLog facts file. Defaults to "
                          "./maps/obstacles_generated.pl (relative to the "
                          "current working directory).")
    ap.add_argument("--epsilon", type=float, default=0.05,
                     help="Polygon simplification tolerance in METRES "
                          "(cv2.approxPolyDP epsilon). Larger = fewer "
                          "vertices per obstacle, looser fit. Default 0.05.")
    ap.add_argument("--min-area", type=float, default=0.02,
                     help="Discard connected obstacle regions smaller than "
                          "this many SQUARE METRES (filters single-pixel "
                          "sensor noise from the raw grid). Default 0.02.")
    ap.add_argument("--clearance", type=float, default=0.0,
                     help="Inflate the occupied mask by this many METRES "
                          "before extracting obstacle polygons -- normally "
                          "robot_radius+safety_buffer (main.py passes this "
                          "explicitly from the problem's own config.yaml). "
                          "Default 0.0 (no inflation).")
    args = ap.parse_args()

    # Fixed default output location: ./maps/obstacles_generated.pl
    out_path = args.out
    if out_path is None:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        out_path = os.path.join(OUTPUT_DIR, "obstacles_generated.pl")

    generate(args.yaml_path, out_path, args.epsilon, args.min_area, args.clearance)


if __name__ == "__main__":
    main()
