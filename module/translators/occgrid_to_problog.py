#!/usr/bin/env python3
"""
module/translators/occgrid_to_problog.py

Convert a ROS nav_msgs/OccupancyGrid map, given in its standard on-disk
map_server form (a .pgm image + a .yaml metadata file), into polygonal
obstacle facts for the continuous-space Golog/ProbLog action theory in
basic_action_theory.pl.

Pipeline (run TWICE, at two different inflation amounts -- see below):
  1. load the .yaml (resolution, origin, negate, thresholds) + .pgm
  2. threshold to an occupied/free binary mask using the SAME convention
     map_server itself uses
  3. INFLATE that mask outward by a clearance amount (a robot-clearance
     Minkowski dilation with a disk structuring element -- the SAME
     technique module/theory/planners.py's own inflate_obstacles
     already uses for its A*-planning grid) -- see inflate_mask below.
  4. find each connected occupied region (of the INFLATED mask) and
     extract its boundary as a polygon (cv2.findContours +
     cv2.approxPolyDP for simplification)
  5. convert every vertex from pixel coordinates to metric map-frame
     coordinates using the resolution/origin from the yaml
  6. write ONE fact per region

This is a deterministic, OFFLINE preprocessing step -- it has nothing to
do with ProbLog's probabilistic machinery and does not affect
world-count/grounding cost at all.

TWO INFLATION LEVELS, ONE FILE: obstacles_generated.pl now carries TWO
independent fact families, from TWO separate runs of steps 2-6 above
against the SAME mask, at TWO DIFFERENT clearance amounts:

  - obstacle_polygon(Id, Points) -- inflated by collision_clearance_m,
    i.e. safety_margin/1 (robot_radius+safety_buffer). This is the
    theory's own COLLISION-DETECTION geometry -- collision_geometry.py
    reads ONLY this family (see that module's own docstring): a bare
    contact/containment test against it already means "within the
    robot's own full physical safety clearance". crashed(ObstacleId),
    obstacle_in_bound(_,ObstacleId), and obstacle_on_path(_,ObstacleId)
    all report an Id from THIS family.

  - obstacle_polygon_planning(Id, Points) -- inflated by
    planning_clearance_m, i.e. robot_radius ALONE, no safety_buffer.
    This is what module/theory/planners.py's Voronoi roadmap
    (plan_voronoi) routes through: a planned path only needs to keep
    the robot's own physical BODY clear of obstacles, not the extra
    reactive safety_buffer margin collision detection itself already
    enforces independently -- baking safety_buffer into the PLANNED
    path too would needlessly narrow (or block) routes a real walk
    could still safely attempt, since obstacle_in_bound/collision are
    already watching for that margin live. follow_boarder(ObstacleId,
    Offset) is the ONE planner that deliberately does NOT use this
    family -- it looks an ObstacleId up that came FROM collision
    detection (recover_obstacle/1, built on last_halt/1), so it has to
    stay on the SAME (collision) polygon set that Id was resolved
    against; using a separately-numbered, separately-shaped planning
    set there would silently look up the wrong (or a nonexistent)
    polygon. See module/theory/planners.py's own header for exactly
    which of its functions read which family.

  Since dilation is monotonic in radius (a bigger clearance can only
  ever grow a region, never shrink it, and can merge two regions that
  stayed separate at a smaller radius), the two families are NOT
  guaranteed to have the same NUMBER of polygons, and their Id numbers
  do NOT correspond to "the same obstacle" across families -- by
  design; nothing in this theory ever needs that cross-family
  correspondence (each family is read by a fully disjoint set of
  callers, see above).

generate(yaml_path, output_path, epsilon_m, min_area_m2,
         collision_clearance_m, planning_clearance_m) is the importable
core; main.py calls it directly, before every run, exactly like it
already does for module/translators/config_to_prolog.py's own
generate() and module/translators/bt_to_prolog.py's
generate_plan_pl() -- so a problem's map.yaml is translated fresh
every run, same as config.yaml/behavior_tree.xml, with no separate
manual step to remember. main.py passes collision_clearance_m =
robot.radius + robot.safety_buffer and planning_clearance_m =
robot.radius alone, both read from THIS SAME problem's own
config.yaml, so neither can ever drift out of sync with
safety_margin/1 (collision_clearance_m) or planners.py's own
PLANNING_INFLATE_M (planning_clearance_m) on the Prolog/planner side.

Usage (the CLI wrapper, for standalone/one-off use):
    python3 module/translators/occgrid_to_problog.py map.yaml
    python3 module/translators/occgrid_to_problog.py map.yaml \
        --out obstacles_generated.pl --epsilon 0.05 --min-area 0.02 \
        --collision-clearance 0.3 --planning-clearance 0.2

By default, all generated output is written into a fixed ./maps/
directory relative to the current working directory (i.e. wherever you
invoke the script from), not next to the script or the input yaml. Pass
--out with your own path if you want to override this (main.py always
passes an explicit --out of <problem>/obstacles_generated.pl).
--collision-clearance/--planning-clearance both default to 0.0 (no
inflation) for the standalone CLI, since it has no problem directory /
config.yaml of its own to read a robot clearance from; main.py's own
pipeline call always passes the real values explicitly.
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
#    clearance Minkowski dilation with a disk structuring element,
#    line-for-line the same technique module/theory/planners.py's own
#    inflate_obstacles uses for its A*-planning grid (kept as an
#    independent copy rather than imported, same "each preprocessing
#    module owns its own copy of small, stable geometry helpers"
#    convention collision_geometry.py's own header already documents
#    for _segments_intersect/_polygon_edges). Applied to the MASK
#    (pixels), not to the polygon vertices AFTER extraction, so the
#    later contour/approxPolyDP step (step 4 below) still produces
#    clean, simplified polygon boundaries around the ALREADY-INFLATED
#    region, including correctly rounded-then-simplified outer corners
#    -- exact polygon offsetting (Minkowski sum on the vertex list
#    directly) would need separate, more involved handling for
#    concave regions and self-intersection; this reuses the
#    already-working raster pipeline instead.
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
def _polygons_at_clearance(img, resolution, origin, negate, occ_thresh,
                            epsilon_m, min_area_m2, clearance_m):
    """Steps 2-5 of the module docstring's pipeline, at ONE clearance
    amount -- called twice by generate() below, once per inflation
    level, against the SAME loaded map."""
    mask = occupied_mask(img, negate, occ_thresh)
    mask = inflate_mask(mask, resolution, clearance_m)
    return extract_polygons(mask, resolution, origin, epsilon_m, min_area_m2)


def generate(yaml_path, output_path, epsilon_m=0.05, min_area_m2=0.02,
             collision_clearance_m=0.0, planning_clearance_m=0.0):
    """Regenerate output_path (obstacle_polygon/2 +
    obstacle_polygon_planning/2 facts) from yaml_path (a map_server
    map.yaml) -- the importable core main() itself calls, same
    "importable function + thin CLI wrapper" shape as
    config_to_prolog.py's generate() and bt_to_prolog.py's
    generate_plan_pl() (both siblings of this file). main.py calls this
    directly before every run, exactly like it already does for those
    other two -- the map pipeline used to be the one manual,
    easy-to-forget step; it no longer is.

    Runs the SAME mask-inflate-extract pipeline TWICE, at TWO
    independent clearance amounts (see this module's own docstring,
    "TWO INFLATION LEVELS, ONE FILE", for exactly which downstream
    caller reads which family and why): collision_clearance_m ->
    obstacle_polygon/2 (main.py passes robot.radius+safety_buffer,
    matching safety_margin/1 in basic_action_theory.pl exactly),
    planning_clearance_m -> obstacle_polygon_planning/2 (main.py passes
    robot.radius alone). Both default to 0.0 (no inflation, reproducing
    the old uninflated behaviour) for the standalone CLI."""
    img, resolution, origin, negate, occ_thresh, free_thresh = load_map(yaml_path)
    collision_polygons = _polygons_at_clearance(
        img, resolution, origin, negate, occ_thresh, epsilon_m, min_area_m2,
        collision_clearance_m)
    planning_polygons = _polygons_at_clearance(
        img, resolution, origin, negate, occ_thresh, epsilon_m, min_area_m2,
        planning_clearance_m)
    write_problog_facts(collision_polygons, planning_polygons, output_path,
                         yaml_path, collision_clearance_m, planning_clearance_m)
    return output_path


def _write_polygon_facts(f, functor, polygons):
    for i, poly in enumerate(polygons, start=1):
        pts_str = ", ".join(f"point({x:.4f},{y:.4f})" for x, y in poly)
        f.write(f"{functor}(obs{i}, [{pts_str}]).\n")


def write_problog_facts(collision_polygons, planning_polygons, out_path,
                         source_yaml, collision_clearance_m=0.0,
                         planning_clearance_m=0.0):
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        f.write("% AUTO-GENERATED by occgrid_to_problog.py -- do not hand-edit.\n")
        f.write(f"% Source map: {source_yaml}\n")
        f.write("% Id, [point(X,Y), ...] -- vertices in metres, map frame,\n")
        f.write("% consistent with the source OccupancyGrid's origin. TWO\n")
        f.write("% independent fact families, from the SAME map at TWO\n")
        f.write("% different inflation amounts -- see this generator's own\n")
        f.write("% module docstring, \"TWO INFLATION LEVELS, ONE FILE\", for\n")
        f.write("% which caller reads which and why. Obstacle Id numbers do\n")
        f.write("% NOT correspond across the two families.\n\n")

        if collision_clearance_m > 0:
            f.write(f"% obstacle_polygon/2 -- pre-inflated by {collision_clearance_m:.4f}m\n")
            f.write("% (robot_radius+safety_buffer, this problem's own config.yaml --\n")
            f.write("% see basic_action_theory.pl's own safety_margin/1 note): every\n")
            f.write("% polygon below is the ROBOT'S OWN safety-inflated obstacle\n")
            f.write("% boundary, not the map's raw occupied cells -- a bare\n")
            f.write("% contact/containment test against these already means \"within\n")
            f.write("% the robot's own physical safety clearance\", no separate\n")
            f.write("% threshold comparison needed. Read by collision_geometry.py\n")
            f.write("% ONLY -- crashed(ObstacleId)/obstacle_in_bound(_,ObstacleId)/\n")
            f.write("% obstacle_on_path(_,ObstacleId) all report an Id from HERE.\n")
        else:
            f.write("% obstacle_polygon/2 -- NOT inflated (collision_clearance_m=0):\n")
            f.write("% the map's raw occupied cells, no robot clearance baked in.\n")
        f.write(f"% {len(collision_polygons)} obstacle region(s) extracted.\n\n")
        _write_polygon_facts(f, "obstacle_polygon", collision_polygons)

        f.write("\n")
        if planning_clearance_m > 0:
            f.write(f"% obstacle_polygon_planning/2 -- pre-inflated by {planning_clearance_m:.4f}m\n")
            f.write("% (robot_radius ALONE, no safety_buffer -- this problem's own\n")
            f.write("% config.yaml): the geometry module/theory/planners.py's own\n")
            f.write("% Voronoi roadmap (plan_voronoi) routes through -- enough\n")
            f.write("% clearance for the robot's own physical body, not the EXTRA\n")
            f.write("% reactive safety_buffer margin collision detection already\n")
            f.write("% enforces live and independently. follow_boarder(ObstacleId,\n")
            f.write("% Offset) deliberately does NOT read this family -- see this\n")
            f.write("% generator's own module docstring.\n")
        else:
            f.write("% obstacle_polygon_planning/2 -- NOT inflated (planning_clearance_m=0):\n")
            f.write("% the map's raw occupied cells, no robot clearance baked in.\n")
        f.write(f"% {len(planning_polygons)} obstacle region(s) extracted.\n\n")
        _write_polygon_facts(f, "obstacle_polygon_planning", planning_polygons)

    print(f"Wrote {len(collision_polygons)} collision + {len(planning_polygons)} "
          f"planning obstacle polygon(s) to {out_path}")


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
    ap.add_argument("--collision-clearance", type=float, default=0.0,
                     help="Inflate the occupied mask by this many METRES "
                          "before extracting obstacle_polygon/2 (the "
                          "collision-detection family) -- normally "
                          "robot_radius+safety_buffer (main.py passes this "
                          "explicitly from the problem's own config.yaml). "
                          "Default 0.0 (no inflation).")
    ap.add_argument("--planning-clearance", type=float, default=0.0,
                     help="Inflate the occupied mask by this many METRES "
                          "before extracting obstacle_polygon_planning/2 "
                          "(the planner-facing family) -- normally "
                          "robot_radius alone, no safety_buffer (main.py "
                          "passes this explicitly). Default 0.0 (no "
                          "inflation).")
    args = ap.parse_args()

    # Fixed default output location: ./maps/obstacles_generated.pl
    out_path = args.out
    if out_path is None:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        out_path = os.path.join(OUTPUT_DIR, "obstacles_generated.pl")

    generate(args.yaml_path, out_path, args.epsilon, args.min_area,
              args.collision_clearance, args.planning_clearance)


if __name__ == "__main__":
    main()