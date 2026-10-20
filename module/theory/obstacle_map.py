#!/usr/bin/env python3
"""
module/theory/obstacle_map.py

The problem's obstacle map, built IN MEMORY from a ROS nav_msgs/
OccupancyGrid in its standard map_server on-disk form (a .pgm image + a
.yaml metadata file), for the Python black boxes basic_action_theory.pl
:- use_module()s (collision_geometry.py, planners.py). Nothing is
written to disk and nothing reaches ProbLog: the Prolog theory never
reads obstacle geometry itself -- every geometric question goes through
one of those Python predicates.

(Formerly module/translators/occgrid_to_problog.py, which wrote the
same polygons to <problem>/obstacles_generated.pl as obstacle_polygon/2
and obstacle_hole/2 facts for the Python modules to parse back. The
polygons, their obsN Ids and their 4-decimal coordinates are unchanged.)

Pipeline (obstacle_polygons()):
  1. load the .yaml (resolution, origin, negate, thresholds) + .pgm
  2. threshold to an occupied/free binary mask using the SAME convention
     map_server itself uses
  3. INFLATE that mask outward by the robot clearance (robot.radius +
     robot.safety_buffer from the problem's config.yaml) -- a Minkowski
     dilation with a disk structuring element, see inflate_mask below
  4. find each connected occupied region (of the INFLATED mask) and
     extract its OUTER boundary as a polygon, PLUS the boundary of any
     HOLE within it -- a hollow, walkable interior, e.g. a perimeter
     fence's own inner face (cv2.findContours(RETR_CCOMP,...) +
     cv2.approxPolyDP for simplification -- see extract_polygons)
  5. convert every vertex from pixel coordinates to metric map-frame
     coordinates using the resolution/origin from the yaml
  6. name the regions obs1, obs2, ... in extraction order

Result: [(Id, rings), ...] with rings = [outer_points, hole1_points,
...] -- rings[0] is ALWAYS the outer boundary, rings[1:] that same
obstacle's holes (empty for an ordinary obstacle). Every consumer
combines an obstacle's rings into ONE containment/clearance test (see
collision_geometry.py's _inside_polygon), so a hole's free interior
reads as free space.

ONE OBSTACLE REPRESENTATION, USED EVERYWHERE: the polygons are inflated
by the robot's own safety clearance, and every consumer reads this same
set:
  - collision_geometry.py: names the regions of its raster distance map
    (crashed(obsN), obstacle_in_bound(T,obsN)) and backs the Bug-
    algorithm line-of-sight / segment primitives;
  - planners.py: follow_boarder(ObstacleId, Offset) walks an
    obstacle's boundary (subtracting the clearance back out of Offset),
    plan_voronoi samples the boundaries.

Built once per process and problem directory (obstacle_polygons() is
cached), and imported as a plain module (no ProbLog predicates here),
so both black boxes share the same in-memory copy.
"""
import functools
import os
import sys

import numpy as np
import yaml
import cv2
from scipy.ndimage import binary_dilation

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
    """Returns [(outer_points, [hole_points, ...]), ...] -- one entry
    per TOP-LEVEL connected obstacle region, each carrying its own
    zero-or-more HOLE boundaries (a hollow, walkable interior -- e.g.
    a perimeter fence's own inner face) as SEPARATE rings, rather than
    folding them into the outer boundary's own point list (which would
    make a naive point-in-polygon test treat the hollow interior as
    solid). See basic_action_theory.pl's own obstacle_hole/2 note for
    the full rationale and how consumers (collision_geometry.py,
    planners.py) combine outer+hole rings back into one containment
    test.

    cv2.RETR_CCOMP retrieves contours in exactly a TWO-level hierarchy
    -- top level: external boundaries of each connected component;
    second level: boundaries of HOLES within them -- which matches
    this geometry precisely: a hole's own interior can itself contain
    a genuinely separate object (e.g. a tree standing inside a fenced
    yard), but that object's own boundary is STILL reported at the TOP
    level (its own, unrelated connected component), never nested a
    third level deep, so an ordinary hole-less obstacle is completely
    unaffected by any of this -- RETR_EXTERNAL (the previous choice)
    only ever saw the top level, silently discarding hole information
    entirely.
    """
    height = mask.shape[0]
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return []
    hierarchy = hierarchy[0]  # cv2 wraps the (N,4) array in an extra outer axis

    epsilon_px = max(epsilon_m / resolution, 0.5)

    def to_map_points(cnt):
        approx = cv2.approxPolyDP(cnt, epsilon_px, closed=True)
        return [pixel_to_map(int(p[1]), int(p[0]), height, resolution, origin)
                for p in approx.reshape(-1, 2)]

    polygons = []
    for i, cnt in enumerate(contours):
        _next, _prev, first_child, parent = hierarchy[i]
        if parent != -1:
            continue  # a hole -- collected below, under its own parent's entry

        area_m2 = cv2.contourArea(cnt) * (resolution ** 2)
        if area_m2 < min_area_m2:
            continue  # discard single-pixel / sensor-noise specks
        outer_pts = to_map_points(cnt)
        if len(outer_pts) < 3:
            continue

        holes = []
        child = first_child
        while child != -1:
            hole_area_m2 = cv2.contourArea(contours[child]) * (resolution ** 2)
            if hole_area_m2 >= min_area_m2:
                hole_pts = to_map_points(contours[child])
                if len(hole_pts) >= 3:
                    holes.append(hole_pts)
            child = hierarchy[child][0]  # next sibling hole, same level

        polygons.append((outer_pts, holes))

    return polygons


# ---------------------------------------------------------------------
# 6. In-memory obstacle set for one problem directory
# ---------------------------------------------------------------------
EPSILON_M = 0.05     # polygon simplification tolerance (cv2.approxPolyDP), metres
MIN_AREA_M2 = 0.02   # regions smaller than this are sensor-noise specks, dropped


def clearance_m(problem_dir):
    """robot.radius + robot.safety_buffer from the problem's config.yaml
    -- the inflation amount, matching safety_margin/1 on the Prolog side."""
    translators = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                               "translators")
    if translators not in sys.path:
        sys.path.insert(0, translators)
    from config_to_prolog import load_config
    robot = load_config(config_path=os.path.join(problem_dir, "config.yaml"))["robot"]
    return float(robot["radius"]) + float(robot["safety_buffer"])


def _round4(points):
    # same 4-decimal coordinates the former obstacles_generated.pl carried
    return [(float(f"{x:.4f}"), float(f"{y:.4f}")) for x, y in points]


@functools.lru_cache(maxsize=None)
def obstacle_polygons(problem_dir):
    """[(Id, rings), ...] for problem_dir's own map.yaml, inflated by its
    own clearance_m -- see this module's docstring. [] if the problem has
    no map.yaml (a map-less problem has no obstacles)."""
    yaml_path = os.path.join(problem_dir, "map.yaml")
    if not os.path.isfile(yaml_path):
        return []
    img, resolution, origin, negate, occ_thresh, _free = load_map(yaml_path)
    mask = inflate_mask(occupied_mask(img, negate, occ_thresh), resolution,
                        clearance_m(problem_dir))
    polygons = extract_polygons(mask, resolution, origin, EPSILON_M, MIN_AREA_M2)
    return [(f"obs{i}", [_round4(outer)] + [_round4(h) for h in holes])
            for i, (outer, holes) in enumerate(polygons, start=1)]


if __name__ == "__main__":
    # quick look: python3 module/theory/obstacle_map.py problems/<name>
    for oid, rings in obstacle_polygons(os.path.abspath(sys.argv[1])):
        print(oid, f"{len(rings[0])} vertices", f"{len(rings) - 1} hole(s)")
