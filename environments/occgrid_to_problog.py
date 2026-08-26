#!/usr/bin/env python3
"""
occgrid_to_problog.py

Convert a ROS nav_msgs/OccupancyGrid map, given in its standard on-disk
map_server form (a .pgm image + a .yaml metadata file), into polygonal
obstacle facts for the continuous-space Golog/ProbLog action theory in
moveto_continuous.pl.

Pipeline:
  1. load the .yaml (resolution, origin, negate, thresholds) + .pgm
  2. threshold to an occupied/free binary mask using the SAME convention
     map_server itself uses
  3. find each connected occupied region and extract its boundary as a
     polygon (cv2.findContours + cv2.approxPolyDP for simplification)
  4. convert every vertex from pixel coordinates to metric map-frame
     coordinates using the resolution/origin from the yaml
  5. write ONE obstacle_polygon(Id, [point(X,Y),...]) fact per region

This is a one-time, deterministic, OFFLINE preprocessing step -- it has
nothing to do with ProbLog's probabilistic machinery and does not affect
world-count/grounding cost at all. Run it once whenever the map changes,
then consult the resulting file from moveto_continuous.pl.

Usage:
    python3 occgrid_to_problog.py map.yaml
    python3 occgrid_to_problog.py map.yaml --out obstacles_generated.pl \
        --epsilon 0.05 --min-area 0.02 --inline PLAN_FILE.pl

By default, all generated output is written into a fixed ./maps/
directory relative to the current working directory (i.e. wherever you
invoke the script from), not next to the script or the input yaml. Pass
--out with your own path if you want to override this.

--inline PLAN_FILE.pl additionally rewrites start/2 and goal/2 in the
given action-theory file if --start/--goal are supplied (handy for
scripting a full map -> theory pipeline), leaving control_points/1
untouched, since you said you'll fill that in by hand.
"""
import argparse
import os
import sys

import numpy as np
import yaml
import cv2

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
# 3+4. Extract simplified polygon contours and convert to map-frame
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
# 5. Write the ProbLog facts
# ---------------------------------------------------------------------
def write_problog_facts(polygons, out_path, source_yaml):
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        f.write("% AUTO-GENERATED by occgrid_to_problog.py -- do not hand-edit.\n")
        f.write(f"% Source map: {source_yaml}\n")
        f.write("% obstacle_polygon(Id, [point(X,Y), ...]) -- vertices in metres,\n")
        f.write("% map frame, consistent with the source OccupancyGrid's origin.\n")
        f.write(f"% {len(polygons)} obstacle region(s) extracted.\n\n")
        for i, poly in enumerate(polygons, start=1):
            pts_str = ", ".join(f"point({x:.4f},{y:.4f})" for x, y in poly)
            f.write(f"obstacle_polygon(obs{i}, [{pts_str}]).\n")
    print(f"Wrote {len(polygons)} obstacle polygon(s) to {out_path}")


# ---------------------------------------------------------------------
# Optional: patch start/2 and goal/2 into the action-theory file.
# control_points/1 is intentionally left untouched -- the user fills
# that in by hand once the spline is chosen.
# ---------------------------------------------------------------------
def patch_start_goal(theory_path, start_xy, goal_xy):
    import re
    with open(theory_path, "r") as f:
        text = f.read()

    # Only match FACT definitions with numeric arguments, anchored at the
    # start of a line -- NOT other uses like "at(X,Y,_,s0) :- start(X,Y)."
    # which use variable names and must be left untouched.
    num = r"-?\d+(?:\.\d+)?"
    if start_xy is not None:
        pattern = re.compile(rf"^start\(\s*{num}\s*,\s*{num}\s*\)\.", re.MULTILINE)
        new_start = f"start({start_xy[0]:.4f}, {start_xy[1]:.4f})."
        text, n = pattern.subn(new_start, text, count=1)
        if n == 0:
            print("[warn] no numeric start(X,Y). fact found to patch", file=sys.stderr)
    if goal_xy is not None:
        pattern = re.compile(rf"^goal\(\s*{num}\s*,\s*{num}\s*\)\.", re.MULTILINE)
        new_goal = f"goal({goal_xy[0]:.4f}, {goal_xy[1]:.4f})."
        text, n = pattern.subn(new_goal, text, count=1)
        if n == 0:
            print("[warn] no numeric goal(X,Y). fact found to patch", file=sys.stderr)

    with open(theory_path, "w") as f:
        f.write(text)
    print(f"Patched start/goal in {theory_path}")


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
    ap.add_argument("--inline", default=None,
                     help="Optionally patch start/2 and goal/2 into this "
                          "action-theory .pl file (control_points/1 is left "
                          "untouched -- fill that in by hand).")
    ap.add_argument("--start", nargs=2, type=float, default=None,
                     metavar=("X", "Y"), help="Start position in metres, "
                     "requires --inline")
    ap.add_argument("--goal", nargs=2, type=float, default=None,
                     metavar=("X", "Y"), help="Goal position in metres, "
                     "requires --inline")
    args = ap.parse_args()

    # Fixed default output location: ./maps/obstacles_generated.pl
    out_path = args.out
    if out_path is None:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        out_path = os.path.join(OUTPUT_DIR, "obstacles_generated.pl")

    img, resolution, origin, negate, occ_thresh, free_thresh = load_map(args.yaml_path)
    mask = occupied_mask(img, negate, occ_thresh)

    polygons = extract_polygons(mask, resolution, origin,
                                 args.epsilon, args.min_area)

    write_problog_facts(polygons, out_path, args.yaml_path)

    if args.inline:
        if not os.path.isfile(args.inline):
            print(f"[warn] --inline target {args.inline} not found, skipping "
                  f"start/goal patch", file=sys.stderr)
        else:
            patch_start_goal(args.inline, args.start, args.goal)


if __name__ == "__main__":
    main()