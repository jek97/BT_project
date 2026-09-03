"""
moveto_calibrator.py

Offline calibration for the simplified atomic moveto(LegId,Algorithm,Goal)
action theory (see basic_action_theory.pl's own header and Section 2's
moveto_outcome/7 interface documentation for the contract calibrate_moveto()'s
output must satisfy).

SCOPE: position noise (z, tangential zt) AND battery drain noise (zbatt)
are drawn from the SAME discretized-Gaussian tables config.yaml already
declares (noise.position/noise.tangential/noise.battery), the exact
values and sigmas ProbLog's own z/2, zt/2, zbatt/1 facts used before
this branch's simplification -- not reinvented, read straight out of
the same config. For each (z, zt, zbatt) combination this function:
  1. computes the deviated landing point the SAME Brownian-bridge-
     consistent way _walk_noisy_point in collision_geometry.py always
     did (sigma * sqrt(duration), perpendicular + tangential unit
     vectors), just evaluated once at completion instead of resampled
     continuously along a walk;
  2. checks whether the straight-line path from start to that deviated
     point comes within safety_margin of any obstacle, reusing
     collision_geometry.py's own point/segment/polygon distance
     primitives directly (not reimplemented) -- see
     _path_min_clearance's own docstring for exactly how;
  3. computes battery drained using the SAME noisy-deviation-from-
     nominal formula battery/3's own moving-phase clause used
     (NominalDrain +/- Zb*SigmaBattery*sqrt(Duration)), and classifies
     battery_depleted if that would exceed the leg's own incoming
     battery level.
Collision is checked before battery depletion (a crash preempts
whatever would have happened to the battery); everything else is
Reason='success'.

NOT YET modeled: reactive threshold crossings (battery_below(T),
obstacle_in_bound(T), obstacle_on_path(T), ...) -- those need the
`triggers` list actually threaded through from the plan tree (currently
always []), and, for the battery ones, some notion of "did drain-so-far
cross T at some point before completion" rather than only "did it
finish above/below T", which the atomic (no continuous elapsed time)
model doesn't carry for free. Both are separate follow-up work, not
silently skipped -- see basic_action_theory.pl Section 2's own note on
the full Reason vocabulary a complete calibrator eventually needs.

Using genuine Wiener-process math and real obstacle geometry INTERNALLY
here does NOT reintroduce the combinatorial blowup the simplified
theory exists to avoid: all of it runs OFFLINE, once per (leg,
incoming-branch) pair (three independent axes now, so a bigger flat
table -- e.g. 5x5x5=125 rows instead of 25 -- but still exactly ONE
flat annotated disjunction per pair, referenced once by ProbLog, never
re-derived through a live search or accumulated across hops). The
blowup was never caused by the noise model's or the geometry's own
sophistication; it was caused by exposing z/zt/zbatt as ProbLog's OWN
random variables, re-referenced through a live bracket-scan search.
Nothing here does that.

Currently only algorithm="straight" is implemented (the nominal path is
the direct segment from start to goal). astar/voronoi/follow_boarder
fall back to the straight-line distance/direction with a documented
TODO -- see _nominal_path_straight's own note.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from collision_geometry import _dist_to_polygon  # noqa: E402 -- reused, not reimplemented


def _nominal_path_straight(start, goal):
    """(distance, tangent_unit, perp_unit) for the direct segment from
    start to goal. astar/voronoi/follow_boarder would need their own
    nominal-path shape (via planners.py) to compute a genuine arc
    length and end tangent direction instead of the straight-line
    approximation used here -- not yet implemented; every algorithm
    currently falls back to this."""
    sx, sy = start
    gx, gy = goal
    dx, dy = gx - sx, gy - sy
    norm = math.hypot(dx, dy)
    if norm <= 1.0e-9:
        return 0.0, (0.0, 0.0), (0.0, 0.0)
    tan_x, tan_y = dx / norm, dy / norm
    perp_x, perp_y = -dy / norm, dx / norm
    return norm, (tan_x, tan_y), (perp_x, perp_y)


def _path_min_clearance(start, end_point, obstacle_polygons, num_samples):
    """Minimum clearance from the STRAIGHT-LINE path start->end_point to
    any obstacle, and which obstacle -- (min_distance, obstacle_id),
    obstacle_id=None if no obstacles are closer than the "so far away
    it never matters" case (or there are no obstacles at all).

    Samples num_samples points along the path and takes the min of
    _dist_to_polygon (collision_geometry.py's own point-to-polygon
    distance, reused directly) over every obstacle at every sample --
    a modest, FINITE sampling, not a bisection search: this runs once
    per branch, offline, so there is no live-inference cost to bound
    the way the old theory's bracket_samples/crossing_eps had to. Does
    NOT check whether an endpoint sample itself is inside a polygon
    (an "already past the boundary" case _dist_to_polygon alone
    doesn't distinguish) -- a real calibrator would want
    collision_geometry.py's own _signed_clearance for that; left as a
    known gap here, not silently assumed away, since it doesn't affect
    problem4's own geometry (no obstacle ever comes close to its
    path)."""
    if not obstacle_polygons:
        return float("inf"), None
    sx, sy = start
    ex, ey = end_point
    best_dist = float("inf")
    best_id = None
    for i in range(num_samples + 1):
        t = i / num_samples if num_samples > 0 else 0.0
        px, py = sx + t * (ex - sx), sy + t * (ey - sy)
        for obstacle_id, points in obstacle_polygons:
            d = _dist_to_polygon(px, py, points)
            if d < best_dist:
                best_dist = d
                best_id = obstacle_id
    return best_dist, best_id


def calibrate_moveto(leg_id, in_branch, start, battery_in, goal, algorithm,
                      triggers, config, obstacle_polygons=()):
    """calibrate_moveto(LegId, InBranch, Start, BatteryIn, Goal, Algorithm,
    Triggers, Config, ObstaclePolygons) -> list of row dicts, one per
    outcome branch, probabilities summing to 1.0. See this file's own
    header for exactly what's modeled (position/tangential/battery
    noise, obstacle collision, battery depletion) and what isn't yet
    (reactive threshold crossings).

    LegId/InBranch: pure labels, not used in the computation below --
    threaded through by the caller for logging/error messages only.
    Triggers: accepted per the interface contract; not yet consumed
    (see header).
    """
    del in_branch, triggers  # not yet used, see docstring

    distance, (tan_x, tan_y), (perp_x, perp_y) = _nominal_path_straight(start, goal)

    speed = config["motion"]["speed"]
    duration = distance / speed if speed > 0 else 0.0

    moving_drain_rate = config["battery"]["moving_drain_rate"]
    nominal_drain = duration * moving_drain_rate

    sigma_pos = config["noise"]["position"]["sigma"]
    sigma_tan = config["noise"]["tangential"]["sigma"]
    sigma_batt = config["noise"]["battery"]["sigma"]
    pos_table = config["noise"]["position"]["discretized_gaussian"]
    tan_table = config["noise"]["tangential"]["discretized_gaussian"]
    batt_table = config["noise"]["battery"]["discretized_gaussian"]

    safety_margin = config["robot"]["radius"] + config["robot"]["safety_buffer"]
    num_samples = int(config["verification"]["bracket_samples"])

    gx, gy = goal
    rows = []
    n = 0
    for pos_entry in pos_table:
        for tan_entry in tan_table:
            z = pos_entry["value"]
            zt = tan_entry["value"]
            normal_dev = z * sigma_pos * math.sqrt(duration) if duration > 0 else 0.0
            tangent_dev = zt * sigma_tan * math.sqrt(duration) if duration > 0 else 0.0
            ex = gx + normal_dev * perp_x + tangent_dev * tan_x
            ey = gy + normal_dev * perp_y + tangent_dev * tan_y

            clearance, obstacle_id = _path_min_clearance(
                start, (ex, ey), obstacle_polygons, num_samples)
            collided = clearance < safety_margin

            for batt_entry in batt_table:
                n += 1
                zb = batt_entry["value"]
                batt_dev = zb * sigma_batt * math.sqrt(duration) if duration > 0 else 0.0
                drain = max(0.0, nominal_drain + batt_dev)
                probability = pos_entry["weight"] * tan_entry["weight"] * batt_entry["weight"]

                if collided:
                    reason = ("crashed", obstacle_id)
                elif drain >= battery_in:
                    reason = "battery_depleted"
                else:
                    reason = "success"

                rows.append({
                    "branch_id": f"p{n}",
                    "probability": probability,
                    "reason": reason,
                    "end_point": (ex, ey),
                    "duration": duration,
                    "drain": drain,
                })
    return rows
