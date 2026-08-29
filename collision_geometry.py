#!/usr/bin/env python3
"""
collision_geometry.py

PROBLOG EXTERNAL-PREDICATE MODULE -- imported directly by ProbLog's own
:- use_module('./collision_geometry.py'). directive inside
moveto_continuous.pl, same mechanism as actions/moveto_planners.py's own
:- use_module(...) directive (see that file's header for the mechanics).
Lives NEXT TO moveto_continuous.pl, not inside ./actions/ -- this isn't
a BT.cpp-facing node/condition, it's an internal performance black box
for the action theory's own obstacle-clearance geometry.

WHY THIS EXISTS: moveto_continuous.pl used to compute
first_threshold_crossing_time/6 (the earliest time a noisy trajectory
comes within a given distance of any obstacle -- the basis for the
`collision` trigger and, now, the generic `obstacle_in_bound(Threshold)`
trigger too) entirely in Prolog: a
60-sample bracket scan, each sample checking clearance against every
obstacle polygon's every edge, followed by a bisection refinement. None
of that computation is actually PROBABILISTIC once Z (the resolved
lateral-noise draw) is fixed -- it's a deterministic function of
(ControlPoints, T0, Duration, Z, Threshold, the obstacle set). But
ProbLog's grounding materializes a full proof-tree node for every one of
those bracket/bisection steps anyway, for every resolved Z, which is
what made collision detection scale so badly with obstacle count/
complexity (see moveto_continuous.pl's own note above dist/5). Moving it
here collapses that whole grounding subtree into ONE black-box call per
resolved world -- exactly the same "stateless computation, no frame
problem, so no reason to pay Reiter's machinery's cost" argument already
used to justify planWith/plan_call being a Python black box instead of
Prolog clauses.

CORRECTNESS: the algorithm below is a LINE-FOR-LINE port of
moveto_continuous.pl's former Prolog implementation (same bracket-scan
sample count, same bisection epsilon, same spline/noise formulas) --
not a re-derivation. bracket_samples, crossing_eps, and position_sigma
are read directly out of config/config.yaml at import time (the SAME
file config/generate_prolog_config.py turns into
config/config_generated.pl for the Prolog side, see that module's own
header) rather than hardcoded a second time here, so config.yaml stays
the single source of truth for both the Prolog and Python halves of the
theory with no risk of the two drifting apart.

Exposes FOUR predicates to ProbLog, all INSTANTANEOUS and stateless,
exactly like moveto_planners.py's plan_astar/plan_straight:

    first_threshold_crossing_time(+ControlPoints,+T0,+Duration,+Z,
                                   +Threshold, -Tcross, -ObstacleId)
    obstacle_within_threshold(+X,+Y,+Threshold)
    first_on_path_crossing_time(+ControlPoints,+T0,+Duration,+Z,
                                 +Threshold, -Tcross, -ObstacleId)
    obstacle_on_path_within_threshold(+ControlPoints,+T0,+Duration,+Z,
                                       +X,+Y,+Threshold)

first_threshold_crossing_time is the TRIGGER-side primitive: searches a
whole future trajectory (bracket scan + bisection) for the earliest
crossing. obstacle_within_threshold is the CONDITION-side primitive:
checks ONE point (the current situation) directly, no search at all --
it's what backs moveto_continuous.pl's holds(obstacle_in_bound(...),S),
the exact same underlying test (within_obstacle_threshold/
_min_clearance_all) the trigger-side search calls repeatedly, called
here just once. This is the "reuse the underlying machinery" the
obstacle_in_bound(Threshold) trigger/condition pair was built around --
see moveto_continuous.pl's own TRIGGERS section note.

first_on_path_crossing_time / obstacle_on_path_within_threshold are the
SAME trigger/condition pair again, for obstacle_on_path(Threshold):
"is Threshold-close to an obstacle the trajectory ACTUALLY ENTERS
somewhere along this walk" rather than any obstacle at all. Built by
restricting the SAME two functions above to a FILTERED obstacle set
(see _path_obstacle_polygons) -- no new geometry, just a different
input list.

ObstacleId is the SAME atom as the crossed obstacle's obstacle_polygon/2
Id (e.g. obs7) -- whichever polygon achieves the minimum clearance at
the exact (bisected) crossing point, i.e. an argmin over obstacles, not
just the min distance. This is what lets moveto_continuous.pl's
trigger_crossing_time/9 report crashed(ObstacleId)/
obstacle_in_bound(Threshold,ObstacleId) instead of a bare atom -- see
this module's own _min_clearance_all for where the argmin actually
happens.

first_threshold_crossing_time FAILS (returns 0 ProbLog solutions) if
the trajectory never comes within Threshold of an obstacle in this
resolved world; obstacle_within_threshold FAILS if the current point
isn't within Threshold right now -- "never/not happening" is
represented by absence in both, not a sentinel value, same convention
as every other exact-detection predicate in this theory.

Obstacle polygons: loaded ONCE, at import time, directly from
./environments/maps/obstacles_generated.pl -- the EXACT SAME file
moveto_continuous.pl's own :- consult(...) directive loads (see that
directive's own comment, a few lines above this module's use_module).
By the time this module is imported, that consult has already either
succeeded or already aborted the whole load with a clearer error, so
there is no meaningful "obstacles file missing" case to handle
gracefully here (unlike moveto_planners.py's map, which is genuinely
optional). If you hand-add extra obstacle_polygon/2 facts somewhere
else in the theory instead of through occgrid_to_problog.py's generated
file, this black box will not see them -- keep obstacles_generated.pl
the single source of truth, exactly as moveto_planners.py's own map
loading already assumes for map.yaml.

Everything below the constant/obstacle loading is PLAIN PYTHON, with no
ProbLog types anywhere -- first_threshold_crossing_time_value is the
testable core; the ProbLog import itself is wrapped in a try/except
(_HAVE_PROBLOG), same pattern as actions/moveto_planners.py, purely so
this module can be exercised directly (e.g. to numerically compare
against the old Prolog implementation) without needing a full ProbLog
run.
"""
import math
import os
import re
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_OBSTACLES_PATH = os.path.join(_THIS_DIR, "environments", "maps", "obstacles_generated.pl")

_CONFIG_DIR = os.path.join(_THIS_DIR, "config")
if _CONFIG_DIR not in sys.path:
    sys.path.insert(0, _CONFIG_DIR)
from generate_prolog_config import load_config  # noqa: E402

POINT_RE = re.compile(r"point\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)")


def _strip_prolog_comments(text):
    """Remove '%'-to-end-of-line comments before regex-parsing obstacle
    facts -- same defensive technique run_plan_continuous_safety.py
    already uses for the same reason (a header comment showing example
    syntax could otherwise match before the real fact)."""
    return re.sub(r"%.*$", "", text, flags=re.MULTILINE)


def _parse_obstacle_polygons(text):
    """Returns [(id, [(x,y),...]), ...] -- id is kept (not just the
    point list) so a crossing can be reported AGAINST a specific
    obstacle, not just "some obstacle" -- see _min_clearance_all."""
    polys = []
    for m in re.finditer(r"obstacle_polygon\(([^,]+),\s*\[(.*?)\]\s*\)\s*\.", text, re.S):
        obstacle_id = m.group(1).strip()
        pts = [(float(x), float(y)) for x, y in POINT_RE.findall(m.group(2))]
        if len(pts) >= 3:
            polys.append((obstacle_id, pts))
    return polys


_config = load_config()
BRACKET_SAMPLES = int(_config["verification"]["bracket_samples"])
CROSSING_EPS = float(_config["verification"]["crossing_eps"])
SIGMA = float(_config["noise"]["position"]["sigma"])

try:
    with open(_OBSTACLES_PATH) as f:
        OBSTACLE_POLYGONS = _parse_obstacle_polygons(_strip_prolog_comments(f.read()))
except FileNotFoundError:
    # Only reachable if this module is imported standalone (e.g. for
    # testing) before any map has been generated -- moveto_continuous.pl
    # itself would already have aborted loading earlier at its own
    # :- consult(...) of this same file. "No obstacles" is the correct
    # degrade here, matching obstacle_polygon/2's own Prolog fallback
    # clause (see moveto_continuous.pl section 0).
    OBSTACLE_POLYGONS = []


# =====================================================================
# SPLINE + NOISE -- line-for-line port of moveto_continuous.pl's
# bezier_point/spline_point/spline_tangent/perp_unit/walk_noisy_point.
# =====================================================================
def _bezier_point(p0, p1, p2, p3, u):
    mu = 1.0 - u
    x = mu**3*p0[0] + 3*mu*mu*u*p1[0] + 3*mu*u*u*p2[0] + u**3*p3[0]
    y = mu**3*p0[1] + 3*mu*mu*u*p1[1] + 3*mu*u*u*p2[1] + u**3*p3[1]
    return x, y


def _bezier_tangent(p0, p1, p2, p3, u):
    mu = 1.0 - u
    dx = 3*mu*mu*(p1[0]-p0[0]) + 6*mu*u*(p2[0]-p1[0]) + 3*u*u*(p3[0]-p2[0])
    dy = 3*mu*mu*(p1[1]-p0[1]) + 6*mu*u*(p2[1]-p1[1]) + 3*u*u*(p3[1]-p2[1])
    return dx, dy


def _spline_segment(control_points, u):
    n_segs = (len(control_points) - 1) // 3
    seg_len = 1.0 / n_segs
    seg_idx = min(n_segs - 1, int(math.floor(u / seg_len)))
    local_u = (u - seg_idx*seg_len) / seg_len
    local_u = max(0.0, min(1.0, local_u))
    p0, p1, p2, p3 = control_points[3*seg_idx:3*seg_idx+4]
    return p0, p1, p2, p3, local_u


def _spline_point(control_points, u):
    p0, p1, p2, p3, local_u = _spline_segment(control_points, u)
    return _bezier_point(p0, p1, p2, p3, local_u)


def _spline_tangent(control_points, u):
    p0, p1, p2, p3, local_u = _spline_segment(control_points, u)
    return _bezier_tangent(p0, p1, p2, p3, local_u)


def _perp_unit(norm, dx, dy):
    if norm <= 1.0e-9:
        return 0.0, 0.0
    return -dy/norm, dx/norm


def _walk_noisy_point(control_points, t0, duration, z, t):
    elapsed0 = t - t0
    elapsed = max(0.0, min(elapsed0, duration))
    frac = elapsed / duration
    nx, ny = _spline_point(control_points, frac)
    dx, dy = _spline_tangent(control_points, frac)
    norm = math.sqrt(dx*dx + dy*dy)
    perp_x, perp_y = _perp_unit(norm, dx, dy)
    deviation = z * SIGMA * math.sqrt(duration) * frac
    return nx + deviation*perp_x, ny + deviation*perp_y


# =====================================================================
# OBSTACLE-CLEARANCE GEOMETRY -- line-for-line port of
# point_segment_dist/polygon_edges/dist_to_polygon/inside_polygon/
# signed_clearance/min_clearance_all/within_obstacle_threshold.
# =====================================================================
def _dist(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)


def _point_segment_dist(px, py, ax, ay, bx, by):
    sdx, sdy = bx-ax, by-ay
    len2 = sdx*sdx + sdy*sdy
    if len2 <= 1.0e-9:
        return _dist(px, py, ax, ay)
    t0 = ((px-ax)*sdx + (py-ay)*sdy) / len2
    t = max(0.0, min(1.0, t0))
    cx, cy = ax + t*sdx, ay + t*sdy
    return _dist(px, py, cx, cy)


def _polygon_edges(points):
    closed = list(points) + [points[0]]
    return list(zip(closed[:-1], closed[1:]))


def _dist_to_polygon(px, py, points):
    return min(_point_segment_dist(px, py, ax, ay, bx, by)
               for (ax, ay), (bx, by) in _polygon_edges(points))


def _edge_crosses(px, py, ax, ay, bx, by):
    if (ay > py and by <= py) or (by > py and ay <= py):
        x_cross = ax + (py-ay)/(by-ay)*(bx-ax)
        return px < x_cross
    return False


def _inside_polygon(px, py, points):
    count = sum(1 for (ax, ay), (bx, by) in _polygon_edges(points)
                if _edge_crosses(px, py, ax, ay, bx, by))
    return count % 2 == 1


def _signed_clearance(px, py, points):
    d_edge = _dist_to_polygon(px, py, points)
    return -d_edge if _inside_polygon(px, py, points) else d_edge


def _min_clearance_all(px, py, obstacle_polygons):
    """obstacle_polygons: [(id, points), ...]. Returns (min_distance,
    nearest_obstacle_id) -- an ARGMIN over obstacles, not just the min
    distance, so callers can report WHICH obstacle a crossing is
    against, not just that one exists. nearest_obstacle_id is None only
    when there are no obstacles at all (min_distance is then the
    original "so far away it never matters" sentinel, unreachable by
    any real threshold)."""
    if not obstacle_polygons:
        return 1000000.0, None
    return min((_signed_clearance(px, py, poly), obstacle_id)
               for obstacle_id, poly in obstacle_polygons)


def _within_obstacle_threshold(px, py, threshold, obstacle_polygons):
    d, _ = _min_clearance_all(px, py, obstacle_polygons)
    return d <= threshold


# =====================================================================
# BRACKET SCAN + BISECTION -- line-for-line port of
# first_unsafe_sample(_from)/bisect_crossing/first_threshold_crossing_time.
# =====================================================================
def _first_unsafe_sample(control_points, t0, duration, z, threshold, obstacle_polygons, n):
    for i in range(0, n + 1):
        frac = i / n
        t = t0 + duration*frac
        x, y = _walk_noisy_point(control_points, t0, duration, z, t)
        if _within_obstacle_threshold(x, y, threshold, obstacle_polygons):
            return i
    return None


def _bisect_crossing(control_points, t0, duration, z, threshold, tlo, thi, eps, obstacle_polygons):
    while thi - tlo > eps:
        tmid = (tlo + thi) / 2.0
        x, y = _walk_noisy_point(control_points, t0, duration, z, tmid)
        if _within_obstacle_threshold(x, y, threshold, obstacle_polygons):
            thi = tmid
        else:
            tlo = tmid
    return (tlo + thi) / 2.0


def _first_threshold_crossing_time(control_points, t0, duration, z, threshold, obstacle_polygons):
    """Returns (Tcross, ObstacleId) or None. ObstacleId is resolved by
    ONE extra _min_clearance_all argmin call at the final crossing
    point -- the bracket scan/bisection loop itself only needs the
    boolean within/not-within-threshold test, so this stays a single
    added lookup, not a change to the search itself."""
    n = BRACKET_SAMPLES
    i = _first_unsafe_sample(control_points, t0, duration, z, threshold, obstacle_polygons, n)
    if i is None:
        return None
    if i == 0:
        tcross = t0
    else:
        tlo = t0 + duration*((i-1)/n)
        thi = t0 + duration*(i/n)
        tcross = _bisect_crossing(control_points, t0, duration, z, threshold, tlo, thi,
                                   CROSSING_EPS, obstacle_polygons)
    x, y = _walk_noisy_point(control_points, t0, duration, z, tcross)
    _, obstacle_id = _min_clearance_all(x, y, obstacle_polygons)
    return tcross, obstacle_id


# =====================================================================
# "ON PATH" -- obstacle_on_path(Threshold): unlike obstacle_in_bound
# (proximity to ANY obstacle, whether or not the trajectory actually
# goes near it), this only cares about obstacles the trajectory itself
# actually enters (_inside_polygon, not just close to the boundary) at
# SOME point across the walk's own full span -- then asks the SAME
# "within Threshold right now" question, restricted to just that
# obstacle set. Reuses _within_obstacle_threshold/_first_threshold_
# crossing_time UNCHANGED: the only new piece is the bracket-scan that
# finds WHICH obstacles are on the path at all; everything downstream
# is the existing machinery called with a FILTERED obstacle_polygons
# list instead of the full one.
# =====================================================================
def _trajectory_obstacle_ids(control_points, t0, duration, z, obstacle_polygons):
    """Which obstacle ids does this resolved trajectory's noisy position
    actually go INSIDE (not just near) at any bracket-sampled instant
    across [t0, t0+duration]? Same sample count as the rest of this
    module's bracket scans (BRACKET_SAMPLES) -- same discretization,
    same aliasing risk as every other bracket-scan check here, not a
    new limitation. Returns a (possibly empty) set of ids."""
    n = BRACKET_SAMPLES
    hit_ids = set()
    for i in range(n + 1):
        frac = i / n
        t = t0 + duration*frac
        x, y = _walk_noisy_point(control_points, t0, duration, z, t)
        for obstacle_id, poly in obstacle_polygons:
            if _inside_polygon(x, y, poly):
                hit_ids.add(obstacle_id)
    return hit_ids


def _path_obstacle_polygons(control_points, t0, duration, z, obstacle_polygons):
    """obstacle_polygons FILTERED down to just the ones the trajectory
    actually enters -- the single restriction point _first_threshold_
    crossing_time/_within_obstacle_threshold are then called with,
    unchanged, everywhere below."""
    path_ids = _trajectory_obstacle_ids(control_points, t0, duration, z, obstacle_polygons)
    return [(oid, poly) for oid, poly in obstacle_polygons if oid in path_ids]


def _first_on_path_crossing_time(control_points, t0, duration, z, threshold, obstacle_polygons):
    """Same shape/return as _first_threshold_crossing_time (Tcross,
    ObstacleId) or None -- None immediately if the trajectory never
    enters any obstacle at all (nothing to restrict to), otherwise
    delegates straight to the existing search over the restricted set."""
    restricted = _path_obstacle_polygons(control_points, t0, duration, z, obstacle_polygons)
    if not restricted:
        return None
    return _first_threshold_crossing_time(control_points, t0, duration, z, threshold, restricted)


def _obstacle_on_path_within_threshold(control_points, t0, duration, z, px, py, threshold, obstacle_polygons):
    """The CONDITION-side counterpart: is (px,py) within threshold of
    an obstacle the trajectory actually enters, RIGHT NOW -- one call,
    no search, exactly mirroring _within_obstacle_threshold's own
    relationship to _first_threshold_crossing_time."""
    restricted = _path_obstacle_polygons(control_points, t0, duration, z, obstacle_polygons)
    if not restricted:
        return False
    return _within_obstacle_threshold(px, py, threshold, restricted)


# =====================================================================
# PLAIN-PYTHON API -- ALWAYS available, testable without ProbLog.
# =====================================================================
def first_threshold_crossing_time_value(control_points, t0, duration, z, threshold):
    """control_points: [(x,y), ...]. Returns (Tcross, ObstacleId), or
    None if the trajectory never comes within `threshold` of any
    obstacle in this resolved world. ObstacleId is the obstacle_polygon/2
    Id (e.g. "obs7") nearest at the exact crossing point."""
    return _first_threshold_crossing_time(
        control_points, float(t0), float(duration), float(z), float(threshold),
        OBSTACLE_POLYGONS)


def obstacle_within_threshold_value(x, y, threshold):
    """A SINGLE-POINT check -- is (x,y) within `threshold` of any
    obstacle right now -- reusing _within_obstacle_threshold directly
    (the SAME primitive _first_unsafe_sample/_bisect_crossing call
    repeatedly above; this just calls it once). This is what backs the
    obstacle_in_bound(Threshold) CONDITION in moveto_continuous.pl
    (holds(obstacle_in_bound(...),S)), as opposed to the
    obstacle_in_bound(Threshold) TRIGGER, which searches a whole future
    trajectory via first_threshold_crossing_time_value above."""
    return _within_obstacle_threshold(float(x), float(y), float(threshold), OBSTACLE_POLYGONS)


def first_on_path_crossing_time_value(control_points, t0, duration, z, threshold):
    """control_points: [(x,y), ...]. Returns (Tcross, ObstacleId), or
    None if the trajectory never comes within `threshold` of an
    obstacle IT ACTUALLY ENTERS somewhere along this walk (as opposed
    to first_threshold_crossing_time_value, which considers every
    obstacle regardless of whether the path goes near it at all)."""
    return _first_on_path_crossing_time(
        control_points, float(t0), float(duration), float(z), float(threshold),
        OBSTACLE_POLYGONS)


def obstacle_on_path_within_threshold_value(control_points, t0, duration, z, x, y, threshold):
    """A SINGLE-POINT check, same relationship to
    first_on_path_crossing_time_value as obstacle_within_threshold_value
    has to first_threshold_crossing_time_value: is (x,y) within
    threshold of an obstacle the trajectory actually enters, right
    now."""
    return _obstacle_on_path_within_threshold(
        control_points, float(t0), float(duration), float(z),
        float(x), float(y), float(threshold), OBSTACLE_POLYGONS)


# =====================================================================
# PROBLOG-FACING PREDICATE -- only registered if ProbLog is importable
# (see actions/moveto_planners.py's header for why this is guarded
# rather than a hard import).
# =====================================================================
try:
    from problog.extern import problog_export_nondet
    _HAVE_PROBLOG = True
except ImportError:
    _HAVE_PROBLOG = False

if _HAVE_PROBLOG:

    # T0 in particular can arrive as a bare Prolog INTEGER (now(s0,0) is
    # the very first walk's T0) rather than a float -- "+term" accepts
    # either representation; first_threshold_crossing_time_value below
    # does the float() conversion itself. Duration/Z/Threshold are
    # always the result of `is` arithmetic or float literals in
    # practice, but are taken as "+term" too for the same robustness.
    @problog_export_nondet("+list", "+term", "+term", "+term", "+term", "-float", "-str")
    def first_threshold_crossing_time(control_points, t0, duration, z, threshold):
        cp = [(float(p.args[0]), float(p.args[1])) for p in control_points]
        result = first_threshold_crossing_time_value(
            cp, float(t0), float(duration), float(z), float(threshold))
        if result is None:
            return []  # no crossing in this world -- predicate FAILS
        tcross, obstacle_id = result
        return [(tcross, obstacle_id)]

    # obstacle_within_threshold(+X,+Y,+Threshold): a pure boolean check,
    # no output ports at all -- backs holds(obstacle_in_bound(...),S) in
    # moveto_continuous.pl. A ProbLog nondet predicate with ZERO "-"
    # specs succeeds (one solution) by returning [()] (a single empty
    # tuple -- "here is one solution binding zero extra outputs") or
    # fails (no solutions) by returning [].
    @problog_export_nondet("+term", "+term", "+term")
    def obstacle_within_threshold(x, y, threshold):
        if obstacle_within_threshold_value(x, y, threshold):
            return [()]
        return []

    # first_on_path_crossing_time(+ControlPoints,+T0,+Duration,+Z,
    # +Threshold,-Tcross,-ObstacleId): backs the obstacle_on_path(Threshold)
    # TRIGGER. Same shape/signature as first_threshold_crossing_time
    # above, just restricted (inside collision_geometry.py itself) to
    # obstacles the trajectory actually enters.
    @problog_export_nondet("+list", "+term", "+term", "+term", "+term", "-float", "-str")
    def first_on_path_crossing_time(control_points, t0, duration, z, threshold):
        cp = [(float(p.args[0]), float(p.args[1])) for p in control_points]
        result = first_on_path_crossing_time_value(
            cp, float(t0), float(duration), float(z), float(threshold))
        if result is None:
            return []
        tcross, obstacle_id = result
        return [(tcross, obstacle_id)]

    # obstacle_on_path_within_threshold(+ControlPoints,+T0,+Duration,+Z,
    # +X,+Y,+Threshold): backs the obstacle_on_path(Threshold)
    # CONDITION. Same zero-output boolean shape as
    # obstacle_within_threshold above.
    @problog_export_nondet("+list", "+term", "+term", "+term", "+term", "+term", "+term")
    def obstacle_on_path_within_threshold(control_points, t0, duration, z, x, y, threshold):
        cp = [(float(p.args[0]), float(p.args[1])) for p in control_points]
        if obstacle_on_path_within_threshold_value(cp, t0, duration, z, x, y, threshold):
            return [()]
        return []
