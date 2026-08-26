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
comes within a given distance of any obstacle -- the basis for both the
`collision` and `obstacle_sighted` triggers) entirely in Prolog: a
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
not a re-derivation. bracket_samples/1, crossing_eps/1, and sigma/1 are
read directly out of moveto_continuous.pl's own text at import time (see
_read_theory_constants below) rather than hardcoded a second time here,
so tuning either value in the .pl file takes effect automatically with
no risk of the Prolog and Python sides drifting apart.

Exposes ONE predicate to ProbLog, INSTANTANEOUS and stateless, exactly
like moveto_planners.py's plan_astar/plan_straight:

    first_threshold_crossing_time(+ControlPoints,+T0,+Duration,+Z,
                                   +Threshold, -Tcross)

FAILS (returns 0 ProbLog solutions) if the trajectory never comes within
Threshold of an obstacle in this resolved world -- "never happens" is
represented by absence, not a sentinel value, same convention as every
other exact-detection predicate in this theory.

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

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_THEORY_PATH = os.path.join(_THIS_DIR, "moveto_continuous.pl")
_OBSTACLES_PATH = os.path.join(_THIS_DIR, "environments", "maps", "obstacles_generated.pl")

POINT_RE = re.compile(r"point\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)")


def _strip_prolog_comments(text):
    """Remove '%'-to-end-of-line comments before regex-parsing facts --
    same defensive technique run_plan_continuous_safety.py already uses
    for the same reason (a header comment showing example syntax could
    otherwise match before the real fact)."""
    return re.sub(r"%.*$", "", text, flags=re.MULTILINE)


def _parse_scalar_fact(text, name, cast):
    m = re.search(rf"^{name}\(\s*(-?\d+(?:\.\d+)?)\s*\)\s*\.", text, re.MULTILINE)
    if not m:
        raise ValueError(f"Could not find {name}/1 fact in {_THEORY_PATH}")
    return cast(m.group(1))


def _parse_obstacle_polygons(text):
    polys = []
    for m in re.finditer(r"obstacle_polygon\([^,]+,\s*\[(.*?)\]\s*\)\s*\.", text, re.S):
        pts = [(float(x), float(y)) for x, y in POINT_RE.findall(m.group(1))]
        if len(pts) >= 3:
            polys.append(pts)
    return polys


def _read_theory_constants():
    with open(_THEORY_PATH) as f:
        text = _strip_prolog_comments(f.read())
    return (
        _parse_scalar_fact(text, "bracket_samples", int),
        _parse_scalar_fact(text, "crossing_eps", float),
        _parse_scalar_fact(text, "sigma", float),
    )


BRACKET_SAMPLES, CROSSING_EPS, SIGMA = _read_theory_constants()

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
    if not obstacle_polygons:
        return 1000000.0
    return min(_signed_clearance(px, py, poly) for poly in obstacle_polygons)


def _within_obstacle_threshold(px, py, threshold, obstacle_polygons):
    return _min_clearance_all(px, py, obstacle_polygons) <= threshold


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
    n = BRACKET_SAMPLES
    i = _first_unsafe_sample(control_points, t0, duration, z, threshold, obstacle_polygons, n)
    if i is None:
        return None
    if i == 0:
        return t0
    tlo = t0 + duration*((i-1)/n)
    thi = t0 + duration*(i/n)
    return _bisect_crossing(control_points, t0, duration, z, threshold, tlo, thi,
                             CROSSING_EPS, obstacle_polygons)


# =====================================================================
# PLAIN-PYTHON API -- ALWAYS available, testable without ProbLog.
# =====================================================================
def first_threshold_crossing_time_value(control_points, t0, duration, z, threshold):
    """control_points: [(x,y), ...]. Returns the crossing time (float),
    or None if the trajectory never comes within `threshold` of any
    obstacle in this resolved world."""
    return _first_threshold_crossing_time(
        control_points, float(t0), float(duration), float(z), float(threshold),
        OBSTACLE_POLYGONS)


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
    @problog_export_nondet("+list", "+term", "+term", "+term", "+term", "-float")
    def first_threshold_crossing_time(control_points, t0, duration, z, threshold):
        cp = [(float(p.args[0]), float(p.args[1])) for p in control_points]
        tcross = first_threshold_crossing_time_value(
            cp, float(t0), float(duration), float(z), float(threshold))
        if tcross is None:
            return []  # no crossing in this world -- predicate FAILS
        return [tcross]
