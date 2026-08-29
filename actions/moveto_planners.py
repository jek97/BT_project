#!/usr/bin/env python3
"""
moveto_planners.py

Lives in ./actions/ alongside bt_actions.py and schema.yaml -- see this
project's top-level layout note (moveto_continuous.pl's own header
comment) for why these three files were grouped together: they are the
three faces of the same node set (Prolog action theory, BT.cpp-facing
Python callables, BT.cpp node-model schema) and are meant to be read/kept
in sync together.

TWO independent roles, kept in ONE file because they share the exact
same underlying A*/spline computation and neither should reimplement it:

  1. PROBLOG EXTERNAL-PREDICATE MODULE -- imported by ProbLog's own
     :- use_module('./actions/moveto_planners.py'). directive inside
     moveto_continuous.pl (see problog.clausedb.ClauseDB.load_external_
     module, which execs this file as a plain Python module the moment
     that directive is loaded, triggering the
     @problog_export_nondet(...)-decorated functions below to register
     themselves as callable ProbLog predicates:
         plan_astar(+SX,+SY,+GX,+GY, -ControlPoints)
         plan_straight(+SX,+SY,+GX,+GY, -ControlPoints)
         follow_boarder0(+SX,+SY,+GX,+GY,+ObstacleId,+Offset, -ControlPoints)
         follow_boarder1(+SX,+SY,+GX,+GY,+ObstacleId,+Offset, -ControlPoints)
     ControlPoints = [point(X0,Y0), ...] ProbLog terms, length 3k+1, in
     EXACTLY the format moveto_continuous.pl's spline_point/4 expects.
     follow_boarder0/follow_boarder1 are Bug0-/Bug1-style boundary-
     following planners, sharing the SAME clockwise offset-boundary
     walk (_boundary_walk_setup) but with different stopping rules --
     see their own _follow_boarderN_control_points header comments.

  2. PLAIN-PYTHON API for a BT.cpp / bt_actions.py caller that has
     nothing to do with ProbLog -- plan_astar_points/plan_straight_points/
     follow_boarder0_points/follow_boarder1_points below, returning
     plain (x,y) float tuples, no ProbLog Term objects and no ProbLog
     import required to call them. The `import problog` needed for role
     1 is wrapped in a try/except (_HAVE_PROBLOG) so that importing this
     module from a pure BT.cpp bridge that never installs ProbLog still
     works -- only the ProbLog predicates themselves become unavailable
     in that case, exactly mirroring how plan_astar already degrades
     gracefully (fails rather than crashing) when the map itself fails
     to load.

Each role funnels through the SAME plain-Python core per planner
(_astar_control_points / _straight_control_points /
_follow_boarder0_control_points / _follow_boarder1_control_points) --
no duplicated logic between the ProbLog-facing and BT.cpp-facing entry
points for any of the four.

FAILURE: if A* finds no path (start/goal unreachable, or the map failed
to load at import time), the core returns None -- the ProbLog predicate
then returns zero solutions (i.e. the predicate call FAILS, exactly like
any other "this doesn't happen" case elsewhere in this theory, e.g.
first_collision_time), and the plain-Python function returns None for
its own BT.cpp caller to check. plan_straight/plan_straight_points only
fail on degenerate (non-finite) input; a straight line between two
ordinary points always exists.

Map location: loaded ONCE, at import time (i.e. once per process), from
../environments/maps/map.yaml relative to THIS FILE's own directory
(./actions/), i.e. environments/maps/map.yaml at the project root --
not the CWD. See MAP_YAML_PATH below. If it can't be loaded, both the
ProbLog predicate and the plain-Python function still work, but
astar-based planning will always fail (no map to plan against) rather
than crashing at import time.
"""
import math
import os
import re
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR)

# occupancy_grid_planner.py lives in ./plan_generation/ at the PROJECT
# ROOT (a sibling of ./actions/, not a child of it), per the established
# project layout (see the header comment above moveto_continuous.pl's
# own obstacle-file consult directive).
_PLAN_GENERATION_DIR = os.path.join(_PROJECT_ROOT, "plan_generation")
if _PLAN_GENERATION_DIR not in sys.path:
    sys.path.insert(0, _PLAN_GENERATION_DIR)

# Reuse the EXACT SAME map-loading / A* / spline pipeline
# occupancy_grid_planner.py already implements -- no duplication.
from occupancy_grid_planner import (
    OccupancyGridMap,
    load_map_yaml,
    inflate_obstacles,
    astar,
    fit_spline,
    bspline_to_bezier_chain,
)

MAP_YAML_PATH = os.path.join(_PROJECT_ROOT, "environments", "maps", "map.yaml")
_OBSTACLES_PATH = os.path.join(_PROJECT_ROOT, "environments", "maps", "obstacles_generated.pl")

# Same default inflation as occupancy_grid_planner.py's own --inflate default.
PLANNING_INFLATE_M = 0.5
OCC_THRESH = 50
CONNECTIVITY = 8

# -- load the map ONCE at import time (i.e. once per process) ------------
try:
    _RAW_MAP = load_map_yaml(MAP_YAML_PATH)
    _PLANNING_MAP = inflate_obstacles(_RAW_MAP, PLANNING_INFLATE_M) \
        if PLANNING_INFLATE_M > 0 else _RAW_MAP
    _MAP_LOAD_ERROR = None
except Exception as exc:  # noqa: BLE001 -- deliberately broad: ANY load
    # failure should degrade to "astar planning always fails", not crash
    # the caller (ProbLog's grounding process, or a BT.cpp bridge).
    _RAW_MAP = None
    _PLANNING_MAP = None
    _MAP_LOAD_ERROR = exc


# -- load obstacle polygons ONCE at import time, for follow_boarder0 -----
# Same source file, same tiny regex parser as collision_geometry.py's own
# OBSTACLE_POLYGONS -- DELIBERATELY duplicated rather than imported from
# there: both files are independently `:- use_module(...)`'d by ProbLog
# (problog.clausedb.load_external_module execs each one fresh via its own
# SourceFileLoader, with problog_export.database pointed at whichever
# ClauseDB is loading it at that moment), and having one black-box module
# `import` the other would run its @problog_export_nondet-decorated
# predicates a second time under an unverified context -- not a risk
# worth taking for ~15 lines of stable parsing code. Every other black
# box in this project already owns its own data loading independently
# (moveto_planners.py's own map.yaml load above, collision_geometry.py's
# obstacles_generated.pl load) -- this follows the same convention.
_POINT_RE = re.compile(r"point\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)")


def _strip_prolog_comments(text):
    return re.sub(r"%.*$", "", text, flags=re.MULTILINE)


def _parse_obstacle_polygons(text):
    """Returns [(id, [(x,y),...]), ...] -- see collision_geometry.py's
    own identical function for the full rationale."""
    polys = []
    for m in re.finditer(r"obstacle_polygon\(([^,]+),\s*\[(.*?)\]\s*\)\s*\.", text, re.S):
        obstacle_id = m.group(1).strip()
        pts = [(float(x), float(y)) for x, y in _POINT_RE.findall(m.group(2))]
        if len(pts) >= 3:
            polys.append((obstacle_id, pts))
    return polys


try:
    with open(_OBSTACLES_PATH) as f:
        _OBSTACLE_POLYGONS = _parse_obstacle_polygons(_strip_prolog_comments(f.read()))
except FileNotFoundError:
    _OBSTACLE_POLYGONS = []


# =====================================================================
# SHARED CORE -- plain Python, no ProbLog types anywhere here. Both the
# ProbLog-facing predicates and the BT.cpp-facing plain functions below
# call straight into this.
# =====================================================================
def _straight_control_points(sx, sy, gx, gy):
    """A single cubic Bezier segment, collinear control points -- reduces
    EXACTLY to a straight line (see bspline_to_bezier_chain's own note on
    this being the degenerate case of a Bezier chain)."""
    dx, dy = gx - sx, gy - sy
    p0 = (sx, sy)
    p1 = (sx + dx / 3.0, sy + dy / 3.0)
    p2 = (sx + 2.0 * dx / 3.0, sy + 2.0 * dy / 3.0)
    p3 = (gx, gy)
    return [p0, p1, p2, p3]


def _astar_control_points(sx, sy, gx, gy):
    """Black-box A* planner: world-frame (sx,sy) -> (gx,gy), via the map
    loaded at import time. Returns None if the map didn't load, if
    start/goal fall outside the map or on an obstacle cell, or if A*
    finds no path at all -- the two callers below turn that into
    whatever "no path" signal fits their own interface (a failed
    ProbLog predicate call, or a plain None)."""
    if _PLANNING_MAP is None:
        return None

    start_rc = _PLANNING_MAP.world_to_grid(sx, sy)
    goal_rc = _PLANNING_MAP.world_to_grid(gx, gy)

    if not _PLANNING_MAP.in_bounds(*start_rc) or not _PLANNING_MAP.in_bounds(*goal_rc):
        return None

    if start_rc == goal_rc:
        # Already there -- a zero-length, degenerate but VALID Bezier
        # (all four control points identical).
        return [(sx, sy)] * 4

    path_rc = astar(_PLANNING_MAP, start_rc, goal_rc,
                     occ_thresh=OCC_THRESH, connectivity=CONNECTIVITY,
                     unknown_is_occupied=True)
    if path_rc is None:
        return None  # no path found

    path_xy = [_PLANNING_MAP.grid_to_world(r, c) for r, c in path_rc]

    if len(path_xy) < 2:
        return [(sx, sy)] * 4

    try:
        tck, _u = fit_spline(path_xy, degree=3, smoothing=0.0)
        control_points, _k = bspline_to_bezier_chain(tck)
    except ValueError:
        # Degenerate/too-short path for a cubic fit -- fall back to a
        # straight line between the actual start/goal rather than failing
        # outright (A* DID find connectivity; only the spline fit failed).
        control_points = _straight_control_points(sx, sy, gx, gy)

    return control_points


# =====================================================================
# FOLLOW_BOARDER0 / FOLLOW_BOARDER1 -- Bug0-/Bug1-style boundary-
# following planners. From the CURRENT position, both trace CLOCKWISE
# around one named obstacle's own boundary, offset outward by a
# caller-given distance, and BOTH deliberately do NOT plan all the way
# to goal -- a subsequent PlanStraight+MoveTo leg is expected to cover
# the rest, per the Bug-algorithm "circle the obstacle, then go
# straight" shape. They differ ONLY in when they stop circling:
#   follow_boarder0 -- as soon as the obstacle no longer occludes a
#                       straight line to goal (Bug0's own rule).
#   follow_boarder1 -- when the boundary walk re-crosses the straight
#                       segment from this planner's own call-time
#                       position to goal (a simplified Bug1 rule -- see
#                       _follow_boarder1_control_points's own note).
# Both share the SAME offset-boundary construction (_offset_boundary_
# clockwise) and SAME fit_spline/bspline_to_bezier_chain fitting step
# _astar_control_points uses above, so their own output is a
# chained-cubic-Bezier control_points list in exactly the same format,
# no special-casing needed downstream.
# =====================================================================
def _polygon_edges(points):
    closed = list(points) + [points[0]]
    return list(zip(closed[:-1], closed[1:]))


def _edge_crosses(px, py, ax, ay, bx, by):
    if (ay > py and by <= py) or (by > py and ay <= py):
        x_cross = ax + (py-ay)/(by-ay)*(bx-ax)
        return px < x_cross
    return False


def _inside_polygon(px, py, points):
    count = sum(1 for (ax, ay), (bx, by) in _polygon_edges(points)
                if _edge_crosses(px, py, ax, ay, bx, by))
    return count % 2 == 1


def _orient(ax, ay, bx, by, cx, cy):
    return (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)


def _segments_intersect(p1, p2, p3, p4):
    """Standard orientation-based proper-crossing test -- collinear/
    touching edge cases are treated as NOT intersecting, an acceptable
    approximation for an occlusion test (see _obstacle_occludes) that
    is re-evaluated at every densely-sampled boundary point anyway."""
    ax, ay = p1
    bx, by = p2
    cx, cy = p3
    dx, dy = p4
    o1 = _orient(ax, ay, bx, by, cx, cy)
    o2 = _orient(ax, ay, bx, by, dx, dy)
    o3 = _orient(cx, cy, dx, dy, ax, ay)
    o4 = _orient(cx, cy, dx, dy, bx, by)
    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)


def _obstacle_occludes(px, py, gx, gy, polygon):
    """True iff the straight segment from (px,py) to the goal (gx,gy)
    crosses `polygon`'s own boundary -- i.e. the obstacle still blocks
    a direct line of sight to goal from here."""
    return any(_segments_intersect((px, py), (gx, gy), a, b)
               for a, b in _polygon_edges(polygon))


def _signed_polygon_area(points):
    return sum(ax*by - bx*ay for (ax, ay), (bx, by) in _polygon_edges(points)) / 2.0


# Geometry constants for the boundary walk -- planner-internal tuning,
# not routed through config.yaml, matching this file's own existing
# precedent (PLANNING_INFLATE_M/OCC_THRESH/CONNECTIVITY above are
# plain module constants too, not config.yaml-sourced -- these are
# planner-geometry knobs, not action-theory tunables that flow into
# ProbLog's own noise/trigger model).
_BOUNDARY_SAMPLES_PER_EDGE = 8
_NORMAL_PROBE_EPS = 1.0e-3


def _offset_boundary_clockwise(polygon, offset):
    """Dense samples along `polygon`'s own boundary, each pushed
    outward by `offset` along its own edge's outward normal (a
    per-edge offset, not a true mitred polygon offset -- a reasonable
    approximation at this project's abstraction level, given the
    result gets spline-fit afterward anyway, same spirit as A*'s own
    grid-path-then-fit-spline pipeline above), ordered so that walking
    the returned list IN ORDER traces the boundary CLOCKWISE in world
    (x right, y up) coordinates -- regardless of polygon's own vertex
    winding order (obstacle_polygon/2 facts make no winding
    guarantee). The outward direction for each edge is picked
    empirically (whichever of the two perpendiculars, probed a small
    epsilon out from the edge midpoint, lands OUTSIDE the polygon) --
    robust to either winding and to mild non-convexity, unlike relying
    on a fixed sign convention."""
    vertices = list(polygon)
    if _signed_polygon_area(vertices) > 0.0:
        # Positive signed area = vertices given counterclockwise (in a
        # y-up frame); reverse to walk them clockwise instead.
        vertices = list(reversed(vertices))

    samples = []
    for (ax, ay), (bx, by) in _polygon_edges(vertices):
        edx, edy = bx-ax, by-ay
        elen = math.hypot(edx, edy)
        if elen <= 1.0e-9:
            continue
        edx, edy = edx/elen, edy/elen
        n1 = (-edy, edx)
        n2 = (edy, -edx)
        mx, my = (ax+bx)/2.0, (ay+by)/2.0
        probe_x, probe_y = mx + n1[0]*_NORMAL_PROBE_EPS, my + n1[1]*_NORMAL_PROBE_EPS
        outward = n1 if not _inside_polygon(probe_x, probe_y, vertices) else n2
        for k in range(_BOUNDARY_SAMPLES_PER_EDGE):
            frac = k / _BOUNDARY_SAMPLES_PER_EDGE
            px, py = ax + edx*elen*frac, ay + edy*elen*frac
            samples.append((px + outward[0]*offset, py + outward[1]*offset))
    return samples


def _boundary_walk_setup(sx, sy, obstacle_id, offset):
    """Shared setup for every follow_boarderN planner: look up
    obstacle_id's own polygon, build its offset boundary, and find
    which sample is closest to the robot's ACTUAL current position --
    NOT assumed to already sit exactly on the offset curve (it
    generally won't, by a small bisection-tolerance residual -- see
    this project's own discussion of first_on_path_crossing_time's
    CROSSING_EPS). The small gap this leaves, if any, is absorbed into
    each planner's own first fitted spline segment, with no separate
    "join" step needed. Returns (polygon, boundary, start_idx), or
    None if obstacle_id names no known obstacle or its polygon is
    degenerate."""
    polygon = None
    for oid, pts in _OBSTACLE_POLYGONS:
        if oid == obstacle_id:
            polygon = pts
            break
    if polygon is None or len(polygon) < 3:
        return None

    boundary = _offset_boundary_clockwise(polygon, offset)
    if not boundary:
        return None

    start_idx = min(range(len(boundary)),
                     key=lambda i: (boundary[i][0]-sx)**2 + (boundary[i][1]-sy)**2)
    return polygon, boundary, start_idx


def _fit_boundary_path(sx, sy, path_xy):
    """Shared tail for every follow_boarderN planner: fit the walked
    polyline (starting at the robot's actual (sx,sy)) to a chained
    cubic Bezier, same fit_spline/bspline_to_bezier_chain pipeline
    _astar_control_points uses above -- falling back to a straight
    line from the actual start to the path's own last point if the
    path is too short/degenerate for a cubic fit (e.g. the closest
    boundary sample already satisfies the stopping test)."""
    try:
        tck, _u = fit_spline(path_xy, degree=3, smoothing=0.0)
        control_points, _k = bspline_to_bezier_chain(tck)
    except ValueError:
        control_points = _straight_control_points(sx, sy, path_xy[-1][0], path_xy[-1][1])
    return control_points


def _follow_boarder0_control_points(sx, sy, gx, gy, obstacle_id, offset):
    """Core computation -- see follow_boarder0_points below for the
    full contract. Returns [(x,y), ...] control points, or None if
    obstacle_id names no known obstacle, its polygon is degenerate, or
    a full circuit of the offset boundary never clears line-of-sight
    to (gx,gy)."""
    setup = _boundary_walk_setup(sx, sy, obstacle_id, offset)
    if setup is None:
        return None
    polygon, boundary, start_idx = setup

    n = len(boundary)
    path_xy = [(sx, sy)]
    cleared = False
    for step in range(n):
        idx = (start_idx + step) % n
        px, py = boundary[idx]
        path_xy.append((px, py))
        if not _obstacle_occludes(px, py, gx, gy, polygon):
            cleared = True
            break
    if not cleared:
        return None  # one full circuit, never cleared line-of-sight -- no_path

    return _fit_boundary_path(sx, sy, path_xy)


def _segment_intersection_point(p1, p2, p3, p4):
    """The EXACT (x,y) point where segments (p1,p2) and (p3,p4)
    properly cross, or None if they don't (reuses _segments_intersect
    for the boolean gate, then solves the standard parametric
    line-intersection formula along (p1,p2) for the precise point --
    used by follow_boarder1 to stop exactly where the boundary walk
    re-crosses the original start-goal line, not just at the nearest
    discrete boundary sample)."""
    if not _segments_intersect(p1, p2, p3, p4):
        return None
    (x1, y1), (x2, y2) = p1, p2
    (x3, y3), (x4, y4) = p3, p4
    denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if abs(denom) <= 1.0e-12:
        return None  # parallel/collinear -- no single clean crossing point
    t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / denom
    return (x1 + t*(x2-x1), y1 + t*(y2-y1))


def _follow_boarder1_control_points(sx, sy, gx, gy, obstacle_id, offset):
    """Core computation -- see follow_boarder1_points below for the
    full contract. Bug1-style stopping rule: unlike follow_boarder0
    (stops as soon as the obstacle stops occluding goal),
    follow_boarder1 keeps circling until the boundary walk itself
    RE-CROSSES the straight segment from the planner's own call-time
    position (sx,sy) to the goal (gx,gy) -- the direct path the robot
    was following before the obstacle forced it to divert. Returns
    [(x,y), ...] control points, or None if obstacle_id names no known
    obstacle, its polygon is degenerate, or a full circuit never
    re-crosses that segment."""
    setup = _boundary_walk_setup(sx, sy, obstacle_id, offset)
    if setup is None:
        return None
    _polygon, boundary, start_idx = setup

    n = len(boundary)
    path_xy = [(sx, sy), boundary[start_idx]]
    start_goal = ((sx, sy), (gx, gy))
    crossed = False
    # The entry edge (sx,sy)->boundary[start_idx] is deliberately NOT
    # tested here -- (sx,sy) is itself an ENDPOINT of the start-goal
    # segment, so that edge trivially touches it at a degenerate
    # (zero-length-along-the-line) point; checking from the first real
    # boundary-to-boundary edge onward avoids a spurious immediate
    # "crossing" right where the walk begins.
    for step in range(1, n):
        idx_prev = (start_idx + step - 1) % n
        idx_curr = (start_idx + step) % n
        p_prev, p_curr = boundary[idx_prev], boundary[idx_curr]
        cross_pt = _segment_intersection_point(start_goal[0], start_goal[1], p_prev, p_curr)
        if cross_pt is not None:
            path_xy.append(cross_pt)
            crossed = True
            break
        path_xy.append(p_curr)
    if not crossed:
        return None  # one full circuit, never re-crossed the start-goal segment -- no_path

    return _fit_boundary_path(sx, sy, path_xy)


# =====================================================================
# PLAIN-PYTHON API -- ALWAYS available, no ProbLog dependency. This is
# what bt_actions.py (and, eventually, a BT.cpp/pybind11 bridge) calls.
# =====================================================================
def plan_astar_points(sx, sy, gx, gy):
    """Plain-Python A* planner. Returns [(x,y), ...] control points
    (length 3k+1), or None if no path exists / the map isn't loaded."""
    return _astar_control_points(float(sx), float(sy), float(gx), float(gy))


def plan_straight_points(sx, sy, gx, gy):
    """Plain-Python straight-line planner. Always returns 4 control
    points for any finite (sx,sy),(gx,gy)."""
    return _straight_control_points(float(sx), float(sy), float(gx), float(gy))


def follow_boarder0_points(sx, sy, gx, gy, obstacle_id, offset):
    """Plain-Python Bug0-style boundary-following planner. Returns
    [(x,y), ...] control points (length 3k+1) tracing CLOCKWISE around
    `obstacle_id`'s own boundary, offset outward by `offset`, starting
    from the CURRENT position (sx,sy) and stopping at the first
    boundary point from which `obstacle_id` no longer occludes a
    straight line to (gx,gy) -- deliberately NOT all the way to the
    goal itself; a subsequent PlanStraight+MoveTo leg is expected to
    cover the rest, per the Bug0 "circle until clear, then go
    straight" pattern. `offset` is typically unified with the SAME
    Threshold as whichever obstacle_on_path(Threshold)/obstacle_in_bound
    (Threshold) trigger or condition supplied `obstacle_id`, so the
    boundary-following path stays exactly as far out as the condition
    that triggered this replan -- see this project's own discussion on
    why a small residual gap between the robot's actual position and
    that nominal offset is harmless (absorbed into the first fitted
    spline segment, not a source of error that compounds).
    Returns None if obstacle_id names no known obstacle, or if a full
    circuit of its offset boundary never clears line-of-sight to
    (gx,gy)."""
    return _follow_boarder0_control_points(
        float(sx), float(sy), float(gx), float(gy), str(obstacle_id), float(offset))


def follow_boarder1_points(sx, sy, gx, gy, obstacle_id, offset):
    """Plain-Python Bug1-style boundary-following planner. Same
    signature/return shape as follow_boarder0_points above, and the
    SAME clockwise-offset-boundary walk, but a DIFFERENT stopping
    rule: keeps circling until the boundary walk itself re-crosses the
    straight segment from (sx,sy) (this planner's own call-time
    position) to (gx,gy) -- the direct path the robot was on before
    the obstacle forced this diversion -- rather than stopping as soon
    as line-of-sight to goal clears. Deliberately NOT all the way to
    the goal itself, same "a subsequent PlanStraight+MoveTo leg covers
    the rest" pattern as follow_boarder0_points.
    Returns None if obstacle_id names no known obstacle, or if a full
    circuit of its offset boundary never re-crosses the (sx,sy)-(gx,gy)
    segment."""
    return _follow_boarder1_control_points(
        float(sx), float(sy), float(gx), float(gy), str(obstacle_id), float(offset))


# =====================================================================
# PROBLOG-FACING PREDICATES -- only registered if ProbLog is actually
# importable. moveto_continuous.pl's :- use_module(...) directive always
# has ProbLog present (ProbLog itself is doing the loading), so this is
# never a problem for the ProbLog side; it only matters for a pure
# BT.cpp/Python caller that imports this module directly without ever
# installing ProbLog -- that caller only ever wants plan_astar_points/
# plan_straight_points above, and importing this module must not fail
# just because `problog` itself isn't installed in that environment.
# =====================================================================
try:
    from problog.extern import problog_export_nondet
    from problog.logic import Term, Constant
    _HAVE_PROBLOG = True
except ImportError:
    _HAVE_PROBLOG = False

if _HAVE_PROBLOG:

    def _control_points_to_terms(control_points):
        """[(x,y), ...] -> [point(x,y), ...] as ProbLog Term objects."""
        return [Term("point", Constant(float(x)), Constant(float(y)))
                for x, y in control_points]

    @problog_export_nondet("+float", "+float", "+float", "+float", "-list")
    def plan_astar(sx, sy, gx, gy):
        """Black-box A* planner (ProbLog predicate) -- see
        _astar_control_points above for the actual computation. Fails
        (returns []) if the map didn't load, if start/goal fall outside
        the map or on an obstacle cell, or if A* finds no path at all."""
        control_points = _astar_control_points(sx, sy, gx, gy)
        if control_points is None:
            return []  # no path found -- predicate FAILS, not an exception
        return [_control_points_to_terms(control_points)]

    @problog_export_nondet("+float", "+float", "+float", "+float", "-list")
    def plan_straight(sx, sy, gx, gy):
        """Black-box straight-line planner (ProbLog predicate) -- no map
        lookup at all, always succeeds for any finite (sx,sy),(gx,gy)."""
        control_points = _straight_control_points(sx, sy, gx, gy)
        return [_control_points_to_terms(control_points)]

    # ObstacleId arrives as a bare Prolog ATOM Term (e.g. obs5) -- "+term"
    # is the right spec (not "+str", which would reject an atom), and
    # .functor is the plain Python string for a zero-arity atom Term
    # (verified directly against problog.logic.Term -- see this
    # project's own testing convention of checking rather than
    # assuming). Offset is "+term" too, for the same bare-int-vs-float
    # robustness every other numeric arg in this theory already takes.
    @problog_export_nondet("+float", "+float", "+float", "+float", "+term", "+term", "-list")
    def follow_boarder0(sx, sy, gx, gy, obstacle_id, offset):
        """Black-box Bug0 boundary-following planner (ProbLog predicate)
        -- see _follow_boarder0_control_points above for the actual
        computation. Fails (returns []) if obstacle_id names no known
        obstacle, or if a full circuit of its offset boundary never
        clears line-of-sight to (gx,gy)."""
        control_points = _follow_boarder0_control_points(
            sx, sy, gx, gy, obstacle_id.functor, float(offset))
        if control_points is None:
            return []
        return [_control_points_to_terms(control_points)]

    @problog_export_nondet("+float", "+float", "+float", "+float", "+term", "+term", "-list")
    def follow_boarder1(sx, sy, gx, gy, obstacle_id, offset):
        """Black-box Bug1 boundary-following planner (ProbLog predicate)
        -- see _follow_boarder1_control_points above for the actual
        computation. Fails (returns []) if obstacle_id names no known
        obstacle, or if a full circuit of its offset boundary never
        re-crosses the (sx,sy)-(gx,gy) segment."""
        control_points = _follow_boarder1_control_points(
            sx, sy, gx, gy, obstacle_id.functor, float(offset))
        if control_points is None:
            return []
        return [_control_points_to_terms(control_points)]
