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
     ControlPoints = [point(X0,Y0), ...] ProbLog terms, length 3k+1, in
     EXACTLY the format moveto_continuous.pl's spline_point/4 expects.

  2. PLAIN-PYTHON API for a BT.cpp / bt_actions.py caller that has
     nothing to do with ProbLog -- plan_astar_points/plan_straight_points
     below, returning plain (x,y) float tuples, no ProbLog Term objects
     and no ProbLog import required to call them. The `import problog`
     needed for role 1 is wrapped in a try/except (_HAVE_PROBLOG) so that
     importing this module from a pure BT.cpp bridge that never installs
     ProbLog still works -- only the plan_astar/plan_straight ProbLog
     predicates themselves become unavailable in that case, exactly
     mirroring how plan_astar already degrades gracefully (fails rather
     than crashing) when the map itself fails to load.

Both roles funnel through the SAME plain-Python core
(_astar_control_points / _straight_control_points) -- no duplicated A*/
spline logic between the ProbLog-facing and BT.cpp-facing entry points.

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
import os
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
