#!/usr/bin/env python3
"""
moveto_planners.py

PROBLOG EXTERNAL-PREDICATE MODULE -- imported directly by ProbLog's own
:- use_module('moveto_planners.py'). directive inside moveto_continuous.pl
(see problog.clausedb.ClauseDB.load_external_module, which execs this file
as a plain Python module the moment that directive is loaded, triggering
the @problog_export_nondet(...)-decorated functions below to register
themselves as callable ProbLog predicates).

This is deliberately a SEPARATE file from run_plan_continuous_safety.py:
this module's only job is to BE the black box ProbLog calls into; the
runner script's job is to load/evaluate the ProbLog model (now via the
ProbLog Python API, not the CLI) and report results. Keeping them apart
means moveto_continuous.pl's own :- use_module(...) directive only ever
imports this small, focused file -- not the whole reporting/plotting
script.

Exposes two predicates to ProbLog, both INSTANTANEOUS (no situation
change, no duration of their own -- they only ever get called to COMPUTE
a moveto leg's ControlPoints argument, right before that leg actually
runs):

    plan_astar(+SX,+SY,+GX,+GY, -ControlPoints)
    plan_straight(+SX,+SY,+GX,+GY, -ControlPoints)

Both take a start and goal position (world-frame metres) and return a
cubic-Bezier control-point list -- ControlPoints = [point(X0,Y0), ...],
length 3k+1 -- in EXACTLY the format moveto_continuous.pl's
spline_point/4 expects, i.e. the SAME format occupancy_grid_planner.py's
own offline pipeline produces. plan_astar reuses that exact pipeline
(A* over the map -> B-spline fit -> exact Bezier-chain extraction, see
occupancy_grid_planner.py); plan_straight instead builds a single,
degenerate (collinear-control-point) Bezier segment that reduces exactly
to a straight line, as discussed for the "nominal plan" placeholder
control_points/1 examples earlier.

FAILURE: if A* finds no path (start/goal unreachable, or the map failed
to load at import time), plan_astar returns zero solutions -- i.e. the
ProbLog predicate call FAILS, exactly like any other "this doesn't
happen" case elsewhere in this theory (first_collision_time, etc.),
rather than raising an exception that would crash the whole grounding
process. plan_straight only fails if given a degenerate (non-finite)
input; a straight line between two ordinary points always exists.

Map location: loaded ONCE, at import time (i.e. once per ProbLog run),
from ./environments/maps/map.yaml relative to THIS FILE's own directory
(not the CWD, and not relative to moveto_continuous.pl's directory,
though in the standard project layout those coincide) -- see
MAP_YAML_PATH below. If it can't be loaded, both predicates still work,
but plan_astar will always fail (no map to plan against) rather than
crashing at import time.
"""
import os
import sys

from problog.extern import problog_export_nondet
from problog.logic import Term, Constant

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))

# occupancy_grid_planner.py lives in ./plan_generation/ relative to THIS
# file's own directory, per the established project layout (see the
# header comment above moveto_continuous.pl's own obstacle-file consult
# directive) -- not as a sibling of moveto_planners.py itself.
_PLAN_GENERATION_DIR = os.path.join(_THIS_DIR, "plan_generation")
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

MAP_YAML_PATH = os.path.join(_THIS_DIR, "environments", "maps", "map.yaml")

# Same default inflation as occupancy_grid_planner.py's own --inflate default.
PLANNING_INFLATE_M = 0.5
OCC_THRESH = 50
CONNECTIVITY = 8

# -- load the map ONCE at import time (i.e. once per ProbLog run) --------
try:
    _RAW_MAP = load_map_yaml(MAP_YAML_PATH)
    _PLANNING_MAP = inflate_obstacles(_RAW_MAP, PLANNING_INFLATE_M) \
        if PLANNING_INFLATE_M > 0 else _RAW_MAP
    _MAP_LOAD_ERROR = None
except Exception as exc:  # noqa: BLE001 -- deliberately broad: ANY load
    # failure should degrade to "plan_astar always fails", not crash
    # ProbLog's grounding process.
    _RAW_MAP = None
    _PLANNING_MAP = None
    _MAP_LOAD_ERROR = exc


def _control_points_to_terms(control_points):
    """[(x,y), ...] -> [point(x,y), ...] as ProbLog Term objects."""
    return [Term("point", Constant(float(x)), Constant(float(y)))
            for x, y in control_points]


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


@problog_export_nondet("+float", "+float", "+float", "+float", "-list")
def plan_astar(sx, sy, gx, gy):
    """Black-box A* planner: world-frame (sx,sy) -> (gx,gy), via the map
    loaded at import time. Fails (returns []) if the map didn't load, if
    start/goal fall outside the map or on an obstacle cell, or if A*
    finds no path at all."""
    if _PLANNING_MAP is None:
        return []

    start_rc = _PLANNING_MAP.world_to_grid(sx, sy)
    goal_rc = _PLANNING_MAP.world_to_grid(gx, gy)

    if not _PLANNING_MAP.in_bounds(*start_rc) or not _PLANNING_MAP.in_bounds(*goal_rc):
        return []

    if start_rc == goal_rc:
        # Already there -- a zero-length, degenerate but VALID Bezier
        # (all four control points identical).
        terms = _control_points_to_terms([(sx, sy)] * 4)
        return [terms]

    path_rc = astar(_PLANNING_MAP, start_rc, goal_rc,
                     occ_thresh=OCC_THRESH, connectivity=CONNECTIVITY,
                     unknown_is_occupied=True)
    if path_rc is None:
        return []  # no path found -- predicate FAILS, not an exception

    path_xy = [_PLANNING_MAP.grid_to_world(r, c) for r, c in path_rc]

    if len(path_xy) < 2:
        terms = _control_points_to_terms([(sx, sy)] * 4)
        return [terms]

    try:
        tck, _u = fit_spline(path_xy, degree=3, smoothing=0.0)
        control_points, _k = bspline_to_bezier_chain(tck)
    except ValueError:
        # Degenerate/too-short path for a cubic fit -- fall back to a
        # straight line between the actual start/goal rather than failing
        # outright (A* DID find connectivity; only the spline fit failed).
        control_points = _straight_control_points(sx, sy, gx, gy)

    terms = _control_points_to_terms(control_points)
    return [terms]


@problog_export_nondet("+float", "+float", "+float", "+float", "-list")
def plan_straight(sx, sy, gx, gy):
    """Black-box straight-line planner: no map lookup at all, always
    succeeds for any finite (sx,sy),(gx,gy)."""
    control_points = _straight_control_points(sx, sy, gx, gy)
    terms = _control_points_to_terms(control_points)
    return [terms]