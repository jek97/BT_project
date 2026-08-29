#!/usr/bin/env python3
"""
bt_actions.py

Lives in ./actions/ alongside moveto_planners.py and schema.yaml.

Canonical action/condition implementations matching schema.yaml,
written to be usable from TWO different callers:

  1. Our own ProbLog-based verification pipeline. moveto_planners.py's
     plan_astar_points/plan_straight_points ARE the shared plain-Python
     planning core, imported and REUSED here unchanged, never
     duplicated -- moveto_continuous.pl itself keeps calling the
     SEPARATE ProbLog-facing plan_astar/plan_straight predicates
     (also defined in moveto_planners.py, on top of the same core)
     directly, unaffected by anything in this file.

  2. A future BehaviorTree.cpp integration. A pybind11 (or ctypes, or
     ROS2 behaviortree_ros2) bridge could register the bt_-prefixed
     functions below directly as C++ node tick() callbacks: their
     signatures and return shapes match schema.yaml's port
     declarations exactly, using PLAIN Python types throughout
     (float / list of (x,y) tuples / str / bool / dict) -- never a
     ProbLog Term object. This file (and the plain-Python half of
     moveto_planners.py it calls into) has NO ProbLog import anywhere,
     so a BT.cpp bridge that never installs ProbLog can still import
     and call bt_plan_astar/bt_plan_straight -- see moveto_planners.py's
     own header for why its ProbLog-specific half is wrapped in a
     try/except instead of a hard import.

MoveTo (and both conditions) are DELIBERATELY NOT given a directly
-executable Python implementation here. MoveTo's real behaviour is
the STOCHASTIC action theory in moveto_continuous.pl -- noisy
position, noisy battery, exact trigger-crossing detection via
closed-form algebra or bracket-scan+bisection. There is no correct
way to "run" that in a plain Python function without reimplementing
the entire probabilistic model outside ProbLog, and a naive
deterministic stand-in would silently misrepresent what the theory
actually says happens -- worse than no implementation at all.
AtGoal/HaltedWith are native Prolog conditions over a situation;
Python has no situation to evaluate them against on its own.

What IS provided for all three is their INTERFACE (matching
schema.yaml's ports exactly) plus a TERM BUILDER -- a function
translating bound port values into the corresponding
moveto_continuous.pl term text. This is the piece a future
BT-tree-to-Prolog translator needs: given a BT.cpp node's bound
inputs, produce the Prolog subterm to splice into a
seq_node(...)/fallback_node(...) list. Building that translator
itself (parsing a whole BT.cpp XML tree) is a separate, larger step
-- not done here; this file only provides the per-node building
blocks it will need.
"""
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from moveto_planners import plan_astar_points, plan_straight_points


# =====================================================================
# ACTIONS -- callable implementations (PlanAstar, PlanStraight)
# =====================================================================
def bt_plan_astar(sx, sy, gx, gy):
    """
    BT.cpp-compatible wrapper around moveto_planners.py's
    plan_astar_points -- matches PlanAstar's three output ports in
    schema.yaml exactly, returned together as one dict:
        {control_points, reason, status}
    control_points is [] and reason is "no_path" if A* found no path
    (unreachable goal, or the map failed to load) -- see
    moveto_planners.py's own _astar_control_points for exactly which
    cases that covers.
    """
    control_points = plan_astar_points(sx, sy, gx, gy)
    if control_points is None:
        return {"control_points": [], "reason": "no_path", "status": False}
    return {
        "control_points": [(float(x), float(y)) for x, y in control_points],
        "reason": "completed",
        "status": True,
    }


def bt_plan_straight(sx, sy, gx, gy):
    """BT.cpp-compatible wrapper around plan_straight_points -- same
    shape and rationale as bt_plan_astar above; a straight line between
    two finite points essentially always succeeds."""
    control_points = plan_straight_points(sx, sy, gx, gy)
    return {
        "control_points": [(float(x), float(y)) for x, y in control_points],
        "reason": "completed",
        "status": True,
    }


# =====================================================================
# ACTIONS -- interface-only (MoveTo): term builder, not an executor
# =====================================================================
def moveto_leg_term(control_points, triggers):
    """
    Build the moveto_continuous.pl TERM TEXT for one MoveTo node's
    bound inputs -- moveto_leg(ControlPoints,Triggers). Triggers is
    REQUIRED, matching moveto_continuous.pl's own moveto_leg/2 (there is
    deliberately no sugar/default form on either side -- every leg
    states its own protection level explicitly; pass [] for a
    genuinely unprotected leg).

    control_points: list of (x,y) pairs.
    triggers: list of strings (e.g. ["collision","battery"]).

    Returns Prolog source text, e.g.:
        "moveto_leg([point(1.0,2.0),point(3.0,4.0)],[collision,battery])"
    """
    cp_text = "[" + ",".join(
        f"point({float(x)},{float(y)})" for x, y in control_points) + "]"
    trig_text = "[" + ",".join(str(t) for t in triggers) + "]"
    return f"moveto_leg({cp_text},{trig_text})"


# =====================================================================
# ACTIONS -- term builder for the planners (PlanAstar/PlanStraight)
# =====================================================================
def plan_with_term(algorithm, goal, cp_var="CP"):
    """
    Build the moveto_continuous.pl TERM TEXT for one PlanAstar/
    PlanStraight node's bound inputs -- planWith(Algorithm,
    point(GoalX,GoalY), CPVar) -- matching planWith's own 3-arg
    signature (Algorithm, Goal, CP) in moveto_continuous.pl. CPVar is
    left as a FREE PROLOG VARIABLE NAME (default "CP"), not a value,
    since ControlPoints is this node's own OUTPUT, meant to be shared
    forward with a subsequent MoveTo node using the SAME variable name
    -- see moveto_continuous.pl's own note on the "leave a variable
    free, let a prior step bind it" pattern. Pass a distinct cp_var
    (e.g. "CP1", "CP2") when building more than one planning call in
    the same plan, per the fallback_node variable-sharing gotcha
    documented in moveto_continuous.pl.

    algorithm: "astar" or "straight" (a bare Prolog atom, unquoted).
    goal: an (x,y) pair.

    Returns Prolog source text, e.g.:
        "planWith(astar,point(17.0,17.0),CP)"
    """
    gx, gy = goal
    return f"planWith({algorithm},point({float(gx)},{float(gy)}),{cp_var})"


# =====================================================================
# CONDITIONS -- interface-only: term builders
# =====================================================================
def at_goal_cond_term(tolerance):
    """cond(at_goal(Tolerance)) term text -- matches AtGoal's
    tolerance port in schema.yaml."""
    return f"cond(at_goal({float(tolerance)}))"


def halted_with_cond_term(reason):
    """cond(halted_with_cond(Reason)) term text -- matches
    HaltedWith's reason port in schema.yaml. `reason` is written
    VERBATIM as Prolog text, unquoted: a bare atom for
    completed/battery_depleted/a trigger name, or "crashed(_)" /
    "crashed(obs5)" / "obstacle_in_bound(_,_)" / "battery_under(20)"
    (etc.) for the Reasons that carry extra info -- see schema.yaml's
    own note on HaltedWith's reason port. A bare "crashed" (no
    obstacle argument) no longer matches anything."""
    return f"cond(halted_with_cond({reason}))"


def obstacle_in_bound_cond_term(threshold):
    """cond(obstacle_in_bound(Threshold)) term text -- matches
    ObstacleInBound's threshold port in schema.yaml."""
    return f"cond(obstacle_in_bound({float(threshold)}))"


def obstacle_on_path_cond_term(threshold):
    """cond(obstacle_on_path(Threshold)) term text -- matches
    ObstacleOnPath's threshold port in schema.yaml. Distinct from
    obstacle_in_bound_cond_term above: this only fires for obstacles
    the CURRENT walk's trajectory actually enters, not any nearby
    obstacle."""
    return f"cond(obstacle_on_path({float(threshold)}))"


def battery_below_cond_term(threshold):
    """cond(battery_below(Threshold)) term text -- matches
    BatteryBelow's threshold port in schema.yaml."""
    return f"cond(battery_below({float(threshold)}))"


def battery_equal_cond_term(threshold):
    """cond(battery_equal(Threshold)) term text -- matches
    BatteryEqual's threshold port in schema.yaml."""
    return f"cond(battery_equal({float(threshold)}))"


def battery_over_cond_term(threshold):
    """cond(battery_over(Threshold)) term text -- matches
    BatteryOver's threshold port in schema.yaml."""
    return f"cond(battery_over({float(threshold)}))"


# =====================================================================
# Registry -- maps schema.yaml's IDs to their implementation here.
# Not required for either caller to function (both can call the
# functions above directly), but gives one place that stays
# consistent with schema.yaml, and a natural hook for future
# consistency-checking or XML/tree-translation tooling.
# =====================================================================
ACTIONS = {
    "MoveTo": {
        "kind": "interface_only",
        "prolog_action": "moveto_leg",
        "term_builder": moveto_leg_term,
    },
    "PlanAstar": {
        "kind": "callable",
        "prolog_action": "planWith",
        "prolog_algorithm": "astar",
        "func": bt_plan_astar,
        "term_builder": plan_with_term,
    },
    "PlanStraight": {
        "kind": "callable",
        "prolog_action": "planWith",
        "prolog_algorithm": "straight",
        "func": bt_plan_straight,
        "term_builder": plan_with_term,
    },
}

CONDITIONS = {
    "AtGoal": {
        "kind": "interface_only",
        "prolog_condition": "at_goal",
        "term_builder": at_goal_cond_term,
    },
    "HaltedWith": {
        "kind": "interface_only",
        "prolog_condition": "halted_with_cond",
        "term_builder": halted_with_cond_term,
    },
    "ObstacleInBound": {
        "kind": "interface_only",
        "prolog_condition": "obstacle_in_bound",
        "term_builder": obstacle_in_bound_cond_term,
    },
    "ObstacleOnPath": {
        "kind": "interface_only",
        "prolog_condition": "obstacle_on_path",
        "term_builder": obstacle_on_path_cond_term,
    },
    "BatteryBelow": {
        "kind": "interface_only",
        "prolog_condition": "battery_below",
        "term_builder": battery_below_cond_term,
    },
    "BatteryEqual": {
        "kind": "interface_only",
        "prolog_condition": "battery_equal",
        "term_builder": battery_equal_cond_term,
    },
    "BatteryOver": {
        "kind": "interface_only",
        "prolog_condition": "battery_over",
        "term_builder": battery_over_cond_term,
    },
}