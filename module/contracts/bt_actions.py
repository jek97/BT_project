#!/usr/bin/env python3
"""
bt_actions.py

Lives in module/contracts/ alongside schema.yaml -- vocabulary.yaml and
goal_formula_check.py are its siblings there too (see this project's
top-level layout note for why these are grouped as "the schemas the
translators/validators check against").

Canonical action/condition implementations matching schema.yaml,
written to be usable from TWO different callers:

  1. Our own ProbLog-based verification pipeline. planners.py's (in
     module/theory/) plan_astar_points/plan_straight_points ARE the
     shared plain-Python planning core, imported and REUSED here
     unchanged, never duplicated -- basic_action_theory.pl itself
     keeps calling the SEPARATE ProbLog-facing plan_astar/plan_straight
     predicates (also defined in planners.py, on top of the same core)
     directly, unaffected by anything in this file.

  2. A future BehaviorTree.cpp integration. A pybind11 (or ctypes, or
     ROS2 behaviortree_ros2) bridge could register the bt_-prefixed
     functions below directly as C++ node tick() callbacks: their
     signatures and return shapes match schema.yaml's port
     declarations, using PLAIN Python types throughout (float / list
     of (x,y) tuples / str / bool / dict) -- never a ProbLog Term
     object, and NEVER a "reason"/"triggers" key/parameter either
     (schema.yaml still declares both -- the ProbLog translation side
     genuinely needs them, see that file's own note -- but neither
     means anything to a real BT.cpp node: "reason" is a Prolog-
     Reason-atom artifact status/control_points/value already make
     redundant, and "triggers" is how THIS project's own reactive-
     interrupt derivation is threaded into a Prolog term, not a
     concept a real C++ node/tree structure needs at all). This file
     (and the plain-Python half of planners.py it calls into) has NO
     ProbLog import anywhere, so a BT.cpp bridge that never installs
     ProbLog can still import and call bt_plan_astar/bt_plan_straight
     -- see planners.py's own header for why its ProbLog-specific half
     is wrapped in a try/except instead of a hard import.

MoveTo (and both conditions) are DELIBERATELY NOT given a directly
-executable Python implementation here. MoveTo's real behaviour is
the STOCHASTIC action theory in basic_action_theory.pl -- noisy
position, noisy battery, exact trigger-crossing detection via
closed-form algebra or bracket-scan+bisection. There is no correct
way to "run" that in a plain Python function without reimplementing
the entire probabilistic model outside ProbLog, and a naive
deterministic stand-in would silently misrepresent what the theory
actually says happens -- worse than no implementation at all.
DistanceBelow/DistanceEqual/DistanceOver/HaltedWith are native Prolog
conditions over a situation; Python has no situation to evaluate them
against on its own. InstallTool/UninstallTool/DeployTool/RetractTool
join this same interface-only group -- UNLIKE TakeSample below, their
own outcome isn't just a bare coin flip: it's a fixed-Duration walk-
alike with its own battery-only Triggers/earliest-halt search
(earliest_halt/13's own Mode=1 case, in basic_action_theory.pl) gating
WHETHER the coin flip even happens, the same non-trivial "stochastic
action theory with real preconditions and a trigger race" shape
MoveTo has, just without the continuous trajectory -- no faithful
plain-Python stand-in for that either.

ToolPosition/ToolsOfKind/NearestToolOfKind, Hitched/Deployed, and
SampleValueBelow/Equal/Over are ALSO interface-only, for the SAME
"Python has no situation to evaluate this against" reason
DistanceBelow/HaltedWith already have -- WHERE a tool instance
currently sits, WHETHER a tool is hitched or deployed, and WHAT VALUE a
named sample recorded are all facts about the CURRENT situation
(tool_position/4, hitch/2, deployed/1, halted_with/2 -- see basic_
action_theory.pl's own Section 5c/5d), not something a stateless
Python function can answer on its own without a real handle onto that
state (a future BT.cpp bridge's own C++ node would read its OWN
blackboard/world-model instead).

TakeSample, despite ALSO being part of the stochastic action theory
(its outcome is a genuine ProbLog annotated disjunction, sample_result/3
in basic_action_theory.pl -- see that predicate's own note), DOES get a
real, directly-executable implementation below (bt_take_sample), same
"callable" treatment as PlanWith -- unlike MoveTo's continuous noisy
trajectory, its own randomness is a single, closed-form Bernoulli(p)
draw with no approximation or partial-reimplementation risk: a plain
random.random() < p reproduces basic_action_theory.pl's own
sample_result/3 EXACTLY, not just approximately.

What IS provided for all three is their INTERFACE (matching
schema.yaml's ports exactly) plus a TERM BUILDER -- a function
translating bound port values into the corresponding
basic_action_theory.pl term text. This is the piece a future
BT-tree-to-Prolog translator needs: given a BT.cpp node's bound
inputs, produce the Prolog subterm to splice into a
seq_node(...)/fallback_node(...) list. Building that translator
itself (parsing a whole BT.cpp XML tree) is a separate, larger step
-- not done here (see module/translators/bt_to_prolog.py, which
implements this translator directly rather than through this file);
this file only provides the per-node building blocks it documents.
"""
import os
import random
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_THEORY_DIR = os.path.join(os.path.dirname(_THIS_DIR), "theory")
if _THEORY_DIR not in sys.path:
    sys.path.insert(0, _THEORY_DIR)

from planners import (
    plan_astar_points, plan_straight_points, plan_voronoi_points, follow_boarder_points,
)


# =====================================================================
# ACTIONS -- callable implementations, one per PlanWith algorithm
# =====================================================================
def bt_plan_astar(sx, sy, gx, gy):
    """
    BT.cpp-compatible wrapper around planners.py's
    plan_astar_points -- matches PlanWith's own control_points/status
    output ports in schema.yaml exactly (algorithm="astar" case),
    returned together as one dict:
        {control_points, status}
    No "reason" key: that's a Prolog-Reason-atom artifact needed by
    the ProbLog translation side (see basic_action_theory.pl's own
    do_node(planWith(...)) and tag_reason/3), not something a real
    BT.cpp caller needs -- status (True/False) IS the NodeStatus
    SUCCESS/FAILURE signal, and control_points being [] already says
    "no path" on its own. control_points is [] and status is False if
    A* found no path (unreachable goal, or the map failed to load) --
    see planners.py's own _astar_control_points for exactly which
    cases that covers.
    """
    control_points = plan_astar_points(sx, sy, gx, gy)
    if control_points is None:
        return {"control_points": [], "status": False}
    return {
        "control_points": [(float(x), float(y)) for x, y in control_points],
        "status": True,
    }


def bt_plan_straight(sx, sy, gx, gy):
    """BT.cpp-compatible wrapper around plan_straight_points -- same
    shape and rationale as bt_plan_astar above (no "reason" key, see
    its own note); a straight line between two finite points
    essentially always succeeds."""
    control_points = plan_straight_points(sx, sy, gx, gy)
    return {
        "control_points": [(float(x), float(y)) for x, y in control_points],
        "status": True,
    }


def bt_plan_voronoi(sx, sy, gx, gy):
    """BT.cpp-compatible wrapper around planners.py's
    plan_voronoi_points -- same shape/rationale as bt_plan_astar above
    (no "reason" key, see its own note). control_points is [] and
    status is False only if a roadmap exists but start/goal are
    genuinely disconnected within it; degrades to a straight line
    (never fails) when there are no obstacles to route around."""
    control_points = plan_voronoi_points(sx, sy, gx, gy)
    if control_points is None:
        return {"control_points": [], "status": False}
    return {
        "control_points": [(float(x), float(y)) for x, y in control_points],
        "status": True,
    }


def bt_follow_boarder(sx, sy, obstacle_id, offset):
    """BT.cpp-compatible wrapper around planners.py's
    follow_boarder_points -- matches PlanWith's obstacle_id/offset
    input ports (algorithm="follow_boarder" case) exactly (no goal
    port -- this planner doesn't decide when to leave the boundary,
    see follow_boarder_points's own docstring). No "reason" key, see
    bt_plan_astar's own note. control_points is [] and status is False
    only if obstacle_id names no known obstacle."""
    control_points = follow_boarder_points(sx, sy, obstacle_id, offset)
    if control_points is None:
        return {"control_points": [], "status": False}
    return {
        "control_points": [(float(x), float(y)) for x, y in control_points],
        "status": True,
    }


_PLAN_ALGORITHM_FUNCS = {
    "astar": bt_plan_astar,
    "straight": bt_plan_straight,
    "voronoi": bt_plan_voronoi,
}


def bt_take_sample(success_probability=0.5, value_mean=5.0, value_sigma=2.0):
    """
    BT.cpp-compatible implementation of TakeSample -- a single
    fixed-probability coin flip, matching basic_action_theory.pl's own
    sample_result/3 EXACTLY (see this module's own header for why this
    gets a real implementation, unlike MoveTo), PLUS, only on success, a
    SECOND independent draw for the value (0..10) -- matching
    sample_value/3 EXACTLY too: round(random.gauss(mean,sigma)) clipped
    to [0,10] is not an approximation of that predicate's own
    discretized-Normal table, it's mathematically the SAME distribution
    -- P(round(X)=v) for X~Normal(mean,sigma) is exactly CDF(v+0.5)-CDF
    (v-0.5) (with the two boundary bins absorbing their own outer tail
    the same way), which is precisely how config_to_prolog.py's own
    _discretized_normal_block computes sample_value/3's weights in the
    first place. success_probability/value_mean/value_sigma are NOT
    read from config.yaml here -- this file never touches config.yaml
    directly; the caller supplies them, same as every other parameter
    throughout this file (a future BT.cpp bridge would read this
    problem's own config.yaml sample.success_probability/sample.value.
    mean/sample.value.sigma itself and pass them through).

    Returns {status, value} matching TakeSample's own status port in
    schema.yaml plus the drawn value -- NO "reason" key: that's a
    Prolog-Reason-atom artifact needed only by the ProbLog translation
    side (see basic_action_theory.pl's own do_node(take_sample(...))
    and tag_reason/3, which build sample_success(X,Y,V,SampleId,
    ActionCode)/sample_failure(X,Y,SampleId,ActionCode) -- ActionCode/
    SampleId-tagging and recording the robot's own position (X,Y)
    happen ONLY there), not something this plain-Python callable needs
    to reproduce -- a real BT.cpp caller already knows its own current
    position and its own configured id without this function echoing
    either back, and status (True/False) alone IS the NodeStatus
    SUCCESS/FAILURE signal. value is the one genuine new piece of
    information only this function's own random draw can supply --
    None on failure (no value is ever drawn then, see poss(take_sample
    (...))'s own note: a failed reading is exactly that, no number to
    report).
    """
    success = random.random() < success_probability
    if not success:
        return {"status": False, "value": None}
    value = min(10, max(0, round(random.gauss(value_mean, value_sigma))))
    return {"status": True, "value": value}


def bt_plan_with(algorithm, sx, sy, gx=None, gy=None, obstacle_id=None, offset=None):
    """ONE dispatch entry point covering every PlanWith algorithm --
    the direct Python-callable analogue of basic_action_theory.pl's own
    planWith(Algorithm,Goal,CP,ActionCode)/plan_call/8 dispatch (see
    schema.yaml's own note on why the four algorithms collapsed into
    one BT.cpp action). Routes to bt_plan_astar/bt_plan_straight/
    bt_plan_voronoi (gx,gy required) for those three algorithm values,
    or bt_follow_boarder (obstacle_id,offset required) for
    "follow_boarder" -- same {control_points, status} return shape
    either way (see bt_plan_astar's own note on why there's no
    "reason" key)."""
    if algorithm == "follow_boarder":
        return bt_follow_boarder(sx, sy, obstacle_id, offset)
    if algorithm in _PLAN_ALGORITHM_FUNCS:
        return _PLAN_ALGORITHM_FUNCS[algorithm](sx, sy, gx, gy)
    raise ValueError(f"Unknown PlanWith algorithm '{algorithm}'.")


# =====================================================================
# ACTIONS -- interface-only (MoveTo): term builder, not an executor
# =====================================================================
def moveto_leg_term(control_points, triggers):
    """
    Build the basic_action_theory.pl TERM TEXT for one MoveTo node's
    bound inputs -- moveto_leg(ControlPoints,Triggers). Triggers is
    REQUIRED, matching basic_action_theory.pl's own moveto_leg/2 (there is
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
# ACTIONS -- interface-only (InstallTool/UninstallTool): term builders
# =====================================================================
def install_tool_leg_term(tool, triggers, action_code):
    """
    Build the basic_action_theory.pl TERM TEXT for one InstallTool
    node's bound inputs -- install_tool_leg(Tool,Triggers,ActionCode).
    triggers is REQUIRED (pass [] for none), same convention as
    moveto_leg_term above, but RESTRICTED to battery-related names only
    -- see schema.yaml's own InstallTool entry.

    tool: a tool INSTANCE id (e.g. "cart1", from this problem's own
        config.yaml tool.instances -- NOT a kind), as text (a bare
        Prolog atom, unquoted). See basic_action_theory.pl's own
        tool_instance/2 note for the kind-vs-instance distinction.
    triggers: list of strings (e.g. ["battery","battery_below(20)"]).
    action_code: a free Prolog variable name or bound atom, as text
        (e.g. "a5").

    Returns Prolog source text, e.g.:
        "install_tool_leg(cart1,[battery],a5)"
    """
    trig_text = "[" + ",".join(str(t) for t in triggers) + "]"
    return f"install_tool_leg({tool},{trig_text},{action_code})"


def uninstall_tool_leg_term(tool, triggers, action_code):
    """Build the basic_action_theory.pl TERM TEXT for one
    UninstallTool node's bound inputs -- uninstall_tool_leg(Tool,
    Triggers,ActionCode). Same shape/rationale as install_tool_leg_term
    above (tool is an INSTANCE id, e.g. "cart1")."""
    trig_text = "[" + ",".join(str(t) for t in triggers) + "]"
    return f"uninstall_tool_leg({tool},{trig_text},{action_code})"


def deploy_tool_leg_term(tool, triggers, action_code):
    """Build the basic_action_theory.pl TERM TEXT for one DeployTool
    node's bound inputs -- deploy_tool_leg(Tool,Triggers,ActionCode).
    Same shape/rationale as install_tool_leg_term above -- tool must be
    an ALREADY-INSTALLED instance id whose own kind is currently plow
    (checked in the theory, not here -- see basic_action_theory.pl's
    own poss(start_deploy_tool(...)))."""
    trig_text = "[" + ",".join(str(t) for t in triggers) + "]"
    return f"deploy_tool_leg({tool},{trig_text},{action_code})"


def retract_tool_leg_term(tool, triggers, action_code):
    """Build the basic_action_theory.pl TERM TEXT for one RetractTool
    node's bound inputs -- retract_tool_leg(Tool,Triggers,ActionCode).
    The mirror image of deploy_tool_leg_term above."""
    trig_text = "[" + ",".join(str(t) for t in triggers) + "]"
    return f"retract_tool_leg({tool},{trig_text},{action_code})"


# =====================================================================
# ACTIONS -- term builder for the consolidated planner (PlanWith)
# =====================================================================
def plan_with_term(algorithm, goal, cp_var, action_code):
    """
    Build the basic_action_theory.pl TERM TEXT for one PlanWith node's
    bound inputs, algorithm in {"astar","straight","voronoi"} --
    planWith(Algorithm,point(GoalX,GoalY),CPVar,ActionCode) -- matching
    planWith's own 4-arg signature (Algorithm, Goal, CP, ActionCode) in
    basic_action_theory.pl. Use follow_boarder_term below instead for
    algorithm="follow_boarder" (no goal port, a compound Algorithm term
    instead). CPVar is left as a FREE PROLOG VARIABLE NAME (e.g. "CP"),
    not a value, since ControlPoints is this node's own OUTPUT, meant
    to be shared forward with a subsequent MoveTo node using the SAME
    variable name -- see basic_action_theory.pl's own note on the
    "leave a variable free, let a prior step bind it" pattern. Pass a
    distinct cp_var (e.g. "CP1", "CP2") when building more than one
    planning call in the same plan, per the fallback_node variable-
    sharing gotcha documented in basic_action_theory.pl. action_code
    (e.g. "a3") identifies THIS PlanWith occurrence -- same per-
    occurrence code moveto_leg_term's own ActionCode already uses --
    and rides through into the RECORDED Reason (completed(Algorithm,
    Goal,ActionCode)/no_path(Algorithm,Goal,ActionCode)), not just this
    call term, via tag_reason/3 (see do_node(planWith(...))'s own note).

    algorithm: "astar", "straight", or "voronoi" (a bare Prolog atom,
        unquoted).
    goal: an (x,y) pair.
    cp_var: a free Prolog variable name, as text (e.g. "CP").
    action_code: a free Prolog variable name or bound atom, as text
        (e.g. "a3").

    Returns Prolog source text, e.g.:
        "planWith(astar,point(17.0,17.0),CP,a3)"
    """
    gx, gy = goal
    return f"planWith({algorithm},point({float(gx)},{float(gy)}),{cp_var},{action_code})"


def follow_boarder_term(obstacle_id, offset, cp_var, action_code):
    """Build the basic_action_theory.pl TERM TEXT for one PlanWith
    node's bound inputs when algorithm="follow_boarder" --
    planWith(follow_boarder(ObstacleId,Offset), point(0.0,0.0), CPVar,
    ActionCode), same "leave CP free"/ActionCode convention as
    plan_with_term above. The point(0.0,0.0) is a PLACEHOLDER, not a
    real goal -- follow_boarder takes no goal port at all (it doesn't
    decide when to leave the boundary; see planners.py's
    follow_boarder_points docstring), but planWith/4's Goal slot is
    part of the shared template every algorithm sits inside, so
    something has to fill it; plan_call/8's own follow_boarder clauses
    ignore it outright (the RECORDED Reason reports the honest atom
    `none` as this call's own Goal instead -- see do_node(planWith
    (...))'s own note in basic_action_theory.pl). obstacle_id is
    written VERBATIM as Prolog text (a bare atom, e.g. "obs5"), same
    convention as halted_with_cond_term's own reason argument below --
    NOT quoted, NOT float-parsed.

    obstacle_id: a Prolog atom, as text (e.g. "obs5").
    offset: distance to maintain from the obstacle's own boundary,
        metres -- typically the SAME Threshold as whichever trigger/
        condition supplied obstacle_id in the first place.
    cp_var: a free Prolog variable name, as text (e.g. "CP").
    action_code: a free Prolog variable name or bound atom, as text
        (e.g. "a4").

    Returns Prolog source text, e.g.:
        "planWith(follow_boarder(obs5,0.6),point(0.0,0.0),CP,a4)"
    """
    return f"planWith(follow_boarder({obstacle_id},{float(offset)}),point(0.0,0.0),{cp_var},{action_code})"


# =====================================================================
# ACTIONS -- interface-only (ToolPosition/ToolsOfKind/NearestToolOfKind):
# term builders. Genuinely state-dependent -- WHERE a tool instance
# currently sits (or whether it's hitched, or which free instance is
# closest to the robot's own current position) is a fact about the
# CURRENT situation, exactly the same "Python has no situation to
# evaluate this against on its own" reasoning DistanceBelow/BatteryOver/
# HaltedWith already have (see this module's own header) -- there is no
# plain-Python stand-in to write here, only the term shape.
# =====================================================================
def tool_position_query_term(tool_id, pos_var, action_code):
    """Build the basic_action_theory.pl TERM TEXT for one ToolPosition
    node's bound inputs -- tool_position_query(Id,Pos,ActionCode). Pos
    is left as a FREE PROLOG VARIABLE NAME (e.g. "Pos"), not a value --
    it's this node's own OUTPUT, same "leave a variable free" pattern
    plan_with_term's own cp_var already uses.

    tool_id: a tool instance id, as text (e.g. "cart1").
    pos_var: a free Prolog variable name, as text (e.g. "Pos").
    action_code: a free Prolog variable name or bound atom, as text.

    Returns Prolog source text, e.g.:
        "tool_position_query(cart1,Pos,a6)"
    """
    return f"tool_position_query({tool_id},{pos_var},{action_code})"


def tools_of_kind_query_term(kind, tools_var, action_code):
    """Build the basic_action_theory.pl TERM TEXT for one ToolsOfKind
    node's bound inputs -- tools_of_kind_query(Kind,Tools,ActionCode).
    Same "leave the output variable free" pattern as
    tool_position_query_term above -- Tools ends up bound to a Prolog
    list of tool(Id,point(X,Y)) terms.

    kind: "cart", "plow", or any future kind, as text (a bare atom).
    tools_var: a free Prolog variable name, as text (e.g. "Tools").
    action_code: a free Prolog variable name or bound atom, as text.
    """
    return f"tools_of_kind_query({kind},{tools_var},{action_code})"


def nearest_tool_of_kind_query_term(kind, id_var, pos_var, action_code):
    """Build the basic_action_theory.pl TERM TEXT for one
    NearestToolOfKind node's bound inputs -- nearest_tool_of_kind_query
    (Kind,Id,Pos,ActionCode). TWO free output variables this time (Id
    AND Pos), same pattern as the two builders above.

    kind: "cart", "plow", or any future kind, as text (a bare atom).
    id_var: a free Prolog variable name, as text (e.g. "ChosenId").
    pos_var: a free Prolog variable name, as text (e.g. "ChosenPos").
    action_code: a free Prolog variable name or bound atom, as text.
    """
    return f"nearest_tool_of_kind_query({kind},{id_var},{pos_var},{action_code})"


def hitched_id_query_term(id_var, action_code):
    """Build the basic_action_theory.pl TERM TEXT for one HitchedId
    node's bound inputs -- hitched_id_query(Id,ActionCode). NO input
    port at all (unlike tool_position_query_term/nearest_tool_of_kind_
    query_term above) -- Id is this node's own OUTPUT, same "leave a
    variable free" pattern.

    id_var: a free Prolog variable name, as text (e.g. "Id").
    action_code: a free Prolog variable name or bound atom, as text.

    Returns Prolog source text, e.g.:
        "hitched_id_query(Id,a3)"
    """
    return f"hitched_id_query({id_var},{action_code})"


# =====================================================================
# CONDITIONS -- interface-only: term builders
# =====================================================================
def distance_below_cond_term(goal, threshold):
    """cond(distance_below(GX,GY,Threshold)) term text -- matches
    DistanceBelow's goal/threshold ports in schema.yaml. PARAMETRIZED,
    same as obstacle_in_bound_cond_term/battery_below_cond_term below
    -- there is no global "the goal" fact this reads instead.

    goal: an (x,y) pair.
    threshold: distance threshold, metres.
    """
    gx, gy = goal
    return f"cond(distance_below({float(gx)},{float(gy)},{float(threshold)}))"


def distance_equal_cond_term(goal, threshold):
    """cond(distance_equal(GX,GY,Threshold)) term text -- matches
    DistanceEqual's goal/threshold ports in schema.yaml. Same shape as
    distance_below_cond_term above, exact-equality comparison."""
    gx, gy = goal
    return f"cond(distance_equal({float(gx)},{float(gy)},{float(threshold)}))"


def distance_over_cond_term(goal, threshold):
    """cond(distance_over(GX,GY,Threshold)) term text -- matches
    DistanceOver's goal/threshold ports in schema.yaml. Same shape as
    distance_below_cond_term above, ">" comparison."""
    gx, gy = goal
    return f"cond(distance_over({float(gx)},{float(gy)},{float(threshold)}))"


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


def line_of_sight_clear_cond_term(obstacle_id, goal):
    """cond(line_of_sight_clear(ObstacleId,GX,GY)) term text -- matches
    LineOfSightClear's obstacle_id/goal ports in schema.yaml. Bug0's
    own boundary-leave rule as a standalone condition; no
    crosses_segment counterpart exists -- see schema.yaml's own note
    on LineOfSightClear for why. obstacle_id is written VERBATIM as
    Prolog text (a bare atom), same convention as halted_with_cond_term
    above."""
    gx, gy = goal
    return f"cond(line_of_sight_clear({obstacle_id},{float(gx)},{float(gy)}))"


def sample_value_below_cond_term(sample_id, threshold):
    """cond(sample_value_below(SampleId,Threshold)) term text --
    matches SampleValueBelow's id/threshold ports in schema.yaml.
    sample_id must match the id= of an earlier TakeSample in the same
    tree (written VERBATIM as Prolog text, a bare atom -- same
    convention as halted_with_cond_term's own reason argument)."""
    return f"cond(sample_value_below({sample_id},{float(threshold)}))"


def sample_value_equal_cond_term(sample_id, threshold):
    """cond(sample_value_equal(SampleId,Threshold)) term text --
    matches SampleValueEqual's ports. Same shape as
    sample_value_below_cond_term above, exact-equality comparison."""
    return f"cond(sample_value_equal({sample_id},{float(threshold)}))"


def sample_value_over_cond_term(sample_id, threshold):
    """cond(sample_value_over(SampleId,Threshold)) term text --
    matches SampleValueOver's ports. Same shape as
    sample_value_below_cond_term above, ">" comparison."""
    return f"cond(sample_value_over({sample_id},{float(threshold)}))"


def hitched_cond_term(kind=None):
    """cond(hitched) or cond(hitched(Kind)) term text -- matches
    Hitched's own optional kind port in schema.yaml. Pass kind=None (or
    omit it) for "is anything attached", a kind string (e.g. "plow")
    for "is specifically this kind attached"."""
    if kind:
        return f"cond(hitched({kind}))"
    return "cond(hitched)"


def deployed_cond_term():
    """cond(deployed) term text -- matches Deployed's own (zero-port)
    entry in schema.yaml. No arguments -- only one tool can ever be
    hitched at a time, so there's nothing to parametrize."""
    return "cond(deployed)"


def ploughed_at_cond_term(goal):
    """cond(ploughed_at(GX,GY)) term text -- matches PloughedAt's goal
    port in schema.yaml. Same shape as distance_below_cond_term above
    (one Point, no threshold).

    goal: an (x,y) pair.
    """
    gx, gy = goal
    return f"cond(ploughed_at({float(gx)},{float(gy)}))"


def ploughed_between_cond_term(p1, p2):
    """cond(ploughed_between(X1,Y1,X2,Y2)) term text -- matches
    PloughedBetween's p1/p2 ports in schema.yaml. TRUE iff every cell
    the straight line connecting p1's own cell center to p2's touches
    (standard integer Bresenham line algorithm over the discretized
    grid -- NOT the full box spanned by p1/p2) is ploughed.

    p1, p2: (x,y) pairs, the line's two endpoints (either order).
    """
    x1, y1 = p1
    x2, y2 = p2
    return f"cond(ploughed_between({float(x1)},{float(y1)},{float(x2)},{float(y2)}))"


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
    # NOT a single "term_builder" entry here -- which one applies
    # depends on this node's own algorithm VALUE, not something fixed
    # per schema id the way every other action/condition here is: use
    # plan_with_term for astar/straight/voronoi, follow_boarder_term
    # for algorithm="follow_boarder" (see each builder's own docstring
    # above).
    "PlanWith": {
        "kind": "callable",
        "prolog_action": "planWith",
        "func": bt_plan_with,
    },
    "TakeSample": {
        "kind": "callable",
        "prolog_action": "take_sample",
        "func": bt_take_sample,
    },
    "InstallTool": {
        "kind": "interface_only",
        "prolog_action": "install_tool_leg",
        "term_builder": install_tool_leg_term,
    },
    "UninstallTool": {
        "kind": "interface_only",
        "prolog_action": "uninstall_tool_leg",
        "term_builder": uninstall_tool_leg_term,
    },
    "DeployTool": {
        "kind": "interface_only",
        "prolog_action": "deploy_tool_leg",
        "term_builder": deploy_tool_leg_term,
    },
    "RetractTool": {
        "kind": "interface_only",
        "prolog_action": "retract_tool_leg",
        "term_builder": retract_tool_leg_term,
    },
    "ToolPosition": {
        "kind": "interface_only",
        "prolog_action": "tool_position_query",
        "term_builder": tool_position_query_term,
    },
    "ToolsOfKind": {
        "kind": "interface_only",
        "prolog_action": "tools_of_kind_query",
        "term_builder": tools_of_kind_query_term,
    },
    "NearestToolOfKind": {
        "kind": "interface_only",
        "prolog_action": "nearest_tool_of_kind_query",
        "term_builder": nearest_tool_of_kind_query_term,
    },
    "HitchedId": {
        "kind": "interface_only",
        "prolog_action": "hitched_id_query",
        "term_builder": hitched_id_query_term,
    },
}

CONDITIONS = {
    "DistanceBelow": {
        "kind": "interface_only",
        "prolog_condition": "distance_below",
        "term_builder": distance_below_cond_term,
    },
    "DistanceEqual": {
        "kind": "interface_only",
        "prolog_condition": "distance_equal",
        "term_builder": distance_equal_cond_term,
    },
    "DistanceOver": {
        "kind": "interface_only",
        "prolog_condition": "distance_over",
        "term_builder": distance_over_cond_term,
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
    "LineOfSightClear": {
        "kind": "interface_only",
        "prolog_condition": "line_of_sight_clear",
        "term_builder": line_of_sight_clear_cond_term,
    },
    "SampleValueBelow": {
        "kind": "interface_only",
        "prolog_condition": "sample_value_below",
        "term_builder": sample_value_below_cond_term,
    },
    "SampleValueEqual": {
        "kind": "interface_only",
        "prolog_condition": "sample_value_equal",
        "term_builder": sample_value_equal_cond_term,
    },
    "SampleValueOver": {
        "kind": "interface_only",
        "prolog_condition": "sample_value_over",
        "term_builder": sample_value_over_cond_term,
    },
    "Hitched": {
        "kind": "interface_only",
        "prolog_condition": "hitched",
        "term_builder": hitched_cond_term,
    },
    "Deployed": {
        "kind": "interface_only",
        "prolog_condition": "deployed",
        "term_builder": deployed_cond_term,
    },
    "PloughedAt": {
        "kind": "interface_only",
        "prolog_condition": "ploughed_at",
        "term_builder": ploughed_at_cond_term,
    },
    "PloughedBetween": {
        "kind": "interface_only",
        "prolog_condition": "ploughed_between",
        "term_builder": ploughed_between_cond_term,
    },
}