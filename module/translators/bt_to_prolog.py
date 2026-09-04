#!/usr/bin/env python3
"""
module/translators/bt_to_prolog.py

The BT.cpp XML tree -> Prolog do_node term translator flagged as
"not yet built" in this project's own history. Reads a real
BehaviorTree.cpp v4 XML tree (a problem's own behavior_tree.xml),
validates it against module/contracts/schema.yaml (every leaf node must
be a known, correctly-instantiated schema action/condition; every
control-flow node must be one of the four BT.cpp built-ins this project
supports -- Sequence/Fallback, which map 1:1 to basic_action_theory.pl's
seq_node/fallback_node and never catch/redescend a reactive(_) status on
their own, and ReactiveSequence/ReactiveFallback, which map to
reactivesequence(Code)/reactivefallback(Code) and DO catch/locally
redescend one whose own code matches -- see _REACTIVE_CONTROL_FLOW's own
note and basic_action_theory.pl's own CONTROL-FLOW REDESCEND TARGETS
note for the full mechanism), and translates it into the nested
do_node/4 term text basic_action_theory.pl's plan/1 expects, plus one
reactive_children/2 fact per ReactiveSequence/ReactiveFallback (see
generate_plan_pl's own note on why those live separately).

WHERE THE RESULT GOES: generate_plan_pl() writes the problem's own
plan_generated.pl, a single plan/1 FACT (not a clause with a body --
see below), which basic_action_theory.pl consults (via its
problem_data.pl bootstrap -- see that file's Section 0) instead of
hand-defining plan/1 itself. This mirrors the existing
generated-artifact pattern (config_generated.pl, obstacles_generated.pl,
both written by this file's own sibling translators into the same
problem directory): the XML is the single source of truth for the
POLICY'S SHAPE from now on -- change the tree by editing the XML and
re-running, not by hand-editing basic_action_theory.pl.
main.py regenerates it automatically before every run, exactly like it
already does for config_generated.pl.

WHY A FACT, NOT A CLAUSE WITH A BODY: the old hand-written plan/1 was
`plan(seq_node([moveto_leg(CP,Triggers)])) :- control_points(CP),
default_triggers(Triggers).` -- CP had to be bound by a BODY goal
because the Node term itself only ever REFERENCED CP, never bound it.
Once a plan's own PlanAstar/PlanStraight leaf computes ControlPoints
itself (via planWith(Algorithm,Goal,CP)), CP is bound INSIDE the Node
term the moment do_node actually runs it -- there is nothing left for a
body to bind, so the generated plan/1 is a plain fact.

BLACKBOARD -> PROLOG VARIABLE TRANSLATION: BT.cpp wires one node's
output port to another's input port by giving both the SAME
"{blackboard_key}" attribute value (this is a REAL, standard BT.cpp
convention, not something invented for this project). control_points is
the one port in this schema that is ALWAYS wired this way -- PlanAstar/
PlanStraight compute it, they never receive it as a literal, and
MoveTo's own leg has no way to invent it, so a literal control_points
value is a hard error, never a valid input here. Each distinct
blackboard key becomes ONE Prolog variable, shared across every node
that references it, via straightforward unification -- e.g. "{cp}" on
both a PlanAstar and a MoveTo node becomes the SAME Prolog variable CP
in planWith(astar,point(GX,GY),CP) and moveto_leg(CP,[...]) -- the
direct Prolog analogue of BT.cpp's blackboard, and exactly the existing
"leave a variable free, let a prior step bind it" pattern already
documented in basic_action_theory.pl for hand-written multi-leg plans.
Never reuse one key across two DIFFERENT PlanAstar/PlanStraight calls
that should compute independent paths -- see basic_action_theory.pl's own
note on giving fallback_node branches distinct CP1/CP2 variables; the
same Prolog-variable-scope reasoning applies here.

OTHER PORT ENCODINGS (this project's own choice; schema.yaml describes
port TYPES, not a serialization -- see its own note pointing here):
    Point               "X;Y"                  e.g. goal="11.675;11.525"
    vector<std::string> ";"-separated           e.g. triggers="collision;battery"
    double / string     the attribute's own text, parsed by Python's
                         float()/left as-is respectively

CONTROL-FLOW GUARD DERIVATION: a MoveTo's own Triggers list is no
longer entirely hand-typed. For every <MoveTo>, this file now walks
UP the tree from it to the root; at each ReactiveSequence/
ReactiveFallback ancestor, every LEFT SIBLING of the branch leading to
the MoveTo (optionally wrapped in one or more <Inverter>) that reduces
to a single Condition leaf becomes an automatically-derived guard --
a Sequence-shaped ancestor requires its left siblings to stay TRUE
(interrupts on becoming false), a Fallback-shaped one requires them to
stay FALSE (interrupts on becoming true; each <Inverter> flips this
once), and the guard is tagged with THAT SPECIFIC ancestor's own code,
not necessarily the nearest enclosing reactive composite (two nested
reactive ancestors contributing guards to the same MoveTo get two
DIFFERENT codes -- see _reduce_guard_condition's own note). Rather
than looking up a pre-built "opposite" trigger name per condition
(which would need both crossing directions hand-implemented for every
condition, and silently do the wrong thing for any gap), the required
condition is built by NEGATING the actual Condition term when the
guard's polarity calls for it (reusing holds/2's own neg/1
combinator), and basic_action_theory.pl's guard_break(Cond,Code)
trigger + holds_leg/9 do a GENERIC bracket-scan+bisection search for
when THAT EXACT term stops holding -- see that file's own note above
holds_leg/9. A left sibling that is a memory-level (plain Sequence/
Fallback) guard, or that reduces to a HISTORY-based condition (e.g.
HaltedWith -- see _NON_CONTINUOUS_CONDITIONS), or that reduces to
neither a Condition leaf nor an <Inverter> chain over one, produces no
Triggers entry / a hard BTValidationError respectively -- see
_reduce_guard_condition.

Every MoveTo also gets `collision` and (when this problem's own
config.yaml has battery.enabled: true) `battery` ADDED AUTOMATICALLY,
regardless of what its own triggers="..." attribute says -- these are
universal physical hazards, not BT-structural guards, so the tree
author no longer has to spell them out (see _translate_leaf's own
moveto_leg branch). The triggers port itself is now OPTIONAL and
purely ADDITIVE: still there for leg-intrinsic termination conditions
that aren't derivable from tree structure at all (e.g. a Bug-algorithm
leg's own line_of_sight_clear/crosses_segment stopping rule).

VALIDATION IS A HARD FAILURE, not a warning: an unknown node tag, a
missing required port, an unrecognized attribute (anything not a
declared port, other than BT.cpp's own universal `name` display
attribute), or a MoveTo/moveto_leg control_points key with no
corresponding PlanAstar/PlanStraight producer anywhere in the tree
(which would silently leave CP unbound) all raise BTValidationError and
stop the run -- these are structural errors in the tree itself, not a
tunable value, so there is nothing sensible to warn-and-continue with
(same reasoning as module/translators/config_to_prolog.py's own hard
requirements, as opposed to its two non-fatal numeric warnings).
"""
import os
import re
import xml.etree.ElementTree as ET

import yaml

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
_DEFAULT_PROBLEM_DIR = os.path.join(_PROJECT_ROOT, "problems", "problem0")
DEFAULT_XML_PATH = os.path.join(_DEFAULT_PROBLEM_DIR, "behavior_tree.xml")
DEFAULT_SCHEMA_PATH = os.path.join(_PROJECT_ROOT, "module", "contracts", "schema.yaml")
DEFAULT_OUTPUT_PATH = os.path.join(_DEFAULT_PROBLEM_DIR, "plan_generated.pl")

# Every schema action's `id` maps to how it's dispatched below: which
# do_node/4 Prolog functor it becomes, and (for the two planners) which
# Algorithm atom plan_call/8 should dispatch on.
_ACTION_DISPATCH = {
    "MoveTo": {"kind": "moveto_leg"},
    "PlanAstar": {"kind": "planWith", "algorithm": "astar"},
    "PlanStraight": {"kind": "planWith", "algorithm": "straight"},
    # A third planner, SAME bare-atom shape, needing no new branch at
    # all -- it reuses the "planWith" kind verbatim.
    "PlanVoronoi": {"kind": "planWith", "algorithm": "voronoi"},
    # A fourth planner, dispatched through the SAME planWith template,
    # but with a COMPOUND Algorithm term (obstacle_id/offset ride
    # inside it, not a fixed atom, and NO goal_point at all -- this
    # planner doesn't take one, see its own "kind" branch below) --
    # zero further interface change needed.
    "FollowBoarder": {"kind": "planWith_follow_boarder"},
}
# "single_float_port": the shared shape of every cond(Functor(Value))
# condition whose one port is a plain float -- ObstacleInBound and
# BatteryBelow/Equal/Over all reduce to this, just with different
# functor/port names, so they share ONE translation branch below
# instead of several near-identical ones.
_CONDITION_DISPATCH = {
    # at_goal(GX,GY,Tol) -- PARAMETRIZED (no global "the goal" fact to
    # read instead), so it needs its own dispatch kind, not the shared
    # single_float_port shape (two ports, a Point AND a float).
    "AtGoal": {"kind": "at_goal_cond"},
    "ObstacleInBound": {"kind": "single_float_port", "functor": "obstacle_in_bound", "port": "threshold"},
    "ObstacleOnPath": {"kind": "single_float_port", "functor": "obstacle_on_path", "port": "threshold"},
    "BatteryBelow": {"kind": "single_float_port", "functor": "battery_below", "port": "threshold"},
    "BatteryEqual": {"kind": "single_float_port", "functor": "battery_equal", "port": "threshold"},
    "BatteryOver": {"kind": "single_float_port", "functor": "battery_over", "port": "threshold"},
    "HaltedWith": {"kind": "halted_with_cond"},
    # line_of_sight_clear(ObstacleId,GX,GY) -- obstacle_id verbatim
    # Prolog text (like HaltedWith's reason), goal a Point literal.
    "LineOfSightClear": {"kind": "line_of_sight_clear_cond"},
}
_CONTROL_FLOW = {"Sequence": "seq_node", "Fallback": "fallback_node"}
# ReactiveSequence/ReactiveFallback are NOT simple functor-renames like
# Sequence/Fallback above -- translating one means assigning it a fresh,
# unique code, recursing into its OWN children with that code as the
# "current enclosing reactive composite" for anything underneath (so
# every reactive-classified trigger inside gets tagged with it -- see
# _REACTIVE_TRIGGER_FUNCTORS below), and factoring those children OUT
# into their own reactive_children/2 fact rather than inlining them --
# see basic_action_theory.pl's own CONTROL-FLOW REDESCEND TARGETS note
# (above do_node(reactivesequence(...))) for why a separately-resolved
# fact is required (the SAME "fresh variables on every resolution"
# reasoning plan(Node) itself already relies on). Handled as its own
# branch in _translate_node, not via _CONTROL_FLOW's simple lookup.
_REACTIVE_CONTROL_FLOW = {"ReactiveSequence": "reactivesequence", "ReactiveFallback": "reactivefallback"}

# Condition ids that are HISTORY-based rather than a live, continuous
# fluent -- they cannot change WHILE a leg is running (nothing appends
# to the situation history until the CURRENT leg itself halts), so
# there is nothing for a crossing-search to ever watch: the composite
# that contains one already checked it, ONCE, before ever descending
# into this branch. Excluded from automatic guard-trigger derivation
# (see _reduce_guard_condition) for exactly that reason -- NOT because
# it's unsupported, but because "make it reactive" would be a no-op at
# best; basic_action_theory.pl's holds_leg/9 (the generic engine behind
# guard derivation) deliberately has NO clause for it either, so this
# exclusion also prevents a missing-clause silently reading as "already
# false at T0" there. A future condition added to schema.yaml that is
# similarly history-based (not a function of the CURRENT leg's own
# position/battery) belongs here too.
_NON_CONTINUOUS_CONDITIONS = {"HaltedWith"}

# Trigger-list functors that are REACTIVE-classified in leg_status/9
# (basic_action_theory.pl) -- i.e. everything except the two original,
# unparametrized, never-reactive names 'collision' and 'battery' (which
# classify straight to false, via crashed(_)/battery_depleted). Every
# token using one of THESE functors, in a MoveTo's own triggers list,
# gets the current enclosing reactive composite's own code appended as
# an extra trailing argument (e.g. "battery_below(70)" in the XML
# becomes battery_below(70,rc3) in the generated Prolog) -- see
# trigger_crossing_time/11's own note in basic_action_theory.pl.
_REACTIVE_TRIGGER_FUNCTORS = {
    "obstacle_in_bound", "obstacle_on_path",
    "battery_below", "battery_equal", "battery_over",
    "line_of_sight_clear", "crosses_segment",
}

# BT.cpp's own universal attribute, present on any node purely for
# display/debugging -- never a real port, always allowed, never
# validated against a schema port list.
_ALWAYS_ALLOWED_ATTRS = {"name"}

_BLACKBOARD_RE = re.compile(r"^\{(\w+)\}$")
_VALID_PROLOG_VAR_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


class BTValidationError(Exception):
    """Raised for any structural problem in the BT XML relative to the
    schema -- always fatal, never a warning (see module docstring)."""


def load_schema(schema_path=DEFAULT_SCHEMA_PATH):
    with open(schema_path) as f:
        return yaml.safe_load(f)


def _schema_port_index(schema):
    """id -> {port_name: port_spec} for every action AND condition."""
    index = {}
    for kind_key in ("actions", "conditions"):
        for entry in schema.get(kind_key, []):
            index[entry["id"]] = {p["name"]: p for p in entry.get("ports", [])}
    return index


def _is_blackboard_ref(value):
    return _BLACKBOARD_RE.match(value) is not None


def _blackboard_key(value):
    return _BLACKBOARD_RE.match(value).group(1)


class _VarPool:
    """Maps each distinct blackboard key to ONE Prolog variable name,
    reused every time that key is seen again -- see the module
    docstring's "BLACKBOARD -> PROLOG VARIABLE TRANSLATION" section.

    ALSO tracks everything needed for ReactiveSequence/ReactiveFallback
    translation: a counter for assigning each one a fresh, unique code;
    the accumulated (code, children_terms) pairs to emit as separate
    reactive_children/2 facts (see _REACTIVE_CONTROL_FLOW's own note);
    and, per blackboard key, EVERY reactive-scope it was touched
    (produced or consumed) under -- a key touched under more than one
    distinct scope (including plain "outside any reactive composite",
    recorded as None) would mean a producer/consumer pair straddles a
    reactive_children/2 boundary, which breaks the SAME way reusing one
    CP across two fallback_node branches already does (see
    basic_action_theory.pl's own "IMPORTANT GOTCHA" note) -- checked
    once, after the whole tree is translated, in translate_tree."""

    def __init__(self):
        self._map = {}
        self.producers = set()   # blackboard keys with an OUTPUT-port producer
        self.consumers = set()   # blackboard keys read by an INPUT port
        self._reactive_counter = 0
        self.reactive_facts = []       # [(code, [child_term, ...]), ...]
        self.key_scopes = {}           # key -> set of codes (None = outside any)

    def var_for(self, key):
        if key not in self._map:
            var = key.upper()
            if not _VALID_PROLOG_VAR_RE.match(var):
                raise BTValidationError(
                    f"Blackboard key '{key}' does not translate to a valid "
                    f"Prolog variable name ('{var}') -- use a plain "
                    f"alphanumeric/underscore key starting with a letter.")
            self._map[key] = var
        return self._map[key]

    def note_key_scope(self, key, reactive_code):
        self.key_scopes.setdefault(key, set()).add(reactive_code)

    def next_reactive_code(self):
        self._reactive_counter += 1
        return f"rc{self._reactive_counter}"


def _validate_ports(tag, elem, port_specs):
    """Raise BTValidationError if a required port is missing or an
    unrecognized attribute is present (other than BT.cpp's own `name`).
    Returns the element's attributes dict for convenience."""
    attrs = dict(elem.attrib)
    unknown = set(attrs) - set(port_specs) - _ALWAYS_ALLOWED_ATTRS
    if unknown:
        raise BTValidationError(
            f"<{tag}> has unrecognized attribute(s) {sorted(unknown)} -- "
            f"not a declared port in schema.yaml for '{tag}'.")
    missing = [name for name, spec in port_specs.items()
               if spec.get("required") and name not in attrs]
    if missing:
        raise BTValidationError(
            f"<{tag}> is missing required port(s) {missing} "
            f"(schema.yaml: module/contracts/schema.yaml's '{tag}' entry).")
    return attrs


def _point_literal(text, tag, port_name):
    x, y = _point_xy(text, tag, port_name)
    return f"point({x},{y})"


def _point_xy(text, tag, port_name):
    """Same "X;Y" parsing as _point_literal, but returns the raw
    (x,y) floats instead of a wrapped point(X,Y) term -- for the rarer
    case (line_of_sight_clear(ObstacleId,GX,GY), notably) where the
    target Prolog predicate takes GX,GY as flat arguments rather than
    a nested point/2 term."""
    parts = text.split(";")
    if len(parts) != 2:
        raise BTValidationError(
            f"<{tag}>'s '{port_name}' port ('{text}') is not a valid Point "
            f"-- expected \"X;Y\", e.g. \"11.675;11.525\".")
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        raise BTValidationError(
            f"<{tag}>'s '{port_name}' port ('{text}') has non-numeric "
            f"X/Y -- expected \"X;Y\" with two floats.")


def _is_battery_trigger(token):
    """True for a Triggers-list token that's battery-related -- the
    bare atom 'battery' or any battery_<whatever>(...) functor
    (battery_below(N), battery_over(N), battery_equal(N), and any
    future one, matched by prefix rather than an enumerated list so a
    new battery_* trigger added later is covered automatically). Used
    to strip battery out of a leg's own Triggers list entirely when
    this problem's own config.yaml sets battery.enabled: false -- see
    that flag's own comment for why "not considering battery in the
    problem" has to happen HERE, at translation time, not just by
    dropping the any_battery_depletion query: a Triggers list is baked
    into plan_generated.pl, a problem-specific generated file, exactly
    like config_generated.pl, so it's the right place for a
    problem-specific flag to take effect, and it's the only way to
    stop battery from ever being able to HALT a walk (as opposed to
    merely not being reported on)."""
    functor = token.split("(", 1)[0].strip()
    return functor == "battery" or functor.startswith("battery_")


_BATTERY_CONDITION_RE = re.compile(r"\bbattery_(below|equal|over)\(")


def _guard_condition_mentions_battery(cond_term):
    """True if cond_term (an auto-derived guard's condition text, e.g.
    "battery_over(70.0)" or "neg(battery_over(70.0))") tests a battery
    threshold -- the SAME "battery.enabled: false means battery can
    never halt a walk" rule _is_battery_trigger enforces for manually-
    typed trigger tokens above, extended to cover auto-derived
    guard_break(Cond,Code) ones, whose OUTER functor is guard_break/
    neg, not battery_*, so _is_battery_trigger's own functor-prefix
    check wouldn't catch it."""
    return _BATTERY_CONDITION_RE.search(cond_term) is not None


def _append_reactive_code(token, reactive_code):
    """token is a Triggers-list entry whose functor is in
    _REACTIVE_TRIGGER_FUNCTORS (e.g. "battery_below(70)" or the bare
    "obstacle_in_bound(0.6)") -- append reactive_code as an extra
    trailing argument (e.g. "battery_below(70,rc3)"). Every one of
    these functors already takes at least one argument (see
    _REACTIVE_TRIGGER_FUNCTORS's own note), so this is always "insert
    before the final close-paren", never "wrap a bare atom in ()"."""
    assert token.endswith(")"), token
    return f"{token[:-1]},{reactive_code})"


def _leaf_condition_term(tag, attrs):
    """The BARE Prolog condition term for a <Condition> leaf (e.g.
    "battery_over(70.0)"), WITHOUT the cond(...) wrapper -- shared by
    _translate_leaf (which wraps it in cond(...) for a genuine, one-
    shot cond() leaf) and _reduce_guard_condition below (which wraps
    it in neg(...) instead, or leaves it bare, depending on the
    required guard polarity). attrs must already be validated (see
    _validate_ports) against tag's own port_specs."""
    info = _CONDITION_DISPATCH[tag]
    if info["kind"] == "single_float_port":
        value = float(attrs[info["port"]])
        return f"{info['functor']}({value})"
    if info["kind"] == "halted_with_cond":
        reason = attrs["reason"].strip()
        return f"halted_with_cond({reason})"
    if info["kind"] == "line_of_sight_clear_cond":
        obstacle_id = attrs["obstacle_id"].strip()
        gx, gy = _point_xy(attrs["goal"], tag, "goal")
        return f"line_of_sight_clear({obstacle_id},{gx},{gy})"
    if info["kind"] == "at_goal_cond":
        gx, gy = _point_xy(attrs["goal"], tag, "goal")
        tolerance = float(attrs["tolerance"])
        return f"at_goal({gx},{gy},{tolerance})"
    raise BTValidationError(f"Unhandled condition kind for <{tag}>.")


def _reduce_guard_condition(elem, required_polarity, schema_ports):
    """A left sibling (under a ReactiveSequence/ReactiveFallback) of
    the branch leading to some reactively-guarded Action, reduced to
    the SINGLE Prolog condition term that must stay TRUE for the guard
    to keep holding -- see the module docstring's CONTROL-FLOW GUARD
    DERIVATION note. required_polarity is what Step 1 (Sequence-shaped
    ancestor -> left siblings must SUCCEED -> True; Fallback-shaped ->
    must FAIL -> False) demands BEFORE accounting for any <Inverter>
    wrapping -- each Inverter layer flips it once, since Inverter(C)
    succeeds iff C fails.

    Returns None (SKIP -- no guard derived, no error) for a left
    sibling that can't be automatically watched: an Action (e.g.
    PlanStraight, a completely ordinary left sibling of a MoveTo -- see
    problem4's own TryGoal branch) or a composite, because once it has
    SUCCEEDED it stays succeeded for the rest of the leg (an already-
    completed action never retroactively fails), so it can never
    supply the SUCCESS->FAILURE / FAILURE->SUCCESS transition a guard
    interrupt needs -- there is nothing WRONG with such a sibling, it
    simply isn't a source of a live interrupt, same as a condition in
    _NON_CONTINUOUS_CONDITIONS (history-based, e.g. HaltedWith -- it
    already gated entry into this branch, once, and can't change mid-
    leg either). Only a (possibly Inverter-wrapped) Condition leaf that
    CAN vary within a leg produces an actual guard term.

    Still raises BTValidationError for a genuinely MALFORMED <Inverter>
    (BT.cpp decorators always take exactly one child) -- a real
    structural bug, independent of guard derivation."""
    tag = elem.tag
    if tag == "Inverter":
        children = list(elem)
        if len(children) != 1:
            raise BTValidationError(
                f"<Inverter> must have exactly one child (found "
                f"{len(children)}).")
        return _reduce_guard_condition(children[0], not required_polarity, schema_ports)
    if tag in _NON_CONTINUOUS_CONDITIONS or tag not in _CONDITION_DISPATCH:
        return None
    attrs = _validate_ports(tag, elem, schema_ports[tag])
    cond_term = _leaf_condition_term(tag, attrs)
    return cond_term if required_polarity else f"neg({cond_term})"


def _translate_leaf(tag, elem, dispatch, port_specs, var_pool, battery_enabled, reactive_code, guard_stack):
    attrs = _validate_ports(tag, elem, port_specs)

    if tag in _ACTION_DISPATCH:
        info = _ACTION_DISPATCH[tag]
        if info["kind"] == "moveto_leg":
            cp_value = attrs["control_points"]
            if not _is_blackboard_ref(cp_value):
                raise BTValidationError(
                    f"<MoveTo>'s control_points port ('{cp_value}') must be "
                    f"a blackboard reference like \"{{cp}}\" -- it is always "
                    f"computed by a planner, never a literal (see this "
                    f"module's own header).")
            key = _blackboard_key(cp_value)
            var_pool.consumers.add(key)
            var_pool.note_key_scope(key, reactive_code)
            cp_var = var_pool.var_for(key)

            # Manual, EXPLICIT extras only -- collision/battery are no
            # longer written here (see below); the port itself is now
            # OPTIONAL (schema.yaml's triggers required: false), so an
            # absent attribute is just "no extras".
            manual_tokens = [t.strip() for t in attrs.get("triggers", "").split(";") if t.strip()]
            if not battery_enabled:
                manual_tokens = [t for t in manual_tokens if not _is_battery_trigger(t)]
            tagged_manual = []
            for t in manual_tokens:
                functor = t.split("(", 1)[0].strip()
                if functor in ("collision", "battery"):
                    # Backward-compatible with older trees that still
                    # spell these out -- covered by default_tokens
                    # below either way, so skip rather than duplicate.
                    continue
                if functor in _REACTIVE_TRIGGER_FUNCTORS:
                    if reactive_code is None:
                        raise BTValidationError(
                            f"<MoveTo>'s triggers port includes '{t}', a "
                            f"reactive-classified trigger, but this MoveTo is "
                            f"not enclosed by any <ReactiveSequence>/"
                            f"<ReactiveFallback> -- there is nowhere for its "
                            f"reactive(_) halt to ever be caught, so it would "
                            f"redescend all the way to the root and be "
                            f"reported as plan_outcome(reactive_escaped) "
                            f"(see basic_action_theory.pl's own CONTROL-FLOW "
                            f"REDESCEND TARGETS note). Wrap this MoveTo (or "
                            f"an ancestor of it) in a ReactiveSequence/"
                            f"ReactiveFallback, or drop '{t}' from triggers.")
                    tagged_manual.append(_append_reactive_code(t, reactive_code))
                else:
                    tagged_manual.append(t)

            # Structural guards, auto-derived from every enclosing
            # ReactiveSequence/ReactiveFallback's own left siblings --
            # see the module docstring's CONTROL-FLOW GUARD DERIVATION
            # note. Each entry in guard_stack is already the exact,
            # polarity-adjusted condition term (see
            # _reduce_guard_condition) paired with the SPECIFIC
            # ancestor level's own code (NOT necessarily the nearest
            # enclosing one -- two nested reactive ancestors can
            # contribute two guards with two DIFFERENT codes here).
            derived_tokens = [
                f"guard_break({cond_term},{code})"
                for cond_term, code in guard_stack
                if battery_enabled or not _guard_condition_mentions_battery(cond_term)
            ]

            # Universal physical hazards -- ALWAYS collision, battery
            # (the fixed 0%-depletion one) only if this problem models
            # battery at all. No longer something the tree author has
            # to write.
            default_tokens = ["collision"] + (["battery"] if battery_enabled else [])

            triggers = "[" + ",".join(default_tokens + tagged_manual + derived_tokens) + "]"
            return f"moveto_leg({cp_var},{triggers})"

        if info["kind"] == "planWith":
            goal_point = _point_literal(attrs["goal"], tag, "goal")
            cp_value = attrs["control_points"]
            if not _is_blackboard_ref(cp_value):
                raise BTValidationError(
                    f"<{tag}>'s control_points port ('{cp_value}') must be "
                    f"a blackboard reference like \"{{cp}}\" -- it is this "
                    f"node's own OUTPUT, never a literal.")
            key = _blackboard_key(cp_value)
            var_pool.producers.add(key)
            var_pool.note_key_scope(key, reactive_code)
            cp_var = var_pool.var_for(key)
            return f"planWith({info['algorithm']},{goal_point},{cp_var})"

        if info["kind"] == "planWith_follow_boarder":
            # obstacle_id is written VERBATIM as Prolog text (a bare
            # atom), same convention as HaltedWith's own reason port
            # below -- NOT quoted, NOT blackboard-ref-checked (nothing
            # in this schema produces obstacle_id as its own port yet;
            # a future producer would need this branch extended the
            # same way MoveTo/PlanAstar's control_points already is).
            obstacle_id = attrs["obstacle_id"].strip()
            offset = float(attrs["offset"])
            cp_value = attrs["control_points"]
            if not _is_blackboard_ref(cp_value):
                raise BTValidationError(
                    f"<{tag}>'s control_points port ('{cp_value}') must be "
                    f"a blackboard reference like \"{{cp}}\" -- it is this "
                    f"node's own OUTPUT, never a literal.")
            key = _blackboard_key(cp_value)
            var_pool.producers.add(key)
            var_pool.note_key_scope(key, reactive_code)
            cp_var = var_pool.var_for(key)
            # planWith/3's Goal slot is part of the SHARED template
            # every planner sits inside (do_node(planWith(Algorithm,
            # Goal,CP),...) in basic_action_theory.pl) -- FollowBoarder
            # itself takes no goal port (see its own schema.yaml entry)
            # and plan_call/8's follow_boarder clauses ignore it
            # outright, so a placeholder point(0.0,0.0) is spliced in
            # here purely to satisfy that shared shape, never read.
            return f"planWith(follow_boarder({obstacle_id},{offset}),point(0.0,0.0),{cp_var})"

    if tag in _CONDITION_DISPATCH:
        return f"cond({_leaf_condition_term(tag, attrs)})"

    raise BTValidationError(f"Unhandled schema entry '{tag}' -- add it to "
                             f"_ACTION_DISPATCH/_CONDITION_DISPATCH.")


def _translate_node(elem, schema_ports, var_pool, battery_enabled, reactive_code, guard_stack):
    tag = elem.tag

    if tag in _CONTROL_FLOW:
        if elem.attrib.keys() - _ALWAYS_ALLOWED_ATTRS:
            raise BTValidationError(
                f"<{tag}> takes no ports of its own (it's a BT.cpp "
                f"built-in control node) -- unexpected attribute(s) "
                f"{sorted(elem.attrib.keys() - _ALWAYS_ALLOWED_ATTRS)}.")
        children = list(elem)
        if not children:
            raise BTValidationError(f"<{tag}> has no children.")
        # Plain Sequence/Fallback -- pass the CURRENT reactive_code AND
        # guard_stack through UNCHANGED. They never catch/redescend on
        # their own, so they don't start a new reactive scope (see
        # basic_action_theory.pl's own CONTROL-FLOW REDESCEND TARGETS
        # note); their own left siblings are ONE-SHOT (checked once on
        # descent, via the cond() already sitting in the tree at that
        # position) rather than live guards, so they contribute nothing
        # to guard_stack either -- see the module docstring's CONTROL-
        # FLOW GUARD DERIVATION note.
        child_terms = [_translate_node(c, schema_ports, var_pool, battery_enabled, reactive_code, guard_stack)
                       for c in children]
        functor = _CONTROL_FLOW[tag]
        return f"{functor}([{','.join(child_terms)}])"

    if tag in _REACTIVE_CONTROL_FLOW:
        if elem.attrib.keys() - _ALWAYS_ALLOWED_ATTRS:
            raise BTValidationError(
                f"<{tag}> takes no ports of its own (it's a BT.cpp "
                f"built-in control node) -- unexpected attribute(s) "
                f"{sorted(elem.attrib.keys() - _ALWAYS_ALLOWED_ATTRS)}.")
        children = list(elem)
        if not children:
            raise BTValidationError(f"<{tag}> has no children.")
        # A NEW reactive scope starts here -- fresh code, and every
        # descendant (until a NESTED ReactiveSequence/ReactiveFallback
        # starts its own) gets tagged with THIS one. Children are
        # translated and then factored OUT into their own
        # reactive_children/2 fact (see _VarPool's own note) rather
        # than inlined -- the returned term references only the code.
        #
        # ALSO: this is exactly where automatic guard derivation
        # happens (see the module docstring's CONTROL-FLOW GUARD
        # DERIVATION note). required_polarity is this level's own
        # baseline: a ReactiveSequence's left siblings must all
        # SUCCEED (True), a ReactiveFallback's must all FAIL (False).
        # For each child index i, every EARLIER sibling (index < i) is
        # reduced to a guard condition and appended to a FRESH
        # guard_stack used ONLY for translating child i -- siblings
        # AFTER i are never guards on it (Step 1), and each child's own
        # guard entries are tagged with THIS level's own_code, added on
        # top of (not replacing) whatever guard_stack already carried
        # in from enclosing levels, so a MoveTo nested under two
        # reactive ancestors accumulates guards -- and codes -- from
        # BOTH. A sibling that _reduce_guard_condition can't turn into
        # a live guard (e.g. an ordinary Action like PlanStraight --
        # see its own note) returns None and is simply skipped, not an
        # error: most Sequence/ReactiveSequence children are actions,
        # not conditions, and that's completely normal.
        own_code = var_pool.next_reactive_code()
        required_polarity = (tag == "ReactiveSequence")
        child_terms = []
        for i, child in enumerate(children):
            own_level_guards = []
            for sibling in children[:i]:
                cond_term = _reduce_guard_condition(sibling, required_polarity, schema_ports)
                if cond_term is not None:
                    own_level_guards.append((cond_term, own_code))
            child_terms.append(_translate_node(
                child, schema_ports, var_pool, battery_enabled, own_code,
                guard_stack + own_level_guards))
        var_pool.reactive_facts.append((own_code, child_terms))
        functor = _REACTIVE_CONTROL_FLOW[tag]
        return f"{functor}({own_code})"

    if tag not in schema_ports:
        raise BTValidationError(
            f"<{tag}> is not a recognized node -- not Sequence/Fallback/"
            f"ReactiveSequence/ReactiveFallback and not an action/condition "
            f"'id' in module/contracts/schema.yaml.")

    return _translate_leaf(tag, elem, None, schema_ports[tag], var_pool, battery_enabled, reactive_code, guard_stack)


def _find_tree_root(xml_root):
    bt_elems = xml_root.findall("BehaviorTree")
    if not bt_elems:
        raise BTValidationError("No <BehaviorTree> element found under <root>.")
    main_id = xml_root.attrib.get("main_tree_to_execute")
    chosen = None
    if main_id:
        for bt in bt_elems:
            if bt.attrib.get("ID") == main_id:
                chosen = bt
                break
        if chosen is None:
            raise BTValidationError(
                f"<root main_tree_to_execute=\"{main_id}\"> but no "
                f"<BehaviorTree ID=\"{main_id}\"> exists.")
    else:
        if len(bt_elems) > 1:
            raise BTValidationError(
                "Multiple <BehaviorTree> elements but no "
                "main_tree_to_execute attribute on <root> to disambiguate.")
        chosen = bt_elems[0]
    children = list(chosen)
    if len(children) != 1:
        raise BTValidationError(
            f"<BehaviorTree ID=\"{chosen.attrib.get('ID')}\"> must have "
            f"EXACTLY ONE root child node (found {len(children)}).")
    return children[0]


def translate_tree(xml_path=DEFAULT_XML_PATH, schema_path=DEFAULT_SCHEMA_PATH,
                    battery_enabled=True):
    """Parse + validate + translate the BT XML into ONE Prolog term
    (Node's own text, e.g. "seq_node([planWith(...),moveto_leg(...)])")
    PLUS the separate reactive_children/2 facts any ReactiveSequence/
    ReactiveFallback in the tree needs -- returns (node_text,
    reactive_facts), where reactive_facts is [(code, [child_term,...]),
    ...]. Raises BTValidationError on any structural problem.

    battery_enabled=False strips every battery-related trigger name
    (battery, battery_below(...), battery_over(...), battery_equal(...)
    -- see _is_battery_trigger's own note) out of every MoveTo's own
    Triggers list -- the problem's own config.yaml battery.enabled
    flag, threaded in by generate_plan_pl's caller (main.py/
    diagnose_pipeline.py)."""
    schema = load_schema(schema_path)
    schema_ports = _schema_port_index(schema)

    try:
        xml_root = ET.parse(xml_path).getroot()
    except ET.ParseError as e:
        raise BTValidationError(f"Malformed XML in {xml_path}: {e}")

    tree_root_elem = _find_tree_root(xml_root)
    var_pool = _VarPool()
    node_text = _translate_node(tree_root_elem, schema_ports, var_pool, battery_enabled, None, [])

    # A MoveTo whose control_points key has no PlanAstar/PlanStraight
    # producer anywhere in the tree would silently leave CP unbound --
    # catch it here rather than let ProbLog fail confusingly later.
    dangling = var_pool.consumers - var_pool.producers
    if dangling:
        raise BTValidationError(
            f"control_points blackboard key(s) {sorted(dangling)} are read "
            f"by a MoveTo node but never produced by any PlanAstar/"
            f"PlanStraight node in the tree -- CP would be unbound.")

    # A blackboard key produced/consumed under more than one reactive
    # scope (including "outside any ReactiveSequence/ReactiveFallback",
    # recorded as None) would straddle a reactive_children/2 boundary --
    # see _VarPool's own note on why that silently breaks the same way
    # reusing one CP across two fallback_node branches already does.
    straddling = {key: scopes for key, scopes in var_pool.key_scopes.items()
                  if len(scopes) > 1}
    if straddling:
        def _label(scope):
            return scope or "outside any reactive composite"
        details = "; ".join(
            f"'{key}' touched under {sorted(_label(s) for s in scopes)}"
            for key, scopes in straddling.items())
        raise BTValidationError(
            f"control_points blackboard key(s) cross a ReactiveSequence/"
            f"ReactiveFallback boundary -- a producer/consumer pair must "
            f"live ENTIRELY inside the same reactive composite (or entirely "
            f"outside all of them): {details}.")

    return node_text, var_pool.reactive_facts


def generate_plan_pl(xml_path=DEFAULT_XML_PATH, schema_path=DEFAULT_SCHEMA_PATH,
                      output_path=DEFAULT_OUTPUT_PATH, battery_enabled=True):
    node_text, reactive_facts = translate_tree(xml_path, schema_path, battery_enabled=battery_enabled)
    lines = [
        "% AUTO-GENERATED by module/translators/bt_to_prolog.py from",
        f"% {os.path.relpath(xml_path, os.path.dirname(output_path))} -- DO NOT HAND-EDIT,",
        "% edit the XML tree instead and regenerate (main.py does this",
        "% automatically before every run).",
    ]
    if not battery_enabled:
        lines += [
            "%",
            "% battery.enabled: false in this problem's own config.yaml --",
            "% every battery-related trigger name has been stripped from",
            "% every leg's own Triggers list below (see bt_to_prolog.py's",
            "% own _is_battery_trigger).",
        ]
    lines += [
        "",
        f"plan({node_text}).",
        "",
    ]
    if reactive_facts:
        lines += [
            "% One reactive_children/2 fact per <ReactiveSequence>/",
            "% <ReactiveFallback> in the tree, keyed by the same code",
            "% embedded in plan/1's own reactivesequence(Code)/",
            "% reactivefallback(Code) markers above -- see basic_action_",
            "% theory.pl's own CONTROL-FLOW REDESCEND TARGETS note for why",
            "% these live as SEPARATE facts rather than being inlined.",
            "",
        ]
        for code, child_terms in reactive_facts:
            lines.append(f"reactive_children({code}, [{','.join(child_terms)}]).")
        lines.append("")
    with open(output_path, "w") as f:
        f.write("\n".join(lines))
    return output_path


if __name__ == "__main__":
    out = generate_plan_pl()
    print(f"Wrote {out}")
