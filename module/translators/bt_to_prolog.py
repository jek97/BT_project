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


def _translate_leaf(tag, elem, dispatch, port_specs, var_pool, battery_enabled, reactive_code):
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
            trigger_tokens = [t.strip() for t in attrs["triggers"].split(";") if t.strip()]
            if not battery_enabled:
                trigger_tokens = [t for t in trigger_tokens if not _is_battery_trigger(t)]
            tagged_tokens = []
            for t in trigger_tokens:
                functor = t.split("(", 1)[0].strip()
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
                    tagged_tokens.append(_append_reactive_code(t, reactive_code))
                else:
                    tagged_tokens.append(t)
            triggers = "[" + ",".join(tagged_tokens) + "]"
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
        info = _CONDITION_DISPATCH[tag]
        if info["kind"] == "single_float_port":
            value = float(attrs[info["port"]])
            return f"cond({info['functor']}({value}))"
        if info["kind"] == "halted_with_cond":
            reason = attrs["reason"].strip()
            return f"cond(halted_with_cond({reason}))"
        if info["kind"] == "line_of_sight_clear_cond":
            obstacle_id = attrs["obstacle_id"].strip()
            gx, gy = _point_xy(attrs["goal"], tag, "goal")
            return f"cond(line_of_sight_clear({obstacle_id},{gx},{gy}))"
        if info["kind"] == "at_goal_cond":
            gx, gy = _point_xy(attrs["goal"], tag, "goal")
            tolerance = float(attrs["tolerance"])
            return f"cond(at_goal({gx},{gy},{tolerance}))"

    raise BTValidationError(f"Unhandled schema entry '{tag}' -- add it to "
                             f"_ACTION_DISPATCH/_CONDITION_DISPATCH.")


def _translate_node(elem, schema_ports, var_pool, battery_enabled, reactive_code):
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
        # Plain Sequence/Fallback -- pass the CURRENT reactive_code
        # through UNCHANGED (they never catch/redescend on their own,
        # so they don't start a new reactive scope -- see
        # basic_action_theory.pl's own CONTROL-FLOW REDESCEND TARGETS
        # note).
        child_terms = [_translate_node(c, schema_ports, var_pool, battery_enabled, reactive_code)
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
        own_code = var_pool.next_reactive_code()
        child_terms = [_translate_node(c, schema_ports, var_pool, battery_enabled, own_code)
                       for c in children]
        var_pool.reactive_facts.append((own_code, child_terms))
        functor = _REACTIVE_CONTROL_FLOW[tag]
        return f"{functor}({own_code})"

    if tag not in schema_ports:
        raise BTValidationError(
            f"<{tag}> is not a recognized node -- not Sequence/Fallback/"
            f"ReactiveSequence/ReactiveFallback and not an action/condition "
            f"'id' in module/contracts/schema.yaml.")

    return _translate_leaf(tag, elem, None, schema_ports[tag], var_pool, battery_enabled, reactive_code)


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
    node_text = _translate_node(tree_root_elem, schema_ports, var_pool, battery_enabled, None)

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
