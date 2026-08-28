#!/usr/bin/env python3
"""
plan_generation/bt_to_prolog.py

The BT.cpp XML tree -> Prolog do_node term translator flagged as
"not yet built" in this project's own history. Reads a real
BehaviorTree.cpp v4 XML tree (plan_generation/plan/behavior_tree.xml),
validates it against actions/schema.yaml (every leaf node must be a
known, correctly-instantiated schema action/condition; every
control-flow node must be Sequence or Fallback -- the two BT.cpp
built-ins moveto_continuous.pl's seq_node/fallback_node already map to
1:1), and translates it into the nested do_node/4 term text
moveto_continuous.pl's plan/1 expects.

WHERE THE RESULT GOES: generate_plan_pl() writes
plan_generation/plan/plan_generated.pl, a single plan/1 FACT (not a
clause with a body -- see below), which moveto_continuous.pl now
:- consult()s instead of hand-defining plan/1 itself. This mirrors the
existing generated-artifact pattern (config/config_generated.pl,
environments/maps/obstacles_generated.pl,
plan_generation/plan/current_plan.pl): the XML is the single source of
truth for the POLICY'S SHAPE from now on -- change the tree by editing
the XML and re-running, not by hand-editing moveto_continuous.pl.
run_plan_continuous_safety.py regenerates it automatically before every
run, exactly like it already does for config_generated.pl.

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
documented in moveto_continuous.pl for hand-written multi-leg plans.
Never reuse one key across two DIFFERENT PlanAstar/PlanStraight calls
that should compute independent paths -- see moveto_continuous.pl's own
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
(same reasoning as config/generate_prolog_config.py's own hard
requirements, as opposed to its two non-fatal numeric warnings).
"""
import os
import re
import xml.etree.ElementTree as ET

import yaml

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_XML_PATH = os.path.join(_THIS_DIR, "plan", "behavior_tree.xml")
DEFAULT_SCHEMA_PATH = os.path.join(os.path.dirname(_THIS_DIR), "actions", "schema.yaml")
DEFAULT_OUTPUT_PATH = os.path.join(_THIS_DIR, "plan", "plan_generated.pl")

# Every schema action's `id` maps to how it's dispatched below: which
# do_node/4 Prolog functor it becomes, and (for the two planners) which
# Algorithm atom plan_call/8 should dispatch on.
_ACTION_DISPATCH = {
    "MoveTo": {"kind": "moveto_leg"},
    "PlanAstar": {"kind": "planWith", "algorithm": "astar"},
    "PlanStraight": {"kind": "planWith", "algorithm": "straight"},
}
# "single_float_port": the shared shape of every cond(Functor(Value))
# condition whose one port is a plain float -- AtGoal, ObstacleInBound,
# and BatteryBelow all reduce to this, just with different functor/port
# names, so they share ONE translation branch below instead of three
# near-identical ones.
_CONDITION_DISPATCH = {
    "AtGoal": {"kind": "single_float_port", "functor": "at_goal", "port": "tolerance"},
    "ObstacleInBound": {"kind": "single_float_port", "functor": "obstacle_in_bound", "port": "threshold"},
    "BatteryBelow": {"kind": "single_float_port", "functor": "battery_below", "port": "threshold"},
    "BatteryEqual": {"kind": "single_float_port", "functor": "battery_equal", "port": "threshold"},
    "BatteryOver": {"kind": "single_float_port", "functor": "battery_over", "port": "threshold"},
    "HaltedWith": {"kind": "halted_with_cond"},
}
_CONTROL_FLOW = {"Sequence": "seq_node", "Fallback": "fallback_node"}

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
    docstring's "BLACKBOARD -> PROLOG VARIABLE TRANSLATION" section."""

    def __init__(self):
        self._map = {}
        self.producers = set()   # blackboard keys with an OUTPUT-port producer
        self.consumers = set()   # blackboard keys read by an INPUT port

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
            f"(schema.yaml: actions/schema.yaml's '{tag}' entry).")
    return attrs


def _point_literal(text, tag, port_name):
    parts = text.split(";")
    if len(parts) != 2:
        raise BTValidationError(
            f"<{tag}>'s '{port_name}' port ('{text}') is not a valid Point "
            f"-- expected \"X;Y\", e.g. \"11.675;11.525\".")
    try:
        x, y = float(parts[0]), float(parts[1])
    except ValueError:
        raise BTValidationError(
            f"<{tag}>'s '{port_name}' port ('{text}') has non-numeric "
            f"X/Y -- expected \"X;Y\" with two floats.")
    return f"point({x},{y})"


def _string_list_literal(text):
    items = [t.strip() for t in text.split(";") if t.strip()]
    return "[" + ",".join(items) + "]"


def _translate_leaf(tag, elem, dispatch, port_specs, var_pool):
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
            cp_var = var_pool.var_for(key)
            triggers = _string_list_literal(attrs["triggers"])
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
            cp_var = var_pool.var_for(key)
            return f"planWith({info['algorithm']},{goal_point},{cp_var})"

    if tag in _CONDITION_DISPATCH:
        info = _CONDITION_DISPATCH[tag]
        if info["kind"] == "single_float_port":
            value = float(attrs[info["port"]])
            return f"cond({info['functor']}({value}))"
        if info["kind"] == "halted_with_cond":
            reason = attrs["reason"].strip()
            return f"cond(halted_with_cond({reason}))"

    raise BTValidationError(f"Unhandled schema entry '{tag}' -- add it to "
                             f"_ACTION_DISPATCH/_CONDITION_DISPATCH.")


def _translate_node(elem, schema_ports, var_pool):
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
        child_terms = [_translate_node(c, schema_ports, var_pool) for c in children]
        functor = _CONTROL_FLOW[tag]
        return f"{functor}([{','.join(child_terms)}])"

    if tag not in schema_ports:
        raise BTValidationError(
            f"<{tag}> is not a recognized node -- not Sequence/Fallback and "
            f"not an action/condition 'id' in actions/schema.yaml.")

    return _translate_leaf(tag, elem, None, schema_ports[tag], var_pool)


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


def translate_tree(xml_path=DEFAULT_XML_PATH, schema_path=DEFAULT_SCHEMA_PATH):
    """Parse + validate + translate the BT XML into ONE Prolog term
    (Node's own text, e.g. "seq_node([planWith(...),moveto_leg(...)])").
    Raises BTValidationError on any structural problem."""
    schema = load_schema(schema_path)
    schema_ports = _schema_port_index(schema)

    try:
        xml_root = ET.parse(xml_path).getroot()
    except ET.ParseError as e:
        raise BTValidationError(f"Malformed XML in {xml_path}: {e}")

    tree_root_elem = _find_tree_root(xml_root)
    var_pool = _VarPool()
    node_text = _translate_node(tree_root_elem, schema_ports, var_pool)

    # A MoveTo whose control_points key has no PlanAstar/PlanStraight
    # producer anywhere in the tree would silently leave CP unbound --
    # catch it here rather than let ProbLog fail confusingly later.
    dangling = var_pool.consumers - var_pool.producers
    if dangling:
        raise BTValidationError(
            f"control_points blackboard key(s) {sorted(dangling)} are read "
            f"by a MoveTo node but never produced by any PlanAstar/"
            f"PlanStraight node in the tree -- CP would be unbound.")

    return node_text


def generate_plan_pl(xml_path=DEFAULT_XML_PATH, schema_path=DEFAULT_SCHEMA_PATH,
                      output_path=DEFAULT_OUTPUT_PATH):
    node_text = translate_tree(xml_path, schema_path)
    lines = [
        "% AUTO-GENERATED by plan_generation/bt_to_prolog.py from",
        f"% {os.path.relpath(xml_path, os.path.dirname(output_path))} -- DO NOT HAND-EDIT,",
        "% edit the XML tree instead and regenerate (run_plan_continuous_safety.py",
        "% does this automatically before every run).",
        "",
        f"plan({node_text}).",
        "",
    ]
    with open(output_path, "w") as f:
        f.write("\n".join(lines))
    return output_path


if __name__ == "__main__":
    out = generate_plan_pl()
    print(f"Wrote {out}")
