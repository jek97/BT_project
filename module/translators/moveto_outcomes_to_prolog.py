"""
moveto_outcomes_to_prolog.py

Offline orchestrator for the simplified branch's moveto_outcome/7 table
(see basic_action_theory.pl Section 2 for the interface, and
moveto_calibrator.py for calibrate_moveto() itself, the per-(leg,
incoming-branch) function this orchestrator drives).

WHY THIS EXISTS, not a repeat of the abandoned in-theory guard (see
basic_action_theory.pl Section 3's own note, and this branch's git
history around commit 0812e1f): an earlier attempt tried to catch a
missing moveto_outcome row INSIDE ProbLog, via negation-as-failure. It
false-positived, because ProbLog's own grounder explores every
(LegId,InBranch) combination reachable by unification while building
its formula -- including ones no resolved world can actually take --
not only the combinations the plan's own Fallback/Sequence semantics
make logically reachable. This module sidesteps that entirely: it
determines the reachable set itself, by walking the plan tree using
the PLAN'S OWN LOGICAL RULES (mirroring do_node/seq_node/fallback_node/
evaluate_plan exactly, in Python, over SYMBOLIC branch outcomes) rather
than by watching what ProbLog happens to explore. The set this produces
is the minimal correct one -- for problem4, it produces exactly the
rows a resolved world can actually reach, not the "structurally
possible but always zero-weight" ones the guard's own stub needed to
placate ProbLog's grounder.

ALGORITHM: a breadth-first search over REDESCEND LEVELS (evaluate_plan's
own Budget-bounded recursion), where each level is a full depth-first
evaluation of the tree's own Sequence/Fallback/Cond/MoveTo composition
rules over a SET of weighted symbolic states (not one concrete resolved
world) -- see eval_node()'s own docstring for the exact correspondence
to do_node/4's four clauses. A MoveTo leaf is where the branching
actually happens: calibrate_moveto() is called AT MOST ONCE per
distinct (LegId, InBranch) pair (memoized in `cache`), returning a
small set of weighted outcome branches, each of which becomes its own
line of continued evaluation. This is the SAME cardinality argument
made when this design was first proposed: one calibration call per
(leg, incoming-branch) pair, not per resolved world.

VALIDATION: this module also sums P(true)/P(false)/P(world_too_large)
across the whole BFS, in Python, entirely independently of ProbLog's
own weighted model counting -- these should match plan_outcome(true)/
plan_outcome(false)/plan_outcome(world_too_large)'s own probabilities
once basic_action_theory.pl is run against the generated table. Use
this as a consistency check (see main() below), not just a courtesy
print.
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "theory"))
from moveto_calibrator import calibrate_moveto  # noqa: E402
from config_to_prolog import load_config  # noqa: E402
from collision_geometry import _parse_obstacle_polygons, _strip_prolog_comments  # noqa: E402


# =====================================================================
# TREE REPRESENTATION -- parsed from behavior_tree.xml's simplified-
# branch shape (Fallback/Sequence built-ins, BatteryOver condition,
# unified MoveTo(leg_id,algorithm,goal) action -- see schema.yaml's
# own MoveToSimplified entry). Deliberately minimal: only the node
# types the simplified theory currently uses, not a general BT.cpp
# parser (bt_to_prolog.py already is one, for the pre-simplification
# shape -- this is not meant to replace it, only to serve this
# orchestrator until bt_to_prolog.py is updated to emit the new shape
# itself, see FUTUREWORK.md).
# =====================================================================
@dataclass(frozen=True)
class CondNode:
    kind: str        # e.g. "BatteryOver"
    threshold: float


@dataclass(frozen=True)
class MoveToNode:
    leg_id: str
    algorithm: str
    goal: tuple       # (x, y)


@dataclass(frozen=True)
class SequenceNode:
    children: tuple


@dataclass(frozen=True)
class FallbackNode:
    children: tuple


def _parse_point(s):
    x_str, y_str = s.split(";")
    return (float(x_str), float(y_str))


def _parse_node(elem):
    tag = elem.tag
    if tag == "Sequence":
        return SequenceNode(tuple(_parse_node(c) for c in elem))
    if tag == "Fallback":
        return FallbackNode(tuple(_parse_node(c) for c in elem))
    if tag == "BatteryOver":
        return CondNode("BatteryOver", float(elem.attrib["threshold"]))
    if tag == "MoveTo":
        return MoveToNode(
            elem.attrib["leg_id"],
            elem.attrib["algorithm"],
            _parse_point(elem.attrib["goal"]),
        )
    raise ValueError(
        f"moveto_outcomes_to_prolog.py's minimal parser doesn't know "
        f"node type <{tag}> -- only Sequence/Fallback/BatteryOver/MoveTo "
        f"are supported (the simplified-branch tree shape); if this is "
        f"a genuinely new node type, both this parser and eval_node() "
        f"below need a matching clause, the same way basic_action_theory.pl's "
        f"do_node/4 would need a new clause for it.")


def parse_tree(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    main_tree_id = root.attrib["main_tree_to_execute"]
    for bt in root.findall("BehaviorTree"):
        if bt.attrib.get("ID") == main_tree_id:
            children = list(bt)
            if len(children) != 1:
                raise ValueError(
                    f"BehaviorTree '{main_tree_id}' must have exactly one "
                    f"root child, found {len(children)}")
            return _parse_node(children[0])
    raise ValueError(f"No BehaviorTree with ID='{main_tree_id}' found in {xml_path}")


# =====================================================================
# SYMBOLIC EVALUATION -- eval_node(node, state, cache, config) mirrors
# do_node(Node,S,S1,Status) EXACTLY, clause for clause, except a leaf
# can now return SEVERAL (status, new_state, probability) rows instead
# of committing to one -- calibrate_moveto's own outcome branches.
# Every composite (Sequence/Fallback) just distributes over whichever
# branches its children produced, multiplying probabilities along the
# way, the same way independent probabilistic choices compose.
# =====================================================================
@dataclass(frozen=True)
class State:
    position: tuple    # (x, y)
    battery: float
    last_branch: str    # 'root' or a BranchId -- incoming_branch/2's own key


def outcome_status(reason):
    """Mirrors outcome_status/2 in basic_action_theory.pl exactly:
    success->true, crashed(_)/battery_depleted->false, everything else
    (any reactive trigger name)->reactive."""
    if reason == "success":
        return "true"
    if reason == "battery_depleted" or (isinstance(reason, tuple) and reason[0] == "crashed"):
        return "false"
    return "reactive"


def eval_node(node, state, cache, config, obstacle_polygons):
    """Returns a list of (status, new_state, probability) triples.
    Probabilities are CONDITIONAL on having reached `state` -- the
    caller (a composite node, or the top-level BFS) is responsible for
    multiplying by the probability of reaching `state` in the first
    place, exactly like chaining independent probabilistic choices."""
    if isinstance(node, CondNode):
        if node.kind == "BatteryOver":
            status = "true" if state.battery > node.threshold else "false"
            return [(status, state, 1.0)]
        raise ValueError(f"Unknown condition kind {node.kind!r}")

    if isinstance(node, MoveToNode):
        key = (node.leg_id, state.last_branch)
        if key not in cache:
            cache[key] = calibrate_moveto(
                node.leg_id, state.last_branch, state.position, state.battery,
                node.goal, node.algorithm, [], config, obstacle_polygons)
        results = []
        for row in cache[key]:
            status = outcome_status(row["reason"])
            new_state = State(
                position=row["end_point"],
                battery=state.battery - row["drain"],
                last_branch=row["branch_id"],
            )
            results.append((status, new_state, row["probability"]))
        return results

    if isinstance(node, SequenceNode):
        if not node.children:
            return [("true", state, 1.0)]
        first, rest = node.children[0], SequenceNode(node.children[1:])
        results = []
        for status, s2, p in eval_node(first, state, cache, config, obstacle_polygons):
            if status == "true":
                for status2, s3, p2 in eval_node(rest, s2, cache, config, obstacle_polygons):
                    results.append((status2, s3, p * p2))
            else:  # false or reactive propagate straight through, unchanged
                results.append((status, s2, p))
        return results

    if isinstance(node, FallbackNode):
        if not node.children:
            return [("false", state, 1.0)]
        first, rest = node.children[0], FallbackNode(node.children[1:])
        results = []
        for status, s2, p in eval_node(first, state, cache, config, obstacle_polygons):
            if status == "false":
                for status2, s3, p2 in eval_node(rest, s2, cache, config, obstacle_polygons):
                    results.append((status2, s3, p * p2))
            else:  # true or reactive propagate straight through, unchanged
                results.append((status, s2, p))
        return results

    raise ValueError(f"Unknown node type {type(node)}")


# =====================================================================
# TOP-LEVEL BFS -- mirrors evaluate_plan/4's own redescend recursion,
# bounded by `budget` (== replan_budget/1). See this file's own header
# for the hop-numbering / world_too_large correspondence.
# =====================================================================
def enumerate_and_calibrate(tree, start, battery_start, budget, config, obstacle_polygons=()):
    cache = {}
    frontier = [(State(start, battery_start, "root"), 1.0)]
    hop = 0
    p_true = p_false = p_world_too_large = 0.0
    while frontier:
        next_frontier = []
        for state, prob in frontier:
            for status, new_state, p in eval_node(tree, state, cache, config, obstacle_polygons):
                total_p = prob * p
                if status == "true":
                    p_true += total_p
                elif status == "false":
                    p_false += total_p
                elif status == "reactive":
                    if hop < budget:
                        next_frontier.append((new_state, total_p))
                    else:
                        p_world_too_large += total_p
        frontier = next_frontier
        hop += 1
    return cache, p_true, p_false, p_world_too_large


# =====================================================================
# EMIT moveto_outcome/7 FACTS
# =====================================================================
def _format_number(x):
    if isinstance(x, float) and x == int(x):
        return f"{x:.1f}"
    return repr(x)


def _format_reason(reason):
    if isinstance(reason, tuple):
        functor, *args = reason
        return f"{functor}({','.join(_format_number(a) if isinstance(a, (int, float)) else str(a) for a in args)})"
    return str(reason)


def render_prolog(cache):
    lines = [
        "% AUTO-GENERATED by module/translators/moveto_outcomes_to_prolog.py",
        "% -- DO NOT HAND-EDIT, edit behavior_tree.xml/config.yaml and",
        "% regenerate instead. See basic_action_theory.pl Section 2 for the",
        "% moveto_outcome/7 interface this file implements, and that",
        "% module's own header for how (LegId,InBranch) pairs were",
        "% determined (a symbolic BFS over the plan's own Fallback/Sequence",
        "% semantics, NOT observed from ProbLog's grounder).",
        "",
    ]
    for (leg_id, in_branch), rows in cache.items():
        clauses = []
        for row in rows:
            ex, ey = row["end_point"]
            clauses.append(
                f"{row['probability']:.12g}::moveto_outcome({leg_id}, {in_branch}, "
                f"{row['branch_id']}, {_format_reason(row['reason'])}, "
                f"point({_format_number(ex)},{_format_number(ey)}), "
                f"{_format_number(row['duration'])}, {_format_number(row['drain'])})"
            )
        lines.append(" ;\n".join(clauses) + ".")
        lines.append("")
    return "\n".join(lines)


def _load_obstacle_polygons(problem_dir):
    obstacles_path = os.path.join(problem_dir, "obstacles_generated.pl")
    try:
        with open(obstacles_path) as f:
            return _parse_obstacle_polygons(_strip_prolog_comments(f.read()))
    except FileNotFoundError:
        return []


def generate(problem_dir, budget):
    config = load_config(os.path.join(problem_dir, "config.yaml"))
    tree = parse_tree(os.path.join(problem_dir, "behavior_tree.xml"))
    obstacle_polygons = _load_obstacle_polygons(problem_dir)
    start = (config["initial_situation"]["start_x"], config["initial_situation"]["start_y"])
    battery_start = config["battery"]["start"]

    cache, p_true, p_false, p_world_too_large = enumerate_and_calibrate(
        tree, start, battery_start, budget, config, obstacle_polygons)

    output_path = os.path.join(problem_dir, "moveto_outcomes_generated.pl")
    with open(output_path, "w") as f:
        f.write(render_prolog(cache))

    return {
        "output_path": output_path,
        "pairs_calibrated": len(cache),
        "p_true": p_true,
        "p_false": p_false,
        "p_world_too_large": p_world_too_large,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--problem-dir", required=True)
    ap.add_argument("--budget", type=int, default=1000, help="replan_budget to enumerate for (default 1000)")
    args = ap.parse_args()
    result = generate(args.problem_dir, args.budget)
    print(f"Wrote {result['output_path']} ({result['pairs_calibrated']} (leg,in_branch) pairs calibrated)")
    print(f"Python-side BFS totals: P(true)={result['p_true']:.6g} "
          f"P(false)={result['p_false']:.6g} "
          f"P(world_too_large)={result['p_world_too_large']:.6g}")


if __name__ == "__main__":
    main()
