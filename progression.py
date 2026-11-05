#!/usr/bin/env python3
"""
progression.py

Forward (progression) evaluation of a problem's BT, as an alternative to
main.py's single regressed ProbLog query. Every noise table is finite and
each step is deterministic once its draws are fixed, so the belief state
is a finite set of weighted configurations. This script expands them leaf
by leaf until every branch has finished:

    configuration = progressed state + what is left of the BT
    b'(step(c, d)) += b(c) * P(d)     for each joint draw d of the leaf

WHERE EACH PART LIVES:
  - module/theory/progression.pl runs ONE leaf for fixed draws, through
    the ordinary axioms of basic_action_theory.pl, and reads the new
    state. ProbLog is used there as a plain Prolog engine
    (DefaultEngine.query): no grounding and no knowledge compilation.
  - this file owns the probabilities and the BT stack. advance() is the
    small-step version of do_node/4's composite clauses (seq_step,
    fallback_step, invert, reactive_after) and must stay in sync with
    them.

zbatt/1 is a single mission-wide draw, so it is split once at the root
and every configuration stays tied to its value. Every other table is
drawn afresh per leaf, as in the theory, where it is keyed on the
situation.

MERGING (--merge):
  none  two configurations merge only if their full history is equal.
        One final state per path, comparable with outcome_signature/1.
  auto  history is compared without the position and value stored in
        sample reasons, and without exact arrivals, unless the goal or a
        condition can read them (needs_arrivals/needs_sample_detail
        below). Merging happens at the start of a walk, once the merge
        grids have rounded T0 and B0 (walk_seed/3). outcome_signature/1
        rows and sample detail rows are not reported in this mode: they
        name the exact values that were merged away.

OUTPUT: output_progression/<problem>/<problem>_<merge>_summary.json
        (probabilities and stats), ..._report.log (the same goal, safety
        and condition tables main.py prints, also shown on the console)
        and ..._graph.json (the execution DAG: nodes are configurations,
        edges are leaves with their conditional probability, and reach
        is computed afterwards in one topological pass -- draw it with
        visualize_progression.py).

    python3 progression.py --problem problem0S --merge auto
    python3 progression.py --problem all
"""

import argparse
import itertools
import json
import os
import re
import subprocess
import sys
import time
from collections import defaultdict, deque

from problog.engine import DefaultEngine
from problog.logic import Or, Term, Var, list2term, term2list, term2str
from problog.program import PrologFile

import main as pipeline

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_ROOT = os.path.join(ROOT_DIR, "output_progression")
PROGRESSION_THEORY = os.path.join(pipeline.THEORY_DIR, "progression.pl")

# Which noise tables each leaf consumes. sample_value is drawn on failure
# too and ignored there: those branches lead to one child, so the edge
# just sums the weights.
LEAF_NOISE = {
    "moveto_leg": ("z", "zt"),
    "take_sample": ("sample_result", "sample_value"),
    "install_tool_leg": ("install_tool_result",),
    "uninstall_tool_leg": ("uninstall_tool_result",),
    "deploy_tool_leg": ("deploy_tool_result",),
    "retract_tool_leg": ("retract_tool_result",),
}

# Triggers that never read the position or history before the leg's
# start, so a leg using only these can be reduced to walk_seed/3.
SEED_SAFE_TRIGGERS = {"collision", "battery"}

# These read the last walk (current_walk/5) after it has ended, which
# st(...) does not store. Better to refuse than to fail silently.
UNSUPPORTED_READS = ("obstacle_on_path", "on_track", "first_hit", "hit_by",
                     "verify_safe", "plan_route_blocked")

_FRESH_VAR = re.compile(r"\bBB\d+\b")


class Bindings(dict):
    """Substitution for Term.apply, which looks every variable up by
    name: the ones not bound here stay as they are."""

    def __missing__(self, name):
        return Var(name)


class Unsupported(Exception):
    """The problem reads something progression does not carry."""


def _strip_comments(text):
    return "\n".join(line.split("%", 1)[0] for line in text.splitlines())


def load_noise_tables(config_path):
    """{name: (arity, [(value_term, weight), ...])} for every annotated
    disjunction in config_generated.pl."""
    tables = {}
    for statement in PrologFile(config_path):
        if isinstance(statement, Or):
            heads = statement.to_list()
        elif isinstance(statement, Term) and statement.probability is not None:
            heads = [statement]
        else:
            continue
        name = heads[0].functor
        entries = [(head.args[-1], float(head.probability)) for head in heads]
        total = sum(weight for _, weight in entries)
        if abs(total - 1.0) > 1e-9:
            raise ValueError(f"noise table {name} sums to {total}, not 1")
        tables[name] = (heads[0].arity, entries)
    return tables


def write_noise_free_config(config_path, output_path, tables):
    """A copy of config_generated.pl with every annotated disjunction
    replaced by a fixed_draw/3 reader. The weights stay in Python."""
    kept, in_table = [], False
    with open(config_path) as f:
        for line in f:
            if not in_table and re.match(r"\s*[0-9.eE+-]+::", line):
                in_table = True
            if in_table:
                in_table = not line.rstrip().endswith(".")
                continue
            kept.append(line)

    with open(output_path, "w") as f:
        f.write("% AUTO-GENERATED by progression.py from config_generated.pl.\n")
        f.writelines(kept)
        f.write("\n")
        for name, (arity, _entries) in sorted(tables.items()):
            if name == "zbatt":
                continue    # read through current_zb/1, see progression.pl
            keys = [f"K{i}" for i in range(arity - 1)]
            head = ",".join(keys + ["V"])
            f.write(f"{name}({head}) :- fixed_draw(K0, {name}, V).\n")


def write_bootstrap(problem_dir, config_path, goal_path, run_label):
    with open(os.path.join(pipeline.THEORY_DIR, "problem_data.pl"), "w") as f:
        f.write(f"% AUTO-GENERATED by progression.py ({run_label}).\n\n"
                f":- consult('{config_path}').\n"
                f":- consult('{os.path.join(problem_dir, 'plan_generated.pl')}').\n"
                f":- consult('{goal_path}').\n")


def fresh_key(text):
    """Blackboard variables renamed by first occurrence, so two copies of
    the same subtree give the same key."""
    names = {}
    return _FRESH_VAR.sub(lambda m: names.setdefault(m.group(0), f"V{len(names)}"), text)


def list_of(term):
    return list(term2list(term, deep=False))


class Node:
    __slots__ = ("id", "zb", "phys", "mem", "work", "stack", "display")

    def __init__(self, node_id, zb, phys, mem, work, stack, display):
        self.id = node_id
        self.zb = zb
        self.phys = phys
        self.mem = mem
        self.work = work        # ("leaf", term) or ("final", outcome string)
        self.stack = stack
        self.display = display  # phys before walk_seed/3, for the graph


class Progression:
    def __init__(self, problem_dir, merge, tables, goal_text, plan_text):
        self.merge = merge
        self.tables = tables
        self.engine = DefaultEngine()
        self.base_db = self.engine.prepare(PrologFile(PROGRESSION_THEORY))
        self.dbs = {}

        checked = _strip_comments(goal_text) + _strip_comments(plan_text)
        for name in UNSUPPORTED_READS:
            if re.search(rf"\b{name}\b", checked):
                raise Unsupported(f"{name} reads the last walk after it ended, "
                                  f"which the progressed state does not store")
        self.needs_arrivals = bool(re.search(r"\bvisited\b", checked))
        self.needs_sample_detail = bool(
            re.search(r"\bsample_success_at\b|\bsample_value_", checked)
            or re.search(r"sample_success\(\s*(?!_,\s*_,\s*_)", checked)
            or re.search(r"sample_failure\(\s*(?!_,\s*_)", checked))

        self.replan_budget = int(self._one("replan_budget", None)[0])
        self.var_counter = itertools.count()
        self.nodes = {}             # merge key -> Node
        self.by_id = {}
        self.edges = {}             # (source, target, action) -> edge dict
        self.next_id = itertools.count(1)
        self.leaf_steps = 0

    # -- engine helpers ------------------------------------------------
    def _query(self, functor, *args, db=None):
        return self.engine.query(db if db is not None else self.base_db,
                                 Term(functor, *args))

    def _one(self, functor, *args, db=None):
        answers = self._query(functor, *args, db=db)
        if len(answers) != 1:
            raise RuntimeError(f"{functor}: expected 1 answer, got {len(answers)}")
        return answers[0]

    def _fresh(self, term):
        # Engine answers hold unbound variables as plain negative ints.
        mapping = {var: Var(f"BB{next(self.var_counter)}") for var in term.variables()}
        return term.apply(mapping) if mapping else term

    # -- BT stack: the small-step do_node/4 ----------------------------
    def _reactive_pass(self, kind, code, index, stack):
        children = list_of(self._fresh(self._one("reactive_children", code, None)[1]))
        functor = "seq_node" if kind == "seq" else "fallback_node"
        return ("run", Term(functor, list2term(children[index:]))), \
               (("react", kind, code),) + stack

    def _resume_index(self, status, code):
        if status == code:
            return 0
        answers = self._query("guard_resume", status, code, None)
        return int(answers[0][2]) if answers else None

    def advance(self, work, stack, reactive_count):
        """Runs composites until the next leaf or the end of the tree.
        work is ("run", node) or ("ret", outcome). Returns ("leaf", term)
        or ("final", outcome), plus the remaining stack."""
        while True:
            if work[0] == "run":
                node = work[1]
                name = getattr(node, "functor", None)
                if name in ("seq_node", "fallback_node"):
                    children = list_of(node.args[0])
                    if not children:
                        empty = "true" if name == "seq_node" else "false"
                        work = ("ret", Term(empty))
                        continue
                    frame = "seq" if name == "seq_node" else "fallback"
                    stack = ((frame, tuple(children[1:])),) + stack
                    work = ("run", children[0])
                elif name == "inverter":
                    stack = (("inverter",),) + stack
                    work = ("run", node.args[0])
                elif name in ("reactivesequence", "reactivefallback"):
                    kind = "seq" if name == "reactivesequence" else "fallback"
                    work, stack = self._reactive_pass(kind, node.args[0], 0, stack)
                else:
                    return ("leaf", node), stack
                continue

            status = work[1]
            if not stack:
                if status.functor == "reactive":
                    return ("final", "reactive_escaped"), ()
                return ("final", status.functor), ()

            frame, stack = stack[0], stack[1:]
            if frame[0] == "seq":
                if status.functor == "true":
                    work = ("run", Term("seq_node", list2term(list(frame[1]))))
            elif frame[0] == "fallback":
                if status.functor == "false":
                    work = ("run", Term("fallback_node", list2term(list(frame[1]))))
            elif frame[0] == "inverter":
                if status.functor in ("true", "false"):
                    flipped = "false" if status.functor == "true" else "true"
                    work = ("ret", Term(flipped))
            elif frame[0] == "react" and status.functor == "reactive":
                kind, code = frame[1], frame[2]
                index = self._resume_index(status.args[0], code)
                if index is not None and reactive_count <= self.replan_budget:
                    work, stack = self._reactive_pass(kind, code, index, stack)
                elif index is not None:
                    work = ("ret", Term("world_too_large"))

    # -- merging -------------------------------------------------------
    def _project_reason(self, reason):
        if self.needs_sample_detail:
            return reason
        if reason.functor == "sample_success" and reason.arity == 5:
            return Term("sample_success", *reason.args[3:])
        if reason.functor == "sample_failure" and reason.arity == 4:
            return Term("sample_failure", *reason.args[2:])
        return reason

    def _memory_key(self, mem):
        if self.merge == "none":
            return term2str(mem)
        reasons, _outcomes, checks, planned, arrivals, last_halt = mem.args
        parts = [term2str(self._project_reason(r)) for r in list_of(reasons)]
        parts += [f"{term2str(c.args[0])}={term2str(c.args[2])}" for c in list_of(checks)]
        parts.append(term2str(planned))
        parts.append(term2str(last_halt))
        if self.needs_arrivals:
            parts.append(term2str(arrivals))
        return ";".join(parts)

    def _config_key(self, zb, phys, mem, work, stack):
        def frame_text(frame):
            return "/".join(
                ",".join(term2str(t) for t in part) if isinstance(part, tuple)
                else (term2str(part) if isinstance(part, Term) else str(part))
                for part in frame)
        tree = term2str(work[1]) if work[0] == "leaf" else str(work[1])
        return fresh_key("|".join([str(zb), term2str(phys), self._memory_key(mem),
                                   work[0], tree]
                                  + [frame_text(frame) for frame in stack]))

    def _seed_leaf(self, phys, mem, work):
        leaf = work[1]
        if work[0] != "leaf" or leaf.functor != "moveto_leg":
            return phys
        triggers = {term2str(t) for t in list_of(leaf.args[1])}
        if not triggers <= SEED_SAFE_TRIGGERS:
            return phys
        return self._one("walk_seed", phys, mem, None, db=self.dbs[self._zb])[2]

    # -- expansion -----------------------------------------------------
    def _intern(self, zb, phys, mem, work, stack):
        """The node for this configuration, and whether it is new."""
        self._zb = zb
        seeded = self._seed_leaf(phys, mem, work)
        key = self._config_key(zb, seeded, mem, work, stack)
        node = self.nodes.get(key)
        if node is not None:
            return node, False
        node = Node(next(self.next_id), zb, seeded, mem, work, stack, phys)
        self.nodes[key] = node
        self.by_id[node.id] = node
        return node, True

    def _add_edge(self, source, target, action, weight, draws, status, reason):
        edge = self.edges.get((source, target, action))
        if edge is None:
            edge = {"source": source, "target": target, "action": action,
                    "probability": 0.0, "draws": [], "status": status,
                    "reason": reason}
            self.edges[(source, target, action)] = edge
        edge["probability"] += weight
        if draws:
            edge["draws"].append(draws)

    @staticmethod
    def _action_label(leaf):
        code = term2str(leaf.args[-1]) if leaf.args else ""
        if leaf.functor == "cond":
            code = f"{term2str(leaf.args[1])}: {term2str(leaf.args[0])}"
        return f"{leaf.functor}({code})"

    def expand(self, node):
        """Runs node's leaf for every joint draw. Returns new child nodes."""
        self.leaf_steps += 1
        leaf = node.work[1]
        names = [n for n in LEAF_NOISE.get(leaf.functor, ()) if n in self.tables]
        choices = list2term([
            Term("choice", Term(name),
                 list2term([value for value, _ in self.tables[name][1]]))
            for name in names])
        answers = self._query("progress_leaf", leaf, node.phys, node.mem, choices,
                              None, None, None, None, db=self.dbs[node.zb])

        action = self._action_label(leaf)
        weights = {name: {term2str(v): w for v, w in self.tables[name][1]}
                   for name in names}
        seen, children = {}, []
        reasons_before = len(list_of(node.mem.args[0]))
        for answer in answers:
            leaf_out, _, _, _, draws_term, outcome, phys, mem = answer
            draws = [(term2str(d.args[0]), term2str(d.args[1]))
                     for d in list_of(draws_term)]
            draw_key = tuple(draws)
            if draw_key in seen:
                raise RuntimeError(f"{action}: two successors for draws {draws}")
            seen[draw_key] = True

            weight = 1.0
            for name, value in draws:
                weight *= weights[name][value]

            bindings = Bindings()
            self._collect_bindings(leaf, leaf_out, bindings)
            stack = self._substitute_stack(node.stack, bindings)
            work, stack = self.advance(("ret", outcome), stack, int(phys.args[10]))
            child, is_new = self._intern(node.zb, phys, mem, work, stack)
            if is_new:
                children.append(child)

            new_reasons = list_of(mem.args[0])[:-reasons_before or None]
            reason = term2str(new_reasons[0]) if new_reasons else None
            self._add_edge(node.id, child.id, action, weight, dict(draws),
                           term2str(outcome), reason)

        # A draw with no successor is a world with no final situation.
        for combo in itertools.product(*(self.tables[n][1] for n in names)):
            draws = [(name, term2str(value)) for name, (value, _) in zip(names, combo)]
            if tuple(draws) in seen:
                continue
            weight = 1.0
            for _, entry_weight in combo:
                weight *= entry_weight
            stuck, is_new = self._intern(node.zb, node.phys, node.mem,
                                         ("final", "stuck"), ())
            if is_new:
                children.append(stuck)
            self._add_edge(node.id, stuck.id, action, weight, dict(draws), "stuck", None)
        return children

    @classmethod
    def _collect_bindings(cls, pattern, value, bindings):
        if isinstance(pattern, Var):
            if not isinstance(value, Var):
                bindings[pattern.name] = value
        elif isinstance(pattern, Term):
            for sub_pattern, sub_value in zip(pattern.args, value.args):
                cls._collect_bindings(sub_pattern, sub_value, bindings)

    @staticmethod
    def _substitute_stack(stack, bindings):
        if not bindings:
            return stack
        result = []
        for frame in stack:
            if frame[0] in ("seq", "fallback"):
                result.append((frame[0], tuple(t.apply(bindings) for t in frame[1])))
            else:
                result.append(frame)
        return tuple(result)

    # -- search --------------------------------------------------------
    def run(self):
        plan = self._fresh(self._one("plan", None)[0])
        self.by_id[0] = None
        frontier = []
        for value, weight in self.tables["zbatt"][1]:
            zb = term2str(value)
            db = self.base_db.extend()
            db.add_fact(Term("current_zb", value))
            self.dbs[zb] = db
            phys, mem = self._one("initial_state", None, None, db=db)
            work, stack = self.advance(("run", plan), (), 0)
            node, _ = self._intern(zb, phys, mem, work, stack)
            self._add_edge(0, node.id, "zbatt", weight, {"zbatt": zb}, "true", None)
            frontier.append(node)

        layer = 0
        while frontier:
            layer += 1
            print(f"  [layer {layer}] {len(frontier)} config(s), "
                  f"{len(self.nodes)} node(s) so far", flush=True)
            next_frontier = []
            for node in frontier:
                if node.work[0] == "leaf":
                    next_frontier.extend(self.expand(node))
            frontier = next_frontier

    def reach_probabilities(self):
        """Topological pass: reach(n) = sum of reach(parent) * edge weight."""
        outgoing, indegree = defaultdict(list), defaultdict(int)
        for edge in self.edges.values():
            outgoing[edge["source"]].append(edge)
            indegree[edge["target"]] += 1
        reach = defaultdict(float)
        reach[0] = 1.0
        queue, processed = deque([0]), 0
        while queue:
            current = queue.popleft()
            processed += 1
            for edge in outgoing[current]:
                reach[edge["target"]] += reach[current] * edge["probability"]
                indegree[edge["target"]] -= 1
                if indegree[edge["target"]] == 0:
                    queue.append(edge["target"])
        if processed != len(self.by_id):
            raise RuntimeError("execution graph has a cycle")
        return reach

    def evaluate_finals(self, reach, queries):
        """Sums reach over the final nodes for every generated query."""
        probabilities = defaultdict(float)
        for name in ("true", "false", "world_too_large", "reactive_escaped"):
            probabilities[f"plan_outcome({name})"] += 0.0
        outcome_mass, goal_flags = defaultdict(float), {}
        skipped, generic = set(), []
        for query in queries:
            if query.functor.startswith(("any_reason", "any_condition", "any_battery")):
                if (self.merge != "none"
                        and query.functor == "any_reason_pattern_detail_by_action"
                        and query.args[0].functor.startswith("sample_")):
                    skipped.add(term2str(query))
                    continue
                generic.append(query)
                if not query.variables():
                    probabilities[term2str(query)] += 0.0
            elif query.functor not in ("verify_goal_formula", "plan_outcome",
                                       "outcome_signature"):
                raise Unsupported(f"no final-state reading for {query}")
        generic_list = list2term(generic)

        for node in self.by_id.values():
            if node is None or node.work[0] != "final":
                continue
            p = reach[node.id]
            outcome = node.work[1]
            outcome_mass[outcome] += p
            if outcome == "stuck":
                continue
            probabilities[f"plan_outcome({outcome})"] += p
            if outcome == "reactive_escaped":
                continue

            db = self.dbs[node.zb]
            goal = bool(self._query("final_goal", node.phys, node.mem, db=db))
            goal_flags[node.id] = goal
            if goal:
                probabilities["verify_goal_formula"] += p
            answers = self._one("final_answers", generic_list, node.phys, node.mem,
                                None, db=db)[3]
            for row in {term2str(a) for a in list_of(answers)}:
                probabilities[row] += p
            if self.merge == "none":
                signature = self._one("final_signature", node.phys, node.mem,
                                      None, db=db)[2]
                probabilities[f"outcome_signature({term2str(signature)})"] += p
        probabilities["verify_goal_formula"] += 0.0
        return dict(probabilities), dict(outcome_mass), goal_flags, sorted(skipped)

    def graph_json(self, reach, goal_flags, labels):
        nodes = [{"id": 0, "kind": "root", "reach": 1.0}]
        for node in self.by_id.values():
            if node is None:
                continue
            phys = node.display
            final = node.work[0] == "final"
            entry = {
                "id": node.id,
                "kind": "final" if final else "state",
                "reach": reach[node.id],
                "zbatt": node.zb,
                "x": term2str(phys.args[0]),
                "y": term2str(phys.args[1]),
                "time": term2str(phys.args[2]),
                "battery_anchor": [term2str(phys.args[3]), term2str(phys.args[4])],
                "hitch": term2str(phys.args[5]),
            }
            if final:
                entry["outcome"] = node.work[1]
                entry["goal"] = goal_flags.get(node.id)
                entry["reasons"] = [term2str(r) for r in list_of(node.mem.args[0])]
            else:
                entry["next_leaf"] = self._action_label(node.work[1])
            nodes.append(entry)
        return {"nodes": nodes, "edges": list(self.edges.values()), "labels": labels}


def _rename_variables(term, names, counter):
    """term with every variable renamed apart from other queries'. Each
    '_' is its own variable, however many times it is written."""
    if isinstance(term, Var):
        if term.name == "_":
            return Var(f"Q{next(counter)}")
        return names.setdefault(term.name, Var(f"Q{next(counter)}"))
    if isinstance(term, Term) and term.args:
        return term.with_args(*(_rename_variables(a, names, counter) for a in term.args))
    return term


def load_queries(path):
    """Every query/1 declared in path: main.py's ProbLog run asks all of
    them, from queries_generated.pl and from config_generated.pl alike."""
    queries = []
    counter = itertools.count()
    for statement in PrologFile(path):
        if isinstance(statement, Term) and statement.functor == "query":
            queries.append(_rename_variables(statement.args[0], {}, counter))
    return queries


def run_problem(problem, merge):
    sys.setrecursionlimit(100000)
    ctx = pipeline.run_shared_setup(problem, f"progression --merge {merge}",
                                    keep_output=True)
    if ctx is None:
        raise SystemExit(1)
    problem_dir = ctx["problem_dir"]
    goal_path = ctx["goal_formula_path"]
    out_dir = os.path.join(OUTPUT_ROOT, problem)
    os.makedirs(out_dir, exist_ok=True)

    config_path = os.path.join(problem_dir, "config_generated.pl")
    tables = load_noise_tables(config_path)
    noise_free = os.path.join(out_dir, f"{problem}_config_noise_free.pl")
    write_noise_free_config(config_path, noise_free, tables)
    write_bootstrap(problem_dir, noise_free, goal_path, f"{problem}, --merge {merge}")

    with open(goal_path) as f:
        goal_text = f.read()
    with open(os.path.join(problem_dir, "plan_generated.pl")) as f:
        plan_text = f.read()

    started = time.perf_counter()
    search = Progression(problem_dir, merge, tables, goal_text, plan_text)
    load_seconds = time.perf_counter() - started
    print(f"\n  Progression: {problem}, --merge {merge} "
          f"(theory loaded in {load_seconds:.2f}s)")

    started = time.perf_counter()
    search.run()
    search_seconds = time.perf_counter() - started

    started = time.perf_counter()
    reach = search.reach_probabilities()
    queries = (load_queries(os.path.join(problem_dir, "queries_generated.pl"))
               + load_queries(config_path))
    probabilities, outcomes, goal_flags, skipped = search.evaluate_finals(reach, queries)
    evaluate_seconds = time.perf_counter() - started

    finals = [n for n in search.by_id.values() if n is not None and n.work[0] == "final"]
    final_mass = sum(reach[n.id] for n in finals)
    summary = {
        "problem": problem,
        "merge": merge,
        "load_seconds": load_seconds,
        "search_seconds": search_seconds,
        "evaluate_seconds": evaluate_seconds,
        "nodes": len(search.by_id) - 1,
        "edges": len(search.edges),
        "leaf_steps": search.leaf_steps,
        "final_states": len(finals),
        "final_mass": final_mass,
        "outcomes": outcomes,
        "skipped_queries": skipped,
        "probabilities": probabilities,
    }
    prefix = os.path.join(out_dir, f"{problem}_{merge}")
    with open(f"{prefix}_summary.json", "w") as f:
        json.dump(summary, f, indent=1)
    with open(f"{prefix}_graph.json", "w") as f:
        labels = {**ctx["action_labels"], **ctx["condition_labels"]}
        json.dump(search.graph_json(reach, goal_flags, labels), f)

    print(f"  nodes={summary['nodes']} edges={summary['edges']} "
          f"finals={summary['final_states']} leaf_steps={summary['leaf_steps']}")
    print(f"  search {search_seconds:.2f}s, evaluate {evaluate_seconds:.2f}s, "
          f"final mass {final_mass:.12f}")
    print(f"  outcomes: {outcomes}")
    write_report(f"{prefix}_report.log", problem, merge, probabilities, outcomes,
                 skipped, ctx)
    print(f"  written : {prefix}_summary.json, {prefix}_report.log, {prefix}_graph.json")
    return summary


def write_report(path, problem, merge, probabilities, outcomes, skipped, ctx):
    """The same tables main.py prints for a regression run, from the same
    code, plus what only progression can say: the mass that has no final
    situation, and the rows --merge auto gives up."""
    with open(path, "w", encoding="utf-8") as log:
        tee = pipeline.Tee(log)
        tee("")
        pipeline.banner(tee, f"Progression report: {problem}, --merge {merge}")
        pipeline.print_compact_summary(tee, probabilities, ctx["goal_formula_path"],
                                       ctx["action_labels"], ctx["condition_labels"])
        stuck = outcomes.get("stuck", 0.0)
        if stuck > 0.0:
            pipeline.section(tee, "Worlds with no final situation")
            tee(f"  stuck{'':<35} {stuck*100:6.2f}%")
            tee("  (a regressed run has no plan_outcome for these either)")
        if merge != "none":
            pipeline.section(tee, "Not reported with --merge auto")
            tee("  outcome_signature (Full outcome enumeration)")
            for row in skipped:
                tee(f"  {row}")
            tee("  These name exact values merging removed; use --merge none.")


def run_all(merge):
    """Every problem* directory, one process each: the Python black boxes
    read BT_PROBLEM_DIR at import time, so they can't move to the next
    problem inside one process. A failure doesn't stop the others."""
    problems = sorted(name for name in os.listdir(pipeline.PROBLEMS_DIR)
                      if name.startswith("problem")
                      and os.path.isdir(os.path.join(pipeline.PROBLEMS_DIR, name)))
    print(f"[info] --problem all: {len(problems)} problem(s) -- {', '.join(problems)}")
    rows, failed = [], []
    for problem in problems:
        print(f"\n===== {problem} =====", flush=True)
        done = subprocess.run([sys.executable, os.path.abspath(__file__),
                               "--problem", problem, "--merge", merge])
        if done.returncode != 0:
            failed.append(problem)
            rows.append(f"  {problem:<12} FAILED (exit code {done.returncode})")
            continue
        path = os.path.join(OUTPUT_ROOT, problem, f"{problem}_{merge}_summary.json")
        with open(path) as f:
            summary = json.load(f)
        seconds = summary["search_seconds"] + summary["evaluate_seconds"]
        rows.append(f"  {problem:<12} P(goal) {summary['probabilities']['verify_goal_formula']*100:6.2f}%"
                    f"   {summary['nodes']:>7} nodes   {seconds:8.1f}s")
    print(f"\n===== summary, --merge {merge} =====")
    print("\n".join(rows))
    if failed:
        print(f"\n[ERROR] {len(failed)} problem(s) failed: {', '.join(failed)}")
        raise SystemExit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--problem", required=True,
                    help="a subdirectory of problems/, or \"all\" for every "
                         "problem* one, in alphabetical order")
    ap.add_argument("--merge", choices=("auto", "none"), default="auto")
    args = ap.parse_args()
    if args.problem == "all":
        run_all(args.merge)
    else:
        run_problem(args.problem, args.merge)


if __name__ == "__main__":
    main()
