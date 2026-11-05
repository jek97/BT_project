#!/usr/bin/env python3
"""
verify_worlds.py

An exact reference for progression.py that needs no knowledge compilation
and shares none of its machinery: it enumerates complete possible worlds
and runs each one through the UNMODIFIED regressed theory, from s0, using
do_node/4 for the whole tree and the theory's own final_situation/1,
plan_outcome/1, verify_goal_formula/0, outcome_signature/1 and
any_reason_pattern... predicates. Nothing from progression.pl is loaded.

For the problems it supports, this computes the same number as ProbLog's
weighted model count: every world is one assignment of the noise tables
per occurrence, its weight is the product of the chosen entries, and
P(query) is the sum over worlds where the query holds. The worlds cover
every table at every action occurrence, reached or not, so their weights
must sum to 1.

A draw is identified by (table, ActionCode), which is one draw per
situation only when each action runs at most once in a world. That
excludes trees with reactive composites or retries, which are refused.

    python3 verify_worlds.py --problem problem0S --jobs 24
"""

import argparse
import itertools
import json
import multiprocessing
import os
import sys
import time
from collections import defaultdict

from problog.engine import DefaultEngine
from problog.logic import Term, list2term, term2list, term2str
from problog.program import PrologFile

import main as pipeline
import progression

OUTPUT_ROOT = progression.OUTPUT_ROOT
ACTION_TABLES = {
    "moveto_leg": ("z", "zt"),
    "take_sample": ("sample_result", "sample_value"),
}


def action_slots(node):
    """[(table, ActionCode), ...] in tree order. Refuses anything that
    can run an action twice or that this script has no table entry for."""
    name = node.functor
    if name in ("seq_node", "fallback_node"):
        return [s for child in term2list(node.args[0], deep=False)
                for s in action_slots(child)]
    if name == "inverter":
        return action_slots(node.args[0])
    if name in ("reactivesequence", "reactivefallback"):
        raise SystemExit(f"verify_worlds.py does not support {name}")
    if name.endswith("_tool_leg"):
        raise SystemExit(f"verify_worlds.py does not support {name}")
    return [(table, node.args[-1]) for table in ACTION_TABLES.get(name, ())]


def write_world_config(config_path, output_path, tables):
    """config_generated.pl with each table replaced by a lookup in the
    current world's world_draw/3 facts, keyed by ActionCode."""
    readers = {
        "z": "z(do(startMoveto(_,_,Code,_),_), V) :- world_draw(z, Code, V).",
        "zt": "zt(do(startMoveto(_,_,Code,_),_), V) :- world_draw(zt, Code, V).",
        "zbatt": "zbatt(V) :- world_draw(zbatt, mission, V).",
        "sample_result":
            "sample_result(_, Code, V) :- world_draw(sample_result, Code, V).",
        "sample_value":
            "sample_value(_, Code, V) :- world_draw(sample_value, Code, V).",
    }
    progression.write_noise_free_config(config_path, output_path, tables)
    with open(output_path) as f:
        kept = [line for line in f if ":- fixed_draw(" not in line]
    with open(output_path, "w") as f:
        f.writelines(kept)
        f.write("world_draw(no_table, no_code, none) :- fail.\n")
        f.write("world_member(X, [X|_]).\n"
                "world_member(X, [_|T]) :- world_member(X, T).\n"
                "world_rows(Queries, Rows) :-\n"
                "    findall(Q, (world_member(Q, Queries), call(Q)), Rows).\n")
        for name, reader in readers.items():
            if name in tables:
                f.write(reader + "\n")


_STATE = {}


def _load(theory, tables, queries):
    engine = DefaultEngine()
    _STATE.update(engine=engine, db=engine.prepare(PrologFile(theory)),
                  tables=tables, queries=queries)
    sys.setrecursionlimit(100000)


def evaluate_world(assignment):
    """assignment: ((table, code_text, value_text), ...). Returns weight
    and the {row: True} of everything that holds in this world."""
    engine, base_db = _STATE["engine"], _STATE["db"]
    tables = _STATE["tables"]
    db = base_db.extend()
    weight = 1.0
    for table, code, value_text in assignment:
        value, entry_weight = next((v, w) for v, w in tables[table][1]
                                   if term2str(v) == value_text)
        weight *= entry_weight
        db.add_fact(Term("world_draw", Term(table), Term(code), value))

    answer = engine.query(db, Term("world_rows", _STATE["queries"], None))[0]
    return weight, {term2str(row) for row in term2list(answer[1], deep=False)}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--problem", required=True)
    ap.add_argument("--jobs", type=int, default=16)
    args = ap.parse_args()
    problem = args.problem

    ctx = pipeline.run_shared_setup(problem, "verify_worlds", keep_output=True)
    if ctx is None:
        raise SystemExit(1)
    problem_dir = ctx["problem_dir"]
    out_dir = os.path.join(OUTPUT_ROOT, problem)
    os.makedirs(out_dir, exist_ok=True)

    config_path = os.path.join(problem_dir, "config_generated.pl")
    tables = progression.load_noise_tables(config_path)
    world_config = os.path.join(out_dir, f"{problem}_config_worlds.pl")
    write_world_config(config_path, world_config, tables)
    progression.write_bootstrap(problem_dir, world_config, ctx["goal_formula_path"],
                                f"{problem}, verify_worlds")

    engine = DefaultEngine()
    db = engine.prepare(PrologFile(pipeline.THEORY_PATH))
    plan = engine.query(db, Term("plan", None))[0][0]
    slots = [(table, term2str(code)) for table, code in action_slots(plan)
             if table in tables]
    if len(set(slots)) != len(slots):
        raise SystemExit("an action code appears twice: draws would be shared")

    queries = list2term(progression.load_queries(
        os.path.join(problem_dir, "queries_generated.pl")))

    def choices(table):
        return [term2str(v) for v, _ in tables[table][1]]

    root = [("zbatt", "mission", v) for v in choices("zbatt")]
    grid = [[(t, c, v) for v in choices(t)] for t, c in slots]
    worlds = [(zb,) + rest for zb in root for rest in itertools.product(*grid)]
    print(f"  {problem}: {len(worlds)} world(s), {len(slots)} draw slot(s), "
          f"{args.jobs} job(s)", flush=True)

    started = time.perf_counter()
    probabilities, mass = defaultdict(float), 0.0
    with multiprocessing.Pool(args.jobs, _load,
                              (pipeline.THEORY_PATH, tables, queries)) as pool:
        for index, (weight, rows) in enumerate(
                pool.imap_unordered(evaluate_world, worlds, chunksize=8), 1):
            mass += weight
            for row in rows:
                probabilities[row] += weight
            if index % 500 == 0:
                print(f"  {index}/{len(worlds)} worlds, "
                      f"{time.perf_counter() - started:.0f}s", flush=True)

    output = os.path.join(out_dir, f"{problem}_baseline.json")
    with open(output, "w") as f:
        json.dump({"problem": problem, "backend": "world_enumeration",
                   "seconds": time.perf_counter() - started, "worlds": len(worlds),
                   "world_mass": mass, "probabilities": dict(probabilities)},
                  f, indent=1)
    print(f"  world mass {mass:.12f}; written {output}")


if __name__ == "__main__":
    main()
