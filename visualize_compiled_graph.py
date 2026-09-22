#!/usr/bin/env python3
"""
visualize_compiled_graph.py

Loads the compiled circuit produced by main.py's own --save-graphs
flag (see module/theory/graph_export.py -- writes
output/<problem>/graphs/compiled.dot + compiled_nodes.json; the
compiler is whichever get_evaluatable() resolves to in this
environment: SDD if pysdd is installed, DSharp d-DNNF otherwise) and
renders it exactly like visualize_ground_graph.py does: nodes as small
grey balls, edges as black lines, white background, spinning around
the vertical axis, the goal query node turning red and "shiny" at
t1_seconds.

WHAT DIFFERS FROM THE GROUND-GRAPH SCRIPT: after the goal lights up,
this script highlights n distinct paths from the goal down to its
TRUE leaves ONLY, one every interval_seconds, same colors/timing
config as the ground-graph script (proof_paths in visualize_config
.yaml -- both scripts share the SAME config file and, by default, the
SAME output_dir, see resolve_output_path in graph_viz_common.py).
"True leaves only" matters here specifically because compilation makes
the circuit DETERMINISTIC: negative children (e.g.
conj(children=(-8,-5))) genuinely mean "this leaf is FALSE along this
branch" (confirmed directly against real dsharp output -- see
graph_viz_common.py's own header), so a leaf only ever reached through
a negative reference in a given proof is NOT one of that proof's true
leaves and is left uncolored, even though it's structurally part of
the same circuit.

Usage:
    python3 visualize_compiled_graph.py [--config visualize_config.yaml]
        [--graphs-dir output/problem0S/graphs] [--goal-query TERM]
        [--output path.mp4] [--output-dir DIR]
"""

import argparse
import sys

import yaml

from graph_viz_common import (
    load_graph, pick_goal_index, enumerate_true_leaf_proofs,
    true_proof_edges, render, resolve_output_path,
)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="visualize_config.yaml",
                     help="Path to the YAML config (default: "
                          "visualize_config.yaml -- SAME file "
                          "visualize_ground_graph.py uses).")
    ap.add_argument("--graphs-dir", default=None,
                     help="Override config's own graphs_dir -- a "
                          "directory containing compiled_nodes.json, "
                          "as written by main.py --save-graphs.")
    ap.add_argument("--goal-query", default=None,
                     help="Override config's own goal_query.")
    ap.add_argument("--output", default=None,
                     help="Render to this EXACT file path instead of "
                          "config's own output_dir auto-naming.")
    ap.add_argument("--output-dir", default=None,
                     help="Override config's own top-level output_dir "
                          "-- the common directory both this script and "
                          "visualize_ground_graph.py write their own "
                          "video into.")
    args = ap.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f) or {}

    graphs_dir = args.graphs_dir or config.get("graphs_dir")
    if not graphs_dir:
        print("[ERROR] No graphs_dir given (in config or --graphs-dir).",
              file=sys.stderr)
        sys.exit(1)

    try:
        nodes_by_index, queries = load_graph(graphs_dir, "compiled_nodes.json")
    except FileNotFoundError:
        print(f"[ERROR] {graphs_dir}/compiled_nodes.json not found -- run "
              f"main.py --problem <name> --save-graphs first.",
              file=sys.stderr)
        sys.exit(1)
    print(f"[info] loaded {len(nodes_by_index)} node(s), "
          f"{len(queries)} declared quer{'y' if len(queries)==1 else 'ies'} "
          f"from {graphs_dir}")

    goal_index, goal_name = pick_goal_index(
        nodes_by_index, queries, args.goal_query or config.get("goal_query"))

    proof_cfg = config.get("proof_paths", {})
    n_proofs = int(proof_cfg.get("count", 3))
    max_steps = int(proof_cfg.get("max_search_steps", 200000))
    proofs = enumerate_true_leaf_proofs(nodes_by_index, goal_index, n_proofs, max_steps)
    proof_edge_sets = [true_proof_edges(nodes_by_index, p) for p in proofs]

    output_path = resolve_output_path(
        config, args.output, args.output_dir, "compiled_graph")

    render(nodes_by_index, goal_index, goal_name, proofs, proof_edge_sets,
           config, output_path=output_path)


if __name__ == "__main__":
    main()
