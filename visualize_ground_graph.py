#!/usr/bin/env python3
"""
visualize_ground_graph.py

Loads a ground graph produced by main.py's own --save-graphs flag (see
module/theory/graph_export.py -- writes output/<problem>/graphs/
ground.dot + ground_nodes.json) and renders it as a spinning 3D point
cloud: nodes as small grey balls, edges as black lines, white
background, rotating around the vertical axis. Then, driven by
visualize_config.yaml (see that file's own comments for every
parameter), it plays out two reveal steps on top of the static graph:

  1. At t1_seconds, the GOAL query node (see --goal-query / config's
     own goal_query) turns red and "shiny".
  2. Starting interval_seconds after that, one at a time, `count`
     DISTINCT PROOFS of the goal light up in their own color, also
     "shiny", one every interval_seconds.

See visualize_compiled_graph.py for the compiled-circuit counterpart
of this script (same look, same config file, same output directory --
see resolve_output_path in graph_viz_common.py) -- the two differ in
what counts as a "proof": this script's own proofs are sign-agnostic
(see graph_viz_common.py's own header for why), since the ground graph
isn't a deterministic circuit the way the compiled one is.

HONEST CAVEAT ON "SHINY": matplotlib's 3D backend has no real specular/
lighting model. "Shiny" here is FAKED: a brighter/larger marker and a
small pure-white "glint" dot drawn on top at the same position. See
graph_viz_common.py's own render() if a real lighting backend (e.g.
PyVista) is ever wanted instead.

Usage:
    python3 visualize_ground_graph.py [--config visualize_config.yaml]
        [--graphs-dir output/problem0S/graphs] [--goal-query TERM]
        [--output path.mp4] [--output-dir DIR]

Requires: numpy, matplotlib, pyyaml (all already used elsewhere in
this project). Does NOT require networkx or any 3D-engine package.
"""

import argparse
import sys

import yaml

from graph_viz_common import (
    load_graph, pick_goal_index, enumerate_proofs, proof_edges,
    render, resolve_output_path,
)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="visualize_config.yaml",
                     help="Path to the YAML config (default: "
                          "visualize_config.yaml, alongside this script).")
    ap.add_argument("--graphs-dir", default=None,
                     help="Override config's own graphs_dir -- a "
                          "directory containing ground_nodes.json, as "
                          "written by main.py --save-graphs.")
    ap.add_argument("--goal-query", default=None,
                     help="Override config's own goal_query.")
    ap.add_argument("--output", default=None,
                     help="Render to this EXACT file path instead of "
                          "config's own output_dir/goal-query auto-naming.")
    ap.add_argument("--output-dir", default=None,
                     help="Override config's own top-level output_dir "
                          "-- the common directory both this script and "
                          "visualize_compiled_graph.py write their own "
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
        nodes_by_index, queries = load_graph(graphs_dir, "ground_nodes.json")
    except FileNotFoundError:
        print(f"[ERROR] {graphs_dir}/ground_nodes.json not found -- run "
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
    proofs = enumerate_proofs(nodes_by_index, goal_index, n_proofs, max_steps)
    proof_edge_sets = [proof_edges(nodes_by_index, p) for p in proofs]

    output_path = resolve_output_path(
        config, args.output, args.output_dir, "ground_graph")

    render(nodes_by_index, goal_index, goal_name, proofs, proof_edge_sets,
           config, output_path=output_path)


if __name__ == "__main__":
    main()
