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
     DISTINCT PROOFS of the goal (sub-DAGs from the AD-fact leaves up
     to the goal node -- see enumerate_proofs()'s own note for exactly
     what a "proof" means here) light up in their own color, also
     "shiny", one every interval_seconds.

HONEST CAVEAT ON "SHINY": matplotlib's 3D backend has no real specular/
lighting model (unlike a proper 3D engine such as Blender, PyVista/VTK,
or a WebGL/Three.js viewer). "Shiny" here is FAKED: a brighter/larger
marker, a light outline, and a small pure-white "glint" dot drawn on
top at the same position. It reads as a highlight at a glance, but it
is not real specular shading, and no real 3D engine dependency was
introduced to get it. If true material shininess is wanted, swap the
rendering backend (e.g. PyVista) for the "static appearance" section --
the graph-loading/layout/proof-enumeration code below is backend-
agnostic and does not need to change for that.

WHAT A "PROOF" MEANS HERE (see also the chat discussion this tool was
written to support): ground_nodes.json's own node table already has,
per node, its type (atom/conj/disj) and its children (see
graph_export.py). A "proof" is one full top-down expansion of the
goal's own OR (disj) nodes -- at a disj node, branch over exactly ONE
child; at a conj node, ALL children are required together; atoms are
the leaves (AD facts). enumerate_proofs() below walks this exactly as
described, lazily, capped by max_search_steps (this can blow up
combinatorially on a large real ground graph -- see that function's
own note).

Usage:
    python3 visualize_ground_graph.py [--config visualize_config.yaml]
        [--graphs-dir output/problem0S/graphs] [--goal-query TERM]
        [--output path.mp4]

Requires: numpy, matplotlib, pyyaml (all already used elsewhere in
this project). Does NOT require networkx or any 3D-engine package --
the layout below is a small hand-rolled layered placement, not a
force-directed one, specifically so this script stays usable on a
graph with thousands of nodes without an O(n^2) blowup (see
layered_layout_3d()'s own note).
"""

import argparse
import json
import math
import os
import sys
from collections import defaultdict

import matplotlib
import numpy as np
import yaml


# -----------------------------------------------------------------------
# Loading
# -----------------------------------------------------------------------
def load_ground_graph(graphs_dir):
    """Reads graphs_dir/ground_nodes.json (see graph_export.py's own
    _write_graph) and returns (nodes_by_index, queries), where
    nodes_by_index maps int -> the node's own JSON record (type, name,
    functor, children, ...) and queries maps query-term-string -> node
    index."""
    path = os.path.join(graphs_dir, "ground_nodes.json")
    with open(path) as f:
        data = json.load(f)
    nodes_by_index = {rec["index"]: rec for rec in data["nodes"]}
    queries = data["queries"]
    return nodes_by_index, queries


def pick_goal_index(nodes_by_index, queries, goal_query):
    """goal_query: an exact key of `queries` (e.g. "verify_goal_formula"),
    or None -- in which case "verify_goal_formula" is used if present
    (the canonical top-level goal query in this project's own theory --
    see basic_action_theory.pl's own verify_goal_formula/0), else the
    alphabetically-first declared query, so this always resolves to
    SOMETHING rather than failing outright on a problem whose queries
    happen to be named differently."""
    if goal_query is not None:
        if goal_query not in queries:
            raise KeyError(
                f"goal_query {goal_query!r} is not one of this graph's own "
                f"declared queries: {sorted(queries)}")
        return queries[goal_query], goal_query
    if "verify_goal_formula" in queries:
        return queries["verify_goal_formula"], "verify_goal_formula"
    if not queries:
        raise ValueError("ground_nodes.json has no declared queries at all.")
    name = sorted(queries)[0]
    return queries[name], name


# -----------------------------------------------------------------------
# Proof enumeration -- see this module's own header for what a "proof"
# means here. Lazy generator + a shared node-visit budget so a caller
# can ask for just `n` proofs without the search fully exploring a
# combinatorially large ground graph first.
# -----------------------------------------------------------------------
def _iter_proofs(nodes_by_index, index, chosen, budget):
    idx = abs(index)
    if idx in chosen:
        yield chosen
        return
    budget["steps"] -= 1
    if budget["steps"] <= 0:
        return
    node = nodes_by_index.get(idx)
    if node is None:
        # A leaf referenced by index but not itself present as its own
        # node record -- treat as a terminal (shouldn't normally
        # happen with graph_export.py's own output, but keeps this
        # robust rather than crashing on an edge case).
        yield chosen | {idx}
        return
    new_chosen = chosen | {idx}
    children = [c for c in (node.get("children") or []) if c != 0]
    if node["type"] == "atom" or not children:
        yield new_chosen
        return
    if node["type"] == "conj":
        yield from _iter_conj(nodes_by_index, children, new_chosen, budget)
    elif node["type"] == "disj":
        for c in children:
            yield from _iter_proofs(nodes_by_index, c, new_chosen, budget)
    else:
        raise TypeError(f"Unexpected node type: {node['type']!r}")


def _iter_conj(nodes_by_index, children, chosen, budget):
    """AND semantics: every child in `children` must be proven, in
    sequence, threading the growing `chosen` set through each -- this
    is what makes a conj node's own proof set the UNION of one proof
    per child, all required together (as opposed to disj's branching,
    which yields ALTERNATIVE proof sets)."""
    if not children:
        yield chosen
        return
    first, rest = children[0], children[1:]
    for partial in _iter_proofs(nodes_by_index, first, chosen, budget):
        yield from _iter_conj(nodes_by_index, rest, partial, budget)


def enumerate_proofs(nodes_by_index, root_index, n, max_search_steps=200000):
    """Returns up to n DISTINCT proof node-index-sets for root_index
    (typically the goal query's own node), each one a full top-down
    AND/OR expansion (see this module's own header). Stops early, with
    a warning to stderr, if max_search_steps node-visits are exhausted
    before n distinct proofs are found -- this is a REAL possibility on
    a large real ground graph (many combinable discretized stochastic
    choices -- see FUTUREWORK.md and basic_action_theory.pl's own notes
    on this project's own combinatorial blowup), not a bug; raise
    max_search_steps (visualize_config.yaml's own proof_paths.
    max_search_steps) if fewer than n proofs come back and a bigger
    search budget is affordable."""
    budget = {"steps": max_search_steps}
    proofs = []
    for p in _iter_proofs(nodes_by_index, root_index, set(), budget):
        if p not in proofs:
            proofs.append(p)
        if len(proofs) >= n:
            break
    if len(proofs) < n:
        print(f"[warn] enumerate_proofs: only found {len(proofs)}/{n} "
              f"distinct proofs before exhausting the graph or the "
              f"{max_search_steps}-step search budget (see "
              f"visualize_config.yaml's own proof_paths.max_search_steps)",
              file=sys.stderr)
    return proofs


def proof_edges(nodes_by_index, proof_node_set):
    """The subset of the full edge list whose BOTH endpoints are in
    proof_node_set -- used to also color a proof's own edges, not just
    its nodes, when it lights up."""
    edges = set()
    for idx in proof_node_set:
        node = nodes_by_index[idx]
        for c in (node.get("children") or []):
            if c == 0:
                continue
            cidx = abs(c)
            if cidx in proof_node_set:
                edges.add((idx, cidx))
    return edges


# -----------------------------------------------------------------------
# Layout -- layered by derivation depth (leaves = depth 0), nodes at
# the same depth spread evenly around a ring. Chosen over a generic
# force-directed layout deliberately: O(n) instead of O(n^2) (a
# force-directed layout's all-pairs repulsion is not viable on a
# ground graph with thousands of nodes -- see this project's own
# problem4/problem45* scale), AND depth is already a semantically
# meaningful axis here (a proof's own leaves-to-goal structure), which
# a force-directed layout would only discover by accident if at all.
# -----------------------------------------------------------------------
def compute_depths(nodes_by_index):
    depth = {}
    visiting = set()

    def get_depth(idx):
        if idx in depth:
            return depth[idx]
        if idx in visiting:
            # A cycle (shouldn't normally happen in an acyclic ground
            # formula, but guards against one anyway rather than
            # infinite-recursing) -- treat as a leaf at this point.
            return 0
        visiting.add(idx)
        node = nodes_by_index.get(idx)
        children = [abs(c) for c in (node.get("children") or []) if c != 0] \
            if node else []
        children = [c for c in children if c in nodes_by_index]
        d = 1 + max((get_depth(c) for c in children), default=-1)
        visiting.discard(idx)
        depth[idx] = d
        return d

    sys.setrecursionlimit(max(sys.getrecursionlimit(), 10000))
    for idx in nodes_by_index:
        get_depth(idx)
    return depth


def layered_layout_3d(nodes_by_index, ring_base_radius=1.0,
                       ring_crowding_factor=0.15):
    depths = compute_depths(nodes_by_index)
    by_depth = defaultdict(list)
    for idx, d in depths.items():
        by_depth[d].append(idx)

    pos = {}
    for d, idxs in by_depth.items():
        idxs = sorted(idxs)
        count = len(idxs)
        radius = ring_base_radius + ring_crowding_factor * math.sqrt(count)
        for i, idx in enumerate(idxs):
            theta = 2 * math.pi * i / count if count > 1 else 0.0
            pos[idx] = (radius * math.cos(theta), radius * math.sin(theta), float(d))
    max_depth = max(depths.values()) if depths else 0
    return pos, max_depth


# -----------------------------------------------------------------------
# Rendering / animation
# -----------------------------------------------------------------------
def _resolve_output_backend(output_path):
    if output_path is not None:
        matplotlib.use("Agg")


def render(nodes_by_index, queries, config, goal_query_override=None,
           output_override=None):
    from matplotlib import animation
    import matplotlib.pyplot as plt

    anim_cfg = config.get("animation", {})
    output_path = output_override or anim_cfg.get("output_path")
    _resolve_output_backend(output_path)

    goal_index, goal_name = pick_goal_index(
        nodes_by_index, queries, goal_query_override or config.get("goal_query"))
    print(f"[info] goal query: {goal_name} (ground node {goal_index})")

    proof_cfg = config.get("proof_paths", {})
    n_proofs = int(proof_cfg.get("count", 3))
    proofs = enumerate_proofs(
        nodes_by_index, goal_index, n_proofs,
        max_search_steps=int(proof_cfg.get("max_search_steps", 200000)))
    proof_edge_sets = [proof_edges(nodes_by_index, p) for p in proofs]
    print(f"[info] {len(proofs)} proof(s) found "
          f"(sizes: {[len(p) for p in proofs]})")

    layout_cfg = config.get("layout", {})
    pos, max_depth = layered_layout_3d(
        nodes_by_index,
        ring_base_radius=float(layout_cfg.get("ring_base_radius", 1.0)),
        ring_crowding_factor=float(layout_cfg.get("ring_crowding_factor", 0.15)))

    indices = sorted(nodes_by_index)
    index_of = {idx: i for i, idx in enumerate(indices)}
    xs = np.array([pos[idx][0] for idx in indices])
    ys = np.array([pos[idx][1] for idx in indices])
    zs = np.array([pos[idx][2] for idx in indices])

    edge_list = []
    for idx in indices:
        node = nodes_by_index[idx]
        for c in (node.get("children") or []):
            if c == 0:
                continue
            cidx = abs(c)
            if cidx in nodes_by_index:
                edge_list.append((idx, cidx))

    base_node_color = config.get("node_color", "#999999")
    base_node_size = float(config.get("node_size", 18))
    base_edge_color = config.get("edge_color", "black")
    base_edge_width = float(config.get("edge_width", 0.5))
    base_edge_alpha = float(config.get("edge_alpha", 0.35))
    background_color = config.get("background_color", "white")

    goal_cfg = config.get("goal_highlight", {})
    t1 = float(goal_cfg.get("t1_seconds", 3.0))
    goal_color = goal_cfg.get("color", "red")
    goal_size_mult = float(goal_cfg.get("size_multiplier", 2.5))

    proof_colors = proof_cfg.get(
        "colors", ["orange", "magenta", "cyan", "lime", "indigo", "gold"])
    proof_interval = float(proof_cfg.get("interval_seconds", 2.0))
    proof_size_mult = float(proof_cfg.get("size_multiplier", 1.8))

    spin_cfg = config.get("spin", {})
    spin_enabled = bool(spin_cfg.get("enabled", True))
    degrees_per_second = float(spin_cfg.get("degrees_per_second", 18))
    elevation = float(spin_cfg.get("elevation", 18))

    fps = int(anim_cfg.get("fps", 30))
    duration = anim_cfg.get("duration_seconds")
    tail = float(anim_cfg.get("tail_seconds", 3.0))
    if duration is None:
        duration = t1 + len(proofs) * proof_interval + tail
    duration = float(duration)
    n_frames = max(1, int(duration * fps))

    # ---- figure/axes ----
    fig = plt.figure(figsize=(9, 9), facecolor=background_color)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor(background_color)
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.set_pane_color((1.0, 1.0, 1.0, 0.0))
        pane.line.set_color((1.0, 1.0, 1.0, 0.0))
    ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
    ax.set_axis_off()

    edge_segments = [((pos[a][0], pos[a][1], pos[a][2]),
                       (pos[b][0], pos[b][1], pos[b][2])) for a, b in edge_list]
    from mpl_toolkits.mplot3d.art3d import Line3DCollection
    edge_collection = Line3DCollection(
        edge_segments, colors=base_edge_color,
        linewidths=base_edge_width, alpha=base_edge_alpha)
    ax.add_collection3d(edge_collection)

    node_scatter = ax.scatter(xs, ys, zs, s=base_node_size,
                               c=base_node_color, depthshade=True,
                               edgecolors="none")
    glint_scatter = ax.scatter([], [], [], s=6, c="white",
                                edgecolors="none", zorder=10)

    margin = 1.5
    ax.set_xlim(-margin, margin)
    ax.set_ylim(-margin, margin)
    ax.set_zlim(-0.5, max_depth + 0.5)

    n_nodes = len(indices)
    base_colors = np.tile(matplotlib.colors.to_rgba(base_node_color), (n_nodes, 1))
    base_sizes = np.full(n_nodes, base_node_size)

    def frame_state(t):
        """Returns (facecolors, sizes, edge_colors, glint_xyz) for time
        t (seconds into the current reveal cycle)."""
        colors = base_colors.copy()
        sizes = base_sizes.copy()
        edge_colors = [base_edge_color] * len(edge_list)
        edge_index = {e: i for i, e in enumerate(edge_list)}
        glint_pts = []

        for i, (pset, eset) in enumerate(zip(proofs, proof_edge_sets)):
            activate_t = t1 + (i + 1) * proof_interval
            if t < activate_t:
                continue
            color_rgba = matplotlib.colors.to_rgba(proof_colors[i % len(proof_colors)])
            for idx in pset:
                j = index_of[idx]
                colors[j] = color_rgba
                sizes[j] = base_node_size * proof_size_mult
                if proof_cfg.get("shiny", True):
                    glint_pts.append(pos[idx])
            for e in eset:
                if e in edge_index:
                    edge_colors[edge_index[e]] = color_rgba
                elif (e[1], e[0]) in edge_index:
                    edge_colors[edge_index[(e[1], e[0])]] = color_rgba

        if t >= t1:
            j = index_of[goal_index]
            colors[j] = matplotlib.colors.to_rgba(goal_color)
            sizes[j] = base_node_size * goal_size_mult
            if goal_cfg.get("shiny", True):
                glint_pts.append(pos[goal_index])

        return colors, sizes, edge_colors, glint_pts

    def update(frame):
        t = (frame / fps) % duration
        colors, sizes, edge_colors, glint_pts = frame_state(t)
        node_scatter.set_facecolor(colors)
        node_scatter.set_sizes(sizes)
        edge_collection.set_color(edge_colors)
        if glint_pts:
            gx, gy, gz = zip(*glint_pts)
            glint_scatter._offsets3d = (gx, gy, gz)
        else:
            glint_scatter._offsets3d = ([], [], [])
        if spin_enabled:
            azim = (degrees_per_second * frame / fps) % 360
            ax.view_init(elev=elevation, azim=azim)
        return node_scatter, edge_collection, glint_scatter

    anim = animation.FuncAnimation(
        fig, update, frames=n_frames, interval=1000.0 / fps,
        blit=False, repeat=(output_path is None))

    if output_path:
        os.makedirs(os.path.dirname(os.path.abspath(output_path)) or ".",
                     exist_ok=True)
        writer_name = anim_cfg.get("writer", "ffmpeg")
        print(f"[info] rendering {n_frames} frames to {output_path} "
              f"(writer={writer_name}) ...")
        anim.save(output_path, writer=writer_name, fps=fps)
        print(f"[info] wrote {output_path}")
    else:
        plt.show()


# -----------------------------------------------------------------------
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
                     help="Override config's own animation.output_path "
                          "-- render to this file instead of showing an "
                          "interactive window.")
    args = ap.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f) or {}

    graphs_dir = args.graphs_dir or config.get("graphs_dir")
    if not graphs_dir:
        print("[ERROR] No graphs_dir given (in config or --graphs-dir).",
              file=sys.stderr)
        sys.exit(1)
    if not os.path.isfile(os.path.join(graphs_dir, "ground_nodes.json")):
        print(f"[ERROR] {graphs_dir}/ground_nodes.json not found -- run "
              f"main.py --problem <name> --save-graphs first.",
              file=sys.stderr)
        sys.exit(1)

    nodes_by_index, queries = load_ground_graph(graphs_dir)
    print(f"[info] loaded {len(nodes_by_index)} node(s), "
          f"{len(queries)} declared quer{'y' if len(queries)==1 else 'ies'} "
          f"from {graphs_dir}")

    render(nodes_by_index, queries, config,
           goal_query_override=args.goal_query,
           output_override=args.output)


if __name__ == "__main__":
    main()
