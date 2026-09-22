"""
graph_viz_common.py

Shared plumbing for visualize_ground_graph.py and
visualize_compiled_graph.py: loading a graph_export.py JSON sidecar,
the layered 3D layout, proof enumeration (two variants -- see below),
and the actual spinning-3D-point-cloud renderer/animator. Kept in one
place so the two graph-specific scripts stay thin (load this graph's
own JSON, pick a proof-enumeration variant appropriate to THIS graph,
call render()) instead of duplicating ~250 lines of matplotlib/
animation plumbing twice.

TWO PROOF-ENUMERATION VARIANTS, because the ground graph and the
compiled graph mean different things by "a proof reaching a leaf":

  - enumerate_proofs/proof_edges (SIGN-AGNOSTIC): used for the ground
    graph. A ground LogicFormula's own conj/disj children are mostly
    positive references to other derived predicate-call nodes (do_node
    /cond/battery/...), with the occasional negative child for a
    genuinely negated subgoal -- but the ground graph isn't a
    deterministic circuit, so "positive vs negative child" doesn't
    carry the same "is this leaf literally asserted true along this
    world" meaning it does after compilation (see the next variant).
    This variant just walks abs(child) regardless of sign, exactly as
    designed/discussed for the ground-graph visualizer.

  - enumerate_true_leaf_proofs/true_proof_edges (SIGN-AWARE): used for
    the COMPILED graph. Compilation makes the circuit deterministic --
    every disj node's children are mutually exclusive alternatives,
    and negative children (e.g. conj(children=(-8,-5))) genuinely mean
    "this leaf is FALSE along this branch", confirmed directly by
    inspecting real dsharp/DDNNF output (see graph_export.py's own
    COMPILED GRAPH NODE NAMING note). Highlighting "paths to the TRUE
    leaves only" therefore has to track sign: a node is only included
    in a proof's highlighted set if the walk reaches it via at least
    one POSITIVE reference; a leaf reached only through negative
    references (i.e. only ever asserted false along this proof) is
    left uncolored, and an edge is only colored if it is itself a
    positive reference between two included nodes.
"""

import json
import math
import os
import sys
from collections import defaultdict

import matplotlib
import numpy as np


# -----------------------------------------------------------------------
# Loading
# -----------------------------------------------------------------------
def load_graph(graphs_dir, filename):
    """filename: "ground_nodes.json" or "compiled_nodes.json" (both
    written by module/theory/graph_export.py with the same schema --
    see that module's own header). Returns (nodes_by_index, queries)."""
    path = os.path.join(graphs_dir, filename)
    with open(path) as f:
        data = json.load(f)
    nodes_by_index = {rec["index"]: rec for rec in data["nodes"]}
    queries = data["queries"]
    return nodes_by_index, queries


def pick_goal_index(nodes_by_index, queries, goal_query):
    """goal_query: an exact key of `queries`, or None -- in which case
    "verify_goal_formula" is used if present (the canonical top-level
    goal query in this project's own theory), else the alphabetically-
    first declared query."""
    if goal_query is not None:
        if goal_query not in queries:
            raise KeyError(
                f"goal_query {goal_query!r} is not one of this graph's own "
                f"declared queries: {sorted(queries)}")
        return queries[goal_query], goal_query
    if "verify_goal_formula" in queries:
        return queries["verify_goal_formula"], "verify_goal_formula"
    if not queries:
        raise ValueError("This graph's own JSON has no declared queries at all.")
    name = sorted(queries)[0]
    return queries[name], name


# -----------------------------------------------------------------------
# Proof enumeration -- SIGN-AGNOSTIC variant (ground graph)
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
    if not children:
        yield chosen
        return
    first, rest = children[0], children[1:]
    for partial in _iter_proofs(nodes_by_index, first, chosen, budget):
        yield from _iter_conj(nodes_by_index, rest, partial, budget)


def enumerate_proofs(nodes_by_index, root_index, n, max_search_steps=200000):
    """Up to n distinct proof node-index-sets for root_index, sign-
    agnostic (see this module's own header). Stops early, with a
    warning to stderr, if the search budget is exhausted first."""
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
              f"{max_search_steps}-step search budget", file=sys.stderr)
    return proofs


def proof_edges(nodes_by_index, proof_node_set):
    """The subset of the full edge list whose both endpoints are in
    proof_node_set (sign-agnostic -- see this module's own header)."""
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
# Proof enumeration -- SIGN-AWARE variant (compiled graph: "paths to
# the TRUE leaves only" -- see this module's own header)
# -----------------------------------------------------------------------
def _iter_proofs_signed(nodes_by_index, index, chosen, budget):
    """Like _iter_proofs, but `chosen` is {idx: sign} (sign=+1 if this
    proof has reached idx via at least one POSITIVE reference so far,
    -1 if only ever negative) instead of a plain set of indices --
    positive is kept if a node is reached both ways in the same proof,
    since "true leaves only" cares whether a POSITIVE path exists, not
    whether every occurrence was positive."""
    sign = 1 if index > 0 else -1
    idx = abs(index)
    if idx in chosen:
        if sign > chosen[idx]:
            chosen = dict(chosen)
            chosen[idx] = sign
        yield chosen
        return
    budget["steps"] -= 1
    if budget["steps"] <= 0:
        return
    node = nodes_by_index.get(idx)
    new_chosen = dict(chosen)
    new_chosen[idx] = sign
    if node is None:
        yield new_chosen
        return
    children = [c for c in (node.get("children") or []) if c != 0]
    if node["type"] == "atom" or not children:
        yield new_chosen
        return
    if node["type"] == "conj":
        yield from _iter_conj_signed(nodes_by_index, children, new_chosen, budget)
    elif node["type"] == "disj":
        for c in children:
            yield from _iter_proofs_signed(nodes_by_index, c, new_chosen, budget)
    else:
        raise TypeError(f"Unexpected node type: {node['type']!r}")


def _iter_conj_signed(nodes_by_index, children, chosen, budget):
    if not children:
        yield chosen
        return
    first, rest = children[0], children[1:]
    for partial in _iter_proofs_signed(nodes_by_index, first, chosen, budget):
        yield from _iter_conj_signed(nodes_by_index, rest, partial, budget)


def enumerate_true_leaf_proofs(nodes_by_index, root_index, n, max_search_steps=200000):
    """Up to n distinct proofs, each returned as the set of node
    indices reached via at least one POSITIVE reference (see
    _iter_proofs_signed) -- i.e. the nodes actually asserted TRUE along
    that derivation, INCLUDING the query root itself. A proof is only
    kept if it contains at least one positively-reached ATOM (a real
    "true leaf") -- a branch built entirely from negative references
    (asserting only that some leaves are false) doesn't establish a
    true leaf and is skipped, since it has nothing to highlight under
    "paths to the true leaves only"."""
    budget = {"steps": max_search_steps}
    proofs = []
    for signed in _iter_proofs_signed(nodes_by_index, root_index, {}, budget):
        positive_nodes = {idx for idx, s in signed.items() if s > 0}
        has_true_leaf = any(
            nodes_by_index.get(idx, {}).get("type") == "atom"
            for idx in positive_nodes)
        if not has_true_leaf:
            continue
        if positive_nodes not in proofs:
            proofs.append(positive_nodes)
        if len(proofs) >= n:
            break
    if len(proofs) < n:
        print(f"[warn] enumerate_true_leaf_proofs: only found {len(proofs)}/{n} "
              f"distinct true-leaf proofs before exhausting the graph or the "
              f"{max_search_steps}-step search budget", file=sys.stderr)
    return proofs


def true_proof_edges(nodes_by_index, positive_nodes):
    """Edges between two positively-included nodes, using ONLY the
    positive (non-negated) reference between them -- a negative
    reference to a node that happens to ALSO be positively reached via
    a different edge in this same proof is deliberately not colored,
    since that specific edge still means "false" (see this module's
    own header)."""
    edges = set()
    for idx in positive_nodes:
        node = nodes_by_index[idx]
        for c in (node.get("children") or []):
            if c > 0 and c in positive_nodes:
                edges.add((idx, c))
    return edges


# -----------------------------------------------------------------------
# Layout -- see visualize_ground_graph.py's original note on why this
# is a hand-rolled O(n) layered/ring placement rather than a generic
# force-directed one.
# -----------------------------------------------------------------------
def compute_depths(nodes_by_index):
    depth = {}
    visiting = set()

    def get_depth(idx):
        if idx in depth:
            return depth[idx]
        if idx in visiting:
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
# Rendering / animation -- graph-agnostic: the caller has already
# picked the goal node and computed its own proof node-sets/edge-sets
# (via whichever enumeration variant fits that graph); this just draws
# and animates them.
# -----------------------------------------------------------------------
def _resolve_output_backend(output_path):
    if output_path is not None:
        matplotlib.use("Agg")


def render(nodes_by_index, goal_index, goal_name, proofs, proof_edge_sets,
           config, output_path=None):
    from matplotlib import animation
    import matplotlib.pyplot as plt

    print(f"[info] goal query: {goal_name} (node {goal_index})")
    print(f"[info] {len(proofs)} proof(s) to animate "
          f"(sizes: {[len(p) for p in proofs]})")

    anim_cfg = config.get("animation", {})
    _resolve_output_backend(output_path)

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

    proof_cfg = config.get("proof_paths", {})
    proof_colors = proof_cfg.get(
        "colors", ["orange", "magenta", "cyan", "lime", "indigo", "gold"])
    proof_interval = float(proof_cfg.get("interval_seconds", 2.0))
    proof_size_mult = float(proof_cfg.get("size_multiplier", 1.8))
    proof_shiny = bool(proof_cfg.get("shiny", True))
    goal_shiny = bool(goal_cfg.get("shiny", True))

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
    edge_index = {e: i for i, e in enumerate(edge_list)}

    def frame_state(t):
        colors = base_colors.copy()
        sizes = base_sizes.copy()
        edge_colors = [base_edge_color] * len(edge_list)
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
                if proof_shiny:
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
            if goal_shiny:
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


def resolve_output_path(config, cli_output, cli_output_dir, default_basename):
    """Shared "where does this script's own video go" logic for both
    visualize_ground_graph.py and visualize_compiled_graph.py, so both
    videos land in the SAME common directory by default:
      1. --output (cli_output): an exact file path -- highest priority.
      2. --output-dir (cli_output_dir), or else config's own top-level
         output_dir: a directory shared by BOTH scripts; this script's
         own video is written there as default_basename + the right
         extension for animation.writer (ffmpeg->.mp4, pillow->.gif).
      3. Neither given: None -- interactive plt.show().
    """
    if cli_output:
        return cli_output
    out_dir = cli_output_dir or config.get("output_dir")
    if not out_dir:
        return None
    writer = config.get("animation", {}).get("writer", "ffmpeg")
    ext = "gif" if writer == "pillow" else "mp4"
    return os.path.join(out_dir, f"{default_basename}.{ext}")
