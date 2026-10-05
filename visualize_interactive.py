"""
visualize_interactive.py

Builds an interactive 3D HTML visualization of a ground graph (see
module/theory/graph_export.py's own ground_nodes.json shape) plus a
plain-text level-by-level clause listing -- called from main.py's own
--visual mode (run_visual), not meant to be run standalone.

WHAT THIS DRAWS:
  - Every node colored by its own CLAUSE FUNCTOR (do_node, poss, cond,
    z, zt, zbatt, sample_result, seq_node, ...) -- one color per
    distinct functor actually present, via a cycled qualitative
    palette, with a legend that also lets you toggle a whole functor
    group on/off (plotly's own default legend-click behavior).
  - The graph is built TOP-DOWN FROM THE ROOT(S): a root is a declared
    query node (graph_export.py's own is_query flag) that NOTHING ELSE
    in the graph points at -- see find_root_indices below for why that
    second condition matters. A problem can, and this project's own
    queries_generated.pl routinely does, declare several queries whose
    derivations happen to expand to IDENTICAL ground content (ProbLog
    stores that once, as one shared node, not as separate copies linked
    by an edge); when that shared node is ALSO reachable as a genuine
    child of another query's own proof, it is NOT a second root -- it's
    leveled at its real BFS distance like any other node, so the tree
    doesn't show a false edge "from" one top-level query "into" another
    as if one were nested inside the other's proof. Every other node's
    position then comes from a breadth-first walk DOWN from the true
    root(s) through their children, children's children, and so on,
    until the walk bottoms out at the leaves (the AD/probabilistic-fact
    draws). See compute_levels_from_root below. A node shared by more
    than one declared query still shows every query name that resolves
    to it -- see load_ground_nodes' own query_names note -- in the
    hover text and the levels file, not just whichever single name
    ProbLog's own internal tie-break left on the node's own .name.
  - Vertical position (z) is a node's own BFS distance from the root
    (root at z=0, each step down into a child adds one, so the tree
    grows upward in z toward the leaves). A node reachable from the
    root via more than one path gets its SHORTEST such distance,
    standard breadth-first behavior. Any node the walk never reaches
    at all (label_all labeled it, but it isn't actually part of the
    goal formula's own proof DAG) is pinned to z=-1, a single layer
    below the root, so it's still visible but clearly set apart from
    the real tree.
  - x/y within each such layer come from a plain GRID (see
    layered_layout_3d): nodes are simply laid out in rows/columns,
    ordered by BFS-visit order so a parent's children tend to land
    next to each other -- no force simulation, nothing that moves a
    node off its own layer. Combined with the actual parent/child
    edges drawn between layers, this is what makes the tree SHAPE
    itself legible at a glance.
  - Hovering a node writes its full clause text into a fixed bar
    pinned to the bottom of the page -- via a small injected JS
    snippet (plotly.js's own plotly_hover/plotly_unhover events),
    since plotly's own default hover tooltip alone doesn't do that.
    (An earlier version also enlarged the hovered marker via
    Plotly.restyle; that broke repeat hovering on a gl3d trace -- see
    the note beside post_script below -- so the highlight is now just
    plotly's own default hover label plus a CSS flash on the bar.)

Camera rotation is plotly's own free, built-in 3D drag/orbit -- no
extra code needed for that (unlike the visual branch's matplotlib
animation, which had to hand-roll a spin via repeated view_init calls
across saved frames; an interactive HTML page doesn't need that).
"""

import json
import math
import os
from collections import defaultdict, deque

import plotly.colors as pcolors
import plotly.graph_objects as go


def load_ground_nodes(graphs_dir):
    """Reads graphs_dir/ground_nodes.json (written by module/theory/
    graph_export.py's own export_ground_graph) and returns
    {index: node_record}, the shape every function below expects.

    Also attaches each record's own query_names: list -- EVERY declared
    query name that resolves to this exact node (ground_nodes.json's own
    "queries" dict is {query_name: node_index}, inverted here to
    index->[names]), not just whichever single name formula.to_dot()/
    label_all happened to leave on the node's own .name. More than one
    query landing on the SAME node is the normal, expected outcome of
    ProbLog's own content-addressed node sharing (two independently
    declared queries whose own derivations happen to expand to
    identical logical content get stored as the literal same node, not
    two nodes joined by an edge) -- see this module's own header and
    find_root_indices below, which uses this to avoid drawing one such
    query as if it were "inside" another's proof tree."""
    with open(os.path.join(graphs_dir, "ground_nodes.json")) as f:
        data = json.load(f)
    nodes_by_index = {rec["index"]: rec for rec in data["nodes"]}

    query_names_by_node = defaultdict(list)
    for query_name, idx in (data.get("queries") or {}).items():
        query_names_by_node[idx].append(query_name)
    for idx, rec in nodes_by_index.items():
        rec["query_names"] = sorted(query_names_by_node.get(idx, []))

    return nodes_by_index


# -----------------------------------------------------------------------
# Layout -- see this module's own header for why root-first/BFS/grid.
# -----------------------------------------------------------------------
def find_root_indices(nodes_by_index):
    """This graph's own declared query/goal-formula node(s) -- the
    root(s) the BFS below walks down from. Found via each record's own
    is_query flag (graph_export.py sets it from formula.queries()) --
    but NOT every query node is a root: if query A's own node is also a
    CHILD of query B's own node (both point at it, see load_ground_
    nodes' own query_names note -- this happens whenever two
    independently declared queries' derivations happen to expand to
    identical content, which ProbLog stores once, not twice), A is not
    a top-level root -- it's a real, direct descendant of B, and
    levelling it at 0 anyway (as an earlier version of this did) drew a
    false edge "from" B "into" A as if A were nested one level inside
    B's own proof, when the only genuine relationship is: both
    independently resolve to that one shared node. So a query only
    counts as a root here if NOTHING in the whole graph points at it;
    every other query still gets colored as is_query (build_interactive_
    html can still mark it) but is leveled by its own real BFS distance
    like any other node, and load_ground_nodes' own query_names list is
    what lets a shared node display ALL the query names that resolve to
    it, not just whichever one ProbLog's own internal tie-break left on
    .name."""
    query_nodes = sorted(idx for idx, rec in nodes_by_index.items() if rec.get("is_query"))
    if not query_nodes:
        # No declared query on this graph (shouldn't normally happen --
        # the theory this export runs against always declares one --
        # but guarded so the walk still has somewhere to start rather
        # than levelling nothing): fall back to every node nothing else
        # points to as a child, i.e. every node with no parent.
        all_children = set()
        for rec in nodes_by_index.values():
            for c in (rec.get("children") or []):
                if c != 0:
                    all_children.add(abs(c))
        return sorted(idx for idx in nodes_by_index if idx not in all_children)

    has_incoming_edge = set()
    for rec in nodes_by_index.values():
        for c in (rec.get("children") or []):
            if c != 0:
                has_incoming_edge.add(abs(c))

    true_roots = [idx for idx in query_nodes if idx not in has_incoming_edge]
    # Degenerate fallback (shouldn't happen in an acyclic proof DAG --
    # every query would have to be someone else's descendant, with
    # nothing left un-dominated): rather than level nothing, fall back
    # to the original "every query is a root" behavior.
    return true_roots if true_roots else query_nodes


def compute_levels_from_root(nodes_by_index):
    """Breadth-first walk DOWN from the root(s) (see find_root_indices)
    through each node's own children -- root(s) at level 0, each step
    into a child adds one level, continuing until the walk bottoms out
    at the leaves. A node reachable via more than one path gets its
    SHORTEST (BFS) distance. Returns (levels, order): levels maps
    index -> int (root=0, climbing toward the leaves; -1 for any node
    the walk never reaches at all -- see this module's own header),
    order maps index -> its own position in BFS-visit order (used by
    layered_layout_3d to keep a parent's children next to each other
    within a grid layer, instead of an arbitrary index sort)."""
    roots = find_root_indices(nodes_by_index)

    levels = {}
    order = {}
    counter = 0
    queue = deque()
    for r in roots:
        if r in nodes_by_index and r not in levels:
            levels[r] = 0
            order[r] = counter
            counter += 1
            queue.append(r)
    while queue:
        idx = queue.popleft()
        rec = nodes_by_index[idx]
        for c in (rec.get("children") or []):
            if c == 0:
                continue
            cidx = abs(c)
            if cidx in nodes_by_index and cidx not in levels:
                levels[cidx] = levels[idx] + 1
                order[cidx] = counter
                counter += 1
                queue.append(cidx)

    for idx in sorted(nodes_by_index):
        if idx not in levels:
            levels[idx] = -1
            order[idx] = counter
            counter += 1

    return levels, order


def layered_layout_3d(nodes_by_index, grid_spacing=1.4):
    """x/y within each BFS level (see compute_levels_from_root) are a
    plain grid -- roughly square rows/columns, centered on the z-axis,
    nodes placed in BFS-visit order so a parent's children tend to
    land next to each other. No force simulation: every node stays
    exactly on its own level's z plane, which is what keeps the tree
    SHAPE (root -> branches -> leaves) legible, with the real parent/
    child edges drawn between levels doing the rest."""
    levels, order = compute_levels_from_root(nodes_by_index)

    by_level = defaultdict(list)
    for idx, lvl in levels.items():
        by_level[lvl].append(idx)

    pos = {}
    for lvl, idxs in by_level.items():
        idxs_sorted = sorted(idxs, key=lambda i: order[i])
        count = len(idxs_sorted)
        cols = max(1, math.ceil(math.sqrt(count)))
        rows = math.ceil(count / cols)
        for i, idx in enumerate(idxs_sorted):
            row, col = divmod(i, cols)
            x = (col - (cols - 1) / 2.0) * grid_spacing
            y = (row - (rows - 1) / 2.0) * grid_spacing
            pos[idx] = (x, y, float(lvl))

    max_level = max(levels.values()) if levels else 0
    return pos, max_level


# -----------------------------------------------------------------------
# Coloring -- one color per distinct clause FUNCTOR (not a grouped
# category), cycling through a 26-color qualitative palette so this
# stays readable even with this theory's real functor vocabulary
# (do_node, poss, cond, z, zt, zbatt, sample_result, sample_value,
# battery, at, seq_node, fallback_node, reactivesequence, ...).
# Unnamed nodes (bare atoms with no label_all-derived name -- shouldn't
# normally happen on a ground graph, see graph_export.py's own note,
# but guarded anyway) fall back to their own node "type" (atom/conj/
# disj) as the grouping key instead, so nothing is ever left uncolored.
# -----------------------------------------------------------------------
_PALETTE = pcolors.qualitative.Alphabet


def _group_key(rec):
    return rec.get("functor") or f"({rec['type']})"


def _color_map(nodes_by_index):
    keys = sorted({_group_key(rec) for rec in nodes_by_index.values()})
    return {key: _PALETTE[i % len(_PALETTE)] for i, key in enumerate(keys)}


def _node_label(idx, rec):
    return rec.get("name") or f"{rec['type']}(node_{idx})"


def _other_query_names(rec):
    """Every OTHER declared query name (besides this node's own .name,
    whatever ProbLog's own internal tie-break left on it) that ALSO
    resolves to this exact node -- see load_ground_nodes' own
    query_names note. Shared by _hover_text (HTML) and write_levels_file
    (plain text) so both surfaces agree on which nodes are shared across
    more than one query."""
    return [q for q in (rec.get("query_names") or []) if q != rec.get("name")]


def _hover_text(idx, rec):
    """_node_label's text, PLUS -- when this node is shared by more than
    one declared query -- every OTHER query name that also resolves to
    this exact node. A node like this isn't "query A nested inside
    query B's proof"; it's one piece of shared content that several
    independently declared queries all happen to land on, so the hover
    text says so explicitly rather than silently showing only whichever
    single name ProbLog's own internal tie-break left on the node."""
    base = _node_label(idx, rec)
    others = _other_query_names(rec)
    if others:
        return base + "\n(shared node -- also the query: " + "; also: ".join(others) + ")"
    return base


# -----------------------------------------------------------------------
# Interactive HTML
# -----------------------------------------------------------------------
def build_interactive_html(nodes_by_index, out_path, title="Ground graph"):
    pos, max_depth = layered_layout_3d(nodes_by_index)
    color_map = _color_map(nodes_by_index)

    groups = defaultdict(list)
    for idx, rec in nodes_by_index.items():
        groups[_group_key(rec)].append(idx)

    fig = go.Figure()

    # Edges first, so they render underneath the node markers.
    edge_x, edge_y, edge_z = [], [], []
    for idx, rec in nodes_by_index.items():
        x0, y0, z0 = pos[idx]
        for c in (rec.get("children") or []):
            if c == 0:
                continue
            cidx = abs(c)
            if cidx in pos:
                x1, y1, z1 = pos[cidx]
                edge_x += [x0, x1, None]
                edge_y += [y0, y1, None]
                edge_z += [z0, z1, None]
    fig.add_trace(go.Scatter3d(
        x=edge_x, y=edge_y, z=edge_z, mode="lines",
        line=dict(color="rgba(70,70,70,0.75)", width=3),
        hoverinfo="skip", showlegend=False, name="edges"))

    # One trace per functor group -- gives a free, click-to-toggle
    # legend, and lets the hover JS below tell groups apart by
    # gd.data[curveNumber] alone.
    for key in sorted(groups):
        idxs = groups[key]
        xs = [pos[i][0] for i in idxs]
        ys = [pos[i][1] for i in idxs]
        zs = [pos[i][2] for i in idxs]
        texts = [_hover_text(i, nodes_by_index[i]) for i in idxs]
        sizes = [7.0] * len(idxs)
        fig.add_trace(go.Scatter3d(
            x=xs, y=ys, z=zs, mode="markers",
            marker=dict(size=sizes, color=color_map[key],
                        line=dict(width=0)),
            text=texts, hovertemplate="%{text}<extra></extra>",
            name=f"{key} ({len(idxs)})"))

    fig.update_layout(
        title=title,
        scene=dict(
            xaxis=dict(visible=False), yaxis=dict(visible=False),
            zaxis=dict(title="BFS distance from the goal/query node (0 = root)"),
            bgcolor="white"),
        paper_bgcolor="white",
        legend=dict(itemsizing="constant"),
        margin=dict(l=0, r=0, t=40, b=0),
    )

    div_id = "ground-graph-div"
    # plotly_hover/plotly_unhover: plotly.js's own events, fired on
    # every marker-trace point (edges are hoverinfo="skip" above, so
    # they never trigger this). data.points[0].text mirrors that
    # point's own hovertemplate text (the full clause label set above).
    #
    # IMPORTANT, confirmed directly (not assumed): an EARLIER version of
    # this also called Plotly.restyle(gd, {'marker.size': ...}, ...) on
    # hover, to visually enlarge the hovered point. That broke repeat
    # hovering outright -- restyling a gl3d/WebGL trace (Scatter3d,
    # which is what every node trace here is) forces a full scene
    # rebuild, not the cheap incremental update a 2D/SVG trace gets, and
    # doing that from inside a plotly_hover handler tears down the
    # picking state: the FIRST hover works, every hover after that never
    # fires again. Verified by removing the restyle call and confirming
    # hover then switches reliably across many different points in a
    # row, whereas keeping it reproduces the freeze every time. So: no
    # marker-resizing here -- the "highlight" is plotly's OWN default
    # hover label (already drawn, free, and doesn't touch trace data) --
    # this handler only ever does plain DOM text updates, which stay
    # reliable because they never call back into plotly at all.
    # The readout BAR itself flashes on hover (a pure CSS class toggle,
    # no plotly call involved) as the "highlight" signal, since the
    # marker itself can't safely be resized -- see the note above.
    post_script = f"""
(function() {{
    var gd = document.getElementById('{div_id}');
    var bar = document.getElementById('clause-readout-bar');
    var readout = document.getElementById('clause-readout');
    gd.on('plotly_hover', function(data) {{
        var pt = data.points && data.points[0];
        if (!pt || typeof pt.text === 'undefined') return;
        readout.textContent = pt.text;
        bar.classList.add('hit');
    }});
    gd.on('plotly_unhover', function(data) {{
        readout.textContent = '(hover a node to see its clause)';
        bar.classList.remove('hit');
    }});
}})();
"""

    # include_plotlyjs=True bundles the full plotly.js library INLINE
    # (adds a few MB to the file) rather than "cdn" (smaller file, but
    # requires network access to cdn.plot.ly every time it's opened --
    # confirmed directly this fails outright on a restricted network/
    # offline machine). Self-contained is the right default for a file
    # meant to be opened later, possibly shared, possibly offline.
    plot_html = fig.to_html(full_html=False, include_plotlyjs=True,
                             div_id=div_id, post_script=post_script)

    page = f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>{title}</title>
<style>
  html, body {{ margin: 0; padding: 0; background: white;
    font-family: -apple-system, Helvetica, Arial, sans-serif; }}
  #plot-wrap {{ padding-bottom: 56px; }}
  #clause-readout-bar {{
    position: fixed; left: 0; right: 0; bottom: 0;
    background: #1e1e1e; color: #eaeaea;
    padding: 10px 16px; font-family: "SF Mono", Menlo, Consolas, monospace;
    font-size: 13px; border-top: 2px solid #444; z-index: 1000;
    white-space: pre-wrap; word-break: break-word;
    transition: background 0.1s, border-color 0.1s;
  }}
  #clause-readout-bar.hit {{ background: #103a1e; border-top-color: #2ecc71; }}
  #clause-readout-bar b {{ color: #9ad; margin-right: 8px; }}
</style>
</head>
<body>
<div id="clause-readout-bar"><b>Clause:</b><span id="clause-readout">(hover a node to see its clause)</span></div>
<div id="plot-wrap">{plot_html}</div>
</body>
</html>
"""
    with open(out_path, "w") as f:
        f.write(page)


# -----------------------------------------------------------------------
# Level-by-level plain-text listing
# -----------------------------------------------------------------------
def write_levels_file(nodes_by_index, out_path):
    """Same levels build_interactive_html's grid layout uses (BFS
    distance from the goal/query root, see compute_levels_from_root) --
    level 0 is the root itself, climbing toward the leaves; any node
    the walk never reached (not part of the goal formula's own proof
    DAG) is listed last, under its own "unreached" heading rather than
    a numbered level."""
    levels, _order = compute_levels_from_root(nodes_by_index)
    by_level = defaultdict(list)
    for idx, lvl in levels.items():
        by_level[lvl].append(idx)
    max_level = max((lvl for lvl in levels.values() if lvl >= 0), default=-1)

    def _line(idx, rec):
        others = _other_query_names(rec)
        suffix = f"  [shared with: {', '.join(others)}]" if others else ""
        return f"  [{idx}] {_node_label(idx, rec)}{suffix}"

    lines = []
    for lvl in range(0, max_level + 1):
        idxs = sorted(by_level.get(lvl, []))
        heading = "Root (goal/query node)" if lvl == 0 else f"Level {lvl}"
        lines.append(f"=== {heading} ({len(idxs)} node(s)) ===")
        for idx in idxs:
            lines.append(_line(idx, nodes_by_index[idx]))
        lines.append("-" * 70)

    unreached = sorted(by_level.get(-1, []))
    if unreached:
        lines.append(f"=== Unreached from goal query ({len(unreached)} node(s)) ===")
        for idx in unreached:
            lines.append(_line(idx, nodes_by_index[idx]))
        lines.append("-" * 70)

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
