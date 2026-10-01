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
  - Vertical position (z) is each node's own DERIVATION DEPTH (leaves
    -- the AD facts -- at z=0, climbing toward whatever sits deepest
    in the proof), via layered_layout_3d below -- ported unchanged
    from the visual branch's own graph_viz_common.py, since it's
    already backend/rendering-agnostic (just node index -> (x,y,z)).
    This is what makes different clause KINDS (an early z(...) draw
    vs. a much-later poss(haltMoveto(...))) visually separate by
    height, not just by color.
  - Hovering a node highlights it (its own marker grows) AND writes
    its full clause text into a fixed bar pinned to the bottom of the
    page -- via a small injected JS snippet (plotly.js's own
    plotly_hover/plotly_unhover events), since plotly's own default
    hover tooltip alone doesn't do either of those.

Camera rotation is plotly's own free, built-in 3D drag/orbit -- no
extra code needed for that (unlike the visual branch's matplotlib
animation, which had to hand-roll a spin via repeated view_init calls
across saved frames; an interactive HTML page doesn't need that).
"""

import json
import math
import os
import sys
from collections import defaultdict

import plotly.colors as pcolors
import plotly.graph_objects as go


def load_ground_nodes(graphs_dir):
    """Reads graphs_dir/ground_nodes.json (written by module/theory/
    graph_export.py's own export_ground_graph) and returns
    {index: node_record}, the shape every function below expects."""
    with open(os.path.join(graphs_dir, "ground_nodes.json")) as f:
        data = json.load(f)
    return {rec["index"]: rec for rec in data["nodes"]}


# -----------------------------------------------------------------------
# Layout -- see this module's own header for why layered-by-depth,
# ported unchanged from graph_viz_common.py (visual branch).
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
        line=dict(color="rgba(110,110,110,0.35)", width=1),
        hoverinfo="skip", showlegend=False, name="edges"))

    # One trace per functor group -- gives a free, click-to-toggle
    # legend, and lets the hover JS below tell groups apart by
    # gd.data[curveNumber] alone.
    for key in sorted(groups):
        idxs = groups[key]
        xs = [pos[i][0] for i in idxs]
        ys = [pos[i][1] for i in idxs]
        zs = [pos[i][2] for i in idxs]
        texts = [_node_label(i, nodes_by_index[i]) for i in idxs]
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
            zaxis=dict(title="derivation depth (0 = leaves/AD facts)"),
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
    depths = compute_depths(nodes_by_index)
    by_depth = defaultdict(list)
    for idx, d in depths.items():
        by_depth[d].append(idx)
    max_depth = max(depths.values()) if depths else 0

    lines = []
    for d in range(0, max_depth + 1):
        idxs = sorted(by_depth.get(d, []))
        lines.append(f"=== Level {d} ({len(idxs)} node(s)) ===")
        for idx in idxs:
            rec = nodes_by_index[idx]
            lines.append(f"  [{idx}] {_node_label(idx, rec)}")
        lines.append("-" * 70)

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
