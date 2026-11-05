#!/usr/bin/env python3
"""
visualize_progression.py

Draws the execution graph progression.py writes (..._graph.json) as one
self-contained HTML page: pan/zoom in a browser, no network needed.

WHAT THIS DRAWS:
  - Time runs left to right. A node is a configuration (progressed state
    + what is left of the BT), placed in the column of its longest path
    from the root, so an edge always points right.
  - An edge is one BT leaf. Its label is the CONDITIONAL probability
    P(child | parent), summed over the noise draws that lead there. p = 1
    labels are off by default, since most leaves are deterministic. Edge
    width follows the absolute mass through it, reach(parent) * p.
  - Node size follows reach, its total probability. A state shows the
    code of the leaf it runs next; a final state shows how the plan ended.
  - Hover anything for its details; click a node to keep only the paths
    through it. The table lists the final states, largest reach first.

    python3 visualize_progression.py --problem problem0S
    python3 visualize_progression.py --problem problem0S --merge none
    python3 visualize_progression.py --graph some_graph.json --output out.html
"""

import argparse
import json
import math
import os
import re
import sys
from collections import defaultdict, deque

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_ROOT = os.path.join(ROOT_DIR, "output_progression")

COLUMN_GAP = 170
ROW_GAP = 46
MIN_RADIUS = 6
MAX_RADIUS = 16
MIN_WIDTH = 1.0
MAX_WIDTH = 6.0
LAYOUT_SWEEPS = 8

_CODE = re.compile(r"\b([ac]\d+)\b")


def leaf_code(text):
    """a2 from moveto_leg(a2), c1 from cond(c1: ...); text if neither."""
    match = _CODE.search(text or "")
    return match.group(1) if match else (text or "")


def final_class(node):
    """(css class, icon, label) of a final state."""
    outcome = node.get("outcome")
    if node.get("goal"):
        return "goal", "✓", "goal reached"
    if outcome == "true":
        return "open", "–", "BT succeeded, goal not reached"
    if outcome == "false":
        return "failed", "✗", "BT failed"
    return "other", "!", outcome or "unknown"


def longest_path_layers(nodes, edges):
    outgoing, indegree = defaultdict(list), {n["id"]: 0 for n in nodes}
    for edge in edges:
        outgoing[edge["source"]].append(edge["target"])
        indegree[edge["target"]] += 1
    layer = {n["id"]: 0 for n in nodes}
    queue = deque(n for n, degree in indegree.items() if degree == 0)
    seen = 0
    while queue:
        current = queue.popleft()
        seen += 1
        for target in outgoing[current]:
            layer[target] = max(layer[target], layer[current] + 1)
            indegree[target] -= 1
            if indegree[target] == 0:
                queue.append(target)
    if seen != len(nodes):
        raise ValueError("graph has a cycle")
    return layer


def place_rows(columns, neighbours, position):
    """Moves each column's nodes towards the mean of their neighbours,
    keeping order and ROW_GAP."""
    for column in columns:
        wanted = {}
        for n in column:
            around = [position[m] for m in neighbours[n] if m in position]
            wanted[n] = sum(around) / len(around) if around else position[n]
        column.sort(key=lambda n: wanted[n])
        placed = []
        for n in column:
            y = wanted[n]
            if placed:
                y = max(y, placed[-1] + ROW_GAP)
            placed.append(y)
        # Pushing only downwards drifts the column: recentre it.
        shift = sum(wanted[n] - y for n, y in zip(column, placed)) / len(column)
        for n, y in zip(column, placed):
            position[n] = y + shift


def layout(nodes, edges):
    """{id: (x, y)}: longest-path columns, rows from barycentre sweeps."""
    layer = longest_path_layers(nodes, edges)
    columns = defaultdict(list)
    for node in nodes:
        columns[layer[node["id"]]].append(node["id"])
    ordered = [columns[i] for i in sorted(columns)]

    parents, children = defaultdict(list), defaultdict(list)
    for edge in edges:
        parents[edge["target"]].append(edge["source"])
        children[edge["source"]].append(edge["target"])

    position = {}
    for column in ordered:
        for row, n in enumerate(column):
            position[n] = row * ROW_GAP
    for _ in range(LAYOUT_SWEEPS):
        place_rows(ordered[1:], parents, position)
        place_rows(list(reversed(ordered[:-1])), children, position)
    place_rows(ordered[1:], parents, position)

    top = min(position.values())
    return {n: (layer[n] * COLUMN_GAP, y - top) for n, y in position.items()}


def build_view(graph, title):
    """The JSON the page draws: geometry and text, nothing to compute."""
    nodes, edges = graph["nodes"], graph["edges"]
    labels = graph.get("labels", {})
    by_id = {n["id"]: n for n in nodes}
    coordinates = layout(nodes, edges)

    largest_flow = max((by_id[e["source"]]["reach"] * e["probability"] for e in edges),
                       default=1.0) or 1.0
    view_nodes = []
    for node in nodes:
        x, y = coordinates[node["id"]]
        reach = node["reach"]
        entry = {
            "id": node["id"], "x": round(x, 1), "y": round(y, 1), "reach": reach,
            "r": round(MIN_RADIUS + (MAX_RADIUS - MIN_RADIUS) * math.sqrt(reach), 1),
        }
        if node["kind"] == "root":
            entry.update(cls="root", icon="", text="start", rows=[["Node", "start"]])
        else:
            rows = [["Node", f"#{node['id']}"],
                    ["Position", f"({node['x']}, {node['y']})"],
                    ["Time", node["time"]],
                    ["Battery anchor", " at t=".join(reversed(node["battery_anchor"]))],
                    ["Hitch", node["hitch"]],
                    ["zbatt draw", node["zbatt"]]]
            if node["kind"] == "final":
                cls, icon, status = final_class(node)
                rows.insert(1, ["Result", status])
                rows.append(["Halt reasons", "; ".join(node.get("reasons", [])) or "none"])
                entry.update(cls=cls, icon=icon, text=node.get("outcome", ""),
                             status=status, rows=rows)
            else:
                code = leaf_code(node["next_leaf"])
                rows.insert(1, ["Next leaf", labels.get(code, node["next_leaf"])])
                entry.update(cls="state", icon="", text=code, rows=rows)
        view_nodes.append(entry)

    view_edges = []
    for edge in edges:
        flow = by_id[edge["source"]]["reach"] * edge["probability"]
        code = leaf_code(edge["action"])
        rows = [["Leaf", labels.get(code, edge["action"])],
                ["Status", edge["status"]]]
        if edge.get("reason"):
            rows.append(["Halt reason", edge["reason"]])
        rows.append(["Noise draws", str(len(edge.get("draws", [])))])
        view_edges.append({
            "s": edge["source"], "t": edge["target"], "p": edge["probability"],
            "flow": flow, "rows": rows,
            "w": round(MIN_WIDTH + (MAX_WIDTH - MIN_WIDTH) * math.sqrt(flow / largest_flow), 2),
        })

    finals = [n for n in view_nodes if "status" in n]
    return {
        "title": title, "nodes": view_nodes, "edges": view_edges,
        "goal": sum(n["reach"] for n in finals if n["cls"] == "goal"),
        "finals": len(finals),
    }


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
:root {
  color-scheme: light;
  --surface: #fcfcfb; --panel: #f4f3ef; --ink: #0b0b0b; --ink-2: #52514e;
  --muted: #898781; --line: #c3c2b7; --hairline: #e1e0d9; --accent: #2a78d6;
  --good: #0ca30c; --serious: #ec835a; --critical: #d03b3b; --warning: #fab219;
}
@media (prefers-color-scheme: dark) {
  :root:where(:not([data-theme="light"])) {
    color-scheme: dark;
    --surface: #1a1a19; --panel: #232322; --ink: #f0efec; --ink-2: #c3c2b7;
    --line: #4a4a46; --hairline: #2c2c2a; --accent: #3987e5;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --surface: #1a1a19; --panel: #232322; --ink: #f0efec; --ink-2: #c3c2b7;
  --line: #4a4a46; --hairline: #2c2c2a; --accent: #3987e5;
}
* { box-sizing: border-box; }
html, body { margin: 0; height: 100%; }
body {
  display: flex; flex-direction: column; background: var(--surface); color: var(--ink);
  font: 14px/1.45 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
}
header { padding: 14px 16px 10px; border-bottom: 1px solid var(--hairline); }
h1 { margin: 0; font-size: 18px; font-weight: 600; }
.stats { margin: 2px 0 8px; color: var(--ink-2); }
.stats b { color: var(--ink); font-weight: 600; font-variant-numeric: tabular-nums; }
.bar { display: flex; flex-wrap: wrap; gap: 6px 16px; align-items: center; }
.legend { display: flex; flex-wrap: wrap; gap: 6px 14px; color: var(--ink-2); font-size: 13px; }
.legend span { display: inline-flex; align-items: center; gap: 5px; }
.dot {
  width: 16px; height: 16px; border-radius: 50%; display: inline-grid; place-items: center;
  color: #fff; font-size: 11px; font-weight: 700; line-height: 1;
}
.dot.state { background: var(--surface); box-shadow: inset 0 0 0 1.5px var(--ink-2); }
.dot.goal { background: var(--good); } .dot.failed { background: var(--critical); }
.dot.open { background: var(--serious); } .dot.other { background: var(--warning); color: #0b0b0b; }
.controls { margin-left: auto; display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
label { color: var(--ink-2); font-size: 13px; display: inline-flex; gap: 5px; align-items: center; }
button {
  font: inherit; font-size: 13px; color: var(--ink); background: var(--panel);
  border: 1px solid var(--line); border-radius: 6px; padding: 3px 10px; cursor: pointer;
}
main { flex: 1; min-height: 0; display: flex; }
#stage { flex: 1; min-width: 0; touch-action: none; cursor: grab; position: relative; overflow: hidden; }
#stage.dragging { cursor: grabbing; }
svg { display: block; width: 100%; height: 100%; }
aside {
  width: 300px; flex: none; border-left: 1px solid var(--hairline); overflow: auto;
  padding: 12px 16px; background: var(--surface);
}
aside h2 { font-size: 14px; font-weight: 600; margin: 0 0 8px; }
.hint { color: var(--muted); }
dl { margin: 0; display: grid; grid-template-columns: auto 1fr; gap: 3px 10px; }
dt { color: var(--ink-2); } dd { margin: 0; overflow-wrap: anywhere; }
details { margin-top: 14px; }
summary { cursor: pointer; font-weight: 600; }
table { width: 100%; border-collapse: collapse; margin-top: 8px; font-size: 13px; }
th { text-align: left; color: var(--ink-2); font-weight: 500; }
th, td { padding: 3px 6px 3px 0; border-bottom: 1px solid var(--hairline); }
td.num, th.num { text-align: right; font-variant-numeric: tabular-nums; padding-right: 0; }
tbody tr { cursor: pointer; }
tbody tr:hover { background: var(--panel); }
.edge { fill: none; stroke: var(--line); }
.edge-hit { fill: none; stroke: transparent; stroke-width: 12; cursor: pointer; }
.edge.on { stroke: var(--accent); }
.edge-label {
  font-size: 11px; fill: var(--ink-2); text-anchor: middle; font-variant-numeric: tabular-nums;
  paint-order: stroke; stroke: var(--surface); stroke-width: 4px; pointer-events: none;
}
.node { cursor: pointer; }
.node circle { stroke: var(--surface); stroke-width: 2px; }
.node.state circle { fill: var(--surface); stroke: var(--ink-2); stroke-width: 1.5px; }
.node.root circle { fill: var(--ink); }
.node.goal circle { fill: var(--good); } .node.failed circle { fill: var(--critical); }
.node.open circle { fill: var(--serious); } .node.other circle { fill: var(--warning); }
.node.on circle { stroke: var(--accent); stroke-width: 3px; }
.icon { fill: #fff; font-size: 12px; font-weight: 700; text-anchor: middle; pointer-events: none; }
.node.other .icon { fill: #0b0b0b; }
.node-text {
  font-size: 11px; fill: var(--ink-2); text-anchor: middle; pointer-events: none;
  paint-order: stroke; stroke: var(--surface); stroke-width: 3px;
}
.faded { opacity: .12; }
.far .edge-label, .far .node-text { display: none; }
.no-certain .certain { display: none; }
@media (max-width: 720px) {
  main { flex-direction: column; }
  aside { width: 100%; height: 38%; border-left: 0; border-top: 1px solid var(--hairline); }
  .controls { margin-left: 0; }
}
</style>
</head>
<body>
<header>
  <h1 id="title"></h1>
  <p class="stats" id="stats"></p>
  <div class="bar">
    <div class="legend">
      <span><i class="dot state"></i>state (next leaf)</span>
      <span><i class="dot goal">&#10003;</i>goal reached</span>
      <span><i class="dot failed">&#10007;</i>BT failed</span>
      <span><i class="dot open">&ndash;</i>BT succeeded, goal not reached</span>
      <span><i class="dot other">!</i>stuck or budget ran out</span>
    </div>
    <div class="controls">
      <label><input type="checkbox" id="certain">label p = 1 edges</label>
      <button id="in" aria-label="Zoom in">+</button>
      <button id="out" aria-label="Zoom out">&minus;</button>
      <button id="fit">Fit</button>
    </div>
  </div>
</header>
<main>
  <div id="stage"><svg id="svg" role="img" aria-label="Execution graph of the progression"><g id="view"></g></svg></div>
  <aside>
    <h2 id="detail-title">Details</h2>
    <div id="detail"><p class="hint">Hover a node or an edge. Click a node to keep only the paths through it; click the background to clear. Scroll to zoom, drag to pan.</p></div>
    <details open>
      <summary>Final states</summary>
      <table><thead><tr><th>#</th><th>Result</th><th class="num">Reach</th></tr></thead><tbody id="finals"></tbody></table>
    </details>
  </aside>
</main>
<script>
const DATA = __DATA__;
const NS = "http://www.w3.org/2000/svg";
const view = document.getElementById("view"), svg = document.getElementById("svg");
const stage = document.getElementById("stage");
const byId = new Map(DATA.nodes.map(n => [n.id, n]));
const percent = p => (p * 100 >= 0.01 || p === 0) ? (p * 100).toFixed(2) + "%" : p.toExponential(2);
const prob = p => p >= 1 - 1e-12 ? "1" : p >= 0.001 ? p.toFixed(4) : p.toExponential(2);

document.title = DATA.title;
document.getElementById("title").textContent = DATA.title;
document.getElementById("stats").innerHTML =
  `P(goal) <b>${percent(DATA.goal)}</b> &middot; <b>${DATA.nodes.length - 1}</b> states &middot; ` +
  `<b>${DATA.finals}</b> final &middot; <b>${DATA.edges.length}</b> edges`;

function element(name, attributes, parent) {
  const node = document.createElementNS(NS, name);
  for (const key in attributes) node.setAttribute(key, attributes[key]);
  parent.appendChild(node);
  return node;
}

function showDetail(title, rows) {
  document.getElementById("detail-title").textContent = title;
  const list = document.createElement("dl");
  for (const [key, value] of rows) {
    list.appendChild(Object.assign(document.createElement("dt"), {textContent: key}));
    list.appendChild(Object.assign(document.createElement("dd"), {textContent: value}));
  }
  document.getElementById("detail").replaceChildren(list);
}

function showNode(n) {
  showDetail(n.status || "State", [["Reach", percent(n.reach)], ...n.rows]);
}

const outgoing = new Map(), incoming = new Map();
const edgeLayer = element("g", {}, view), labelLayer = element("g", {}, view), nodeLayer = element("g", {}, view);

for (const e of DATA.edges) {
  const a = byId.get(e.s), b = byId.get(e.t), bend = (b.x - a.x) * 0.5;
  const d = `M${a.x},${a.y} C${a.x + bend},${a.y} ${b.x - bend},${b.y} ${b.x},${b.y}`;
  e.line = element("path", {class: "edge", d, "stroke-width": e.w}, edgeLayer);
  e.hit = element("path", {class: "edge-hit", d}, edgeLayer);
  const certain = e.p >= 1 - 1e-12;
  // The curve at t = 0.75, where a fan of edges has spread out.
  e.label = element("text", {class: "edge-label" + (certain ? " certain" : ""),
                             x: a.x + (b.x - a.x) * 0.703, y: a.y + (b.y - a.y) * 0.844 + 3.5}, labelLayer);
  e.label.textContent = prob(e.p);
  e.hit.addEventListener("pointerenter", () => showDetail("Edge", [
    ["From, to", `#${e.s} → #${e.t}`], ["P(child | parent)", prob(e.p)],
    ["Mass through edge", percent(e.flow)], ...e.rows]));
  if (!outgoing.has(e.s)) outgoing.set(e.s, []);
  if (!incoming.has(e.t)) incoming.set(e.t, []);
  outgoing.get(e.s).push(e);
  incoming.get(e.t).push(e);
}

for (const n of DATA.nodes) {
  n.group = element("g", {class: "node " + n.cls, transform: `translate(${n.x},${n.y})`, tabindex: 0}, nodeLayer);
  element("circle", {r: n.r}, n.group);
  if (n.icon) element("text", {class: "icon", y: 4}, n.group).textContent = n.icon;
  element("text", {class: "node-text", y: n.r + 13}, n.group).textContent = n.text;
  n.group.addEventListener("pointerenter", () => showNode(n));
  n.group.addEventListener("click", event => { event.stopPropagation(); select(n); });
  n.group.addEventListener("keydown", event => { if (event.key === "Enter") select(n); });
}

let selected = null;
function select(n) {
  selected = selected === n ? null : n;
  const nodes = new Set(), edges = new Set();
  if (selected) {
    nodes.add(n);
    for (const [links, side] of [[incoming, "s"], [outgoing, "t"]]) {
      const queue = [n];
      while (queue.length) {
        for (const e of links.get(queue.pop().id) || []) {
          edges.add(e);
          const next = byId.get(e[side]);
          if (!nodes.has(next)) { nodes.add(next); queue.push(next); }
        }
      }
    }
    showNode(n);
  }
  for (const m of DATA.nodes) {
    m.group.classList.toggle("faded", !!selected && !nodes.has(m));
    m.group.classList.toggle("on", m === selected);
  }
  for (const e of DATA.edges) {
    const off = !!selected && !edges.has(e);
    e.line.classList.toggle("faded", off);
    e.label.classList.toggle("faded", off);
    e.line.classList.toggle("on", !!selected && !off);
  }
}

const finals = document.getElementById("finals");
for (const n of DATA.nodes.filter(m => m.status).sort((a, b) => b.reach - a.reach)) {
  const row = finals.insertRow();
  row.insertCell().textContent = n.id;
  row.insertCell().textContent = `${n.icon} ${n.status}`;
  const reach = row.insertCell();
  reach.className = "num";
  reach.textContent = percent(n.reach);
  row.addEventListener("click", () => { if (selected !== n) select(n); centre(n); });
}

let scale = 1, tx = 0, ty = 0;
const LABEL_SCALE = 0.45;
function apply() {
  view.setAttribute("transform", `translate(${tx},${ty}) scale(${scale})`);
  view.classList.toggle("far", scale < LABEL_SCALE);
}
function fit() {
  const box = view.getBBox(), w = stage.clientWidth, h = stage.clientHeight, pad = 28;
  scale = Math.min((w - 2 * pad) / box.width, (h - 2 * pad) / box.height, 1.2);
  tx = (w - box.width * scale) / 2 - box.x * scale;
  ty = (h - box.height * scale) / 2 - box.y * scale;
  apply();
}
function centre(n) {
  scale = Math.max(scale, 0.9);
  tx = stage.clientWidth / 2 - n.x * scale;
  ty = stage.clientHeight / 2 - n.y * scale;
  apply();
}
function zoom(factor, x, y) {
  const next = Math.min(4, Math.max(0.03, scale * factor));
  tx = x - (x - tx) * next / scale;
  ty = y - (y - ty) * next / scale;
  scale = next;
  apply();
}
stage.addEventListener("wheel", event => {
  event.preventDefault();
  const box = stage.getBoundingClientRect();
  zoom(Math.exp(-event.deltaY * 0.0015), event.clientX - box.left, event.clientY - box.top);
}, {passive: false});
let drag = null, moved = false;
stage.addEventListener("pointerdown", event => {
  drag = {x: event.clientX - tx, y: event.clientY - ty};
  moved = false;
});
window.addEventListener("pointermove", event => {
  if (!drag) return;
  moved = true;
  stage.classList.add("dragging");
  tx = event.clientX - drag.x;
  ty = event.clientY - drag.y;
  apply();
});
window.addEventListener("pointerup", () => { drag = null; stage.classList.remove("dragging"); });
svg.addEventListener("click", () => { if (!moved && selected) select(selected); });
document.getElementById("in").onclick = () => zoom(1.3, stage.clientWidth / 2, stage.clientHeight / 2);
document.getElementById("out").onclick = () => zoom(1 / 1.3, stage.clientWidth / 2, stage.clientHeight / 2);
document.getElementById("fit").onclick = fit;
document.getElementById("certain").onchange = event =>
  view.classList.toggle("no-certain", !event.target.checked);
view.classList.add("no-certain");
window.addEventListener("resize", fit);
fit();
</script>
</body>
</html>
"""


def write_page(view, path):
    data = json.dumps(view, separators=(",", ":")).replace("</", "<\\/")
    title = view["title"].replace("&", "&amp;").replace("<", "&lt;")
    with open(path, "w", encoding="utf-8") as f:
        f.write(PAGE.replace("__TITLE__", title).replace("__DATA__", data))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--problem", help="reads output_progression/<problem>/")
    ap.add_argument("--merge", choices=("auto", "none"), default="auto")
    ap.add_argument("--graph", help="a ..._graph.json, instead of --problem")
    ap.add_argument("--output", help="default: the graph's path with .html")
    args = ap.parse_args()

    if args.graph:
        graph_path = args.graph
        title = os.path.basename(graph_path).replace("_graph.json", "").replace("_", ", merge ", 1)
    elif args.problem:
        graph_path = os.path.join(OUTPUT_ROOT, args.problem,
                                  f"{args.problem}_{args.merge}_graph.json")
        title = f"{args.problem}, merge {args.merge}"
    else:
        ap.error("pass --problem or --graph")
    if not os.path.isfile(graph_path):
        sys.exit(f"[ERROR] {graph_path} not found -- run progression.py first.")

    with open(graph_path) as f:
        graph = json.load(f)
    output = args.output or re.sub(r"\.json$", "", graph_path) + ".html"
    view = build_view(graph, f"Progression graph: {title}")
    write_page(view, output)
    print(f"wrote {output} ({len(graph['nodes'])} nodes, {len(graph['edges'])} edges)")


if __name__ == "__main__":
    main()
