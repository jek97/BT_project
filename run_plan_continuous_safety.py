#!/usr/bin/env python3
"""
run_plan_continuous_safety.py

Runs the continuous-time / continuous-space single-moveto() ProbLog
action theory (moveto_continuous.pl) and produces a full safety report,
analogous in spirit to run_plan_weave_safety.py but re-indexed from
"discrete grid step N" to "sampled instant I along the one continuous
walk", and from "grid obstacle cells" to "obstacle polygons" (as
produced by occgrid_to_problog.py).

Four safety features, same structure as before, ALL specifically about
COLLISION (first_hit/hit_by are collision-only PMFs/CDFs -- see below
for the separate battery report):
  Feature 1  - verify_safe: under NO noise, does the plan fail via ANY
               known cause -- collision OR battery depletion?
  Feature 2a - hit_by(N): P(collision has occurred by sample N)
  Feature 2b - first_hit(I): P(first COLLISION is exactly at sample I)
               -- a proper PMF over sampled instants, no double-counting
  (implied)  - riskiest sample = argmax of first_hit(I)

Also reports, separately (kept distinct rather than folded into the
collision-specific features above, so the two failure modes stay
individually diagnosable):
  - any_battery_depletion: P(the battery runs out before the walk
    would otherwise complete)
  - on_track(I): P(actual position stays within tolerance of the
    nominal spline at sample I) -- shows cumulative drift over time

Usage:
    python3 run_plan_continuous_safety.py moveto_continuous.pl

Before running inference, this script:
  1. regenerates environments/maps/obstacles_generated.pl from
     environments/maps/map.yaml (see environments/occgrid_to_problog.py's
     own header) -- map.yaml is the single source of truth for the
     obstacle layout.
  2. regenerates config/config_generated.pl from config/config.yaml
     (see that module's own header) -- config.yaml is the single
     source of truth for every tunable constant in the theory (noise
     sigmas, the Z discretization tables, battery drain rates,
     robot/safety thresholds, tolerances, verification resolution, and
     the robot's own starting position).
  3. translates plan_generation/plan/behavior_tree.xml -- a real
     BT.cpp v4 tree, the single source of truth for the POLICY'S
     SHAPE -- into plan_generation/plan/plan_generated.pl, validating
     it against actions/schema.yaml on the way (see
     plan_generation/bt_to_prolog.py's own header).
  4. validates plan_generation/plan/goal_formula.pl -- the hand-
     authored verification goal for THIS particular plan, and the
     ONLY place goal information lives in this theory -- against
     plan_generation/vocabulary.yaml (see
     plan_generation/goal_formula_check.py's own header): every
     predicate it calls must be a known fluent, and the whole formula
     must be uniform in one situation (Reiter's own sense).
All four steps mean a normal run always reflects whatever is currently
in map.yaml / config.yaml / behavior_tree.xml / goal_formula.pl, with
no separate regeneration step needed.

Requires: `problog` importable/runnable on PATH.

Saves a timestamped log file and a PNG with the obstacle polygons, the
nominal spline, the sampled hazard heatmap along it, and the riskiest
sample marked.
"""

import argparse
import os
import re
import sys
import time
from datetime import datetime

from problog.program import PrologFile
from problog import get_evaluatable
from problog.errors import ProbLogError

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.colors as mcolors
from matplotlib.lines import Line2D

W = 68


# -----------------------------------------------------------------------
# Parsing the .pl files (regex-based -- we control the fact format that
# moveto_continuous.pl / occgrid_to_problog.py emit, so this is safe and
# avoids needing a Prolog engine just to read back ground facts)
# -----------------------------------------------------------------------
POINT_RE = re.compile(r"point\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)")


CONSULT_RE = re.compile(r":-\s*consult\(\s*'([^']+)'\s*\)\s*\.")


def strip_prolog_comments(text):
    """Remove '%'-to-end-of-line comments before regex-parsing facts. This
    matters specifically because moveto_continuous.pl's own documentation
    comments include example/placeholder syntax like
    'control_points([point(...),...]).' to illustrate the expected shape
    -- which otherwise matches parse_control_points' regex BEFORE the real
    data (found via re.search, which stops at the first match), silently
    parsing to an empty point list instead of raising or finding the real
    fact further down."""
    return re.sub(r"%.*$", "", text, flags=re.MULTILINE)


def resolve_consulted_text(theory_path, _seen=None):
    """
    Read theory_path's text, AND the text of every file it :- consult()s
    (resolved relative to theory_path's own directory), recursively, and
    return it all concatenated. This mirrors what ProbLog itself does at
    load time -- moveto_continuous.pl no longer contains start/1 or
    obstacle_polygon/2 directly; they live in files it consults
    (config_generated.pl, obstacles_generated.pl), so parsing only
    theory_path's own text (as earlier versions of this script did) finds
    nothing. _seen guards against accidental consult cycles.
    """
    theory_path = os.path.abspath(theory_path)
    if _seen is None:
        _seen = set()
    if theory_path in _seen or not os.path.isfile(theory_path):
        return ""
    _seen.add(theory_path)

    with open(theory_path) as f:
        text = strip_prolog_comments(f.read())

    combined = [text]
    theory_dir = os.path.dirname(theory_path)
    for m in CONSULT_RE.finditer(text):
        consult_target = m.group(1)
        resolved = os.path.normpath(os.path.join(theory_dir, consult_target))
        combined.append(resolve_consulted_text(resolved, _seen))

    return "\n".join(combined)


def parse_control_points(theory_text):
    m = re.search(r"control_points\(\s*\[(.*?)\]\s*\)\s*\.", theory_text, re.S)
    if not m:
        raise ValueError("Could not find control_points/1 in the theory file")
    pts = [(float(x), float(y)) for x, y in POINT_RE.findall(m.group(1))]
    if len(pts) < 4 or (len(pts) - 1) % 3 != 0:
        print(f"[warn] control_points has {len(pts)} points -- expected 3k+1 "
              f"for k cubic Bezier segments", file=sys.stderr)
    return pts


def parse_scalar_fact(theory_text, name):
    m = re.search(rf"^{name}\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)\s*\.",
                  theory_text, re.MULTILINE)
    if not m:
        raise ValueError(f"Could not find numeric {name}/2 fact")
    return (float(m.group(1)), float(m.group(2)))


PLANWITH_GOAL_RE = re.compile(
    r"planWith\([^,]+,\s*point\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)")


def parse_last_plan_goal(theory_text):
    """The LAST planWith(...,point(GX,GY),...) literal appearing in the
    (fully resolved) theory text -- used as a plottable "goal" marker
    now that there is no longer a single global goal/2 fact (goal
    information lives entirely in plan_generation/plan/goal_formula.pl,
    which has no fixed shape a plotting script could reliably parse
    instead). The LAST occurrence, not the first, since a multi-leg
    plan's own final target is the more representative "where the
    mission is headed" point. KNOWN LIMITATION: a plan ending on a
    follow_boarder(...) leaf would mark the wrong point, since that
    planner's own goal argument is a meaningless point(0.0,0.0)
    placeholder (see moveto_continuous.pl's own note on why) -- not hit
    by the shipped plan."""
    matches = PLANWITH_GOAL_RE.findall(theory_text)
    if not matches:
        raise ValueError(
            "Could not find any planWith(...,point(X,Y),...) literal in "
            "the theory file to use as a goal marker")
    x, y = matches[-1]
    return (float(x), float(y))


def parse_int_fact(theory_text, name, default):
    m = re.search(rf"^{name}\(\s*(\d+)\s*\)\s*\.", theory_text, re.MULTILINE)
    return int(m.group(1)) if m else default


def parse_obstacle_polygons(obstacles_text):
    polys = []
    for m in re.finditer(r"obstacle_polygon\([^,]+,\s*\[(.*?)\]\s*\)\s*\.",
                          obstacles_text, re.S):
        pts = [(float(x), float(y)) for x, y in POINT_RE.findall(m.group(1))]
        if len(pts) >= 3:
            polys.append(pts)
    return polys


# -----------------------------------------------------------------------
# Closed-form spline evaluation in Python, mirroring the ProbLog theory's
# bezier_point/spline_point EXACTLY, so the plot matches what ProbLog
# actually reasoned about.
# -----------------------------------------------------------------------
def bezier_point(p0, p1, p2, p3, u):
    mu = 1 - u
    x = mu**3*p0[0] + 3*mu**2*u*p1[0] + 3*mu*u**2*p2[0] + u**3*p3[0]
    y = mu**3*p0[1] + 3*mu**2*u*p1[1] + 3*mu*u**2*p2[1] + u**3*p3[1]
    return x, y


def spline_point(control_points, u):
    n_segs = (len(control_points) - 1) // 3
    seg_len = 1.0 / n_segs
    seg_idx = min(n_segs - 1, int(u // seg_len))
    local_u = (u - seg_idx*seg_len) / seg_len
    local_u = max(0.0, min(1.0, local_u))
    p0, p1, p2, p3 = control_points[3*seg_idx:3*seg_idx+4]
    return bezier_point(p0, p1, p2, p3, local_u)


def nominal_trajectory(control_points, n=200):
    return [spline_point(control_points, i/n) for i in range(n+1)]


# -----------------------------------------------------------------------
# Run ProbLog via its PYTHON API (not the CLI/subprocess) and return a
# results dict directly. str(term) for a query like plan_outcome(true)
# or first_hit(5) is EXACTLY the same string PARSE_RESULTS used to
# extract from CLI text output -- so everything downstream (
# print_safety_results, plotting) needs zero changes; only how the
# numbers get INTO the dict changes.
# -----------------------------------------------------------------------
def run_problog_api(plan_file):
    t0 = time.perf_counter()
    model = PrologFile(plan_file)
    raw_result = get_evaluatable().create_from(model).evaluate()
    elapsed = time.perf_counter() - t0
    results = {str(term): float(prob) for term, prob in raw_result.items()}
    return results, elapsed


# -----------------------------------------------------------------------
# Plotting
# -----------------------------------------------------------------------
def save_plot(control_points, obstacle_polygons, start, goal, num_samples,
              first_hit_probs, output_path, world_bounds=None):
    """
    first_hit_probs: list of (I, x, y, prob) for I = 0..num_samples, where
    (x,y) is the NOMINAL spline position at that sample (the segment
    ending at I is coloured by prob).
    """
    cmap = plt.cm.RdYlGn_r
    lw = 3.5

    all_probs = [p for _, _, _, p in first_hit_probs]
    max_prob = max(all_probs) if all_probs and max(all_probs) > 0 else 1e-9
    norm = mcolors.Normalize(vmin=0, vmax=max_prob)

    riskiest = max(first_hit_probs, key=lambda t: t[3]) if first_hit_probs else None

    fig, ax = plt.subplots(figsize=(11, 11))
    ax.set_aspect("equal")

    if world_bounds:
        xmin, xmax, ymin, ymax = world_bounds
    else:
        all_xy = [p for p in control_points] + [pt for poly in obstacle_polygons for pt in poly]
        xs = [p[0] for p in all_xy]; ys = [p[1] for p in all_xy]
        pad = 2.0
        xmin, xmax = min(xs)-pad, max(xs)+pad
        ymin, ymax = min(ys)-pad, max(ys)+pad
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)

    # -- obstacle polygons -----------------------------------------------
    for poly in obstacle_polygons:
        patch = patches.Polygon(poly, closed=True, facecolor="#1a1a1a",
                                 edgecolor="#555555", linewidth=0.8, zorder=2)
        ax.add_patch(patch)

    # -- nominal spline, colour-scaled by first_hit hazard ----------------
    dense = nominal_trajectory(control_points, n=200)
    xs_d = [p[0] for p in dense]; ys_d = [p[1] for p in dense]
    ax.plot(xs_d, ys_d, color="#999999", linewidth=1.0, linestyle="--",
             zorder=3, label="_nolegend_")  # faint full nominal path underneath

    if len(first_hit_probs) > 1:
        for i in range(len(first_hit_probs) - 1):
            _, x0, y0, _ = first_hit_probs[i]
            _, x1, y1, p1 = first_hit_probs[i+1]
            color = cmap(norm(p1))
            ax.plot([x0, x1], [y0, y1], color=color, linewidth=lw,
                     solid_capstyle="round", zorder=4)

    # -- riskiest sample marker -------------------------------------------
    if riskiest is not None and riskiest[3] > 0:
        ri, rx, ry, rp = riskiest
        ax.plot(rx, ry, "x", markersize=22, markeredgewidth=3.5,
                 color="#cc0000", zorder=9)
        ax.text(rx + 0.3, ry - 0.3, f"I={ri}\np={rp:.4f}",
                fontsize=7, color="#cc0000", fontweight="bold", va="top",
                zorder=10, bbox=dict(facecolor="white", edgecolor="#cc0000",
                                      linewidth=0.7, alpha=0.9, pad=1.5))

    # -- start / goal --------------------------------------------------
    ax.plot(start[0], start[1], "o", markersize=14, color="#16a34a",
             markeredgecolor="white", markeredgewidth=1.5, zorder=7)
    ax.text(start[0], start[1], "S", ha="center", va="center",
             fontsize=8, fontweight="bold", color="white", zorder=8)
    ax.plot(goal[0], goal[1], "*", markersize=18, color="#dc2626",
             markeredgecolor="white", markeredgewidth=1.0, zorder=7)
    ax.text(goal[0] + 0.4, goal[1] + 0.4, "G", ha="center", va="center",
             fontsize=8, fontweight="bold", color="#dc2626", zorder=8)

    ax.set_xlabel("x (m)", fontsize=10)
    ax.set_ylabel("y (m)", fontsize=10)
    ax.set_title(
        "Continuous-space Robot Trajectory - Safety Verification\n"
        f"Start: {start}   Goal: {goal}   {num_samples} verification samples\n"
        "Trajectory colour: P(first_hit at sample I)  -  green=safe  red=risky\n"
        "Red X: most likely failure sample",
        fontsize=9, pad=10)

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, fraction=0.03, pad=0.02)
    cbar.set_label("P(first_hit at sample I)", fontsize=8)
    cbar.ax.tick_params(labelsize=7)

    legend_elements = [
        patches.Patch(facecolor="#1a1a1a", edgecolor="#555", label="Obstacle"),
        Line2D([0], [0], color="#999999", linestyle="--", linewidth=1,
               label="Nominal spline (full)"),
        Line2D([0], [0], color=cmap(0.0), linewidth=3, label="Trajectory - safe"),
        Line2D([0], [0], color=cmap(0.5), linewidth=3, label="Trajectory - moderate risk"),
        Line2D([0], [0], color=cmap(1.0), linewidth=3, label="Trajectory - high risk"),
        Line2D([0], [0], marker="x", color="#cc0000", linewidth=0,
               markersize=12, markeredgewidth=2.5, label="Most likely failure sample"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#16a34a",
               markersize=10, label="Start"),
        Line2D([0], [0], marker="*", color="w", markerfacecolor="#dc2626",
               markersize=13, label="Goal"),
    ]
    ax.legend(handles=legend_elements, loc="lower right", fontsize=8, framealpha=0.9)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()


# -----------------------------------------------------------------------
# Report printing
# -----------------------------------------------------------------------
class Tee:
    def __init__(self, log_fh):
        self._log = log_fh
    def __call__(self, text=""):
        print(text)
        self._log.write(text + "\n")
        self._log.flush()


def banner(tee, text, char="="):
    tee(char * W); tee(f"  {text}"); tee(char * W)

def section(tee, text):
    tee(f"\n{'-'*W}"); tee(f"  {text}"); tee(f"{'-'*W}")

def bar(prob, width=30):
    filled = round(max(0.0, min(1.0, prob)) * width)
    return "#" * filled + "-" * (width - filled)


def print_safety_results(tee, results, num_samples):
    section(tee, "Feature 1 - Deterministic nominal-path safety [verify_safe]")
    tee("  Checks: under NO noise (zero-noise/modal case), does the plan ever")
    tee("          fail for ANY known cause -- currently: coming within the")
    tee("          safety margin of an obstacle, OR the battery running out")
    tee("          before the walk would otherwise complete?")
    vs = results.get("verify_safe", 0.0)
    tee(f"  verify_safe : {vs:.0f}  ->  "
        + ("PASS - nominal plan is clear of every known failure cause" if vs >= 0.999
           else "FAIL - nominal plan fails via collision and/or battery depletion "
                "(see any_collision / any_battery_depletion below for which)"))

    section(tee, "Feature 2a - P(ever hits obstacle along trajectory) [hit_by]")
    ua = results.get(f"hit_by({num_samples})", 0.0)
    tee(f"  hit_by({num_samples})        : {bar(ua)}  {ua*100:6.3f}%")
    tee(f"  Safe probability      : {bar(1-ua)}  {(1-ua)*100:6.3f}%")

    section(tee, "Feature 2b - Per-sample first-passage hazard [first_hit]")
    tee("  P(first collision is exactly at sample I) - proper PMF")
    fh_vals = [(i, results.get(f"first_hit({i})", 0.0)) for i in range(num_samples+1)]
    nonzero = [(i, p) for i, p in fh_vals if p > 0]
    total_fh = sum(p for _, p in fh_vals)
    if nonzero:
        max_p = max(p for _, p in nonzero)
        tee(f"\n  {'I':<4} {'first_hit(I)':>14}  {'Hazard bar':<22}")
        tee(f"  {'-'*4} {'-'*14}  {'-'*22}")
        for i, p in fh_vals:
            if p <= 0:
                continue
            b = bar(p/max_p, 20) if max_p > 0 else "-"*20
            note = "  <- most dangerous" if p == max_p else ""
            tee(f"  {i:<4} {p:>14.6f}  {b}{note}")
        tee(f"\n  Sum of first_hit PMF : {total_fh:.6f}  (should equal hit_by = {ua:.6f})")
    else:
        tee("  No non-zero first_hit values - never hits obstacle stochastically")

    section(tee, "Implied - Most likely failure sample (argmax of first_hit)")
    if nonzero:
        i_r, p_r = max(nonzero, key=lambda t: t[1])
        tee(f"  Riskiest sample I = {i_r} (of {num_samples})  with P(first_hit) = {p_r:.6f}")
    else:
        tee("  No failure samples detected.")

    section(tee, "Overall outcome")
    for key, label in [("verify_goal_formula", "verify_goal_formula  "),
                        ("any_collision", "any_collision        "),
                        ("any_battery_depletion", "any_battery_depletion")]:
        p = results.get(key, 0.0)
        tee(f"  {label}  {bar(p)}  {p*100:6.2f}%")

    section(tee, "On-track probability per sample")
    tee("  P(actual position stays within tolerance of nominal spline)")
    ot_vals = [(i, results.get(f"on_track({i})", None)) for i in range(num_samples+1)]
    ot_vals = [(i, p) for i, p in ot_vals if p is not None]
    if ot_vals:
        min_p = min(p for _, p in ot_vals)
        tee(f"\n  {'I':<4} {'on_track':>10}  {'Bar':<22}")
        tee(f"  {'-'*4} {'-'*10}  {'-'*22}")
        for i, p in ot_vals:
            marker = "  <- riskiest" if p == min_p else ""
            tee(f"  {i:<4} {p:>10.4f}  {bar(p,20)}{marker}")


def control_points_via_planner(theory_dir, algorithm, start, goal):
    """Fallback for when control_points/1 can't be found by static
    parsing -- e.g. the plan uses planAstar/planStraight (see
    moveto_continuous.pl's plan/1 documentation) instead of a static
    fact, so there's nothing for parse_control_points' regex to find.
    Calls the SAME black-box predicate directly (importing
    moveto_planners.py from the theory file's own ./actions/
    subdirectory, exactly where moveto_continuous.pl's own
    :- use_module(...) directive expects it to live) to get a plottable
    nominal path. Uses the plain-Python plan_*_points functions (no
    ProbLog Term objects involved) -- see moveto_planners.py's own
    header. Returns None if the import or the planner call itself fails
    (e.g. no map, or A* finds no path) -- the caller decides what to do
    next."""
    actions_dir = os.path.join(theory_dir, "actions")
    if actions_dir not in sys.path:
        sys.path.insert(0, actions_dir)
    try:
        import moveto_planners as mp
    except ImportError:
        return None
    func = mp.plan_astar_points if algorithm == "astar" else mp.plan_straight_points
    try:
        control_points = func(float(start[0]), float(start[1]), float(goal[0]), float(goal[1]))
    except Exception:
        return None
    if not control_points:
        return None
    return [(float(x), float(y)) for x, y in control_points]


# -----------------------------------------------------------------------
# main
# -----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plan_file", nargs="?", default="moveto_continuous.pl",
                     help="Path to the ProbLog action-theory file "
                          "(default: moveto_continuous.pl)")
    ap.add_argument("--obstacles", default=None,
                     help="Optional EXTRA obstacle facts file to also parse, "
                          "on top of whatever moveto_continuous.pl itself "
                          "already :- consult()s (normally you don't need "
                          "this -- obstacle_polygon/2 facts are found "
                          "automatically by following the theory file's own "
                          "consult directives, same as ProbLog does at load "
                          "time).")
    args = ap.parse_args()

    script_dir = os.path.dirname(os.path.abspath(args.plan_file)) or "."
    plan_path = os.path.abspath(args.plan_file)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(script_dir, f"run_plan_continuous_safety_{ts}.log")
    img_path = os.path.join(script_dir, f"run_plan_continuous_safety_{ts}.png")

    with open(log_path, "w", encoding="utf-8") as fh:
        tee = Tee(fh)
        banner(tee, f"ProbLog Continuous-Space Safety Verification - {datetime.now():%Y-%m-%d %H:%M:%S}")
        tee(f"  Log file    : {log_path}")
        tee(f"  Image       : {img_path}")
        tee(f"  Plan file   : {plan_path}")

        if not os.path.isfile(plan_path):
            tee(f"\n  [ERROR] File not found: {plan_path}")
            sys.exit(1)

        # Regenerate environments/maps/obstacles_generated.pl from
        # environments/maps/map.yaml BEFORE anything reads the theory --
        # map.yaml is the single source of truth for the obstacle
        # layout (see environments/occgrid_to_problog.py), same
        # automatic-every-run treatment config.yaml/behavior_tree.xml
        # already get, rather than the separate manual step this used
        # to be. Must happen before resolve_consulted_text() below, for
        # the same staleness reason as config_generated.pl.
        env_dir = os.path.join(script_dir, "environments")
        if env_dir not in sys.path:
            sys.path.insert(0, env_dir)
        try:
            from occgrid_to_problog import generate as generate_obstacles
            generated_obstacles_path = generate_obstacles(
                yaml_path=os.path.join(env_dir, "maps", "map.yaml"),
                output_path=os.path.join(env_dir, "maps", "obstacles_generated.pl"))
            tee(f"  Obstacles   : {generated_obstacles_path} (regenerated from "
                f"{os.path.join(env_dir, 'maps', 'map.yaml')})")
        except Exception as e:
            tee(f"\n  [ERROR] Could not regenerate obstacles_generated.pl: {e}")
            sys.exit(1)

        # Regenerate config/config_generated.pl from config/config.yaml
        # BEFORE anything reads the theory -- config.yaml is the single
        # source of truth for every tunable constant (see config/
        # generate_prolog_config.py), so every run picks up whatever is
        # currently there with no separate step. Must happen before the
        # resolve_consulted_text() call below, since that follows
        # moveto_continuous.pl's own :- consult('./config/config_generated.pl')
        # directive and would otherwise read a stale or missing file.
        config_dir = os.path.join(script_dir, "config")
        if config_dir not in sys.path:
            sys.path.insert(0, config_dir)
        try:
            from generate_prolog_config import generate as generate_prolog_config
            generated_path = generate_prolog_config(
                config_path=os.path.join(config_dir, "config.yaml"),
                output_path=os.path.join(config_dir, "config_generated.pl"))
            tee(f"  Config      : {generated_path} (regenerated from "
                f"{os.path.join(config_dir, 'config.yaml')})")
        except Exception as e:
            tee(f"\n  [ERROR] Could not regenerate config_generated.pl: {e}")
            sys.exit(1)

        # Translate plan_generation/plan/behavior_tree.xml (the real
        # BT.cpp v4 tree that is now the single source of truth for the
        # POLICY'S SHAPE) into plan_generation/plan/plan_generated.pl,
        # validating it against actions/schema.yaml on the way -- see
        # plan_generation/bt_to_prolog.py's own header. Any structural
        # problem (unknown node, missing/unrecognized port, a
        # control_points blackboard key with no producer) is a hard
        # failure here, same as a missing config fact above; there is no
        # sensible way to run inference against a tree that doesn't
        # actually match its own schema. Must also happen before
        # resolve_consulted_text() below, for the same staleness reason
        # as config_generated.pl.
        plan_gen_dir = os.path.join(script_dir, "plan_generation")
        if plan_gen_dir not in sys.path:
            sys.path.insert(0, plan_gen_dir)
        try:
            from bt_to_prolog import generate_plan_pl, BTValidationError
            generated_plan_path = generate_plan_pl(
                xml_path=os.path.join(plan_gen_dir, "plan", "behavior_tree.xml"),
                schema_path=os.path.join(script_dir, "actions", "schema.yaml"),
                output_path=os.path.join(plan_gen_dir, "plan", "plan_generated.pl"))
            tee(f"  Plan (BT)   : {generated_plan_path} (translated + validated "
                f"from {os.path.join(plan_gen_dir, 'plan', 'behavior_tree.xml')})")
        except BTValidationError as e:
            tee(f"\n  [ERROR] behavior_tree.xml failed validation: {e}")
            sys.exit(1)
        except Exception as e:
            tee(f"\n  [ERROR] Could not translate behavior_tree.xml: {e}")
            sys.exit(1)

        # Validate plan_generation/plan/goal_formula.pl against
        # plan_generation/vocabulary.yaml -- same "structural
        # validation is a hard failure, not a warning" posture as
        # behavior_tree.xml's own validation just above; see
        # plan_generation/goal_formula_check.py's own header. Must
        # also happen before resolve_consulted_text() below, for the
        # same staleness reason as config_generated.pl/plan_generated.pl.
        try:
            from goal_formula_check import validate_goal_formula, GoalFormulaValidationError
            goal_formula_path = os.path.join(plan_gen_dir, "plan", "goal_formula.pl")
            validate_goal_formula(
                goal_formula_path=goal_formula_path,
                vocab_path=os.path.join(plan_gen_dir, "vocabulary.yaml"))
            tee(f"  Goal formula: {goal_formula_path} (validated against "
                f"{os.path.join(plan_gen_dir, 'vocabulary.yaml')})")
        except GoalFormulaValidationError as e:
            tee(f"\n  [ERROR] goal_formula.pl failed validation: {e}")
            sys.exit(1)
        except Exception as e:
            tee(f"\n  [ERROR] Could not validate goal_formula.pl: {e}")
            sys.exit(1)

        # Follow moveto_continuous.pl's own :- consult(...) directives --
        # start/1 now lives in config_generated.pl, obstacle_polygon/2
        # in obstacles_generated.pl -- not in the theory file's own
        # text. There is no goal/2 fact anywhere anymore (see
        # parse_last_plan_goal's own note on where "the goal" is read
        # from instead, for plotting purposes).
        theory_text = resolve_consulted_text(plan_path)

        try:
            start = parse_scalar_fact(theory_text, "start")
            goal = parse_last_plan_goal(theory_text)
            num_samples = parse_int_fact(theory_text, "num_samples", 20)
        except ValueError as e:
            tee(f"\n  [ERROR] {e}")
            sys.exit(1)

        try:
            control_points = parse_control_points(theory_text)
        except ValueError:
            tee("  [info] No static control_points/1 found -- plan likely "
                "uses planAstar/planStraight (see moveto_continuous.pl's "
                "plan/1 docs) instead of a static fact. Calling the SAME "
                "black-box predicate directly for a plottable nominal path.")
            control_points = None
            for algorithm in ("astar", "straight"):
                control_points = control_points_via_planner(
                    script_dir, algorithm, start, goal)
                if control_points is not None:
                    tee(f"  [info] Got a nominal path via plan_{algorithm}.")
                    break
            if control_points is None:
                tee("\n  [ERROR] Could not obtain control points, neither "
                    "statically nor via the planner fallback.")
                sys.exit(1)

        obstacle_polygons = parse_obstacle_polygons(theory_text)
        if args.obstacles and os.path.isfile(args.obstacles):
            with open(args.obstacles) as f:
                obstacle_polygons += parse_obstacle_polygons(f.read())
        tee(f"\n  Parsed: {len(control_points)} control points, "
            f"{len(obstacle_polygons)} obstacle polygon(s), "
            f"{num_samples} verification samples")

        tee(f"\n  Started : {datetime.now():%H:%M:%S}")
        try:
            results, elapsed = run_problog_api(plan_path)
        except ProbLogError as e:
            tee(f"\n  [ERROR] ProbLog error: {e}")
            sys.exit(1)
        except Exception as e:
            tee(f"\n  [ERROR] Unexpected error running the model: {e}")
            sys.exit(1)
        tee(f"  Finished: {datetime.now():%H:%M:%S}  ({elapsed:.3f}s)")

        if not results:
            tee("\n  [warn] No results returned from ProbLog -- check the "
                "file has query(...) declarations.")
            sys.exit(1)

        print_safety_results(tee, results, num_samples)

        # -- build (I, x, y, prob) using the NOMINAL spline position ------
        fh_data = []
        for i in range(num_samples + 1):
            frac = i / num_samples
            x, y = spline_point(control_points, frac)
            prob = results.get(f"first_hit({i})", 0.0)
            fh_data.append((i, x, y, prob))

        tee(f"\n  Generating safety plot...")
        save_plot(control_points, obstacle_polygons, start, goal, num_samples,
                   fh_data, img_path)
        tee(f"  Image saved : {img_path}")

        tee("")
        banner(tee, f"Log : {log_path}")
        banner(tee, f"PNG : {img_path}")


if __name__ == "__main__":
    main()