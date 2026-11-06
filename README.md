from here i kept the same regressable format, but this time we will have contineous time actions in contineous space, the action move to will be modeled as a single action from start to goal,
the motion will be modelled as a nominal trajectory affected by gaussian noise. we will evaluate the same parameters as before, but this time over the contineous trajectory. additionally, by
passing to contineous space the map will be obtained from a nav_msgs/OccupancyGrid, that will be used and tranformed in a series of obstacle polygones representing the obstacles for the basic
action theory to work. 

command to run all

python3 main.py --problem problem0
# main.py regenerates every generated file (obstacles/config/plan) and
# validates goal_formula.pl automatically before each run -- see its
# own module docstring, and module/theory/basic_action_theory.pl's
# Section 0, for the current problems/<name>/ + module/ layout.

Introduced battery consumption as a linear decreasing model affected by stochastic error where the buttery consumption rate is dependent by the action being executed.

Modified the action moveto to work as a template action, where the different reasons/conditions for which the action may fail can be provided as an argument, still they need to be formalized inside the file: for example a collision with an obstacle must be axiomatized based on the robot position and the map, so that we can simply pass to the action the condition collision.

added action outcome (status) to reflect success/failure in BT, also created the sequence  and fallback process.# BT_project

## ablation_study branch: backend/propagate-weights sweep

`python3 main.py --problem <name> --ablation-study` sweeps every
ProbLog knowledge-compilation backend (`sdd`, `sddx`, `bdd`,
`nnf`/DSharp d-DNNF, `fsdd`, `fbdd`) crossed with propagate-weights
on/off (12 combinations total), and writes:
  - one `output/<problem>/<problem>_<ts>_<backend>_<0|1>.log` per
    combination (`0`/`1` = propagate-weights off/on) -- same full
    report content as a normal single run (goal formula, query
    results, reason/condition breakdowns, ...), just per combination.
  - one `output/<problem>/<problem>_<ts>_ablation_summary.csv` with,
    per combination: ground/compiled node counts, per-stage timings
    (parse/ground/compile/evaluate), total time, and status (`ok`/
    `skipped`/`timeout`/`error`/`no_results`, or `running` if the run
    was killed mid-combination). The CSV is rewritten after every
    stage, so a problem that grounds but can't compile still reports
    its ground node count.
Then `python3 summarize_ablation.py` merges every run and writes the
time, ground-node and compiled-node tables into `output/`.
A combination whose backend isn't installed is SKIPPED with a clear
note (both in its own log and the CSV), not crashed past -- see
_backend_available() in main.py. `--approximate`/`--approximate-
convergence` are parsed but ignored here (a warning is printed) --
`kbest` is an anytime approximate evaluator, not a knowledge-
compilation backend, so it doesn't fit this sweep; it's already its
own separate mode on main/visual.

**Do you need to install anything first?** Checked directly, not
assumed:
  - `nnf` (DSharp d-DNNF) and `kbest`: **nothing to install** -- both
    ship bundled inside the `problog` package itself.
  - `sdd`, `sddx`, `fsdd`: need `pysdd`. Check first with
    `python3 -c "from problog.sdd_formula import SDD; print(SDD.is_available())"`;
    if that prints `False`, install with `pip install pysdd` (a
    prebuilt wheel installed cleanly when this was tested).
  - `bdd`, `fbdd`: need `pyeda`. Check first with
    `python3 -c "from problog.bdd_formula import BDD; print(BDD.is_available())"`;
    if `False`, `pip install pyeda`. Fair warning, confirmed directly:
    `pyeda`'s own plain `pip install` FAILS on a modern Python/
    setuptools (it's an old, unmaintained package). If that happens,
    `bdd`/`fbdd` just show up as `skipped` in the summary CSV rather
    than block the rest of the sweep -- not something this project's
    own code can work around, it's `pyeda`'s own packaging.

    Building it from source DOES work around this (confirmed):
    ```
    git clone https://github.com/cjdrake/pyeda.git
    cd pyeda
    python setup.py install --prefix /path/to/some/install/dir
    ```
    This installs into a NON-standard location Python won't find on
    its own --
    `/path/to/some/install/dir/local/lib/python3.X/dist-packages/`
    (the exact `.../local/lib/pythonX.Y/dist-packages` suffix is a
    Debian/Ubuntu distutils convention, confirmed directly -- adjust
    `python3.X` to your own Python version). **Before running the
    pipeline** (`main.py`, or any script that imports `problog`), put
    that directory on `PYTHONPATH`:
    ```
    export PYTHONPATH=/path/to/some/install/dir/local/lib/python3.X/dist-packages:$PYTHONPATH
    ```
    Add that `export` line to `~/.bashrc` (or your shell's own profile)
    to make it permanent, instead of retyping it every session. Also
    avoid running any `python3 -c "import pyeda"` check FROM INSIDE the
    `pyeda` source checkout itself -- Python puts the current directory
    first on its import path, so it'll silently pick up the raw,
    unbuilt source folder (no compiled `.so` files in it) instead of
    the real installed copy, and `BDD.is_available()` will read `False`
    even though the real install is fine; `cd` elsewhere first.
  - The translator pipeline itself (obstacles/config/plan translation,
    run regardless of backend) also needs `numpy`, `pillow` (PIL),
    `opencv-python` (`cv2`), and `scipy` -- none of this project's own
    dependencies were pinned anywhere before now; install whichever of
    these `pip install` reports missing.

## Inference flags (main.py, regression)

`python3 main.py --problem <name> [flags]` regresses the whole plan to
`s0` and runs one ProbLog query (parse, ground, compile, evaluate).

| Flag | Default | What it does |
|---|---|---|
| `--problem NAME` | `problem0` (`all` with `--resume-from`) | A subdirectory of `problems/`, or `all` for every `problem*` one, in sequence |
| `--ablation-study` | off | Sweep all backends x propagate-weights (see above) |
| `--resume-from PROBLEM[:BACKEND_PW]` | off | Restart an interrupted run, e.g. `problem3L:bdd_1`; a bare backend means `_0`. Keeps that problem's earlier output |
| `--visual` | off | Ground the problem and write the interactive ground graph plus the SLD tree, instead of running inference |
| `--sld-query GOAL` | `verify_goal_formula` | With `--visual`: the query whose SLD tree is written; `''` skips it |
| `--sld-max-nodes N` | 1000000 | With `--visual`: stop the SLD tree after N nodes |
| `--phase-timeout S` | 300 | Timeout in seconds for each stage |
| `--parse-timeout`, `--ground-timeout`, `--compile-timeout`, `--evaluate-timeout` | `--phase-timeout` | Override one stage's timeout |
| `--approximate` | off | k-best anytime lower bound instead of exact compilation; ignored by `--ablation-study` |
| `--approximate-convergence X` | 0.01 | With `--approximate`: stop when the bounds are within X |
| `--float-precision N\|none` | 15 | Decimal places ProbLog rounds floats to. It also rounds the probabilities in the program, so a small N changes the model |
| `--no-gc` | off | Turn Python's garbage collector off during each stage, collecting between stages, so its pauses don't inflate the timings |

`--visual` and `--ablation-study` exclude each other, and `--visual`
doesn't support `--problem all`.

## Progression

`progression.py` evaluates the same BT forwards instead of regressing
it. It gives the same probabilities as `main.py`, for the goal, the
plan outcomes and the safety and condition queries, without grounding
or knowledge compilation.

```
python3 progression.py --problem problem0S       # one problem
python3 progression.py --problem all             # every problem, one at a time
python3 visualize_progression.py --problem problem0S
```

**How it works.** Every noise table is finite, and a leaf is
deterministic once its draws are fixed, so the belief state is a finite
set of configurations:

    configuration = zbatt draw + Phys + Mem + current leaf + rest of the BT

`Phys` holds the current value of every fluent (position, time, battery
anchor, hitch, tools, ...) and `Mem` what history-reading predicates
need (halt reasons, condition checks, arrivals, ...). `zbatt` is one
draw for the whole mission, so it is split once at the root. From each
configuration:

1. Pick the noise tables the leaf uses: `z`, `zt` for a MoveTo,
   `sample_result`, `sample_value` for a TakeSample, a `*_result` table
   for a tool leg, none for conditions and planners.
2. For each joint draw, run the leaf once. `progress_leaf/8` in
   `module/theory/progression.pl` wraps the state as the situation
   `st(Phys,Mem,Draws)` and calls the ordinary `do_node/4`. This is a
   plain Prolog query (`DefaultEngine.query`): nothing is probabilistic
   here, because the config it loads reads fixed draws instead of
   annotated disjunctions. The weights stay in Python.
3. Read the new state. `next_state/4` evaluates every fluent through
   the ordinary axioms, which regress across the leaf's own `do(...)`
   terms and stop at `st(...)`, the base case that replaces `s0`. The
   `do(...)` terms are then discarded, so situations never grow.
4. The edge probability is the product of the draws' weights. A draw
   with no successor goes to an explicit `stuck` final state.
5. `advance()` passes the leaf's outcome up the BT stack, following
   `seq_step`, `fallback_step`, `invert` and `reactive_after` in
   `basic_action_theory.pl`, which it must stay in sync with. It stops
   at the next leaf or at a final state.
6. The child merges with an existing configuration if their keys match;
   otherwise it joins the next layer of the search.

Afterwards, one topological pass computes
`reach(child) = sum of reach(parent) * P(edge)`, and every query is
evaluated once per final state, adding that state's reach when it
holds.

`basic_action_theory.pl` is loaded unchanged. Problems reading the last
walk after it ended (`obstacle_on_path`, `on_track`, `first_hit`,
`hit_by`, `verify_safe`, `plan_route_blocked`) are refused, because
`st(...)` doesn't store it.

**Flags**

| Script | Flag | Default | What it does |
|---|---|---|---|
| `progression.py` | `--problem NAME` | required | A subdirectory of `problems/`, or `all` for every `problem*` one, each in its own process |
| | `--merge auto\|none` | `auto` | See below |
| `visualize_progression.py` | `--problem NAME` | | Reads `output_progression/<problem>/` |
| | `--merge auto\|none` | `auto` | Which graph to draw |
| | `--graph PATH` | | A `..._graph.json`, instead of `--problem` |
| | `--output PATH` | the graph's path with `.html` | Where to write the page |
| `verify_progression.py` | `--problems NAME...` | required | Compare both merge modes against a ProbLog run, query by query |
| | `--backend NAME` | ProbLog's choice (SDD, which can exhaust memory while compiling) | Backend for the ProbLog run |
| | `--propagate-weights` | off | Propagate weights in the ProbLog run |
| | `--stage-timeout S` | 3600 | Timeout for each ProbLog stage |
| | `--rerun-baseline` | off | Recompute the cached `<problem>_baseline.json` |
| | `--tolerance X` | 1e-9 | Largest accepted difference |
| | `--baseline-suffix TEXT` | none | With the internal `--baseline-only` stage: suffix for the baseline file name, to compare backends side by side |
| `verify_worlds.py` | `--problem NAME` | required | Exact reference: enumerates complete worlds through the unmodified theory. Refuses trees with reactive nodes or retries |
| | `--jobs N` | 16 | Parallel workers |

**`--merge`** decides when two configurations reached by different
paths count as one:
  - `none`: only if their whole history is equal, so there is one final
    state per path.
  - `auto`: history is compared without details nothing later can read
    (the position and value stored in sample reasons, and exact
    arrivals unless the goal or a condition reads them). Walks also
    merge at their start, once the merge grids have rounded T0 and B0.
    The graph is smaller, but the outcome enumeration and the sample
    detail rows aren't reported, since they name the values merged
    away.

**Output** goes to `output_progression/<problem>/`:
  - `<problem>_<merge>_report.log`: the same tables as `main.py`, also
    printed to the console, plus any `stuck` probability.
  - `<problem>_<merge>_summary.json`: every probability, node counts
    and timings.
  - `<problem>_<merge>_graph.json`: the execution graph, which
    `visualize_progression.py` turns into a self-contained
    `..._graph.html`, with P(child | parent) on the edges.

Only run one progression or verification at a time: they all rewrite
`module/theory/problem_data.pl`.
