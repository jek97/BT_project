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

On this branch, `python3 main.py --problem <name>` (same arguments as
always, no new flags needed) no longer runs the problem once -- it
sweeps every ProbLog knowledge-compilation backend (`sdd`, `sddx`,
`bdd`, `nnf`/DSharp d-DNNF, `fsdd`, `fbdd`) crossed with
`--propagate-weights` on/off (12 combinations total), and writes:
  - one `output/<problem>/<problem>_<ts>_<backend>_<0|1>.log` per
    combination (`0`/`1` = propagate-weights off/on) -- same full
    report content as a normal single run (goal formula, query
    results, reason/condition breakdowns, ...), just per combination.
  - one `output/<problem>/<problem>_<ts>_ablation_summary.csv` with,
    per combination: ground/compiled node counts, per-stage timings
    (parse/ground/compile/evaluate), total time, and status (`ok`/
    `skipped`/`timeout`/`error`/`no_results`).
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
    if `False`, `pip install pyeda`. Fair warning, confirmed directly
    on this same setup: `pyeda`'s own build FAILED here (an old package
    incompatible with modern setuptools/Python 3.11 -- `AttributeError:
    install_layout` from its `setup.py`). If that happens on your side
    too, `bdd`/`fbdd` will just show up as `skipped` in the summary CSV
    rather than block the rest of the sweep -- not something this
    project's own code can work around, since it's `pyeda`'s own
    packaging that's broken on newer Python.
  - The translator pipeline itself (obstacles/config/plan translation,
    run regardless of backend) also needs `numpy`, `pillow` (PIL),
    `opencv-python` (`cv2`), and `scipy` -- none of this project's own
    dependencies were pinned anywhere before now; install whichever of
    these `pip install` reports missing.
