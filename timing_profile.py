#!/usr/bin/env python3
"""
timing_profile.py

Breaks ProbLog's GROUND stage down into where its time actually goes:
  - time spent inside the external Python black boxes basic_action_theory.pl
    :- use_module()s (planners.py, collision_geometry.py) -- per predicate:
    how many times it is called, how many of those calls are repeats of an
    argument tuple already seen, and total/mean/min/max time;
  - time spent importing those modules (load_external_module, which runs
    their module-level code: map/config loading, etc.);
  - everything else = ProbLog's own grounding engine (SLD resolution,
    unification, builtins, formula construction).

Same problem setup as main.py (run_shared_setup: regenerate obstacles/
config/plan/queries, write problem_data.pl), then Parse + Ground through
pipeline_stages.run_stage. Compile/Evaluate are skipped unless --full.

How the externals are measured (no changes to the black boxes themselves):
before ProbLog loads anything, problog.extern is patched so that
  - problog_export_nondet.__call__ wraps each decorated function with a
    timer -> "inner" time = the pure Python computation;
  - problog_export.add_function wraps the callable ProbLog registers ->
    "outer" time = inner + ProbLog's own term<->Python argument/result
    conversion and unification (extern.py's _wrapped_function).
Outer time is what the grounding engine actually pays, so it is the one
subtracted from the ground stage to get the engine-only time.

Usage:
    python3 timing_profile.py --problem problem3L
    python3 timing_profile.py --problem problem3L --cprofile   # + engine hot spots
    python3 timing_profile.py --problem problem3L --full       # + compile/evaluate

Writes, into output/<problem>/ (wiped first, like main.py):
    <problem>_<ts>_timing.log         the report printed to the console
    <problem>_<ts>_extern_stats.csv   one row per external predicate
    <problem>_<ts>_extern_calls.csv   one row per call (stage, start offset,
                                      inner/outer seconds, #results, args)
"""

import argparse
import csv
import functools
import inspect
import os
import sys
import time
from collections import defaultdict

import problog.extern as pl_extern
from problog.clausedb import ClauseDB


# --------------------------------------------------------------------------
# Instrumentation -- must be installed BEFORE ProbLog loads the theory.
# --------------------------------------------------------------------------

class ExternProfiler:
    def __init__(self):
        self.calls = []            # dicts, one per call (outer wrapper)
        self.imports = []          # (module filename, seconds, stage)
        self.stage = "setup"
        self.t_origin = time.perf_counter()
        self._inner_stack = []     # inner seconds of the call in flight

    def install(self):
        prof = self
        orig_call = pl_extern.problog_export_nondet.__call__
        orig_add = pl_extern.problog_export.add_function.__func__
        orig_load = ClauseDB.load_external_module

        def timed_call(self_dec, func, funcname=None, modname="#auto#"):
            # Wrap the user function for the "inner" time. extern.py checks
            # getfullargspec(func).varkw on the function it is given and
            # passes the engine's kwargs only if it has one, so keep that
            # behaviour for the real function.
            takes_kw = inspect.getfullargspec(func).varkw is not None

            @functools.wraps(func)
            def inner(*args, **kw):
                t0 = time.perf_counter()
                try:
                    res = func(*args, **kw) if takes_kw else func(*args)
                    res = list(res)  # generators: count their work here
                finally:
                    if prof._inner_stack:
                        prof._inner_stack[-1] += time.perf_counter() - t0
                return res

            if funcname is None:
                funcname = self_dec.functor or func.__name__
            if modname == "#auto#":
                modname = func.__module__
            orig_call(self_dec, inner, funcname=funcname, modname=modname)
            return func

        def timed_add(cls, name, in_args, out_args, func, module_name):
            arity = in_args + out_args

            def outer(*args, **kw):
                prof._inner_stack.append(0.0)
                t0 = time.perf_counter()
                n_results = None
                try:
                    res = func(*args, **kw)
                    n_results = len(res)
                    return res
                finally:
                    dt = time.perf_counter() - t0
                    inner_dt = prof._inner_stack.pop()
                    prof.calls.append({
                        "predicate": f"{name}/{arity}",
                        "module": module_name,
                        "stage": prof.stage,
                        "start": t0 - prof.t_origin,
                        "outer": dt,
                        "inner": inner_dt,
                        "n_results": n_results,
                        "args": tuple(str(a) for a in args[:in_args]),
                    })

            orig_add(cls, name, in_args, out_args, outer, module_name)

        def timed_load(self_db, filename):
            t0 = time.perf_counter()
            try:
                return orig_load(self_db, filename)
            finally:
                prof.imports.append((filename, time.perf_counter() - t0, prof.stage))

        pl_extern.problog_export_nondet.__call__ = timed_call
        pl_extern.problog_export.add_function = classmethod(timed_add)
        ClauseDB.load_external_module = timed_load

    def per_predicate(self):
        agg = defaultdict(lambda: {"calls": 0, "outer": 0.0, "inner": 0.0,
                                    "min": float("inf"), "max": 0.0,
                                    "keys": set(), "results": 0, "fails": 0,
                                    "module": ""})
        for c in self.calls:
            a = agg[c["predicate"]]
            a["module"] = c["module"]
            a["calls"] += 1
            a["outer"] += c["outer"]
            a["inner"] += c["inner"]
            a["min"] = min(a["min"], c["outer"])
            a["max"] = max(a["max"], c["outer"])
            a["keys"].add(c["args"])
            a["results"] += c["n_results"] or 0
            if not c["n_results"]:
                a["fails"] += 1
        return dict(sorted(agg.items(), key=lambda kv: -kv[1]["outer"]))


PROFILER = ExternProfiler()
PROFILER.install()

# Imported only after the patch so nothing can register externs unpatched.
import main as bt_main                                    # noqa: E402
from pipeline_stages import run_stage, StageTimeout        # noqa: E402
from problog.program import PrologFile                     # noqa: E402
from problog.formula import LogicFormula                   # noqa: E402
from problog import get_evaluatable                        # noqa: E402
from problog.evaluator import SemiringLogProbability       # noqa: E402
from grounding_probe import EngineSampler, FormulaProbe     # noqa: E402


class Tee:
    def __init__(self):
        self.lines = []

    def __call__(self, text=""):
        print(text)
        self.lines.append(text)


def fmt_ms(seconds):
    return f"{seconds * 1000:10.3f}"


def report_formula(tee, fp, t_ground, n_nodes):
    tee("")
    tee("  Ground-graph construction (LogicFormula calls made by the engine)")
    tee("  " + "-" * 70)
    tee(f"  total time inside these calls: {fp.total_time:.3f}s "
        f"({100 * fp.total_time / t_ground:.2f}% of ground)  -> final graph: {n_nodes} nodes")
    for m in fp.METHODS:
        if not fp.calls[m]:
            continue
        tee(f"  {m:<14}{fp.calls[m]:>10} calls  {fp.time[m]:8.3f}s  "
            f"({1e6 * fp.time[m] / fp.calls[m]:.1f} us/call)")
        for o, n in fp.outcomes[m].most_common():
            if o != "call":
                tee(f"      {o:<44}{n:>10}  {100 * n / fp.calls[m]:6.2f}%")
    tee(f"  TRUE children dropped from ANDs: {fp.true_inputs_dropped}, "
        f"FALSE children dropped from ORs: {fp.false_inputs_dropped}")


def report_engine(tee, s, top, base):
    n = s.samples
    if not n:
        tee("\n  [engine breakdown] no samples collected")
        return
    tee("")
    tee(f"  Ground time by operation ({n} samples, 1 per {s.interval * 1000:g} ms CPU)")
    tee("  " + "-" * 70)
    for cat, c in s.category.most_common():
        tee(f"  {cat:<58}{100 * c / n:7.2f}%")

    tee("")
    tee(f"  Ground time by theory predicate (top {top}; inclusive = with everything it "
        f"calls, self = innermost Prolog predicate)")
    tee(f"  {'predicate':<40}{'inclusive':>10}{'self':>9}{'evals':>10}{'table hits':>12}")
    tee("  " + "-" * 79)
    for pred, c in s.inclusive.most_common(top):
        tee(f"  {pred:<40}{100 * c / n:9.2f}%{100 * s.self_[pred] / n:8.2f}%"
            f"{s.define_calls[pred]:>10}{s.define_cache_hits[pred]:>12}")
    tee("")
    tee(f"  Top {top} by self time")
    for pred, c in s.self_.most_common(top):
        tee(f"  {pred:<40}{100 * c / n:8.2f}%")
    if s.no_chain:
        tee(f"  ({100 * s.no_chain / n:.2f}% of samples had no Prolog call chain: "
            f"engine setup/bookkeeping between calls)")
    with open(base + "_engine_predicates.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["predicate", "inclusive_pct", "self_pct", "evals", "table_hits"])
        for pred in set(s.inclusive) | set(s.define_calls):
            w.writerow([pred, f"{100 * s.inclusive[pred] / n:.3f}",
                        f"{100 * s.self_[pred] / n:.3f}", s.define_calls[pred],
                        s.define_cache_hits[pred]])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--problem", default="problem3L")
    ap.add_argument("--phase-timeout", type=int, default=1800,
                    help="Per-stage timeout in seconds (default 1800).")
    ap.add_argument("--propagate-weights", action="store_true",
                    help="Ground with propagate_weights (as the ablation does).")
    ap.add_argument("--full", action="store_true",
                    help="Also run Compile + Evaluate (default backend) and write every "
                         "query's probability to <base>_results.csv.")
    ap.add_argument("--cprofile", action="store_true",
                    help="Run the Ground stage under cProfile and report the "
                         "top functions by self time.")
    ap.add_argument("--top", type=int, default=25,
                    help="Rows to show in the cProfile / per-predicate tables (default 25).")
    ap.add_argument("--engine-breakdown", action="store_true",
                    help="Sample the engine's Prolog call chain during Ground: time "
                         "per theory predicate and per kind of operation "
                         "(grounding_probe.EngineSampler).")
    ap.add_argument("--sample-interval", type=float, default=0.001,
                    help="CPU seconds between engine samples (default 0.001).")
    ap.add_argument("--formula-ops", action="store_true",
                    help="Count/time every ground-graph construction call and "
                         "classify it: new node, folded deterministic, collapsed, "
                         "merged (grounding_probe.FormulaProbe).")
    args = ap.parse_args()

    ctx = bt_main.run_shared_setup(args.problem, "timing profile")
    if ctx is None:
        sys.exit(1)
    out_dir, ts = ctx["problem_output_dir"], ctx["ts"]
    base = os.path.join(out_dir, f"{args.problem}_{ts}")

    tee = Tee()
    tee("")
    tee("=" * 72)
    tee(f"  Grounding time profile -- {args.problem}"
        f"{'  (propagate_weights)' if args.propagate_weights else ''}"
        f"{'  [under cProfile: absolute times inflated]' if args.cprofile else ''}")
    tee("=" * 72)

    semiring = SemiringLogProbability()
    timings = {}
    profiler = None
    sampler = fprobe = None
    results = None
    try:
        PROFILER.stage = "parse"
        model, timings["parse"] = run_stage(
            tee, "Parse", lambda: PrologFile(bt_main.THEORY_PATH), args.phase_timeout)

        PROFILER.stage = "ground"

        def ground():
            return LogicFormula.create_from(
                model, label_all=True,
                propagate_weights=(semiring if args.propagate_weights else None))

        if args.engine_breakdown:
            sampler = EngineSampler(args.sample_interval)
            sampler.start()
        if args.formula_ops:
            fprobe = FormulaProbe()
            fprobe.start()
        if args.cprofile:
            import cProfile
            profiler = cProfile.Profile()
            lf, timings["ground"] = run_stage(
                tee, "Ground", lambda: profiler.runcall(ground), args.phase_timeout)
        else:
            lf, timings["ground"] = run_stage(tee, "Ground", ground, args.phase_timeout)
        tee(f"    -> {len(lf)} ground node(s)")
        if sampler:
            sampler.stop()
        if fprobe:
            fprobe.stop()

        if args.full:
            PROFILER.stage = "compile"
            compiled, timings["compile"] = run_stage(
                tee, "Compile", lambda: get_evaluatable().create_from(lf),
                args.phase_timeout)
            PROFILER.stage = "evaluate"
            results, timings["evaluate"] = run_stage(
                tee, "Evaluate", lambda: compiled.evaluate(semiring=semiring),
                args.phase_timeout)
    except StageTimeout:
        tee(f"\n  [WARN] stage '{PROFILER.stage}' timed out -- the numbers below "
            f"cover only the calls made before the timeout.")
        timings.setdefault(PROFILER.stage,
                           time.perf_counter() - PROFILER.t_origin)  # rough
    PROFILER.stage = "done"

    # ---- breakdown --------------------------------------------------------
    stats = PROFILER.per_predicate()
    ground_calls = [c for c in PROFILER.calls if c["stage"] == "ground"]
    ext_outer = sum(c["outer"] for c in ground_calls)
    ext_inner = sum(c["inner"] for c in ground_calls)
    imp_ground = sum(dt for _, dt, st in PROFILER.imports if st == "ground")
    t_ground = timings.get("ground", 0.0)
    engine = t_ground - ext_outer - imp_ground

    def pct(x):
        return f"{100 * x / t_ground:6.2f}%" if t_ground else "   n/a"

    tee("")
    tee("  Stage times (s): " + ", ".join(f"{k} {v:.3f}" for k, v in timings.items()))
    tee("")
    tee("  Ground stage breakdown")
    tee("  " + "-" * 70)
    tee(f"  {'total ground':<44}{t_ground:10.3f}s  100.00%")
    tee(f"  {'  module import (use_module of .py files)':<44}{imp_ground:10.3f}s {pct(imp_ground)}")
    tee(f"  {'  external Python predicates (outer)':<44}{ext_outer:10.3f}s {pct(ext_outer)}")
    tee(f"  {'    of which: Python computation (inner)':<44}{ext_inner:10.3f}s {pct(ext_inner)}")
    tee(f"  {'    of which: ProbLog arg/result conversion':<44}{ext_outer - ext_inner:10.3f}s {pct(ext_outer - ext_inner)}")
    tee(f"  {'  ProbLog grounding engine (remainder)':<44}{engine:10.3f}s {pct(engine)}")
    tee(f"  external calls during ground: {len(ground_calls)}")

    tee("")
    tee("  Module imports")
    for fn, dt, st in PROFILER.imports:
        tee(f"    {os.path.basename(fn):<28} {dt:8.3f}s  (during {st})")

    tee("")
    tee("  External predicates (all stages; times in ms, 'outer' unless noted)")
    hdr = (f"  {'predicate':<38}{'calls':>7}{'unique':>8}{'fails':>7}"
           f"{'total':>11}{'inner':>11}{'mean':>11}{'min':>11}{'max':>11}")
    tee(hdr)
    tee("  " + "-" * (len(hdr) - 2))
    for pred, a in stats.items():
        tee(f"  {pred:<38}{a['calls']:>7}{len(a['keys']):>8}{a['fails']:>7}"
            f"{fmt_ms(a['outer']):>11}{fmt_ms(a['inner']):>11}"
            f"{fmt_ms(a['outer'] / a['calls']):>11}{fmt_ms(a['min']):>11}"
            f"{fmt_ms(a['max']):>11}")
    n_calls = sum(a["calls"] for a in stats.values())
    n_unique = sum(len(a["keys"]) for a in stats.values())
    if n_calls:
        tee(f"\n  {n_calls} calls, {n_unique} distinct argument tuples -> "
            f"{n_calls - n_unique} repeated calls ({100 * (n_calls - n_unique) / n_calls:.1f}%) "
            f"that a memo cache would have skipped.")

    if fprobe is not None:
        report_formula(tee, fprobe, t_ground, len(lf))
    if sampler is not None:
        report_engine(tee, sampler, args.top, base)

    if profiler is not None:
        import io
        import pstats
        buf = io.StringIO()
        ps = pstats.Stats(profiler, stream=buf).sort_stats("tottime")
        ps.print_stats(args.top)
        tee("")
        tee(f"  cProfile of Ground -- top {args.top} by self time")
        for line in buf.getvalue().splitlines():
            tee("  " + line)
        profiler.dump_stats(base + "_ground.prof")
        tee(f"  (raw profile: {base}_ground.prof -- e.g. `snakeviz` it)")

    # ---- files ------------------------------------------------------------
    with open(base + "_extern_stats.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["predicate", "module", "calls", "unique_args", "fails",
                    "results", "outer_total_s", "inner_total_s", "outer_mean_s",
                    "outer_min_s", "outer_max_s"])
        for pred, a in stats.items():
            w.writerow([pred, a["module"], a["calls"], len(a["keys"]), a["fails"],
                        a["results"], f"{a['outer']:.6f}", f"{a['inner']:.6f}",
                        f"{a['outer'] / a['calls']:.6f}", f"{a['min']:.6f}",
                        f"{a['max']:.6f}"])
    with open(base + "_extern_calls.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["predicate", "stage", "start_s", "outer_s", "inner_s",
                    "n_results", "args"])
        for c in PROFILER.calls:
            w.writerow([c["predicate"], c["stage"], f"{c['start']:.6f}",
                        f"{c['outer']:.6f}", f"{c['inner']:.6f}", c["n_results"],
                        " | ".join(c["args"])])
    if results is not None:
        with open(base + "_results.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["query", "probability"])
            for q, pr in sorted((str(k), float(v)) for k, v in results.items()):
                w.writerow([q, repr(pr)])
    with open(base + "_timing.log", "w") as f:
        f.write("\n".join(ctx["setup"].lines + tee.lines) + "\n")
    print(f"\n[info] written: {base}_timing.log, _extern_stats.csv, _extern_calls.csv")


if __name__ == "__main__":
    main()
