#!/usr/bin/env python3
"""
verify_progression.py

Compares progression.py against the regressed ProbLog run on one or more
problems, query by query, for both --merge modes. Each stage is its own
process, because the Python black boxes read BT_PROBLEM_DIR at import
time and the stages rewrite the same module/theory/problem_data.pl: run
one instance of this script at a time.

The ProbLog result is cached as <problem>_baseline.json, since it is the
slow part; pass --rerun-baseline to recompute it.

    python3 verify_progression.py --problems problem0S problem0M problem0L
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_ROOT = os.path.join(ROOT_DIR, "output_progression")

_FLOAT = re.compile(r"-?\d+\.\d+(?:[eE][-+]?\d+)?")


def normalize(row):
    """Query text with floats rounded, so a last-digit difference in a
    reported coordinate cannot hide a matching row."""
    return _FLOAT.sub(lambda m: f"{float(m.group(0)):.9f}", row.replace(" ", ""))


def run_baseline(problem, stage_timeout, backend=None, propagate_weights=False,
                 suffix=""):
    """Runs in its own process: the normal main.py pipeline, one backend."""
    import main as pipeline

    ctx = pipeline.run_shared_setup(problem, "verify_progression baseline",
                                    keep_output=True)
    if ctx is None:
        raise SystemExit(1)
    started = time.perf_counter()
    results, elapsed, timings, sizes = pipeline.run_problog_api(
        pipeline.THEORY_PATH, print, stage_timeout, knowledge_name=backend,
        propagate_weights=propagate_weights)
    out_dir = os.path.join(OUTPUT_ROOT, problem)
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, f"{problem}_baseline{suffix}.json"), "w") as f:
        json.dump({"problem": problem,
                   "backend": backend or "auto",
                   "propagate_weights": propagate_weights,
                   "seconds": time.perf_counter() - started,
                   "timings": {k: v for k, v in timings.items()},
                   "sizes": {k: v for k, v in sizes.items()},
                   "probabilities": {str(k): float(v) for k, v in results.items()}},
                  f, indent=1)


def compare(baseline, summary):
    """Compares every row present on both sides and lists what is not."""
    theirs = {normalize(k): v for k, v in baseline["probabilities"].items()}
    ours = {normalize(k): v for k, v in summary["probabilities"].items()}
    skipped = {normalize(k) for k in summary["skipped_queries"]}

    def ignored(row):
        if summary["merge"] == "none":
            return False
        return row.startswith("outcome_signature(") or row.startswith(
            "any_reason_pattern_detail_by_action(sample_")

    diffs, only_baseline, only_progression = [], [], []
    for row, value in theirs.items():
        if row in ours:
            diffs.append((abs(value - ours[row]), row, value, ours[row]))
        elif not ignored(row) and row not in skipped and value > 1e-12:
            only_baseline.append((row, value))
    for row, value in ours.items():
        if row not in theirs and value > 1e-12:
            only_progression.append((row, value))
    diffs.sort(reverse=True)
    return {
        "rows_compared": len(diffs),
        "max_abs_diff": diffs[0][0] if diffs else None,
        "worst_rows": diffs[:5],
        "only_baseline": only_baseline,
        "only_progression": only_progression,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--problems", nargs="+")
    ap.add_argument("--stage-timeout", type=int, default=3600)
    ap.add_argument("--rerun-baseline", action="store_true")
    ap.add_argument("--tolerance", type=float, default=1e-9)
    ap.add_argument("--backend", default=None,
                    help="ProbLog backend for the baseline (default: auto, "
                         "which is SDD and can exhaust memory while compiling)")
    ap.add_argument("--propagate-weights", action="store_true")
    ap.add_argument("--baseline-suffix", default="",
                    help="with --baseline-only: appended to the result's file "
                         "name, to compare several backends side by side")
    ap.add_argument("--baseline-only", metavar="PROBLEM",
                    help="internal: run the ProbLog stage for PROBLEM")
    args = ap.parse_args()

    if args.baseline_only:
        run_baseline(args.baseline_only, args.stage_timeout, args.backend,
                     args.propagate_weights, args.baseline_suffix)
        return

    failed = False
    report = {}
    for problem in args.problems:
        print(f"\n===== {problem} =====", flush=True)
        out_dir = os.path.join(OUTPUT_ROOT, problem)
        baseline_path = os.path.join(out_dir, f"{problem}_baseline.json")
        if args.rerun_baseline or not os.path.isfile(baseline_path):
            command = [sys.executable, __file__, "--baseline-only", problem,
                       "--stage-timeout", str(args.stage_timeout)]
            if args.backend:
                command += ["--backend", args.backend]
            if args.propagate_weights:
                command.append("--propagate-weights")
            subprocess.run(command, check=True)
        with open(baseline_path) as f:
            baseline = json.load(f)

        report[problem] = {"baseline_seconds": baseline["seconds"]}
        for merge in ("none", "auto"):
            subprocess.run([sys.executable, os.path.join(ROOT_DIR, "progression.py"),
                            "--problem", problem, "--merge", merge], check=True)
            with open(os.path.join(out_dir, f"{problem}_{merge}_summary.json")) as f:
                summary = json.load(f)
            result = compare(baseline, summary)
            ok = (result["rows_compared"] > 0
                  and result["max_abs_diff"] <= args.tolerance
                  and not result["only_baseline"]
                  and not result["only_progression"]
                  and abs(summary["final_mass"] - 1.0) <= args.tolerance)
            failed = failed or not ok
            report[problem][merge] = {
                "ok": ok,
                "seconds": summary["search_seconds"] + summary["evaluate_seconds"],
                "final_states": summary["final_states"],
                "nodes": summary["nodes"],
                "final_mass": summary["final_mass"],
                **result,
            }
            print(f"  [{'OK' if ok else 'FAIL'}] merge={merge}: "
                  f"{result['rows_compared']} rows, "
                  f"max |diff| = {result['max_abs_diff']}, "
                  f"{len(result['only_baseline'])} only in ProbLog, "
                  f"{len(result['only_progression'])} only in progression",
                  flush=True)

    os.makedirs(OUTPUT_ROOT, exist_ok=True)
    report_path = os.path.join(OUTPUT_ROOT, "verification_report.json")
    with open(report_path, "w") as f:
        json.dump(report, f, indent=1)
    print(f"\nreport: {report_path}")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
