#!/usr/bin/env python3
"""
summarize_ablation.py

Scans output/problem*/ for each problem's own <problem>_<ts>_ablation_
summary.csv (the one main.py --ablation-study writes -- see main.py's
own _SUMMARY_FIELDS/write_ablation_summary) and, for EVERY (knowledge
backend, propagate_weights) combination found across all of them,
builds one table: rows = map/size suffix (L/M/S/... -- whatever
actually follows the problem number on disk), columns = problem number
(0, 1, 2, ... -- whatever actually exists), parsed from each problem
directory's own name via ^problem(\\d+)([A-Za-z]*)$. Nothing here is
hardcoded to "0-4" or "L/M/S" specifically -- both axes are discovered
fresh from whatever output/problem* directories and summary rows
actually exist.

Three output files, one metric each, each holding ALL 12 per-
combination tables back to back:
  - output/ablation_time_tables.txt      (total_s -- time to solution)
  - output/ablation_ground_tables.txt    (ground_nodes)
  - output/ablation_compiled_tables.txt  (compiled_nodes)

A cell is "-" whenever that (problem, backend, propagate_weights)
combination's own status isn't "ok" -- covers a timeout, but also
skipped/error/no_results the same way, since none of those have a real
number to show either.

If a problem directory has MULTIPLE *_ablation_summary.csv files (run
more than once), the most recent one (by filename timestamp, which
sorts lexicographically the same as chronologically given main.py's
own YYYYMMDD_HHMMSS format) is used -- older ones are ignored, not
merged.

Usage:
    python3 summarize_ablation.py [--output-dir DIR]
        (default DIR: output/, alongside this script)
"""

import argparse
import csv
import glob
import os
import re
import sys
from collections import defaultdict

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_OUTPUT_DIR = os.path.join(_THIS_DIR, "output")

_PROBLEM_DIR_RE = re.compile(r"^problem(\d+)([A-Za-z]*)$")

# (csv field, output filename, human title) -- the three reports this
# script produces, all driven off the SAME discovered rows.
_METRICS = [
    ("total_s", "ablation_time_tables.txt", "Time to solution (s)"),
    ("ground_nodes", "ablation_ground_tables.txt", "Ground nodes"),
    ("compiled_nodes", "ablation_compiled_tables.txt", "Compiled nodes"),
]


def find_latest_summary(problem_dir):
    """The most recent *_ablation_summary.csv in problem_dir, by
    filename (main.py's own YYYYMMDD_HHMMSS timestamp sorts correctly
    as a plain string), or None if there isn't one -- e.g. a problem
    that's only ever been run in the plain single or --visual mode,
    never --ablation-study."""
    candidates = sorted(glob.glob(os.path.join(problem_dir, "*_ablation_summary.csv")))
    return candidates[-1] if candidates else None


def load_rows(output_dir):
    """Returns {(problem_number, size_suffix): [row dict, ...]} --
    one entry per output/problem* directory that has its own ablation
    summary CSV, parsed via csv.DictReader (so every column is still a
    plain string, matching how it was written)."""
    data = {}
    problem_dirs = sorted(glob.glob(os.path.join(output_dir, "problem*")))
    for problem_dir in problem_dirs:
        if not os.path.isdir(problem_dir):
            continue
        name = os.path.basename(problem_dir)
        m = _PROBLEM_DIR_RE.match(name)
        if not m:
            print(f"[warn] {name!r} doesn't match problem<number><suffix> -- skipping", file=sys.stderr)
            continue
        number, suffix = m.group(1), m.group(2) or ""

        summary_path = find_latest_summary(problem_dir)
        if summary_path is None:
            print(f"[warn] no *_ablation_summary.csv in {problem_dir} "
                  f"(run with --ablation-study to get one) -- skipping", file=sys.stderr)
            continue

        with open(summary_path, newline="") as f:
            rows = list(csv.DictReader(f))
        data[(number, suffix)] = rows
    return data


def discover_combinations(data):
    """Every (knowledge, propagate_weights) pair actually present
    across every loaded row, sorted for a stable, repeatable table
    order -- not hardcoded to _ABLATION_BACKENDS in main.py, so this
    stays correct even if that list ever changes."""
    combos = set()
    for rows in data.values():
        for row in rows:
            combos.add((row["knowledge"], row["propagate_weights"]))
    return sorted(combos)


def cell_value(data, key, combo, metric_field):
    """The metric_field value for (problem_number, suffix)=key under
    this (knowledge, propagate_weights) combo, or "-" if that problem
    has no row for this combo, or the row's own status isn't "ok"
    (covers timeout, skipped, error, no_results alike -- none of them
    have a real number to show). total_s is rounded to 2 decimals for
    table readability (the raw CSV keeps full precision -- this is
    just the summary view); node counts are plain integers either way."""
    rows = data.get(key)
    if not rows:
        return "-"
    knowledge, propagate_weights = combo
    for row in rows:
        if row["knowledge"] == knowledge and row["propagate_weights"] == propagate_weights:
            if row.get("status") != "ok":
                return "-"
            value = row.get(metric_field, "")
            if value in (None, ""):
                return "-"
            if metric_field == "total_s":
                try:
                    return f"{float(value):.2f}"
                except ValueError:
                    return value
            return value
    return "-"


def format_table(title, data, combo, metric_field, numbers, suffixes):
    cells_by_row = {
        suffix: [cell_value(data, (n, suffix), combo, metric_field) for n in numbers]
        for suffix in suffixes
    }
    col_header_w = max(
        [len(f"problem{n}") for n in numbers]
        + [len(c) for cells in cells_by_row.values() for c in cells]
        + [7],
    ) + 2
    row_label_w = max((len(s) for s in suffixes), default=4) + 2
    row_label_w = max(row_label_w, 6)

    header = " " * row_label_w + "".join(f"problem{n}".rjust(col_header_w) for n in numbers)
    lines = [title, header]
    for suffix in suffixes:
        row_label = suffix if suffix else "(none)"
        cells = "".join(c.rjust(col_header_w) for c in cells_by_row[suffix])
        lines.append(f"{row_label:<{row_label_w}}{cells}")
    return "\n".join(lines)


def write_report(output_dir, filename, title_prefix, metric_field, data, combos, numbers, suffixes):
    path = os.path.join(output_dir, filename)
    with open(path, "w") as f:
        for knowledge, propagate_weights in combos:
            pw_label = "on" if propagate_weights == "1" else "off"
            title = (f"=== {title_prefix} -- backend={knowledge}, "
                     f"propagate_weights={pw_label} ===")
            f.write(format_table(title, data, (knowledge, propagate_weights),
                                  metric_field, numbers, suffixes))
            f.write("\n\n")
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--output-dir", default=_DEFAULT_OUTPUT_DIR,
                     help="Directory holding problem*/ subdirectories "
                          "(default: output/, alongside this script). "
                          "Report files are written into this same "
                          "directory.")
    args = ap.parse_args()

    data = load_rows(args.output_dir)
    if not data:
        print(f"[ERROR] No problem*/*_ablation_summary.csv found under "
              f"{args.output_dir} -- run main.py --ablation-study first.",
              file=sys.stderr)
        sys.exit(1)

    numbers = sorted({k[0] for k in data}, key=int)
    suffixes = sorted({k[1] for k in data})
    combos = discover_combinations(data)

    print(f"[info] {len(data)} problem(s), {len(combos)} backend/"
          f"propagate_weights combination(s), problem numbers "
          f"{numbers}, map suffixes {suffixes}")

    for metric_field, filename, title in _METRICS:
        write_report(args.output_dir, filename, title, metric_field,
                     data, combos, numbers, suffixes)


if __name__ == "__main__":
    main()
