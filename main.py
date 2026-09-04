#!/usr/bin/env python3
"""
main.py

Runs the continuous-time / continuous-space single-moveto() ProbLog
action theory (module/theory/basic_action_theory.pl) against one
problem (problems/<name>/, default "problem0") and produces a full
safety report, analogous in spirit to run_plan_weave_safety.py but
re-indexed from "discrete grid step N" to "sampled instant I along the
one continuous walk", and from "grid obstacle cells" to "obstacle
polygons" (as produced by module/translators/occgrid_to_problog.py).

Prints a COMPACT summary (the problem's own goal_formula.pl, plus a
small probability table) rather than the earlier verbose per-sample
report -- trimmed together with basic_action_theory.pl's own Section
10 QUERIES list to a small, fixed set of queries, for the reactive-
redescend/merge-grid grounding-performance investigation (see
FUTUREWORK.md and this project's own conversation log):
  - verify_goal_formula: P(the problem's own goal_formula.pl holds at
    the final situation)
  - any_collision / any_battery_depletion: P(the plan ends via that
    cause)
  - plan_outcome(true) / plan_outcome(false) / plan_outcome
    (world_too_large): the BT's own three possible outcomes
  - plan_outcome(reactive_escaped): safety net for the localized
    reactive-redescend mechanism (reactivesequence(Code)/
    reactivefallback(Code) in basic_action_theory.pl) -- a `reactive(_)`
    status escaping all the way to the root is always a translator bug,
    so this should read 0.00% on every problem; a nonzero reading here
    means some reactive-classified trigger's code has no matching
    enclosing reactivesequence/reactivefallback in the tree.

hit_by/1, first_hit/1, on_track/1, verify_safe/0, and plan_route_
blocked/0 are all still DEFINED in basic_action_theory.pl -- only their
query(...) declarations (and this script's own per-sample report
formatting) were removed, not the underlying predicates. Re-add
whichever query(...) line(s) are wanted again, and a matching report
section, to bring per-sample hazard/drift reporting back.

Usage:
    python3 main.py [--problem NAME]
        (default NAME: problem0 -- see problems/problem0/ for its
        config.yaml, behavior_tree.xml, goal_formula.pl, and map.yaml)

Before running inference, this script, for the SELECTED problem:
  1. regenerates <problem>/obstacles_generated.pl from
     <problem>/map.yaml (see module/translators/occgrid_to_problog.py's
     own header) -- map.yaml is the single source of truth for the
     obstacle layout.
  2. regenerates <problem>/config_generated.pl from <problem>/config.yaml
     (see module/translators/config_to_prolog.py's own header) --
     config.yaml is the single source of truth for every tunable
     constant in the theory (noise sigmas, the Z discretization
     tables, battery drain rates, robot/safety thresholds, tolerances,
     verification resolution, and the robot's own starting position).
  3. translates <problem>/behavior_tree.xml -- a real BT.cpp v4 tree,
     the single source of truth for the POLICY'S SHAPE -- into
     <problem>/plan_generated.pl, validating it against
     module/contracts/schema.yaml on the way (see module/translators/
     bt_to_prolog.py's own header).
  4. validates <problem>/goal_formula.pl -- the hand-authored
     verification goal for THIS particular plan, and the ONLY place
     goal information lives in this theory -- against module/contracts/
     vocabulary.yaml (see module/contracts/goal_formula_check.py's own
     header): every predicate it calls must be a known fluent, and the
     whole formula must be uniform in one situation (Reiter's own
     sense).
  5. (re)writes module/theory/problem_data.pl, a small bootstrap file
     basic_action_theory.pl itself consults (see that file's Section 0)
     that points -- via absolute paths -- at the four files above, so
     the SAME theory file serves whichever problem was selected. Also
     sets the BT_PROBLEM_DIR environment variable to the selected
     problem's own directory, which module/theory/planners.py and
     module/theory/collision_geometry.py (Python black boxes ProbLog
     loads directly) read at import time for the same reason.
All five steps mean a normal run always reflects whatever is currently
in the selected problem's own map.yaml / config.yaml / behavior_tree.xml
/ goal_formula.pl, with no separate regeneration step needed.

Requires: `problog` importable/runnable on PATH.

Saves a timestamped log file with a compact query-result summary --
no image/plot is produced (see print_compact_summary).
"""

import argparse
import os
import shutil
import sys
from datetime import datetime

from problog.errors import ProbLogError

from pipeline_stages import run_staged_inference, write_problem_data_pl, StageTimeout

W = 68

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_DIR = os.path.join(_THIS_DIR, "module")
THEORY_DIR = os.path.join(MODULE_DIR, "theory")
TRANSLATORS_DIR = os.path.join(MODULE_DIR, "translators")
CONTRACTS_DIR = os.path.join(MODULE_DIR, "contracts")
PROBLEMS_DIR = os.path.join(_THIS_DIR, "problems")
THEORY_PATH = os.path.join(THEORY_DIR, "basic_action_theory.pl")
OUTPUT_DIR = os.path.join(_THIS_DIR, "output")


# -----------------------------------------------------------------------
# Run ProbLog via its PYTHON API (not the CLI/subprocess) and return a
# results dict directly. str(term) for a query like plan_outcome(true)
# is EXACTLY the same string PARSE_RESULTS used to extract from CLI
# text output -- so everything downstream (print_compact_summary)
# needs zero changes; only how the numbers get INTO the dict changes.
#
# Delegates to pipeline_stages.run_staged_inference for the actual
# parse/ground/compile/evaluate work -- see that module's own header
# for why this is staged (one [STAGE] log line + timeout per phase)
# rather than the single opaque call this function used to make
# directly. elapsed here is the SUM of the four stage timings, kept
# for the "Finished: ...(elapsed)" line further down in main(); the
# per-stage breakdown is already visible in the [STAGE] lines
# run_staged_inference logs as it goes.
# -----------------------------------------------------------------------
def run_problog_api(plan_file, tee, phase_timeout=300):
    results, timings = run_staged_inference(plan_file, tee, phase_timeout=phase_timeout)
    return results, sum(timings.values())


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

def extract_goal_formula_text(goal_formula_path):
    """The problem's own goal_formula.pl, stripped down to its actual
    clause(s) -- drops '%'-prefixed comment lines and blank lines, so
    the compact report can show WHAT was actually verified without
    dumping that file's own (often long) header comment. Returns the
    remaining lines joined with '\n', or a placeholder if the file
    turns out to be all comments/blank (shouldn't happen in practice,
    but this is report formatting, not validation -- goal_formula_check
    .py already validated the real file earlier in this same run)."""
    with open(goal_formula_path) as f:
        lines = [ln.rstrip() for ln in f
                 if ln.strip() and not ln.strip().startswith("%")]
    return "\n".join(lines) if lines else "(no clause found)"


# query(...) names this report covers -- kept in sync with
# basic_action_theory.pl's own Section 10 QUERIES list by hand (there
# are only six now, trimmed specifically to what this reactive-
# redescend/merge-grid investigation needs -- see that section's own
# comment for what else is still DEFINED but no longer queried).
SUMMARY_QUERIES = [
    "verify_goal_formula",
    "any_collision",
    "any_battery_depletion",
    "plan_outcome(true)",
    "plan_outcome(false)",
    "plan_outcome(world_too_large)",
    "plan_outcome(reactive_escaped)",
]


def print_compact_summary(tee, results, goal_formula_path):
    section(tee, "Goal formula")
    tee(f"  {goal_formula_path}")
    for line in extract_goal_formula_text(goal_formula_path).split("\n"):
        tee(f"    {line}")

    section(tee, "Query results")
    label_w = max(len(name) for name in SUMMARY_QUERIES)
    tee(f"  {'Query':<{label_w}}   Probability")
    tee(f"  {'-'*label_w}   -----------")
    for name in SUMMARY_QUERIES:
        if name not in results:
            # any_battery_depletion specifically: absent (not just 0)
            # when this problem's own config.yaml has battery.enabled:
            # false -- config_to_prolog.py then never declares that
            # query at all (see its own header). Reporting 0.00% here
            # would misleadingly read as "verified never happens"
            # rather than "not modeled for this problem".
            tee(f"  {name:<{label_w}}   N/A (not queried)")
            continue
        p = results[name]
        tee(f"  {name:<{label_w}}   {p*100:6.2f}%")


# -----------------------------------------------------------------------
# main
# -----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--problem", default="problem0",
                     help="Name of the problem to run -- a subdirectory "
                          "of problems/ holding this problem's own "
                          "config.yaml, behavior_tree.xml, "
                          "goal_formula.pl, and map.yaml (default: "
                          "problem0).")
    ap.add_argument("--phase-timeout", type=int, default=300,
                     help="Per-stage timeout in seconds for the ProbLog "
                          "resolution pipeline -- parse/ground/compile/"
                          "evaluate each get their own budget (default: "
                          "300 = 5 minutes). See pipeline_stages.py.")
    args = ap.parse_args()

    problem_dir = os.path.join(PROBLEMS_DIR, args.problem)

    # Each run's own report lives in output/<problem>/, wiped and
    # recreated fresh every time -- this is a REPORT of the run, not an
    # input any other file depends on, so there's no reason to keep
    # stale runs around the way problems/<name>/'s own generated
    # Prolog facts are (those get committed; this doesn't -- see
    # .gitignore).
    problem_output_dir = os.path.join(OUTPUT_DIR, args.problem)
    if os.path.isdir(problem_output_dir):
        shutil.rmtree(problem_output_dir)
    os.makedirs(problem_output_dir)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(problem_output_dir, f"{args.problem}_{ts}.log")

    with open(log_path, "w", encoding="utf-8") as fh:
        tee = Tee(fh)
        banner(tee, f"ProbLog Continuous-Space Safety Verification - {datetime.now():%Y-%m-%d %H:%M:%S}")
        tee(f"  Log file    : {log_path}")
        tee(f"  Problem     : {args.problem} ({problem_dir})")
        tee(f"  Theory file : {THEORY_PATH}")

        if not os.path.isdir(problem_dir):
            tee(f"\n  [ERROR] No such problem directory: {problem_dir}")
            sys.exit(1)
        if not os.path.isfile(THEORY_PATH):
            tee(f"\n  [ERROR] File not found: {THEORY_PATH}")
            sys.exit(1)

        # Every Python black box basic_action_theory.pl :- use_module()s
        # (planners.py, collision_geometry.py) reads BT_PROBLEM_DIR at
        # IMPORT time to find this problem's own map.yaml/config.yaml/
        # obstacles_generated.pl -- must be set before ProbLog ever
        # loads the theory (see run_problog_api below).
        os.environ["BT_PROBLEM_DIR"] = problem_dir

        # Regenerate <problem>/obstacles_generated.pl from
        # <problem>/map.yaml BEFORE anything reads the theory --
        # map.yaml is the single source of truth for the obstacle
        # layout (see module/translators/occgrid_to_problog.py), same
        # automatic-every-run treatment config.yaml/behavior_tree.xml
        # already get.
        if TRANSLATORS_DIR not in sys.path:
            sys.path.insert(0, TRANSLATORS_DIR)
        try:
            from occgrid_to_problog import generate as generate_obstacles
            generated_obstacles_path = generate_obstacles(
                yaml_path=os.path.join(problem_dir, "map.yaml"),
                output_path=os.path.join(problem_dir, "obstacles_generated.pl"))
            tee(f"  Obstacles   : {generated_obstacles_path} (regenerated from "
                f"{os.path.join(problem_dir, 'map.yaml')})")
        except Exception as e:
            tee(f"\n  [ERROR] Could not regenerate obstacles_generated.pl: {e}")
            sys.exit(1)

        # Regenerate <problem>/config_generated.pl from
        # <problem>/config.yaml BEFORE anything reads the theory --
        # config.yaml is the single source of truth for every tunable
        # constant (see module/translators/config_to_prolog.py), so
        # every run picks up whatever is currently there with no
        # separate step.
        try:
            from config_to_prolog import generate as generate_config, load_config
            generated_config_path = generate_config(
                config_path=os.path.join(problem_dir, "config.yaml"),
                output_path=os.path.join(problem_dir, "config_generated.pl"))
            battery_enabled = load_config(
                os.path.join(problem_dir, "config.yaml")
            ).get("battery", {}).get("enabled", True)
            tee(f"  Config      : {generated_config_path} (regenerated from "
                f"{os.path.join(problem_dir, 'config.yaml')})")
        except Exception as e:
            tee(f"\n  [ERROR] Could not regenerate config_generated.pl: {e}")
            sys.exit(1)

        # Translate <problem>/behavior_tree.xml (the real BT.cpp v4
        # tree that is now the single source of truth for the POLICY'S
        # SHAPE) into <problem>/plan_generated.pl, validating it
        # against module/contracts/schema.yaml on the way -- see
        # module/translators/bt_to_prolog.py's own header. Any
        # structural problem (unknown node, missing/unrecognized port,
        # a control_points blackboard key with no producer) is a hard
        # failure here, same as a missing config fact above; there is
        # no sensible way to run inference against a tree that doesn't
        # actually match its own schema. battery_enabled (this
        # problem's own config.yaml battery.enabled, read above) is
        # threaded through so a disabled problem gets every battery-
        # related trigger name stripped from its own Triggers lists --
        # see bt_to_prolog.py's own generate_plan_pl/_is_battery_trigger.
        try:
            from bt_to_prolog import generate_plan_pl, BTValidationError
            generated_plan_path = generate_plan_pl(
                xml_path=os.path.join(problem_dir, "behavior_tree.xml"),
                schema_path=os.path.join(CONTRACTS_DIR, "schema.yaml"),
                output_path=os.path.join(problem_dir, "plan_generated.pl"),
                battery_enabled=battery_enabled)
            tee(f"  Plan (BT)   : {generated_plan_path} (translated + validated "
                f"from {os.path.join(problem_dir, 'behavior_tree.xml')})")
        except BTValidationError as e:
            tee(f"\n  [ERROR] behavior_tree.xml failed validation: {e}")
            sys.exit(1)
        except Exception as e:
            tee(f"\n  [ERROR] Could not translate behavior_tree.xml: {e}")
            sys.exit(1)

        # Validate <problem>/goal_formula.pl against
        # module/contracts/vocabulary.yaml -- same "structural
        # validation is a hard failure, not a warning" posture as
        # behavior_tree.xml's own validation just above; see
        # module/contracts/goal_formula_check.py's own header.
        if CONTRACTS_DIR not in sys.path:
            sys.path.insert(0, CONTRACTS_DIR)
        try:
            from goal_formula_check import validate_goal_formula, GoalFormulaValidationError
            goal_formula_path = os.path.join(problem_dir, "goal_formula.pl")
            validate_goal_formula(
                goal_formula_path=goal_formula_path,
                vocab_path=os.path.join(CONTRACTS_DIR, "vocabulary.yaml"))
            tee(f"  Goal formula: {goal_formula_path} (validated against "
                f"{os.path.join(CONTRACTS_DIR, 'vocabulary.yaml')})")
        except GoalFormulaValidationError as e:
            tee(f"\n  [ERROR] goal_formula.pl failed validation: {e}")
            sys.exit(1)
        except Exception as e:
            tee(f"\n  [ERROR] Could not validate goal_formula.pl: {e}")
            sys.exit(1)

        # Rewrite module/theory/problem_data.pl -- the small bootstrap
        # file basic_action_theory.pl itself :- consult()s (see that
        # file's Section 0) to find the four problem-specific files
        # above. Written with ABSOLUTE paths since problems/<name>/ is
        # nowhere near module/theory/ on disk, and regenerated fresh
        # every run so basic_action_theory.pl never has to change to
        # serve a different --problem.
        problem_data_path = os.path.join(THEORY_DIR, "problem_data.pl")
        write_problem_data_pl(problem_data_path, problem_dir, goal_formula_path,
                               run_label=f"main.py --problem {args.problem}, run {ts}",
                               tee=tee)

        tee(f"\n  Started : {datetime.now():%H:%M:%S}  "
            f"(phase timeout: {args.phase_timeout}s per stage)")
        try:
            results, elapsed = run_problog_api(THEORY_PATH, tee, args.phase_timeout)
        except StageTimeout:
            tee(f"\n  [ERROR] Aborting -- a pipeline stage exceeded its "
                f"{args.phase_timeout}s timeout (see [STAGE] line above for which).")
            sys.exit(1)
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

        print_compact_summary(tee, results, goal_formula_path)

        tee("")
        banner(tee, f"Log : {log_path}")


if __name__ == "__main__":
    main()
