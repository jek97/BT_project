#!/usr/bin/env python3
"""
plan_generation/goal_formula_check.py

Validates plan_generation/plan/goal_formula.pl against
plan_generation/vocabulary.yaml BEFORE it's ever consulted/queried by
ProbLog -- exactly the same "structural validation is a hard failure,
not a warning" posture bt_to_prolog.py already takes for
behavior_tree.xml against actions/schema.yaml (see that file's own
header; this module is its direct sibling for the goal-formula side
of the pipeline).

TWO checks, matching this project's own discussion of what "a
well-formed goal formula" means:

  1. Every predicate goal_formula.pl's body calls (by name/arity) is a
     KNOWN predicate in vocabulary.yaml -- catches a typo, or a
     reference to a fluent that was renamed/removed/never existed, the
     same class of error an unknown BT.cpp node tag already catches
     for behavior_tree.xml.

  2. The formula is UNIFORM in its own head's situation argument
     (Reiter's own sense, "Knowledge in Action": the SAME single
     situation term appears in every situation-argument slot across
     the whole formula, never two different ones -- see
     vocabulary.yaml's own header for the full definition and why the
     situation-argument POSITION has to be looked up per-predicate,
     not assumed). Catches an accidentally-introduced second situation
     variable, which would silently change the formula's meaning from
     "check this property at ONE situation" to something else.

Uses ProbLog's own Prolog parser (problog.program.PrologString) to get
a REAL parsed term structure for goal_formula.pl's clause, rather than
a hand-rolled regex over the text -- the same reasoning this project
already used for bt_to_prolog.py's own XML parsing
(xml.etree.ElementTree, not a hand-rolled tag scanner).

Usage:
    python3 plan_generation/goal_formula_check.py
        (validates the default goal_formula.pl against the default
        vocabulary.yaml, printing OK or a validation error)

run_plan_continuous_safety.py calls validate_goal_formula() itself
before every run, so you don't normally need to run this by hand.
"""
import os
import sys

import yaml
from problog.logic import And, Not, Var
from problog.program import PrologString

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_VOCAB_PATH = os.path.join(_THIS_DIR, "vocabulary.yaml")
DEFAULT_GOAL_FORMULA_PATH = os.path.join(_THIS_DIR, "plan", "goal_formula.pl")


class GoalFormulaValidationError(Exception):
    """Raised for any structural problem in goal_formula.pl relative to
    vocabulary.yaml -- always fatal, never a warning (same posture as
    bt_to_prolog.py's own BTValidationError)."""


def load_vocabulary(vocab_path=DEFAULT_VOCAB_PATH):
    with open(vocab_path) as f:
        vocab = yaml.safe_load(f)
    index = {}
    for entry in vocab.get("predicates", []):
        index[(entry["name"], entry["arity"])] = entry
    return index


def _iter_conjuncts(body):
    """Flatten a right-associative And(...) tree into a flat list of
    leaf subgoals to VALIDATE, in source order. Negation (\\+/1) is
    transparent here -- \\+ crashed_in(S) is still perfectly uniform
    (the situation argument inside is still just S), \\+ itself is a
    logical connective, not a vocabulary predicate, so it's unwrapped
    and its own argument is recursed into rather than looked up.
    Disjunction (';'/2) is NOT handled yet -- goal_formula.pl is
    expected to be a plain conjunction (possibly negated), see this
    module's own header on what's NOT supported."""
    if isinstance(body, And):
        left, right = body.args
        return _iter_conjuncts(left) + _iter_conjuncts(right)
    if isinstance(body, Not):
        return _iter_conjuncts(body.args[0])
    return [body]


def _arity(term):
    return len(term.args) if term.args else 0


def _situation_arg_position(vocab_index, term, goal_formula_path):
    """Looks up term's own functor/arity in the vocabulary. Returns
    the 1-based situation-argument position, or None if this
    predicate has no situation argument at all (a plain fact/helper --
    see vocabulary.yaml's own note on situation_arg: null). Raises
    GoalFormulaValidationError if the predicate isn't in the
    vocabulary at all (check 1)."""
    arity = _arity(term)
    key = (term.functor, arity)
    if key not in vocab_index:
        raise GoalFormulaValidationError(
            f"{goal_formula_path} calls '{term.functor}/{arity}', which is "
            f"not a known predicate in vocabulary.yaml -- typo, or a "
            f"fluent that isn't (yet) documented there? See "
            f"vocabulary.yaml's own predicates list for what's available.")
    return vocab_index[key].get("situation_arg")


def validate_goal_formula(goal_formula_path=DEFAULT_GOAL_FORMULA_PATH,
                           vocab_path=DEFAULT_VOCAB_PATH):
    """Parses goal_formula_path, checks it defines EXACTLY ONE
    goal_formula/1 clause whose own head argument is a variable,
    checks every subgoal in its body against vocab_path (check 1), and
    checks the whole formula is uniform in that one head variable
    (check 2). Raises GoalFormulaValidationError on any failure;
    returns nothing on success."""
    vocab_index = load_vocabulary(vocab_path)

    with open(goal_formula_path) as f:
        text = f.read()

    clauses = list(PrologString(text))
    goal_clauses = [c for c in clauses
                    if getattr(c, "functor", None) == ":-"
                    and c.args[0].functor == "goal_formula"]
    if len(goal_clauses) != 1:
        raise GoalFormulaValidationError(
            f"{goal_formula_path} must define EXACTLY ONE goal_formula/1 "
            f"clause (found {len(goal_clauses)}). Local helper predicates "
            f"alongside it are not yet supported by this checker -- see "
            f"vocabulary.yaml's own 'CURRENT LIMITATION' note.")

    clause = goal_clauses[0]
    head, body = clause.args
    if _arity(head) != 1:
        raise GoalFormulaValidationError(
            f"{goal_formula_path} defines goal_formula/{_arity(head)} -- "
            f"must be goal_formula/1, one argument, the situation to "
            f"check the formula at.")
    situation_var = head.args[0]
    if not isinstance(situation_var, Var):
        raise GoalFormulaValidationError(
            f"{goal_formula_path}'s goal_formula(...) argument must be a "
            f"VARIABLE (e.g. 'S'), not a ground term ('{situation_var}') "
            f"-- it has to be applicable at whichever situation the "
            f"caller supplies (final_situation, in practice via "
            f"moveto_continuous.pl's own verify_goal_formula wrapper), "
            f"not hardwired to one here.")

    for term in _iter_conjuncts(body):
        pos = _situation_arg_position(vocab_index, term, goal_formula_path)
        if pos is None:
            continue
        arity = _arity(term)
        if pos < 1 or pos > arity:
            raise GoalFormulaValidationError(
                f"vocabulary.yaml's entry for {term.functor}/{arity} "
                f"declares situation_arg={pos}, out of range for this "
                f"predicate's own arity -- fix vocabulary.yaml.")
        actual = term.args[pos - 1]
        if actual != situation_var:
            raise GoalFormulaValidationError(
                f"{goal_formula_path} is not uniform in its own situation "
                f"argument '{situation_var}': {term.functor}/{arity} is "
                f"applied to '{actual}' (argument {pos}) instead. Every "
                f"fluent in a goal formula must be checked at the SAME "
                f"situation -- see vocabulary.yaml's own header for "
                f"Reiter's 'uniform in s' definition. A formula that "
                f"genuinely needs to relate two different situations "
                f"(e.g. 'visited A before visited B') isn't supported yet.")


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_GOAL_FORMULA_PATH
    try:
        validate_goal_formula(path)
    except GoalFormulaValidationError as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)
    print(f"{path}: OK")
