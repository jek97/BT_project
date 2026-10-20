"""
grounding_probe.py

Two probes into ProbLog's GROUND stage, used by timing_profile.py.
Neither modifies ProbLog on disk -- both patch classes in-process.

EngineSampler -- where does the engine spend its time, per THEORY predicate
and per kind of operation?
  ProbLog's engine is a non-recursive stack machine (engine_stack.py): a
  Prolog call is an EvalDefine record on engine.stack, linked to its caller
  by a parent pointer, so a Python profiler cannot see the Prolog call
  chain. This sampler fires on SIGPROF every `interval` seconds of CPU
  time, finds the EvalNode(s) being worked on in the current Python frames,
  and follows their parent pointers to rebuild the Prolog call chain.
  Per sample it records:
    - inclusive: every distinct predicate on the chain (the time "under"
      that predicate, its callees included -- like cumtime);
    - self: the innermost predicate on the chain (like tottime);
    - operation: which part of ProbLog's code the interpreter was in
      (innermost frame from a recognised file, see _CATEGORIES).
  Also counts, per predicate, how many times the engine was asked to
  evaluate it (eval_define) and how many of those were answered from
  ProbLog's table (cache hit) instead of being resolved again.

FormulaProbe -- how often are ground nodes created, folded or merged,
and what does it cost?
  The ground graph is built by LogicFormula.add_atom/add_and/add_or/
  add_disjunct/add_name, which are called by the engine as proofs are
  found. Deterministic sub-proofs are folded INSIDE these calls
  (formula.py _add_compound): a TRUE child is dropped from an AND, a
  FALSE child from an OR, a one-child AND/OR returns the child itself,
  and an AND/OR identical to an existing node is reused (hash-consing).
  Every call is timed (outermost only, so nested calls aren't counted
  twice) and classified by its outcome.
"""

import os
import signal
import time
from collections import Counter, defaultdict

from problog import engine_stack
from problog.eval_nodes import EvalNode, EvalDefine
from problog.formula import LogicFormula


# ---------------------------------------------------------------------------
# EngineSampler
# ---------------------------------------------------------------------------

# Innermost frame from one of these files decides the sample's operation.
# Helper modules (logic.py term ops, util.py, ...) are skipped, so their time
# goes to whoever called them.
_CATEGORIES = {
    "collision_geometry.py": "external Python predicates (computation)",
    "planners.py": "external Python predicates (computation)",
    "extern.py": "external Python predicates (ProbLog conversion)",
    "formula.py": "ground-graph construction (add_and/or/atom, folding)",
    "engine_builtin.py": "builtins (is/2, comparisons, findall, ...)",
    "engine_unify.py": "unification / substitution",
    "clausedb.py": "clause lookup (indexing in the ClauseDB)",
    "eval_nodes.py": "engine core (eval nodes, result propagation, tabling)",
    "engine_stack.py": "engine core (eval nodes, result propagation, tabling)",
}


class EngineSampler:
    def __init__(self, interval=0.001):
        self.interval = interval
        self.samples = 0
        self.inclusive = Counter()
        self.self_ = Counter()
        self.category = Counter()
        self.no_chain = 0
        self.define_calls = Counter()
        self.define_cache_hits = Counter()
        self._old_handler = None
        self._orig_eval_define = None

    # -- per-predicate call counts ------------------------------------------
    def _patch_eval_define(self):
        sampler = self
        orig = engine_stack.StackBasedEngine.eval_define
        self._orig_eval_define = orig

        def eval_define(self_eng, node, context, target, parent, *a, **kw):
            key = f"{node.functor}/{len(context)}"
            sampler.define_calls[key] += 1
            if not kw.get("no_cache") and hasattr(target, "_cache"):
                if target._cache.get((node.functor, context)) is not None:
                    sampler.define_cache_hits[key] += 1
            return orig(self_eng, node, context, target, parent, *a, **kw)

        engine_stack.StackBasedEngine.eval_define = eval_define

    # -- sampling --------------------------------------------------------------
    @staticmethod
    def _chain(node, seen):
        """Predicates on the parent chain starting at EvalNode `node`."""
        preds = []
        stack = node.engine.stack
        n = node
        while n is not None:
            if n.pointer in seen:
                break
            seen.add(n.pointer)
            if isinstance(n, EvalDefine):
                preds.append(f"{n.node.functor}/{len(n.context)}")
            p = n.parent
            if p is None or p >= len(stack):
                break
            n = stack[p]
        return preds

    def _handler(self, signum, frame):
        self.samples += 1
        category = None
        preds = []          # innermost first
        seen = set()
        f = frame
        while f is not None:
            fn = os.path.basename(f.f_code.co_filename)
            if category is None:
                category = _CATEGORIES.get(fn)
            slf = f.f_locals.get("self") if fn in ("eval_nodes.py", "engine_stack.py") else None
            try:
                if isinstance(slf, EvalNode):
                    preds += self._chain(slf, seen)
                elif slf is not None and fn == "engine_stack.py":
                    # engine.eval_*(..., parent=<pointer>, ...): a call being
                    # set up, not yet an EvalNode of its own.
                    p = f.f_locals.get("parent")
                    if isinstance(p, int) and 0 <= p < len(slf.stack) and slf.stack[p] is not None:
                        node = f.f_locals.get("node")
                        ctx = f.f_locals.get("context")
                        if (f.f_code.co_name == "eval_define" and node is not None
                                and ctx is not None):
                            preds.append(f"{node.functor}/{len(ctx)}")
                        preds += self._chain(slf.stack[p], seen)
            except Exception:
                pass
            f = f.f_back
        self.category[category or "other (outside the engine)"] += 1
        if not preds:
            self.no_chain += 1
            return
        self.self_[preds[0]] += 1
        for p in set(preds):
            self.inclusive[p] += 1

    def start(self):
        self._patch_eval_define()
        self._old_handler = signal.signal(signal.SIGPROF, self._handler)
        signal.setitimer(signal.ITIMER_PROF, self.interval, self.interval)

    def stop(self):
        signal.setitimer(signal.ITIMER_PROF, 0, 0)
        if self._old_handler is not None:
            signal.signal(signal.SIGPROF, self._old_handler)
        if self._orig_eval_define is not None:
            engine_stack.StackBasedEngine.eval_define = self._orig_eval_define


# ---------------------------------------------------------------------------
# FormulaProbe
# ---------------------------------------------------------------------------

class FormulaProbe:
    METHODS = ("add_atom", "add_and", "add_or", "add_disjunct", "add_name", "add_not")

    def __init__(self):
        self.outcomes = defaultdict(Counter)   # method -> outcome -> count
        self.time = Counter()                  # method -> outermost seconds
        self.calls = Counter()
        self.total_time = 0.0
        self.true_inputs_dropped = 0           # TRUE children dropped from ANDs
        self.false_inputs_dropped = 0          # FALSE children dropped from ORs
        self._depth = 0
        self._orig = {}

    @staticmethod
    def _classify_compound(lf, result, components, n_before, nodetype):
        if len(lf._nodes) > n_before:
            return "new node created"
        if result == lf.TRUE:
            return ("folded to TRUE (deterministic)" if nodetype == "and"
                    else "folded to TRUE (a TRUE disjunct)")
        if result is None:
            return ("folded to FALSE (a FALSE conjunct)" if nodetype == "and"
                    else "folded to FALSE (no disjuncts)")
        if result in components:
            return "collapsed to its single remaining child"
        return "merged into an identical existing node"

    def _wrap(self, name):
        probe = self
        orig = getattr(LogicFormula, name)
        self._orig[name] = orig

        def wrapper(lf, *args, **kw):
            outer = probe._depth == 0
            probe._depth += 1
            n_before = len(lf._nodes)
            t0 = time.perf_counter()
            try:
                result = orig(lf, *args, **kw)
            finally:
                dt = time.perf_counter() - t0
                probe._depth -= 1
            probe.calls[name] += 1
            if outer:
                probe.time[name] += dt
                probe.total_time += dt

            if name in ("add_and", "add_or"):
                comps = tuple(args[0]) if args else tuple(kw.get("components", ()))
                kind = "and" if name == "add_and" else "or"
                if kind == "and":
                    probe.true_inputs_dropped += sum(1 for c in comps if c == lf.TRUE)
                else:
                    probe.false_inputs_dropped += sum(1 for c in comps if c is None)
                probe.outcomes[name][probe._classify_compound(
                    lf, result, comps, n_before, kind)] += 1
            elif name == "add_atom":
                if len(lf._nodes) > n_before:
                    o = "new node created"
                elif result == lf.TRUE:
                    o = "deterministic (TRUE, no node)"
                elif result is None:
                    o = "deterministic (FALSE, no node)"
                else:
                    o = "merged into an identical existing node"
                probe.outcomes[name][o] += 1
            elif name == "add_disjunct":
                probe.outcomes[name]["new node created" if len(lf._nodes) > n_before
                                     else "updated/no-op"] += 1
            else:
                probe.outcomes[name]["call"] += 1
            return result

        setattr(LogicFormula, name, wrapper)

    def start(self):
        for m in self.METHODS:
            self._wrap(m)

    def stop(self):
        for m, orig in self._orig.items():
            setattr(LogicFormula, m, orig)
