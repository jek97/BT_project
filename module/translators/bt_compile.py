#!/usr/bin/env python3
"""
module/translators/bt_compile.py

Compiles a translated behavior tree -- the plan/1 term and the
reactive_children/2 facts bt_to_prolog.py writes into plan_generated.pl
-- into ONE PROLOG PREDICATE PER CONTROL-FLOW NODE, so the theory never
has to interpret the tree at run time.

WHY: the interpreted form (do_node(Node, S, S1, Outcome) with ~33
clauses, one per node type) makes ProbLog try every do_node clause on
every call: its clause index only uses arguments that are fully ground
in the clause head, and neither seq_node([Child|Rest]) nor a call like
do_node(seq_node([moveto_leg(CP1,...),...]),...) is. Every visit to a
node therefore leaves ~30 "head failed" dead ends in the SLD tree, and
seq_step/fallback_step/invert add one more each through their
non-ground reactive(Code) clause. The translator already knows every
node's type, so it can write the dispatch out instead.

WHAT IS GENERATED (S/S1 situations, O the outcome, as in do_node/4):

  bt_root(S, S1, O)                the tree's entry point (used by
                                   final_situation/1 and plan_outcome/1).

  Sequence #N, children C1..Cn:
    bt_seqN(P.., S, S1, O) :- C1(.., S, S2, O1), bt_seqN_after1(O1, Q1.., S2, S1, O).
    bt_seqN_afterK(true, QK.., S, S1, O) :- C(K+1)(.., S, S2, O1), bt_seqN_after(K+1)(O1, ..).
    bt_seqN_afterN(true, _.., S, S, true).
    bt_seqN_afterK(false, _.., S, S, false).
    bt_seqN_afterK(reactive(rc1), _.., S, S, reactive(rc1)).   (see below)
  Fallback #N: the same with true/false swapped.
  Inverter #N: bt_invN(..) :- C(..), bt_invN_flip(O1, O), flip/2 facts.
  ReactiveSequence/ReactiveFallback with code rcK:
    bt_rcK(S, S1, O) :- replan_budget(B), bt_rcK_pass(B, S, S1, O).
    bt_rcK_pass(B, S, S1, O) :- bt_<body>(S, S2, O1), bt_rcK_after(O1, B, S, S2, S1, O).
    bt_rcK_after/6: reactive_after/8 of the interpreted theory,
    specialised to this code.

  A leaf F(A1..An) is called directly as leaf_F(A1..An, S, S2, O1) (see
  basic_action_theory.pl's leaf_* predicates); planWith with a
  follow_boarder(_,_) algorithm calls leaf_planWith_follow_boarder.

Every "after"/"flip" clause has a GROUND first argument, so ProbLog's
index picks exactly one clause per call. That is why a reactive(Code)
clause is written once per concrete code, and only for the codes that
can actually come out of that child: a leaf can only halt with
reactive(Code) for a Code written in its own Triggers list, and a
reactive composite catches its own code and passes the rest on. A
child that can't produce any reactive outcome (every non-reactive tree)
gets no reactive clause at all.

VARIABLES: blackboard variables (CP1, Obst1, ...) keep the scope they
had in the interpreted tree -- the plan/1 term, or one
reactive_children/2 fact, whose variables are fresh on every restart.
A generated predicate takes as arguments exactly the variables that
occur both inside its subtree and elsewhere in the same scope (an
"after" predicate: in the children still to run and elsewhere), so
bindings flow between siblings exactly as they did through the shared
term, and each tabled call is keyed on the same bindings the
interpreted do_node call was.

Semantics are unchanged clause for clause: same situations, same
outcomes, same dropped outcomes (an outcome with no after clause, e.g.
world_too_large from a nested reactive composite, fails the parent just
as it had no seq_step/fallback_step clause before).
"""

from collections import Counter

from problog.logic import Constant, Term, Var
from problog.program import PrologString

COMPOSITES = {"seq_node", "fallback_node", "inverter",
              "reactivesequence", "reactivefallback"}
LEAVES = {"cond", "take_sample", "moveto_leg", "install_tool_leg",
          "uninstall_tool_leg", "deploy_tool_leg", "retract_tool_leg",
          "planWithWaypoints", "planWith", "tool_position_query",
          "tools_of_kind_query", "nearest_tool_of_kind_query",
          "hitched_id_query"}
# Variable names the generated clauses use themselves; a blackboard
# variable with one of these names is renamed (suffix _bb) on output.
RESERVED = {"S", "S1", "S2", "O", "O1", "B", "B1"}


class BTCompileError(Exception):
    """The plan/reactive_children terms can't be compiled."""


def _plist_items(t):
    items = []
    while isinstance(t, Term) and t.functor == "." and t.arity == 2:
        items.append(t.args[0])
        t = t.args[1]
    if not (isinstance(t, Term) and t.functor == "[]" and t.arity == 0):
        raise BTCompileError(f"expected a proper list, got tail {t}")
    return items


def _var_occurrences(t, counter):
    """Count named-variable occurrences in t ('_' is anonymous: never shared)."""
    stack = [t]
    while stack:
        x = stack.pop()
        if isinstance(x, Var):
            if x.name != "_":
                counter[x.name] += 1
        elif isinstance(x, Term):
            stack.extend(x.args)
    return counter


def _atoms(t):
    out, stack = set(), [t]
    while stack:
        x = stack.pop()
        if isinstance(x, Term) and not isinstance(x, (Var, Constant)):
            if x.arity == 0:
                out.add(str(x.functor))
            stack.extend(x.args)
    return out


class _Renamer(dict):
    def __missing__(self, name):
        return Var(name)


class _Compiler:
    def __init__(self, plan_term, reactive):
        self.plan = plan_term
        self.reactive = reactive                       # code -> [child terms]
        self.codes = set(reactive)
        self.clauses = {}                              # predicate -> [clause text], in first-emit order
        self.counters = Counter()
        self._esc_cache = {}
        rename = _Renamer()
        for t in [plan_term] + [c for cs in reactive.values() for c in cs]:
            for name in _var_occurrences(t, Counter()):
                if name in RESERVED:
                    rename[name] = Var(name + "_bb")
        self.rename = rename

    # -- helpers ---------------------------------------------------------
    def fmt(self, t):
        return str(t.apply(self.rename)) if self.rename else str(t)

    def var(self, name):
        return self.fmt(Var(name))

    def args(self, *parts):
        return ", ".join(p for p in parts if p != "")

    def vars_text(self, names):
        return ", ".join(self.var(n) for n in names)

    def blanks(self, names):
        return ", ".join("_" for _ in names)

    def new_name(self, kind):
        self.counters[kind] += 1
        return f"bt_{kind}{self.counters[kind]}"

    def emit(self, text):
        self.clauses.setdefault(text.split("(", 1)[0], []).append(text)

    @staticmethod
    def shared(scope_total, inside):
        """Variables of `inside` (a Counter) that also occur elsewhere in scope."""
        return [n for n in inside if scope_total[n] > inside[n]]

    # -- reactive codes that can escape a node ----------------------------
    def escapes(self, node, path=()):
        f = node.functor
        if f in ("seq_node", "fallback_node"):
            out = set()
            for c in _plist_items(node.args[0]):
                out |= self.escapes(c, path)
            return out
        if f == "inverter":
            return self.escapes(node.args[0], path)
        if f in ("reactivesequence", "reactivefallback"):
            code = str(node.args[0])
            if code in path:
                raise BTCompileError(f"reactive composite {code} contains itself")
            if code not in self._esc_cache:
                if code not in self.reactive:
                    raise BTCompileError(f"no reactive_children/2 fact for {code}")
                out = set()
                for c in self.reactive[code]:
                    out |= self.escapes(c, path + (code,))
                self._esc_cache[code] = out - {code}
            return self._esc_cache[code]
        return _atoms(node) & self.codes          # a leaf: codes in its own Triggers

    # -- calls -----------------------------------------------------------
    def call(self, node, scope_total, s_in, s_out, o):
        """Goal text running `node` from s_in to s_out with outcome o."""
        f = node.functor
        if f in LEAVES:
            name = f"leaf_{f}"
            if f == "planWith" and isinstance(node.args[0], Term) \
                    and node.args[0].functor == "follow_boarder" and node.args[0].arity == 2:
                name = "leaf_planWith_follow_boarder"
            return f"{name}({self.args(*(self.fmt(a) for a in node.args), s_in, s_out, o)})"
        if f in ("reactivesequence", "reactivefallback"):
            return f"bt_{node.args[0]}({s_in}, {s_out}, {o})"
        if f in COMPOSITES:
            inside = _var_occurrences(node, Counter())
            params = self.shared(scope_total, inside)
            pred = self.compile_composite(node, scope_total, params)
            return f"{pred}({self.args(self.vars_text(params), s_in, s_out, o)})"
        raise BTCompileError(f"unknown behavior-tree node {f}/{node.arity}: {node}")

    # -- composites ------------------------------------------------------
    def compile_composite(self, node, scope_total, params):
        f = node.functor
        if f == "inverter":
            return self.compile_inverter(node, scope_total, params)
        return self.compile_chain(f, _plist_items(node.args[0]), scope_total, params)

    def compile_chain(self, kind, children, scope_total, params):
        seq = kind == "seq_node"
        cont, stop = ("true", "false") if seq else ("false", "true")
        pred = self.new_name("seq" if seq else "fb")
        if not children:      # seq_node([]) succeeds, fallback_node([]) fails
            self.emit(f"{pred}({self.args(self.blanks(params), 'S', 'S', cont)}).")
            return pred
        n = len(children)
        # Q[k]: variables the children after child k need from outside them.
        suffix = [Counter() for _ in range(n + 1)]
        for k in range(n - 1, -1, -1):
            suffix[k] = _var_occurrences(children[k], Counter(suffix[k + 1]))
        q = [self.shared(scope_total, suffix[k + 1]) for k in range(n)]

        def after(k):
            return f"{pred}_after{k + 1}"

        first = self.call(children[0], scope_total, "S", "S2", "O1")
        self.emit(f"{pred}({self.args(self.vars_text(params), 'S', 'S1', 'O')}) :-\n"
                  f"    {first},\n"
                  f"    {after(0)}({self.args('O1', self.vars_text(q[0]), 'S2', 'S1', 'O')}).")
        for k in range(n):
            if k + 1 < n:
                nxt = self.call(children[k + 1], scope_total, "S", "S2", "O1")
                self.emit(f"{after(k)}({self.args(cont, self.vars_text(q[k]), 'S', 'S1', 'O')}) :-\n"
                          f"    {nxt},\n"
                          f"    {after(k + 1)}({self.args('O1', self.vars_text(q[k + 1]), 'S2', 'S1', 'O')}).")
            else:
                self.emit(f"{after(k)}({self.args(cont, self.blanks(q[k]), 'S', 'S', cont)}).")
            self.emit(f"{after(k)}({self.args(stop, self.blanks(q[k]), 'S', 'S', stop)}).")
            for code in sorted(self.escapes(children[k])):
                r = f"reactive({code})"
                self.emit(f"{after(k)}({self.args(r, self.blanks(q[k]), 'S', 'S', r)}).")
        return pred

    def compile_inverter(self, node, scope_total, params):
        pred = self.new_name("inv")
        child = node.args[0]
        self.emit(f"{pred}({self.args(self.vars_text(params), 'S', 'S1', 'O')}) :-\n"
                  f"    {self.call(child, scope_total, 'S', 'S1', 'O1')},\n"
                  f"    {pred}_flip(O1, O).")
        self.emit(f"{pred}_flip(true, false).")
        self.emit(f"{pred}_flip(false, true).")
        for code in sorted(self.escapes(child)):
            self.emit(f"{pred}_flip(reactive({code}), reactive({code})).")
        return pred

    def compile_reactive(self, code, kind):
        children = self.reactive[code]
        scope_total = Counter()
        for c in children:
            _var_occurrences(c, scope_total)
        body = self.compile_chain(kind, children, scope_total, [])
        p = f"bt_{code}"
        esc_body = set()
        for c in children:
            esc_body |= self.escapes(c, (code,))
        self.emit(f"{p}(S, S1, O) :-\n"
                  f"    replan_budget(B),\n"
                  f"    {p}_pass(B, S, S1, O).")
        self.emit(f"{p}_pass(B, S, S1, O) :-\n"
                  f"    {body}(S, S2, O1),\n"
                  f"    {p}_after(O1, B, S, S2, S1, O).")
        if code in esc_body:
            self.emit(f"{p}_after(reactive({code}), B, _, S2, S1, O) :-\n"
                      f"    B > 0,\n"
                      f"    B1 is B - 1,\n"
                      f"    {p}_pass(B1, S2, S1, O).")
            self.emit(f"{p}_after(reactive({code}), 0, S, _, S, world_too_large).")
        for other in sorted(esc_body - {code}):
            self.emit(f"{p}_after(reactive({other}), B, _, S2, S2, reactive({other})) :-\n"
                      f"    B > 0.")
        for o in ("true", "false"):
            self.emit(f"{p}_after({o}, B, _, S2, S2, {o}) :-\n"
                      f"    B > 0.")

    def run(self):
        root_total = _var_occurrences(self.plan, Counter())
        self.clauses["bt_root"] = []                  # listed first
        root_call = self.call(self.plan, root_total, "S", "S1", "O")
        self.emit(f"bt_root(S, S1, O) :-\n    {root_call}.")
        kinds = {}
        stack = [self.plan] + [c for cs in self.reactive.values() for c in cs]
        while stack:
            t = stack.pop()
            if isinstance(t, Term) and not isinstance(t, (Var, Constant)):
                if t.functor in ("reactivesequence", "reactivefallback") and t.arity == 1:
                    kinds[str(t.args[0])] = "seq_node" if t.functor == "reactivesequence" else "fallback_node"
                stack.extend(t.args)
        for code in sorted(self.reactive):
            if code not in kinds:
                continue      # a reactive_children/2 fact nothing refers to
            self.compile_reactive(code, kinds[code])
        return ["\n".join(cls) for cls in self.clauses.values()]


def compile_plan_text(text):
    """Compile the plan/1 + reactive_children/2 facts in `text` (Prolog
    source, e.g. a plan_generated.pl) and return the generated clauses
    as Prolog source."""
    saved = Constant.FLOAT_PRECISION
    Constant.FLOAT_PRECISION = None    # print numbers back exactly as written
    try:
        plan, reactive = None, {}
        for cl in PrologString(text):
            if isinstance(cl, Term) and cl.functor == "plan" and cl.arity == 1:
                if plan is not None:
                    raise BTCompileError("more than one plan/1 fact")
                plan = cl.args[0]
            elif isinstance(cl, Term) and cl.functor == "reactive_children" and cl.arity == 2:
                reactive[str(cl.args[0])] = _plist_items(cl.args[1])
        if plan is None:
            raise BTCompileError("no plan/1 fact to compile")
        clauses = _Compiler(plan, reactive).run()
    finally:
        Constant.FLOAT_PRECISION = saved
    header = [
        "% ---------------------------------------------------------------",
        "% COMPILED TREE (module/translators/bt_compile.py): one predicate",
        "% per control-flow node, entered through bt_root/3. This is what",
        "% basic_action_theory.pl runs; plan/1 and reactive_children/2",
        "% above are kept only as the readable source it was compiled from.",
        "% ---------------------------------------------------------------",
        "",
    ]
    return "\n".join(header) + "\n\n".join(clauses) + "\n"


if __name__ == "__main__":
    import sys
    print(compile_plan_text(open(sys.argv[1]).read()))
