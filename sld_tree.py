"""
sld_tree.py

Records how ProbLog's own engine resolves ONE query, as a tree -- the
SLD derivation exactly as executed during grounding, nothing merged --
and writes it as a collapsible HTML page (plus the raw JSON). Called
from main.py's --visual mode (--sld-query); can also be run standalone:

    python3 sld_tree.py --problem problem3L --query 'any_reason_pattern(completed)'

(standalone assumes main.py already prepared the problem, i.e.
module/theory/problem_data.pl points at it).

WHAT A NODE IS
  call    one goal as called, arguments instantiated at call time
          (a theory predicate, a builtin such as is/2 or =.., an external
          Python predicate, or the query itself). Its children are the
          clauses / facts / choices tried for it.
  clause  one program clause tried for its parent call: "head failed"
          if the call does not unify with the head, otherwise its
          children are the body goals as they were called, in order,
          once per binding produced by the goals before them.
  fact    a fact tried for its parent call (matched / did not unify).
  choice  a probabilistic choice (annotated disjunction head / AD
          alternative): a leaf of the derivation, with its probability.
  not     a negated goal \\+ G (succeeds iff G has no answer).

TABLING -- "nothing merged": ProbLog tables (caches) every goal it has
fully solved. When the same goal is called again, the engine does not
search again: it reuses the stored answers. Such a call is still its
own node here (one node per call, as executed), marked "from table"
with a link to the call where those answers were derived; the subtree
is not copied under it, since the engine never searched it again. A
call to a goal that is still being solved higher up (a recursive cycle)
is marked "in progress" with a link to that ancestor.

Every call shows how many answers it returned (0 = failed branch).
Large terms are interned to keep labels readable: every situation term
do(A,S) becomes S<n> and every list of more than 6 elements L<n>; the
viewer shows the full term on click.

WORK -- which of the search was useful: each node is marked useful
(its answers fed an answer of the query) or dead end (explored, but
nothing it produced was used: failed heads, failed bodies, calls whose
answers were all rejected), and gets a time (see classify_work). The
viewer shows this as a faded tree, a flame view (width = cost of the
subtree, green useful / red dead) and a per-predicate cost table.

The tree is recorded while grounding the query ALONE, from an empty
table, so it shows the query's complete derivation. (In a normal run,
queries grounded earlier fill the table, so later queries would show
more "from table" leaves.)
"""

import argparse
import json
import os
import sys
import time

from problog import engine_stack
from problog.engine import DefaultEngine
from problog.engine_stack import Transformations
from problog.engine_unify import substitute_call_args
from problog.eval_nodes import EvalNode, EvalBuiltIn, EvalNot
from problog.formula import LogicFormula
from problog.logic import Term, Constant
from problog.program import PrologFile, PrologString


KIND_CODES = {"root": 0, "call": 1, "clause": 2, "fact": 3, "choice": 4, "not": 5}
INFIX = {"is", "=", "\\=", "==", "\\==", "<", ">", "=<", ">=", "=:=", "=\\=", "+", "-", "*",
         "/", "//", "**", "^", "=..", "->", ";", ",", "mod"}


class SLDRecorder:
    """Patches ProbLog's StackBasedEngine/EvalNode in-process while a
    trace runs (see install/uninstall), building the tree in flat
    arrays: parent, kind, label, status, answers, detail, link."""

    def __init__(self, max_nodes=1_000_000):
        self.max_nodes = max_nodes
        self.truncated = False
        self.parent, self.kind, self.label = [], [], []
        self.status, self.answers, self.detail, self.link = [], [], [], []
        self.pred = []           # "functor/arity" for call nodes, "" otherwise
        self.t = []              # perf_counter() when the node was created
        self.terms = {}          # interned term -> "S12" / "L3"
        self.term_text = {}      # "S12" -> full text
        self._n_terms = {"S": 0, "L": 0}
        self.solved_at = {}      # (functor, label) -> call node that derived it
        self._pending = None     # call node waiting for its EvalDefine/EvalBuiltIn
        self._override = None    # clause node owning the body being started
        self._builtin_ctx = []   # call nodes of builtins running sub-queries
        self._src_lines = {}
        self._orig = {}
        self.root = self._add(None, "root", "", "")

    # ---- tree -----------------------------------------------------------
    def _add(self, parent, kind, label, status, detail="", link=-1):
        if len(self.parent) >= self.max_nodes:
            self.truncated = True
            return None
        self.parent.append(-1 if parent is None else parent)
        self.kind.append(KIND_CODES[kind])
        self.label.append(label)
        self.status.append(status)
        self.answers.append(0)
        self.detail.append(detail)
        self.link.append(link)
        self.pred.append("")
        self.t.append(time.perf_counter())
        return len(self.parent) - 1

    def _counter(self, nid):
        def count(result):
            if nid is not None and result is not None:
                self.answers[nid] += 1
            return result
        return count

    def _resolve(self, engine, p):
        """Tree node that a new engine record with parent pointer p hangs under."""
        if self._override is not None:
            o, self._override = self._override, None
            return o
        if p is not None and 0 <= p < len(engine.stack):
            rec = engine.stack[p]
            if rec is not None and getattr(rec, "_sld", None) is not None:
                return rec._sld
        return self._builtin_ctx[-1] if self._builtin_ctx else self.root

    # ---- term formatting ---------------------------------------------------
    def fmt(self, t):
        if t is None or (isinstance(t, int) and not isinstance(t, bool)):
            return "_"
        if isinstance(t, Constant):
            return str(t)
        if not isinstance(t, Term):
            return str(t)
        if t.functor == "do" and t.arity == 2:
            return self._intern(t, "S")
        if t.functor == "." and t.arity == 2:
            items, cur = [], t
            while isinstance(cur, Term) and cur.functor == "." and cur.arity == 2:
                items.append(cur.args[0])
                cur = cur.args[1]
            if len(items) > 6:
                return self._intern(t, "L")
            tail = "" if (isinstance(cur, Term) and cur.functor == "[]") else "|" + self.fmt(cur)
            return "[" + ", ".join(self.fmt(a) for a in items) + tail + "]"
        if t.arity == 0:
            return str(t)
        return self.goal(t.functor, t.args)

    def _intern(self, t, prefix):
        try:
            key = self.terms.get(t)
        except TypeError:
            return str(t)
        if key is None:
            self._n_terms[prefix] += 1
            key = f"{prefix}{self._n_terms[prefix]}"
            self.terms[t] = key
            if prefix == "S":   # show the action and the interned predecessor
                self.term_text[key] = f"do({self.fmt(t.args[0])}, {self.fmt(t.args[1])})"
            else:
                items, cur = [], t
                while isinstance(cur, Term) and cur.functor == "." and cur.arity == 2:
                    items.append(self.fmt(cur.args[0]))
                    cur = cur.args[1]
                self.term_text[key] = "[" + ", ".join(items) + "]"
        return key

    def goal(self, functor, args):
        if len(args) == 0:
            return str(functor)
        op = str(functor).strip("'")
        if len(args) == 2 and op in INFIX:
            a, b = (self.fmt(x) for x in args)
            return f"({a} {op} {b})" if op in ("+", "-", "*", "/") else f"{a} {op} {b}"
        if len(args) == 1 and op == "-":
            return f"-{self.fmt(args[0])}"
        return f"{functor}({', '.join(self.fmt(a) for a in args)})"

    def source(self, database, location):
        """'file:line' and the clause's source text (up to its final '.')."""
        try:
            fn, line, _ = database.lineno(location, force_filename=True)
        except Exception:
            return "", ""
        if fn not in self._src_lines:
            try:
                with open(fn) as f:
                    self._src_lines[fn] = f.read().split("\n")
            except OSError:
                self._src_lines[fn] = []
        lines, text = self._src_lines[fn], []
        for ln in lines[line - 1: line + 24]:
            text.append(ln)
            if ln.split("%")[0].rstrip().endswith("."):
                break
        return f"{os.path.basename(fn)}:{line}", "\n".join(text)

    # ---- engine hooks --------------------------------------------------------
    def install(self):
        rec = self
        E = engine_stack.StackBasedEngine
        self._orig = {
            "eval_call": E.eval_call, "eval_define": E.eval_define,
            "eval_clause": E.eval_clause, "eval_fact": E.eval_fact,
            "eval_choice": E.eval_choice, "node_init": EvalNode.__init__,
            "builtin_call": EvalBuiltIn.__call__,
        }
        o = self._orig

        def eval_call(eng, node_id, node, context, parent, transform=None, identifier=None, **kw):
            tparent = rec._resolve(eng, parent)
            call_args, _ = substitute_call_args(node.args, context, eng.context_min_var(context))
            if node.defnode is not None and node.defnode < 0:
                kind = "builtin"
            else:
                dnode = kw["database"].get_node(node.defnode)
                kind = "python" if type(dnode).__name__ == "extern" else "predicate"
            nid = rec._add(tparent, "call", rec.goal(node.functor, call_args), kind)
            if nid is not None:
                rec.pred[nid] = f"{str(node.functor).strip(chr(39))}/{len(call_args)}"
            if transform is None:
                transform = Transformations()
            transform.addFunction(rec._counter(nid))
            rec._pending = nid
            try:
                return o["eval_call"](eng, node_id, node, context, parent, transform=transform,
                                      identifier=identifier, **kw)
            finally:
                rec._pending = None

        def eval_define(eng, node, context, target, parent, *a, **kw):
            nid = rec._pending
            if nid is None:   # not reached through eval_call: the query itself, or a sub-query
                nid = rec._add(rec._resolve(eng, parent), "call", rec.goal(node.functor, context),
                               "predicate")
                if nid is not None:
                    rec.pred[nid] = f"{node.functor}/{len(context)}"
                transform = kw.get("transform") or Transformations()
                transform.addFunction(rec._counter(nid))
                kw["transform"] = transform
            rec._pending = nid
            key = (node.functor, rec.label[nid]) if nid is not None else None
            no_cache = kw.get("no_cache", False)
            cached = None if no_cache else target._cache.get((node.functor, context))
            active = None if (no_cache or cached is not None) else target._cache.getEvalNode((node.functor, context))
            try:
                actions = o["eval_define"](eng, node, context, target, parent, *a, **kw)
            finally:
                rec._pending = None
            if nid is not None:
                if cached is not None:
                    rec.status[nid] = "table"
                    rec.link[nid] = rec.solved_at.get(key, -1)
                elif active is not None:
                    rec.status[nid] = "in progress"
                    rec.link[nid] = getattr(active, "_sld", -1) or -1
                elif not node.children.find(context):
                    rec.status[nid] = "no clause"
                else:
                    rec.solved_at.setdefault(key, nid)
            return actions

        def eval_clause(eng, context, node, node_id, parent, transform=None, identifier=None,
                        current_clause=None, **kw):
            where, text = rec.source(kw["database"], node.location)
            first = " ".join(text.split("\n", 1)[0].split()) if text else f"clause {node_id}"
            nid = rec._add(rec._resolve(eng, parent), "clause",
                           f"{first[:110]}   [{where}]" if where else first, "", detail=text)
            if transform is None:
                transform = Transformations()
            transform.addFunction(rec._counter(nid))
            n_before = len(rec.parent)
            rec._override = nid
            try:
                actions = o["eval_clause"](eng, context, node, node_id, parent, transform=transform,
                                           identifier=identifier, current_clause=current_clause, **kw)
            finally:
                rec._override = None
            # head unification failure: the engine completes at once, no body started
            if nid is not None and len(rec.parent) == n_before and actions \
                    and all(a[0] == "c" for a in actions):
                rec.status[nid] = "head failed"
            return actions

        def eval_fact(eng, parent, node_id, node, context, target, identifier, **kw):
            nid = rec._add(rec._resolve(eng, parent), "fact", rec.goal(node.functor, node.args),
                           "", detail="" if node.probability is None else f"p = {node.probability}")
            actions = o["eval_fact"](eng, parent, node_id, node, context, target, identifier, **kw)
            if nid is not None:
                matched = any(a[0] == "r" for a in actions)
                rec.status[nid] = "matched" if matched else "did not unify"
                rec.answers[nid] = int(matched)
            return actions

        def eval_choice(eng, parent, node_id, node, context, target, database, identifier, **kw):
            name = node.functor if isinstance(node.functor, Term) else Term(node.functor)
            nid = rec._add(rec._resolve(eng, parent), "choice", rec.fmt(name), "choice",
                           detail=f"p = {node.probability}")
            if nid is not None:
                rec.answers[nid] = 1
            return o["eval_choice"](eng, parent, node_id, node, context, target, database,
                                    identifier, **kw)

        def node_init(self_node, **kw):
            o["node_init"](self_node, **kw)
            if isinstance(self_node, (EvalBuiltIn,)) or type(self_node).__name__ in ("EvalDefine",):
                if rec._pending is not None:
                    self_node._sld = rec._pending
                    rec._pending = None
                    return
            tparent = rec._resolve(self_node.engine, kw.get("parent"))
            if isinstance(self_node, EvalNot):
                self_node._sld = rec._add(tparent, "not", "\\+", "")
            else:
                self_node._sld = tparent

        def builtin_call(self_node):
            rec._builtin_ctx.append(getattr(self_node, "_sld", None) or rec.root)
            try:
                return o["builtin_call"](self_node)
            finally:
                rec._builtin_ctx.pop()

        E.eval_call, E.eval_define, E.eval_clause = eval_call, eval_define, eval_clause
        E.eval_fact, E.eval_choice = eval_fact, eval_choice
        EvalNode.__init__ = node_init
        EvalBuiltIn.__call__ = builtin_call

    def uninstall(self):
        E = engine_stack.StackBasedEngine
        o = self._orig
        E.eval_call, E.eval_define, E.eval_clause = o["eval_call"], o["eval_define"], o["eval_clause"]
        E.eval_fact, E.eval_choice = o["eval_fact"], o["eval_choice"]
        EvalNode.__init__ = o["node_init"]
        EvalBuiltIn.__call__ = o["builtin_call"]

    def finish(self):
        """Statuses that are only known once the whole query is grounded."""
        for i, k in enumerate(self.kind):
            if k == KIND_CODES["call"]:
                if self.status[i] in ("predicate", "builtin", "python"):
                    self.status[i] = (f"{self.status[i]}: " +
                                      (f"{self.answers[i]} answer(s)" if self.answers[i] else "failed"))
                elif self.status[i] == "table":
                    self.status[i] = f"from table: {self.answers[i]} answer(s)"
                elif self.status[i] == "in progress":
                    self.status[i] = f"in progress (cycle): {self.answers[i]} answer(s)"
            elif k == KIND_CODES["clause"] and self.status[i] == "":
                self.status[i] = f"{self.answers[i]} answer(s)" if self.answers[i] else "body failed"
            elif k == KIND_CODES["not"]:
                kids = [j for j in range(i + 1, len(self.parent)) if self.parent[j] == i][:1]
                inner = self.answers[kids[0]] if kids else 0
                self.status[i] = "succeeded (goal failed)" if not inner else "failed (goal succeeded)"
                self.answers[i] = 0 if inner else 1


def classify_work(rec, t_end):
    """Per node: was it useful, and what did it cost.

    useful   1 if the node's answers fed an answer of the query: the root
             (if it has answers), every node with >= 1 answer whose parent
             is useful, the node where a useful "from table" / "in
             progress" call's answers were derived, and the whole subtree
             of a useful negation (proving \\+ G needs G's full search).
             0 = dead end: explored, but nothing it produced was used.
             Granularity: a useful call counts as useful as a whole even
             if later goals rejected some of its answers.
    self_us  time from this node's creation to the next node's creation,
             in microseconds: the engine work (and Python predicate time)
             spent right after this step, while tracing (the tracer's own
             overhead inflates it roughly uniformly)."""
    n = len(rec.parent)
    kids = [[] for _ in range(n)]
    for i in range(1, n):
        kids[rec.parent[i]].append(i)
    useful = [0] * n
    not_code, ans, link = KIND_CODES["not"], rec.answers, rec.link
    stack = [(rec.root, False)] if ans[rec.root] > 0 else []
    while stack:
        i, whole = stack.pop()
        if useful[i]:
            continue
        useful[i] = 1
        whole = whole or rec.kind[i] == not_code
        stack.extend((c, whole) for c in kids[i] if whole or ans[c] > 0)
        if link[i] >= 0 and ans[i] > 0:
            stack.append((link[i], False))
    times = rec.t + [t_end]
    self_us = [int((times[i + 1] - times[i]) * 1e6) for i in range(n)]
    return useful, self_us


def trace_query(theory_path, query_text, max_nodes=1_000_000):
    """Ground `query_text` alone against theory_path and return the
    recorded tree as a JSON-ready dict."""
    query = next(iter(PrologString(f"sld_query_term({query_text})."))).args[0]
    engine = DefaultEngine(label_all=True)
    db = engine.prepare(PrologFile(theory_path))   # loads the theory (and its Python modules) untraced
    rec = SLDRecorder(max_nodes)
    rec.label[rec.root] = f"?- {query_text}"
    rec.install()
    # the engine binds its eval_* methods into node_types when it is
    # created; rebind them so the patched class methods are used
    saved = dict(engine.node_types)
    engine.node_types = {k: getattr(engine, v.__name__) for k, v in saved.items()}
    t0 = time.perf_counter()
    target = LogicFormula(label_all=True)
    try:
        engine.ground(db, query, target=target, label=LogicFormula.LABEL_QUERY)
    finally:
        rec.uninstall()
        engine.node_types = saved
    t_end = time.perf_counter()
    elapsed = t_end - t0
    rec.finish()
    try:   # the query's probability, from the formula ground while tracing
        from problog import get_evaluatable
        probability = {str(k): float(v) for k, v in
                       get_evaluatable().create_from(target).evaluate().items()}
    except Exception as e:  # noqa: BLE001 -- the tree is still useful without it
        probability = {"error": str(e)}
    root_kids = [i for i, p in enumerate(rec.parent) if p == rec.root]
    rec.answers[rec.root] = sum(rec.answers[i] for i in root_kids)
    rec.status[rec.root] = "query"
    rec.t[rec.root] = t0
    useful, self_us = classify_work(rec, t_end)
    strings, index = [], {}

    def ref(values):
        out = []
        for v in values:
            k = index.get(v)
            if k is None:
                k = index[v] = len(strings)
                strings.append(v)
            out.append(k)
        return out

    return {
        "query": query_text, "theory": theory_path, "ground_seconds": elapsed,
        "probability": probability,
        "truncated": rec.truncated, "max_nodes": max_nodes,
        "kinds": list(KIND_CODES),
        # label/status/detail are indices into "strings" (each distinct text stored once)
        "parent": rec.parent, "kind": rec.kind, "label": ref(rec.label), "status": ref(rec.status),
        "answers": rec.answers, "detail": ref(rec.detail), "link": rec.link,
        "pred": ref(rec.pred), "useful": useful, "self_us": self_us,
        "strings": strings, "terms": rec.term_text,
    }


def tree_stats(data):
    kinds = data["kinds"]
    by_kind = {k: 0 for k in kinds}
    table = failed = 0
    for i, k in enumerate(data["kind"]):
        by_kind[kinds[k]] += 1
        st = data["strings"][data["status"][i]]
        table += st.startswith("from table")
        failed += st.endswith("failed") or st in ("head failed", "did not unify", "no clause")
    useful = sum(data["useful"])
    t_all = sum(data["self_us"]) / 1e6
    t_useful = sum(t for t, u in zip(data["self_us"], data["useful"]) if u) / 1e6
    return {"nodes": len(data["kind"]), "by_kind": by_kind, "from_table": table, "failed": failed,
            "useful_nodes": useful, "dead_nodes": len(data["kind"]) - useful,
            "useful_seconds": round(t_useful, 3), "dead_seconds": round(t_all - t_useful, 3)}


def write_sld_html(data, html_path, title):
    payload = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(_HTML.replace("__TITLE__", title).replace("__DATA__", payload))


_HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
:root { --bg:#fbfbfa; --fg:#1f2328; --muted:#6b7280; --line:#d8dbe0; --panel:#ffffff;
  --call:#1f5fbf; --clause:#6b7280; --fact:#2e7d32; --choice:#8e44ad; --not:#b26a00;
  --fail:#c62828; --table:#00838f; --hl:#fff3b0;
  --useful:#3f9b5a; --useful2:#76c18c; --dead:#d9806f; --dead2:#ecc0b6; --tablefl:#4fb3bf; --hh:120px; }
@media (prefers-color-scheme: dark) { :root { --bg:#16181c; --fg:#e6e6e6; --muted:#9aa0a6;
  --line:#2f333a; --panel:#1d2026; --call:#7fb0ff; --clause:#a0a6ae; --fact:#7ccf83;
  --choice:#c99ae0; --not:#f0b35a; --fail:#ff7b7b; --table:#5fd0db; --hl:#5a4d12;
  --useful:#2f7d47; --useful2:#3f9b5a; --dead:#9c4a3d; --dead2:#6e3a32; --tablefl:#2a7f89; } }
* { box-sizing:border-box; }
body { margin:0; background:var(--bg); color:var(--fg); font:13px/1.45 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace; }
header { position:sticky; top:0; background:var(--panel); border-bottom:1px solid var(--line); padding:10px 16px; z-index:2; }
h1 { font-size:15px; margin:0 0 6px; font-family:system-ui,sans-serif; }
.bar { display:flex; flex-wrap:wrap; gap:8px; align-items:center; }
.bar input[type=text] { flex:1 1 220px; min-width:0; padding:4px 8px; background:var(--bg); color:var(--fg); border:1px solid var(--line); border-radius:4px; font:inherit; }
button, select { font:inherit; padding:3px 9px; background:var(--bg); color:var(--fg); border:1px solid var(--line); border-radius:4px; cursor:pointer; }
.tabs { display:flex; gap:4px; margin-bottom:8px; }
.tabs button.on { background:var(--fg); color:var(--bg); }
.stats { color:var(--muted); margin-top:6px; font-family:system-ui,sans-serif; font-size:12px; }
main { display:grid; grid-template-columns:minmax(0,1fr) minmax(0,380px); gap:0; }
@media (max-width: 800px) { main { grid-template-columns:minmax(0,1fr); } #side { position:static !important; height:auto !important; border-left:0; border-top:1px solid var(--line); } }
.view { padding:8px 16px 40px; overflow-x:auto; min-width:0; }
#side { position:sticky; top:var(--hh); height:calc(100vh - var(--hh)); overflow:auto; border-left:1px solid var(--line); padding:12px 16px; background:var(--panel); }
ul { list-style:none; margin:0; padding-left:18px; }
#tree > ul { padding-left:0; }
li > .row { white-space:nowrap; cursor:pointer; padding:1px 2px; border-radius:3px; }
li > .row:hover { background:var(--line); }
.row.sel { background:var(--hl); }
.row.dead { opacity:.5; }
.tw { display:inline-block; width:14px; color:var(--muted); }
.k { font-size:10px; padding:0 4px; border-radius:3px; border:1px solid currentColor; margin-right:5px; }
.k1 { color:var(--call); } .k2 { color:var(--clause); } .k3 { color:var(--fact); } .k4 { color:var(--choice); } .k5 { color:var(--not); } .k0 { color:var(--fg); }
.st { color:var(--muted); margin-left:8px; }
.st.fail { color:var(--fail); } .st.table { color:var(--table); }
.cost { color:var(--muted); margin-left:8px; font-size:11px; }
.hidden { display:none !important; }
#side h2 { font-size:13px; margin:0 0 8px; font-family:system-ui,sans-serif; }
#side pre { white-space:pre-wrap; word-break:break-word; background:var(--bg); border:1px solid var(--line); padding:8px; border-radius:4px; }
.term { color:var(--table); text-decoration:underline; cursor:pointer; }
.legend span { margin-right:10px; }
.sw { display:inline-block; width:10px; height:10px; border-radius:2px; vertical-align:middle; margin-right:4px; }
#crumbs { font-family:system-ui,sans-serif; font-size:12px; margin:4px 0 8px; color:var(--muted); word-break:break-word; }
#crumbs a { color:var(--call); cursor:pointer; text-decoration:underline; }
#flamewrap { position:relative; }
#flame { display:block; width:100%; cursor:pointer; }
#tip { position:fixed; pointer-events:none; background:var(--panel); border:1px solid var(--line); padding:6px 8px; border-radius:4px; max-width:520px; font-size:12px; z-index:5; box-shadow:0 2px 8px rgba(0,0,0,.15); }
table { border-collapse:collapse; font-size:12px; width:100%; }
th, td { padding:3px 8px; border-bottom:1px solid var(--line); text-align:right; white-space:nowrap; }
th:first-child, td:first-child { text-align:left; }
th { cursor:pointer; position:sticky; top:0; background:var(--panel); }
tbody tr { cursor:pointer; } tbody tr:hover { background:var(--line); }
.bar2 { display:inline-block; height:8px; background:var(--useful); vertical-align:middle; }
.bar2.d { background:var(--dead); }
.note { color:var(--muted); font-family:system-ui,sans-serif; font-size:12px; max-width:900px; }
</style></head><body>
<header>
  <h1>__TITLE__</h1>
  <div class="tabs"><button data-v="tree" class="on">Tree</button><button data-v="flame">Flame (where the work went)</button><button data-v="preds">By predicate</button></div>
  <div class="bar">
    <input id="q" type="text" placeholder="search goals / clauses (e.g. halted_with, is(, obs7)">
    <button id="find">Find</button><button id="next">Next</button>
    <label><input id="hideb" type="checkbox"> hide builtins</label>
    <label><input id="hideh" type="checkbox"> hide clauses whose head failed</label>
    <label><input id="hided" type="checkbox"> hide dead ends</label>
    <button id="expand">Expand 3 levels</button><button id="collapse">Collapse all</button>
  </div>
  <div class="stats" id="stats"></div>
  <div class="stats" id="work"></div>
</header>
<main>
<div>
  <div id="tree" class="view"></div>
  <div id="flameview" class="view hidden">
    <div class="bar"><label>width = <select id="metric"><option value="t">time</option><option value="n">steps</option></select></label>
      <button id="zout">Zoom out</button><button id="zroot">Reset zoom</button>
      <span class="note">Click a box to zoom into it (and see it in the details panel). Width is the cost of the node's whole subtree; a box is laid out under its parent in execution order.</span></div>
    <div id="crumbs"></div>
    <div id="flamewrap"><canvas id="flame"></canvas></div>
  </div>
  <div id="predview" class="view hidden">
    <p class="note">Every step is charged to the predicate of the closest call above it (a call, the clauses tried for it, the facts and choices it matched). Body goals are charged to their own predicate. Click a row to step through that predicate's calls in the tree.</p>
    <table id="ptab"><thead></thead><tbody></tbody></table>
  </div>
</div>
<aside id="side"><h2>Details</h2><div id="det">Click a node.</div>
<h2 style="margin-top:14px">Legend</h2><div class="legend">
<span class="k k1">call</span><span class="k k2">clause</span><span class="k k3">fact</span><span class="k k4">choice</span><span class="k k5">not</span>
<p><span class="sw" style="background:var(--useful)"></span>useful: its answers fed an answer of the query &nbsp;
<span class="sw" style="background:var(--dead)"></span>dead end: explored, nothing it produced was used (faded in the tree) &nbsp;
<span class="sw" style="background:var(--tablefl)"></span>answered from the table</p>
<p>Every node is one step as ProbLog executed it; nothing is merged. A call marked <span style="color:var(--table)">from table</span> reused answers derived at the linked node (the engine did not search again). S&lt;n&gt; / L&lt;n&gt; are interned situation terms / long lists: click them.</p>
<p>Time of a step = time from its creation to the next step's creation, while tracing (includes Python predicate time and answer bookkeeping; the tracer's overhead inflates all steps roughly alike). A useful call counts as useful as a whole even if later goals rejected some of its answers.</p></div></aside></main>
<div id="tip" class="hidden"></div>
<script>
const D = __DATA__;
for (const f of ["label", "status", "detail", "pred"]) D[f] = D[f].map(k => D.strings[k]);
const N = D.parent.length, KN = D.kinds;
const kids = Array.from({length:N}, () => []);
for (let i = 0; i < N; i++) if (D.parent[i] >= 0) kids[D.parent[i]].push(i);
// subtree cost (children always have larger ids than their parent)
const subN = new Float64Array(N).fill(1), subT = Float64Array.from(D.self_us);
const deadN = new Float64Array(N), deadT = new Float64Array(N);
for (let i = 0; i < N; i++) if (!D.useful[i]) { deadN[i] = 1; deadT[i] = D.self_us[i]; }
for (let i = N - 1; i > 0; i--) { const p = D.parent[i]; subN[p] += subN[i]; subT[p] += subT[i]; deadN[p] += deadN[i]; deadT[p] += deadT[i]; }
const owner = new Int32Array(N);
for (let i = 0; i < N; i++) owner[i] = (D.kind[i] === 1 || i === 0) ? i : owner[D.parent[i]];
const fmtT = us => us >= 1e6 ? (us/1e6).toFixed(2) + " s" : us >= 1e3 ? (us/1e3).toFixed(1) + " ms" : Math.round(us) + " µs";
const fmtN = n => n >= 1e6 ? (n/1e6).toFixed(2) + "M" : n >= 1e4 ? (n/1e3).toFixed(1) + "k" : String(n);
const costText = i => `${fmtN(subN[i])} step${subN[i]>1?"s":""} · ${fmtT(subT[i])}` + (deadN[i] && D.useful[i] ? ` · ${Math.round(100*deadT[i]/Math.max(subT[i],1))}% dead` : "");

const isBuiltin = i => D.kind[i] === 1 && D.status[i].startsWith("builtin");
const isHeadFail = i => D.kind[i] === 2 && D.status[i] === "head failed";
let hideB = false, hideH = false, hideD = false, sel = -1;
const hidden = i => (hideB && isBuiltin(i)) || (hideH && isHeadFail(i)) || (hideD && !D.useful[i]);
const el = {}; // node id -> li
function esc(s){return s.replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));}
function linkTerms(s){return esc(s).replace(/\b([SL]\d+)\b/g,(m)=> D.terms[m]!==undefined?`<span class="term" data-t="${m}">${m}</span>`:m);}
function stClass(s){ if (s.startsWith("from table")) return "st table";
  return (/failed$|head failed|did not unify|no clause/.test(s)) ? "st fail" : "st"; }
function makeLi(i){
  const li = document.createElement("li"); el[i] = li;
  const row = document.createElement("div"); row.className = "row" + (D.useful[i] ? "" : " dead"); row.dataset.i = i;
  const has = kids[i].length > 0;
  row.innerHTML = `<span class="tw">${has?"▸":"·"}</span><span class="k k${D.kind[i]}">${KN[D.kind[i]]}</span>${linkTerms(D.label[i])}<span class="${stClass(D.status[i])}">${esc(D.status[i])}</span>` +
    (has ? `<span class="cost">${costText(i)}</span>` : "");
  li.appendChild(row);
  if (hidden(i)) li.classList.add("hidden");
  return li;
}
function expand(i){
  const li = el[i]; if (!li || li.dataset.open === "1" || !kids[i].length) return;
  let ul = li.querySelector(":scope > ul");
  if (!ul) { ul = document.createElement("ul"); const frag = document.createDocumentFragment();
    for (const c of kids[i]) frag.appendChild(makeLi(c)); ul.appendChild(frag); li.appendChild(ul); }
  ul.classList.remove("hidden"); li.dataset.open = "1"; li.querySelector(":scope > .row .tw").textContent = "▾";
}
function collapse(i){ const li = el[i]; if (!li) return; const ul = li.querySelector(":scope > ul");
  if (ul) ul.classList.add("hidden"); li.dataset.open = "0"; if (kids[i].length) li.querySelector(":scope > .row .tw").textContent = "▸"; }
function reveal(i){ show("tree"); const path = []; for (let p = D.parent[i]; p >= 0; p = D.parent[p]) path.push(p);
  for (const p of path.reverse()) { if (!el[p]) continue; expand(p); }
  if (el[i]) el[i].classList.remove("hidden");
  select(i); el[i].scrollIntoView({block:"center"}); }
function select(i){
  if (sel >= 0 && el[sel]) el[sel].querySelector(":scope > .row").classList.remove("sel");
  sel = i; if (el[i]) el[i].querySelector(":scope > .row").classList.add("sel");
  let depth = 0; for (let p = D.parent[i]; p >= 0; p = D.parent[p]) depth++;
  let h = `<div><b>${KN[D.kind[i]]}</b> #${i} · depth ${depth} · ${kids[i].length} child node(s) · <b style="color:${D.useful[i]?"var(--useful)":"var(--fail)"}">${D.useful[i]?"useful":"dead end"}</b></div><pre>${linkTerms(D.label[i])}</pre><div>${esc(D.status[i])}</div>`;
  h += `<div class="cost" style="margin:4px 0">this step ${fmtT(D.self_us[i])}; subtree ${fmtN(subN[i])} steps, ${fmtT(subT[i])}, of which dead ends ${fmtN(deadN[i])} steps, ${fmtT(deadT[i])}</div>`;
  if (D.detail[i]) h += `<pre>${linkTerms(D.detail[i])}</pre>`;
  if (D.link[i] >= 0) h += `<div><button data-go="${D.link[i]}">Go to node #${D.link[i]} (where it was derived)</button></div>`;
  h += `<div style="margin-top:6px">` + (D.parent[i] >= 0 ? `<button data-go="${D.parent[i]}">Go to parent</button> ` : "") +
       `<button data-go="${i}">Show in tree</button> <button data-zoom="${i}">Show in flame</button></div>`;
  document.getElementById("det").innerHTML = h;
}
document.getElementById("tree").addEventListener("click", e => {
  const t = e.target.closest(".term"); if (t) { showTerm(t.dataset.t); return; }
  const row = e.target.closest(".row"); if (!row) return; const i = +row.dataset.i;
  if (el[i].dataset.open === "1") collapse(i); else expand(i); select(i); });
document.getElementById("side").addEventListener("click", e => {
  const b = e.target.closest("button[data-go]"); if (b) reveal(+b.dataset.go);
  const z = e.target.closest("button[data-zoom]"); if (z) { show("flame"); zoomTo(+z.dataset.zoom); }
  const t = e.target.closest(".term"); if (t) showTerm(t.dataset.t); });
function showTerm(k){ document.getElementById("det").innerHTML = `<div><b>${k}</b></div><pre>${linkTerms(D.terms[k])}</pre>` +
  (sel >= 0 ? `<button data-go="${sel}">Back to node #${sel}</button>` : ""); }
let hits = [], hitPos = -1, hitsWhat = "match";
function find(){ const q = document.getElementById("q").value.trim().toLowerCase(); hits = []; hitPos = -1; hitsWhat = "match";
  if (!q) return; for (let i = 0; i < N; i++) if (D.label[i].toLowerCase().includes(q) || D.detail[i].toLowerCase().includes(q)) hits.push(i);
  next(); }
function next(){ if (!hits.length) { document.getElementById("stats").textContent = baseStats + " · no match"; return; }
  hitPos = (hitPos + 1) % hits.length; reveal(hits[hitPos]);
  document.getElementById("stats").textContent = baseStats + ` · ${hitsWhat} ${hitPos+1}/${hits.length}`; }
document.getElementById("find").onclick = find; document.getElementById("next").onclick = next;
document.getElementById("q").addEventListener("keydown", e => { if (e.key === "Enter") (hits.length ? next() : find()); });
document.getElementById("q").addEventListener("input", () => { hits = []; });
function refilter(){ for (const k in el) el[+k].classList.toggle("hidden", hidden(+k)); }
document.getElementById("hideb").onchange = e => { hideB = e.target.checked; refilter(); };
document.getElementById("hideh").onchange = e => { hideH = e.target.checked; refilter(); };
document.getElementById("hided").onchange = e => { hideD = e.target.checked; refilter(); };
document.getElementById("expand").onclick = () => { show("tree"); const go = (i, d) => { if (d === 0) return; expand(i); for (const c of kids[i]) go(c, d-1); }; go(0, 4); };
document.getElementById("collapse").onclick = () => { show("tree"); for (const k in el) collapse(+k); expand(0); };
const cnt = {}; for (let i = 0; i < N; i++) cnt[KN[D.kind[i]]] = (cnt[KN[D.kind[i]]]||0) + 1;
const baseStats = `${N} nodes (` + Object.entries(cnt).map(([k,v]) => `${v} ${k}`).join(", ") +
  `) · grounding this query alone: ${D.ground_seconds.toFixed(2)} s · ` +
  Object.entries(D.probability).map(([k,v]) => `P(${k}) = ${typeof v === "number" ? v.toPrecision(6) : v}`).join(", ") +
  (D.truncated ? ` · TRUNCATED at ${D.max_nodes} nodes` : "");
document.getElementById("stats").textContent = baseStats;
document.getElementById("work").innerHTML =
  `work: <span class="sw" style="background:var(--useful)"></span>useful ${fmtN(N - deadN[0])} steps, ${fmtT(subT[0] - deadT[0])} · ` +
  `<span class="sw" style="background:var(--dead)"></span>dead ends ${fmtN(deadN[0])} steps (${Math.round(100*deadN[0]/N)}%), ${fmtT(deadT[0])} (${Math.round(100*deadT[0]/Math.max(subT[0],1))}% of the time)`;

// ---- views -------------------------------------------------------------
let view = "tree";
function show(v){ view = v;
  document.getElementById("tree").classList.toggle("hidden", v !== "tree");
  document.getElementById("flameview").classList.toggle("hidden", v !== "flame");
  document.getElementById("predview").classList.toggle("hidden", v !== "preds");
  for (const b of document.querySelectorAll(".tabs button")) b.classList.toggle("on", b.dataset.v === v);
  if (v === "flame") drawFlame(); if (v === "preds") drawPreds(); }
for (const b of document.querySelectorAll(".tabs button")) b.onclick = () => show(b.dataset.v);
const setHH = () => document.documentElement.style.setProperty("--hh", document.querySelector("header").offsetHeight + "px");
window.addEventListener("resize", () => { setHH(); if (view === "flame") drawFlame(); }); setHH();

// ---- flame (icicle) ------------------------------------------------------
const cv = document.getElementById("flame"), ctx2 = cv.getContext("2d"), tip = document.getElementById("tip");
const ROW = 17; let zroot = 0, rects = [];
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
function zoomTo(i){ zroot = i; select(i); drawFlame(); }
document.getElementById("zout").onclick = () => { if (D.parent[zroot] >= 0) zoomTo(D.parent[zroot]); };
document.getElementById("zroot").onclick = () => zoomTo(0);
document.getElementById("metric").onchange = drawFlame;
function drawFlame(){
  const metric = document.getElementById("metric").value === "t" ? subT : subN;
  const W = cv.parentElement.clientWidth; rects = [];
  const st = [[zroot, 0, W, 0]];
  while (st.length) { const [i, x, w, d] = st.pop(); rects.push([i, x, w, d]);
    const tot = metric[i] || 1; let cx = x;
    for (const c of kids[i]) { const cw = w * metric[c] / tot; if (cw >= 0.35 && !hidden(c)) st.push([c, cx, cw, d + 1]); cx += cw; } }
  const depth = rects.reduce((m, r) => Math.max(m, r[3]), 0) + 1, dpr = window.devicePixelRatio || 1;
  cv.width = W * dpr; cv.height = depth * ROW * dpr; cv.style.height = depth * ROW + "px";
  ctx2.setTransform(dpr, 0, 0, dpr, 0, 0); ctx2.clearRect(0, 0, W, depth * ROW);
  const col = { u: css("--useful"), u2: css("--useful2"), d: css("--dead"), d2: css("--dead2"), t: css("--tablefl"), fg: css("--fg"), bg: css("--bg") };
  ctx2.font = "11px ui-monospace,Menlo,Consolas,monospace"; ctx2.textBaseline = "middle";
  for (const [i, x, w, d] of rects) {
    const table = D.status[i].startsWith("from table");
    ctx2.fillStyle = table ? col.t : D.useful[i] ? (D.kind[i] === 1 ? col.u : col.u2) : (D.kind[i] === 1 ? col.d : col.d2);
    ctx2.fillRect(x, d * ROW, Math.max(w - 0.5, 0.35), ROW - 1);
    if (i === sel) { ctx2.strokeStyle = col.fg; ctx2.lineWidth = 1.5; ctx2.strokeRect(x + 0.75, d * ROW + 0.75, Math.max(w - 2, 0.5), ROW - 2.5); }
    if (w > 30) { const maxc = Math.floor((w - 6) / 6.6); let s = D.label[i]; if (s.length > maxc) s = s.slice(0, Math.max(maxc - 1, 1)) + "…";
      ctx2.fillStyle = col.fg; ctx2.fillText(s, x + 3, d * ROW + ROW / 2); }
  }
  const path = []; for (let p = zroot; p >= 0; p = D.parent[p]) path.push(p);
  document.getElementById("crumbs").innerHTML = "zoom: " + path.reverse().map(p =>
    `<a data-z="${p}">${esc(D.label[p].slice(0, 50))}</a>`).join(" › ") + ` — ${costText(zroot)}`;
}
document.getElementById("crumbs").addEventListener("click", e => { const a = e.target.closest("a[data-z]"); if (a) zoomTo(+a.dataset.z); });
function hit(e){ const r = cv.getBoundingClientRect(), x = e.clientX - r.left, d = Math.floor((e.clientY - r.top) / ROW);
  for (const q of rects) if (q[3] === d && x >= q[1] && x < q[1] + q[2]) return q[0]; return -1; }
cv.addEventListener("mousemove", e => { const i = hit(e); if (i < 0) { tip.classList.add("hidden"); return; }
  tip.innerHTML = `<b style="color:${D.useful[i]?"var(--useful)":"var(--fail)"}">${D.useful[i]?"useful":"dead end"}</b> · ${KN[D.kind[i]]} #${i}<br>${esc(D.label[i])}<br><span class="cost" style="margin:0">${esc(D.status[i])} · ${costText(i)}</span>`;
  tip.classList.remove("hidden"); tip.style.left = Math.min(e.clientX + 12, window.innerWidth - 540) + "px"; tip.style.top = (e.clientY + 14) + "px"; });
cv.addEventListener("mouseleave", () => tip.classList.add("hidden"));
cv.addEventListener("click", e => { const i = hit(e); if (i >= 0) zoomTo(i); });

// ---- by predicate ----------------------------------------------------------
let PR = null, psort = "t";
function drawPreds(){
  if (!PR) { const m = new Map();
    for (let i = 0; i < N; i++) { const o = owner[i], name = D.pred[o] || "(query)";
      let r = m.get(name); if (!r) m.set(name, r = { name, calls:0, table:0, n:0, dn:0, t:0, dt:0, ids:[] });
      if (o === i && D.kind[i] === 1) { r.calls++; r.ids.push(i); if (D.status[i].startsWith("from table")) r.table++; }
      r.n++; r.t += D.self_us[i]; if (!D.useful[i]) { r.dn++; r.dt += D.self_us[i]; } }
    PR = [...m.values()]; }
  const cols = [["name","predicate"],["calls","calls"],["table","from table"],["n","steps"],["dn","dead steps"],["t","time"],["dt","dead time"],["bar","share of time"]];
  document.querySelector("#ptab thead").innerHTML = "<tr>" + cols.map(([k, h]) => `<th data-k="${k}">${h}${k===psort?" ▾":""}</th>`).join("") + "</tr>";
  const rows = [...PR].sort((a, b) => psort === "name" ? a.name.localeCompare(b.name) : b[psort] - a[psort]);
  const total = subT[0] || 1;
  document.querySelector("#ptab tbody").innerHTML = rows.map(r => `<tr data-p="${esc(r.name)}"><td>${esc(r.name)}</td><td>${r.calls}</td><td>${r.table}</td><td>${fmtN(r.n)}</td><td>${fmtN(r.dn)}</td><td>${fmtT(r.t)}</td><td>${fmtT(r.dt)}</td>` +
    `<td style="text-align:left"><span class="bar2" style="width:${(160*(r.t-r.dt)/total).toFixed(1)}px"></span><span class="bar2 d" style="width:${(160*r.dt/total).toFixed(1)}px"></span> ${(100*r.t/total).toFixed(1)}%</td></tr>`).join("");
}
document.querySelector("#ptab thead").addEventListener("click", e => { const th = e.target.closest("th"); if (th && th.dataset.k !== "bar") { psort = th.dataset.k; drawPreds(); } });
document.querySelector("#ptab tbody").addEventListener("click", e => { const tr = e.target.closest("tr"); if (!tr) return;
  const r = PR.find(x => x.name === tr.dataset.p); if (!r || !r.ids.length) return;
  hits = r.ids; hitPos = -1; hitsWhat = `call of ${r.name}`; next(); });

const ul = document.createElement("ul"); ul.appendChild(makeLi(0)); document.getElementById("tree").appendChild(ul); expand(0);
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--problem", required=True)
    ap.add_argument("--query", default="verify_goal_formula")
    ap.add_argument("--max-nodes", type=int, default=1_000_000)
    ap.add_argument("--out", default=None, help="HTML path (default: output/<problem>/<problem>_sld_tree.html)")
    args = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    os.environ["BT_PROBLEM_DIR"] = os.path.join(here, "problems", args.problem)
    theory = os.path.join(here, "module", "theory", "basic_action_theory.pl")
    data = trace_query(theory, args.query, args.max_nodes)
    out = args.out or os.path.join(here, "output", args.problem, f"{args.problem}_sld_tree.html")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    write_sld_html(data, out, f"{args.problem} -- SLD tree of {args.query}")
    with open(os.path.splitext(out)[0] + ".json", "w") as f:
        json.dump(data, f, separators=(",", ":"))
    print(json.dumps(tree_stats(data)), f"ground {data['ground_seconds']:.2f}s ->", out)


if __name__ == "__main__":
    sys.exit(main())
