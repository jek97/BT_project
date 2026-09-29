"""
graph_export.py

Dumps the two intermediate ProbLog artifacts pipeline_stages.py's own
Ground and Compile stages already build in memory -- the ground
LogicFormula (lf) and the compiled circuit (compiled, an SDD or DDNNF
instance depending on which knowledge compiler is available in this
environment: pysdd if installed, dsharp otherwise -- see get_evaluatable's
own resolution logic in problog/__init__.py) -- to disk as GraphViz .dot
files PLUS a companion JSON sidecar per graph giving, for every node,
enough information to know what it IS (a declared query, a do_node(...)
call, a cond(...) check, a bare AD fact, a plain AND/OR gate the compiler
introduced, ...), not just its dot-rendered shape/label.

WHY A SEPARATE JSON SIDECAR, NOT JUST THE .dot FILE:  LogicFormula.to_dot()
(inherited by both the ground LogicFormula and the compiled circuit, since
both are LogicDAG subclasses) already writes AND/OR/atom shapes plus
whatever name each node happens to carry, but that name is dropped for
any node the caller doesn't explicitly keep -- and, more importantly,
to_dot()'s query markers only cover formula.queries() (this run's own
declared query(...) terms), not "which functor produced this node" for
every OTHER node too. The ground graph. by contrast, has FULL per-node
provenance available (see GROUND GRAPH NODE NAMING below) that to_dot()
doesn't surface directly -- the JSON sidecar is what actually exposes it.

GROUND GRAPH NODE NAMING: run_staged_inference's own Ground stage already
calls LogicFormula.create_from(model, label_all=True) -- label_all=True is
exactly what makes ProbLog's own grounding engine (engine_stack.py/
eval_nodes.py) attach a name=Term(functor, *result_args) to EVERY
resolved predicate-call node, not just declared queries -- so a node's
own .name here is genuinely "which Prolog call produced this node"
(do_node(seq_node(...),s0,S1,true), cond(battery_below(20.0),c3,S,true),
battery(B,T,S), ...), for literally every conj/disj node in the graph.
Bare atom nodes (leaves) are the resolved AD/probabilistic-fact draws
(z(...), zt(...), zbatt(...), the coin-flip facts annotated disjunctions
expand into) -- see basic_action_theory.pl's own MERGE-GRID QUANTIZATION
section for what those represent physically.

COMPILED GRAPH NODE NAMING -- IMPORTANT ASYMMETRY, confirmed directly
(not assumed) by grounding+compiling a small toy ProbLog program and
inspecting the result: compilation does NOT preserve label_all's per-node
names. Only formula.queries()/evidence() (the REGISTERED name lookup,
populated via add_query/add_evidence, a completely separate mechanism
from a node's own .name attribute) survives the round trip -- and even
those only land on LEAF (atom) nodes of the compiled circuit, because of
how the CNF/DIMACS <-> .nnf round trip works (ddnnf_formula.py's own
_compile_with_dsharp -> _load_nnf: only 'L' literal lines get a name
reattached from cnf.get_names_with_label(); 'A'/'O' gate lines are
compiler-introduced pure structure with no name at all). So on the
COMPILED graph: use compiled.queries() to find each query's own node
(this DOES work, confirmed on a toy program: compiled.queries() returns
the same {term: node_index} shape as lf.queries(), just re-numbered),
but expect internal AND/OR nodes to show up unlabeled -- this is not a
representation gap here, it is what dsharp's/pysdd's own output
genuinely contains (the whole point of compilation is restructuring the
proof DAG into a disjoint, decomposable circuit -- see the chat
discussion this export was written to support -- so "which do_node
produced this AND-gate" is not even a well-formed question after
compilation for most gates; only which LEAF corresponds to which
original AD draw, and which node is each declared query, survive).

Backend-agnostic by construction: SDD and DDNNF are both LogicDAG
subclasses (formula.py), so the same export function works for either
without checking which one is active -- the sidecar records
type(compiled).__name__ so a reader can tell which compiler actually ran
this time without re-deriving it.
"""

import json
import os

from problog.logic import Term


def _node_record(index, node, node_type):
    """One JSON-serializable record for a single LogicFormula/LogicDAG
    node -- shared by both export_ground_graph and export_compiled_graph
    since atom/conj/disj (formula.py's own node namedtuples) have the
    same shape in either graph."""
    record = {
        "index": index,
        "type": node_type,
        "name": None,
        "functor": None,
        "arity": None,
        "args": None,
    }
    name = getattr(node, "name", None)
    if isinstance(name, Term):
        record["name"] = str(name)
        record["functor"] = str(name.functor)
        record["arity"] = name.arity
        record["args"] = [str(a) for a in name.args] if name.args else []
    elif name is not None:
        record["name"] = str(name)
    if node_type == "atom":
        record["probability"] = (str(node.probability)
                                  if node.probability is not None else None)
        record["is_probabilistic"] = node.probability not in (True, None)
    else:
        record["children"] = list(node.children)
    return record


def _write_graph(formula, dot_path, json_path, extra_meta=None):
    """Shared body for export_ground_graph/export_compiled_graph: writes
    formula.to_dot()'s own output verbatim (so the rendered picture is
    exactly what ProbLog itself thinks this graph looks like -- no
    reformatting), plus the node-by-node JSON sidecar this module adds on
    top, plus which nodes are this run's own declared queries (formula.
    queries() -- works the same way on both the ground formula and the
    compiled circuit, confirmed directly; see this module's own header)."""
    with open(dot_path, "w") as f:
        f.write(formula.to_dot())

    query_nodes = {str(term): key for term, key in formula.queries()}

    nodes = []
    for index, node, node_type in formula:
        record = _node_record(index, node, node_type)
        record["is_query"] = index in query_nodes.values()
        nodes.append(record)

    sidecar = {
        "backend_class": type(formula).__name__,
        "node_count": len(formula),
        "queries": query_nodes,
        "nodes": nodes,
    }
    if extra_meta:
        sidecar.update(extra_meta)

    with open(json_path, "w") as f:
        json.dump(sidecar, f, indent=2)


def export_ground_graph(lf, out_dir):
    """lf: the ground LogicFormula from run_staged_inference's own Ground
    stage (built with label_all=True -- see this module's own header for
    why that flag is what makes every node's own .name meaningful, not
    just declared queries). Writes ground.dot + ground_nodes.json into
    out_dir (created if missing)."""
    os.makedirs(out_dir, exist_ok=True)
    _write_graph(lf,
                 os.path.join(out_dir, "ground.dot"),
                 os.path.join(out_dir, "ground_nodes.json"),
                 extra_meta={
                     "label_all": True,
                     "note": ("Every conj/disj node's own name/functor/args "
                               "is the actual grounded Prolog call that "
                               "produced it (do_node/4, cond/3, battery/3, "
                               "...) -- see this module's own GROUND GRAPH "
                               "NODE NAMING note."),
                 })


def export_compiled_graph(compiled, out_dir):
    """compiled: the compiled circuit from run_staged_inference's own
    Compile stage (an SDD or DDNNF instance -- whichever knowledge
    compiler this environment resolved to; see get_evaluatable's own
    logic). Writes compiled.dot + compiled_nodes.json into out_dir
    (created if missing). Internal AND/OR nodes are expected to come back
    unnamed here -- see this module's own COMPILED GRAPH NODE NAMING
    note for why that's the compiler's own output, not a bug in this
    export."""
    os.makedirs(out_dir, exist_ok=True)
    _write_graph(compiled,
                 os.path.join(out_dir, "compiled.dot"),
                 os.path.join(out_dir, "compiled_nodes.json"),
                 extra_meta={
                     "note": ("Only declared queries/evidence survive "
                               "compilation with a name attached (on LEAF "
                               "nodes only) -- internal AND/OR gates here "
                               "are compiler-introduced structure with no "
                               "Prolog-level provenance; see this module's "
                               "own COMPILED GRAPH NODE NAMING note."),
                 })
