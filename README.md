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

## Saving the ground/compiled ProbLog graphs (--save-graphs)

main.py has a --save-graphs flag, OFF by default (it adds a real
amount of overhead -- a full pass over the ground formula, plus
writing potentially large .dot/.json files -- that most runs don't
want):

    python3 main.py --problem problem0S --save-graphs

This dumps, into this run's own output/<problem>/graphs/:
  - ground.dot / ground_nodes.json -- the ground LogicFormula
    ProbLog's own grounding stage builds (SLD-resolution proof search
    over every query, unrolled and regressed back to s0). Because the
    grounding call already uses label_all=True, EVERY node here is
    named after the actual grounded Prolog call that produced it
    (do_node(...), cond(...), battery(...), ...), not just the
    declared queries -- see module/theory/graph_export.py's own header
    for exactly what the JSON sidecar records.
  - compiled.dot / compiled_nodes.json -- the compiled circuit (SDD if
    pysdd is installed, DSharp d-DNNF otherwise -- see
    get_evaluatable()'s own resolution logic) used for the actual
    weighted model counting. This graph is NOT as richly labeled as
    the ground one: only declared queries survive compilation with a
    name, and only on leaf nodes -- internal AND/OR gates the compiler
    introduces come back unnamed. This asymmetry is real (confirmed
    directly, not assumed) and documented in graph_export.py's own
    header.

--save-graphs=false explicitly turns it off (same as the default);
bare --save-graphs or --save-graphs=true/1 turns it on.

## Visualizing the ground graph (visualize_ground_graph.py)

visualize_ground_graph.py loads a saved ground_nodes.json (from
--save-graphs above) and renders it as a spinning 3D point cloud --
small grey balls for nodes, black lines for edges, white background,
rotating around the vertical axis. Driven by visualize_config.yaml
(see that file's own comments for every parameter), it then plays out
two reveal steps on top of the static graph:

  1. After t1_seconds, the goal query node (goal_query in the config;
     defaults to verify_goal_formula) turns red and "shiny".
  2. Starting interval_seconds later, n distinct PROOFS of the goal
     (sub-DAGs from the AD-fact leaves up to the goal node) light up
     one at a time, each its own color, one every interval_seconds.

Usage:

    python3 main.py --problem problem0S --save-graphs
    python3 visualize_ground_graph.py --graphs-dir output/problem0S/graphs

Or just point visualize_config.yaml's own graphs_dir at the right
output/<problem>/graphs/ directory and run
`python3 visualize_ground_graph.py` with no arguments. Set
animation.output_path in the config (or pass --output path.mp4/.gif)
to render to a file instead of opening an interactive window --
required in a headless environment.

Honest caveat: matplotlib has no real specular/lighting model, so
"shiny" here is faked (a brighter/larger marker, a light outline, and
a small white "glint" dot on top) -- it reads as a highlight, but it
is not true 3D shading. See visualize_ground_graph.py's own header if
a real lighting model (e.g. via PyVista) is ever wanted instead --
the graph-loading/layout/proof-enumeration code doesn't need to change
for that, only the rendering section.
