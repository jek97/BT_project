% ============================================================
% ProbLog -- CONTINUOUS-TIME / CONTINUOUS-SPACE moveto()
%
%   - obstacles: POLYGONS (auto-generated from a nav_msgs/
%     OccupancyGrid by occgrid_to_problog.py, see companion file
%     obstacles_generated.pl)
%   - nominal trajectory: a single cubic-Bezier SPLINE, given as
%     a control-point list, evaluated by CLOSED-FORM arithmetic
%     (no discretized time-steps in the action theory itself)
%   - noise: ONE Gaussian lateral-drift draw per walk instance
%     (Brownian-bridge-consistent: zero at the start, growing with
%     elapsed time), resolved as a situation-indexed probabilistic
%     fact -- NOT resampled per query
%   - policy: an EXPLICIT start/end ACTION PAIR
%         startMoveto(ControlPoints, Triggers, T0)  ...  haltMoveto(T,Reason)
%     marking the durative interval, with a BUSY fluent tracking
%     "walk currently in progress" between them. A generic
%     interrupt(T) action can end the walk EARLY (before natural
%     completion), so the walk can be preempted by another action
%     if needed. This is the genuine Reiter-style start/end
%     interval-action pattern (Ch.7), used here specifically
%     because we now want (a) other actions to be able to gate on
%     "is the robot currently walking", and (b) the ability to cut
%     a walk short. Position freezes at whatever point the walk
%     was interrupted -- it does NOT keep evolving toward the
%     original target after the walk has ended.
%   - startMoveto is a genuine TEMPLATE action: Triggers is the
%     COMPLETE list of halting conditions this leg reacts to --
%     collision, battery depletion, obstacle sighting, and any future
%     condition are ALL ordinary entries here, on identical footing.
%     NOTHING is hardcoded: Triggers=[] means the walk halts ONLY on
%     natural completion of its nominal duration, passing straight
%     through an obstacle's margin or running the battery dry without
%     ever noticing, if collision/battery aren't in this leg's own
%     Triggers list. There is NO default Triggers list anywhere in this
%     theory -- every moveto_leg(CP,Triggers) call states its own
%     Triggers explicitly, by design, so a plan's protection level is
%     always visible at the call site rather than inherited from
%     configuration. Different occurrences of moveto in a policy can
%     react to different conditions this way -- see plan/1 near the end
%     of this file for the shipped plan's own explicit choice
%     ([collision,battery]). Whichever entry in Triggers occurs
%     EARLIEST in a given resolved world (or natural completion, if
%     none does) determines Reason -- see the earliest-wins machinery
%     below Poss(haltMoveto(...)).
%   - fluents (at/4, moving/1) are PURE Reiter-style regressions over
%     the situation term S -- given (T,S) and a resolved possible
%     world, they are deterministic functions, never cached, never
%     re-sampled.
%   - safety queries are indexed by "which SAMPLED INSTANT along the
%     walk's ACTUAL executed span" (from T0 to whenever the walk
%     ended, naturally or via interrupt). Sampling is a
%     VERIFICATION-TIME choice (num_samples/1) completely decoupled
%     from the action theory and from the noise model's world count.
%   - FUTURE EXTENSION (not implemented yet, flagged where relevant):
%     time-varying speed. Currently speed/1 is a constant; a varying
%     speed would replace walk_duration's arc-length/speed division
%     with a numeric integral of dt = ds/speed(t) along the spline.
% ============================================================

% ---------------------------------------------------------------
% 0. PROBLEM-SPECIFIC DATA -- obstacles, tunable config, the BT policy,
%    and the goal formula ALL come from problem_data.pl, a small
%    AUTO-GENERATED bootstrap file living next to this one
%    (module/theory/problem_data.pl), rewritten by main.py on every
%    run to point (via absolute paths) at whichever problem was
%    selected (--problem NAME, default problem0; see
%    problems/<NAME>/ for that problem's own config.yaml,
%    behavior_tree.xml, goal_formula.pl, and map.yaml). Consulting ONE
%    fixed file here -- rather than hardcoding four problem-specific
%    paths into THIS file -- is what lets the same theory serve any
%    problem folder without ever being hand-edited itself.
%
%    problem_data.pl provides:
%      - obstacle_polygon/2   from obstacles_generated.pl, itself
%                              generated from map.yaml by
%                              module/translators/occgrid_to_problog.py.
%      - start/1, robot_radius/1, safety_buffer/1, speed/1, sigma/1,
%        sigma_tangential/1, sigma_battery/1, battery_start/1,
%        idle_drain_rate/1, moving_drain_rate/1,
%        tolerance/1, num_samples/1, bracket_samples/1, crossing_eps/1,
%        z/2, zt/2, zbatt/1, disc_step_position/1, disc_step_battery/1,
%        disc_step_time/1 (see the MERGE-GRID QUANTIZATION note above
%        dist/5's own section)
%                              from config_generated.pl, itself
%                              generated from config.yaml by
%                              module/translators/config_to_prolog.py.
%      - plan/1                from plan_generated.pl, itself
%                              translated + validated from
%                              behavior_tree.xml against
%                              module/contracts/schema.yaml by
%                              module/translators/bt_to_prolog.py.
%      - goal_formula/1        the problem's own hand-authored
%                              goal_formula.pl, validated against
%                              module/contracts/vocabulary.yaml by
%                              module/contracts/goal_formula_check.py.
%    main.py regenerates/validates all four automatically before every
%    run -- you never need to run a generator by hand for a normal run.
% ---------------------------------------------------------------
% Fallback clause so obstacle_polygon/2 is always a KNOWN predicate to
% ProbLog even if problem_data.pl's own obstacles_generated.pl defines
% zero real obstacles (its body always fails, so it never contributes
% an actual obstacle).
obstacle_polygon(no_obstacles_placeholder, []) :- fail.

% obstacle_hole/2: the ADDITIVE counterpart to obstacle_polygon/2 --
% zero or more per obstacle Id, each one a HOLLOW interior boundary
% WITHIN that same obstacle (e.g. a perimeter fence's own inner face,
% surrounding the walkable ground inside it) -- see occgrid_to_
% problog.py's own module docstring for how these get extracted
% (cv2.RETR_CCOMP, a two-level contour hierarchy) and module/theory/
% collision_geometry.py's own OBSTACLE_POLYGONS/_inside_polygon note
% for how every consumer (collision_geometry.py AND planners.py)
% combines an obstacle's own outer+hole rings back into ONE
% containment/clearance test, so a hole's own free interior correctly
% reads as free space rather than "inside the obstacle". An ordinary,
% hole-less obstacle (a tree, a building) has ZERO obstacle_hole facts
% and behaves exactly as it always has. Same "always a KNOWN predicate"
% placeholder-clause fix as obstacle_polygon/2 above, for the SAME
% reason (a problem whose own map has no holes at all -- the common
% case -- would otherwise have ZERO obstacle_hole facts anywhere in
% the loaded program).
obstacle_hole(no_obstacle_holes_placeholder, []) :- fail.

:- consult('./problem_data.pl').
% Must exist relative to wherever this file itself lives (module/theory/),
% not CWD -- same resolution rule every consult/use_module directive in
% this file follows.

% planners.py provides plan_astar/5, plan_straight/5, plan_voronoi/5,
% plan_dastar/6, and follow_boarder/5 as BLACK-BOX (Python-implemented)
% predicates --
% see that file's own header for the full explanation. ProbLog imports
% and executes it directly the moment this directive loads
% (problog.clausedb's load_external_module), registering both
% predicates before anything below that calls them is ever evaluated.
% Path is resolved relative to THIS file's own directory (not CWD) --
% planners.py lives right next to this file, in module/theory/.
:- use_module('./planners.py').

% collision_geometry.py provides first_threshold_crossing_time/8 as a
% BLACK-BOX (Python-implemented) predicate -- the obstacle-clearance
% geometry and bracket-scan/bisection crossing-time search that used to
% be plain Prolog in sections 1 and the TRIGGERS section below (see
% their own notes for why this moved). Lives right next to this file
% too, resolved relative to THIS file's own directory, same as
% planners.py.
:- use_module('./collision_geometry.py').

% ---------------------------------------------------------------
% 1. GEOMETRY HELPERS -- general-purpose arithmetic used throughout
%    the theory (distance, arc-length summation). The OBSTACLE-SPECIFIC
%    geometry that used to live here (point/segment/polygon distance,
%    ray-casting point-in-polygon, signed clearance, min-clearance-to-
%    any-obstacle) has been MOVED to collision_geometry.py, a Python
%    black box exactly like planners.py's planners -- see the
%    TRIGGERS section further down (first_threshold_crossing_time/8)
%    for why: that geometry is pure deterministic arithmetic once Z is
%    resolved, with no probabilistic content of its own, so ProbLog was
%    paying full SLD-grounding cost (a materialized proof node per
%    bracket sample, per bisection step, per obstacle vertex) for a
%    computation that has no bearing on the weighted model count beyond
%    its single Tcross answer. Moving it to native Python collapses
%    that whole grounding subtree into one black-box call per resolved
%    world -- see collision_geometry.py's own header for the full
%    rationale and for why its results are IDENTICAL to this file's
%    former Prolog implementation (same bracket-sample count, same
%    bisection epsilon, ported line-for-line, not re-derived).
% ---------------------------------------------------------------
dist(X1,Y1,X2,Y2,D) :- D is sqrt((X2-X1)**2 + (Y2-Y1)**2).

% ---------------------------------------------------------------
% MERGE-GRID QUANTIZATION -- three rounding primitives used ONLY at
% the specific "seam" where a NEW leg (a fresh startMoveto) reads its
% own starting condition from wherever the PREVIOUS leg's actual halt
% left things. See disc_step_position/1, disc_step_battery/1, and
% disc_step_time/1 (config facts, from the problem's own config.yaml
% -- see that file's own "grounding:" section) for what actually
% enables this, and leg_start_battery/3, poss(startMoveto(...)), and
% do_node(planWith(...)) further down for where each is applied.
%
% WHY: this project's own investigation (see FUTUREWORK.md and the
% conversation that produced this feature) found that ProbLog's
% grounder DOES merge multiple proofs of an identical ground atom into
% one shared formula node for free -- but a NEW leg's own control
% points (via planWith), starting battery, and starting time are each
% CONTINUOUS functions of the noise (Z,Zt,Zb) an EARLIER leg resolved,
% so they are essentially NEVER bit-identical across different worlds,
% and merging never actually happens on its own. Rounding these three
% quantities to a config-chosen grid, EXACTLY at the point a new leg
% reads them, makes different worlds that land close enough together
% produce the SAME ground term -- letting ProbLog's existing, exact
% sharing machinery do the compression, with no change to how anything
% is combined/weighted.
%
% WITHIN one leg, this changes NOTHING: Tcross (bracket-scan or
% closed-form battery algebra), position (walk_noisy_point), and
% battery drain (noisy_drain) are all still computed at FULL FLOAT
% PRECISION exactly as before this feature existed. Quantization
% happens EXACTLY ONCE per leg boundary -- on the value a leg reads as
% its OWN starting condition -- never mid-computation, and never on
% anything used for REPORTING (first_hit/on_track/verify_safe/
% halted_with/2 all still read the exact, un-rounded Tcross/position/
% battery of whichever leg actually produced them; only what SEEDS the
% NEXT leg is coarsened). cond() checks (holds(battery_over(...)),
% holds(distance_below(...)), etc.) also stay exact, on purpose -- a
% decision boundary like "is battery actually over 70%" should not be
% shifted by a floor/ceiling/round choice made for an unrelated reason.
%
% All three grids default to 0 in config.yaml (disabled): every
% quantize* predicate below passes its Value straight through
% UNCHANGED at Grid =< 0, so a problem that never sets these stays
% byte-for-byte identical to this feature not existing at all.
%
% Direction matters for two of the three, not the first:
%   quantize/3       -- round-to-NEAREST. Used for position: rounding
%                        either way is an equally valid approximation,
%                        no physical-consistency constraint.
%   quantize_down/3  -- FLOOR. Used for a new leg's own starting
%                        battery: never OVERESTIMATE remaining charge,
%                        the same "approximation must never look safer
%                        than reality" convention noisy_drain/3 above
%                        already follows (it clamps drain at 0 for
%                        exactly this reason).
%   quantize_up/3    -- CEILING. Used for a new leg's own start time:
%                        a leg can never plausibly begin BEFORE the
%                        previous one actually ended, so rounding down
%                        (or to nearest) could produce a non-causal
%                        T0 earlier than the real halt instant.
% ---------------------------------------------------------------
quantize(Value, Grid, Value) :- Grid =< 0.
quantize(Value, Grid, Quantized) :-
    Grid > 0,
    Quantized is round(Value / Grid) * Grid.

quantize_down(Value, Grid, Value) :- Grid =< 0.
quantize_down(Value, Grid, Quantized) :-
    Grid > 0,
    Quantized is floor(Value / Grid) * Grid.

quantize_up(Value, Grid, Value) :- Grid =< 0.
quantize_up(Value, Grid, Quantized) :-
    Grid > 0,
    Quantized is ceiling(Value / Grid) * Grid.

% cell_index(+Value, +CellSize, -Index): the ploughed/3 fluent's own
% discretization primitive (see that predicate's own note, further
% down) -- DELIBERATELY NOT quantize/3 above, even though it looks
% similar. quantize/3 snaps Value to the NEAREST GRID-ALIGNED VALUE,
% still a float in the original coordinate units (round(Value/Grid)*
% Grid) -- fine for merge-grid quantization, where the snapped position
% is used again as a real coordinate. A cell fluent needs the opposite:
% a bare INTEGER cell index, with no residual floating-point
% representation at all, so that two samples landing in the same cell
% (from the same leg, or from two DIFFERENT worlds/legs whose merge-
% grid-quantized starting conditions happen to coincide) produce the
% EXACT SAME Prolog term and merge -- a rounded-but-still-float value
% risks not doing so (5.0 and 5.000000000000001 don't unify), which
% would make ploughed(Cx,Cy,S) effectively never share proofs across
% worlds.
cell_index(Value, CellSize, Index) :- Index is round(Value / CellSize).

sum_list([], 0.0).
sum_list([H|T], Sum) :- sum_list(T, SumT), Sum is H + SumT.

% ---------------------------------------------------------------
% 2. ROBOT / SAFETY PARAMETERS
% ---------------------------------------------------------------
% robot_radius/1 and safety_buffer/1 are now config facts
% (the problem's own config.yaml -> config_generated.pl, consulted above)
% -- see that file for the tunable values themselves. safety_margin/1
% below is a DERIVED value, not a raw constant -- always recomputed
% from the two config facts, never itself config data.
%
% sight_threshold/1 and sight_threshold_valid are GONE: obstacle
% proximity used to be checked against ONE global sight_threshold
% constant (the old obstacle_sighted trigger); it is now
% obstacle_in_bound(Threshold), a genuinely per-call parameter (see the
% TRIGGERS section and holds(obstacle_in_bound(...)) below) -- there is
% no longer one global value to validate against safety_margin. NOT YET
% BUILT: a per-instance check (e.g. in module/translators/bt_to_prolog.py)
% that a given obstacle_in_bound(Threshold)'s Threshold is itself
% sensible (> safety_margin) -- currently unchecked, same as any other
% trigger-list entry's argument.
safety_margin(M) :- robot_radius(R), safety_buffer(B), M is R + B.

% MAP-PREPROCESSING INFLATION: <problem>/obstacles_generated.pl's own
% obstacle_polygon/2 facts are no longer the map's raw occupied cells --
% module/translators/occgrid_to_problog.py's own generate() now
% inflates the occupied mask by safety_margin/1's own value (main.py
% passes robot_radius+safety_buffer straight through, computed from
% THIS SAME problem's own config.yaml) BEFORE extracting polygons -- see
% that file's own module docstring. So `collision` (a robot's own
% CENTER point coming into physical contact with an obstacle, once its
% own radius+buffer are accounted for) is now a bare containment/
% contact test against the ALREADY-INFLATED polygon -- first_collision_
% time/7 below calls the generalized machinery at Threshold=0.0, not
% safety_margin, and needs no distance comparison at all beyond
% "is the point on or inside the (inflated) obstacle". Every OTHER
% distance-based test that still means "how close to the REAL,
% uninflated obstacle surface" -- obstacle_in_bound(Threshold) and
% obstacle_on_path(Threshold), whose own Threshold argument keeps its
% documented meaning unchanged -- must correct for the same inflation
% by subtracting safety_margin back out before calling into
% collision_geometry.py's distance primitives (which only ever see the
% inflated polygons): see clearance_adjusted_threshold/2 immediately
% below, used at every one of their call sites (trigger_crossing_time/11,
% holds/2, and holds_leg/9).
%
% obstacles_generated.pl also carries a SECOND, SEPARATE family,
% obstacle_polygon_planning/2 -- inflated by robot_radius ALONE, no
% safety_buffer -- that collision_geometry.py never reads at all; it
% is planners.py's own plan_voronoi that routes through it (and
% plan_astar's raster grid, independently, at the same robot_radius-
% only amount). See occgrid_to_problog.py's own module docstring, "TWO
% INFLATION LEVELS, ONE FILE", and planners.py's own module docstring
% for the full split.
%
% clearance_adjusted_threshold(+Threshold, -Adjusted): Adjusted is
% Threshold minus safety_margin -- can go negative (a caller asking for
% a bound TIGHTER than the robot's own physical clearance), which
% correctly degrades to "never fires beyond what collision itself
% already would", same not-yet-validated "Threshold > safety_margin"
% caveat this file already documented above, now just enforced
% arithmetically instead of geometrically.
clearance_adjusted_threshold(Threshold, Adjusted) :-
    safety_margin(M), Adjusted is Threshold - M.

% within_obstacle_threshold/3 (the generalized "is (PX,PY) within
% Threshold of the nearest obstacle" test, parametrized by threshold so
% the SAME primitive serves collision, obstacle_in_bound, and any
% future distance-based trigger) now lives inside collision_geometry.py's
% within_obstacle_threshold helper, alongside the rest of the
% obstacle-clearance geometry it was
% moved with -- see the note above dist/5 in section 1.

% ---------------------------------------------------------------
% 3. SPLINE -- chained cubic Bezier segments. ControlPoints =
%    [point(X0,Y0), point(X1,Y1), point(X2,Y2), point(X3,Y3),
%    point(X4,Y4), ...], length must be 3k+1 for k segments (segment i
%    uses control points 3i..3i+3). A straight line is the degenerate
%    case where the interior control points are collinear with the
%    endpoints.
%
%    NO Bezier/spline arithmetic is implemented in Prolog any more --
%    it all lives in exactly ONE place, collision_geometry.py's own
%    _walk_noisy_point (and the walk_noisy_point/8 black-box predicate
%    it backs, registered by the :- use_module('./collision_geometry.py')
%    directive in Section 0, same as first_threshold_crossing_time/8
%    and friends). This file used to carry its OWN, separate
%    reimplementation (bezier_point/tangent, spline_point/tangent,
%    perp_unit, tangent_unit) that had to be kept "identical" to
%    collision_geometry.py's copy by discipline, not by construction --
%    a real duplication/drift risk, now removed: walk_noisy_point/8
%    below IS the foreign predicate (no Prolog clause of that name
%    exists here), and spline_point/4 is a two-line wrapper calling
%    the SAME predicate with zero deviation.
% ---------------------------------------------------------------

% spline_point(+ControlPoints, +U, -X, -Y): U in [0,1] spans the WHOLE
% spline -- the deterministic/nominal point, no noise. Delegates to
% walk_noisy_point/8 with Z=0.0, Zt=0.0 (no deviation) and T0=0.0,
% Duration=1.0, T=U (so Frac=U exactly) -- the SAME underlying
% arithmetic walk_noisy_point/8 itself uses, evaluated at zero
% deviation rather than a second, independent implementation.
spline_point(ControlPoints, U, X, Y) :-
    walk_noisy_point(ControlPoints, 0.0, 1.0, 0.0, 0.0, U, X, Y).

% arc length via one-time numeric integration (deterministic,
% computed ONCE per distinct ControlPoints -- not a random draw,
% not re-simulated per query; this is exactly the "approximate"
% arc-length option discussed: U is treated as advancing linearly
% with elapsed-time fraction, i.e. speed is only approximately
% constant along strongly-curved segments)
arc_length(ControlPoints, Length) :-
    ArcSamples = 50,
    ArcSamplesHi is ArcSamples - 1,
    findall(D,
        ( between(0, ArcSamplesHi, I),
          U0 is I / ArcSamples, U1 is (I+1) / ArcSamples,
          spline_point(ControlPoints, U0, X0,Y0),
          spline_point(ControlPoints, U1, X1,Y1),
          dist(X0,Y0,X1,Y1,D)
        ), Ds),
    sum_list(Ds, Length).

% speed/1 is now a config fact -- see the problem's own config.yaml's motion.speed.
% tool_speed/2 is its EQUIPPED-TOOL-AWARE generalization -- tool_speed
% (free,Speed) reuses the SAME base speed/1 value (config_to_prolog.py
% emits all three tool_speed(free/cart/plow,_) facts uniformly, cart/
% plow defaulting to speed/1's own value if config.yaml's tool.equipped
% section doesn't override them -- see that file's own note), so
% walk_duration/3 below never needs to special-case "no tool equipped"
% itself. speed/1 stays a separate fact (not removed) since tool_speed
% (free,_) is DERIVED from it at generation time, not the other way
% round.

% walk_duration(+ControlPoints,+Tool,+S,-Duration): Tool is WHICH tool
% (if any -- free/cart/plow, see hitch/2) is equipped for the walk this
% Duration is being computed for, per this feature's own request
% ("if the robot is equipped with the cart or the plow... the moveto
% action will use these parameters (velocity and battery drain rate)
% for its evaluations. once the tool is uninstalled they get back to
% the normal drain rate and velocity"). Callers resolve Tool via
% hitch(Tool,S) at the SAME situation-term point they already resolve
% CP from (current_walk/6 or the leg's own startMoveto(...) term) --
% hitch/2 is PROVABLY constant for the whole span of one walk (only
% halt_install_tool/halt_uninstall_tool ever change it, and both
% REQUIRE \+ moving(S) just to start -- see their own poss/2 notes), so
% it makes no difference whether a given call site re-derives it from S
% or from SPrev; either reads the SAME value. deployed/1 (Section 5c)
% has the SAME provably-constant-for-one-walk property, for the SAME
% reason (start_deploy_tool/start_retract_tool also require \+ moving
% (S)) -- S is threaded through here ONLY so effective_tool_speed/3
% below can tell whether Tool is currently DEPLOYED (a plow lowered
% into the ground moves differently than one just carried along), same
% situation argument, same "either S or SPrev works" property.
walk_duration(ControlPoints, Tool, S, Duration) :-
    arc_length(ControlPoints, Length),
    effective_tool_speed(Tool, S, Speed),
    Duration is Length / Speed.

% effective_tool_speed(+Tool,+S,-Speed) / effective_tool_moving_drain_
% rate(+Tool,+S,-Rate): the deployed-aware wrapper around tool_speed/2/
% tool_moving_drain_rate/2 -- TWO mutually exclusive clauses (deployed(
% S) vs. \+ deployed(S)), not an if-then-else (ProbLog's own dialect
% doesn't support '->'/2, same convention every other multi-case
% predicate in this file already uses). tool_speed_deployed/2 and
% tool_moving_drain_rate_deployed/2 (this problem's own config.yaml,
% tool.equipped.<kind>.deployed_speed/.deployed_moving_drain_rate) are
% emitted for EVERY tool kind, same as tool_speed/2 itself, even though
% only the plow can currently ever actually be deployed (see poss(
% start_deploy_tool(...))'s own kind restriction) -- a kind that can
% never deploy simply never has deployed(S) true while it's the one
% hitched, so its own _deployed value is harmless, never-consulted
% config, not a special case to avoid emitting.
effective_tool_speed(Tool, S, Speed) :-
    deployed(S),
    tool_speed_deployed(Tool, Speed).
effective_tool_speed(Tool, S, Speed) :-
    \+ deployed(S),
    tool_speed(Tool, Speed).

effective_tool_moving_drain_rate(Tool, S, Rate) :-
    deployed(S),
    tool_moving_drain_rate_deployed(Tool, Rate).
effective_tool_moving_drain_rate(Tool, S, Rate) :-
    \+ deployed(S),
    tool_moving_drain_rate(Tool, Rate).

% ---------------------------------------------------------------
% 4. STOCHASTIC LATERAL DRIFT -- ONE discretized-Gaussian draw
%    per WALK INSTANCE (i.e. per startMoveto occurrence), resolved
%    as a situation-indexed probabilistic fact (never resampled).
%    Deviation grows with elapsed-time fraction of the ORIGINAL
%    (full) walk duration, Brownian-bridge style: 0 at the start,
%    sigma*sqrt(Duration) (in std-dev units) at natural completion.
%    If the walk is cut short by an interrupt, the deviation simply
%    stops growing wherever it was at the moment of interruption --
%    it does not "reset" or get resampled.
% ---------------------------------------------------------------
% z/2's annotated-disjunction table and sigma/1 (its scale) are now
% config facts -- see the problem's own config.yaml's
% position.lateral.discretized_gaussian and position.lateral.sigma,
% generated into config_generated.pl (consulted at the top of this
% file) by module/translators/config_to_prolog.py. NOTE kept here as a
% standing reminder even though the table itself moved: these weights
% MUST sum to EXACTLY 1.0 -- an earlier version of this table used
% 0.3854 for the centre weight where 0.3954 was required, which summed
% to only 0.99, and ProbLog silently treats missing mass as an
% implicit "none of these" failure branch, which caps EVERY downstream
% probability at 0.99 in every world. The generator checks each
% table's own sum and warns if it's off; always verify it after
% editing config.yaml regardless.

% ---------------------------------------------------------------
% 5. THE MOVING FLUENT -- true while a walk is in progress (between
%    startMoveto and whatever action ends it: haltMoveto OR
%    interrupt). Other actions can gate their Poss axioms on
%    \+ moving(S) (can't start a new walk while moving) or on moving(S)
%    (an interrupt can only fire while a walk IS in progress).
%    Pure regression, exactly on the same footing as at/4.
% ---------------------------------------------------------------
moving(do(startMoveto(_,_,_,_), _)).
moving(do(A,S)) :-
    A \= haltMoveto(_,_,_), A \= interrupt(_),
    moving(S).
% (s0 is not moving: no clause covers it, so moving(s0) correctly fails)

% current_walk(+S, -ControlPoints, -T0): the control points and
% start time of the MOST RECENT startMoveto in S's history. Used
% whenever moving(S) holds (i.e. there is exactly one open walk to
% find), and also to recover the just-closed walk's parameters
% right after an haltMoveto/interrupt action.
current_walk(S, CP, T0) :- current_walk(S, CP, _Triggers, T0, _SPrev).

% current_walk/4 additionally exposes SPrev, the situation
% IMMEDIATELY BEFORE the startMoveto occurred -- needed to look up
% that walk's resolved noise draw Z via
% z(do(startMoveto(CP,Triggers,T0),SPrev),Z), since z/2's key is the
% exact ground startMoveto action term, which includes the situation
% it was added to (and, now, the Triggers list it was called with).
current_walk(S, CP, T0, SPrev) :- current_walk(S, CP, _Triggers, T0, SPrev).

% current_walk/5 additionally exposes Triggers -- the leg's own list
% of EXTRA halting conditions -- needed wherever the earliest-wins
% computation over Triggers has to run (Poss(haltMoveto(...)),
% interrupt's Poss). UNCHANGED signature/behaviour for every one of
% its own (many) existing callers -- now a thin wrapper dropping
% ActionCode from current_walk/6 below, exactly the same "/3 and /4
% are thin wrappers" pattern as when SPrev was added to /3 earlier.
current_walk(S, CP, Triggers, T0, SPrev) :-
    current_walk(S, CP, Triggers, _ActionCode, T0, SPrev).

% current_walk/6 -- the REAL base fact/recursion, over startMoveto/4
% (CP,Triggers,ActionCode,T0) now instead of startMoveto/3. ActionCode
% is the per-MoveTo-OCCURRENCE code bt_to_prolog.py assigns (mirrors
% next_reactive_code()'s own per-reactive-composite code -- see that
% file's own _VarPool note), embedded into the action term by
% poss(startMoveto(...)) below and carried straight through by do_node
% (moveto_leg(...)) -- ONLY poss(haltMoveto(...)) actually needs the
% real value (to tag the final halt Reason with it -- see tag_reason/3
% further down); every other current_walk/5 caller is unaffected.
current_walk(do(startMoveto(CP,Triggers,ActionCode,T0),SPrev), CP, Triggers, ActionCode, T0, SPrev).
current_walk(do(A,S), CP, Triggers, ActionCode, T0, SPrev) :-
    A \= startMoveto(_,_,_,_),
    current_walk(S, CP, Triggers, ActionCode, T0, SPrev).

% ---------------------------------------------------------------
% 5b. INSTALLING_TOOL / UNINSTALLING_TOOL -- the install_tool/
%     uninstall_tool analogue of moving/1 above: true while a fixed-
%     Duration install (or uninstall) is in progress, between
%     start_install_tool(Tool,...)/start_uninstall_tool(Tool,...) and
%     whichever halt_install_tool(...)/halt_uninstall_tool(...) ends
%     it -- there is no interrupt(...) counterpart for either (not
%     asked for; the plan never voluntarily cuts one short the way it
%     can a walk). Tool is carried in the base clause's own head so a
%     caller can ask "is THIS tool currently being installed" (used as
%     halt_install_tool's own PRECONDITION, per this action's own
%     request) as well as just "is something being installed".
installing_tool(Tool, do(start_install_tool(Tool,_,_,_), _)).
installing_tool(Tool, do(A,S)) :-
    A \= halt_install_tool(_,_,_),
    installing_tool(Tool, S).

uninstalling_tool(Tool, do(start_uninstall_tool(Tool,_,_,_), _)).
uninstalling_tool(Tool, do(A,S)) :-
    A \= halt_uninstall_tool(_,_,_),
    uninstalling_tool(Tool, S).

% current_install_tool(+S, -Tool,-Triggers,-ActionCode,-T0,-SPrev):
% the install_tool analogue of current_walk/6 above -- the most recent
% start_install_tool in S's history, generically skipping over any
% OTHER action layered on top (same pass-through shape). No CP slot at
% all -- install_tool has no path, nothing to recover. Used exactly
% where current_walk/6 is: recovering a leg's own parameters at
% poss(halt_install_tool(...)) time.
current_install_tool(do(start_install_tool(Tool,Triggers,ActionCode,T0),SPrev), Tool, Triggers, ActionCode, T0, SPrev).
current_install_tool(do(A,S), Tool, Triggers, ActionCode, T0, SPrev) :-
    A \= start_install_tool(_,_,_,_),
    current_install_tool(S, Tool, Triggers, ActionCode, T0, SPrev).

current_uninstall_tool(do(start_uninstall_tool(Tool,Triggers,ActionCode,T0),SPrev), Tool, Triggers, ActionCode, T0, SPrev).
current_uninstall_tool(do(A,S), Tool, Triggers, ActionCode, T0, SPrev) :-
    A \= start_uninstall_tool(_,_,_,_),
    current_uninstall_tool(S, Tool, Triggers, ActionCode, T0, SPrev).

% deploying_tool/2, retracting_tool/2, current_deploy_tool/6,
% current_retract_tool/6: the deploy_tool/retract_tool analogues of
% installing_tool/2, uninstalling_tool/2, current_install_tool/6,
% current_uninstall_tool/6 just above -- IDENTICAL shape, start_deploy_
% tool/start_retract_tool in place of start_install_tool/start_
% uninstall_tool, halt_deploy_tool/halt_retract_tool in place of
% halt_install_tool/halt_uninstall_tool.
deploying_tool(Tool, do(start_deploy_tool(Tool,_,_,_), _)).
deploying_tool(Tool, do(A,S)) :-
    A \= halt_deploy_tool(_,_,_),
    deploying_tool(Tool, S).

retracting_tool(Tool, do(start_retract_tool(Tool,_,_,_), _)).
retracting_tool(Tool, do(A,S)) :-
    A \= halt_retract_tool(_,_,_),
    retracting_tool(Tool, S).

current_deploy_tool(do(start_deploy_tool(Tool,Triggers,ActionCode,T0),SPrev), Tool, Triggers, ActionCode, T0, SPrev).
current_deploy_tool(do(A,S), Tool, Triggers, ActionCode, T0, SPrev) :-
    A \= start_deploy_tool(_,_,_,_),
    current_deploy_tool(S, Tool, Triggers, ActionCode, T0, SPrev).

current_retract_tool(do(start_retract_tool(Tool,Triggers,ActionCode,T0),SPrev), Tool, Triggers, ActionCode, T0, SPrev).
current_retract_tool(do(A,S), Tool, Triggers, ActionCode, T0, SPrev) :-
    A \= start_retract_tool(_,_,_,_),
    current_retract_tool(S, Tool, Triggers, ActionCode, T0, SPrev).

% ---------------------------------------------------------------
% 5c. THE HITCH FLUENT -- which tool KIND (if any) is currently
%     attached: free (nothing attached), or the kind's own name
%     (cart/plow). Starts free in s0. Flips to Kind ONLY on a
%     SUCCESSFUL halt_install_tool(Id,...) (a FAILED attempt leaves it
%     unchanged -- nothing actually got attached), and back to free
%     ONLY on a SUCCESSFUL halt_uninstall_tool(...) -- per this
%     action's own request. Monotonic-persistence (frame axiom) is the
%     generic pass-through clause, same shape as at/4's/battery/3's own
%     -- everything else in the history leaves hitch/2 unchanged.
%     Reason (install_tool_success(Id,ActionCode)/uninstall_tool_
%     success(Id,ActionCode)) is matched here in its FULLY TAGGED shape
%     (see tag_reason/3) -- halt_install_tool/halt_uninstall_tool
%     always carry the tagged Reason directly as their own 2nd
%     argument (see do_node(install_tool_leg(...))'s own note further
%     down), so there is no separate untagged form to also match here.
%
%     KIND vs. INSTANCE: a BT tree's own <InstallTool tool="..."> names
%     a specific tool INSTANCE id (e.g. "cart1", from this problem's
%     own config.yaml tool.instances -- see tool_instance/2 below), not
%     a kind, since multiple instances of the same kind can exist. But
%     every OTHER predicate that used to read hitch(Tool,S) purely to
%     look up KIND-level physics (walk_duration/2 via tool_speed/2,
%     tool_moving_drain_rate/2, install_tool_duration/2, ...) still
%     wants the KIND, not which specific instance -- so hitch/2 itself
%     stays kind-valued (join through tool_instance/2 to recover Kind
%     from the event's own Id), leaving every one of those existing
%     callers UNTOUCHED. hitch_id/2 immediately below is the sibling
%     fluent for the few places that genuinely need to know WHICH
%     instance (tool_position/4's own hitched-exclusion, and
%     poss(start_uninstall_tool(...))'s own precondition -- uninstalling
%     Id requires THAT SPECIFIC instance, not just "some instance of
%     its kind", to be the one attached).
hitch(free, s0).
hitch(Kind, do(halt_install_tool(_T,install_tool_success(Id,_ActionCode),true), _)) :-
    tool_instance(Id, Kind).
hitch(free, do(halt_uninstall_tool(_T,uninstall_tool_success(_Id,_ActionCode),true), _)).
hitch(State, do(A,S)) :-
    A \= halt_install_tool(_,install_tool_success(_,_),true),
    A \= halt_uninstall_tool(_,uninstall_tool_success(_,_),true),
    hitch(State, S).

% hitch_id/2: the INSTANCE-level sibling of hitch/2 above -- IDENTICAL
% successor-state shape, just reporting the tool instance Id directly
% (no tool_instance/2 join), so it can distinguish "cart1 is attached"
% from "cart2 is attached" where hitch/2 would only ever say "cart" for
% either.
hitch_id(free, s0).
hitch_id(Id, do(halt_install_tool(_T,install_tool_success(Id,_ActionCode),true), _)).
hitch_id(free, do(halt_uninstall_tool(_T,uninstall_tool_success(_Id,_ActionCode),true), _)).
hitch_id(State, do(A,S)) :-
    A \= halt_install_tool(_,install_tool_success(_,_),true),
    A \= halt_uninstall_tool(_,uninstall_tool_success(_,_),true),
    hitch_id(State, S).

% ---------------------------------------------------------------
% 5d. THE DEPLOYED FLUENT -- true while the currently-hitched tool is
%     LOWERED (deployed), as opposed to merely attached. Starts false
%     (no clause covers s0, same "absence is false" convention moving/1
%     itself relies on). Flips to true ONLY on a SUCCESSFUL
%     halt_deploy_tool(...), back to false ONLY on a SUCCESSFUL
%     halt_retract_tool(...) -- a FAILED attempt at either leaves it
%     unchanged, same "only a SUCCESSFUL halt flips it" discipline
%     hitch/2 already uses (the frame clause's own negative guard
%     simply doesn't match a successful retract, so there's no separate
%     "becomes false" clause to also write -- same trick moving/1 uses
%     for its own ending case). No Tool/Id argument, unlike hitch/hitch_
%     id -- only ONE tool can ever be hitched at a time, so "is the
%     currently-hitched tool deployed" needs no further qualification.
%
%     WHY THIS EXISTS: merely installing the plow (hitch(plow,S)) does
%     NOT plough anything while the robot moves -- see ploughed/3's own
%     note, further down -- and does NOT switch MoveTo to the deployed-
%     specific speed/drain-rate either (effective_tool_speed/3,
%     effective_tool_moving_drain_rate/3, near walk_duration/4 above).
%     Both require deployed(S) too, which only a genuine DeployTool
%     (poss(start_deploy_tool(...)), below) can make true.
deployed(do(halt_deploy_tool(_T,deploy_tool_success(_Id,_ActionCode),true), _)).
deployed(do(A,S)) :-
    A \= halt_deploy_tool(_,deploy_tool_success(_,_),true),
    A \= halt_retract_tool(_,retract_tool_success(_,_),true),
    deployed(S).

% tool_position(+Id, -GX, -GY, +S): relational fluent -- WHERE tool
% instance Id currently sits, or fails outright if Id is currently
% attached (its "position" is trivially wherever the robot is, i.e.
% at/4, not a separate thing to track -- and if a caller only means
% "some instance of this kind, wherever it is", tools_of_kind/5 below
% already excludes hitched instances for exactly this reason, so no
% consumer should ever need a stale value here). Base case seeds from
% this problem's own config.yaml (tool.instances[].x/y -- see
% tool_start_position/3, config_to_prolog.py's own note). Updates ONLY
% on a SUCCESSFUL uninstall of THIS Id, to wherever the robot actually
% was (at/4) at that exact moment -- i.e. the tool is physically
% dropped there. Persistence clause additionally requires \+ hitch_id
% (Id, do(A,S)) -- NOT just "nothing changed since the last update",
% same as at/4's/hitch/2's own frame clauses, but ALSO gated on "not
% currently equipped" so the fluent goes silent (fails) for exactly as
% long as Id stays attached, per this feature's own request, rather
% than continuing to report a now-meaningless frozen value.
%
% tool_instance/2 and tool_start_position/3 THEMSELVES are pure config
% data (config_to_prolog.py's own tool.instances -- see that file's own
% note), never defined by a real clause anywhere in THIS file -- a
% problem whose config.yaml has no tool.instances at all (or an empty
% list) then has ZERO facts for either, anywhere in the whole consulted
% program. ProbLog's own engine treats that as an outright "unknown
% procedure" ERROR (not a graceful failure) the moment anything calls
% either one -- unlike ordinary Prolog, ProbLog's parser also doesn't
% support a ':- dynamic Name/Arity.' declaration to pre-empt this (tried
% directly, confirmed unsupported). The fix is the classic Prolog
% workaround instead: one placeholder clause per predicate whose own
% body can never succeed, purely so the predicate is "known" to the
% engine -- any REAL config-generated facts simply add MORE clauses
% alongside this one, and a query against an unknown id/kind still
% fails cleanly, exactly as intended, instead of throwing.
tool_instance(no_tool_instances_configured, no_tool_instances_configured) :- fail.
tool_start_position(no_tool_instances_configured, 0.0, 0.0) :- fail.

tool_position(Id, GX, GY, s0) :-
    tool_start_position(Id, GX, GY).
tool_position(Id, GX, GY, do(halt_uninstall_tool(T,uninstall_tool_success(Id,_ActionCode),true), S)) :-
    at(GX, GY, T, S).
tool_position(Id, GX, GY, do(A,S)) :-
    A \= halt_uninstall_tool(_,uninstall_tool_success(Id,_),true),
    tool_position(Id, GX, GY, S),
    \+ hitch_id(Id, do(A,S)).

% tools_of_kind(+Kind, +S, -Id, -GX, -GY): nondeterministically
% enumerates every tool instance of Kind that currently HAS a position
% -- i.e. every instance tool_position/4 above will actually report for
% right now, so a currently-hitched instance of this Kind is silently
% skipped (the exclusion already lives in tool_position/4 itself, not
% duplicated here). Backtracks over every tool_instance(Id,Kind) fact;
% a Kind with no free instance simply fails outright, the same
% "unsatisfied precondition, not a special error case" shape every
% other fluent in this file already uses for "nothing to report".
tools_of_kind(Kind, S, Id, GX, GY) :-
    tool_instance(Id, Kind),
    tool_position(Id, GX, GY, S).

% nearest_tool_of_kind(+Kind, +S, -Id, -GX, -GY, -Dist): composes
% tools_of_kind/5 (which itself composes tool_instance/2 with
% tool_position/4) with the robot's own CURRENT position (now/2+at/4,
% same "read S at exactly this point" pattern do_node(planWith(...))
% already uses) to pick whichever free instance of Kind is closest
% right now. The argmin fold is ordinary Prolog -- findall/3 to collect
% every candidate's own distance, then min_candidate/2 below to fold
% down to the smallest -- not a new KIND of computation this file
% hasn't already needed (the same "closest of several candidates" shape
% an obstacle-argmin Reason already resolves elsewhere, just over tool
% instances instead). Fails outright if Kind has no free instance at
% all (Candidates=[]), same "nothing to report" shape as tools_of_kind/5
% itself.
nearest_tool_of_kind(Kind, S, Id, GX, GY, Dist) :-
    now(T, S), at(RX,RY,T,S),
    findall(D-CandId-CX-CY,
            (tools_of_kind(Kind,S,CandId,CX,CY), dist(RX,RY,CX,CY,D)),
            Candidates),
    Candidates \= [],
    min_candidate(Candidates, Dist-Id-GX-GY).

% min_candidate(+[D-Id-X-Y|...], -Best): folds a non-empty list of
% Dist-Id-X-Y candidates down to the single smallest-Dist one. TWO
% mutually exclusive clauses on D1=<D2 vs. D1>D2, rather than an if-
% then-else -- ProbLog's own Prolog dialect doesn't support '->'/2,
% same reason every other multi-case predicate in this file (e.g.
% guard_bracket_scan) is written this way instead. A tie (D1=:=D2)
% deterministically keeps the FIRST-encountered candidate (only the
% D1=<D2 clause matches), never both -- so this always yields exactly
% one Best, never two competing derivations for the same Dist.
min_candidate([D-Id-X-Y], D-Id-X-Y).
min_candidate([D1-Id1-X1-Y1,D2-Id2-X2-Y2|Rest], Best) :-
    D1 =< D2,
    min_candidate([D1-Id1-X1-Y1|Rest], Best).
min_candidate([D1-Id1-X1-Y1,D2-Id2-X2-Y2|Rest], Best) :-
    D1 > D2,
    min_candidate([D2-Id2-X2-Y2|Rest], Best).

% ---------------------------------------------------------------
% 4b. THE BATTERY FLUENT -- a second clock fluent, on the exact same
%     footing as at/4: battery(Level,T,S) is the charge level (0..100,
%     percent) at real clock-time T in situation S.
%
%     - starts at 100 in s0
%     - drains LINEARLY in time, at a rate that depends on whether the
%       robot is moving or idle (idle_drain_rate/1 while \+ moving(S),
%       moving_drain_rate/1 while moving(S))
%     - UNLIKE position, battery does NOT freeze after a halt/interrupt
%       -- it keeps draining at the idle rate for as long as the robot
%       sits still afterwards. Position freezing (at/4) and battery's
%       continued idle drain are genuinely different physical
%       behaviours, so they need different regression clauses here,
%       even though both are "clock fluents" in the same formal sense.
%     - stochastic: ONE extra discretized-Gaussian draw per walk
%       instance (zbatt/2, mirroring z/2's role for position), added
%       to the moving drain rate. Because the resulting rate is still
%       CONSTANT over a given walk (the noise is a single per-walk
%       draw, not a fresh draw per instant), battery level stays
%       EXACTLY LINEAR in elapsed time within one walk -- so, unlike
%       collision (which needs bracket-scan + bisection because it's a
%       cubic-spline-vs-polygon test with no closed form), the exact
%       depletion time is solvable by plain algebra. See
%       first_battery_depletion_time/6 below.
% ---------------------------------------------------------------
% battery_start/1, idle_drain_rate/1, moving_drain_rate/1, and
% sigma_battery/1 (the SAME value used in every phase -- moving, idle,
% and s0 -- see zbatt/1 below) are now config facts -- see
% the problem's own config.yaml's battery.* (battery.sigma for
% sigma_battery/1 specifically -- everything battery-related, noise
% included, lives in that one section now).

% zbatt/1: ONE noise draw for the WHOLE MISSION -- deliberately
% decoupled from any specific startMoveto occurrence. This is an
% annotated disjunction with NO ARGUMENTS at all, which ProbLog
% grounds EXACTLY ONCE for the entire program: every reference to
% zbatt(Zb), anywhere in the theory, in every world, refers to the
% SAME single resolved value. This treats "how much this particular
% battery underperforms" as a persistent property of the battery
% itself -- present from the very start, not a fresh, independent
% draw manufactured at each walk. Its annotated-disjunction table is
% now ALSO a config fact -- the SAME noise.discretized_gaussian entries
% as z/2 (see config.yaml), instantiated a second time with zbatt/1's
% own zero-argument functor by module/translators/config_to_prolog.py, so the
% two tables can never drift apart.

% s0's idle phase now ALSO carries genuine stochasticity, using the
% SAME global Zb -- consistent with every other phase, no more
% special-cased determinism here. Uses sqrt(Elapsed) scaling (true
% Wiener-consistent growth, Var(Deviation)=sigma^2*Elapsed) rather
% than the Duration-normalized form used below: at s0 there is no
% walk yet (past OR upcoming) to borrow a reference Duration from,
% and idle time here is genuinely UNBOUNDED (T can be arbitrarily
% large before the first action ever fires -- you noted the first
% action can be scheduled to start after some delay, not just at
% T=0), so there is nothing else to normalize against.
% noisy_drain(+NominalDrain, +Deviation, -TotalDrain): the SHARED,
% MONOTONICITY-SAFE combination used by every phase below. Deviation
% > 0 means "drained LESS than nominal" (a well-performing battery
% this episode); Deviation < 0 means "drained MORE". Clamped at 0 so
% TotalDrain can NEVER be negative -- guaranteeing Level = B_start -
% TotalDrain never EXCEEDS B_start, i.e. battery level is
% monotonically NON-INCREASING over time, BY CONSTRUCTION, regardless
% of how large Deviation gets or which parameters are chosen. In the
% extreme case (noise would suggest giving back more than the full
% nominal drain), the physically sensible floor is "the battery
% simply doesn't drain during this stretch" -- not "the battery
% gains charge", which unclamped additive-to-LEVEL noise could
% otherwise produce for small Elapsed (sqrt(Elapsed) growing faster
% than the linear IdleRate*Elapsed term near Elapsed=0).
noisy_drain(NominalDrain, Deviation, TotalDrain) :-
    TotalDrain is max(0, NominalDrain - Deviation).

% ALL THREE idle-phase clauses (s0, after-halt, after-interrupt) now
% share the IDENTICAL structure: sqrt(Elapsed) scaling, no Duration
% reference at all -- idle time is genuinely UNBOUNDED in every one
% of these cases (T can be arbitrarily large before the first action
% ever fires, or arbitrarily long after any halt/interrupt), so
% there is nothing walk-specific to normalize against, and using the
% SAME formula everywhere removes the earlier inconsistency where
% walk-adjacent idle borrowed the preceding walk's Duration while s0
% did not, despite both being the same kind of unbounded stretch.
battery(Level, T, s0) :-
    battery_start(B0),
    idle_drain_rate(IdleRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Elapsed is max(0.0, T),
    Deviation is Zb * SigmaB * sqrt(Elapsed),
    NominalDrain is IdleRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B0 - TotalDrain)).

% leg_start_battery(+T0, +SPrev, -B0): a NEW leg's own starting
% battery level, rounded DOWN to disc_step_battery/1's own
% granularity (see the MERGE-GRID QUANTIZATION note above dist/5's own
% section) -- the SINGLE shared definition used both here (the MOVING-
% phase clause just below) and by poss(haltMoveto(...))/poss(interrupt
% (...)) further down, so a leg's own crossing-time computation and its
% own battery/3 regression can never read a different B0 for the same
% leg. Quantizing HERE -- exactly where a NEW leg's own physics starts
% -- is what keeps every OTHER battery/3 read (idle-phase reporting,
% mid-walk queries, first_hit/on_track/verify_safe) at full float
% precision, unaffected: only the value that seeds a brand new leg is
% coarsened, once, at the seam.
leg_start_battery(T0, SPrev, B0) :-
    battery(B0Exact, T0, SPrev),
    disc_step_battery(Grid),
    quantize_down(B0Exact, Grid, B0).

% moving_phase_deviation(+Zb,+SigmaB,+Elapsed,+Duration,-Deviation):
% the shared "Duration-normalized battery-noise deviation" formula
% EVERY fixed-Duration battery-draining action's own MOVING/ACTIVE-
% phase battery/3 clause below uses (MoveTo, InstallTool,
% UninstallTool, DeployTool, RetractTool -- and battery_at_leg/7's own
% by-hand-synced copy of MoveTo's), factored into one place rather
% than six copies of the same arithmetic. Duration=<0.0 is a genuine,
% valid case (a zero-length/zero-Duration leg -- every planner in
% planners.py returns a degenerate, zero-arc-length control_points
% list for an "already at the goal" leg, see e.g. _dastar_control_
% points' own start_rc==goal_rc special case, and walk_duration/3
% computes Duration=Length/Speed=0.0 for it; install/uninstall/deploy/
% retract can likewise have a config-set Duration of 0), not something
% to reject: Elapsed is ALWAYS 0.0 too whenever Duration=<0.0 (every
% caller below clamps it to max(0.0,min(Elapsed0,Duration)) first), so
% Deviation=0.0 is the correct, well-defined limit -- zero elapsed
% time means no opportunity for drift to accumulate, exactly the "no
% noise" reading this formula would give anyway if 0.0/sqrt(Duration)
% were merely undefined rather than a hard ZeroDivisionError. TWO
% mutually exclusive clauses, same "no if-then-else" convention every
% other multi-case predicate in this file already uses.
moving_phase_deviation(_Zb, _SigmaB, _Elapsed, Duration, 0.0) :-
    Duration =< 0.0.
moving_phase_deviation(Zb, SigmaB, Elapsed, Duration, Deviation) :-
    Duration > 0.0,
    Deviation is Zb * SigmaB * Elapsed / sqrt(Duration).

% MOVING phase keeps the Duration-normalized scaling (Elapsed/sqrt(D),
% not sqrt(Elapsed)) -- a walk DOES have a genuine, known, fixed
% Duration, and that normalization is what keeps Level EXACTLY LINEAR
% in elapsed time (needed for first_battery_depletion_time's
% closed-form algebraic solve, rather than bracket-scan+bisection).
% Structurally this is now the SAME "nominal drain minus a signed
% deviation, clamped at zero" pattern as the idle phases -- only the
% Deviation formula's normalization differs, for the reason above.
% Tool -- hitch(Tool,S), S being the situation right before this walk
% started (same one leg_start_battery/3 already reads B0 from) --
% picks which of tool_speed/2 (via walk_duration/3)/tool_moving_drain_
% rate/2 applies for THIS walk's own Duration/MovingRate, per this
% feature's own request. See walk_duration/3's own note on why it's
% safe to resolve Tool from S here (rather than, say, some later point
% mid-walk): hitch/2 cannot change while moving(S) holds.
battery(Level, T, do(startMoveto(CP,_Triggers,_ActionCode,T0), S)) :-
    leg_start_battery(T0, S, B0),
    hitch(Tool, S),
    walk_duration(CP, Tool, S, Duration),
    Elapsed0 is T - T0,
    Elapsed is max(0.0, min(Elapsed0, Duration)),
    effective_tool_moving_drain_rate(Tool, S, MovingRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    moving_phase_deviation(Zb, SigmaB, Elapsed, Duration, Deviation),
    NominalDrain is MovingRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B0 - TotalDrain)).

% after a halt/interrupt: for T at or before the halt, delegate
% straight through (same value as during the walk); for T after it,
% the walk's own value at the halt instant becomes a new anchor and
% IDLE drain resumes from there -- this is the "does not freeze"
% behaviour that distinguishes battery from at/4. No current_walk/
% Duration lookup is needed here anymore -- see the note above.
battery(Level, T, do(haltMoveto(T1,Reason,Status), S)) :-
    T =< T1,
    battery(Level, T, S).
battery(Level, T, do(haltMoveto(T1,Reason,Status), S)) :-
    T > T1,
    battery(B1, T1, S),
    idle_drain_rate(IdleRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Elapsed is T - T1,
    Deviation is Zb * SigmaB * sqrt(Elapsed),
    NominalDrain is IdleRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B1 - TotalDrain)).

battery(Level, T, do(interrupt(T1), S)) :-
    T =< T1,
    battery(Level, T, S).
battery(Level, T, do(interrupt(T1), S)) :-
    T > T1,
    battery(B1, T1, S),
    idle_drain_rate(IdleRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Elapsed is T - T1,
    Deviation is Zb * SigmaB * sqrt(Elapsed),
    NominalDrain is IdleRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B1 - TotalDrain)).

% install_tool/uninstall_tool: OWN Duration-normalized anchor clauses,
% EXACTLY mirroring startMoveto/haltMoveto above -- install_tool_drain_
% rate/1 (or uninstall_tool_drain_rate/1) in place of moving_drain_
% rate, a DEDICATED rate for the span of the action itself, distinct
% from idle_drain_rate (this feature's own request: "keep a time and
% drain rate of the battery specific for the action of installing or
% uninstalling any tool" -- ONE rate per ACTION TYPE, defaulting to
% idle_drain_rate's own configured value if config.yaml doesn't
% override it, see config_to_prolog.py's own note), and install_tool_
% duration(Tool,Duration) (or uninstall_tool_duration(Tool,Duration))
% in place of walk_duration(CP,Tool,S,Duration). Even though install_
% tool/uninstall_tool never move the robot, each one is still a FIXED-
% DURATION leg exactly like a MoveTo leg is -- so it gets the SAME
% "nominal drain minus a Duration-normalized signed deviation"
% treatment that keeps Level EXACTLY LINEAR in elapsed time within the
% leg (needed for first_battery_depletion_time's own closed-form
% algebraic solve, shared with MoveTo -- same reason startMoveto's own
% comment gives),
% rather than the open-ended sqrt(Elapsed) idle formula the post-halt
% clauses below (and interrupt's) use -- that one is for a genuinely
% UNBOUNDED wait, where no Duration is known ahead of time, which is
% not the case here. The post-halt clauses are themselves an EXACT
% copy of haltMoveto's own post-halt clause, deliberately STILL using
% plain idle_drain_rate (not install_tool_drain_rate/uninstall_tool_
% drain_rate) -- once the leg ends the robot is just idle again, no
% different from after a walk halts, exactly this feature's own "if
% the robot doesn't move [and isn't mid-install/uninstall] we use the
% same idle drain rate, no change on that" note.
battery(Level, T, do(start_install_tool(Id,_Triggers,_ActionCode,T0), S)) :-
    leg_start_battery(T0, S, B0),
    tool_instance(Id, Kind),
    install_tool_duration(Kind, Duration),
    install_tool_drain_rate(Rate),
    Elapsed0 is T - T0,
    Elapsed is max(0.0, min(Elapsed0, Duration)),
    sigma_battery(SigmaB),
    zbatt(Zb),
    moving_phase_deviation(Zb, SigmaB, Elapsed, Duration, Deviation),
    NominalDrain is Rate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B0 - TotalDrain)).

battery(Level, T, do(halt_install_tool(T1,Reason,Status), S)) :-
    T =< T1,
    battery(Level, T, S).
battery(Level, T, do(halt_install_tool(T1,Reason,Status), S)) :-
    T > T1,
    battery(B1, T1, S),
    idle_drain_rate(IdleRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Elapsed is T - T1,
    Deviation is Zb * SigmaB * sqrt(Elapsed),
    NominalDrain is IdleRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B1 - TotalDrain)).

battery(Level, T, do(start_uninstall_tool(Id,_Triggers,_ActionCode,T0), S)) :-
    leg_start_battery(T0, S, B0),
    tool_instance(Id, Kind),
    uninstall_tool_duration(Kind, Duration),
    uninstall_tool_drain_rate(Rate),
    Elapsed0 is T - T0,
    Elapsed is max(0.0, min(Elapsed0, Duration)),
    sigma_battery(SigmaB),
    zbatt(Zb),
    moving_phase_deviation(Zb, SigmaB, Elapsed, Duration, Deviation),
    NominalDrain is Rate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B0 - TotalDrain)).

battery(Level, T, do(halt_uninstall_tool(T1,Reason,Status), S)) :-
    T =< T1,
    battery(Level, T, S).
battery(Level, T, do(halt_uninstall_tool(T1,Reason,Status), S)) :-
    T > T1,
    battery(B1, T1, S),
    idle_drain_rate(IdleRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Elapsed is T - T1,
    Deviation is Zb * SigmaB * sqrt(Elapsed),
    NominalDrain is IdleRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B1 - TotalDrain)).

% start_deploy_tool/halt_deploy_tool/start_retract_tool/halt_retract_
% tool: SAME Duration-normalized anchor shape as the install/uninstall
% pair just above -- deploy_tool_duration/2 (or retract_tool_
% duration/2, both KIND-keyed via tool_instance/2) and deploy_tool_
% drain_rate/1 (or retract_tool_drain_rate/1) in place of install_tool_
% duration/uninstall_tool_duration/install_tool_drain_rate/
% uninstall_tool_drain_rate; post-halt idle drain is the SAME plain
% idle_drain_rate every other action's own post-halt clause already
% uses (once the leg ends the robot -- and the tool -- are just idle
% again).
battery(Level, T, do(start_deploy_tool(Id,_Triggers,_ActionCode,T0), S)) :-
    leg_start_battery(T0, S, B0),
    tool_instance(Id, Kind),
    deploy_tool_duration(Kind, Duration),
    deploy_tool_drain_rate(Rate),
    Elapsed0 is T - T0,
    Elapsed is max(0.0, min(Elapsed0, Duration)),
    sigma_battery(SigmaB),
    zbatt(Zb),
    moving_phase_deviation(Zb, SigmaB, Elapsed, Duration, Deviation),
    NominalDrain is Rate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B0 - TotalDrain)).

battery(Level, T, do(halt_deploy_tool(T1,Reason,Status), S)) :-
    T =< T1,
    battery(Level, T, S).
battery(Level, T, do(halt_deploy_tool(T1,Reason,Status), S)) :-
    T > T1,
    battery(B1, T1, S),
    idle_drain_rate(IdleRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Elapsed is T - T1,
    Deviation is Zb * SigmaB * sqrt(Elapsed),
    NominalDrain is IdleRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B1 - TotalDrain)).

battery(Level, T, do(start_retract_tool(Id,_Triggers,_ActionCode,T0), S)) :-
    leg_start_battery(T0, S, B0),
    tool_instance(Id, Kind),
    retract_tool_duration(Kind, Duration),
    retract_tool_drain_rate(Rate),
    Elapsed0 is T - T0,
    Elapsed is max(0.0, min(Elapsed0, Duration)),
    sigma_battery(SigmaB),
    zbatt(Zb),
    moving_phase_deviation(Zb, SigmaB, Elapsed, Duration, Deviation),
    NominalDrain is Rate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B0 - TotalDrain)).

battery(Level, T, do(halt_retract_tool(T1,Reason,Status), S)) :-
    T =< T1,
    battery(Level, T, S).
battery(Level, T, do(halt_retract_tool(T1,Reason,Status), S)) :-
    T > T1,
    battery(B1, T1, S),
    idle_drain_rate(IdleRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Elapsed is T - T1,
    Deviation is Zb * SigmaB * sqrt(Elapsed),
    NominalDrain is IdleRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B1 - TotalDrain)).

% pass-through: any future non-movement, non-tool action doesn't change
% how battery is computed -- it's a pure function of T and of whichever
% startMoveto/haltMoveto/interrupt/start_install_tool/halt_install_tool/
% start_uninstall_tool/halt_uninstall_tool/start_deploy_tool/halt_
% deploy_tool/start_retract_tool/halt_retract_tool anchors exist in the
% history, same principle as at/4's own pass-through clause.
battery(Level, T, do(A,S)) :-
    A \= startMoveto(_,_,_,_), A \= haltMoveto(_,_,_), A \= interrupt(_),
    A \= start_install_tool(_,_,_,_), A \= halt_install_tool(_,_,_),
    A \= start_uninstall_tool(_,_,_,_), A \= halt_uninstall_tool(_,_,_),
    A \= start_deploy_tool(_,_,_,_), A \= halt_deploy_tool(_,_,_),
    A \= start_retract_tool(_,_,_,_), A \= halt_retract_tool(_,_,_),
    battery(Level, T, S).

% moving_phase_effective_rate(+Rate,+Zb,+SigmaB,+Duration,-EffectiveRate):
% the shared "Duration-normalized effective drain rate" formula EVERY
% closed-form battery-crossing predicate below uses (first_battery_
% depletion_time/first_battery_below_time/first_battery_equal_time),
% factored into one place rather than three copies of the same
% arithmetic. Duration=<0.0 (a zero-length/zero-Duration leg -- see
% moving_phase_deviation/5's own note just above for why this is a
% genuine, valid case) makes this predicate simply FAIL rather than
% divide by sqrt(0.0) -- correct BY CONSTRUCTION, not a special case
% bolted on: a walk with zero elapsed time available can never
% experience ANY threshold crossing DURING it, so "no solution" is
% exactly the right answer, the same "FAILS = no crossing in this
% walk" convention every caller below already documents for its OTHER
% failure case (non-positive effective rate).
moving_phase_effective_rate(Rate, Zb, SigmaB, Duration, EffectiveRate) :-
    Duration > 0.0,
    EffectiveRate is Rate - Zb*SigmaB/sqrt(Duration).

% first_battery_depletion_time(+CP,+T0,+Duration,+B0,+Zb,+Tool,-Tcross):
% CLOSED-FORM (not bracket/bisect) -- battery is exactly LINEAR in
% elapsed time within one walk (fixed Zb => fixed EffectiveRate), so
% the crossing at Level=0 is a direct algebraic solve. FAILS (no
% clause matches) if the effective rate is non-positive (battery
% isn't actually decreasing -- noise happened to push it flat/up) or
% the algebraic crossing falls beyond this walk's own span -- both
% correctly represent "no depletion in this walk," exactly mirroring
% how first_collision_time fails when there's no crossing. Rate is the
% moving-phase battery drain rate, already resolved to a plain number
% by whichever poss/2 clause started this chain (tool_moving_drain_
% rate(Tool,Rate), same lookup battery/3's own do(startMoveto(...),S)
% clause makes) -- see earliest_halt/12's own note.
first_battery_depletion_time(CP,T0,Duration,B0,Zb,Rate,Tcross) :-
    sigma_battery(SigmaB),
    moving_phase_effective_rate(Rate, Zb, SigmaB, Duration, EffectiveRate),
    EffectiveRate > 0,
    Tcross0 is T0 + B0/EffectiveRate,
    Tcross0 =< T0 + Duration,
    Tcross = Tcross0.

% first_battery_below_time(+CP,+T0,+Duration,+B0,+Zb,+Rate,+Threshold,-Tcross):
% the SAME closed-form algebra as first_battery_depletion_time above,
% generalized to an arbitrary Threshold instead of hardcoded Level=0 --
% this is what battery_below(Threshold) (see TRIGGERS section and
% holds(battery_below(...)) further down) uses, kept as a genuinely
% SEPARATE predicate from first_battery_depletion_time/7 (not a
% generalize-in-place rename) since battery_depleted (Level=0 exactly)
% stays its own distinct trigger/Reason, on request. UNLIKE
% first_battery_depletion_time, B0 can legitimately already be AT OR
% BELOW an arbitrary Threshold at T0 (e.g. an earlier leg already
% drained below a later leg's chosen warning level without itself
% using battery_below) -- so this needs the explicit "already true at
% T0" graceful clause every other threshold-crossing predicate in this
% theory already has, rather than relying on the algebra to degrade
% gracefully on its own (it only does that at exactly Threshold=0).
first_battery_below_time(CP,T0,Duration,B0,Zb,_Rate,Threshold,T0) :-
    B0 =< Threshold.
first_battery_below_time(CP,T0,Duration,B0,Zb,Rate,Threshold,Tcross) :-
    B0 > Threshold,
    sigma_battery(SigmaB),
    moving_phase_effective_rate(Rate, Zb, SigmaB, Duration, EffectiveRate),
    EffectiveRate > 0,
    Tcross0 is T0 + (B0-Threshold)/EffectiveRate,
    Tcross0 =< T0 + Duration,
    Tcross = Tcross0.

% first_battery_equal_time(+CP,+T0,+Duration,+B0,+Zb,+Rate,+Threshold,-Tcross):
% the SAME closed-form algebra again, this time for Level(T) = Threshold
% EXACTLY. Battery is a continuous, monotonically non-increasing
% function of T within one walk (see noisy_drain/3's own note), so for
% any Threshold strictly between the walk's start and end level there
% is EXACTLY ONE instant it passes through it -- the same Tcross0
% formula as first_battery_below_time above, not a separate derivation.
% UNLIKE first_battery_below_time, there is NO "already true" grace
% clause for B0 < Threshold: if the battery is already BELOW Threshold
% at T0, it was EQUAL to it at some earlier, already-elapsed instant
% not covered by this walk -- it can only keep draining further away
% from Threshold from here, so this correctly FAILS (no clause matches)
% rather than firing at T0. B0 = Threshold exactly is its own graceful
% "already true" case (Tcross=T0), same convention as everywhere else.
first_battery_equal_time(CP,T0,Duration,B0,Zb,_Rate,Threshold,T0) :-
    B0 =:= Threshold.
first_battery_equal_time(CP,T0,Duration,B0,Zb,Rate,Threshold,Tcross) :-
    B0 > Threshold,
    sigma_battery(SigmaB),
    moving_phase_effective_rate(Rate, Zb, SigmaB, Duration, EffectiveRate),
    EffectiveRate > 0,
    Tcross0 is T0 + (B0-Threshold)/EffectiveRate,
    Tcross0 =< T0 + Duration,
    Tcross = Tcross0.

% first_battery_over_time(+CP,+T0,+Duration,+B0,+Zb,+Rate,+Threshold,-Tcross):
% Same TWO-CLAUSE shape as first_battery_below_time/first_battery_equal_time
% (an "already true at T0" clause plus a general future-crossing search),
% not a bespoke single-fact shortcut, so this stays a genuinely VALID,
% extensible check even though battery/3 only ever DRAINS today.
%
% Clause 1 (below): B0 > Threshold already at the walk's own start --
% fires immediately, exactly like the other two predicates' own
% "already true" clauses.
%
% Clause 2 (a genuine future-crossing search) is NOT YET WRITTEN, and
% deliberately so, rather than filled in with a guessed formula: under
% the CURRENT battery/3 regression, noisy_drain/3 clamps TotalDrain at
% max(0, ...), so Level is PROVABLY non-increasing for the whole walk
% (Level(T) <= B0 for all T) -- there is no future instant where
% Level(T) > Threshold unless it was already true at T0, so a second
% clause could only ever correctly FAIL today; there is nothing for it
% to compute. Writing a "symmetric to first_battery_below_time" formula
% now (dividing by a negative EffectiveRate to predict a future rise)
% would NOT be a safe no-op -- EffectiveRate < 0 under the CURRENT
% clamp means "flat, no drain this whole walk", not "charging", so that
% formula would predict a Tcross the actual battery/3 fluent
% contradicts (still flat, never risen) -- a genuine correctness bug,
% not just unreachable code, the moment a Zb draw makes EffectiveRate
% negative. If/when battery/3's regression is extended to model real
% recharging, add clause 2 here using WHATEVER rate formula that
% extension introduces (mirroring how clause 2 of
% first_battery_below_time/first_battery_equal_time use Rate/sigma_
% battery today) -- CP/Duration/Zb/Rate are already threaded through,
% genuinely used (not wildcarded away), specifically so that clause can
% be added without changing this predicate's signature or any of its
% callers.
first_battery_over_time(CP,T0,Duration,B0,Zb,_Rate,Threshold,T0) :-
    B0 > Threshold.

% battery_at_leg(+T0,+Duration,+Zb,+B0,+Rate,+T,-Level): Level(T) during
% ONE moveto leg, as a function of T alone -- the EXACT SAME formula as
% battery/3's own do(startMoveto(...),S) clause above (leg_start_
% battery(T0,S,B0) resolved to a plain value, not re-derived from S),
% just without S, for use by holds_leg/10 further down (which only ever
% has CP/T0/Duration/Z/Zt/Zb/B0/Rate on hand -- the same flat signature
% every trigger_crossing_time/12 clause already receives, not a full
% situation term). MUST be kept in sync BY HAND with battery/3's own
% do(startMoveto(...),S) clause -- there is no single shared definition
% because that clause additionally resolves B0/Tool (then Rate, via
% tool_moving_drain_rate/2) from S via leg_start_battery/3 and hitch/2,
% which this one takes already-resolved.
battery_at_leg(T0,Duration,Zb,B0,Rate,T,Level) :-
    Elapsed0 is T - T0,
    Elapsed is max(0.0, min(Elapsed0, Duration)),
    sigma_battery(SigmaB),
    moving_phase_deviation(Zb, SigmaB, Elapsed, Duration, Deviation),
    NominalDrain is Rate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, B0 - TotalDrain)).

% first_tool_battery_depletion_time/6, first_tool_battery_below_time/7,
% first_tool_battery_equal_time/7, first_tool_battery_over_time/7 USED
% to live here -- install_tool/uninstall_tool's own twins of first_
% battery_depletion_time/first_battery_below_time/first_battery_equal_
% time/first_battery_over_time above, differing only by NOT threading
% an (unused) CP parameter through. Removed once trigger_crossing_
% time/13's own battery clauses (above -- Mode-wildcarded, since Rate
% is what actually varies, not Mode) started calling the CP-taking
% originals directly, passing whatever placeholder CP the caller
% supplies (unused inside either version -- see first_battery_
% depletion_time's own note above on why CP was always dead weight
% there).

% tool_battery_at_leg/7 USED to live here -- an install_tool/
% uninstall_tool-only twin of battery_at_leg/7 above, differing only in
% variable names, not in signature or body (both take Rate directly by
% this point). Removed as dead code once holds_leg/11's own Mode=1
% battery clauses (see that predicate's own note) started calling
% battery_at_leg/7 directly instead -- the ONLY thing that used to call
% tool_battery_at_leg/7 was tool_holds_leg's own battery clauses,
% themselves removed for the same reason (see the note above tool_
% trigger_crossing_time(guard_break(...)) further down).

% ---------------------------------------------------------------
% TRIGGERS -- the TEMPLATE mechanism. A leg's Triggers argument is
% the COMPLETE list of halting conditions this leg reacts to --
% collision, battery depletion, obstacle sighting, and any future
% condition are ALL ordinary entries here, on identical footing.
% There is NO hardcoded always-on cause anymore: Triggers=[] means
% the walk halts ONLY on natural completion of its nominal duration
% -- it will pass straight through an obstacle's safety margin, or
% run the battery to empty and beyond, without halting for either,
% if neither `collision` nor `battery` is in its own Triggers list.
% Each recognized trigger name has ONE clause of
% trigger_crossing_time/9 below, giving its own crossing-time AND
% Reason -- the "standard interface" every trigger type must supply:
% given the walk's parameters and the resolved noise, produce a
% (Reason, crossing time) pair, or fail if it never fires. Adding a
% new trigger type later means adding ONE more clause here; nothing
% else in this file needs to change.
%
% Reason is a SEPARATE output from the trigger name itself for
% collision/battery specifically, so the Reason atoms every other
% part of the theory already keys on (crashed(ObstacleId),
% battery_depleted -- via halted_with/2, crashed_in/1,
% battery_depleted_in/1, first_hit/1, hit_by/1, etc.) stay STRUCTURALLY
% as they were; only how those Reasons get triggered changed, not what
% they're called. crashed carries WHICH obstacle (crashed(ObstacleId))
% rather than being a bare atom, since first_collision_time/6 now
% returns the crossed obstacle's own obstacle_polygon/2 Id alongside
% Tcross -- see collision_geometry.py's own header for where that
% argmin actually happens. battery_depleted stays a bare atom: draining
% isn't tied to any specific obstacle. Matching "any crash regardless
% of which obstacle" now needs crashed(_), not bare crashed -- see the
% TODO note near holds(halted_with_cond(...)) for the one place this is
% user-facing.
%
% collision (fixed threshold=0.0 against the ALREADY safety_margin-
% inflated obstacle_polygon/2 geometry -- a bare contact/containment
% test, see the MAP-PREPROCESSING INFLATION note above
% clearance_adjusted_threshold/2) and battery (fixed threshold=exactly-0,
% Reason=battery_depleted) are the ORIGINAL, UNPARAMETRIZED trigger
% names -- kept exactly as they were, on request, rather than folded
% into the generic versions below. obstacle_in_bound(Threshold) and
% battery_below(Threshold) are GENUINELY SEPARATE, ADDITIONAL trigger
% names -- a leg can react to EITHER or BOTH of a fixed floor and an
% arbitrary per-call threshold at once, e.g.
% Triggers=[collision,battery,battery_below(20)] halts on whichever of
% "hits an obstacle", "hits exactly empty", or "drops under 20%"
% happens earliest. obstacle_in_bound is what "obstacle sighted" was
% renamed to (see the note above first_threshold_crossing_time below
% for why sight_threshold/1 is gone): it reuses
% first_threshold_crossing_time DIRECTLY, with Threshold now the
% CALLER'S OWN argument (still meant as "distance from the REAL,
% uninflated obstacle surface" -- clearance_adjusted_threshold/2
% corrects for the polygon's own inflation before the call, see that
% predicate's own note) instead of a fixed config constant -- the exact
% same black box collision already used, no new machinery. Reason
% carries Threshold too, not just ObstacleId (unlike collision's
% crashed(ObstacleId)) -- so two obstacle_in_bound(...) triggers at
% different thresholds in the same Triggers list stay distinguishable
% by which one actually fired. battery_below(Threshold) is the SAME
% relationship to battery: reuses first_battery_below_time (the
% Threshold-generalized twin of first_battery_depletion_time, see that
% predicate's own note), Reason battery_under(Threshold) -- a
% DIFFERENT word from the trigger name, by request, mirroring
% collision/crashed's own asymmetry.
%
% battery_equal(Threshold) and battery_over(Threshold) are two more
% additional, SEPARATE battery triggers, same family as battery_below.
% battery_equal(Threshold) fires at the exact instant Level(T)=Threshold
% (no "already below" grace -- see first_battery_equal_time's own
% note); Reason keeps the SAME functor as the trigger (battery_equal),
% not a distinct word, since no distinct Reason was requested for these
% two (unlike battery_below/battery_under). battery_over(Threshold)
% currently only ever fires "already true at T0" or never, since
% battery never increases within a walk TODAY -- but its predicate is
% structured with the SAME two-clause shape as battery_below/
% battery_equal, ready for a genuine future-crossing search once
% battery/3 models real recharging -- see first_battery_over_time's
% own note for exactly why a shortcut formula can't safely be
% written in ahead of that.
%
% obstacle_on_path(Threshold) is a DIFFERENT geometric test from
% obstacle_in_bound(Threshold), not another threshold value for the
% same one: obstacle_in_bound asks "is the CURRENT position close to
% ANY obstacle's BOUNDARY", regardless of whether the trajectory ever
% actually enters that obstacle (a path can graze within safety_margin
% of an obstacle's edge -- e.g. because safety_margin already includes
% the robot's own radius -- without the robot's own center ever being
% geometrically INSIDE the polygon). obstacle_on_path instead asks "is
% the CURRENT position close to an obstacle the trajectory ACTUALLY
% ENTERS (goes geometrically inside, not just near) somewhere across
% this WHOLE walk" -- confirmed by direct test to genuinely differ:
% at Z=-1.0 and Z=+2.0 in the real 12-obstacle map, the trajectory
% comes within safety_margin of an obstacle's boundary (obstacle_in_bound
% WOULD fire) but never actually enters it (obstacle_on_path does NOT).
% Reuses first_threshold_crossing_time/obstacle_within_threshold
% UNCHANGED, just called (inside collision_geometry.py) against a
% obstacle set FILTERED to "obstacles this trajectory enters somewhere"
% -- see collision_geometry.py's own header, "ON PATH" section.
%
% obstacle_in_bound(Threshold), obstacle_on_path(Threshold),
% battery_below(Threshold), battery_equal(Threshold), and
% battery_over(Threshold) are ALSO directly usable as cond() leaves,
% checking the CURRENT situation instead of searching a future walk --
% see holds(obstacle_in_bound(...)), holds(obstacle_on_path(...)),
% holds(battery_below(...)), holds(battery_equal(...)), and
% holds(battery_over(...)) further down, which reuse the exact same
% underlying primitives (obstacle_within_threshold/
% obstacle_on_path_within_threshold/battery/3) a single time instead of
% across a bracket-scanned trajectory.
%
% line_of_sight_clear(ObstacleId,GX,GY) and crosses_segment(SX,SY,GX,GY)
% are the Bug-algorithm boundary-LEAVE triggers, for use on a MoveTo leg
% whose ControlPoints came from planners.py's follow_boarder
% (ObstacleId,Offset) planner (a full clockwise loop around the
% obstacle's offset boundary, no stopping logic of its own -- see that
% predicate's own header). Which bug variant a leg implements is
% entirely a matter of WHICH of these two names its own Triggers list
% carries -- line_of_sight_clear for Bug0 (fires as soon as ObstacleId
% stops occluding a straight line to (GX,GY)), crosses_segment for
% Bug2 (fires when the boundary walk re-crosses the straight segment
% from (SX,SY) -- wherever the leg's own circling began -- to (GX,GY),
% at a point strictly closer to goal than (SX,SY) was; see
% collision_geometry.py's own "BUG-ALGORITHM BOUNDARY-LEAVE PRIMITIVES"
% section for exactly why that distance condition is part of the
% definition). line_of_sight_clear(ObstacleId,GX,GY) is ALSO usable as
% a cond() leaf (holds(line_of_sight_clear(...)) further down);
% crosses_segment deliberately is NOT -- see collision_geometry.py's
% own note on why "has my trajectory crossed this segment" doesn't
% have a meaningful point-in-time reading the way the others do.
% ---------------------------------------------------------------
% trigger_crossing_time/11's LAST argument, Code, is the REDESCEND-
% TARGET CODE this specific trigger OCCURRENCE was tagged with by
% bt_to_prolog.py at translation time -- see the CONTROL-FLOW
% REDESCEND TARGETS note above do_node(reactivesequence(...)) further
% down for the full mechanism. collision/battery (never reactive --
% they classify straight to false in leg_status, never to a
% reactive(Code) status) always produce Code=none, since nothing ever
% reads it for them. Every genuinely reactive-classified trigger name
% below (obstacle_in_bound, obstacle_on_path, battery_below,
% battery_equal, battery_over, line_of_sight_clear, crosses_segment)
% now takes Code as an EXTRA trailing argument in the Triggers list
% ITSELF (e.g. battery_below(70,rc3), not battery_below(70)) -- bt_to_
% prolog.py appends it automatically to every reactive trigger token,
% based on which ReactiveSequence/ReactiveFallback (if any) currently
% encloses that leg, so nothing about how a BT.xml author writes
% triggers="..." needs to change; Code just rides along as a plain
% INPUT here, unify-passed straight through to the OUTPUT Reason-Time
% pair's own third slot (Reason's own shape, e.g. battery_under
% (Threshold), is UNCHANGED -- Code is carried ALONGSIDE it, not
% embedded inside it, so halted_with/2 and everything built on it
% keeps working exactly as before).
% Mode (0 = MoveTo, 1 = install_tool/uninstall_tool -- SHARED with
% holds_leg/11's own Mode, see that predicate's own note) restricts
% these clauses the SAME way it restricts holds_leg/11's own motion
% clauses: collision/obstacle_in_bound/obstacle_on_path/line_of_sight_
% clear/crosses_segment below all pattern-match Mode=0 directly IN THE
% HEAD, so under Mode=1 none of them unify at all -- CP is NEVER
% touched, and all_trigger_candidates/11's own existing "no matching
% clause, silently skip" convention (see its own note below) handles
% the rest with NO new code, exactly the same way it already handles a
% genuinely unrecognized trigger name. Unlike guard_break further down,
% these clauses need NO extra outer gate the way tool_cond_supported/1
% guards guard_break -- there is no first_becomes_false_time-style
% "absence at T0 read as already-false" ambiguity here: all_trigger_
% candidates' own two clauses are a plain "did this trigger fire, or
% not" dispatch, so a Mode-blocked clause and a genuinely-inapplicable
% trigger name look identical to it, both correctly contributing
% nothing. battery/battery_below/battery_equal/battery_over wildcard
% Mode away entirely -- both MoveTo and install_tool/uninstall_tool
% legitimately want battery triggers evaluated the same way.
trigger_crossing_time(collision, CP,T0,Duration,Z,Zt,_Zb,_B0,_Rate,0, crashed(ObstacleId), Tcross, none) :-
    first_collision_time(CP,T0,Duration,Z,Zt,Tcross,ObstacleId).

trigger_crossing_time(battery, CP,T0,Duration,_Z,_Zt,Zb,B0,Rate,_Mode, battery_depleted, Tcross, none) :-
    first_battery_depletion_time(CP,T0,Duration,B0,Zb,Rate,Tcross).

trigger_crossing_time(obstacle_in_bound(Threshold,Code), CP,T0,Duration,Z,Zt,_Zb,_B0,_Rate,0, obstacle_in_bound(Threshold,ObstacleId), Tcross, Code) :-
    clearance_adjusted_threshold(Threshold, AdjThreshold),
    first_threshold_crossing_time(CP,T0,Duration,Z,Zt,AdjThreshold,Tcross,ObstacleId).

trigger_crossing_time(obstacle_on_path(Threshold,Code), CP,T0,Duration,Z,Zt,_Zb,_B0,_Rate,0, obstacle_on_path(Threshold,ObstacleId), Tcross, Code) :-
    clearance_adjusted_threshold(Threshold, AdjThreshold),
    first_on_path_crossing_time(CP,T0,Duration,Z,Zt,AdjThreshold,Tcross,ObstacleId).

trigger_crossing_time(battery_below(Threshold,Code), CP,T0,Duration,_Z,_Zt,Zb,B0,Rate,_Mode, battery_under(Threshold), Tcross, Code) :-
    first_battery_below_time(CP,T0,Duration,B0,Zb,Rate,Threshold,Tcross).

trigger_crossing_time(battery_equal(Threshold,Code), CP,T0,Duration,_Z,_Zt,Zb,B0,Rate,_Mode, battery_equal(Threshold), Tcross, Code) :-
    first_battery_equal_time(CP,T0,Duration,B0,Zb,Rate,Threshold,Tcross).

trigger_crossing_time(battery_over(Threshold,Code), CP,T0,Duration,_Z,_Zt,Zb,B0,Rate,_Mode, battery_over(Threshold), Tcross, Code) :-
    first_battery_over_time(CP,T0,Duration,B0,Zb,Rate,Threshold,Tcross).

trigger_crossing_time(line_of_sight_clear(ObstacleId,GX,GY,Code), CP,T0,Duration,Z,Zt,_Zb,_B0,_Rate,0, line_of_sight_clear(ObstacleId,GX,GY), Tcross, Code) :-
    first_line_of_sight_clear_time(CP,T0,Duration,Z,Zt,ObstacleId,GX,GY,Tcross).

trigger_crossing_time(crosses_segment(SX,SY,GX,GY,Code), CP,T0,Duration,Z,Zt,_Zb,_B0,_Rate,0, crosses_segment(SX,SY,GX,GY), Tcross, Code) :-
    first_segment_crossing_time(CP,T0,Duration,Z,Zt,SX,SY,GX,GY,Tcross).

% guard_break(Cond,Code): the GENERIC, AUTOMATICALLY-DERIVED trigger
% bt_to_prolog.py's guard-derivation pass emits for a MoveTo (Mode=0)
% or an InstallTool/UninstallTool (Mode=1) sitting under a
% ReactiveSequence/ReactiveFallback ancestor with a Condition left
% sibling (see that file's own CONTROL-FLOW GUARD DERIVATION note) --
% Cond is ALREADY the exact term that must stay TRUE for the guard to
% keep holding: the bare condition itself for a ReactiveSequence-style
% "left siblings must all SUCCEED" guard, or neg(Condition) for a
% ReactiveFallback-style "left siblings must all FAIL" guard (bt_to_
% prolog.py builds this by NEGATING the actual condition per the
% required polarity -- and per any <Inverter> in the chain -- rather
% than by looking up a pre-built "opposite" trigger name, so this works
% uniformly for every condition in schema.yaml's conditions: list with
% no per-condition crossing-direction table to keep in sync or leave a
% gap in). Fires at the first instant Cond stops holding -- see first_
% becomes_false_time/11 and holds_leg/11 further down (right after
% holds/2's own condition clauses, which holds_leg/11 mirrors).
%
% TWO clauses, split on Mode, NOT one clause forwarding Mode straight
% through -- Mode=1 needs the SAME tool_cond_supported/1 gate this
% predicate's install_tool/uninstall_tool analogue used to carry on its
% own separate guard_break clause (see that predicate's own note above
% for exactly why: first_becomes_false_time's own first clause reads
% "no matching holds_leg/11 clause" as "already false at T0", so a
% Mode=1-blocked motion Cond needs to be intercepted BEFORE ever
% reaching first_becomes_false_time, not just have its holds_leg call
% silently fail). Mode=0 needs no such gate -- holds_leg/11's own Mode=0
% clause set already covers every functor _reduce_guard_condition can
% ever emit (see holds_leg/11's own note), so there is no "unsupported
% Cond" case for MoveTo to guard against.
trigger_crossing_time(guard_break(Cond,Code), CP,T0,Duration,Z,Zt,Zb,B0,Rate,0, guard_break(Cond), Tcross, Code) :-
    first_becomes_false_time(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,0,Tcross).
trigger_crossing_time(guard_break(Cond,Code), CP,T0,Duration,Z,Zt,Zb,B0,Rate,1, guard_break(Cond), Tcross, Code) :-
    tool_cond_supported(Cond),
    first_becomes_false_time(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,1,Tcross).

% all_trigger_candidates(+Triggers,...,-Candidates): Candidates is a
% list of Reason-Time-Code TRIPLES now (was Reason-Time pairs), one
% per trigger in Triggers that ACTUALLY fires in this resolved world
% (triggers that don't fire contribute nothing -- same "absence, not
% sentinel" convention as everywhere else). Unrecognized trigger names,
% AND every Mode=0-only clause when Mode=1 (see trigger_crossing_time/
% 13's own note above), are silently skipped the SAME way -- no
% matching clause at all -- lenient by design, so a typo in a Triggers
% list doesn't halt the whole theory, just means that trigger never
% contributes. Rate -- the battery drain rate for whichever consumer is
% calling in (moving-phase for MoveTo via tool_moving_drain_rate(Tool,
% Rate), install/uninstall-phase for install_tool/uninstall_tool via
% install_tool_drain_rate/1 or uninstall_tool_drain_rate/1 -- see
% earliest_halt/13's own note), already resolved to a plain number --
% rides alongside CP/Z/Zt/Zb/B0/Mode, resolved ONCE by earliest_halt/
% 13's own caller and passed straight through, same as every other
% leg-constant parameter here.
all_trigger_candidates([], _,_,_,_,_,_,_,_,_, []).
all_trigger_candidates([Trig|Rest], CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode, [Reason-Tcross-Code|RestCands]) :-
    trigger_crossing_time(Trig, CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode, Reason, Tcross, Code),
    all_trigger_candidates(Rest, CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode, RestCands).
all_trigger_candidates([Trig|Rest], CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode, RestCands) :-
    \+ trigger_crossing_time(Trig, CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode, _, _, _),
    all_trigger_candidates(Rest, CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode, RestCands).

% earliest_of(+TriplesList, -ReasonTimeCodeTriple): generic "minimum by
% SECOND element (Time)" over a non-empty list of Reason-Time-Code
% triples. Used to combine natural completion with however many
% Triggers-derived candidates happen to apply, into ONE earliest-wins
% choice -- this is the mechanism that makes moveto a genuine
% template: it works identically whether Triggers is empty, or
% contains one, or many conditions (including collision/battery
% themselves now), with no change needed here.
earliest_of([Triple], Triple).
earliest_of([R1-T1-C1|Rest], Result) :-
    earliest_of(Rest, _-T2-_),
    T1 =< T2,
    Result = R1-T1-C1.
earliest_of([R1-T1-C1|Rest], Result) :-
    earliest_of(Rest, R2-T2-C2),
    T1 > T2,
    Result = R2-T2-C2.

% ---------------------------------------------------------------
% TOOL TRIGGERS -- install_tool/uninstall_tool USED to have their own,
% separate RESTRICTED analogue of the TRIGGERS section above
% (tool_trigger_crossing_time/all_tool_trigger_candidates/tool_
% earliest_halt) -- REMOVED on request: trigger_crossing_time/13 and
% all_trigger_candidates/11 above, and earliest_halt/13 further down,
% are now the SINGLE SHARED "what happens first" engine for BOTH
% MoveTo (Mode=0) and install_tool/uninstall_tool (Mode=1), the exact
% same Mode mechanism that already let holds_leg/first_becomes_false_
% time/guard_bracket_scan/guard_bisect be shared. install_tool/
% uninstall_tool never move the robot, so a MOTION-based trigger
% (collision, obstacle_in_bound, obstacle_on_path, line_of_sight_
% clear, crosses_segment) has no meaningful geometry to check -- but
% (as with holds_leg's own motion clauses) that no longer needs a
% SEPARATE restricted predicate to enforce: trigger_crossing_time/13's
% own motion-based clauses pattern-match Mode=0 directly in their own
% heads (see that predicate's own note, back where it's defined), so
% under Mode=1 none of them unify, CP is never touched, and all_
% trigger_candidates/11's own EXISTING third clause -- "no matching
% trigger_crossing_time/13 clause, silently skip" -- treats that
% identically to "never fires", with NO new code needed for it (that
% third clause already existed for the unrelated case of a genuinely
% unrecognized trigger NAME, e.g. a typo -- a Mode-blocked trigger
% just falls into the exact same bucket). module/translators/bt_to_
% prolog.py's own _is_battery_trigger validation at translation time
% is still what actually stops a BT author from writing a motion
% trigger on <InstallTool>/<UninstallTool> in the first place -- this
% Mode-blocking is the theory-level defense-in-depth BEHIND that (same
% "hand-written plan_generated.pl bypassing the translator, same as
% problem3's own" scenario every such defense in this file guards
% against), not a substitute for it.
%
% guard_break(Cond,Code) is the ONE trigger_crossing_time/13 clause
% that DOES need an explicit outer gate under Mode=1 (tool_cond_
% supported/1, kept below) -- unlike the motion clauses above, it has
% no per-trigger-name matching to fall back on (a SINGLE clause covers
% every possible Cond shape), so a Mode=1-blocked motion Cond reaching
% it would make holds_leg/11 correctly have no matching clause, which
% first_becomes_false_time/11's own first clause (\+ holds_leg(...,
% T0)) would misread as "already false at T0" -- firing IMMEDIATELY
% instead of never. See trigger_crossing_time/13's own guard_break
% clauses (above, split by Mode) for exactly where this gate applies.
% Under NORMAL (translator-produced) operation this can never actually
% happen -- bt_to_prolog.py's own _guard_condition_is_battery_only
% validation already guarantees Cond is battery-only for install_tool/
% uninstall_tool before a plan is ever written -- so, same as the
% motion-trigger case above, this gate exists purely as defense-in-
% depth for a bypassed-validation plan, not because ordinary use can
% trigger it.
tool_cond_supported(and(P,Q)) :- tool_cond_supported(P), tool_cond_supported(Q).
tool_cond_supported(or(P,Q)) :- tool_cond_supported(P), tool_cond_supported(Q).
tool_cond_supported(neg(P)) :- tool_cond_supported(P).
tool_cond_supported(battery_below(_)).
tool_cond_supported(battery_equal(_)).
tool_cond_supported(battery_over(_)).

% tool_leg_status(+Reason,+Code,-Status): the install_tool/
% uninstall_tool/deploy_tool/retract_tool analogue of leg_status/9
% above -- but NOTE the difference from moveto's own rule: reaching
% "natural completion" is NOT itself a success (unlike moveto's
% Reason=completed->true) -- *_tool_success(Tool,_)/*_tool_failure
% (Tool,_) are BOTH possible outcomes of natural completion (the coin
% flip already resolved which one -- see poss(halt_install_tool(...))
% below), so THIS predicate
% only ever sees the ALREADY-DECIDED Reason, never the completed
% sentinel itself (see earliest_halt/13's own WARNING on why that
% shared spelling with MoveTo's own completed does NOT share its
% meaning). battery_depleted is the one hard, non-reactive
% failure (mirroring battery_depleted's own classification in
% leg_status/9); every OTHER battery-related trigger (battery_under/
% equal/over(Threshold)) reactive-classifies, same convention.
tool_leg_status(install_tool_success(_Tool), _Code, true).
tool_leg_status(uninstall_tool_success(_Tool), _Code, true).
tool_leg_status(install_tool_failure(_Tool), _Code, false).
tool_leg_status(uninstall_tool_failure(_Tool), _Code, false).
tool_leg_status(deploy_tool_success(_Tool), _Code, true).
tool_leg_status(retract_tool_success(_Tool), _Code, true).
tool_leg_status(deploy_tool_failure(_Tool), _Code, false).
tool_leg_status(retract_tool_failure(_Tool), _Code, false).
tool_leg_status(battery_depleted, _Code, false).
tool_leg_status(Reason, Code, reactive(Code)) :-
    Reason \= install_tool_success(_), Reason \= uninstall_tool_success(_),
    Reason \= install_tool_failure(_), Reason \= uninstall_tool_failure(_),
    Reason \= deploy_tool_success(_), Reason \= retract_tool_success(_),
    Reason \= deploy_tool_failure(_), Reason \= retract_tool_failure(_),
    Reason \= battery_depleted.

% walk_noisy_point(+CP,+T0,+Duration,+Z,+Zt,+T,-X,-Y): position along
% the spline at time T, given TWO ALREADY-RESOLVED, INDEPENDENT noise
% draws (rather than looking them up via z/2 or zt/2 itself): Z (lateral
% /normal drift) and Zt (tangential/along-path drift -- a straight
% metric push along the spline's own tangent direction at each point,
% NOT a reparametrization of Frac; see collision_geometry.py's own
% _walk_noisy_point for the reasoning behind Option B, a metric offset,
% over shifting Frac itself). NO Prolog clause of this name exists in
% this file -- this call resolves DIRECTLY against the black-box
% walk_noisy_point predicate collision_geometry.py registers (see
% Section 0's own :- use_module directive, and Section 3 above for the
% full "why this moved" rationale), the SAME way
% first_threshold_crossing_time/8 below already does for the
% obstacle-clearance search. Used here so the same formula is reused by
% first_collision_time's bracket/bisection search below and by at/4,
% without re-deriving Z/Zt through a different situation, and reused
% (at Z=Zt=0) by spline_point/4 above -- one formula, one place, both
% the noisy and the nominal case.

% ---------------------------------------------------------------
% FIRST-THRESHOLD-CROSSING-TIME -- a NATURAL (not chosen) event: the
% earliest time, within a given resolved world (fixed Z), at which
% the noisy trajectory comes within a given distance THRESHOLD of an
% (already safety_margin-inflated, see the MAP-PREPROCESSING INFLATION
% note above clearance_adjusted_threshold/2) obstacle. GENERALIZED over
% the threshold so the SAME machinery serves collision (threshold=0.0,
% via first_collision_time/6 below -- a bare contact test against the
% inflated polygon), obstacle_in_bound(Threshold) (called with the
% caller's own Threshold, corrected for the inflation via
% clearance_adjusted_threshold/2 -- see trigger_crossing_time/9 above
% and holds(obstacle_in_bound(...)) below, no separate wrapper
% predicate needed since this black box was already threshold-generic),
% and any future distance-based trigger.
%
% first_threshold_crossing_time(+ControlPoints,+T0,+Duration,+Z,+Zt,
% +Threshold,-Tcross,-ObstacleId) is now a BLACK-BOX Python predicate,
% registered by collision_geometry.py's own :- use_module(...)
% directive (see below) -- exactly the same "deliberately NOT part of
% the situation-calculus machinery" reasoning already used for
% planWith/plan_call: this is a deterministic, stateless computation
% over an ALREADY-RESOLVED noise value Z, not a probabilistic choice in
% itself, so there is no frame problem here to justify keeping it in
% Prolog. ObstacleId is the crossed obstacle's own obstacle_polygon/2
% Id (an argmin over obstacles at the exact crossing point, not just
% the crossing time itself). bracket_samples/1 and crossing_eps/1 --
% the bracket-scan count and bisection tolerance -- are now config
% facts too (see the problem's own config.yaml's
% verification.bracket_samples/crossing_eps). collision_geometry.py
% reads them (and sigma/1's value) directly out of the problem's own config.yaml
% itself, not out of this generated fact, since config.yaml is the
% actual single source of truth both sides are driven from -- see
% collision_geometry.py's own header. FAILS (0 ProbLog solutions) if
% the trajectory never comes within Threshold of an obstacle in this
% resolved world -- correctly representing "never happens" via
% absence, not a sentinel value, same convention as everywhere else in
% this theory.

% first_collision_time/6 kept as a thin, name-preserving wrapper over
% the generalized machinery, now at threshold=0.0 (obstacle_polygon/2
% is already inflated by safety_margin -- see the MAP-PREPROCESSING
% INFLATION note above clearance_adjusted_threshold/2 -- so "within
% safety_margin of the real obstacle" is now just "on or inside the
% obstacle" against the already-inflated polygon) -- every EXISTING
% caller (crashed_in, verify_safe, etc.) is unaffected beyond the new
% ObstacleId output.
first_collision_time(CP,T0,Duration,Z,Zt,Tcross,ObstacleId) :-
    first_threshold_crossing_time(CP,T0,Duration,Z,Zt,0.0,Tcross,ObstacleId).

% ---------------------------------------------------------------
% Poss AXIOMS for the primitive actions.
% ---------------------------------------------------------------
% NOTE: startMoveto deliberately does NOT check battery > 0 (or
% "not already colliding", or "not already within some obstacle_in_bound Threshold")
% as a precondition. Doing so would make do_action FAIL ENTIRELY when
% the battery is already empty (or the robot already unsafe) --
% classical Golog non-derivability, i.e. "no situation exists" -- the
% wrong semantics for a BT-style outcome (see the do_node/outcome
% discussion earlier). Instead, ALL of collision, battery depletion,
% obstacle_in_bound, and battery_below already have a graceful "already
% true at T0" case built into first_threshold_crossing_time /
% first_battery_depletion_time / first_battery_below_time (Tcross = T0
% exactly), so starting a walk with an
% empty battery -- or already inside an obstacle's margin -- still
% produces a well-formed, immediately-halted situation with the
% correct Reason, consistent with every other halting cause, rather
% than a special-cased blocking precondition for battery alone.
% T0 is rounded UP to disc_step_time/1's own granularity (see the
% MERGE-GRID QUANTIZATION note above dist/5's own section) -- the
% previous leg's own recorded halt instant (embedded in ITS OWN
% haltMoveto/interrupt term, read by REPORTING queries) stays exact;
% only the value THIS new leg treats as its own start time is
% coarsened, and CEILING (never floor/round) guarantees a new leg can
% never appear to start before the previous one actually ended.
poss(startMoveto(_,_Triggers,_ActionCode,T0), S) :-
    \+ moving(S),
    now(T0Exact, S),
    disc_step_time(Grid),
    quantize_up(T0Exact, Grid, T0).

% now(-T,+S): current wall-clock time -- needed above only to know
% WHEN to check the battery level (battery/3 needs a query time).
% Situation argument S is LAST, per Reiter's own convention (see
% module/contracts/vocabulary.yaml's own note -- this used to be
% now(S,T), the one remaining exception flagged when the other six
% out-of-convention accessors were fixed; fixed here too, along with
% every one of its own call sites throughout this file).
now(0, s0).
now(T, do(startMoveto(_,_,_,T),_)).
now(T, do(haltMoveto(T,_,_),_)).
now(T, do(interrupt(T),_)).
% start_install_tool/halt_install_tool (and the uninstall pair) ALSO
% genuinely advance the clock -- start_install_tool(...) takes real
% time to REACH (T0 is when the install itself begins, exactly
% mirroring startMoveto's own T0), and halt_install_tool(...) reports
% the instant it ends, exactly mirroring haltMoveto's own T. UNLIKE
% at/4/moving/1/current_walk/6 (whose own generic pass-through clauses
% already handle these four new action functors correctly with no
% changes at all) and battery/3 (which DOES need its own new anchor
% clauses for these four functors -- see do_node(install_tool_leg
% (...))'s own note), now/2 would be WRONG without these: its own
% generic pass-through simply reuses whatever "now" already was,
% which is only correct for a bookkeeping MARKER with no clock effect
% of its own (checked/planned/sampled), not for an action that
% GENUINELY advances time.
now(T0, do(start_install_tool(_,_,_,T0),_)).
now(T, do(halt_install_tool(T,_,_),_)).
now(T0, do(start_uninstall_tool(_,_,_,T0),_)).
now(T, do(halt_uninstall_tool(T,_,_),_)).
% start_deploy_tool/halt_deploy_tool/start_retract_tool/halt_retract_
% tool: SAME genuine-clock-advance shape as the install/uninstall pair
% just above -- deploying/retracting is a fixed-Duration action too.
now(T0, do(start_deploy_tool(_,_,_,T0),_)).
now(T, do(halt_deploy_tool(T,_,_),_)).
now(T0, do(start_retract_tool(_,_,_,T0),_)).
now(T, do(halt_retract_tool(T,_,_),_)).
now(T, do(A,S)) :-
    A \= startMoveto(_,_,_,_), A \= haltMoveto(_,_,_), A \= interrupt(_),
    A \= start_install_tool(_,_,_,_), A \= halt_install_tool(_,_,_),
    A \= start_uninstall_tool(_,_,_,_), A \= halt_uninstall_tool(_,_,_),
    A \= start_deploy_tool(_,_,_,_), A \= halt_deploy_tool(_,_,_),
    A \= start_retract_tool(_,_,_,_), A \= halt_retract_tool(_,_,_),
    now(T, S).

% haltMoveto(T,Reason): the ways a walk stops other than an interrupt.
% ALL are NATURAL events, not choices -- T/Reason are DERIVED, never
% chosen by the plan. Whichever candidate cause -- natural completion,
% or any condition in this leg's own Triggers list (collision, battery,
% obstacle_in_bound(Threshold), battery_below(Threshold), or a future
% one) -- occurs EARLIEST in
% this resolved world wins. This is the genuine TEMPLATE mechanism:
% NOTHING is hardcoded here -- a leg with Triggers=[] halts ONLY on
% natural completion, passing straight through an obstacle's margin
% or running the battery dry without ever noticing, if collision/
% battery aren't in its own Triggers list.
%
% earliest_halt/12 is the SINGLE SHARED definition of "what happens
% first" -- used here, by Poss(interrupt(...)) below, AND by
% verify_safe further down (called there with Z=0.0,Zt=0.0,Zb=0.0
% instead of the resolved noise). Having exactly ONE definition, rather
% than the same computation duplicated at each call site, is what
% guarantees every query stays consistent with what Poss(haltMoveto(...))
% itself actually derives -- see the "SAFETY QUERIES READ THE ACTUAL
% OUTCOME" note further down for why this matters. Code is the winning
% candidate's own redescend-target code (none for natural completion/
% collision/battery, since those never classify to reactive(_) --
% see leg_status/9 further down and the CONTROL-FLOW REDESCEND TARGETS
% note above do_node(reactivesequence(...)) further down). Rate is the
% battery drain rate for whichever consumer is calling in, already
% resolved to a plain number by this predicate's own FIVE callers --
% poss(haltMoveto(...))/poss(interrupt(...))/verify_safe (MoveTo,
% effective_tool_moving_drain_rate(Tool,S,Rate), Mode=0) and poss(halt_install_
% tool(...))/poss(halt_uninstall_tool(...)) (install_tool_drain_rate/1
% or uninstall_tool_drain_rate/1, Mode=1) -- it rides alongside CP/Z/
% Zt/Zb/B0/Mode into all_trigger_candidates/11, same as every other
% leg-constant parameter; it does NOT affect NaturalEnd here (Duration
% itself already reflects whatever Speed/no-Speed applies, resolved
% separately by this predicate's own caller before Duration ever
% reaches here). Mode is likewise just forwarded -- see trigger_
% crossing_time/13's and holds_leg/11's own notes for what it means.
% This predicate is now the SINGLE SHARED "what happens first" engine
% for BOTH MoveTo and install_tool/uninstall_tool -- see the module's
% own note above tool_cond_supported/1 for why install_tool/uninstall_
% tool's own trigger dispatch (formerly tool_earliest_halt/all_tool_
% trigger_candidates/tool_trigger_crossing_time, all removed) could be
% folded in here with no CP-tolerance work needed, the same reasoning
% that already let holds_leg/first_becomes_false_time/guard_bracket_
% scan/guard_bisect be shared.
%
% WARNING -- SAME SENTINEL ATOM, NOT THE SAME MEANING ACROSS Mode: for
% MoveTo (Mode=0), Reason=completed IS the final answer -- leg_status/9
% maps it straight to Status=true, since reaching a walk's own natural
% end already means success, nothing else to decide. For install_tool/
% uninstall_tool (Mode=1), Reason=completed here is NOT a final answer
% at all -- it is a pure SENTINEL meaning "reached the full Duration
% with nothing halting it early", consumed immediately by poss(halt_
% install_tool(...))/poss(halt_uninstall_tool(...)) (which match on it
% exactly, in their own first clause), turning THAT specific outcome
% into a SEPARATE success-or-failure coin flip (install_tool_result/3/
% uninstall_tool_result/3) BEFORE Status is ever decided. This atom's
% own value NEVER reaches tool_leg_status/3 or gets recorded as a real
% Reason for install_tool/uninstall_tool -- it's discarded in favor of
% whatever Reason0 the coin flip produces the moment it's matched. Do
% not assume "Reason=completed" means "succeeded" under Mode=1 the way
% it does under Mode=0 -- see poss(halt_install_tool(...))'s own note
% for why the two spellings were deliberately merged anyway.
earliest_halt(CP,Triggers,T0,Duration,Z,Zt,Zb,B0,Rate,Mode, Reason,T,Code) :-
    all_trigger_candidates(Triggers, CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode, ExtraCandidates),
    NaturalEnd is T0 + Duration,
    earliest_of([completed-NaturalEnd-none], ExtraCandidates, Reason-T-Code).

% poss(haltMoveto(...)) is the ONE place a leg's own ActionCode
% (read off the SAME startMoveto term current_walk/6 already resolves
% CP/Triggers/T0/SPrev from -- see that predicate's own note) gets
% baked into the RECORDED Reason, via tag_reason/3 further down --
% leg_status/9 itself still decides Status from the UNTAGGED Reason0
% (it needs to recognize completed/crashed(_)/battery_depleted in
% their ORIGINAL shapes), so tagging happens strictly AFTER Status is
% already settled, on the value that actually gets written into S1's
% own haltMoveto term.
poss(haltMoveto(T, Reason, Status), S) :-
    moving(S),
    current_walk(S, CP, Triggers, ActionCode, T0, SPrev),
    hitch(Tool, SPrev),
    walk_duration(CP, Tool, SPrev, Duration),
    effective_tool_moving_drain_rate(Tool, SPrev, Rate),
    z(do(startMoveto(CP,Triggers,ActionCode,T0),SPrev), Z),
    zt(do(startMoveto(CP,Triggers,ActionCode,T0),SPrev), Zt),
    zbatt(Zb),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(CP,Triggers,T0,Duration,Z,Zt,Zb,B0,Rate,0, Reason0,T,Code),
    leg_status(Reason0, CP, T0, Duration, Z, Zt, T, Code, Status),
    tag_reason(Reason0, ActionCode, Reason).

% tag_reason(+Reason0, +ActionCode, -Reason): appends ActionCode as
% Reason0's own new TRAILING argument -- crashed(ObstacleId) becomes
% crashed(ObstacleId,ActionCode), completed becomes completed
% (ActionCode), guard_break(Cond) becomes guard_break(Cond,ActionCode),
% and so on for every Reason shape trigger_crossing_time/11 or
% earliest_halt/11 can ever produce -- GENERICALLY, via univ (=..),
% rather than one clause per Reason functor (the same "one generic
% mechanism instead of a per-case table" choice as holds_leg/9's own
% design). This is what lets a safety query check
% halted_with_cond(crashed(Obst1,ActionCode)) -- Obst1 unbound to match
% ANY obstacle, or bound to ask about a specific one, and ActionCode
% likewise -- to distinguish WHICH MoveTo occurrence in the tree
% produced a given halt, using the ordinary cond()/halted_with_cond
% machinery, no new query predicate needed.
tag_reason(Reason0, ActionCode, Reason) :-
    Reason0 =.. [Functor|Args0],
    append(Args0, [ActionCode], Args),
    Reason =.. [Functor|Args].

% leg_target(+ControlPoints, -GX,-GY): a leg's own intended endpoint
% is the LAST point in its OWN control_points list -- NOT necessarily
% the same as the global goal/2 fact, since a leg inside a future
% multi-leg sequence is trying to reach ITS OWN waypoint, not
% necessarily the overall plan's final destination. This is what lets
% status/leg_status generalize correctly to multi-leg plans without
% change: each leg's own success is about achieving its own target.
leg_target(ControlPoints, GX,GY) :-
    last_element(ControlPoints, point(GX,GY)).

last_element([X], X).
last_element([_|T], X) :- T \= [], last_element(T, X).

% leg_status(+Reason,+CP,+T0,+Duration,+Z,+Zt,+T,+Code,-Status): THREE
% possible SHAPES of output now -- true/false are plain Prolog ATOMS
% (there is no built-in boolean type restricting this to two values),
% and the third is now the COMPOUND term reactive(Code), not the bare
% atom reactive -- Code (threaded straight through from earliest_halt/
% 11's own Code output, itself read off the WINNING trigger's own
% Triggers-list occurrence -- see trigger_crossing_time/11's own note)
% is what lets do_node(reactivesequence(...))/do_node(reactivefallback
% (...)) further down decide whether THEY are the redescend target for
% THIS particular reactive halt, instead of every enclosing composite
% unconditionally redescending the way evaluate_plan/4 used to.
%
%   true         -- Reason=completed: the walk ran its full nominal
%                duration without any Trigger cutting it short. Says
%                NOTHING about whether the actual (noisy) final
%                position also landed close enough to the leg's own
%                endpoint -- that used to be baked in here via
%                goal_tolerance/1, but is now its OWN explicit,
%                inspectable BT condition (distance_below/3), hand-
%                placed by the tree author right after a MoveTo in a
%                Sequence wherever that check is wanted, same as any
%                other cond() leaf, rather than something silently
%                folded into MoveTo's own Status.
%   false        -- a genuine, unrecoverable failure: crashed
%                (ObstacleId) or battery_depleted. Nothing downstream
%                can react to these and continue; the leg (and, per
%                Sequence/Fallback's own do_node rules, quite possibly
%                the whole plan) is simply done.
%   reactive(Code) -- every OTHER trigger (obstacle_in_bound(...),
%                obstacle_on_path(...), battery_under/equal/over(...),
%                line_of_sight_clear(...), crosses_segment(...), and
%                any future trigger name not explicitly listed as a
%                hard failure above): the walk was cut short by a
%                condition that's meant to be REACTED to, not treated
%                as outright success or failure -- see the CONTROL-
%                FLOW REDESCEND TARGETS note above do_node(reactive
%                sequence(...)) further down for what happens when a
%                do_node/4 call anywhere in the tree returns this.
leg_status(completed, _CP, _T0, _Duration, _Z, _Zt, _T, _Code, true).
leg_status(crashed(_), _,_,_,_,_,_, _Code, false).
leg_status(battery_depleted, _,_,_,_,_,_, _Code, false).
leg_status(Reason, _,_,_,_,_,_, Code, reactive(Code)) :-
    Reason \= completed,
    Reason \= crashed(_),
    Reason \= battery_depleted.

% earliest_of/3: like earliest_of/2, but takes a fixed head list
% (currently just [completed-NaturalEnd-none]) and a (possibly empty)
% list of Triggers-derived candidates separately, and combines them --
% kept as a distinct small wrapper so call sites read as "natural
% completion ++ whatever this leg's Triggers produced," the intent,
% at a glance.
earliest_of(BuiltIns, ExtraCandidates, Result) :-
    append(BuiltIns, ExtraCandidates, All),
    earliest_of(All, Result).

% append/3 (ProbLog has no built-in append/3 -- see the note on
% drop_n/3 earlier in this file for why hand-rolled list utilities
% appear throughout).
append([], L, L).
append([H|T], L, [H|R]) :- append(T, L, R).

% interrupt(T): the one genuinely CHOSEN action -- the plan decides
% when to cut the walk short. Only possible while a walk is in
% progress, at or after its start, and STRICTLY BEFORE whichever
% happens first in this resolved world among ALL candidate causes in
% this leg's own Triggers list -- you can't "interrupt" a walk that,
% in this world, has already halted on its own for any reason. Uses
% the SAME earliest_halt/10 as Poss(haltMoveto(...)) above, so this
% bound can never drift out of sync with what haltMoveto itself would
% derive.
poss(interrupt(T), S) :-
    moving(S),
    current_walk(S, CP, Triggers, T0, SPrev),
    hitch(Tool, SPrev),
    walk_duration(CP, Tool, SPrev, Duration),
    effective_tool_moving_drain_rate(Tool, SPrev, Rate),
    z(do(startMoveto(CP,Triggers,_ActionCode,T0),SPrev), Z),
    zt(do(startMoveto(CP,Triggers,_ActionCode,T0),SPrev), Zt),
    zbatt(Zb),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(CP,Triggers,T0,Duration,Z,Zt,Zb,B0,Rate,0, _Reason,Tend,_Code),
    T >= T0, T < Tend.

% poss(take_sample(SampleId,ActionCode,Reason,Status), S): take_sample's
% own ONE precondition -- \+ moving(S), can't take a sample mid-walk
% (the robot's own position is only meaningfully "where I am right
% now" once a walk has actually stopped -- see at/4's own note on
% freezing after a halt). Otherwise derives X,Y (at/4, already
% resolved) and draws sample_result/3, binding Reason/Status TOGETHER
% -- the SAME "Poss derives the action term's own output arguments"
% idiom poss(haltMoveto(T,Reason,Status),S) already uses above. See
% do_node(take_sample(...))'s own note (Section 5, ACTION leaf) for
% the full picture.
%
% SampleId is the BT tree author's OWN chosen name for this occurrence
% (schema.yaml's own TakeSample id port -- e.g. "soil1"), NOT the
% auto-generated ActionCode: unlike ActionCode (assigned invisibly,
% sequentially, during translation -- see _VarPool.next_action_code()),
% a later SampleValueOver/Below/Equal condition node needs a name the
% TREE AUTHOR can actually write down ahead of time to point back at
% THIS specific sample, the same reason tool_instance ids exist (see
% that predicate's own note) -- ActionCode and SampleId serve two
% different purposes and are BOTH carried in Reason, exactly the same
% "domain identifier plus auto-appended ActionCode" shape install_tool/
% uninstall_tool's own Reason (install_tool_success(Id,ActionCode))
% already established.
%
% ONLY on SUCCESS is a VALUE actually drawn (sample_value/3, this
% problem's own config.yaml sample.value.mean/sigma -- a discretized
% Normal distribution, see config_to_prolog.py's own note) -- a failed
% sample reading is exactly that, a failed reading, there is no number
% to report, same reasoning a failed install_tool never gets a
% Duration-elapsed coin-flip outcome of its own either. sample_value/3
% is keyed by (S,ActionCode), the SAME "fresh draw per genuinely new
% situation" convention sample_result/3 itself already uses -- SampleId
% plays no role in ITS OWN key, it only travels along inside Reason for
% a later condition to read back out.
poss(take_sample(SampleId,ActionCode,Reason,true), S) :-
    \+ moving(S),
    now(T, S), at(X,Y,T,S),
    sample_result(S, ActionCode, true),
    sample_value(S, ActionCode, V),
    tag_reason(sample_success(X,Y,V,SampleId), ActionCode, Reason).
poss(take_sample(SampleId,ActionCode,Reason,false), S) :-
    \+ moving(S),
    now(T, S), at(X,Y,T,S),
    sample_result(S, ActionCode, false),
    tag_reason(sample_failure(X,Y,SampleId), ActionCode, Reason).

% poss(start_install_tool(Id,Triggers,ActionCode,T0), S): the three
% preconditions this action's own design calls for -- \+ moving(S)
% (can't start installing mid-walk, same reasoning as take_sample's own
% precondition); hitch(free,S) (nothing already attached -- see
% hitch/2's own note, Section 5c); and PROXIMITY -- the robot's current
% position must be close to Id's own CURRENT location before it can be
% installed. Reuses holds(distance_below(...)) DIRECTLY (the exact same
% predicate a BT tree's own cond(distance_below(...)) leaf or a
% holds_leg/9 reactive guard would use) rather than re-deriving the
% dist/5 call by hand -- "close to the tool" IS "distance_below the
% tool's own point, at Range", no different in kind from "close to the
% goal". tool_position(Id,GX,GY,S) is the relational fluent above --
% wherever Id started (config.yaml tool.instances) or was last dropped,
% and it simply fails (making install impossible) for an Id that is
% unknown OR already attached; install_tool_range/1 (tool.install.range,
% defaulting to safety_margin) is the ONE proximity threshold, shared
% across every tool kind (matching install_tool_drain_rate/1's own
% "specific to the ACTION, not the tool" shape). T0 quantization
% mirrors poss(startMoveto(...)) above exactly (same merge-grid
% rationale).
poss(start_install_tool(Id,_Triggers,_ActionCode,T0), S) :-
    \+ moving(S),
    hitch(free, S),
    tool_position(Id, GX, GY, S),
    install_tool_range(Range),
    holds(distance_below(GX,GY,Range), S),
    now(T0Exact, S),
    disc_step_time(Grid),
    quantize_up(T0Exact, Grid, T0).

% poss(halt_install_tool(T,Reason,Status), S): installing_tool(Id,S)
% is checked EXPLICITLY (this action's own request), even though
% current_install_tool/6 below would already implicitly require it --
% same redundant-but-explicit style poss(haltMoveto(...)) already uses
% for moving(S) alongside current_walk/6. tool_instance(Id,Kind) is the
% ONE new step versus before Id/Kind were split apart -- install_tool_
% duration/2 is still keyed by KIND (config.yaml tool.install.
% duration_seconds is per-kind, not per-instance: every cart takes the
% same time to install), so this joins through tool_instance/2 to
% recover it. TWO clauses, matching tool_earliest_halt/9's own
% "completed is a SENTINEL here, not a real Reason -- see its own
% WARNING" contract: clause 1 is what actually happens when Duration
% elapses with nothing halting it early -- ONLY THEN does install_tool_
% result/3 (this problem's own config.yaml, tool.install.
% success_probability -- see config_generated.pl) get drawn, keyed on
% (S,ActionCode) exactly like take_sample's own sample_result/3 (same
% "fresh draw per genuinely new situation, shared only if the exact
% situation term recurs" semantics -- see that predicate's own note);
% clause 2 is a genuine battery trigger firing first, used as-is, same
% shape poss(haltMoveto(...)) itself uses for its own ExtraCandidates.
% Matching Reason0 against the bare atom `completed` in clause 1 (and
% excluding it in clause 2) is exactly why that atom can never leak out
% as this action's own recorded Reason -- it's consumed and replaced by
% Reason0 (install_tool_success(Id)/install_tool_failure(Id)) before
% Status is even decided.
poss(halt_install_tool(T,Reason,Status), S) :-
    installing_tool(Id, S),
    current_install_tool(S, Id, Triggers, ActionCode, T0, SPrev),
    tool_instance(Id, Kind),
    install_tool_duration(Kind, Duration),
    install_tool_drain_rate(Rate),
    zbatt(Zb),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(_CP,Triggers,T0,Duration,_Z,_Zt,Zb,B0,Rate,1, completed,T,_Code),
    install_tool_result(S, ActionCode, CoinStatus),
    tool_install_reason(CoinStatus, Id, Reason0),
    tool_leg_status(Reason0, none, Status),
    tag_reason(Reason0, ActionCode, Reason).
poss(halt_install_tool(T,Reason,Status), S) :-
    installing_tool(Id, S),
    current_install_tool(S, Id, Triggers, ActionCode, T0, SPrev),
    tool_instance(Id, Kind),
    install_tool_duration(Kind, Duration),
    install_tool_drain_rate(Rate),
    zbatt(Zb),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(_CP,Triggers,T0,Duration,_Z,_Zt,Zb,B0,Rate,1, Reason0,T,Code),
    Reason0 \= completed,
    tool_leg_status(Reason0, Code, Status),
    tag_reason(Reason0, ActionCode, Reason).

tool_install_reason(true, Id, install_tool_success(Id)).
tool_install_reason(false, Id, install_tool_failure(Id)).

% poss(start_uninstall_tool(Id,Triggers,ActionCode,T0), S): the
% mirror image of poss(start_install_tool(...)) above -- \+ moving(S),
% and hitch_id(Id,S) (per this action's own request: uninstalling A
% SPECIFIC tool INSTANCE requires THAT instance -- not just "something
% of its kind" -- to currently be the one attached; hitch_id/2, not
% hitch/2, is the fluent that can tell cart1 apart from cart2). ALSO
% \+ deployed(S) -- a tool currently deployed (Section 5d) must be
% retracted first, per this feature's own request: uninstalling
% something still lowered into the ground isn't allowed.
poss(start_uninstall_tool(Id,_Triggers,_ActionCode,T0), S) :-
    \+ moving(S),
    hitch_id(Id, S),
    \+ deployed(S),
    now(T0Exact, S),
    disc_step_time(Grid),
    quantize_up(T0Exact, Grid, T0).

% poss(halt_uninstall_tool(T,Reason,Status), S): the mirror image of
% poss(halt_install_tool(...)) above, uninstall_tool_result/3 (this
% problem's own config.yaml, tool.uninstall.success_probability) in
% place of install_tool_result/3, and uninstall_tool_duration/2 (also
% KIND-keyed, same tool_instance/2 join as install's own) in place of
% install_tool_duration/2.
poss(halt_uninstall_tool(T,Reason,Status), S) :-
    uninstalling_tool(Id, S),
    current_uninstall_tool(S, Id, Triggers, ActionCode, T0, SPrev),
    tool_instance(Id, Kind),
    uninstall_tool_duration(Kind, Duration),
    uninstall_tool_drain_rate(Rate),
    zbatt(Zb),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(_CP,Triggers,T0,Duration,_Z,_Zt,Zb,B0,Rate,1, completed,T,_Code),
    uninstall_tool_result(S, ActionCode, CoinStatus),
    tool_uninstall_reason(CoinStatus, Id, Reason0),
    tool_leg_status(Reason0, none, Status),
    tag_reason(Reason0, ActionCode, Reason).
poss(halt_uninstall_tool(T,Reason,Status), S) :-
    uninstalling_tool(Id, S),
    current_uninstall_tool(S, Id, Triggers, ActionCode, T0, SPrev),
    tool_instance(Id, Kind),
    uninstall_tool_duration(Kind, Duration),
    uninstall_tool_drain_rate(Rate),
    zbatt(Zb),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(_CP,Triggers,T0,Duration,_Z,_Zt,Zb,B0,Rate,1, Reason0,T,Code),
    Reason0 \= completed,
    tool_leg_status(Reason0, Code, Status),
    tag_reason(Reason0, ActionCode, Reason).

tool_uninstall_reason(true, Id, uninstall_tool_success(Id)).
tool_uninstall_reason(false, Id, uninstall_tool_failure(Id)).

% poss(start_deploy_tool(Id,Triggers,ActionCode,T0), S): the deploy_
% tool analogue of poss(start_install_tool(...)) above -- \+ moving(S)
% (can't deploy mid-walk); hitch_id(Id,S) (Id must ALREADY be attached
% -- can't lower a tool that isn't installed); tool_instance(Id,plow)
% (currently restricted to the plow ONLY -- per this feature's own
% request, "for now"; a future tool kind that can also deploy just
% needs its own atom added here, or this replaced with a proper
% deployable-kinds table if the list grows past one); \+ deployed(S)
% (can't deploy what's already deployed). T0 quantization mirrors every
% other durative action's own poss/2 (same merge-grid rationale).
poss(start_deploy_tool(Id,_Triggers,_ActionCode,T0), S) :-
    \+ moving(S),
    hitch_id(Id, S),
    tool_instance(Id, plow),
    \+ deployed(S),
    now(T0Exact, S),
    disc_step_time(Grid),
    quantize_up(T0Exact, Grid, T0).

% poss(halt_deploy_tool(T,Reason,Status), S): the deploy_tool analogue
% of poss(halt_install_tool(...)) above -- deploying_tool(Id,S) checked
% EXPLICITLY (same redundant-but-explicit style), tool_instance(Id,
% Kind) joined through for deploy_tool_duration/2 (KIND-keyed, same
% reasoning install_tool_duration/2 already has -- every plow takes the
% same time to deploy), deploy_tool_drain_rate/1 its own dedicated
% action-specific rate (mirroring install_tool_drain_rate/1). Same TWO-
% clause "completed is a sentinel, not a real Reason" contract as
% poss(halt_install_tool(...)).
poss(halt_deploy_tool(T,Reason,Status), S) :-
    deploying_tool(Id, S),
    current_deploy_tool(S, Id, Triggers, ActionCode, T0, SPrev),
    tool_instance(Id, Kind),
    deploy_tool_duration(Kind, Duration),
    deploy_tool_drain_rate(Rate),
    zbatt(Zb),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(_CP,Triggers,T0,Duration,_Z,_Zt,Zb,B0,Rate,1, completed,T,_Code),
    deploy_tool_result(S, ActionCode, CoinStatus),
    tool_deploy_reason(CoinStatus, Id, Reason0),
    tool_leg_status(Reason0, none, Status),
    tag_reason(Reason0, ActionCode, Reason).
poss(halt_deploy_tool(T,Reason,Status), S) :-
    deploying_tool(Id, S),
    current_deploy_tool(S, Id, Triggers, ActionCode, T0, SPrev),
    tool_instance(Id, Kind),
    deploy_tool_duration(Kind, Duration),
    deploy_tool_drain_rate(Rate),
    zbatt(Zb),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(_CP,Triggers,T0,Duration,_Z,_Zt,Zb,B0,Rate,1, Reason0,T,Code),
    Reason0 \= completed,
    tool_leg_status(Reason0, Code, Status),
    tag_reason(Reason0, ActionCode, Reason).

tool_deploy_reason(true, Id, deploy_tool_success(Id)).
tool_deploy_reason(false, Id, deploy_tool_failure(Id)).

% poss(start_retract_tool(Id,Triggers,ActionCode,T0), S): the mirror
% image of poss(start_deploy_tool(...)) above -- \+ moving(S), Id
% currently hitched AND deployed(S) (can't retract what isn't
% deployed). No tool_instance(Id,plow) check needed here explicitly --
% deployed(S) can only ever be true for a plow in the first place (the
% ONLY way it becomes true is via a successful halt_deploy_tool, which
% itself already required tool_instance(Id,plow) -- see poss(start_
% deploy_tool(...)) above), so requiring deployed(S) already implies it.
poss(start_retract_tool(Id,_Triggers,_ActionCode,T0), S) :-
    \+ moving(S),
    hitch_id(Id, S),
    deployed(S),
    now(T0Exact, S),
    disc_step_time(Grid),
    quantize_up(T0Exact, Grid, T0).

% poss(halt_retract_tool(T,Reason,Status), S): the mirror image of
% poss(halt_deploy_tool(...)) above, retract_tool_result/3 (this
% problem's own config.yaml, tool.retract.success_probability) in
% place of deploy_tool_result/3, and retract_tool_duration/2 (also
% KIND-keyed, same tool_instance/2 join) in place of deploy_tool_
% duration/2.
poss(halt_retract_tool(T,Reason,Status), S) :-
    retracting_tool(Id, S),
    current_retract_tool(S, Id, Triggers, ActionCode, T0, SPrev),
    tool_instance(Id, Kind),
    retract_tool_duration(Kind, Duration),
    retract_tool_drain_rate(Rate),
    zbatt(Zb),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(_CP,Triggers,T0,Duration,_Z,_Zt,Zb,B0,Rate,1, completed,T,_Code),
    retract_tool_result(S, ActionCode, CoinStatus),
    tool_retract_reason(CoinStatus, Id, Reason0),
    tool_leg_status(Reason0, none, Status),
    tag_reason(Reason0, ActionCode, Reason).
poss(halt_retract_tool(T,Reason,Status), S) :-
    retracting_tool(Id, S),
    current_retract_tool(S, Id, Triggers, ActionCode, T0, SPrev),
    tool_instance(Id, Kind),
    retract_tool_duration(Kind, Duration),
    retract_tool_drain_rate(Rate),
    zbatt(Zb),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(_CP,Triggers,T0,Duration,_Z,_Zt,Zb,B0,Rate,1, Reason0,T,Code),
    Reason0 \= completed,
    tool_leg_status(Reason0, Code, Status),
    tag_reason(Reason0, ActionCode, Reason).

tool_retract_reason(true, Id, retract_tool_success(Id)).
tool_retract_reason(false, Id, retract_tool_failure(Id)).

% ---------------------------------------------------------------
% 6. THE POSITION FLUENT -- pure situation-calculus regression.
%    at(X,Y,T,S): position of the robot at time T in situation S.
%
%    Three cases:
%      (a) base case: before any walk, position is the fixed start
%      (b) INSIDE an open walk (do(startMoveto(...),S)): interpolate
%          along the spline + noise, exactly as before
%      (c) AFTER the walk has ended (do(haltMoveto(T1,_),S) -- for
%          EITHER reason, completed or crashed -- or do(interrupt(T1),S)):
%          position FREEZES at whatever it was at time T1. This is
%          what makes both natural halting and interruption actually
%          stop the robot rather than letting it keep gliding toward
%          the original target -- and in particular, a crash freezes
%          the robot exactly at the point of collision, not beyond it.
% ---------------------------------------------------------------
at(X,Y,_,s0) :- start(X,Y).

at(X,Y,T, do(startMoveto(ControlPoints,Triggers,ActionCode,T0), S)) :-
    hitch(Tool, S),
    walk_duration(ControlPoints, Tool, S, Duration),
    z(do(startMoveto(ControlPoints,Triggers,ActionCode,T0),S), Z),
    zt(do(startMoveto(ControlPoints,Triggers,ActionCode,T0),S), Zt),
    walk_noisy_point(ControlPoints, T0, Duration, Z, Zt, T, X, Y).

at(X,Y,T, do(haltMoveto(T1,_Reason,_Status), S)) :-
    Tc is min(T,T1),
    at(X,Y,Tc,S).

at(X,Y,T, do(interrupt(T1), S)) :-
    Tc is min(T,T1),
    at(X,Y,Tc,S).

% pass-through clause: kept for extensibility, so that additional
% actions that DON'T affect position (e.g. a future sensing action)
% can be appended without breaking the regression.
at(X,Y,T, do(A,S)) :-
    A \= startMoveto(_,_,_,_), A \= haltMoveto(_,_,_), A \= interrupt(_),
    at(X,Y,T,S).

% nominal (zero-noise) position -- for Feature-1-style deterministic
% checks and for the on_track discrepancy fluent. Frac here is
% ALWAYS relative to the walk's own full nominal duration (not to
% however much of it actually got executed before an interrupt).
nominal_at(X,Y,Frac,ControlPoints) :- spline_point(ControlPoints, Frac, X, Y).

% ============================================================
% 7. PRIMITIVE ACTION EXECUTION + BEHAVIOR-TREE INTERFACE.
%
%     do_node(Node, S, S1, Outcome)
%
% Outcome is 'true' or 'false' -- the two-valued signal every node,
% leaf or composite, reports through the SAME predicate. This is the
% "standard interface" every action/condition must supply: given the
% current situation, produce a resulting situation and an outcome.
% Sequence and Fallback below are written PURELY in terms of this
% interface -- they never inspect what KIND of thing a child is,
% which is what makes them genuine reusable templates over a LIST of
% arbitrarily many, arbitrarily nested children.
%
% A single true/false vocabulary is used everywhere -- not
% success/failure -- so every leaf's own natural output (a
% moveto_leg's Status is already true/false; a cond(C)'s holds/2 is
% already a true/false question) can flow straight through as Outcome
% with no translation step in between.
%
% Node is one of:
%     cond(C,Code)              -- CONDITION leaf: tests C against the
%                                  CURRENT situation via holds/2. UNLIKE
%                                  an earlier version of this file
%                                  (cond(C), no Code, S1=S, a genuine
%                                  no-op), this now DOES extend the
%                                  situation, with a checked(Code,C,
%                                  Status) marker -- the direct
%                                  analogue of planWith's own planned
%                                  (Algorithm,Reason) marker just below,
%                                  for the SAME reason: otherwise a
%                                  condition's own outcome leaves no
%                                  trace once execution moves past it,
%                                  making it unqueryable after the fact
%                                  (an action's own Reason survives in
%                                  haltMoveto(...)/planned(...); a bare
%                                  cond(C) with S1=S never did). Code is
%                                  a per-CONDITION-OCCURRENCE identifier
%                                  bt_to_prolog.py assigns (same
%                                  per-occurrence-counter idiom as
%                                  MoveTo/PlanWith's own ActionCode),
%                                  letting checked_with/4 further down
%                                  (the direct analogue of planned_with/
%                                  3) distinguish WHICH cond() leaf in
%                                  the tree a given checked(...) marker
%                                  came from -- there can be many, e.g.
%                                  a DistanceBelow after EVERY MoveTo.
%     moveto_leg(CP,Triggers)   -- ACTION leaf. Triggers is ALWAYS
%                                  given explicitly at the call site --
%                                  there is no sugar/default form, by
%                                  design (see the note above Triggers
%                                  in this file's own header). Runs one
%                                  startMoveto/haltMoveto pair to its
%                                  halt; T0 auto-derived via now/2.
%                                  Outcome IS the Status output,
%                                  unchanged.
%     planWith(Algorithm,Goal,CP) -- PLANNING leaf: ONE TEMPLATE covering
%                                  every planner (plan_astar,
%                                  plan_straight, and any future one --
%                                  see plan_call/8's own dispatch on
%                                  Algorithm) -- not a separate do_node
%                                  clause per planner. A stateless
%                                  black-box call into
%                                  planners.py, binding CP for a
%                                  subsequent moveto_leg(CP,...) to
%                                  use. Goal is EXPLICIT (point(GX,GY)),
%                                  an argument of planWith itself, not
%                                  the global goal/2 fact -- lets two
%                                  planWith calls in one plan target
%                                  genuinely different destinations.
%                                  Interface analogous to haltMoveto's
%                                  own (Reason,Status) formalization
%                                  via plan_call/8 -- Reason is
%                                  completed/no_path -- without any of
%                                  its situation-calculus machinery
%                                  (no primitive_action, no Poss).
%                                  UNLIKE plan_call/8 itself, this DOES
%                                  extend the situation, with a bare
%                                  planned(Algorithm,Reason) MARKER (no
%                                  precondition -- see planned_with/3),
%                                  so a plan calling planning more than
%                                  once stays traceable.
%     seq_node(ChildList)       -- BT SEQUENCE: run children in
%                                  order; stop and fail as soon as one
%                                  fails; succeed if all do.
%     fallback_node(ChildList)  -- BT FALLBACK/SELECTOR: try children
%                                  in order; stop and succeed as soon
%                                  as one succeeds; fail if all do.
% ============================================================
primitive_action(startMoveto(_,_,_,_)).
primitive_action(haltMoveto(_,_,_)).
primitive_action(interrupt(_)).
primitive_action(take_sample(_,_,_,_)).
primitive_action(start_install_tool(_,_,_,_)).
primitive_action(halt_install_tool(_,_,_)).
primitive_action(start_uninstall_tool(_,_,_,_)).
primitive_action(halt_uninstall_tool(_,_,_)).
primitive_action(start_deploy_tool(_,_,_,_)).
primitive_action(halt_deploy_tool(_,_,_)).
primitive_action(start_retract_tool(_,_,_,_)).
primitive_action(halt_retract_tool(_,_,_)).

do_action(A, S, do(A,S)) :- primitive_action(A), poss(A, S).

% -- CONDITION leaf ---------------------------------------------------
% checked(Code,C,Status) is a bare MARKER, exactly like planned(Algorithm,
% Reason) below -- no primitive_action entry, no Poss, just recorded via
% do(...) the same way do_action itself would. Status is fully
% DETERMINED by holds(C,S) at an ALREADY-RESOLVED situation -- recording
% it introduces no new probabilistic choice (unlike z/2's own per-
% startMoveto annotated disjunction), so this adds no branching to the
% underlying inference, only one more do(...) layer for the generic
% pass-through fluents (at/4, battery/3, moving/1, now/2, current_walk/6)
% to skip over -- exactly the same, already-proven-cheap shape planned
% (Algorithm,Reason) already added for PlanWith.
do_node(cond(C,Code), S, do(checked(Code,C,true), S), true)  :- holds(C, S).
do_node(cond(C,Code), S, do(checked(Code,C,false),S), false) :- \+ holds(C, S).

% -- ACTION leaf: take_sample -------------------------------------------
% take_sample(SampleId,ActionCode) -- INSTANTANEOUS (no Duration/
% Triggers/continuous trajectory of any kind), but UNLIKE PlanWith/
% cond(C,Code) above, it DOES go through the full primitive_action/
% poss/do_action machinery every OTHER genuine action in this theory
% uses -- Reiter's own convention gates EVERY action on a Poss
% precondition regardless of whether it takes time; PlanWith/cond are
% the deliberate exceptions (pure computation / a fact about the
% CURRENT situation with no precondition of its own to state), not the
% default. The one precondition, for now: \+ moving(S) -- can't take a
% sample while mid-walk. See primitive_action(take_sample(_,_,_,_)) and
% poss(take_sample(...),S) further up/down (grep for both).
%
% Its own outcome is a genuine new probabilistic choice, NOT determined
% by the current situation at all (unlike cond(C,Code)'s own
% holds(C,S)) -- sample_result/3 (this problem's own config.yaml,
% sample.success_probability -- see config_generated.pl), an annotated
% disjunction keyed by (S,ActionCode): a FRESH draw for a genuinely new
% situation, but the SAME cached draw if the exact same (S,ActionCode)
% pair is ever re-derived (e.g. two upstream noise realizations that
% landed on the same merge-grid-quantized point before reaching this
% node) -- same "keyed on the situation term" convention z/2 and zt/2
% already use for their own per-startMoveto draws, so "does the same
% PHYSICAL STATE always sample the same way" is emphatically NOT what
% this guarantees; only an identical SITUATION TERM does. sample_value/3
% is a SECOND, independent annotated disjunction of the SAME shape,
% drawn ONLY when sample_result/3 comes back true (see poss(take_sample
% (...))'s own note) -- a discretized Normal(mean,sigma) over the
% integers 0..10 (config.yaml sample.value.mean/sigma), NOT a fixed
% coin flip like sample_result/3 itself: this is what actually lets a
% later condition ask "was the READING above 7", not just "did the
% sample succeed".
%
% X,Y (the robot's own CURRENT position, at/4, already resolved -- no
% further noise of its own here) and V (the drawn value, success only)
% are baked directly into the Reason, sample_success(X,Y,V,SampleId) or
% sample_failure(X,Y,SampleId), then tagged with ActionCode via the
% SAME generic tag_reason/3 haltMoveto/PlanWith already use -- giving
% sample_success(X,Y,V,SampleId,ActionCode)/sample_failure(X,Y,SampleId,
% ActionCode), the standard "trailing ActionCode" Reason shape every
% other action in this theory produces, all bound TOGETHER by Poss --
% the exact same "Poss derives the action term's own output arguments"
% idiom poss(haltMoveto(T,Reason,Status),S) already uses, now that
% take_sample is a genuine primitive_action too. This still needed its
% OWN dedicated clauses at every "outermost do(...) layer" pattern-
% match site -- halted_with/2 and outcome_entry/2, mirroring haltMoveto/
% planned's own clauses at each -- verified directly (a standalone
% ProbLog run caught any_reason_pattern(sample_success(wild,wild))
% silently reading 0% before those clauses were added: halted_with/2's
% own GENERIC do(_A,S) clause only SKIPS an unrecognized marker while
% searching deeper, it does not also try to unify Reason against
% whatever's inside it, so this doesn't come "for free" the way
% at/4/battery/3/moving/1/now/2's OWN generic pass-through clauses do
% for a bare marker with no Reason of its own to extract). Once
% halted_with/2 sees it, any_reason_pattern(_by_action) and the
% outcome-enumeration table (outcome_entry/2's own new clause below)
% both work correctly with no further changes. See also
% sample_success_at/3 further down, for querying WHERE a successful
% sample landed, and holds(sample_value_over(...)) and its Below/Equal
% siblings (Section 8, near distance_below/3) for querying WHAT VALUE a
% specific, author-named sample (SampleId) came back with.
do_node(take_sample(SampleId,ActionCode), S, S1, Status) :-
    do_action(take_sample(SampleId,ActionCode,_Reason,Status), S, S1).

% -- ACTION leaf --------------------------------------------------------
% moveto_leg(CP,Triggers) -- Triggers is ALWAYS given explicitly here;
% there is deliberately NO sugar/default form (no moveto_leg/1, no
% config-driven fallback) -- every call site states its own protection
% level, e.g. moveto_leg(CP,[collision,battery]) or moveto_leg(CP,[])
% for a genuinely unprotected leg. Status flows straight through as
% Outcome -- no translation predicate needed, since both already speak
% true/false.
do_node(moveto_leg(CP,Triggers,ActionCode), S, S1, Status) :-
    do_action(startMoveto(CP,Triggers,ActionCode,_T0), S, S2),
    do_action(haltMoveto(_T,_Reason,Status), S2, S1).

% -- ACTION leaf: install_tool / uninstall_tool -------------------------
% install_tool_leg(Tool,Triggers,ActionCode) -- a DURATIVE action, same
% start/halt shape as moveto_leg above (do_action(start_install_tool
% (...)),do_action(halt_install_tool(...))), just with a FIXED,
% config-driven Duration (install_tool_duration(Tool,Duration) -- see
% config_generated.pl) instead of one derived from spline arc length,
% and no continuous position of its own at all: the robot never moves,
% so at/4's/moving/1's/current_walk/6's own EXISTING generic pass-
% through clauses already handle these two new action functors
% correctly with NO changes needed to any of them. battery/3 and now/2
% both DO need their own new explicit clauses (see each predicate's
% own note) -- now/2's generic pass-through would otherwise silently
% ignore a genuine clock advance, and battery/3 needed a real Duration-
% normalized anchor (the SAME shape as startMoveto/haltMoveto's own,
% install_tool_drain_rate/1 or uninstall_tool_drain_rate/1 in place of
% moving_drain_rate) so its own Level computation during the leg stays
% consistent with the closed-form crossing-time search (first_battery_
% depletion_time and friends, SHARED with MoveTo via trigger_crossing_
% time/13's own Mode argument -- see that predicate's own note) -- both
% need to agree on the SAME Elapsed basis (from T0, not from whatever
% idle anchor came before it). Triggers is the SAME mechanism as
% moveto_leg's own, RESTRICTED to battery-related names only (see the
% TOOL TRIGGERS section, above tool_cond_supported/1, for why and how
% motion-based ones gracefully contribute nothing instead of erroring).
% installing_tool(Id,S) is TRUE for
% every situation between start_install_tool and halt_install_tool
% (Section 5b) -- an explicit precondition of halt_install_tool's own
% Poss, per this action's own request, even though current_install_
% tool/6 would already implicitly require it. hitch(free,S) gates
% starting an install; hitch_id(Id,S) (THIS specific tool INSTANCE, not
% just "something of its kind") gates starting an uninstall -- see
% hitch/2's/hitch_id/2's own notes (Section 5c) for the full state
% machine, including why only a SUCCESSFUL halt flips either one. Tool
% here (and throughout install_tool_leg/uninstall_tool_leg's own
% do_action calls below) is always an Id, e.g. "cart1" from this
% problem's own config.yaml tool.instances -- never a bare kind name --
% see tool_instance/2's own note.
do_node(install_tool_leg(Tool,Triggers,ActionCode), S, S1, Status) :-
    do_action(start_install_tool(Tool,Triggers,ActionCode,_T0), S, S2),
    do_action(halt_install_tool(_T,_Reason,Status), S2, S1).

do_node(uninstall_tool_leg(Tool,Triggers,ActionCode), S, S1, Status) :-
    do_action(start_uninstall_tool(Tool,Triggers,ActionCode,_T0), S, S2),
    do_action(halt_uninstall_tool(_T,_Reason,Status), S2, S1).

% do_node(deploy_tool_leg(...))/do_node(retract_tool_leg(...)): the
% deploy_tool/retract_tool analogues of do_node(install_tool_leg(...))/
% do_node(uninstall_tool_leg(...)) just above -- IDENTICAL shape.
do_node(deploy_tool_leg(Tool,Triggers,ActionCode), S, S1, Status) :-
    do_action(start_deploy_tool(Tool,Triggers,ActionCode,_T0), S, S2),
    do_action(halt_deploy_tool(_T,_Reason,Status), S2, S1).

do_node(retract_tool_leg(Tool,Triggers,ActionCode), S, S1, Status) :-
    do_action(start_retract_tool(Tool,Triggers,ActionCode,_T0), S, S2),
    do_action(halt_retract_tool(_T,_Reason,Status), S2, S1).

% -- PLANNING actions: deliberately NOT part of the full action theory
%    -- no primitive_action/1 entry, no Poss axiom, no do_action call
%    with its precondition check (these are stateless, purely-
%    computational black-box calls into planners.py, not
%    physical processes -- Reiter's machinery exists to solve the
%    frame problem for things that CHANGE THE WORLD over time; a
%    lookup that returns instantly has no frame problem to solve, so
%    building out that whole apparatus for it would be unnecessary
%    weight, not faithfulness).
%
%    The interface is FORMALIZED THE SAME WAY AS haltMoveto's own
%    (T,Reason,Status): plan_call/8 binds Reason AND Status TOGETHER,
%    directly, within each clause -- exactly as leg_status/7 binds
%    Status alongside a given Reason for moveto, rather than via a
%    SEPARATE Reason->Status lookup table. Status is a genuine second
%    output, not a name recomputed from Reason after the fact; it just
%    happens (for these two planners) to always agree with Reason one-
%    for-one, since there's no extra condition to check beyond "was a
%    path found" (moveto's leg_status has an EXTRA condition --
%    within-tolerance position -- planning does not). plan_call/8 is a
%    genuine standalone predicate, callable directly (not only through
%    do_node), so Reason is truly accessible, not swallowed on the way
%    to Status.
%
%    plan_call/8 is ALREADY a genuine TEMPLATE over Algorithm (its own
%    first argument) -- astar and straight are just two clauses of the
%    SAME predicate, dispatched by ordinary clause selection. Adding a
%    future THIRD planner (e.g. an RRT, or a different black-box
%    module entirely) means adding one more plan_astar-style function
%    to planners.py plus one more pair of plan_call/8 clauses
%    here; nothing about do_node, planned_with/3, or anything
%    downstream needs to change. Because Reason/Status are bound
%    DIRECTLY per clause rather than through a shared central lookup
%    table, a future planner is also free to introduce its OWN
%    distinct Reason atoms (e.g. a timeout-capable planner might use
%    `timeout` alongside `no_path`, both mapping to Status=false) --
%    it declares that mapping in its own clauses, with no need to
%    extend a shared table every other planner also depends on.
%
%    Common Reason vocabulary, shared by EVERY planner (only one
%    possible failure cause: no path exists): completed on success
%    (matching moveto's own success-Reason naming), no_path on
%    failure. CP is [] in the no_path case.
plan_call(astar, SX,SY,GX,GY, CP, completed, true) :-
    plan_astar(SX,SY,GX,GY, CP).
plan_call(astar, SX,SY,GX,GY, [], no_path, false) :-
    \+ plan_astar(SX,SY,GX,GY, _).

plan_call(straight, SX,SY,GX,GY, CP, completed, true) :-
    plan_straight(SX,SY,GX,GY, CP).
plan_call(straight, SX,SY,GX,GY, [], no_path, false) :-
    \+ plan_straight(SX,SY,GX,GY, _).

% voronoi -- a THIRD planner, SAME bare-atom Algorithm shape as astar/
% straight (no compound term needed -- this planner takes no extra
% parameters beyond the shared SX,SY,GX,GY every planWith call already
% carries), exactly the "add one more plan_astar-style function plus
% one more pair of plan_call/8 clauses" recipe this section's own
% header comment anticipated -- needing NO new dispatch machinery
% anywhere downstream (do_node/4, schema.yaml, bt_to_prolog.py, and
% bt_actions.py all reuse their EXISTING astar/straight branches
% verbatim for this one). Builds a roadmap from a generalized Voronoi
% diagram of the obstacle map (planners.py's plan_voronoi/5),
% connecting SX,SY and GX,GY to the closest POINT on the closest EDGE
% of that roadmap -- see that predicate's own header for the geometry.
% Degrades to a straight line (never fails) when there are no
% obstacles to route around at all; only fails (no_path) if a roadmap
% exists but start/goal are genuinely disconnected within it.
plan_call(voronoi, SX,SY,GX,GY, CP, completed, true) :-
    plan_voronoi(SX,SY,GX,GY, CP).
plan_call(voronoi, SX,SY,GX,GY, [], no_path, false) :-
    \+ plan_voronoi(SX,SY,GX,GY, _).

% dastar(Step) -- a FIFTH planner: plans the SAME A* raster path
% plan_astar/plan_call(astar,...) itself would (planners.py's own
% _astar_control_points core, before that function's own spline-fit
% step), but instead of fitting one smooth curve through the raw grid
% path, DISCRETIZES it first -- one waypoint every Step metres of arc
% length walked along that raw path (planners.py's own
% _resample_path_every_step) -- then reuses PlanWithWaypoints' own
% multi-leg STRAIGHT-LINE chaining (_chain_multi_leg_control_points)
% to connect those waypoints, exactly like plan_straight_waypoints'
% own contract except the waypoints are CHOSEN by A* itself rather
% than supplied by the tree author. Algorithm here is a COMPOUND term
% (dastar(Step)), same reason as follow_boarder(ObstacleId,Offset)
% above: Step is a planner-specific extra parameter carried INSIDE
% Algorithm, so planWith/do_node/planned_with need no interface
% change. UNLIKE follow_boarder, dastar(Step) DOES have a real Goal
% point (GX,GY) -- it falls through to do_node(planWith(Algorithm,
% point(GX,GY),...))'s own second clause below (guarded by Algorithm
% \= follow_boarder(_,_), which dastar(Step) satisfies), no third
% do_node clause needed.
plan_call(dastar(Step), SX,SY,GX,GY, CP, completed, true) :-
    plan_dastar(SX,SY,GX,GY,Step, CP).
plan_call(dastar(Step), SX,SY,GX,GY, [], no_path, false) :-
    \+ plan_dastar(SX,SY,GX,GY,Step, _).

% follow_boarder(ObstacleId,Offset) -- a FOURTH planner, exactly the
% "add one more plan_astar-style function plus one more pair of
% plan_call/8 clauses" recipe this section's own header comment
% already anticipated. Algorithm here is a COMPOUND term, not a bare
% atom like astar/straight -- ObstacleId (which obstacle_polygon/2 to
% circle) and Offset (how far out to stay from its boundary) are
% carried INSIDE Algorithm itself, so planWith/do_node/planned_with
% need NO interface change at all: they already treat Algorithm as
% opaque. Offset is typically unified with the SAME Threshold as
% whichever obstacle_on_path(Threshold)/obstacle_in_bound(Threshold)
% trigger or condition supplied ObstacleId in the first place.
%
% UNLIKE an earlier version of this planner, follow_boarder does NOT
% decide when to stop circling -- it plans a FULL clockwise loop around
% ObstacleId's offset boundary (planners.py's follow_boarder/5),
% always succeeding for a known obstacle (Reason=completed regardless
% of Goal, since there is no Goal-relative stopping decision to make
% here at all -- notice the /5 arity below has no GX,GY). WHEN to
% actually leave the boundary and hand off to a straight-line planner
% is entirely the job of whichever trigger halts the SUBSEQUENT
% moveto_leg(CP,[...]) using this CP -- line_of_sight_clear(ObstacleId,
% GX,GY) for Bug0, crosses_segment(SX,SY,GX,GY) for Bug2 (see the
% TRIGGERS section above) -- so the bug-variant choice is a matter of
% that Triggers list, not a different planner call. If the attached
% trigger never fires, the leg just completes the whole loop naturally
% and Status comes out true via the ordinary leg_status mechanism --
% leg_status/9 no longer judges whether the final position is close to
% the leg's own endpoint (see that predicate's own note: that check is
% now the separate, explicit distance_below/3 BT condition), so a fully
% -circled loop with nothing meaningful downstream of it simply reads
% as a completed leg, same as any other MoveTo. No special no_path case
% needed here for that.
plan_call(follow_boarder(ObstacleId,Offset), SX,SY,_GX,_GY, CP, completed, true) :-
    follow_boarder(SX,SY,ObstacleId,Offset, CP).
plan_call(follow_boarder(ObstacleId,Offset), SX,SY,_GX,_GY, [], no_path, false) :-
    \+ follow_boarder(SX,SY,ObstacleId,Offset, _).

% plan_waypoints_call(+Algorithm,+SX,+SY,+Waypoints,-CP,-Reason,-Status):
% the MULTI-WAYPOINT generalization of plan_call/8 just above -- SAME
% "Reason/Status bound together, one clause pair per Algorithm" shape,
% but for astar/straight ONLY (voronoi/follow_boarder have no multi-
% waypoint form; see PlanWithWaypoints' own schema.yaml entry for why
% this is scoped that way "for now"). Waypoints is a Prolog list,
% [point(G1X,G1Y), point(G2X,G2Y), ...], at least one point -- plans
% (SX,SY) -> Waypoints[0] -> Waypoints[1] -> ... -> the LAST point,
% via planners.py's own plan_astar_waypoints/plan_straight_waypoints
% (which chain the SAME single-goal plan_astar/plan_straight this
% file already calls above, concatenating each leg's own control
% points into ONE combined chain -- see that module's own "MULTI-
% WAYPOINT MERGING" section header for the full rationale and the
% concatenation arithmetic). FAILS OUTRIGHT (no_path, CP=[]) if ANY
% single leg between consecutive waypoints has no path -- there is no
% partial-credit result, exactly like plan_call/8's own astar/straight
% clauses already fail outright rather than partially.
plan_waypoints_call(astar, SX,SY, Waypoints, CP, completed, true) :-
    plan_astar_waypoints(SX,SY, Waypoints, CP).
plan_waypoints_call(astar, SX,SY, Waypoints, [], no_path, false) :-
    \+ plan_astar_waypoints(SX,SY, Waypoints, _).

plan_waypoints_call(straight, SX,SY, Waypoints, CP, completed, true) :-
    plan_straight_waypoints(SX,SY, Waypoints, CP).
plan_waypoints_call(straight, SX,SY, Waypoints, [], no_path, false) :-
    \+ plan_straight_waypoints(SX,SY, Waypoints, _).

% -- PLANNING leaf: planWithWaypoints(Algorithm,Waypoints,CP,ActionCode)
%    -- the multi-waypoint sibling of planWith/4 just below, SAME shape
%    (quantize current position via disc_step_position/1, dispatch via
%    a plan_*_call predicate, tag_reason the result), just calling
%    plan_waypoints_call/7 instead of plan_call/8 and threading
%    Waypoints straight through as BOTH the call's own third argument
%    AND the "Goal" slot of the recorded Reason (completed(Algorithm,
%    Waypoints,ActionCode) / no_path(Algorithm,Waypoints,ActionCode)) --
%    a list is just another Prolog term as far as tag_reason/3's own
%    generic =.. reconstruction and halted_with_pattern/3's own
%    generic match_wild/2 are concerned, so nothing downstream needed
%    any change for a Reason whose second argument happens to be a
%    list of points, not a single point/2 term.
%
%    THE WHOLE POINT of this action, stated plainly: a plan with N
%    separate planWith/4 + moveto_leg/2 legs draws N independent z/zt
%    noise-variable pairs (once per leg -- see basic_action_theory.pl's
%    top-of-file FUTUREWORK.md reference and planners.py's own "MULTI-
%    WAYPOINT MERGING" section header), all of which stay simultaneously
%    "live" through Reiter regression once the plan's own final
%    situation is reached, driving up the compiled ProbLog formula's
%    size. Collapsing those SAME N legs into ONE planWithWaypoints +
%    ONE moveto_leg (see bt_to_prolog.py's own automatic merge pass,
%    which detects exactly this pattern -- a Sequence of Sequence(
%    PlanWith,MoveTo) siblings sharing one algorithm -- and rewrites it
%    this way without the tree author needing to hand-author
%    PlanWithWaypoints at all) draws exactly ONE z/zt pair for the
%    WHOLE merged route instead of N, directly shrinking the grounded/
%    compiled formula. TRADEOFF, stated equally plainly: this is a
%    genuine change to the probabilistic MODEL, not just an
%    implementation optimization -- positional noise is no longer
%    independently reset at each original waypoint (N independent
%    "restart points" become ONE noise realization governing drift
%    across the entire merged route). See PlanWithWaypoints' own
%    schema.yaml entry for this same tradeoff from the BT-author's
%    side.
do_node(planWithWaypoints(Algorithm, Waypoints, CP, ActionCode), S,
        do(planned(Algorithm,Reason), S), Status) :-
    now(T, S), at(SXExact,SYExact,T,S),
    disc_step_position(Grid),
    quantize(SXExact, Grid, SX), quantize(SYExact, Grid, SY),
    plan_waypoints_call(Algorithm, SX,SY, Waypoints, CP, Reason0, Status),
    Reason0 =.. [Functor],
    Reason1 =.. [Functor, Algorithm, Waypoints],
    tag_reason(Reason1, ActionCode, Reason).

% -- PLANNING leaf: ONE template, planWith(Algorithm,Goal,CP), covering
%    every planner via plan_call/8's own dispatch on Algorithm --
%    NOT two (or more) separate hand-written do_node clauses. Called
%    right before a moveto_leg sharing the SAME CP variable, e.g.:
%        seq_node([planWith(astar,point(17.0,17.0),CP), moveto_leg(CP,[collision,battery])])
%    -- exactly the same "leave a variable free, let it get bound by
%    whichever step derives it" pattern already used for auto-timing
%    T0 across chained legs (see now/2's role in Poss(startMoveto...)).
%    Current position comes from now/2 + at/4 (i.e. "plan from HERE,
%    right now"); Goal is EXPLICIT, an argument of planWith itself
%    (point(GX,GY)), NOT read from the global goal/2 fact -- this is
%    what makes it possible for TWO planWith calls in the SAME plan to
%    target genuinely DIFFERENT destinations (e.g. a "go to P1, then
%    go to P2" multi-leg plan: seq_node([planWith(astar,point(P1x,P1y),CP1),
%    moveto_leg(CP1,...), planWith(astar,point(P2x,P2y),CP2),
%    moveto_leg(CP2,...)])) -- the direct Prolog analogue of
%    instantiating a parametrized "GoTo(target)" BT.cpp subtree twice
%    with two different port bindings, rather than both calls silently
%    aiming at one shared destination (there is no longer a global
%    goal/2 fact at all -- see distance_below/3's own note, and plan_generation
%    /plan/goal_formula.pl for where a plan's own goal information
%    lives now). Status flows
%    straight through as Outcome, exactly like moveto_leg's own Status
%    -- no translation predicate, since plan_call/8 already produces
%    it directly.
%
%    UNLIKE the pure-lookup plan_call/8 it wraps, this DOES extend the
%    situation -- with a bare planned(Algorithm,Reason) MARKER, not a
%    real primitive action (no Poss, no primitive_action entry; the
%    marker is simply CONSTRUCTED via do(...) the same way do_action
%    itself would, minus the precondition gate there's nothing to
%    gate). This is the lightweight piece needed for TRACEABILITY once
%    a plan calls planning MORE THAN ONCE (e.g. a fallback_node trying
%    astar then straight, or several legs each replanning from wherever
%    the previous one ended) -- otherwise there would be nothing in a
%    no-path failure's situation to distinguish it from any other, or
%    to tell WHICH of several planning attempts is which. The marker
%    is transparent to every existing fluent: at/4, battery/3,
%    moving/1, now/2, current_walk/5, and last_action_time/4 ALL
%    already pass through any action other than startMoveto/
%    haltMoveto/interrupt unchanged, so recording it changes nothing
%    about elapsed time, position, or battery -- still genuinely
%    instantaneous, just no longer INVISIBLE to later inspection.
%
%    IMPORTANT GOTCHA when using planWith inside a fallback_node with
%    SEPARATE algorithms per branch: give EACH branch its OWN CP
%    variable, e.g.
%        fallback_node([seq_node([planWith(astar,point(GX,GY),CP1,a1), moveto_leg(CP1,...)]),
%                        seq_node([planWith(straight,point(GX,GY),CP2,a2), moveto_leg(CP2,...)])])
%    NOT a single CP variable shared across both branches. Reusing one
%    CP across fallback alternatives silently breaks: a FAILING
%    planWith still SUCCEEDS as a do_node call (with Outcome=false,
%    CP=[]) -- do_node calls don't get "undone" on Outcome=false the
%    way a genuine Prolog failure would -- so CP is left bound to []
%    by the first branch, and the SECOND branch's own attempt to bind
%    CP to its own (non-empty) result then fails to unify, breaking
%    the fallback in a confusing way that looks unrelated to variable
%    scoping. This was hit directly while testing this exact feature.
% SX,SY are rounded to disc_step_position/1's own granularity (see
% the MERGE-GRID QUANTIZATION note above dist/5's own section) before
% being handed to plan_call/plan straight/plan_astar/... -- this is
% what actually makes CP (hence the NEW leg's own startMoveto(CP,...)
% term) merge-friendly: since a planner's own path always begins
% EXACTLY at the point it's given, quantizing the point handed in here
% is enough on its own to make every downstream CP identical across
% worlds that round to the same grid cell -- no separate CP-level
% rounding needed. Every OTHER read of at/4 (collision detection,
% goal-tolerance checking in leg_status, first_hit/on_track/
% verify_safe reporting) stays exact, unaffected -- only the position
% a NEW leg gets planned FROM is coarsened.
%
% ActionCode (a FOURTH argument now, one per planWith OCCURRENCE in
% the tree, from the SAME var_pool.next_action_code() counter MoveTo's
% own ActionCode already comes from -- codes stay unique tree-wide
% regardless of node kind) is baked into the RECORDED Reason via
% tag_reason/3, the EXACT same generic mechanism poss(haltMoveto(...))
% already uses -- see that predicate's own note. Reason0 out of
% plan_call/8 is always the bare atom completed/no_path; before
% tagging, it's first rebuilt (via =..) to also carry Algorithm and
% the point this call was actually working toward, so the FINAL
% recorded Reason is completed(Algorithm,Goal,ActionCode) or
% no_path(Algorithm,Goal,ActionCode) -- ActionCode LAST is not
% arbitrary, it's what lets halted_with_pattern/3 (built generically
% on "the recorded Reason's own trailing argument is ActionCode",
% see that predicate's own note) work for planning outcomes with NO
% change of its own. TWO mutually exclusive clauses below, split on
% Algorithm's own shape, ONLY because follow_boarder has no real Goal
% point to report (see follow_boarder(ObstacleId,Offset)'s own note
% above plan_call/8) --
% embedding the harmless point(0.0,0.0) placeholder threaded through
% planWith's own second argument for template-sharing purposes would
% be misleading if surfaced in a Reason meant to be read by a human/
% query, so that case reports the honest atom `none` instead. The
% explicit Algorithm \= follow_boarder(_,_) guard on the second clause
% is NOT redundant with the first clause's own head shape: without it,
% a follow_boarder call's own point(0.0,0.0) placeholder Goal argument
% would ALSO unify against the second clause's point(GX,GY) head
% pattern, giving ProbLog TWO derivations of the same fact instead of
% one -- exactly the kind of silent solution-doubling this project has
% already hit once (see match_wild/2's own history note) and now
% checks for on purpose.
do_node(planWith(follow_boarder(ObstacleId,Offset), _Goal, CP, ActionCode), S,
        do(planned(follow_boarder(ObstacleId,Offset),Reason), S), Status) :-
    now(T, S), at(SXExact,SYExact,T,S),
    disc_step_position(Grid),
    quantize(SXExact, Grid, SX), quantize(SYExact, Grid, SY),
    plan_call(follow_boarder(ObstacleId,Offset), SX,SY,_GX,_GY, CP, Reason0, Status),
    Reason0 =.. [Functor],
    Reason1 =.. [Functor, follow_boarder(ObstacleId,Offset), none],
    tag_reason(Reason1, ActionCode, Reason).
do_node(planWith(Algorithm, point(GX,GY), CP, ActionCode), S,
        do(planned(Algorithm,Reason), S), Status) :-
    Algorithm \= follow_boarder(_,_),
    now(T, S), at(SXExact,SYExact,T,S),
    disc_step_position(Grid),
    quantize(SXExact, Grid, SX), quantize(SYExact, Grid, SY),
    plan_call(Algorithm, SX,SY,GX,GY, CP, Reason0, Status),
    Reason0 =.. [Functor],
    Reason1 =.. [Functor, Algorithm, point(GX,GY)],
    tag_reason(Reason1, ActionCode, Reason).

% -- QUERY leaves: ToolPosition/ToolsOfKind/NearestToolOfKind --------
% Same "pure computation, no primitive_action/poss layer" shape as
% planWith/4 just above and cond(C,Code) further down -- read-only,
% side-effect-free lookups against tool_position/4, tools_of_kind/5,
% nearest_tool_of_kind/6 (Section 5d), with no precondition of their
% own to state (they can be asked ANYTIME, unlike a durative action).
% Each gets its OWN distinct marker functor (tool_position_result/1,
% tools_of_kind_result/1, nearest_tool_result/1), mirroring planned/2's
% own "record what happened" role, but kept SEPARATE per query type
% (rather than one shared marker the way every PlanWith algorithm
% shares planned/2) since these three have different output shapes --
% sharing one marker would make halted_with/2's own pattern-match
% ambiguous across them. TWO mutually exclusive clauses each (found vs
% not), same "no if-then-else" convention every other multi-case
% predicate in this file already uses.
do_node(tool_position_query(Id,Pos,ActionCode), S, do(tool_position_result(Reason), S), true) :-
    tool_position(Id, GX, GY, S),
    Pos = point(GX,GY),
    tag_reason(tool_position_found(Id), ActionCode, Reason).
do_node(tool_position_query(Id,_Pos,ActionCode), S, do(tool_position_result(Reason), S), false) :-
    \+ tool_position(Id, _, _, S),
    tag_reason(tool_position_unavailable(Id), ActionCode, Reason).

do_node(tools_of_kind_query(Kind,Tools,ActionCode), S, do(tools_of_kind_result(Reason), S), true) :-
    findall(tool(Id,point(GX,GY)), tools_of_kind(Kind,S,Id,GX,GY), Tools),
    Tools \= [],
    tag_reason(tools_of_kind_found(Kind), ActionCode, Reason).
do_node(tools_of_kind_query(Kind,Tools,ActionCode), S, do(tools_of_kind_result(Reason), S), false) :-
    findall(tool(Id,point(GX,GY)), tools_of_kind(Kind,S,Id,GX,GY), Tools),
    Tools == [],
    tag_reason(tools_of_kind_empty(Kind), ActionCode, Reason).

do_node(nearest_tool_of_kind_query(Kind,Id,Pos,ActionCode), S, do(nearest_tool_result(Reason), S), true) :-
    nearest_tool_of_kind(Kind, S, Id, GX, GY, _Dist),
    Pos = point(GX,GY),
    tag_reason(nearest_tool_found(Kind,Id), ActionCode, Reason).
do_node(nearest_tool_of_kind_query(Kind,_Id,_Pos,ActionCode), S, do(nearest_tool_result(Reason), S), false) :-
    \+ nearest_tool_of_kind(Kind, S, _, _, _, _),
    tag_reason(no_tool_of_kind(Kind), ActionCode, Reason).

% hitched_id_query(-Id,+ActionCode): a FOURTH query leaf, same "pure
% computation, no primitive_action/poss layer" shape as the three
% above -- outputs the id of whatever tool is CURRENTLY hitched (Section
% 5c's own hitch_id/2, a plain persistent fluent, single-valued: either
% free or exactly one instance id), fails (Status=false) if nothing is.
% Exists specifically so a tree can say "deploy/retract/uninstall
% WHATEVER is hitched right now" without needing to have plumbed that
% id in from wherever it originally got installed -- e.g. a plow that
% was ALREADY hitched before a "make sure it's installed" guard even
% ran never flows through an InstallTool/NearestToolOfKind occurrence
% at all in THIS run, so there is no earlier output port to reuse; this
% queries hitch_id/2 fresh, works regardless of how the tool got
% hitched. TWO mutually exclusive clauses (hitch_id(free,S) is a total,
% single-valued relation -- never BOTH free and some Id at once -- so,
% unlike tool_position_query's own \+ tool_position(...) false clause,
% the false clause here can match hitch_id(free,S) directly, no
% negation needed).
do_node(hitched_id_query(Id,ActionCode), S, do(hitched_id_result(Reason), S), true) :-
    hitch_id(Id, S),
    Id \= free,
    tag_reason(hitched_id_found(Id), ActionCode, Reason).
do_node(hitched_id_query(_Id,ActionCode), S, do(hitched_id_result(Reason), S), false) :-
    hitch_id(free, S),
    tag_reason(hitched_id_unavailable, ActionCode, Reason).

% planned_with(+Algorithm, +Reason, +S): the direct parallel to
% halted_with/2, for the (now-recorded) planning marker. Searches the
% WHOLE history, so it can distinguish which of SEVERAL planning
% attempts (across different legs, or different fallback branches)
% produced a given Reason, and with which algorithm -- though Reason
% ITSELF now already carries Algorithm and ActionCode too (see
% do_node(planWith(...))'s own note), so this predicate's own
% Algorithm argument is redundant with Reason's own first argument in
% practice; kept as-is since dropping it would be a needless interface
% change for something already unambiguous.
planned_with(Algorithm, Reason, do(planned(Algorithm,Reason), _)).
planned_with(Algorithm, Reason, do(_A, S)) :- planned_with(Algorithm, Reason, S).

% checked_with(+Code, -C, -Status, +S): the direct analogue of
% planned_with/3 above, for cond(C,Code)'s own checked(Code,C,Status)
% marker -- searches the WHOLE history for Code (same "search
% everything, one solution per match" shape halted_with/2 and
% planned_with/3 both already use), NOT just the most recent one: a
% cond() leaf sitting inside a reactive_children/2 list can be
% re-checked on every redescend, so a given Code can legitimately carry
% more than one (possibly different) Status across a single resolved
% world's own history -- each is reported as its own solution, exactly
% mirroring how halted_with/2 already handles a Reason that could, in
% principle, recur. Fails outright (no solution) for a Code whose own
% cond() leaf was never reached in a given resolved world at all (e.g.
% it sits in a Fallback branch that never got tried) -- same "absence,
% not sentinel" convention as everywhere else in this file.
checked_with(Code, C, Status, do(checked(Code,C,Status), _)).
checked_with(Code, C, Status, do(_A, S)) :- checked_with(Code, C, Status, S).

% -- SEQUENCE composite: stop and FAIL at the first failing child; --
%    succeed only if every child succeeds, in order. A REACTIVE child
%    (see leg_status/9's own note on the three-valued Status) stops
%    the sequence too, same as false -- but is NOT the same as false:
%    it propagates straight through, UNCHANGED (Code and all -- this
%    clause never inspects WHICH code it is, unlike reactivesequence/
%    reactivefallback below), to whatever node contains THIS seq_node
%    -- see the CONTROL-FLOW REDESCEND TARGETS note below for the full
%    picture of why and where this eventually gets caught. A plain
%    seq_node NEVER catches/redescends on its own, by design -- exactly
%    matching real BT.cpp's own plain Sequence, which a ReactiveSequence
%    is a genuinely DIFFERENT node type from, not a special case of.
do_node(seq_node([]), S, S, true).
do_node(seq_node([Child|Rest]), S, S1, Outcome) :-
    do_node(Child, S, S2, true),
    do_node(seq_node(Rest), S2, S1, Outcome).
do_node(seq_node([Child|_]), S, S1, false) :-
    do_node(Child, S, S1, false).
do_node(seq_node([Child|_]), S, S1, reactive(Code)) :-
    do_node(Child, S, S1, reactive(Code)).

% -- FALLBACK (Selector) composite: stop and SUCCEED at the first ---
%    succeeding child; fail only if every child fails, in order.
%    NOTE: on a failing child, the NEXT child starts from THAT
%    child's resulting situation, not from the original S -- a failed
%    PHYSICAL action (e.g. a crashed moveto_leg) still consumed real
%    time and moved the robot; unlike a classical BT's usual
%    assumption that failed leaves are side-effect-free, a fallback
%    over durative ACTIONS here means "try the next option from
%    wherever the failed attempt left us," not "rewind and try the
%    next option from the start." A REACTIVE child does NOT try the
%    next sibling this way -- same as seq_node above, it propagates
%    straight through unchanged instead (see the CONTROL-FLOW
%    REDESCEND TARGETS note below).
do_node(fallback_node([]), S, S, false).
do_node(fallback_node([Child|_]), S, S1, true) :-
    do_node(Child, S, S1, true).
do_node(fallback_node([Child|Rest]), S, S1, Outcome) :-
    do_node(Child, S, S2, false),
    do_node(fallback_node(Rest), S2, S1, Outcome).
do_node(fallback_node([Child|_]), S, S1, reactive(Code)) :-
    do_node(Child, S, S1, reactive(Code)).

% -- INVERTER decorator: BT.cpp's built-in single-child negation ------
%    node (<Inverter>C</Inverter>). Flips true<->false; a REACTIVE
%    child's reactive(Code) status passes straight through UNCHANGED,
%    same "never inspects, never catches" passthrough seq_node/
%    fallback_node's own reactive clauses already do -- an interrupt
%    signal isn't a true/false outcome to negate, it's a request to be
%    caught by a MATCHING reactivesequence(Code)/reactivefallback(Code)
%    further up, wherever that is; an Inverter never IS one (it takes
%    no Code of its own -- see _REACTIVE_CONTROL_FLOW's own note in
%    bt_to_prolog.py for why the guard-derivation machinery already
%    treats Inverter as fully transparent for that purpose too, not
%    just for do_node's own control flow).
do_node(inverter(Child), S, S1, false) :-
    do_node(Child, S, S1, true).
do_node(inverter(Child), S, S1, true) :-
    do_node(Child, S, S1, false).
do_node(inverter(Child), S, S1, reactive(Code)) :-
    do_node(Child, S, S1, reactive(Code)).

% ---------------------------------------------------------------
% CONTROL-FLOW REDESCEND TARGETS -- reactivesequence(Code) and
% reactivefallback(Code) are the ONLY two node types that ever catch a
% reactive(_) status and act on it; plain seq_node/fallback_node above
% NEVER do (they just pass it straight up, unconditionally, forever).
% This is the direct Prolog analogue of BT.cpp's own real distinction
% between Sequence/Fallback (checked once, never re-monitored) and
% ReactiveSequence/ReactiveFallback (continuously re-ticked) -- see
% this project's own conversation log for the fuller discussion this
% design came out of.
%
% Code is a plain atom (bt_to_prolog.py assigns one, e.g. rc1, rc2,
% ..., to each ReactiveSequence/ReactiveFallback it translates,
% uniquely across the whole tree) IDENTIFYING one specific reactive
% composite. Every reactive-classified trigger name in the Triggers
% list of every MoveTo leg underneath it (obstacle_in_bound(Threshold,
% Code), battery_below(Threshold,Code), etc. -- see trigger_crossing_
% time/11's own note) is tagged, at translation time, with THIS SAME
% Code -- the code of its own NEAREST enclosing ReactiveSequence/
% ReactiveFallback (an intervening plain seq_node/fallback_node
% doesn't matter, since it never inspects Code at all -- reactive(Code)
% passes straight through it unchanged either way). So when a leg
% halts reactively, reactive(Code) bubbles up through zero or more
% plain composites, completely inert, until it reaches the ONE
% composite whose own Code matches -- which is, by construction,
% always its own nearest enclosing reactive composite -- and THAT one
% catches it.
%
% "Catching" means: instead of propagating further, re-run this same
% composite's own children FRESH -- see reactive_children/2 (a
% SEPARATE fact per reactive composite, one row per Code, generated by
% bt_to_prolog.py) and why it has to be a separately-resolved fact,
% not the SAME children term reused across restarts: exactly the
% "planWith inside a fallback_node" gotcha documented above (a second
% pass's own planWith would try to unify a genuinely different control-
% point list against an ALREADY-BOUND CP left over from the first
% pass, and fail outright) -- reactive_children/2 resolved as a FRESH
% goal each time gives genuinely unbound variables on every pass,
% exactly mirroring how plan(Node) itself already works for the
% (now removed) whole-tree redescend evaluate_plan/4 used to do.
%
% Each reactive composite carries its OWN budget, read fresh from
% replan_budget/1 on first entry (see reactivesequence_budgeted/5
% below) and decremented on every restart -- exhausting it resolves to
% world_too_large right there rather than continuing to restart or
% propagating upward (mirrors evaluate_plan/4's own former clause 3,
% just localized). This budget is what stands between a degenerate
% zero-duration leg and an infinite restart loop -- see this project's
% own conversation log for why a PER-composite budget is needed now
% that redescend can happen below the root, not just at it.
%
% A reactive(_) that reaches THE ROOT of the whole tree without any
% composite's own Code ever matching means bt_to_prolog.py tagged a
% leg's own reactive trigger with a Code that no enclosing
% ReactiveSequence/ReactiveFallback actually carries -- a TRANSLATOR
% BUG (that generator also validates this at translation time, as a
% hard failure, so this should never actually happen at runtime) --
% see plan_outcome/1's own reactive_escaped clause further down for
% how this is reported if it somehow does.
do_node(reactivesequence(Code), S, S1, Outcome) :-
    replan_budget(Budget),
    reactivesequence_budgeted(Code, Budget, S, S1, Outcome).

reactivesequence_budgeted(Code, Budget, S, S1, Outcome) :-
    Budget > 0,
    reactive_children(Code, Children),
    do_node(seq_node(Children), S, S2, reactive(Code)),
    Budget1 is Budget - 1,
    reactivesequence_budgeted(Code, Budget1, S2, S1, Outcome).
reactivesequence_budgeted(Code, 0, S, S, world_too_large) :-
    reactive_children(Code, Children),
    do_node(seq_node(Children), S, _, reactive(Code)).
reactivesequence_budgeted(Code, Budget, S, S1, reactive(OtherCode)) :-
    Budget > 0,
    reactive_children(Code, Children),
    do_node(seq_node(Children), S, S1, reactive(OtherCode)),
    OtherCode \= Code.
reactivesequence_budgeted(Code, Budget, S, S1, Outcome) :-
    Budget > 0,
    reactive_children(Code, Children),
    do_node(seq_node(Children), S, S1, Outcome),
    Outcome \= reactive(_).

do_node(reactivefallback(Code), S, S1, Outcome) :-
    replan_budget(Budget),
    reactivefallback_budgeted(Code, Budget, S, S1, Outcome).

reactivefallback_budgeted(Code, Budget, S, S1, Outcome) :-
    Budget > 0,
    reactive_children(Code, Children),
    do_node(fallback_node(Children), S, S2, reactive(Code)),
    Budget1 is Budget - 1,
    reactivefallback_budgeted(Code, Budget1, S2, S1, Outcome).
reactivefallback_budgeted(Code, 0, S, S, world_too_large) :-
    reactive_children(Code, Children),
    do_node(fallback_node(Children), S, _, reactive(Code)).
reactivefallback_budgeted(Code, Budget, S, S1, reactive(OtherCode)) :-
    Budget > 0,
    reactive_children(Code, Children),
    do_node(fallback_node(Children), S, S1, reactive(OtherCode)),
    OtherCode \= Code.
reactivefallback_budgeted(Code, Budget, S, S1, Outcome) :-
    Budget > 0,
    reactive_children(Code, Children),
    do_node(fallback_node(Children), S, S1, Outcome),
    Outcome \= reactive(_).

% ---------------------------------------------------------------
% holds/2: minimal condition language for cond(C) leaves -- standard
% logical combinators plus domain-specific atomic conditions. Extend
% with more atomic conditions as new leaf/condition types are needed;
% nothing about do_node's cond(C) clause above needs to change.
% ---------------------------------------------------------------
holds(and(P,Q), S) :- holds(P,S), holds(Q,S).
holds(or(P,Q),  S) :- holds(P,S) ; holds(Q,S).
holds(neg(P),   S) :- \+ holds(P,S).

% halted_with_cond(Reason): reads the LAST halt's Reason via
% halted_with/2 -- lets a cond() leaf branch on how the PREVIOUS leg
% ended, e.g. cond(halted_with_cond(battery_depleted)). ACTION-
% INDEPENDENT (see halted_with/2's own note): Reason can just as well
% be a planning call's own completed(Algorithm,Goal,Code)/no_path
% (Algorithm,Goal,Code), e.g. cond(halted_with_cond(no_path(_,_,_)))
% to branch on "did the last planning attempt fail, regardless of
% which algorithm/goal" -- same wildcard convention as every other
% Reason shape below.
%
% TODO / KNOWN INTERFACE CHANGE: crashed/obstacle_in_bound/battery_under
% Reasons are compound terms carrying extra info (crashed(ObstacleId),
% obstacle_in_bound(Threshold,ObstacleId), battery_under(Threshold) --
% see trigger_crossing_time/9's own note), not bare atoms -- battery_depleted
% stays a bare atom, unaffected. To match ANY crash regardless of
% obstacle, write cond(halted_with_cond(crashed(_))), NOT
% cond(halted_with_cond(crashed)) (which no longer unifies against
% anything: a bare atom never matches a compound term of the same
% name); similarly obstacle_in_bound(_,_) / battery_under(_) for "any
% threshold/obstacle". To match SPECIFIC values, write e.g.
% cond(halted_with_cond(crashed(obs5))) or
% cond(halted_with_cond(battery_under(20))). In a BT.cpp XML tree (see
% module/translators/bt_to_prolog.py), this is HaltedWith's reason port,
% written the same way: reason="crashed(_)" or reason="crashed(obs5)".
holds(halted_with_cond(Reason), S) :- halted_with(Reason, S).

% distance_below(GX,GY,Threshold) / distance_equal(GX,GY,Threshold) /
% distance_over(GX,GY,Threshold): true iff the CURRENT position (at
% the current time, via now/2) is, respectively, below/exactly-equal-
% to/above Threshold distance from the EXPLICIT point (GX,GY) --
% PARAMETRIZED, same as obstacle_in_bound(Threshold)/battery_below
% (Threshold)/etc., not a lookup against any global "the goal" fact
% (there is no such fact anymore -- see the problem's own
% goal_formula.pl for where a plan's own goal information now lives
% entirely; distance_below is a DIFFERENT, complementary thing: a
% REACTIVE in-tree check, evaluated possibly many times at different
% situations as the policy runs, not a one-time post-hoc verification
% query). This is also what used to be baked directly into
% moveto_leg's own Status output (see leg_status/9's own note) --
% factored out into its own explicit, inspectable condition node
% instead, same as any other cond() leaf.
% Typical use: a fallback child that skips moveto entirely if already
% there, or a check placed right after a MoveTo to confirm it actually
% landed close enough to its own intended target --
%   fallback_node([cond(distance_below(11.675,11.525,0.3)), moveto_leg(CP,[collision,battery])])
% Pass the SAME point as whichever PlanWith node's own
% goal port targets, if that's the intent -- being explicit here means
% there is no longer a global/local goal-point mismatch to drift out
% of sync (the risk a single shared goal/2 fact used to carry).
% distance_equal/distance_over exist for the SAME reason
% battery_equal/battery_over exist alongside battery_below: a
% caller-chosen threshold at a different comparison, not a
% replacement.
holds(distance_below(GX,GY,Threshold), S) :-
    now(T, S), at(X,Y,T,S), dist(X,Y,GX,GY,D), D < Threshold.
holds(distance_equal(GX,GY,Threshold), S) :-
    now(T, S), at(X,Y,T,S), dist(X,Y,GX,GY,D), D =:= Threshold.
holds(distance_over(GX,GY,Threshold), S) :-
    now(T, S), at(X,Y,T,S), dist(X,Y,GX,GY,D), D > Threshold.

% point(GX,GY)-term adapters -- lets a <DistanceBelow>/<DistanceEqual>/
% <DistanceOver>'s own goal port be a BLACKBOARD REFERENCE (e.g.
% goal="{p1}", wired from a SubTree's own input port, or from
% NearestToolOfKind's own position output) instead of only a literal
% "X;Y", the same "either a literal or a blackboard reference bound to
% point(GX,GY) at runtime" treatment PlanWith's own goal port already
% gets (see planWith's own do_node(planWith(Algorithm,point(GX,GY),
% CP,ActionCode),...) clause). module/translators/bt_to_prolog.py's own
% _leaf_condition_term emits the flat 3-arg form above ONLY for a
% literal goal; for a blackboard-ref goal it emits this 2-arg form
% instead, with Point left as the bare Prolog variable that goal's own
% producer binds to point(GX,GY) -- these three clauses just unify that
% back apart and delegate to the existing flat-arg clauses above, so
% those (and every existing caller depending on their own 3-arg shape,
% e.g. Section 5c's own "close to the tool" reuse) are unchanged.
holds(distance_below(point(GX,GY),Threshold), S) :-
    holds(distance_below(GX,GY,Threshold), S).
holds(distance_equal(point(GX,GY),Threshold), S) :-
    holds(distance_equal(GX,GY,Threshold), S).
holds(distance_over(point(GX,GY),Threshold), S) :-
    holds(distance_over(GX,GY,Threshold), S).

% sample_value_below(SampleId,Threshold) / sample_value_equal(SampleId,
% Threshold) / sample_value_over(SampleId,Threshold): true iff the
% VALUE (0..10, see sample_value/3) a SUCCESSFUL take_sample recorded
% under SampleId -- the tree author's own id port, schema.yaml's own
% TakeSample entry, NOT the auto-generated ActionCode -- is,
% respectively, below/exactly-equal-to/above Threshold. Reads back out
% of halted_with/2 (searches S's WHOLE history, same "history-based,
% not a live fluent" shape halted_with_cond/1 itself has), NOT a fresh
% dist/5-style computation the way distance_below/3 above is -- a
% sample's value is fixed the INSTANT it's drawn (do_node(take_sample
% (...)) is INSTANTANEOUS, no Duration to elapse), so there is nothing
% to re-derive at a later situation, only something to look up. Fails
% outright (same as every OTHER condition here) if SampleId never had a
% SUCCESSFUL sample in S's history at all -- a failed or never-taken
% sample has no value to compare, not a special error case.
%
% NON-CONTINUOUS for the SAME reason halted_with_cond/1 is (see
% bt_to_prolog.py's own _NON_CONTINUOUS_CONDITIONS note): a value that
% cannot change WHILE a leg is running has nothing for a reactive
% crossing-search to watch for, so these three are excluded from
% automatic guard-trigger derivation, same treatment HaltedWith gets.
holds(sample_value_below(SampleId,Threshold), S) :-
    halted_with(sample_success(_X,_Y,V,SampleId,_ActionCode), S),
    V < Threshold.
holds(sample_value_equal(SampleId,Threshold), S) :-
    halted_with(sample_success(_X,_Y,V,SampleId,_ActionCode), S),
    V =:= Threshold.
holds(sample_value_over(SampleId,Threshold), S) :-
    halted_with(sample_success(_X,_Y,V,SampleId,_ActionCode), S),
    V > Threshold.

% sample_value_below/3, sample_value_equal/3, sample_value_over/3 (bare,
% NOT holds(...)-wrapped): thin pass-throughs so a goal_formula.pl can
% reference "was SampleId's own reading over Threshold" directly, the
% same way it already calls hitch/2 or sample_success_at/3 straight,
% without needing holds/2 itself to be a goal-formula-callable
% predicate (see vocabulary.yaml's own entries for these three and
% module/contracts/goal_formula_check.py's own vocabulary-lookup, which
% checks bare predicate names/arities, not holds(...)-wrapped ones).
sample_value_below(SampleId, Threshold, S) :- holds(sample_value_below(SampleId,Threshold), S).
sample_value_equal(SampleId, Threshold, S) :- holds(sample_value_equal(SampleId,Threshold), S).
sample_value_over(SampleId, Threshold, S) :- holds(sample_value_over(SampleId,Threshold), S).

% hitched / hitched(Kind): the BT-tree-facing CONDITION wrapper around
% the ALREADY-existing hitch/2 fluent (Section 5c) -- hitch/2 itself is
% a plain relational fluent, not holds(...)-wrapped, so a cond(C,Code)
% BT leaf (which always dispatches through holds/2) needs this thin
% bridge to use it, the same reason sample_value_below/3's own holds(
% ...) clause exists alongside the bare wrapper, just in the opposite
% direction here (bare fluent -> holds(...), not the other way round).
% hitched (no argument) is TRUE iff ANYTHING is currently attached
% (\+ hitch(free,S)) -- "is the hitch busy". hitched(Kind) checks ONE
% SPECIFIC kind instead (cart/plow) -- "is it busy WITH THIS kind".
% Both are NON-CONTINUOUS (see bt_to_prolog.py's own _NON_CONTINUOUS_
% CONDITIONS note) -- hitch/2 is PROVABLY constant for the whole span
% of any single MoveTo leg (only a successful (un)install can change
% it, and both require \+ moving(S) to even start), so there is
% nothing for a reactive crossing-search to watch mid-leg.
holds(hitched, S) :- \+ hitch(free, S).
holds(hitched(Kind), S) :- hitch(Kind, S).

% deployed (bare atom, zero-arg CONDITION term -- NOT the same thing as
% the deployed/1 FLUENT it wraps, disambiguated by arity/position the
% same way every other bare-atom condition already is): the BT-tree-
% facing wrapper around deployed/1 (Section 5d), same reasoning as
% hitched/0 above. No Kind argument -- only one tool can ever be
% hitched at a time, so "is the CURRENTLY-hitched tool deployed" needs
% no further qualification, mirroring deployed/1's own shape exactly.
% Also NON-CONTINUOUS, for the same reason hitched/0 is (deploy_tool/
% retract_tool also require \+ moving(S) to start).
holds(deployed, S) :- deployed(S).

% ploughed_at(GX,GY) / ploughed_between(X1,Y1,X2,Y2): BT-tree-facing
% wrappers around ploughed/3 (Section on ploughing, above quantize_up/3
% /cell_index/3) -- ploughed/3 itself takes CELL INDICES (Cx,Cy), not
% continuous coordinates, so these discretize a continuous point (or
% pair of points) through the SAME plough_cell_size/cell_index/3 the
% ploughing engine itself already uses, then defer straight to
% ploughed/3 -- no new marking logic, purely a coordinate-space
% adapter.
%
% ploughed_at(GX,GY): TRUE iff the single cell (GX,GY) falls into is
% ploughed.
%
% ploughed_between(X1,Y1,X2,Y2): TRUE iff EVERY cell the STRAIGHT LINE
% connecting (X1,Y1)'s own cell center to (X2,Y2)'s own touches is
% ploughed -- "has this whole SWATH already been ploughed" (per this
% feature's own request -- an earlier version of this clause checked
% the full axis-aligned BOUNDING BOX instead; that's not what was
% wanted, see bresenham_cells/5 below for the replacement). Discretizes
% each endpoint to a cell index first (same as ploughed_at above), then
% walks the discrete-grid line between them via the standard integer
% Bresenham line algorithm (bresenham_cells/5 below) and requires every
% cell on it to be ploughed.
%
% Both NON-CONTINUOUS, same reason hitched/0 and deployed/0 above are:
% ploughed/3 only ever becomes newly true at a MoveTo leg's own halt/
% interrupt situation (see its own note), never incrementally mid-leg,
% so there is nothing for a reactive crossing-search to watch for
% either -- see bt_to_prolog.py's own _NON_CONTINUOUS_CONDITIONS note.
holds(ploughed_at(GX,GY), S) :-
    plough_cell_size(CellSize),
    cell_index(GX, CellSize, Cx),
    cell_index(GY, CellSize, Cy),
    ploughed(Cx, Cy, S).
holds(ploughed_between(X1,Y1,X2,Y2), S) :-
    plough_cell_size(CellSize),
    cell_index(X1, CellSize, Cx0),
    cell_index(Y1, CellSize, Cy0),
    cell_index(X2, CellSize, Cx1),
    cell_index(Y2, CellSize, Cy1),
    bresenham_cells(Cx0,Cy0,Cx1,Cy1,Cells),
    all_cells_ploughed(Cells, S).

% bresenham_sign(+A,+B,-Sign): Sign is +1 if A<B, else -1 (covers
% A=:=B too, picking -1 arbitrarily -- see bresenham_cells/5's own note
% on why that axis's Sign is never actually consulted in that case).
% TWO mutually exclusive clauses, not an if-then-else -- ProbLog's own
% Prolog dialect has no '->'/2 (same convention every other multi-case
% predicate in this file already follows, e.g. effective_tool_speed/3
% above).
bresenham_sign(A,B,1)  :- A < B.
bresenham_sign(A,B,-1) :- A >= B.

% bresenham_x_step(+E2,+Dy,+Sx,+Cx,+Err,-Cx2,-Err2) / bresenham_y_step
% (+E2,+Dx,+Sy,+Cy,+Err,-Cy2,-Err2): one axis's own share of a single
% Bresenham step -- TWO mutually exclusive clauses each (E2 below/above
% the axis's own threshold), matching the classic algorithm's two
% independent per-axis conditionals (E2>=Dy for X, E2=<Dx for Y) EXCEPT
% expressed without '->'/2. Both conjuncts are called on every step
% (bresenham_walk/10 below) -- they are independent, not alternatives
% of each other, so a "diagonal" step where BOTH actually advance
% (Cx2\=Cx AND Cy2\=Cy) is completely normal, not a separate case to
% handle: Dy<=0=<Dx always holds (Dy is defined as -abs(...), Dx as
% abs(...)), which makes "neither fires" mathematically impossible --
% verified directly (a 300k-line scratch trace against a hand-derived
% example, plus this predicate's own use in ploughed_between above)
% before adopting this split.
bresenham_x_step(E2,Dy,_Sx,Cx,Err,Cx,Err) :- E2 < Dy.
bresenham_x_step(E2,Dy,Sx,Cx,Err,Cx2,Err2) :- E2 >= Dy, Cx2 is Cx+Sx, Err2 is Err+Dy.
bresenham_y_step(E2,Dx,_Sy,Cy,Err,Cy,Err) :- E2 > Dx.
bresenham_y_step(E2,Dx,Sy,Cy,Err,Cy2,Err2) :- E2 =< Dx, Cy2 is Cy+Sy, Err2 is Err+Dx.

% bresenham_walk(+Cx,+Cy,+Cx1,+Cy1,+Dx,+Dy,+Sx,+Sy,+Err,-Cells): the
% recursive walk itself -- Cells accumulates cell(Cx,Cy) terms from the
% CURRENT position down to (and including) the target (Cx1,Cy1). Base
% case relies on Prolog UNIFICATION, not an explicit =:= test, to
% detect "already at the target": repeating the SAME variable (Cx, Cy)
% in both the "current" and "target" argument positions only unifies
% when the two ALREADY-BOUND integers this predicate is actually called
% with are equal -- the recursive clause's own \+ (Cx=:=Cx1,Cy=:=Cy1)
% guard is the exact complement, so the two clauses are mutually
% exclusive and jointly exhaustive, same "no cut needed" style as every
% other multi-case predicate in this file.
bresenham_walk(Cx,Cy,Cx,Cy,_Dx,_Dy,_Sx,_Sy,_Err,[cell(Cx,Cy)]).
bresenham_walk(Cx,Cy,Cx1,Cy1,Dx,Dy,Sx,Sy,Err,[cell(Cx,Cy)|Rest]) :-
    \+ (Cx =:= Cx1, Cy =:= Cy1),
    E2 is 2*Err,
    bresenham_x_step(E2,Dy,Sx,Cx,Err,Cx2,ErrA),
    bresenham_y_step(E2,Dx,Sy,Cy,ErrA,Cy2,ErrB),
    bresenham_walk(Cx2,Cy2,Cx1,Cy1,Dx,Dy,Sx,Sy,ErrB,Rest).

% bresenham_cells(+Cx0,+Cy0,+Cx1,+Cy1,-Cells): Cells is the list of
% cell(Cx,Cy) pairs (INCLUDING both endpoints) the standard integer
% Bresenham line algorithm visits walking from (Cx0,Cy0) to (Cx1,Cy1)
% -- "every cell the straight line connecting these two cell centers
% passes through", the discrete-grid analogue of dist/5's own straight-
% line distance. Dy is defined NEGATIVE (-abs(...), not +abs(...)) --
% the standard formulation's own convention, needed for bresenham_x_
% step/bresenham_y_step's own threshold comparisons to work out; Sx/Sy
% (this axis's own step direction) are never actually consulted when
% that axis's own Dx/Dy is 0 (see bresenham_x_step/bresenham_y_step's
% own E2<Dy / E2>Dx "no step" clauses -- Dy<=0 there makes E2>=Dy/
% E2=<Dx false whenever Dx/Dy(abs) is 0, for ANY Sx/Sy), so
% bresenham_sign's own arbitrary A=:=B tie-break is always safe.
bresenham_cells(Cx0,Cy0,Cx1,Cy1,Cells) :-
    Dx is abs(Cx1-Cx0),
    Dy is -abs(Cy1-Cy0),
    bresenham_sign(Cx0,Cx1,Sx),
    bresenham_sign(Cy0,Cy1,Sy),
    Err0 is Dx + Dy,
    bresenham_walk(Cx0,Cy0,Cx1,Cy1,Dx,Dy,Sx,Sy,Err0,Cells).

% all_cells_ploughed(+Cells,+S): TRUE iff EVERY cell(Cx,Cy) in Cells
% (a plain, already-finite list -- see bresenham_cells/5 above) is
% ploughed(Cx,Cy,S) -- plain list recursion, no forall/2 or double-
% negation needed here (unlike an EARLIER, box-based version of
% ploughed_between/4 this replaced): the candidate cells are already
% fully enumerated by bresenham_cells/5, not generated lazily via
% between/3, so there's nothing to universally quantify OVER, just a
% list to walk.
all_cells_ploughed([], _S).
all_cells_ploughed([cell(Cx,Cy)|Rest], S) :-
    ploughed(Cx, Cy, S),
    all_cells_ploughed(Rest, S).

% ploughed_at/3, ploughed_between/5 (bare, NOT holds(...)-wrapped): thin
% pass-throughs so a goal_formula.pl can reference either directly, same
% reasoning as sample_value_below/3's own bare wrapper above -- see
% vocabulary.yaml's own entries for these two.
ploughed_at(GX, GY, S) :- holds(ploughed_at(GX,GY), S).
ploughed_between(X1, Y1, X2, Y2, S) :- holds(ploughed_between(X1,Y1,X2,Y2), S).

% obstacle_in_bound(Threshold): true iff the CURRENT position (at the
% current time, via now/2) is within Threshold of ANY obstacle. Same
% parameter, same underlying geometry as the obstacle_in_bound(Threshold)
% TRIGGER (see trigger_crossing_time/9) -- but this checks ONE point
% (the current situation) via a single call to
% obstacle_within_threshold/3, a plain boolean ProbLog predicate
% collision_geometry.py registers directly over
% within_obstacle_threshold/_min_clearance_all (the SAME primitive the
% trigger's bracket-scan calls repeatedly across a whole future
% trajectory) -- see that module's own header. No bracket-scan/
% bisection here: a condition only ever asks "is it true RIGHT NOW",
% not "will it ever become true during this walk".
holds(obstacle_in_bound(Threshold), S) :-
    now(T, S), at(X,Y,T,S),
    clearance_adjusted_threshold(Threshold, AdjThreshold),
    obstacle_within_threshold(X,Y,AdjThreshold).

% line_of_sight_clear(ObstacleId,GX,GY): true iff the CURRENT position
% is NOT occluded from (GX,GY) by ObstacleId's own boundary. Same
% underlying primitive as the line_of_sight_clear(ObstacleId,GX,GY)
% TRIGGER (Bug0's own leave rule -- see trigger_crossing_time/10) --
% but this checks ONE point (the current situation) via a single call
% to line_of_sight_clear/5, exactly the same "single check, no
% bracket-scan" relationship obstacle_in_bound has to its own trigger.
holds(line_of_sight_clear(ObstacleId,GX,GY), S) :-
    now(T, S), at(X,Y,T,S),
    line_of_sight_clear(X,Y,ObstacleId,GX,GY).

% obstacle_on_path(Threshold): true iff the CURRENT position is within
% Threshold of an obstacle THIS WALK'S OWN TRAJECTORY actually enters
% somewhere across its full span -- see trigger_crossing_time/9's own
% note on how this differs from obstacle_in_bound. Needs the CURRENT
% walk's own (ControlPoints,Triggers,T0,SPrev) and its resolved Z --
% current_walk/5 + the z(do(startMoveto(...),SPrev),Z) lookup is the
% EXACT SAME pattern poss(haltMoveto(...)) already uses (works whether
% the walk is still in progress or has just ended, per current_walk/5's
% own doc comment); fails outright if no walk has started yet (nothing
% to check a trajectory against, at s0).
holds(obstacle_on_path(Threshold), S) :-
    current_walk(S, CP, Triggers, T0, SPrev),
    hitch(Tool, SPrev),
    walk_duration(CP, Tool, SPrev, Duration),
    z(do(startMoveto(CP,Triggers,_ActionCode,T0),SPrev), Z),
    zt(do(startMoveto(CP,Triggers,_ActionCode,T0),SPrev), Zt),
    now(T, S), at(X,Y,T,S),
    clearance_adjusted_threshold(Threshold, AdjThreshold),
    obstacle_on_path_within_threshold(CP,T0,Duration,Z,Zt,X,Y,AdjThreshold).

% battery_below(Threshold): true iff the CURRENT battery level (at the
% current time, via now/2) is below Threshold. Same parameter, same
% underlying fluent as the battery_below(Threshold) TRIGGER (see
% trigger_crossing_time/9) -- but this is a single battery/3 lookup at
% the current situation, not first_battery_below_time's forward-looking
% closed-form solve; no black box involved at all, battery/3 is already
% plain Prolog.
holds(battery_below(Threshold), S) :-
    now(T, S), battery(Level, T, S),
    Level < Threshold.

% battery_equal(Threshold) / battery_over(Threshold): the SAME shape as
% battery_below above -- a single battery/3 lookup at the current
% situation, no black box, just a different arithmetic comparison.
% battery_equal uses exact arithmetic equality (=:=) per its own name;
% in a continuous, noise-driven model this is true only at whatever
% instant Level(T) genuinely passes through Threshold (see
% first_battery_equal_time's own note on why that's a well-defined,
% single instant, not a probability-zero non-event) -- checking it at
% an ARBITRARY current time will usually be false, same as asking "is
% it exactly noon" at a random moment.
holds(battery_equal(Threshold), S) :-
    now(T, S), battery(Level, T, S),
    Level =:= Threshold.
holds(battery_over(Threshold), S) :-
    now(T, S), battery(Level, T, S),
    Level > Threshold.

% ---------------------------------------------------------------
% holds_leg/11: the T-PARAMETERIZED twin of holds/2 above, used ONLY by
% first_becomes_false_time/11 below (in turn used ONLY by trigger_
% crossing_time/13's own guard_break(Cond,Code) clauses, far above --
% ONE shared pair of clauses now, serving BOTH MoveTo and install_
% tool/uninstall_tool, split by Mode) -- NOT part of the do_node/cond()
% interface itself (holds/2 stays the ONLY thing cond(C) ever calls).
% holds/2 always asks "is C true RIGHT NOW" via now(T,S) -- which, for
% an IN-PROGRESS leg's own situation term do(startMoveto(...),SPrev),
% always returns that leg's own START time T0, never a later instant
% WITHIN the leg (see the "no bracket-scan/bisection here" note above
% holds(obstacle_in_bound(...)) further up) -- so it cannot be reused
% as-is to watch a condition CONTINUOUSLY across a leg's own future.
% holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T) asks the SAME
% question at an EXPLICIT T instead, using the SAME flat (CP,T0,
% Duration,Z,Zt,Zb,B0,Rate) signature every trigger_crossing_time/13
% clause already receives (no situation term needed at all -- X,Y come
% from walk_noisy_point/8, battery Level from battery_at_leg/7 above,
% both already pure functions of T within one leg, Rate included).
%
% Mode is SHARED with install_tool/uninstall_tool, on request -- 0
% means "MoveTo: every clause below is available" (trigger_crossing_
% time/13's own Mode=0 guard_break clause, plus poss(haltMoveto(...))/
% poss(interrupt(...))/verify_safe further up, which resolve Mode=0
% before it ever enters this whole chain, always pass 0), 1 means
% "install_tool/uninstall_tool: BATTERY-ONLY" (trigger_crossing_time/
% 13's own Mode=1 guard_break clause, and poss(halt_install_tool(...))/
% poss(halt_uninstall_tool(...)), always pass 1). EVERY motion-based
% clause below (distance_below/equal/over, obstacle_in_bound, obstacle_
% on_path, line_of_sight_clear) pattern-matches Mode=0 directly IN THE
% HEAD, so under Mode=1 none of them even attempt to unify -- CP is
% NEVER touched, NEVER dereferenced, so the placeholder (a fresh
% unbound variable) trigger_crossing_time/13's own Mode=1 guard_break
% clause and poss(halt_install_tool(...))/poss(halt_uninstall_tool
% (...)) pass in is safe no matter what it is. The battery clauses
% (battery_below/equal/over) don't care which Mode they're called
% under -- both callers legitimately want battery conditions evaluated,
% so those three clauses wildcard Mode away entirely. Mode=1 alone does
% NOT make an unsupported Cond (e.g. a motion condition reaching here
% for install_tool/uninstall_tool) degrade safely on its OWN -- see
% tool_cond_supported/1's own note (in the TOOL TRIGGERS section
% further down) for why an explicit outer gate is still needed, on
% trigger_crossing_time/13's own Mode=1 guard_break clause, before this
% predicate is ever reached under Mode=1.
%
% ONE clause per condition in schema.yaml's conditions: list that can
% MEANINGFULLY vary within a single leg (i.e. everything except
% HaltedWith, which is history-based and cannot change mid-leg --
% bt_to_prolog.py's own guard-derivation pass rejects HaltedWith as an
% auto-derived guard BEFORE this predicate is ever reached, precisely
% to avoid a missing clause here silently reading as "already false at
% T0"). Adding a new schema.yaml condition later needs a matching
% clause HERE (Mode=0, if it's motion-based) for it to be usable as an
% automatically-derived reactive guard -- or, if it genuinely can't
% vary mid-leg either, adding it to _NON_CONTINUOUS_CONDITIONS in bt_
% to_prolog.py instead, same as HaltedWith.
holds_leg(and(P,Q), CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T) :-
    holds_leg(P,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T), holds_leg(Q,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T).
holds_leg(or(P,Q), CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T) :-
    holds_leg(P,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T) ; holds_leg(Q,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T).
holds_leg(neg(P), CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T) :-
    \+ holds_leg(P,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T).

holds_leg(distance_below(GX,GY,Threshold), CP,T0,Duration,Z,Zt,_Zb,_B0,_Rate,0,T) :-
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    dist(X,Y,GX,GY,D), D < Threshold.
holds_leg(distance_equal(GX,GY,Threshold), CP,T0,Duration,Z,Zt,_Zb,_B0,_Rate,0,T) :-
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    dist(X,Y,GX,GY,D), D =:= Threshold.
holds_leg(distance_over(GX,GY,Threshold), CP,T0,Duration,Z,Zt,_Zb,_B0,_Rate,0,T) :-
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    dist(X,Y,GX,GY,D), D > Threshold.

% point(GX,GY)-term adapters, same blackboard-ref-goal treatment (and
% same reasoning) as holds/2's own three just above -- needed here too
% since an auto-derived REACTIVE GUARD (a DistanceBelow/Equal/Over left
% sibling under a ReactiveSequence/ReactiveFallback) is checked
% CONTINUOUSLY, mid-leg, via holds_leg/11 (exact bracket-scan crossing-
% time detection), not via holds/2's one-shot check.
holds_leg(distance_below(point(GX,GY),Threshold), CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T) :-
    holds_leg(distance_below(GX,GY,Threshold), CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T).
holds_leg(distance_equal(point(GX,GY),Threshold), CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T) :-
    holds_leg(distance_equal(GX,GY,Threshold), CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T).
holds_leg(distance_over(point(GX,GY),Threshold), CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T) :-
    holds_leg(distance_over(GX,GY,Threshold), CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T).

holds_leg(obstacle_in_bound(Threshold), CP,T0,Duration,Z,Zt,_Zb,_B0,_Rate,0,T) :-
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    clearance_adjusted_threshold(Threshold, AdjThreshold),
    obstacle_within_threshold(X,Y,AdjThreshold).

holds_leg(obstacle_on_path(Threshold), CP,T0,Duration,Z,Zt,_Zb,_B0,_Rate,0,T) :-
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    clearance_adjusted_threshold(Threshold, AdjThreshold),
    obstacle_on_path_within_threshold(CP,T0,Duration,Z,Zt,X,Y,AdjThreshold).

holds_leg(line_of_sight_clear(ObstacleId,GX,GY), CP,T0,Duration,Z,Zt,_Zb,_B0,_Rate,0,T) :-
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    line_of_sight_clear(X,Y,ObstacleId,GX,GY).

holds_leg(battery_below(Threshold), _CP,T0,Duration,_Z,_Zt,Zb,B0,Rate,_Mode,T) :-
    battery_at_leg(T0,Duration,Zb,B0,Rate,T,Level), Level < Threshold.
holds_leg(battery_equal(Threshold), _CP,T0,Duration,_Z,_Zt,Zb,B0,Rate,_Mode,T) :-
    battery_at_leg(T0,Duration,Zb,B0,Rate,T,Level), Level =:= Threshold.
holds_leg(battery_over(Threshold), _CP,T0,Duration,_Z,_Zt,Zb,B0,Rate,_Mode,T) :-
    battery_at_leg(T0,Duration,Zb,B0,Rate,T,Level), Level > Threshold.

% first_becomes_false_time(+Cond,+CP,+T0,+Duration,+Z,+Zt,+Zb,+B0,+Rate,
% +Mode,-Tcross): the FIRST instant in (T0,T0+Duration] that Cond
% (already polarity-adjusted by bt_to_prolog.py -- see guard_break/2's
% own note in trigger_crossing_time/12 above) stops holding, given it
% holds at T0 -- the same "already true at T0 / genuine future search"
% two-shape convention every other first_*_time predicate in this file
% already follows (two MUTUALLY EXCLUSIVE clauses, on \+holds_leg(...,
% T0) vs holds_leg(...,T0), no cut needed -- same style as first_
% battery_below_time above), just GENERIC over any holds_leg/11-
% recognized Cond instead of one bespoke formula per condition. FAILS
% (no crossing) if Cond holds for the WHOLE walk -- same "absence, not
% sentinel" convention as everywhere else in this file. Rate is the
% battery drain rate for whichever consumer is calling in (moving-phase
% for MoveTo, install/uninstall-phase for install_tool/uninstall_tool),
% already resolved to a plain number by whichever caller started this
% chain -- see earliest_halt/12's own note and tool_trigger_crossing_
% time's own guard_break clause. Mode is just forwarded straight to
% holds_leg/11 -- see that predicate's own note for what it means and
% why Mode=1 alone isn't a complete safety story on its own.
%
% Bracket-scans bracket_samples/1 equal steps (the SAME config knob
% first_threshold_crossing_time's own black-box search already uses --
% see Section 0/verification.bracket_samples), then bisects the found
% bracket down to crossing_eps/1 -- returning the HIGH (verified-FALSE)
% end of the final bracket, never the low end or the midpoint, so a
% merge-grid-quantized restart seeded from Tcross is never seeded
% slightly EARLY (i.e. while Cond might still actually hold) -- same
% "never let approximation look safer than reality" convention as
% quantize_down/3.
first_becomes_false_time(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T0) :-
    \+ holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T0).
first_becomes_false_time(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,Tcross) :-
    holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,T0),
    bracket_samples(N),
    TEnd is T0 + Duration,
    guard_bracket_scan(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,N,1,TEnd,T0,Tlo,Thi),
    crossing_eps(Eps),
    guard_bisect(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,Eps,Tlo,Thi,Tcross).

% guard_bracket_scan(...,+N,+I,+TEnd,+Tprev,-Tlo,-Thi): walk forward
% from I=1 to N, one bracket_samples/1-th of the way from T0 to TEnd
% each step, stopping at the FIRST step where Cond has flipped from
% true (Tprev) to false (the current sample) -- Tlo/Thi bracket the
% crossing. Fails (no crossing anywhere in the scan) once I exceeds N,
% exactly mirroring every other first_*_time predicate's own "fails if
% it never happens" convention -- Cond held at every single sample.
% TWO MUTUALLY EXCLUSIVE clauses (on holds_leg(...,Ti) succeeding or
% failing) rather than if-then-else -- ProbLog's own Prolog dialect
% doesn't support '->'/2, same reason every other multi-case predicate
% in this file (e.g. first_battery_below_time above) is written this
% way instead.
guard_bracket_scan(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,N,I,TEnd,Tprev,Tlo,Thi) :-
    I =< N,
    Ti is T0 + (TEnd-T0) * I / N,
    holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,Ti),
    I1 is I + 1,
    guard_bracket_scan(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,N,I1,TEnd,Ti,Tlo,Thi).
guard_bracket_scan(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,N,I,TEnd,Tprev,Tprev,Ti) :-
    I =< N,
    Ti is T0 + (TEnd-T0) * I / N,
    \+ holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,Ti).

% guard_bisect(...,+Eps,+Tlo,+Thi,-Tcross): standard bisection --
% invariant Cond holds at Tlo, doesn't hold at Thi; narrows until the
% bracket is under Eps wide, then reports the FALSE end (see
% first_becomes_false_time's own note on why the high end, not the
% midpoint or the low end). Same mutually-exclusive-clauses style as
% guard_bracket_scan above, no '->'/2.
guard_bisect(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,Eps,Tlo,Thi,Thi) :-
    Width is Thi - Tlo,
    Width =< Eps.
guard_bisect(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,Eps,Tlo,Thi,Tcross) :-
    Width is Thi - Tlo,
    Width > Eps,
    Tmid is (Tlo + Thi) / 2,
    holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,Tmid),
    guard_bisect(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,Eps,Tmid,Thi,Tcross).
guard_bisect(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,Eps,Tlo,Thi,Tcross) :-
    Width is Thi - Tlo,
    Width > Eps,
    Tmid is (Tlo + Thi) / 2,
    \+ holds_leg(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,Tmid),
    guard_bisect(Cond,CP,T0,Duration,Z,Zt,Zb,B0,Rate,Mode,Eps,Tlo,Tmid,Tcross).

% ---------------------------------------------------------------
% 8. VERIFICATION-TIME SAMPLING (NOT part of the action theory) --
%    a purely deterministic choice of how finely to CHECK/REPORT the
%    already-closed-form at/4 fluent for VISUALIZATION purposes.
%    NOTE: collision DETECTION itself is now EXACT (see
%    first_collision_time above) -- num_samples/1 only controls the
%    resolution used for plotting and for on_track's drift reporting,
%    it no longer determines whether a collision is found at all.
%    num_samples/1 is now a config fact -- see
%    the problem's own config.yaml's verification.num_samples.
% ---------------------------------------------------------------

sample_frac(I, Frac) :- num_samples(N), Frac is I / N.

% final_situation(+S)/plan_outcome(-Outcome): drive plan/1's tree to a
% genuine true/false/world_too_large conclusion with a SINGLE do_node
% call, from s0 -- NOT the redescend-from-root loop this predicate
% used to be (see git history for that version, and this project's own
% conversation log for why it was replaced). Redescending on a
% reactive(_) halt is now entirely reactivesequence/reactivefallback's
% own job (see the CONTROL-FLOW REDESCEND TARGETS note above
% do_node(reactivesequence(...))) -- every reactive-classified trigger
% is tagged, at translation time, with the code of its own nearest
% enclosing reactive composite, which catches and locally restarts it,
% so a bare reactive(_) should never reach THIS call at all.
%
% If it somehow does (a translator bug -- bt_to_prolog.py is supposed
% to catch this at translation time as a hard failure, so this is a
% defense-in-depth check, not the primary guard), final_situation/1
% simply has no solution in that world (the first clause's own
% Outcome \= reactive(_) guard fails), and plan_outcome/1's OWN second
% clause reports it explicitly as reactive_escaped rather than silently
% folding it into any of the three legitimate outcomes -- P(plan_
% outcome(reactive_escaped)) > 0 in a report is the signal that
% something is structurally wrong with the translated plan, and is
% never expected to be nonzero for a plan that translated cleanly.
final_situation(S) :-
    plan(Node),
    do_node(Node, s0, S, Outcome),
    Outcome \= reactive(_).

% -- FULL OUTCOME ENUMERATION -----------------------------------------
% outcome_entry(+Action, -Entry): Entry = Code-Value for whichever
% ACTION or CONDITION marker carries a queryable outcome of its own --
% Value is Reason with its own trailing ActionCode STRIPPED back off
% (the exact same generic univ+append technique halted_with_pattern/3
% already uses to do the reverse) for a MoveTo's haltMoveto or a
% PlanWith's planned(...) marker, or simply Status for a cond(C,Code)'s
% own checked(Code,C,Status) marker (see that predicate's own note).
% Fails outright (no Entry, no choicepoint) for every other action
% (startMoveto, interrupt) -- those carry no reportable outcome of
% their own, only advance the clock/position.
outcome_entry(haltMoveto(_,Reason,_), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].
outcome_entry(planned(_,Reason), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].
outcome_entry(checked(Code,_,Status), Code-Status).
outcome_entry(take_sample(_SampleId,_ActionCode,Reason,_Status), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].
outcome_entry(halt_install_tool(_,Reason,_), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].
outcome_entry(halt_uninstall_tool(_,Reason,_), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].
outcome_entry(halt_deploy_tool(_,Reason,_), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].
outcome_entry(halt_retract_tool(_,Reason,_), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].
outcome_entry(tool_position_result(Reason), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].
outcome_entry(tools_of_kind_result(Reason), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].
outcome_entry(nearest_tool_result(Reason), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].
outcome_entry(hitched_id_result(Reason), Code-Pattern) :-
    Reason =.. [Functor|Args],
    append(Args0, [Code], Args),
    Pattern =.. [Functor|Args0].

% history_outcomes(+S, -Entries): every outcome_entry/2 found ANYWHERE
% in S's own history, oldest-first -- the one GENERIC pass behind
% outcome_signature/1 below. UNLIKE halted_with/2 (finds ONE matching
% Reason, nondeterministically, one solution per match), this COLLECTS
% ALL of them at once, since a full outcome needs every action's/
% condition's own contribution together, not one at a time. No sort/
% dedup step needed: do_node/4 always visits a GIVEN tree's own
% children in the SAME left-to-right structural order regardless of
% which Reason/Status values actually occur, so two resolved worlds
% that reach the same SET of codes always visited them in the same
% relative order already -- Entries is already canonical across
% worlds, for free.
history_outcomes(s0, []).
history_outcomes(do(A,S), [Entry|Rest]) :-
    outcome_entry(A, Entry),
    history_outcomes(S, Rest).
history_outcomes(do(A,S), Rest) :-
    \+ outcome_entry(A, _),
    history_outcomes(S, Rest).

% outcome_signature(-Sig): Sig is the full list of every Code-Value
% pair this resolved world's own final_situation actually produced --
% one entry per action Reason and per condition check reached along
% the way, enclosing the WHOLE outcome of the system in one term rather
% than the separate per-action/per-condition MARGINALS halted_with_
% pattern/3 and any_condition_status/2 already report. Queried
% DELIBERATELY NON-GROUND (query(outcome_signature(_))), reusing the
% SAME "ProbLog reports one result row per distinct grounding instead
% of aggregating" behavior halted_with_pattern_detail/3 already relies
% on (see that predicate's own note) -- here that's exactly the wanted
% behavior: one row per DISTINCT combination of Reason/Condition values
% actually reached, each with its own aggregated probability, a full
% enumeration of every possible outcome instead of one marginal at a
% time. NOTE: a cond() leaf re-checked more than once in the SAME world
% (e.g. one sitting inside a reactive_children/2 list that gets
% redescended) contributes ONE Code-Value entry PER actual check, not
% just its last one -- a redescended condition whose own Status
% genuinely differed between checks shows up as two distinct entries
% for the same Code in Sig, which is correct (both really happened in
% that one world), if unusual to read.
outcome_signature(Sig) :-
    final_situation(S),
    history_outcomes(S, Sig).

% plan_outcome(Outcome): the WHOLE tree's own outcome, a first-class
% query -- P(plan_outcome(true)) is the BT-level analogue of verify_
% goal_formula, but based on Status/Outcome rather than an explicit
% goal formula. Outcome is one of true/false/world_too_large (the
% three legitimate outcomes; world_too_large now comes from a LOCAL
% reactivesequence/reactivefallback exhausting its own budget, see
% that note again) or reactive_escaped (a translator-bug signal, see
% final_situation/1's own note just above -- should never actually be
% nonzero).
plan_outcome(Outcome) :-
    plan(Node),
    do_node(Node, s0, _, Outcome),
    Outcome \= reactive(_).
plan_outcome(reactive_escaped) :-
    plan(Node),
    do_node(Node, s0, _, reactive(_)).

% replan_budget/1: the STARTING budget every reactivesequence/
% reactivefallback occurrence reads (fresh, independently) on its own
% first entry -- see reactivesequence_budgeted/5 and
% reactivefallback_budgeted/5 above. Previously this bounded ONE
% global whole-tree redescend loop; now each reactive composite gets
% its OWN counter seeded from this SAME shared value, decremented
% independently as THAT composite restarts. Still exists for
% ProbLog's own sake, not the robot's -- see the CONTROL-FLOW
% REDESCEND TARGETS note's own discussion of why nothing guarantees
% termination on its own (a degenerate zero-duration leg can restart
% forever without this bound, same reasoning as before, now just
% localized to whichever composite it's under).
replan_budget(1000).

% plan_time_span(+S, -T0, -TEnd): T0 is when the (most recent) walk
% started; TEnd is the wall-clock time the PLAN actually ends at --
% the time argument of the final haltMoveto/interrupt action, or (if
% the plan somehow ends mid-walk with no closing action) the walk's
% own natural completion time, as a safe fallback.
plan_time_span(S, T0, TEnd) :-
    current_walk(S, CP, T0),
    last_action_time(S, CP, T0, TEnd).

last_action_time(do(haltMoveto(T,_,_),_), _, _, T).
last_action_time(do(interrupt(T),_), _, _, T).
last_action_time(do(startMoveto(_,_,_,_),SPrev), CP, T0, TEnd) :-
    hitch(Tool, SPrev),
    walk_duration(CP, Tool, SPrev, Duration),
    TEnd is T0 + Duration.
last_action_time(do(A,S), CP, T0, TEnd) :-
    A \= haltMoveto(_,_,_), A \= interrupt(_), A \= startMoveto(_,_,_,_),
    last_action_time(S, CP, T0, TEnd).

sample_time(I, S, T) :-
    plan_time_span(S, T0, TEnd),
    sample_frac(I, Frac),
    T is T0 + (TEnd-T0)*Frac.

% sample_walk_frac(I,S,Frac): the sampled instant expressed as a
% fraction of the WALK's own full nominal duration (for comparing
% against nominal_at/4, which is parametrized the same way) --
% distinct from sample_frac/2, which is a fraction of however much
% of the plan actually got executed (0..(TEnd-T0)) if interrupted.
% Duration=0.0 is a genuine, valid case here too (see collision_
% geometry.py's own _walk_noisy_point note on why -- a zero-length
% "already at the goal" leg), not something to reject: TWO mutually
% exclusive clauses on Duration=<0.0 vs >0.0, same "no if-then-else"
% convention every other multi-case predicate in this file already
% uses (ProbLog's own Prolog dialect doesn't support '->'/2). A leg
% that never actually progresses anywhere has WalkFrac=0.0 throughout
% -- the same well-defined limit collision_geometry.py's own frac
% picks for the identical reason.
sample_walk_frac(I, S, WalkFrac) :-
    plan_time_span(S, T0, TEnd),
    current_walk(S, CP, T0, SPrev),
    hitch(Tool, SPrev),
    walk_duration(CP, Tool, SPrev, Duration),
    Duration =< 0.0,
    sample_frac(I, _Frac),
    WalkFrac is 0.0.
sample_walk_frac(I, S, WalkFrac) :-
    plan_time_span(S, T0, TEnd),
    current_walk(S, CP, T0, SPrev),
    hitch(Tool, SPrev),
    walk_duration(CP, Tool, SPrev, Duration),
    Duration > 0.0,
    sample_frac(I, Frac),
    T is T0 + (TEnd-T0)*Frac,
    WalkFrac is (T - T0) / Duration.


% ---------------------------------------------------------------
% 7. SAFETY QUERIES.
%    SAFETY QUERIES READ THE ACTUAL OUTCOME, THEY DO NOT RE-DERIVE IT.
%    An earlier version of crashed_in/1 independently recomputed
%    first_collision_time over the walk's FULL nominal duration,
%    regardless of whether some OTHER cause (e.g. obstacle_in_bound)
%    had already halted the walk earlier in this same world. That's a
%    real bug, not a subtlety: it answers "would this trajectory
%    eventually reach the collision margin if nothing else stopped
%    it", which can disagree with "did the executed plan actually
%    crash" the moment more than one halting cause can compete to be
%    first -- exactly what Triggers introduces. The fix: every "did X
%    actually happen" query below reads the Reason ALREADY RECORDED
%    in the resolved situation (via halted_with/2), rather than
%    recomputing anything -- this is correct BY CONSTRUCTION for any
%    number of triggers, present or future, since it never touches
%    the trigger-specific detection machinery at all, only the
%    situation's own history.
% ---------------------------------------------------------------

% halted_with(+Reason, +S): TRUE iff SOMEWHERE in S's action history
% EITHER a haltMoveto OR a planWith occurred with exactly this Reason
% -- ACTION-INDEPENDENT on purpose: a query built on this (or on
% halted_with_pattern/3 further down, which is layered directly on
% top of this) shouldn't have to know or care whether a given Reason
% came from a MoveTo leg finishing or a planning call finishing, only
% that SOME action in the history produced it. The two Reason
% vocabularies never collide by construction: a MoveTo's own Reasons
% (completed(Code), crashed(ObstId,Code), battery_under(Threshold,Code),
% ...) and a planning call's own (completed(Algorithm,Goal,Code),
% no_path(Algorithm,Goal,Code)) differ in ARITY even where they share a
% functor name (completed/1 vs completed/3), so Prolog unification
% already keeps "any MoveTo completion" (completed(_)) and "any
% planning success" (completed(_,_,_)) as two distinct, never-
% overlapping queries with no extra disambiguation needed. Searches the
% WHOLE history (not just the most recent halt/plan), so a future
% multi-leg plan where an earlier leg/planning call had a different
% fate than the final one is still handled correctly.
halted_with(Reason, do(haltMoveto(_,Reason,_), _)).
halted_with(Reason, do(planned(_Algorithm,Reason), _)).
halted_with(Reason, do(take_sample(_SampleId,_ActionCode,Reason,_Status), _)).
halted_with(Reason, do(halt_install_tool(_,Reason,_), _)).
halted_with(Reason, do(halt_uninstall_tool(_,Reason,_), _)).
halted_with(Reason, do(halt_deploy_tool(_,Reason,_), _)).
halted_with(Reason, do(halt_retract_tool(_,Reason,_), _)).
halted_with(Reason, do(tool_position_result(Reason), _)).
halted_with(Reason, do(tools_of_kind_result(Reason), _)).
halted_with(Reason, do(nearest_tool_result(Reason), _)).
halted_with(Reason, do(hitched_id_result(Reason), _)).
halted_with(Reason, do(_A, S)) :- halted_with(Reason, S).

% visited(+Loc, +Tol, +S): TRUE iff the robot ACTUALLY ARRIVED at
% Loc=point(GX,GY) -- the endpoint of SOME already-completed leg
% (Reason=completed(_ActionCode), Status=true, i.e. the walk wasn't
% cut short by a trigger), AND its own actual (noisy) final position
% is within Tol of Loc. Status=true alone no longer implies this (see
% leg_status/9's own note: that check used to be baked into Status,
% but is now the separate, explicit distance_below/3 condition --
% visited/3 re-derives the SAME dist/5 comparison directly, via at/4,
% rather than depending on any particular BT node having been placed
% in the tree to check it) -- anywhere in S's
% history. SAME "search the whole history" shape as halted_with/2
% above, and for the same reason needs no separate persistence/frame
% axiom: situation histories only ever grow by appending do(...), so
% "did this ever happen in S's past" is already monotonic for free --
% a fluent that starts false and, once made true, stays true in every
% situation built on top of that one, exactly the shape a multi-leg
% "visited(A,Tol,S), visited(B,Tol,S), visited(C,Tol,S)" goal formula
% needs, verified
% the same way verify_goal_formula/any_collision already are (P(...) over
% resolved worlds, since a collision or battery depletion partway
% through a multi-leg plan can genuinely truncate the history before
% a later waypoint is ever reached -- this is NOT something you could
% check by inspecting the plan's own static structure instead).
%
% NOTE: for a plain LINEAR Sequence of legs (no Fallback in between),
% seq_node/1's own definition (do_node(seq_node([Child|Rest]),S,S1,
% Outcome):-do_node(Child,S,S2,true),...) already REQUIRES each
% child's Status=true before the next one even starts -- so checking
% visited/3 on just the LAST waypoint already logically entails every
% earlier one was visited too; only worth checking each individually
% once a Fallback sits somewhere before the waypoint you care about.
visited(point(GX,GY), Tol, do(haltMoveto(T,completed(_ActionCode),true), S)) :-
    current_walk(S, CP, _Triggers, _T0, _SPrev),
    leg_target(CP, GX, GY),
    at(X,Y,T, do(haltMoveto(T,completed(_ActionCode),true), S)),
    dist(X,Y,GX,GY,D),
    D =< Tol.
visited(Loc, Tol, do(_A, S)) :- visited(Loc, Tol, S).

% ploughed(-Cx, -Cy, +S): relational fluent -- TRUE iff macro-cell
% (Cx,Cy) (see cell_index/3, above quantize_up/3; cell size is
% plough_cell_size/1, this problem's own config.yaml ploughing.
% cell_size, deliberately coarser than disc_step_position -- see
% config_to_prolog.py's own note) was swept by the robot's ACTUAL path
% during some MoveTo leg while the plow was BOTH equipped AND DEPLOYED
% (hitch(plow,SPrev), deployed(SPrev) -- see Section 5c/5d) -- merely
% having the plow installed is not enough, per this feature's own
% request: nothing gets marked ploughed between InstallTool and the
% first DeployTool, or after RetractTool, only in between a successful
% deploy and its matching retract. Anywhere in S's history. Same
% monotonic "search the whole history, no frame axiom
% needed beyond a plain pass-through" shape as visited/3 above and
% halted_with/2 -- once true for a situation, stays true for every
% situation built on top of it.
%
% Deliberately NOT gated on Reason=completed(_)/Status=true the way
% visited/3 is: a leg cut short by a collision, a trigger, or an
% interrupt still physically ploughed whatever ground it actually
% covered before halting, so BOTH do(haltMoveto(...),S) (any Reason/
% Status) and do(interrupt(...),S) contribute -- Elapsed is T1-T0 (the
% leg's own ACTUAL duration, however far it got), never the full
% nominal Duration walk_duration/3 would report.
%
% Sampling, not a closed-form crossing search: unlike holds_leg/first_
% becomes_false_time (which need the EXACT time a condition flips),
% "which cells did the path pass through" has no useful closed form --
% ploughed_cell_sample/11 below just walks the SAME noisy path
% collision detection already samples (walk_noisy_point/8, at
% bracket_samples/1 resolution) and buckets each sampled point into a
% cell via cell_index/3. No left/right offset sampling either (an
% earlier design considered marking a whole cross-strip either side of
% the centerline) -- once the cell size itself approximates the plow's
% own physical width, a centerline sample landing in a cell already
% implies the plow's edges are within that cell or its immediate
% neighbor, so nothing is gained by sampling the edges separately.
%
% THE ONE THING THIS APPROXIMATION HONESTLY TRADES AWAY: a pass exactly
% along a cell boundary could miss marking the adjacent cell the plow's
% edge technically grazed. That's a boundary rounding effect, not a
% systematic bias, and it's the same kind of approximation error
% already accepted by choosing a coarse cell size in the first place --
% not a new source of imprecision on top of it.
%
% plough_cell_size/1 ITSELF is pure config data (config_to_prolog.py's
% own ploughing.cell_size), only ever emitted as a fact when a
% problem's config.yaml has a ploughing: section with a cell_size key
% -- same "config-generated, no real clause in this file" shape as
% tool_instance/2 above, and the same placeholder-clause fix applies:
% a problem with a plow tool.instance but NO ploughing.cell_size could
% still reach hitch(plow,S)=true (installing a plow needs no
% ploughing config at all), and PloughedAt/PloughedBetween (see
% holds(ploughed_at(...))/holds(ploughed_between(...)) above) query
% plough_cell_size directly, unconditionally, before ever touching
% ploughed/3 -- so this predicate needs to be "known" even in a
% problem that never configures ploughing.
plough_cell_size(no_ploughing_configured) :- fail.

ploughed(Cx, Cy, do(haltMoveto(T1,_Reason,_Status), S)) :-
    current_walk(S, CP, Triggers, ActionCode, T0, SPrev),
    hitch(plow, SPrev),
    deployed(SPrev),
    walk_duration(CP, plow, SPrev, Duration),
    z(do(startMoveto(CP,Triggers,ActionCode,T0),SPrev), Z),
    zt(do(startMoveto(CP,Triggers,ActionCode,T0),SPrev), Zt),
    Elapsed is T1 - T0,
    plough_cell_size(CellSize),
    bracket_samples(N),
    ploughed_cell_sample(CP,T0,Duration,Elapsed,Z,Zt,CellSize,N,0,Cx,Cy).
ploughed(Cx, Cy, do(interrupt(T1), S)) :-
    current_walk(S, CP, Triggers, ActionCode, T0, SPrev),
    hitch(plow, SPrev),
    deployed(SPrev),
    walk_duration(CP, plow, SPrev, Duration),
    z(do(startMoveto(CP,Triggers,ActionCode,T0),SPrev), Z),
    zt(do(startMoveto(CP,Triggers,ActionCode,T0),SPrev), Zt),
    Elapsed is T1 - T0,
    plough_cell_size(CellSize),
    bracket_samples(N),
    ploughed_cell_sample(CP,T0,Duration,Elapsed,Z,Zt,CellSize,N,0,Cx,Cy).
ploughed(Cx, Cy, do(A,S)) :-
    A \= haltMoveto(_,_,_), A \= interrupt(_),
    ploughed(Cx, Cy, S).

% ploughed_cell_sample(+CP,+T0,+Duration,+Elapsed,+Z,+Zt,+CellSize,+N,
% +I,-Cx,-Cy): nondeterministically enumerates the macro-cell for each
% of I=0..N evenly-spaced samples across [T0,T0+Elapsed] (Elapsed, NOT
% the full nominal Duration -- see ploughed/3's own note on why a leg
% cut short still ploughs whatever it actually covered). Duration
% itself is still passed to walk_noisy_point/8 (the SAME noisy-path
% primitive at/4 and collision detection already use) since that's
% what the spline's own parameterization needs, independent of how far
% the leg actually got. Backtracks over every I -- ploughed/3's own
% caller just needs ANY one sample to match a queried Cx,Cy, so plain
% backtracking (no findall/list needed) is the simpler fit here, unlike
% nearest_tool_of_kind/6's own use of findall/3+min_candidate/2, which
% genuinely needs every candidate collected at once to fold down to a
% minimum.
ploughed_cell_sample(CP,T0,Duration,Elapsed,Z,Zt,CellSize,N,I,Cx,Cy) :-
    I =< N,
    T is T0 + Elapsed * I / N,
    walk_noisy_point(CP,T0,Duration,Z,Zt,T,X,Y),
    cell_index(X,CellSize,Cx),
    cell_index(Y,CellSize,Cy).
ploughed_cell_sample(CP,T0,Duration,Elapsed,Z,Zt,CellSize,N,I,Cx,Cy) :-
    I < N,
    I1 is I + 1,
    ploughed_cell_sample(CP,T0,Duration,Elapsed,Z,Zt,CellSize,N,I1,Cx,Cy).

% sample_success_at(+Loc,+Tol,+S): the take_sample analogue of
% visited/3 above -- TRUE iff SOME take_sample action SUCCEEDED
% (Reason=sample_success(X,Y,_ActionCode) -- see do_node(take_sample
% (...)) further up) anywhere in S's history, at a recorded position
% within Tol of Loc=point(GX,GY). Same monotonic, whole-history-search
% shape as visited/3 (once true for a situation, true for every
% situation built on top of it), so "did you take the sample at THESE
% locations" is exactly "sample_success_at(point(GX1,GY1),Tol,S),
% sample_success_at(point(GX2,GY2),Tol,S), ..." in a goal formula --
% one conjunct per location, same idiom multi-waypoint goal formulas
% already use for visited/3.
sample_success_at(point(GX,GY), Tol, do(take_sample(_SampleId,_ActionCode,sample_success(X,Y,_V,_SampleId2,_ActionCode2),true), _S)) :-
    dist(X,Y,GX,GY,D), D =< Tol.
sample_success_at(Loc, Tol, do(_A, S)) :- sample_success_at(Loc, Tol, S).

% -- crashed_in(S) / battery_depleted_in(S) / obstacle_in_bound_in(S) /
%    battery_under_in(S): trivial one-liners reading the actual Reason,
%    not re-deriving anything. crashed(_)/obstacle_in_bound(_,_) use
%    wildcards since those Reasons carry WHICH obstacle (and, for
%    obstacle_in_bound, WHICH threshold too -- see trigger_crossing_time/9's
%    own note) -- unbound arguments here correctly mean "regardless of
%    which obstacle/threshold". battery_under(_) similarly wildcards
%    its Threshold. Any FUTURE trigger's own "did it actually fire"
%    diagnostic is exactly this same one-liner pattern -- no
%    trigger-specific re-derivation logic to get wrong.
% Every Reason shape below now carries an extra TRAILING ActionCode
% argument (see tag_reason/3, above poss(haltMoveto(...))) -- one more
% wildcard per functor, same "regardless of which" convention as the
% obstacle/threshold wildcards already here.
crashed_in(S) :- halted_with(crashed(_,_), S).
battery_depleted_in(S) :- halted_with(battery_depleted(_), S).
obstacle_in_bound_in(S) :- halted_with(obstacle_in_bound(_,_,_), S).
obstacle_on_path_in(S) :- halted_with(obstacle_on_path(_,_,_), S).
battery_under_in(S) :- halted_with(battery_under(_,_), S).
battery_equal_in(S) :- halted_with(battery_equal(_,_), S).
battery_over_in(S) :- halted_with(battery_over(_,_), S).

% -- crashed_obstacle(ObstacleId,S) / obstacle_in_bound_obstacle
%    (Threshold,ObstacleId,S) / obstacle_on_path_obstacle(Threshold,
%    ObstacleId,S) / battery_under_threshold(Threshold,S) /
%    battery_equal_threshold(Threshold,S) /
%    battery_over_threshold(Threshold,S): the direct accessors for
%    WHICH obstacle/threshold -- unlike the *_in(S) checks above, the
%    extra argument(s) are left bound, not wildcarded. Fails (no
%    solution) if S didn't halt for that reason, same "absence, not
%    sentinel" convention as everywhere else. Situation argument S is
%    LAST in every one of these, per Reiter's own convention (see
%    module/contracts/vocabulary.yaml's own note on this -- these six
%    used to put S FIRST, an inconsistency with visited/2, halted_with
%    /2, at/4, and battery/3 above, all of which already had S last;
%    fixed here since nothing outside this file's own definitions
%    referenced the old argument order).
% One more trailing wildcard each, same reason as the *_in(S) family
% above -- ActionCode itself isn't exposed as an argument HERE (these
% predicates' own job is "which obstacle/threshold", unchanged); query
% halted_with_cond(crashed(ObstacleId,ActionCode)) directly (see
% tag_reason/3's own note) when ActionCode itself is what's wanted.
crashed_obstacle(ObstacleId, S) :- halted_with(crashed(ObstacleId,_), S).
obstacle_in_bound_obstacle(Threshold, ObstacleId, S) :- halted_with(obstacle_in_bound(Threshold,ObstacleId,_), S).
obstacle_on_path_obstacle(Threshold, ObstacleId, S) :- halted_with(obstacle_on_path(Threshold,ObstacleId,_), S).
battery_under_threshold(Threshold, S) :- halted_with(battery_under(Threshold,_), S).
battery_equal_threshold(Threshold, S) :- halted_with(battery_equal(Threshold,_), S).
battery_over_threshold(Threshold, S) :- halted_with(battery_over(Threshold,_), S).

% -- GENERIC per-action safety-query machinery -----------------------
% match_wild(+PatternArgs, +ActualArgs): PatternArgs unifies against
% ActualArgs position-by-position, where the GROUND ATOM 'wild' in
% PatternArgs matches ANY value at that position (an argmin ObstacleId
% that's only known at RUNTIME, never at translation time) while every
% OTHER PatternArgs element must match EXACTLY (a Threshold/GX/GY/Cond
% already known when the query itself was generated). 'wild' is a
% plain ATOM here, deliberately NOT a genuine unbound Prolog variable
% (e.g. '_') -- see halted_with_pattern/3's own note on why that
% distinction is exactly what keeps a query(...) declaration reporting
% ONE aggregated probability instead of ProbLog silently splitting it
% into one row per distinct grounding.
match_wild([], []).
match_wild([wild|Ws], [_|As]) :-
    match_wild(Ws, As).
match_wild([W|Ws], [W|As]) :-
    W \= wild,
    match_wild(Ws, As).

% halted_with_pattern(+GroundPattern, +ActionCode, +S): true iff S's
% history contains a halt OR a planning call (see halted_with/2's own
% action-independence note) whose Reason, once ActionCode (tag_reason/
% 3's own trailing argument -- see poss(haltMoveto(...)) and
% do_node(planWith(...))'s own notes; ALWAYS the trailing argument,
% regardless of which action produced it) is stripped back off,
% MATCHES GroundPattern -- i.e. GroundPattern is the
% UNTAGGED Reason0 shape (crashed(wild), guard_break(battery_over
% (70.0)), battery_under(20), completed, ...), with any part that's
% only known at RUNTIME (an argmin ObstacleId for crashed/obstacle_
% in_bound/obstacle_on_path) written as the literal atom 'wild' by
% whoever constructs it, and any part that's ALREADY known at
% TRANSLATION time (a guard's own Cond, a trigger's own Threshold/
% GX/GY) kept LITERAL -- this is what correctly distinguishes e.g.
% guard_break(battery_over(70.0)) from guard_break(neg(obstacle_in_
% bound(0.6))) as two separate rows, rather than collapsing every
% guard into one combined "guard_break" total (a bare-functor grouping
% would lose exactly that distinction, since guard_break's own
% semantic identity lives entirely in its first argument, not its
% functor name). GroundPattern MUST be fully ground (every position
% either 'wild' or a literal, never a bare Prolog variable) -- a real
% unbound variable here would make query(any_reason_pattern_by_action
% (Pattern,ActionCode)) itself non-ground, and ProbLog reports ONE
% result row per distinct GROUNDING of a non-ground query rather than
% aggregating them (verified directly against ProbLog's own engine
% before writing this) -- 'wild' avoids that entirely by keeping the
% query term itself ground, while match_wild/2 still supplies the
% "any value here" matching semantics internally, never exposed to the
% query's own outer term. module/contracts/goal_formula_check.py's
% generate_safety_queries builds exactly this GroundPattern per
% (MoveTo, trigger) pair at translation time, mirroring bt_to_prolog.py
% 's own trigger-name -> Reason-functor mapping (e.g. battery_below ->
% battery_under).
halted_with_pattern(GroundPattern, ActionCode, S) :-
    halted_with(Reason, S),
    Reason =.. [Functor|Args],
    append(Args0, [ActionCode], Args),
    GroundPattern =.. [Functor|WildArgs],
    match_wild(WildArgs, Args0).

% any_reason_pattern(+GroundPattern): P(GroundPattern occurred, from
% ANY action) -- the auto-generated replacement for hand-picked
% aggregates like the old any_collision/any_battery_depletion (now
% any_reason_pattern(crashed(wild)) / any_reason_pattern(battery_
% depleted), generated automatically for every reason this problem's
% own tree can actually produce, not just the two someone thought to
% hand-write).
any_reason_pattern(GroundPattern) :-
    final_situation(S), halted_with_pattern(GroundPattern, _ActionCode, S).

% any_reason_pattern_by_action(+GroundPattern, +ActionCode): the SAME
% probability, split by WHICH MoveTo occurrence produced it -- e.g.
% any_reason_pattern_by_action(crashed(wild), a1) vs. (..., a2) is
% exactly "20% on the goto-goal leg, 10% on the goto-home leg" for a
% combined any_reason_pattern(crashed(wild)) of 30%.
any_reason_pattern_by_action(GroundPattern, ActionCode) :-
    final_situation(S), halted_with_pattern(GroundPattern, ActionCode, S).

% halted_with_pattern_detail(+Pattern, +ActionCode, +S): the DELIBERATE
% OPPOSITE of halted_with_pattern/3's own ground-only discipline --
% Pattern here is expected to contain a genuine UNBOUND Prolog variable
% at whichever position halted_with_pattern's own GroundPattern would
% have written as the atom 'wild' (e.g. crashed(_) rather than
% crashed(wild)). Plain unification via =.. does the matching, with NO
% match_wild/2 involved -- this is intentional: a query built on THIS
% predicate is exactly the non-ground case halted_with_pattern/3's own
% note warns about, and that's the whole point here, not a bug to
% avoid -- ProbLog reports one result row PER DISTINCT GROUNDING of a
% non-ground query, so any_reason_pattern_detail_by_action(crashed(_),
% a1) is what actually ENUMERATES every concrete obstacle a1 could
% have crashed into, each with its own probability, as the sub-rows
% underneath any_reason_pattern_by_action(crashed(wild),a1)'s own
% single aggregated total -- see main.py's print_reason_breakdown for
% how the two are nested together in the report.
halted_with_pattern_detail(Pattern, ActionCode, S) :-
    halted_with(Reason, S),
    Reason =.. [Functor|Args],
    append(Args0, [ActionCode], Args),
    Pattern =.. [Functor|Args0].

% any_reason_pattern_detail_by_action(+Pattern, +ActionCode): see
% halted_with_pattern_detail/3's own note -- one query(...) declaration
% here (Pattern containing a genuine variable, e.g. crashed(_)) yields
% MANY result rows, one per obstacle/whatever-was-runtime-only actually
% observed, each already showing its own CONCRETE value substituted in
% (e.g. any_reason_pattern_detail_by_action(crashed(obs5),a1)) --
% module/contracts/goal_formula_check.py's generate_safety_queries only
% emits this companion query for a (Pattern,ActionCode) pair whose own
% GroundPattern actually contains 'wild' somewhere; a Pattern with
% nothing runtime-only in it (battery_under(20), guard_break(Cond),
% completed, ...) has no meaningful detail level beyond its own
% aggregate and gets no companion query at all.
any_reason_pattern_detail_by_action(Pattern, ActionCode) :-
    final_situation(S), halted_with_pattern_detail(Pattern, ActionCode, S).

% any_condition_status(+Code, +Status): P(the cond() leaf identified by
% Code was actually checked, AND its own Status came out this way, in
% the world resolved by final_situation) -- the direct analogue of
% any_reason_pattern_by_action/2 above, but for CONDITIONS instead of
% action Reasons: Code identifies WHICH cond(C,Code) occurrence in the
% tree (same per-occurrence-code idiom as ActionCode), Status is true
% or false (never a Pattern -- a condition's own outcome has no
% runtime-only argument structure to distinguish, unlike a MoveTo's own
% Reason). module/contracts/goal_formula_check.py's generate_safety_
% queries emits one query(any_condition_status(Code,true)) and one
% query(any_condition_status(Code,false)) per condition occurrence
% bt_to_prolog.py assigned a code to, mirroring exactly how it already
% emits one any_reason_pattern_by_action query per (Pattern,ActionCode)
% pair. Fails outright (contributes zero probability) in any world
% where this Code's own cond() leaf was never reached at all -- same
% "absence, not sentinel" convention checked_with/4 itself already
% follows, so P(any_condition_status(Code,true)) + P(any_condition_
% status(Code,false)) need NOT sum to 1.0 (exactly like a MoveTo's own
% Reason probabilities not summing to 100% when the leg was never
% reached in some worlds).
any_condition_status(Code, Status) :-
    final_situation(S), checked_with(Code, _C, Status, S).

% last_halt(-Reason): a cond() leaf that reads off WHY the MOST RECENT
% moveto_leg halted. Works by searching BACKWARD through S, but ONLY
% ever skipping past BOOKKEEPING markers -- checked(_,_,_) from a
% cond(C,Code) leaf (see do_node(cond(...))'s own note) and
% planned(_,_) from a PlanWith call (see do_node(planWith(...))'s own
% note) -- neither of which represents anything PHYSICALLY happening
% (no time elapsed, no position/battery change, nothing that could make
% "the last halt" stale); it genuinely STOPS (fails, no further skip)
% at any OTHER action (startMoveto, interrupt), which DO mean something
% new has happened since the last halt.
%
% UPDATED from an EARLIER version that pattern-matched ONLY S's own
% OUTERMOST layer directly against do(haltMoveto(...),_), reasoning
% that do_node(moveto_leg(...),...) always ends with haltMoveto as its
% very last step and no do_node level (plain or reactive) ever layered
% another action on top of that S1 on the way up -- which was true
% right up until cond(C,Code) started recording its own checked(...)
% marker (see that predicate's own note): a DistanceBelow placed right
% after a MoveTo -- exactly this project's own now-standard idiom --
% appends ONE MORE do(...) layer on top of the halt before a LATER
% branch's own cond(last_halt(...)) (or recover_obstacle/1, built on
% top of it) ever gets to read it, which the old direct-unification
% version could no longer see through. Skipping bookkeeping markers
% restores the original "single deterministic unification, not a
% search" behavior for the REAL halt underneath, without giving up
% cond()'s own traceability: this still can only ever produce the ONE
% most recent halt (unlike halted_with/2, which walks the WHOLE history
% and can match more than one past action), so it stays a single
% ProbLog world instead of branching into one world per historical
% match. Fails outright (no solution) if S isn't shaped like a
% just-halted moveto (skipping bookkeeping markers) at all -- same
% "absence, not sentinel" convention as everywhere else.
%
% NOT YET GENERIC across leaf/action types -- a caveat for whoever adds
% the next reactive-capable leaf (e.g. a robotic-arm action halting via
% its own haltArmMove(...) instead of haltMoveto(...)): the first
% clause below will simply FAIL to match such an S, silently, not with
% an error -- do(haltArmMove(...),_) doesn't unify with do(haltMoveto
% (...),_), so neg(last_halt(...))-based guards elsewhere would
% trivially succeed even though something genuinely just halted. Every
% new reactive-capable leaf type needs its OWN base clause added here
% (or all leaves funneled through one shared halt-action functor with a
% Kind tag, instead of a differently-named action per leaf type) --
% install_tool/uninstall_tool (below) are the first to actually take
% this up: two MORE base clauses, same shape as haltMoveto's own,
% since halt_install_tool(T,Reason,Status)/halt_uninstall_tool(T,
% Reason,Status) share that EXACT (T,Reason,Status) argument shape by
% deliberate design (see do_node(install_tool_leg(...))'s own note).
holds(last_halt(Reason), do(haltMoveto(_T,Reason,_Status),_SPrev)).
holds(last_halt(Reason), do(halt_install_tool(_T,Reason,_Status),_SPrev)).
holds(last_halt(Reason), do(halt_uninstall_tool(_T,Reason,_Status),_SPrev)).
holds(last_halt(Reason), do(checked(_,_,_), S)) :- holds(last_halt(Reason), S).
holds(last_halt(Reason), do(planned(_,_), S)) :- holds(last_halt(Reason), S).
holds(last_halt(Reason), do(take_sample(_,_,_,_), S)) :- holds(last_halt(Reason), S).

% KNOWN LIMITATION, found while adding cond(C,Code)'s own checked(...)
% marker (Option B): cond(neg(last_halt(...))) -- problem3's OWN Bug0
% guard, "retry the direct path unless the last halt was an
% obstacle_on_path" -- makes ProbLog's grounder raise a spurious
% "non-ground probabilistic clause" error against z/2's own template
% clause in config_generated.pl, even though the actual derivation
% needs no such thing (verified directly: replan_budget(1) still fails
% just as fast, so it isn't the known reactive-redescend blowup; a bare
% cond(last_halt(...)) with NO neg() wrapper, or a cond() whose own
% Status doesn't change do_node(cond(...))'s own S1 shape, both work
% fine). The negation itself is the trigger -- ProbLog's own grounding
% of \+ over a goal that (transitively, via this predicate's own
% checked(...)-skipping clauses above) reaches back through the
% situation apparently forces broader exploration than a plain,
% non-negated call needs, tripping over z/2's non-ground template
% clause somewhere in that wider search. Root-caused down to "negation
% over last_halt/1 specifically, once cond() stopped being S1=S" but
% NOT resolved further: every reformulation tried (recursion moved out
% of holds/2 into a separate skip_bookkeeping/2 helper; a single do_node
% (cond(...)) clause with a shared checked(...) term shape regardless
% of Status) still reproduced it, pointing at something in ProbLog's
% own negation-grounding internals rather than a fixable shape in this
% theory's own clauses. Affects ONLY problem3's own hand-written
% plan_generated.pl (the one tree in this project using neg(last_halt
% (...)) at all) -- every translator-generated tree (problems 0/1/2/4)
% neither uses last_halt/recover_obstacle nor is affected. problem3 was
% already unable to complete a run within any practical timeout before
% this (a separate, pre-existing reactive-redescend combinatorial
% blowup, see perf_diag_two_hop/), so this is a new FAILURE MODE on an
% already-nonfunctional tree, not a regression on a working one.

% recover_obstacle(-ObstacleId): a cond() leaf that RETRIEVES which
% obstacle the branch that led here just halted against, wildcarding
% Threshold -- built on last_halt/1 above, NOT on the whole-history
% obstacle_on_path_obstacle/3 (see that predicate's own family comment
% above it): a Fallback whose first branch plans+walks straight
% (watching obstacle_on_path(Threshold) as a trigger) and whose SECOND
% branch needs to know WHICH obstacle to hand planWith(follow_boarder
% (ObstacleId,...),...) -- Golog's own fallback_node semantics threads the SITUATION S
% forward from a failed branch into the next one (see fallback_node's
% own do_node note near the top of this section), but NEVER a raw
% Prolog variable binding a failed branch happened to make, so the
% second branch cannot simply "reuse" a variable the first branch
% bound; it has to look the obstacle back up from S. Built on last_halt
% /1 rather than halted_with/2 SPECIFICALLY so this stays correct with
% more than one obstacle in play (e.g. straight -> hits obstacle A ->
% follow A's boundary -> line of sight clears -> straight again -> hits
% a DIFFERENT obstacle B): halted_with/2 would match BOTH A's and B's
% halt actions once B is also in S's history, grounding two alternative
% (one stale) worlds; last_halt/1 can only ever see the outermost one,
% so recover_obstacle always reports the obstacle THIS branch is
% actually reacting to. Fails (no solution) if the most recent halt
% wasn't an obstacle_on_path one at all -- same "absence, not
% sentinel" convention as everywhere else -- so a Fallback branch
% guarded by cond(recover_obstacle(Obst)) simply doesn't apply unless
% there really is one to recover.
holds(recover_obstacle(ObstacleId), S) :-
    holds(last_halt(obstacle_on_path(_Threshold,ObstacleId,_ActionCode)), S).

% -- overall collision probability (exact) --------------------------
any_collision :- final_situation(S), crashed_in(S).

% -- overall battery-depletion probability (exact) -------------------
any_battery_depletion :- final_situation(S), battery_depleted_in(S).

% -- sample_index_for_time(+T,+T0,+Duration,-I): bucket an exact time
%    into the nearest reporting sample, for continuity with plotting.
%    Duration=0.0 is a genuine, valid case (a zero-length "already at
%    the goal" leg -- see collision_geometry.py's own _walk_noisy_
%    point note); such a leg's own single instant always buckets to
%    sample 0, the same well-defined choice sample_walk_frac/3 above
%    makes for the identical reason. TWO mutually exclusive clauses,
%    same "no if-then-else" convention as everywhere else in this file.
sample_index_for_time(T,T0,Duration,0) :-
    Duration =< 0.0.
sample_index_for_time(T,T0,Duration,I) :-
    Duration > 0.0,
    num_samples(N),
    FracRaw is (T-T0)/Duration,
    IReal is FracRaw*N,
    IRound is round(IReal),
    I is max(0, min(N, IRound)).

% -- Feature 2b analogue: which reporting sample the (exact) first
%    collision falls nearest to -- a genuine PMF, since crashed_in/1
%    is itself all-or-nothing per world and each crashing world maps
%    to exactly one bucket. T is read DIRECTLY off the actual halted
%    situation (no re-derivation) -- if some OTHER cause preempted
%    collision in this world, S's outermost haltMoveto simply won't
%    match Reason=crashed(_), and this correctly contributes nothing.
%    _ObstacleId is deliberately unbound/ignored here -- first_hit is
%    a PMF over WHEN, not WHICH obstacle; see crashed_obstacle/2 for
%    that question, e.g. crashed_obstacle(ObstacleId,S) alongside
%    final_situation(S) for a specific resolved situation.
first_hit(I) :-
    final_situation(S),
    S = do(haltMoveto(Tcross,crashed(_ObstacleId,_ActionCode),_), _),
    current_walk(S, CP, _Triggers, T0, SPrev),
    hitch(Tool, SPrev),
    walk_duration(CP, Tool, SPrev, Duration),
    sample_index_for_time(Tcross,T0,Duration,I).

% -- Feature 2a analogue: P(the exact collision, if any, falls at or
%    before reporting sample N) ----------------------------------
hit_by(N) :-
    final_situation(S),
    S = do(haltMoveto(Tcross,crashed(_ObstacleId,_ActionCode),_), _),
    current_walk(S, CP, _Triggers, T0, SPrev),
    hitch(Tool, SPrev),
    walk_duration(CP, Tool, SPrev, Duration),
    sample_index_for_time(Tcross,T0,Duration,I),
    I =< N.

% ---------------------------------------------------------------
% FUTURE EXTENSION NOTE: "safety" here is meant to cover EVERY cause
% that could prevent the robot from reaching the goal. Currently
% there are seven, ALL expressed as ordinary Triggers entries (see
% the TRIGGERS section and trigger_crossing_time/9 far above):
% collision (first_collision_time / crashed_in / any_collision),
% battery depletion (first_battery_depletion_time /
% battery_depleted_in / any_battery_depletion), obstacle_in_bound
% (obstacle_in_bound_in/2, first_threshold_crossing_time),
% obstacle_on_path (obstacle_on_path_in/2, first_on_path_crossing_time),
% battery_below (battery_under_in/2, first_battery_below_time),
% battery_equal (battery_equal_in/2, first_battery_equal_time), and
% battery_over (battery_over_in/2, first_battery_over_time).
% Adding an EIGHTH cause later -- a mechanical fault, a comms timeout,
% whatever -- means exactly:
%   (a) one more trigger_crossing_time/10 clause, giving its own
%       Reason and crossing-time computation
%   (b) a dedicated *_in(S) exact-detection predicate (one line, via
%       halted_with/2) + a corresponding any_* diagnostic query, if
%       you want it separately reportable
%   (c) including the new trigger's name in whichever leg(s)' own
%       explicit Triggers list should react to it
% Nothing else in the theory needs to change; earliest_halt/10 and
% verify_safe below already handle an arbitrary Triggers list with no
% further edits.
% ---------------------------------------------------------------

% -- Feature 1 analogue: deterministic nominal-path safety w.r.t.
%    EVERY cause in THIS LEG'S OWN Triggers list, using the SAME
%    shared earliest_halt/9 that Poss(haltMoveto(...)) itself uses
%    for real execution -- just with noise fixed at its zero/modal
%    value (Z=0.0, Zb=0.0) instead of resolved per-world values.
%    Because this is the SAME predicate, not a separate
%    reimplementation, it cannot drift out of sync the way the
%    earlier crashed_in/1 did. If collision/battery aren't in this
%    leg's Triggers, verify_safe correctly won't flag them, matching
%    exactly what real execution would (or wouldn't) halt on.
% B0 here MUST be leg_start_battery/3, not a raw battery(B0,T0,SPrev)
% read: T0 (from current_walk/5 on the ALREADY-RESOLVED final
% situation) is whatever quantized start time real resolution actually
% used for this leg (see poss(startMoveto(...))'s own note), so this
% has to read the SAME quantized B0 that leg's own real
% Poss(haltMoveto(...)) used -- otherwise this "was it safe from THIS
% leg's own start" check could disagree with what the leg ACTUALLY
% started with, exactly the kind of drift this predicate's own header
% comment (just above) warns against.
verify_safe :-
    final_situation(S),
    current_walk(S, CP, Triggers, T0, SPrev),
    hitch(Tool, SPrev),
    walk_duration(CP, Tool, SPrev, Duration),
    effective_tool_moving_drain_rate(Tool, SPrev, Rate),
    leg_start_battery(T0, SPrev, B0),
    earliest_halt(CP,Triggers,T0,Duration,0.0,0.0,0.0,B0,Rate,0, completed,_,_).

plan_route_blocked :- \+ verify_safe.

% -- on-track: does the noisy position stay within tolerance of ----
%    the nominal spline position at sample I? (still sampled -- this
%    is a REPORTING diagnostic about drift magnitude, not a safety
%    detector, so fixed-resolution sampling remains appropriate here)
%    tolerance/1 is now a config fact -- see
%    the problem's own config.yaml's tolerances.on_track.

on_track(I) :-
    final_situation(S),
    current_walk(S, ControlPoints, _),
    sample_time(I, S, T),
    at(X,Y,T,S),
    sample_walk_frac(I, S, WalkFrac),
    nominal_at(NX,NY,WalkFrac,ControlPoints),
    dist(X,Y,NX,NY,D),
    tolerance(Tol),
    D =< Tol.

% goal_reached (the old single-point "arrives near THE goal AND the
% walk's actual recorded outcome was completed" query) is GONE --
% superseded by goal_formula.pl's own goal_formula/1 (verified,
% earlier, to compute the EXACT SAME probability as goal_reached did
% for a single-leg plan), and by visited/2's own more general history
% search for anything beyond that. See this file's own
% verify_goal_formula wrapper further down, and
% the problem's own goal_formula.pl for where "did the plan
% succeed" is now formalized -- there is no longer a global goal/2
% fact anywhere in this theory for a query like this to read.

% ============================================================
% 9. THE POLICY -- an explicit start/end ACTION PAIR:
%        startMoveto(ControlPoints, T0)  ...  haltMoveto(T,Reason)
%    ControlPoints is computed at runtime by a planWith leaf (see
%    plan/1's own note further down) -- there is no longer a
%    hand-authored control_points/1 fact anywhere in this theory
%    (plan_generation/plan/current_plan.pl, which used to hold one
%    alongside start/2 and goal/2, is gone entirely; see start/1's own
%    note below and goal_formula.pl for where the two things it used
%    to carry now live instead).
%
%    T and Reason are left as FREE VARIABLES -- they are DERIVED by
%    Poss(haltMoveto(T,Reason),S), never chosen by the plan: Reason
%    comes out `completed` if the walk finishes without ever coming
%    within the safety margin of an obstacle in that resolved world,
%    or `crashed(ObstacleId)` (with T = the exact collision time, and
%    ObstacleId = which obstacle) otherwise.
%
%    Default plan below runs the walk to its natural halt (no
%    interruption). To model an interruption, replace it with
%    something like:
%
%        plan(seq(startMoveto(CP,0), seq(interrupt(5.0), nil))).
%
%    which ends the walk at T=5.0 regardless of the walk's own
%    natural completion or collision time -- poss(interrupt(T),S)
%    requires moving(S) and T strictly before whichever of those two
%    happens first in the resolved world, so this composes with any
%    future action that also wants to interrupt a walk: just give it
%    its own poss/2 clause requiring moving(S) plus its own trigger,
%    same pattern as interrupt/1 above.
% ============================================================

% start/1 is now a config.yaml fact (the problem's own
% initial_situation.start_x/start_y, via config_generated.pl,
% consulted via problem_data.pl near the top of this file) -- it used to live in
% plan_generation/plan/current_plan.pl, alongside a global goal/2 fact
% and a control_points/1 static fallback, both now GONE: goal
% information lives entirely in the problem's own goal_formula.pl
% (see verify_goal_formula further down), and control_points/1 has had
% no consumer since planWith started computing ControlPoints at
% runtime.

% plan/1 is no longer hand-written here -- it comes from the problem's
% own plan_generated.pl (consulted via problem_data.pl, see Section 0
% above), generated by module/translators/bt_to_prolog.py translating
% the REAL BT.cpp v4 XML tree at the problem's own behavior_tree.xml
% against module/contracts/schema.yaml (main.py does this automatically
% before every run, exactly like it already does for config_generated.pl).
% This is now the single source of truth for the POLICY'S SHAPE: to
% change the tree, edit behavior_tree.xml and re-run, don't hand-edit a
% plan/1 clause here.
%
% Every moveto_leg node in the XML states its Triggers EXPLICITLY, in
% place -- there is no default_triggers/1 fact and no sugar moveto_leg/1
% form anywhere in this theory (removed deliberately: a plan's
% protection level should be visible where the leg is written, not
% inherited from configuration or a convenience default elsewhere).
% [collision,battery] (the shipped tree's own choice) is a PLAIN CHOICE,
% exactly like any other Triggers list -- nothing about collision/
% battery is hardcoded into the action theory itself (see the TRIGGERS
% section above trigger_crossing_time/9). An empty triggers="" list
% would give a genuinely unprotected leg that completes its full
% nominal duration even through an obstacle's margin or an empty
% battery; adding obstacle_in_bound(0.6) (triggers="collision;battery;
% obstacle_in_bound(0.6)") would ALSO reactively halt when an obstacle
% first comes within 0.6 metres, and battery_below(20)
% (triggers="collision;battery;battery_below(20)") would ALSO halt the
% first time the battery drops under 20%, alongside (not instead of)
% the fixed collision/battery(=0%) triggers.
%
% Since the translator handles arbitrary Sequence/Fallback nesting and
% every schema.yaml action/condition, sequence/fallback/multi-leg
% policies -- and conditions like DistanceBelow/HaltedWith/ObstacleInBound/
% BatteryBelow -- are already expressible in the XML with no further
% changes here; see
% bt_to_prolog.py's own header for the blackboard-to-Prolog-variable
% translation this relies on (e.g. giving two different PlanWith
% nodes distinct blackboard keys, same as the CP1/CP2
% convention this file already documents for hand-written multi-leg
% plans). Only the AUTOMATIC generation of multi-leg XML trees and
% reacting to a planWith/moveto_leg FAILURE by re-planning are future
% work; the theory and the translator both already support authoring
% either by hand in the XML.

% goal_formula/1 is hand-authored, tied to THIS PARTICULAR plan's own
% waypoints -- see the problem's own goal_formula.pl header for the
% full rationale and the "must be kept in sync with behavior_tree.xml"
% caveat. This is the ONLY place a plan's own goal information lives --
% there is no separate global goal/2 fact anywhere in this theory (see
% distance_below/3's own note). It is a UNIFORM formula (Reiter's sense -- one
% free situation argument, every fluent inside applied to exactly it);
% verify_goal_formula below is what actually applies it AT
% final_situation, same "zero-arg convenience wrapper hardwired to
% final_situation" shape as any_collision/plan_outcome below.
% Both plan/1 and goal_formula/1 are consulted via problem_data.pl --
% see Section 0 above, near the top of this file.

verify_goal_formula :- final_situation(S), goal_formula(S).

% ============================================================
% 10. QUERIES
%
% NONE hardcoded here anymore -- every query(...) this problem needs
% (verify_goal_formula, plan_outcome(true/false/world_too_large),
% plan_outcome(reactive_escaped), and one any_reason_pattern(...)/
% any_reason_pattern_by_action(...,ActionCode) pair per Reason this
% problem's own tree can actually produce) is written into this
% problem's own queries_generated.pl by module/contracts/
% goal_formula_check.py's generate_safety_queries, called right after
% goal_formula.pl's own validation (see main.py) -- a problem-
% independent theory file can't itself vary per-problem, but the
% generated file it consults (via problem_data.pl -- see Section 0)
% can, and now ALL of it does, not just the one any_battery_depletion
% special case this used to carry by hand.
%
% hit_by/1, first_hit/1, on_track/1, verify_safe/0, and
% plan_route_blocked/0 are all still DEFINED above (Section 7/8) --
% simply never queried by the generator, so they stay ungrounded
% (ProbLog only grounds what a query(...) or something it depends on
% actually reaches), not deleted. Add them to generate_safety_queries'
% own output if per-sample hazard/drift reporting is wanted again.
%
% plan_outcome(reactive_escaped) remains the runtime safety net for
% the "a reactive trigger's own code matched no enclosing reactive
% composite" translator-bug case -- see final_situation/1 and
% plan_outcome/1's own notes above. Always expected to be exactly 0
% for a correctly-translated plan; a nonzero value here is a bug
% report, not a legitimate outcome.
% ============================================================