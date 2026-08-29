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
% 0. OBSTACLES (polygons) -- auto-generated file, see
%    occgrid_to_problog.py. Falls back to a tiny built-in example
%    so this file is runnable stand-alone before you've generated
%    a real map.
% ---------------------------------------------------------------
% Fallback clause so obstacle_polygon/2 is always a KNOWN predicate to
% ProbLog even if obstacles_generated.pl defines zero real obstacles
% (its body always fails, so it never contributes an actual obstacle).
obstacle_polygon(no_obstacles_placeholder, []) :- fail.

% Expected project layout (paths below are relative to THIS file):
%   ./environments/maps/map.yaml (+ .pgm)       -- the source map
%   ./environments/maps/obstacles_generated.pl  -- consulted here
%   ./plan_generation/occupancy_grid_planner.py
%   ./plan_generation/plan/current_plan.pl      -- consulted further down
% Generate the obstacle file with (no --out override needed -- this is
% occgrid_to_problog.py's own actual default filename):
%   python3 occgrid_to_problog.py environments/maps/map.yaml \
%       --out environments/maps/obstacles_generated.pl
:- consult('./environments/maps/obstacles_generated.pl').
% Must exist relative to wherever ProbLog is invoked FROM (typically the
% directory containing this file). Expected fact shape:
%   obstacle_polygon(Id, [point(X1,Y1), point(X2,Y2), ...]).
% If you don't have a map yet, create a stub file containing at least
% one such fact (or an empty file) so this program still loads.

% config/config_generated.pl provides EVERY tunable constant in this
% theory -- robot_radius/1, safety_buffer/1,
% speed/1, sigma/1, sigma_battery/1, battery_start/1,
% idle_drain_rate/1, moving_drain_rate/1, goal_tolerance/1, tolerance/1,
% num_samples/1, bracket_samples/1, crossing_eps/1, and the z/2 and
% zbatt/1 annotated disjunctions -- generated from config/config.yaml by
% config/generate_prolog_config.py (run_plan_continuous_safety.py does
% this automatically before every run). config.yaml is the single
% source of truth; edit IT, not this generated file, to retune the
% system. See config.yaml's own header for the full rationale.
:- consult('./config/config_generated.pl').

% actions/moveto_planners.py provides plan_astar/5, plan_straight/5,
% and follow_boarder/5 as BLACK-BOX (Python-implemented) predicates --
% see that file's own header for the full explanation. ProbLog imports
% and executes it
% directly the moment this directive loads (problog.clausedb's
% load_external_module), registering both predicates before anything
% below that calls them is ever evaluated. Path is resolved relative to
% THIS file's own directory (not CWD) -- moveto_planners.py lives in
% ./actions/ alongside bt_actions.py and schema.yaml (the BT.cpp-facing
% side of the same node set).
:- use_module('./actions/moveto_planners.py').

% collision_geometry.py provides first_threshold_crossing_time/8 as a
% BLACK-BOX (Python-implemented) predicate -- the obstacle-clearance
% geometry and bracket-scan/bisection crossing-time search that used to
% be plain Prolog in sections 1 and the TRIGGERS section below (see
% their own notes for why this moved). Lives NEXT TO this file (not in
% ./actions/ -- it isn't a BT.cpp-facing node, just an internal
% performance black box for the action theory itself), resolved
% relative to THIS file's own directory, same as moveto_planners.py.
:- use_module('./collision_geometry.py').

% ---------------------------------------------------------------
% 1. GEOMETRY HELPERS -- general-purpose arithmetic used throughout
%    the theory (distance, arc-length summation). The OBSTACLE-SPECIFIC
%    geometry that used to live here (point/segment/polygon distance,
%    ray-casting point-in-polygon, signed clearance, min-clearance-to-
%    any-obstacle) has been MOVED to collision_geometry.py, a Python
%    black box exactly like moveto_planners.py's planners -- see the
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

sum_list([], 0.0).
sum_list([H|T], Sum) :- sum_list(T, SumT), Sum is H + SumT.

% ---------------------------------------------------------------
% 2. ROBOT / SAFETY PARAMETERS
% ---------------------------------------------------------------
% robot_radius/1 and safety_buffer/1 are now config facts
% (config/config.yaml -> config/config_generated.pl, consulted above)
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
% BUILT: a per-instance check (e.g. in plan_generation/bt_to_prolog.py)
% that a given obstacle_in_bound(Threshold)'s Threshold is itself
% sensible (> safety_margin) -- currently unchecked, same as any other
% trigger-list entry's argument.
safety_margin(M) :- robot_radius(R), safety_buffer(B), M is R + B.

% within_obstacle_threshold/3 (the generalized "is (PX,PY) within
% Threshold of the nearest obstacle" test, parametrized by threshold so
% the SAME primitive serves collision, obstacle_in_bound, and any
% future distance-based trigger) now lives inside collision_geometry.py's
% within_obstacle_threshold helper, alongside the rest of the
% obstacle-clearance geometry it was
% moved with -- see the note above dist/5 in section 1.

% ---------------------------------------------------------------
% 3. SPLINE -- chained cubic Bezier segments, evaluated in
%    closed form. ControlPoints = [point(X0,Y0), point(X1,Y1),
%    point(X2,Y2), point(X3,Y3), point(X4,Y4), ...], length must
%    be 3k+1 for k segments (segment i uses control points
%    3i..3i+3). A straight line is the degenerate case where the
%    interior control points are collinear with the endpoints.
% ---------------------------------------------------------------
bezier_point(P0x,P0y,P1x,P1y,P2x,P2y,P3x,P3y, U, X, Y) :-
    Mu is 1-U,
    X is Mu*Mu*Mu*P0x + 3*Mu*Mu*U*P1x + 3*Mu*U*U*P2x + U*U*U*P3x,
    Y is Mu*Mu*Mu*P0y + 3*Mu*Mu*U*P1y + 3*Mu*U*U*P2y + U*U*U*P3y.

bezier_tangent(P0x,P0y,P1x,P1y,P2x,P2y,P3x,P3y, U, DX, DY) :-
    Mu is 1-U,
    DX is 3*Mu*Mu*(P1x-P0x) + 6*Mu*U*(P2x-P1x) + 3*U*U*(P3x-P2x),
    DY is 3*Mu*Mu*(P1y-P0y) + 6*Mu*U*(P2y-P1y) + 3*U*U*(P3y-P2y).

spline_num_segments(ControlPoints, NSegs) :-
    length(ControlPoints, N),
    NSegs is (N - 1) // 3.

% drop_n(+N, +List, -Rest): Rest is List with its first N elements removed
drop_n(0, List, List).
drop_n(N, [_|T], Rest) :- N > 0, N1 is N-1, drop_n(N1, T, Rest).

spline_segment_points(ControlPoints, SegIdx, P0,P1,P2,P3) :-
    Skip is SegIdx*3,
    drop_n(Skip, ControlPoints, [P0,P1,P2,P3|_]).

% spline_point(+ControlPoints, +U, -X, -Y): U in [0,1] spans the WHOLE spline
spline_point(ControlPoints, U, X, Y) :-
    spline_num_segments(ControlPoints, NSegs),
    NSegs > 0,
    SegLen is 1.0 / NSegs,
    SegIdx0 is min(NSegs-1, floor(U / SegLen)),
    SegIdx is integer(SegIdx0),
    LocalU0 is (U - SegIdx*SegLen) / SegLen,
    LocalU is max(0.0, min(1.0, LocalU0)),
    spline_segment_points(ControlPoints, SegIdx,
                           point(P0x,P0y),point(P1x,P1y),
                           point(P2x,P2y),point(P3x,P3y)),
    bezier_point(P0x,P0y,P1x,P1y,P2x,P2y,P3x,P3y, LocalU, X, Y).

spline_tangent(ControlPoints, U, DX, DY) :-
    spline_num_segments(ControlPoints, NSegs),
    NSegs > 0,
    SegLen is 1.0 / NSegs,
    SegIdx0 is min(NSegs-1, floor(U / SegLen)),
    SegIdx is integer(SegIdx0),
    LocalU0 is (U - SegIdx*SegLen) / SegLen,
    LocalU is max(0.0, min(1.0, LocalU0)),
    spline_segment_points(ControlPoints, SegIdx,
                           point(P0x,P0y),point(P1x,P1y),
                           point(P2x,P2y),point(P3x,P3y)),
    bezier_tangent(P0x,P0y,P1x,P1y,P2x,P2y,P3x,P3y, LocalU, DX, DY).

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

% speed/1 is now a config fact -- see config/config.yaml's motion.speed.

walk_duration(ControlPoints, Duration) :-
    arc_length(ControlPoints, Length),
    speed(Speed),
    Duration is Length / Speed.

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
% config facts -- see config/config.yaml's noise.discretized_gaussian
% and noise.position_sigma, generated into config/config_generated.pl
% (consulted at the top of this file) by
% config/generate_prolog_config.py. NOTE kept here as a standing
% reminder even though the table itself moved: these five weights must
% sum to EXACTLY 1.0 (0.0606*2 + 0.2417*2 + 0.3954 = 1.0000) -- an
% earlier version of this table used 0.3854 for the centre weight,
% which summed to only 0.99, and ProbLog silently treats missing mass
% as an implicit "none of these" failure branch, which caps EVERY
% downstream probability at 0.99 in every world. The generator checks
% this sum and warns if it's off; always verify it after editing
% config.yaml regardless.

% ---------------------------------------------------------------
% 5. THE MOVING FLUENT -- true while a walk is in progress (between
%    startMoveto and whatever action ends it: haltMoveto OR
%    interrupt). Other actions can gate their Poss axioms on
%    \+ moving(S) (can't start a new walk while moving) or on moving(S)
%    (an interrupt can only fire while a walk IS in progress).
%    Pure regression, exactly on the same footing as at/4.
% ---------------------------------------------------------------
moving(do(startMoveto(_,_,_), _)).
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
% interrupt's Poss). This is now the PRIMARY definition; /3 and /4
% above are thin wrappers over it, same pattern as when SPrev was
% added to /3 earlier.
current_walk(do(startMoveto(CP,Triggers,T0),SPrev), CP, Triggers, T0, SPrev).
current_walk(do(A,S), CP, Triggers, T0, SPrev) :-
    A \= startMoveto(_,_,_),
    current_walk(S, CP, Triggers, T0, SPrev).

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
% config/config.yaml's battery.* and noise.battery_sigma.

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
% own zero-argument functor by config/generate_prolog_config.py, so the
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

% MOVING phase keeps the Duration-normalized scaling (Elapsed/sqrt(D),
% not sqrt(Elapsed)) -- a walk DOES have a genuine, known, fixed
% Duration, and that normalization is what keeps Level EXACTLY LINEAR
% in elapsed time (needed for first_battery_depletion_time's
% closed-form algebraic solve, rather than bracket-scan+bisection).
% Structurally this is now the SAME "nominal drain minus a signed
% deviation, clamped at zero" pattern as the idle phases -- only the
% Deviation formula's normalization differs, for the reason above.
battery(Level, T, do(startMoveto(CP,Triggers,T0), S)) :-
    battery(B0, T0, S),
    walk_duration(CP, Duration),
    Elapsed0 is T - T0,
    Elapsed is max(0.0, min(Elapsed0, Duration)),
    moving_drain_rate(MovingRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Deviation is Zb * SigmaB * Elapsed / sqrt(Duration),
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

% pass-through: any future non-movement action doesn't change how
% battery is computed -- it's a pure function of T and of whichever
% startMoveto/haltMoveto/interrupt anchors exist in the history, same
% principle as at/4's own pass-through clause.
battery(Level, T, do(A,S)) :-
    A \= startMoveto(_,_,_), A \= haltMoveto(_,_,_), A \= interrupt(_),
    battery(Level, T, S).

% first_battery_depletion_time(+CP,+T0,+Duration,+B0,+Zb,-Tcross):
% CLOSED-FORM (not bracket/bisect) -- battery is exactly LINEAR in
% elapsed time within one walk (fixed Zb => fixed EffectiveRate), so
% the crossing at Level=0 is a direct algebraic solve. FAILS (no
% clause matches) if the effective rate is non-positive (battery
% isn't actually decreasing -- noise happened to push it flat/up) or
% the algebraic crossing falls beyond this walk's own span -- both
% correctly represent "no depletion in this walk," exactly mirroring
% how first_collision_time fails when there's no crossing.
first_battery_depletion_time(CP,T0,Duration,B0,Zb,Tcross) :-
    moving_drain_rate(MovingRate),
    sigma_battery(SigmaB),
    EffectiveRate is MovingRate - Zb*SigmaB/sqrt(Duration),
    EffectiveRate > 0,
    Tcross0 is T0 + B0/EffectiveRate,
    Tcross0 =< T0 + Duration,
    Tcross = Tcross0.

% first_battery_below_time(+CP,+T0,+Duration,+B0,+Zb,+Threshold,-Tcross):
% the SAME closed-form algebra as first_battery_depletion_time above,
% generalized to an arbitrary Threshold instead of hardcoded Level=0 --
% this is what battery_below(Threshold) (see TRIGGERS section and
% holds(battery_below(...)) further down) uses, kept as a genuinely
% SEPARATE predicate from first_battery_depletion_time/6 (not a
% generalize-in-place rename) since battery_depleted (Level=0 exactly)
% stays its own distinct trigger/Reason, on request. UNLIKE
% first_battery_depletion_time, B0 can legitimately already be AT OR
% BELOW an arbitrary Threshold at T0 (e.g. an earlier leg already
% drained below a later leg's chosen warning level without itself
% using battery_below) -- so this needs the explicit "already true at
% T0" graceful clause every other threshold-crossing predicate in this
% theory already has, rather than relying on the algebra to degrade
% gracefully on its own (it only does that at exactly Threshold=0).
first_battery_below_time(CP,T0,Duration,B0,Zb,Threshold,T0) :-
    B0 =< Threshold.
first_battery_below_time(CP,T0,Duration,B0,Zb,Threshold,Tcross) :-
    B0 > Threshold,
    moving_drain_rate(MovingRate),
    sigma_battery(SigmaB),
    EffectiveRate is MovingRate - Zb*SigmaB/sqrt(Duration),
    EffectiveRate > 0,
    Tcross0 is T0 + (B0-Threshold)/EffectiveRate,
    Tcross0 =< T0 + Duration,
    Tcross = Tcross0.

% first_battery_equal_time(+CP,+T0,+Duration,+B0,+Zb,+Threshold,-Tcross):
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
first_battery_equal_time(CP,T0,Duration,B0,Zb,Threshold,T0) :-
    B0 =:= Threshold.
first_battery_equal_time(CP,T0,Duration,B0,Zb,Threshold,Tcross) :-
    B0 > Threshold,
    moving_drain_rate(MovingRate),
    sigma_battery(SigmaB),
    EffectiveRate is MovingRate - Zb*SigmaB/sqrt(Duration),
    EffectiveRate > 0,
    Tcross0 is T0 + (B0-Threshold)/EffectiveRate,
    Tcross0 =< T0 + Duration,
    Tcross = Tcross0.

% first_battery_over_time(+CP,+T0,+Duration,+B0,+Zb,+Threshold,-Tcross):
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
% first_battery_below_time/first_battery_equal_time use
% moving_drain_rate/sigma_battery today) -- CP/Duration/Zb are already
% threaded through, genuinely used (not wildcarded away), specifically
% so that clause can be added without changing this predicate's
% signature or any of its callers.
first_battery_over_time(CP,T0,Duration,B0,Zb,Threshold,T0) :-
    B0 > Threshold.

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
% collision (fixed threshold=safety_margin) and battery (fixed
% threshold=exactly-0, Reason=battery_depleted) are the ORIGINAL,
% UNPARAMETRIZED trigger names -- kept exactly as they were, on
% request, rather than folded into the generic versions below.
% obstacle_in_bound(Threshold) and battery_below(Threshold) are
% GENUINELY SEPARATE, ADDITIONAL trigger names -- a leg can react to
% EITHER or BOTH of a fixed floor and an arbitrary per-call threshold
% at once, e.g. Triggers=[collision,battery,battery_below(20)] halts on
% whichever of "hits an obstacle", "hits exactly empty", or "drops
% under 20%" happens earliest. obstacle_in_bound is what "obstacle
% sighted" was renamed to (see the note above first_threshold_crossing_
% time below for why sight_threshold/1 is gone): it reuses
% first_threshold_crossing_time DIRECTLY, with Threshold now the
% CALLER'S OWN argument instead of a fixed config constant -- the exact
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
% whose ControlPoints came from moveto_planners.py's follow_boarder
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
trigger_crossing_time(collision, CP,T0,Duration,Z,Zt,_Zb,_B0, crashed(ObstacleId), Tcross) :-
    first_collision_time(CP,T0,Duration,Z,Zt,Tcross,ObstacleId).

trigger_crossing_time(battery, CP,T0,Duration,_Z,_Zt,Zb,B0, battery_depleted, Tcross) :-
    first_battery_depletion_time(CP,T0,Duration,B0,Zb,Tcross).

trigger_crossing_time(obstacle_in_bound(Threshold), CP,T0,Duration,Z,Zt,_Zb,_B0, obstacle_in_bound(Threshold,ObstacleId), Tcross) :-
    first_threshold_crossing_time(CP,T0,Duration,Z,Zt,Threshold,Tcross,ObstacleId).

trigger_crossing_time(obstacle_on_path(Threshold), CP,T0,Duration,Z,Zt,_Zb,_B0, obstacle_on_path(Threshold,ObstacleId), Tcross) :-
    first_on_path_crossing_time(CP,T0,Duration,Z,Zt,Threshold,Tcross,ObstacleId).

trigger_crossing_time(battery_below(Threshold), CP,T0,Duration,_Z,_Zt,Zb,B0, battery_under(Threshold), Tcross) :-
    first_battery_below_time(CP,T0,Duration,B0,Zb,Threshold,Tcross).

trigger_crossing_time(battery_equal(Threshold), CP,T0,Duration,_Z,_Zt,Zb,B0, battery_equal(Threshold), Tcross) :-
    first_battery_equal_time(CP,T0,Duration,B0,Zb,Threshold,Tcross).

trigger_crossing_time(battery_over(Threshold), CP,T0,Duration,_Z,_Zt,Zb,B0, battery_over(Threshold), Tcross) :-
    first_battery_over_time(CP,T0,Duration,B0,Zb,Threshold,Tcross).

trigger_crossing_time(line_of_sight_clear(ObstacleId,GX,GY), CP,T0,Duration,Z,Zt,_Zb,_B0, line_of_sight_clear(ObstacleId,GX,GY), Tcross) :-
    first_line_of_sight_clear_time(CP,T0,Duration,Z,Zt,ObstacleId,GX,GY,Tcross).

trigger_crossing_time(crosses_segment(SX,SY,GX,GY), CP,T0,Duration,Z,Zt,_Zb,_B0, crosses_segment(SX,SY,GX,GY), Tcross) :-
    first_segment_crossing_time(CP,T0,Duration,Z,Zt,SX,SY,GX,GY,Tcross).

% all_trigger_candidates(+Triggers,...,-Candidates): Candidates is a
% list of Reason-Time pairs, one per trigger in Triggers that ACTUALLY
% fires in this resolved world (triggers that don't fire contribute
% nothing -- same "absence, not sentinel" convention as everywhere
% else). Unrecognized trigger names (no matching
% trigger_crossing_time/10 clause) are silently skipped, same as "never
% fires" -- lenient by design, so a typo in a Triggers list doesn't
% halt the whole theory, just means that trigger never contributes.
all_trigger_candidates([], _,_,_,_,_,_,_, []).
all_trigger_candidates([Trig|Rest], CP,T0,Duration,Z,Zt,Zb,B0, [Reason-Tcross|RestCands]) :-
    trigger_crossing_time(Trig, CP,T0,Duration,Z,Zt,Zb,B0, Reason, Tcross),
    all_trigger_candidates(Rest, CP,T0,Duration,Z,Zt,Zb,B0, RestCands).
all_trigger_candidates([Trig|Rest], CP,T0,Duration,Z,Zt,Zb,B0, RestCands) :-
    \+ trigger_crossing_time(Trig, CP,T0,Duration,Z,Zt,Zb,B0, _, _),
    all_trigger_candidates(Rest, CP,T0,Duration,Z,Zt,Zb,B0, RestCands).

% earliest_of(+PairsList, -ReasonTimePair): generic "minimum by second
% element" over a non-empty list of Reason-Time pairs. Used to combine
% natural completion with however many Triggers-derived candidates
% happen to apply, into ONE earliest-wins choice -- this is the
% mechanism that makes moveto a genuine template: it works identically
% whether Triggers is empty, or contains one, or many conditions
% (including collision/battery themselves now), with no change needed
% here.
earliest_of([Pair], Pair).
earliest_of([R1-T1|Rest], Result) :-
    earliest_of(Rest, _-T2),
    T1 =< T2,
    Result = R1-T1.
earliest_of([R1-T1|Rest], Result) :-
    earliest_of(Rest, R2-T2),
    T1 > T2,
    Result = R2-T2.

% walk_noisy_point(+CP,+T0,+Duration,+Z,+Zt,+T,-X,-Y): position along
% the spline at time T, given TWO ALREADY-RESOLVED, INDEPENDENT noise
% draws (rather than looking them up via z/2 or zt/2 itself): Z (lateral
% /normal drift, unchanged from before) and Zt (tangential/along-path
% drift -- a straight metric push along the spline's own tangent
% direction at each point, NOT a reparametrization of Frac; see
% collision_geometry.py's _walk_noisy_point for the identical formula
% and the reasoning for why Option B, a metric offset, was chosen over
% shifting Frac itself). Factored out of at/4 so the same formula can
% be reused by first_collision_time's bracket/bisection search below,
% without re-deriving Z/Zt through a different situation.
walk_noisy_point(ControlPoints, T0, Duration, Z, Zt, T, X, Y) :-
    Elapsed0 is T - T0,
    Elapsed is max(0.0, min(Elapsed0, Duration)),
    Frac is Elapsed / Duration,
    spline_point(ControlPoints, Frac, NX, NY),
    spline_tangent(ControlPoints, Frac, DX, DY),
    Norm is sqrt(DX*DX + DY*DY),
    perp_unit(Norm, DX, DY, PerpX, PerpY),
    tangent_unit(Norm, DX, DY, TanX, TanY),
    sigma(Sigma),
    sigma_tangential(SigmaT),
    Deviation is Z * Sigma * sqrt(Duration) * Frac,
    TangentDev is Zt * SigmaT * sqrt(Duration) * Frac,
    X is NX + Deviation*PerpX + TangentDev*TanX,
    Y is NY + Deviation*PerpY + TangentDev*TanY.

% ---------------------------------------------------------------
% FIRST-THRESHOLD-CROSSING-TIME -- a NATURAL (not chosen) event: the
% earliest time, within a given resolved world (fixed Z), at which
% the noisy trajectory comes within a given distance THRESHOLD of an
% obstacle. GENERALIZED over the threshold (rather than hardcoded to
% collision's safety_margin) so the SAME machinery serves collision
% (threshold=safety_margin, via first_collision_time/6 below),
% obstacle_in_bound(Threshold) (called DIRECTLY with the caller's own
% Threshold -- see trigger_crossing_time/9 above and
% holds(obstacle_in_bound(...)) below, no separate wrapper predicate
% needed since this black box was already threshold-generic), and any
% future distance-based trigger.
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
% facts too (see config/config.yaml's
% verification.bracket_samples/crossing_eps). collision_geometry.py
% reads them (and sigma/1's value) directly out of config/config.yaml
% itself, not out of this generated fact, since config.yaml is the
% actual single source of truth both sides are driven from -- see
% collision_geometry.py's own header. FAILS (0 ProbLog solutions) if
% the trajectory never comes within Threshold of an obstacle in this
% resolved world -- correctly representing "never happens" via
% absence, not a sentinel value, same convention as everywhere else in
% this theory.

% first_collision_time/6 kept as a thin, name-preserving wrapper over
% the generalized machinery, at threshold=safety_margin -- every
% EXISTING caller (crashed_in, verify_safe, etc.) is unaffected beyond
% the new ObstacleId output.
first_collision_time(CP,T0,Duration,Z,Zt,Tcross,ObstacleId) :-
    safety_margin(M),
    first_threshold_crossing_time(CP,T0,Duration,Z,Zt,M,Tcross,ObstacleId).

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
poss(startMoveto(_,_Triggers,T0), S) :-
    \+ moving(S),
    now(S, T0).

% now(+S,-T): current wall-clock time -- needed above only to know
% WHEN to check the battery level (battery/3 needs a query time).
now(s0, 0).
now(do(startMoveto(_,_,T),_), T).
now(do(haltMoveto(T,_,_),_), T).
now(do(interrupt(T),_), T).
now(do(A,S), T) :-
    A \= startMoveto(_,_,_), A \= haltMoveto(_,_,_), A \= interrupt(_),
    now(S, T).

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
% earliest_halt/10 is the SINGLE SHARED definition of "what happens
% first" -- used here, by Poss(interrupt(...)) below, AND by
% verify_safe further down (called there with Z=0.0,Zt=0.0,Zb=0.0
% instead of the resolved noise). Having exactly ONE definition, rather
% than the same computation duplicated at each call site, is what
% guarantees every query stays consistent with what Poss(haltMoveto(...))
% itself actually derives -- see the "SAFETY QUERIES READ THE ACTUAL
% OUTCOME" note further down for why this matters.
earliest_halt(CP,Triggers,T0,Duration,Z,Zt,Zb,B0, Reason,T) :-
    all_trigger_candidates(Triggers, CP,T0,Duration,Z,Zt,Zb,B0, ExtraCandidates),
    NaturalEnd is T0 + Duration,
    earliest_of([completed-NaturalEnd], ExtraCandidates, Reason-T).

poss(haltMoveto(T, Reason, Status), S) :-
    moving(S),
    current_walk(S, CP, Triggers, T0, SPrev),
    walk_duration(CP, Duration),
    z(do(startMoveto(CP,Triggers,T0),SPrev), Z),
    zt(do(startMoveto(CP,Triggers,T0),SPrev), Zt),
    zbatt(Zb),
    battery(B0, T0, SPrev),
    earliest_halt(CP,Triggers,T0,Duration,Z,Zt,Zb,B0, Reason,T),
    leg_status(Reason, CP, T0, Duration, Z, Zt, T, Status).

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

% leg_status(+Reason,+CP,+T0,+Duration,+Z,+Zt,+T,-Status): the NEW
% OUTPUT -- Status is TRUE iff Reason=completed AND the actual (noisy)
% final position lands within goal_tolerance of the leg's OWN endpoint.
% Deliberately DISTINCT from Reason: Reason=completed only means the
% walk wasn't cut short (by collision/battery/a trigger) before its
% nominal duration elapsed -- it says nothing about whether noise
% carried the robot far enough off course to miss the target despite
% "completing". Status is what a Sequence/Fallback composite (below)
% actually branches on.
leg_status(completed, CP, T0, Duration, Z, Zt, T, true) :-
    walk_noisy_point(CP, T0, Duration, Z, Zt, T, X, Y),
    leg_target(CP, GX, GY),
    dist(X, Y, GX, GY, D),
    goal_tolerance(Tol),
    D =< Tol.
leg_status(completed, CP, T0, Duration, Z, Zt, T, false) :-
    walk_noisy_point(CP, T0, Duration, Z, Zt, T, X, Y),
    leg_target(CP, GX, GY),
    dist(X, Y, GX, GY, D),
    goal_tolerance(Tol),
    D > Tol.
leg_status(Reason, _,_,_,_,_,_, false) :- Reason \= completed.

% earliest_of/3: like earliest_of/2, but takes a fixed head list
% (currently just [completed-NaturalEnd]) and a (possibly empty) list
% of Triggers-derived candidates separately, and combines them --
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
    walk_duration(CP, Duration),
    z(do(startMoveto(CP,Triggers,T0),SPrev), Z),
    zt(do(startMoveto(CP,Triggers,T0),SPrev), Zt),
    zbatt(Zb),
    battery(B0, T0, SPrev),
    earliest_halt(CP,Triggers,T0,Duration,Z,Zt,Zb,B0, _Reason,Tend),
    T >= T0, T < Tend.

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

at(X,Y,T, do(startMoveto(ControlPoints,Triggers,T0), S)) :-
    walk_duration(ControlPoints, Duration),
    z(do(startMoveto(ControlPoints,Triggers,T0),S), Z),
    zt(do(startMoveto(ControlPoints,Triggers,T0),S), Zt),
    walk_noisy_point(ControlPoints, T0, Duration, Z, Zt, T, X, Y).

at(X,Y,T, do(haltMoveto(T1,_Reason,_Status), S)) :-
    Tc is min(T,T1),
    at(X,Y,Tc,S).

at(X,Y,T, do(interrupt(T1), S)) :-
    Tc is min(T,T1),
    at(X,Y,Tc,S).

% perp_unit(+Norm, +DX, +DY, -PerpX, -PerpY): unit vector perpendicular
% to tangent (DX,DY); degenerate (near-zero tangent) case gives (0,0)
perp_unit(Norm, _, _, 0.0, 0.0) :- Norm =< 1.0e-9.
perp_unit(Norm, DX, DY, PerpX, PerpY) :-
    Norm > 1.0e-9,
    PerpX is -DY/Norm, PerpY is DX/Norm.

% tangent_unit(+Norm, +DX, +DY, -TanX, -TanY): unit vector ALONG
% tangent (DX,DY) itself -- the along-path direction tangential noise
% is applied in, as opposed to perp_unit's across-path direction;
% degenerate (near-zero tangent) case gives (0,0), same convention.
tangent_unit(Norm, _, _, 0.0, 0.0) :- Norm =< 1.0e-9.
tangent_unit(Norm, DX, DY, TanX, TanY) :-
    Norm > 1.0e-9,
    TanX is DX/Norm, TanY is DY/Norm.

% pass-through clause: kept for extensibility, so that additional
% actions that DON'T affect position (e.g. a future sensing action)
% can be appended without breaking the regression.
at(X,Y,T, do(A,S)) :-
    A \= startMoveto(_,_,_), A \= haltMoveto(_,_,_), A \= interrupt(_),
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
%     cond(C)                  -- CONDITION leaf: tests C against the
%                                  CURRENT situation via holds/2, no
%                                  side effect (S1 = S).
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
%                                  actions/moveto_planners.py, binding CP for a
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
primitive_action(startMoveto(_,_,_)).
primitive_action(haltMoveto(_,_,_)).
primitive_action(interrupt(_)).

do_action(A, S, do(A,S)) :- primitive_action(A), poss(A, S).

% -- CONDITION leaf ---------------------------------------------------
do_node(cond(C), S, S, true)  :- holds(C, S).
do_node(cond(C), S, S, false) :- \+ holds(C, S).

% -- ACTION leaf --------------------------------------------------------
% moveto_leg(CP,Triggers) -- Triggers is ALWAYS given explicitly here;
% there is deliberately NO sugar/default form (no moveto_leg/1, no
% config-driven fallback) -- every call site states its own protection
% level, e.g. moveto_leg(CP,[collision,battery]) or moveto_leg(CP,[])
% for a genuinely unprotected leg. Status flows straight through as
% Outcome -- no translation predicate needed, since both already speak
% true/false.
do_node(moveto_leg(CP,Triggers), S, S1, Status) :-
    do_action(startMoveto(CP,Triggers,_T0), S, S2),
    do_action(haltMoveto(_T,_Reason,Status), S2, S1).

% -- PLANNING actions: deliberately NOT part of the full action theory
%    -- no primitive_action/1 entry, no Poss axiom, no do_action call
%    with its precondition check (these are stateless, purely-
%    computational black-box calls into actions/moveto_planners.py, not
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
%    to actions/moveto_planners.py plus one more pair of plan_call/8 clauses
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

% follow_boarder(ObstacleId,Offset) -- a THIRD planner, exactly the
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
% ObstacleId's offset boundary (moveto_planners.py's follow_boarder/5),
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
% and Status comes out false (not within goal_tolerance of the loop's
% own arbitrary endpoint) via the ordinary leg_status mechanism -- no
% special no_path case needed here for that.
plan_call(follow_boarder(ObstacleId,Offset), SX,SY,_GX,_GY, CP, completed, true) :-
    follow_boarder(SX,SY,ObstacleId,Offset, CP).
plan_call(follow_boarder(ObstacleId,Offset), SX,SY,_GX,_GY, [], no_path, false) :-
    \+ follow_boarder(SX,SY,ObstacleId,Offset, _).

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
%    aiming at one shared destination. goal/2 itself is UNCHANGED and
%    still used elsewhere (at_goal/1, goal_reached) -- only planWith's
%    OWN target is now per-call rather than global. Status flows
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
%        fallback_node([seq_node([planWith(astar,point(GX,GY),CP1), moveto_leg(CP1,...)]),
%                        seq_node([planWith(straight,point(GX,GY),CP2), moveto_leg(CP2,...)])])
%    NOT a single CP variable shared across both branches. Reusing one
%    CP across fallback alternatives silently breaks: a FAILING
%    planWith still SUCCEEDS as a do_node call (with Outcome=false,
%    CP=[]) -- do_node calls don't get "undone" on Outcome=false the
%    way a genuine Prolog failure would -- so CP is left bound to []
%    by the first branch, and the SECOND branch's own attempt to bind
%    CP to its own (non-empty) result then fails to unify, breaking
%    the fallback in a confusing way that looks unrelated to variable
%    scoping. This was hit directly while testing this exact feature.
do_node(planWith(Algorithm, point(GX,GY), CP), S, do(planned(Algorithm,Reason), S), Status) :-
    now(S, T), at(SX,SY,T,S),
    plan_call(Algorithm, SX,SY,GX,GY, CP, Reason, Status).

% planned_with(+Algorithm, +Reason, +S): the direct parallel to
% halted_with/2, for the (now-recorded) planning marker. Searches the
% WHOLE history, so it can distinguish which of SEVERAL planning
% attempts (across different legs, or different fallback branches)
% produced a given Reason, and with which algorithm.
planned_with(Algorithm, Reason, do(planned(Algorithm,Reason), _)).
planned_with(Algorithm, Reason, do(_A, S)) :- planned_with(Algorithm, Reason, S).

% -- SEQUENCE composite: stop and FAIL at the first failing child; --
%    succeed only if every child succeeds, in order.
do_node(seq_node([]), S, S, true).
do_node(seq_node([Child|Rest]), S, S1, Outcome) :-
    do_node(Child, S, S2, true),
    do_node(seq_node(Rest), S2, S1, Outcome).
do_node(seq_node([Child|_]), S, S1, false) :-
    do_node(Child, S, S1, false).

% -- FALLBACK (Selector) composite: stop and SUCCEED at the first ---
%    succeeding child; fail only if every child fails, in order.
%    NOTE: on a failing child, the NEXT child starts from THAT
%    child's resulting situation, not from the original S -- a failed
%    PHYSICAL action (e.g. a crashed moveto_leg) still consumed real
%    time and moved the robot; unlike a classical BT's usual
%    assumption that failed leaves are side-effect-free, a fallback
%    over durative ACTIONS here means "try the next option from
%    wherever the failed attempt left us," not "rewind and try the
%    next option from the start."
do_node(fallback_node([]), S, S, false).
do_node(fallback_node([Child|_]), S, S1, true) :-
    do_node(Child, S, S1, true).
do_node(fallback_node([Child|Rest]), S, S1, Outcome) :-
    do_node(Child, S, S2, false),
    do_node(fallback_node(Rest), S2, S1, Outcome).

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
% ended, e.g. cond(halted_with_cond(battery_depleted)).
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
% plan_generation/bt_to_prolog.py), this is HaltedWith's reason port,
% written the same way: reason="crashed(_)" or reason="crashed(obs5)".
holds(halted_with_cond(Reason), S) :- halted_with(Reason, S).

% at_goal(Tol): true iff the CURRENT position (at the current time,
% via now/2) is within Tol of goal/2. Typical use: a fallback child
% that skips moveto entirely if already there --
%   fallback_node([cond(at_goal(0.3)), moveto_leg(CP,[collision,battery])])
%
% TODO / KNOWN GAP: this reads the GLOBAL goal/2 fact (from
% plan_generation/plan/current_plan.pl) -- NOT any PlanAstar/
% PlanStraight node's own `goal` port (a literal in behavior_tree.xml,
% see plan_generation/bt_to_prolog.py's translation). The two are
% independent numbers that only happen to agree today because
% behavior_tree.xml was hand-written to match goal/2 at the time. If
% either changes without the other, at_goal/goal_reached below would
% silently check against a stale/different point than what PlanAstar
% actually planned toward. Left as-is for now (accepted, discussed
% explicitly) -- a real fix belongs here eventually, e.g. having the
% translator read goal/2 itself and refuse a disagreeing XML value, or
% substitute it automatically when the XML's goal port is left unset.
holds(at_goal(Tol), S) :-
    now(S, T), at(X,Y,T,S), goal(GX,GY), dist(X,Y,GX,GY,D), D =< Tol.

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
    now(S, T), at(X,Y,T,S),
    obstacle_within_threshold(X,Y,Threshold).

% line_of_sight_clear(ObstacleId,GX,GY): true iff the CURRENT position
% is NOT occluded from (GX,GY) by ObstacleId's own boundary. Same
% underlying primitive as the line_of_sight_clear(ObstacleId,GX,GY)
% TRIGGER (Bug0's own leave rule -- see trigger_crossing_time/10) --
% but this checks ONE point (the current situation) via a single call
% to line_of_sight_clear/5, exactly the same "single check, no
% bracket-scan" relationship obstacle_in_bound has to its own trigger.
holds(line_of_sight_clear(ObstacleId,GX,GY), S) :-
    now(S, T), at(X,Y,T,S),
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
    walk_duration(CP, Duration),
    z(do(startMoveto(CP,Triggers,T0),SPrev), Z),
    zt(do(startMoveto(CP,Triggers,T0),SPrev), Zt),
    now(S, T), at(X,Y,T,S),
    obstacle_on_path_within_threshold(CP,T0,Duration,Z,Zt,X,Y,Threshold).

% battery_below(Threshold): true iff the CURRENT battery level (at the
% current time, via now/2) is below Threshold. Same parameter, same
% underlying fluent as the battery_below(Threshold) TRIGGER (see
% trigger_crossing_time/9) -- but this is a single battery/3 lookup at
% the current situation, not first_battery_below_time's forward-looking
% closed-form solve; no black box involved at all, battery/3 is already
% plain Prolog.
holds(battery_below(Threshold), S) :-
    now(S, T), battery(Level, T, S),
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
    now(S, T), battery(Level, T, S),
    Level =:= Threshold.
holds(battery_over(Threshold), S) :-
    now(S, T), battery(Level, T, S),
    Level > Threshold.

% ---------------------------------------------------------------
% 8. VERIFICATION-TIME SAMPLING (NOT part of the action theory) --
%    a purely deterministic choice of how finely to CHECK/REPORT the
%    already-closed-form at/4 fluent for VISUALIZATION purposes.
%    NOTE: collision DETECTION itself is now EXACT (see
%    first_collision_time above) -- num_samples/1 only controls the
%    resolution used for plotting and for on_track's drift reporting,
%    it no longer determines whether a collision is found at all.
%    num_samples/1 is now a config fact -- see
%    config/config.yaml's verification.num_samples.
% ---------------------------------------------------------------

sample_frac(I, Frac) :- num_samples(N), Frac is I / N.

final_situation(S) :- plan(Node), do_node(Node, s0, S, _).

% plan_outcome(Outcome): the WHOLE tree's true/false outcome, a
% first-class query -- P(plan_outcome(true)) is the BT-level analogue
% of goal_reached, but based on Status/Outcome rather than a
% hand-written distance check.
plan_outcome(Outcome) :- plan(Node), do_node(Node, s0, _, Outcome).

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
last_action_time(do(startMoveto(_,_,_),_), CP, T0, TEnd) :-
    walk_duration(CP, Duration),
    TEnd is T0 + Duration.
last_action_time(do(A,S), CP, T0, TEnd) :-
    A \= haltMoveto(_,_,_), A \= interrupt(_), A \= startMoveto(_,_,_),
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
sample_walk_frac(I, S, WalkFrac) :-
    plan_time_span(S, T0, TEnd),
    current_walk(S, CP, T0),
    walk_duration(CP, Duration),
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

% halted_with(+Reason, +S): TRUE iff SOMEWHERE in S's action history a
% haltMoveto occurred with exactly this Reason. Searches the WHOLE
% history (not just the most recent halt), so a future multi-leg plan
% where an earlier leg had a different fate than the final leg is
% still handled correctly.
halted_with(Reason, do(haltMoveto(_,Reason,_), _)).
halted_with(Reason, do(_A, S)) :- halted_with(Reason, S).

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
crashed_in(S) :- halted_with(crashed(_), S).
battery_depleted_in(S) :- halted_with(battery_depleted, S).
obstacle_in_bound_in(S) :- halted_with(obstacle_in_bound(_,_), S).
obstacle_on_path_in(S) :- halted_with(obstacle_on_path(_,_), S).
battery_under_in(S) :- halted_with(battery_under(_), S).
battery_equal_in(S) :- halted_with(battery_equal(_), S).
battery_over_in(S) :- halted_with(battery_over(_), S).

% -- crashed_obstacle(S,ObstacleId) / obstacle_in_bound_obstacle(S,
%    Threshold,ObstacleId) / obstacle_on_path_obstacle(S,Threshold,
%    ObstacleId) / battery_under_threshold(S,Threshold) /
%    battery_equal_threshold(S,Threshold) /
%    battery_over_threshold(S,Threshold): the direct accessors for
%    WHICH obstacle/threshold -- unlike the *_in(S) checks above, the
%    extra argument(s) are left bound, not wildcarded. Fails (no
%    solution) if S didn't halt for that reason, same "absence, not
%    sentinel" convention as everywhere else.
crashed_obstacle(S, ObstacleId) :- halted_with(crashed(ObstacleId), S).
obstacle_in_bound_obstacle(S, Threshold, ObstacleId) :- halted_with(obstacle_in_bound(Threshold,ObstacleId), S).
obstacle_on_path_obstacle(S, Threshold, ObstacleId) :- halted_with(obstacle_on_path(Threshold,ObstacleId), S).
battery_under_threshold(S, Threshold) :- halted_with(battery_under(Threshold), S).
battery_equal_threshold(S, Threshold) :- halted_with(battery_equal(Threshold), S).
battery_over_threshold(S, Threshold) :- halted_with(battery_over(Threshold), S).

% -- overall collision probability (exact) --------------------------
any_collision :- final_situation(S), crashed_in(S).

% -- overall battery-depletion probability (exact) -------------------
any_battery_depletion :- final_situation(S), battery_depleted_in(S).

% -- sample_index_for_time(+T,+T0,+Duration,-I): bucket an exact time
%    into the nearest reporting sample, for continuity with plotting.
sample_index_for_time(T,T0,Duration,I) :-
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
%    that question, e.g. crashed_obstacle(S,ObstacleId) alongside
%    final_situation(S) for a specific resolved situation.
first_hit(I) :-
    final_situation(S),
    S = do(haltMoveto(Tcross,crashed(_ObstacleId),_), _),
    current_walk(S, CP, _Triggers, T0, _SPrev),
    walk_duration(CP, Duration),
    sample_index_for_time(Tcross,T0,Duration,I).

% -- Feature 2a analogue: P(the exact collision, if any, falls at or
%    before reporting sample N) ----------------------------------
hit_by(N) :-
    final_situation(S),
    S = do(haltMoveto(Tcross,crashed(_ObstacleId),_), _),
    current_walk(S, CP, _Triggers, T0, _SPrev),
    walk_duration(CP, Duration),
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
verify_safe :-
    final_situation(S),
    current_walk(S, CP, Triggers, T0, SPrev),
    walk_duration(CP, Duration),
    battery(B0, T0, SPrev),
    earliest_halt(CP,Triggers,T0,Duration,0.0,0.0,0.0,B0, completed,_).

plan_route_blocked :- \+ verify_safe.

% -- on-track: does the noisy position stay within tolerance of ----
%    the nominal spline position at sample I? (still sampled -- this
%    is a REPORTING diagnostic about drift magnitude, not a safety
%    detector, so fixed-resolution sampling remains appropriate here)
%    tolerance/1 is now a config fact -- see
%    config/config.yaml's tolerances.on_track.

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

% -- goal reached: arrives near the goal AND the walk's actual
%    recorded outcome was `completed` (read via halted_with/2, not an
%    ad-hoc list of \+ crashed_in/\+ battery_depleted_in-style
%    exclusions -- that style would need a new exclusion added by
%    hand for every future trigger; checking halted_with(completed,S)
%    directly is correct for any number of current or future causes
%    with no changes needed here).
%    goal_tolerance/1 is now a config fact -- see
%    config/config.yaml's tolerances.goal.
%    TODO / KNOWN GAP: reads the global goal/2 fact, same caveat as
%    at_goal/1 above (see the TODO note on that clause) -- not any
%    PlanAstar/PlanStraight node's own `goal` port.

goal_reached :-
    final_situation(S),
    halted_with(completed, S),
    plan_time_span(S, _, TEnd),
    at(GX,GY,TEnd,S),
    goal(GXt,GYt),
    dist(GX,GY,GXt,GYt,D),
    goal_tolerance(Tol),
    D =< Tol.

% ============================================================
% 9. THE POLICY -- an explicit start/end ACTION PAIR:
%        startMoveto(ControlPoints, T0)  ...  haltMoveto(T,Reason)
%    ControlPoints is now computed at runtime by a planWith leaf (see
%    plan/1's own note further down) rather than hand-filled here, but
%    a hand-authored control_points/1 fact (from
%    plan_generation/plan/current_plan.pl) still has to start at
%    start/2 and end at goal/2, with length 3k+1 for some k >= 1 (k
%    cubic Bezier segments), for anything that still uses it directly
%    (e.g. plotting -- see run_plan_continuous_safety.py).
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
%        plan(seq(startMoveto(CP,0), seq(interrupt(5.0), nil))) :-
%            control_points(CP).
%
%    which ends the walk at T=5.0 regardless of the walk's own
%    natural completion or collision time -- poss(interrupt(T),S)
%    requires moving(S) and T strictly before whichever of those two
%    happens first in the resolved world, so this composes with any
%    future action that also wants to interrupt a walk: just give it
%    its own poss/2 clause requiring moving(S) plus its own trigger,
%    same pattern as interrupt/1 above.
% ============================================================

% start/2, goal/2, and control_points/1 are produced by
% occupancy_grid_planner.py (A* over the map -> B-spline fit -> exact
% cubic-Bezier-chain extraction) and consulted directly here -- no
% hand-typed numbers, no separate parsing step. Path is relative to
% wherever ProbLog is invoked FROM (typically the directory containing
% this file); see occupancy_grid_planner.py's own header for its output
% locations. current_plan.pl is ALWAYS the most recently generated plan
% (overwritten on every run); per-map dated copies are also kept
% alongside it under the same folder if you need to go back to an
% earlier one.
:- consult('./plan_generation/plan/current_plan.pl').
% Must define exactly: start(X,Y). goal(X,Y). control_points([point(...),...]).
% control_points/1 MUST have length 3k+1 for some k>=1 (k cubic Bezier
% segments) and must start at start/2's point and end at goal/2's point
% -- occupancy_grid_planner.py's output always satisfies this by
% construction, so this is only a concern if you hand-author the file.

% plan/1 is no longer hand-written here -- it comes from
% plan_generation/plan/plan_generated.pl, generated by
% plan_generation/bt_to_prolog.py translating the REAL BT.cpp v4 XML
% tree at plan_generation/plan/behavior_tree.xml against
% actions/schema.yaml (run_plan_continuous_safety.py does this
% automatically before every run, exactly like it already does for
% config/config_generated.pl). This is now the single source of truth
% for the POLICY'S SHAPE: to change the tree, edit behavior_tree.xml
% and re-run, don't hand-edit a plan/1 clause here.
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
% policies -- and conditions like AtGoal/HaltedWith/ObstacleInBound/
% BatteryBelow -- are already expressible in the XML with no further
% changes here; see
% bt_to_prolog.py's own header for the blackboard-to-Prolog-variable
% translation this relies on (e.g. giving two different PlanAstar/
% PlanStraight nodes distinct blackboard keys, same as the CP1/CP2
% convention this file already documents for hand-written multi-leg
% plans). Only the AUTOMATIC generation of multi-leg XML trees (from
% occupancy_grid_planner.py, which still emits one spline) and reacting
% to a planWith/moveto_leg FAILURE by re-planning are future work; the
% theory and the translator both already support authoring either by
% hand in the XML.
:- consult('./plan_generation/plan/plan_generated.pl').

% ============================================================
% 10. QUERIES
% ============================================================
query(goal_reached).
query(plan_outcome(true)).
query(plan_outcome(false)).
query(any_collision).
query(any_battery_depletion).
query(verify_safe).
query(plan_route_blocked).
query(hit_by(20)).

query(first_hit(0)).
query(first_hit(1)).
query(first_hit(2)).
query(first_hit(3)).
query(first_hit(4)).
query(first_hit(5)).
query(first_hit(6)).
query(first_hit(7)).
query(first_hit(8)).
query(first_hit(9)).
query(first_hit(10)).
query(first_hit(11)).
query(first_hit(12)).
query(first_hit(13)).
query(first_hit(14)).
query(first_hit(15)).
query(first_hit(16)).
query(first_hit(17)).
query(first_hit(18)).
query(first_hit(19)).
query(first_hit(20)).

query(on_track(0)).
query(on_track(1)).
query(on_track(2)).
query(on_track(3)).
query(on_track(4)).
query(on_track(5)).
query(on_track(6)).
query(on_track(7)).
query(on_track(8)).
query(on_track(9)).
query(on_track(10)).
query(on_track(11)).
query(on_track(12)).
query(on_track(13)).
query(on_track(14)).
query(on_track(15)).
query(on_track(16)).
query(on_track(17)).
query(on_track(18)).
query(on_track(19)).
query(on_track(20)).