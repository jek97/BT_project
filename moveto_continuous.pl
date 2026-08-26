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
%     Triggers list. Different occurrences of moveto in a policy can
%     react to different conditions -- see default_triggers/1 near
%     plan/1 for the shipped default's actual choice (protected, via
%     [collision,battery]). Whichever entry in Triggers occurs
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

% actions/moveto_planners.py provides plan_astar/5 and plan_straight/5
% as BLACK-BOX (Python-implemented) predicates -- see that file's own
% header for the full explanation. ProbLog imports and executes it
% directly the moment this directive loads (problog.clausedb's
% load_external_module), registering both predicates before anything
% below that calls them is ever evaluated. Path is resolved relative to
% THIS file's own directory (not CWD) -- moveto_planners.py lives in
% ./actions/ alongside bt_actions.py and schema.yaml (the BT.cpp-facing
% side of the same node set).
:- use_module('./actions/moveto_planners.py').

% collision_geometry.py provides first_threshold_crossing_time/6 as a
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
%    TRIGGERS section further down (first_threshold_crossing_time/6)
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
robot_radius(0.20).
safety_buffer(0.10).
safety_margin(M) :- robot_radius(R), safety_buffer(B), M is R + B.

% sight_threshold: a SECOND, separate distance threshold for the
% obstacle_sighted trigger (see the TRIGGERS section) -- deliberately
% kept as its own independently-tunable fact, NOT derived from
% safety_margin, but REQUIRED to be strictly larger than it (a
% sighting that isn't detected before collision range is useless as
% an early-warning trigger).
%
% Enforcement: sight_threshold_valid below is the check, exposed as
% an ORDINARY QUERY (see the query list at the end of the file) --
% tested and confirmed this is the actual, SOLE enforcement
% mechanism. A bare `:- sight_threshold_valid.` load-time directive
% was also tried, but ProbLog does NOT hard-stop or warn when a plain
% directive fails to prove -- it silently continues loading -- so a
% failed directive here would give no visible signal at all. The
% query is reliable (shows 0 in every run's output if misconfigured);
% the directive was removed since it added complexity with no real
% enforcement benefit. If you want a genuine load-time hard-stop,
% check sight_threshold_valid manually before running problog (e.g.
% from run_plan_continuous_safety.py) rather than relying on ProbLog
% itself to refuse to load.
sight_threshold(0.6).

sight_threshold_valid :- sight_threshold(ST), safety_margin(M), ST > M.

% within_obstacle_threshold/3 (the generalized "is (PX,PY) within
% Threshold of the nearest obstacle" test, parametrized by threshold so
% the SAME primitive serves both collision and obstacle_sighted) now
% lives inside collision_geometry.py's within_obstacle_threshold
% helper, alongside the rest of the obstacle-clearance geometry it was
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

speed(1.0).  % metres per time unit

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
% NOTE: these five weights sum to EXACTLY 1.0 (0.0606*2 + 0.2417*2 +
% 0.3954 = 1.0000). An earlier version of this table used 0.3854 for
% the centre weight, which summed to only 0.99 -- ProbLog silently
% treats missing mass as an implicit "none of these" failure branch,
% which caps EVERY downstream probability at 0.99 in every world. Kept
% as a cautionary note: always check your annotated-disjunction weights
% sum to 1.0 exactly.
0.0606::z(do(startMoveto(CP,Triggers,T0),S), -2.0) ;
0.2417::z(do(startMoveto(CP,Triggers,T0),S), -1.0) ;
0.3954::z(do(startMoveto(CP,Triggers,T0),S),  0.0) ;
0.2417::z(do(startMoveto(CP,Triggers,T0),S),  1.0) ;
0.0606::z(do(startMoveto(CP,Triggers,T0),S),  2.0).

sigma(0.15).  % lateral noise scale, metres per sqrt(time-unit)

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
battery_start(100).
idle_drain_rate(0.05).    % %/time-unit while not moving -- tune as needed
moving_drain_rate(0.5).   % %/time-unit while moving -- tune as needed
sigma_battery(0.5).       % noise scale, % per sqrt(time-unit) -- SAME
                          % value used in every phase (moving, idle,
                          % and s0) -- see zbatt/1 below.

% zbatt/1: ONE noise draw for the WHOLE MISSION -- deliberately
% decoupled from any specific startMoveto occurrence. This is an
% annotated disjunction with NO ARGUMENTS at all, which ProbLog
% grounds EXACTLY ONCE for the entire program: every reference to
% zbatt(Zb), anywhere in the theory, in every world, refers to the
% SAME single resolved value. This treats "how much this particular
% battery underperforms" as a persistent property of the battery
% itself -- present from the very start, not a fresh, independent
% draw manufactured at each walk. Same corrected 5-point discretized
% N(0,1) table as z/2 (see the note above z/2 about weights needing
% to sum to exactly 1.0).
0.0606::zbatt(-2.0) ;
0.2417::zbatt(-1.0) ;
0.3954::zbatt( 0.0) ;
0.2417::zbatt( 1.0) ;
0.0606::zbatt( 2.0).

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
% part of the theory already keys on (crashed, battery_depleted --
% via halted_with/2, crashed_in/1, battery_depleted_in/1,
% first_hit/1, hit_by/1, etc.) stay EXACTLY as they were; only how
% those Reasons get triggered changed, not what they're called.
% ---------------------------------------------------------------
trigger_crossing_time(collision, CP,T0,Duration,Z,_Zb,_B0, crashed, Tcross) :-
    first_collision_time(CP,T0,Duration,Z,Tcross).

trigger_crossing_time(battery, CP,T0,Duration,_Z,Zb,B0, battery_depleted, Tcross) :-
    first_battery_depletion_time(CP,T0,Duration,B0,Zb,Tcross).

trigger_crossing_time(obstacle_sighted, CP,T0,Duration,Z,_Zb,_B0, obstacle_sighted, Tcross) :-
    first_obstacle_sighted_time(CP,T0,Duration,Z,Tcross).

% all_trigger_candidates(+Triggers,...,-Candidates): Candidates is a
% list of Reason-Time pairs, one per trigger in Triggers that ACTUALLY
% fires in this resolved world (triggers that don't fire contribute
% nothing -- same "absence, not sentinel" convention as everywhere
% else). Unrecognized trigger names (no matching
% trigger_crossing_time/9 clause) are silently skipped, same as "never
% fires" -- lenient by design, so a typo in a Triggers list doesn't
% halt the whole theory, just means that trigger never contributes.
all_trigger_candidates([], _,_,_,_,_,_, []).
all_trigger_candidates([Trig|Rest], CP,T0,Duration,Z,Zb,B0, [Reason-Tcross|RestCands]) :-
    trigger_crossing_time(Trig, CP,T0,Duration,Z,Zb,B0, Reason, Tcross),
    all_trigger_candidates(Rest, CP,T0,Duration,Z,Zb,B0, RestCands).
all_trigger_candidates([Trig|Rest], CP,T0,Duration,Z,Zb,B0, RestCands) :-
    \+ trigger_crossing_time(Trig, CP,T0,Duration,Z,Zb,B0, _, _),
    all_trigger_candidates(Rest, CP,T0,Duration,Z,Zb,B0, RestCands).

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

% walk_noisy_point(+CP,+T0,+Duration,+Z,+T,-X,-Y): position along the
% spline at time T, given an ALREADY-RESOLVED noise draw Z (rather
% than looking Z up via z/2 itself). Factored out of at/4 so the same
% formula can be reused by first_collision_time's bracket/bisection
% search below, without re-deriving Z through a different situation.
walk_noisy_point(ControlPoints, T0, Duration, Z, T, X, Y) :-
    Elapsed0 is T - T0,
    Elapsed is max(0.0, min(Elapsed0, Duration)),
    Frac is Elapsed / Duration,
    spline_point(ControlPoints, Frac, NX, NY),
    spline_tangent(ControlPoints, Frac, DX, DY),
    Norm is sqrt(DX*DX + DY*DY),
    perp_unit(Norm, DX, DY, PerpX, PerpY),
    sigma(Sigma),
    Deviation is Z * Sigma * sqrt(Duration) * Frac,
    X is NX + Deviation*PerpX,
    Y is NY + Deviation*PerpY.

% ---------------------------------------------------------------
% FIRST-THRESHOLD-CROSSING-TIME -- a NATURAL (not chosen) event: the
% earliest time, within a given resolved world (fixed Z), at which
% the noisy trajectory comes within a given distance THRESHOLD of an
% obstacle. GENERALIZED over the threshold (rather than hardcoded to
% collision's safety_margin) so the SAME machinery serves both
% collision (threshold=safety_margin) and obstacle_sighted
% (threshold=sight_threshold), and any future distance-based trigger.
%
% first_threshold_crossing_time(+ControlPoints,+T0,+Duration,+Z,
% +Threshold,-Tcross) is now a BLACK-BOX Python predicate, registered
% by collision_geometry.py's own :- use_module(...) directive (see
% below) -- exactly the same "deliberately NOT part of the situation-
% calculus machinery" reasoning already used for planWith/plan_call:
% this is a deterministic, stateless computation over an ALREADY-
% RESOLVED noise value Z, not a probabilistic choice in itself, so
% there is no frame problem here to justify keeping it in Prolog.
% bracket_samples(60) and crossing_eps(0.01) -- the bracket-scan count
% and bisection tolerance -- are now READ DIRECTLY out of this .pl
% file's own text by collision_geometry.py at import time (regex, the
% same lightweight technique run_plan_continuous_safety.py already uses
% to read back ground facts) rather than duplicated as separate Python
% constants, so tuning either value here take effect automatically with
% no risk of the two implementations drifting apart. FAILS (0 ProbLog
% solutions) if the trajectory never comes within Threshold of an
% obstacle in this resolved world -- correctly representing "never
% happens" via absence, not a sentinel value, same convention as
% everywhere else in this theory.
bracket_samples(60).
crossing_eps(0.01).

% first_collision_time/5 kept as a thin, name-preserving wrapper over
% the generalized machinery, at threshold=safety_margin -- every
% EXISTING caller (crashed_in, verify_safe, etc.) is unaffected.
first_collision_time(CP,T0,Duration,Z,Tcross) :-
    safety_margin(M),
    first_threshold_crossing_time(CP,T0,Duration,Z,M,Tcross).

% first_obstacle_sighted_time/5: the SAME machinery, at the (larger)
% sight_threshold -- the crossing-time computation for the
% obstacle_sighted trigger.
first_obstacle_sighted_time(CP,T0,Duration,Z,Tcross) :-
    sight_threshold(ST),
    first_threshold_crossing_time(CP,T0,Duration,Z,ST,Tcross).


% ---------------------------------------------------------------
% Poss AXIOMS for the primitive actions.
% ---------------------------------------------------------------
% NOTE: startMoveto deliberately does NOT check battery > 0 (or
% "not already colliding", or "not already within sight_threshold")
% as a precondition. Doing so would make do_action FAIL ENTIRELY when
% the battery is already empty (or the robot already unsafe) --
% classical Golog non-derivability, i.e. "no situation exists" -- the
% wrong semantics for a BT-style outcome (see the do_node/outcome
% discussion earlier). Instead, ALL of collision, battery depletion,
% and obstacle_sighted already have a graceful "already true at T0"
% case built into first_threshold_crossing_time / first_battery_
% depletion_time (Tcross = T0 exactly), so starting a walk with an
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
% or any condition in this leg's own Triggers list (collision,
% battery, obstacle_sighted, or a future one) -- occurs EARLIEST in
% this resolved world wins. This is the genuine TEMPLATE mechanism:
% NOTHING is hardcoded here -- a leg with Triggers=[] halts ONLY on
% natural completion, passing straight through an obstacle's margin
% or running the battery dry without ever noticing, if collision/
% battery aren't in its own Triggers list.
%
% earliest_halt/9 is the SINGLE SHARED definition of "what happens
% first" -- used here, by Poss(interrupt(...)) below, AND by
% verify_safe further down (called there with Z=0.0,Zb=0.0 instead of
% the resolved noise). Having exactly ONE definition, rather than the
% same computation duplicated at each call site, is what guarantees
% every query stays consistent with what Poss(haltMoveto(...)) itself
% actually derives -- see the "SAFETY QUERIES READ THE ACTUAL OUTCOME"
% note further down for why this matters.
earliest_halt(CP,Triggers,T0,Duration,Z,Zb,B0, Reason,T) :-
    all_trigger_candidates(Triggers, CP,T0,Duration,Z,Zb,B0, ExtraCandidates),
    NaturalEnd is T0 + Duration,
    earliest_of([completed-NaturalEnd], ExtraCandidates, Reason-T).

poss(haltMoveto(T, Reason, Status), S) :-
    moving(S),
    current_walk(S, CP, Triggers, T0, SPrev),
    walk_duration(CP, Duration),
    z(do(startMoveto(CP,Triggers,T0),SPrev), Z),
    zbatt(Zb),
    battery(B0, T0, SPrev),
    earliest_halt(CP,Triggers,T0,Duration,Z,Zb,B0, Reason,T),
    leg_status(Reason, CP, T0, Duration, Z, T, Status).

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

% leg_status(+Reason,+CP,+T0,+Duration,+Z,+T,-Status): the NEW OUTPUT
% -- Status is TRUE iff Reason=completed AND the actual (noisy) final
% position lands within goal_tolerance of the leg's OWN endpoint.
% Deliberately DISTINCT from Reason: Reason=completed only means the
% walk wasn't cut short (by collision/battery/a trigger) before its
% nominal duration elapsed -- it says nothing about whether noise
% carried the robot far enough off course to miss the target despite
% "completing". Status is what a Sequence/Fallback composite (below)
% actually branches on.
leg_status(completed, CP, T0, Duration, Z, T, true) :-
    walk_noisy_point(CP, T0, Duration, Z, T, X, Y),
    leg_target(CP, GX, GY),
    dist(X, Y, GX, GY, D),
    goal_tolerance(Tol),
    D =< Tol.
leg_status(completed, CP, T0, Duration, Z, T, false) :-
    walk_noisy_point(CP, T0, Duration, Z, T, X, Y),
    leg_target(CP, GX, GY),
    dist(X, Y, GX, GY, D),
    goal_tolerance(Tol),
    D > Tol.
leg_status(Reason, _,_,_,_,_, false) :- Reason \= completed.

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
% the SAME earliest_halt/9 as Poss(haltMoveto(...)) above, so this
% bound can never drift out of sync with what haltMoveto itself would
% derive.
poss(interrupt(T), S) :-
    moving(S),
    current_walk(S, CP, Triggers, T0, SPrev),
    walk_duration(CP, Duration),
    z(do(startMoveto(CP,Triggers,T0),SPrev), Z),
    zbatt(Zb),
    battery(B0, T0, SPrev),
    earliest_halt(CP,Triggers,T0,Duration,Z,Zb,B0, _Reason,Tend),
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
    walk_noisy_point(ControlPoints, T0, Duration, Z, T, X, Y).

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
%     moveto_leg(CP)            -- ACTION leaf, Triggers defaulting to
%                                  default_triggers/1 (sugar).
%     moveto_leg(CP,Triggers)   -- ACTION leaf, explicit Triggers.
%                                  Runs one startMoveto/haltMoveto
%                                  pair to its halt; T0 auto-derived
%                                  via now/2. Outcome IS the Status
%                                  output, unchanged.
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
% moveto_leg(CP) is sugar defaulting to default_triggers/1 (see the
% policy section further down) -- NOT to [] -- so the convenience form
% stays protected the same way the shipped default plan is. Use
% moveto_leg(CP,Triggers) explicitly for anything else, including
% Triggers=[] for a genuinely unprotected leg. Status flows straight
% through as Outcome -- no translation predicate needed, since both
% already speak true/false.
do_node(moveto_leg(CP), S, S1, Status) :-
    default_triggers(Triggers),
    do_action(startMoveto(CP,Triggers,_T0), S, S2),
    do_action(haltMoveto(_T,_Reason,Status), S2, S1).
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
holds(halted_with_cond(Reason), S) :- halted_with(Reason, S).

% at_goal(Tol): true iff the CURRENT position (at the current time,
% via now/2) is within Tol of goal/2. Typical use: a fallback child
% that skips moveto entirely if already there --
%   fallback_node([cond(at_goal(0.3)), moveto_leg(CP)])
holds(at_goal(Tol), S) :-
    now(S, T), at(X,Y,T,S), goal(GX,GY), dist(X,Y,GX,GY,D), D =< Tol.

% ---------------------------------------------------------------
% 8. VERIFICATION-TIME SAMPLING (NOT part of the action theory) --
%    a purely deterministic choice of how finely to CHECK/REPORT the
%    already-closed-form at/4 fluent for VISUALIZATION purposes.
%    NOTE: collision DETECTION itself is now EXACT (see
%    first_collision_time above) -- num_samples/1 only controls the
%    resolution used for plotting and for on_track's drift reporting,
%    it no longer determines whether a collision is found at all.
% ---------------------------------------------------------------
num_samples(20).

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
%    regardless of whether some OTHER cause (e.g. obstacle_sighted)
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

% -- crashed_in(S) / battery_depleted_in(S): now trivial one-liners --
%    reading the actual Reason, not re-deriving anything. Any FUTURE
%    trigger's own "did it actually fire" diagnostic is exactly this
%    same one-liner pattern, e.g.
%      obstacle_sighted_in(S) :- halted_with(obstacle_sighted, S).
%    -- no trigger-specific re-derivation logic to get wrong.
crashed_in(S) :- halted_with(crashed, S).
battery_depleted_in(S) :- halted_with(battery_depleted, S).

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
%    match Reason=crashed, and this correctly contributes nothing.
first_hit(I) :-
    final_situation(S),
    S = do(haltMoveto(Tcross,crashed,_), _),
    current_walk(S, CP, _Triggers, T0, _SPrev),
    walk_duration(CP, Duration),
    sample_index_for_time(Tcross,T0,Duration,I).

% -- Feature 2a analogue: P(the exact collision, if any, falls at or
%    before reporting sample N) ----------------------------------
hit_by(N) :-
    final_situation(S),
    S = do(haltMoveto(Tcross,crashed,_), _),
    current_walk(S, CP, _Triggers, T0, _SPrev),
    walk_duration(CP, Duration),
    sample_index_for_time(Tcross,T0,Duration,I),
    I =< N.

% ---------------------------------------------------------------
% FUTURE EXTENSION NOTE: "safety" here is meant to cover EVERY cause
% that could prevent the robot from reaching the goal. Currently
% there are three, ALL expressed as ordinary Triggers entries (see
% the TRIGGERS section and trigger_crossing_time/9 far above):
% collision (first_collision_time / crashed_in / any_collision),
% battery depletion (first_battery_depletion_time /
% battery_depleted_in / any_battery_depletion), and obstacle_sighted.
% Adding a FOURTH cause later -- a mechanical fault, a comms timeout,
% whatever -- means exactly:
%   (a) one more trigger_crossing_time/9 clause, giving its own
%       Reason and crossing-time computation
%   (b) a dedicated *_in(S) exact-detection predicate (one line, via
%       halted_with/2) + a corresponding any_* diagnostic query, if
%       you want it separately reportable
%   (c) including the new trigger's name in whichever Triggers
%       list(s) should react to it -- e.g. default_triggers/1, or a
%       specific leg's own list
% Nothing else in the theory needs to change; earliest_halt/9 and
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
    earliest_halt(CP,Triggers,T0,Duration,0.0,0.0,B0, completed,_).

plan_route_blocked :- \+ verify_safe.

% -- on-track: does the noisy position stay within tolerance of ----
%    the nominal spline position at sample I? (still sampled -- this
%    is a REPORTING diagnostic about drift magnitude, not a safety
%    detector, so fixed-resolution sampling remains appropriate here)
tolerance(0.5).

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
goal_tolerance(0.3).

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
%    Fill in control_points/1 by hand once you have the spline
%    (e.g. from a path planner or hand-authored waypoints). It
%    MUST start at start/2 and end at goal/2, and must have
%    length 3k+1 for some k >= 1 (k cubic Bezier segments).
%
%    T and Reason are left as FREE VARIABLES -- they are DERIVED by
%    Poss(haltMoveto(T,Reason),S), never chosen by the plan: Reason
%    comes out `completed` if the walk finishes without ever coming
%    within the safety margin of an obstacle in that resolved world,
%    or `crashed` (with T = the exact collision time) otherwise.
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

% default_triggers/1 is [collision, battery] here -- NOT [] -- so the
% SHIPPED default plan keeps its existing protected behavior (halts on
% collision or battery depletion, as every example throughout this
% file's development assumed). This is now a PLAIN CHOICE of which
% conditions to react to, exactly like any other entry in a Triggers
% list -- nothing about collision/battery is hardcoded into the
% action theory anymore (see the TRIGGERS section above
% trigger_crossing_time/9). To get a genuinely UNPROTECTED leg that
% completes its full nominal duration even through an obstacle's
% margin or an empty battery, use moveto_leg(CP,[]) explicitly, or
% override default_triggers/1 to [].
%
% To ALSO reactively halt when an obstacle first comes within
% sight_threshold, add it alongside the others:
%   plan(seq_node([moveto_leg(CP,[collision,battery,obstacle_sighted])])) :-
%       control_points(CP).
% Since plan/1 is itself just a do_node/4 Node term, arbitrary
% sequence/fallback/multi-leg policies are already expressible with
% today's machinery, e.g.:
%   plan(fallback_node([cond(at_goal(0.3)), moveto_leg(CP)])) :- control_points(CP).
%   plan(seq_node([moveto_leg(CP1), moveto_leg(CP2)])) :- CP1=..., CP2=....
% -- only the AUTOMATIC generation of multi-leg plans (from
% occupancy_grid_planner.py, which still emits one spline) is future
% work; the theory itself already supports it.
%
% "Plan a path with A*, then go there" -- using the planWith leaf
% (see actions/moveto_planners.py) instead of a hand/offline-computed
% control_points/1 fact -- is expressed exactly the same way, with no
% new machinery beyond what's already in this file:
%   plan(seq_node([planWith(astar,point(17.0,17.0),CP), moveto_leg(CP,[collision,battery])])).
% This computes ControlPoints FROM THE ACTUAL CURRENT POSITION (via
% now/2 + at/4) at the moment planWith runs, rather than baking in a
% path computed offline beforehand.
%
% Because Goal is now explicit per planWith call (not the global
% goal/2 fact), "go to P1, then go to P2" -- two genuinely different
% destinations in one plan, the direct analogue of instantiating a
% parametrized GoTo(target) BT.cpp subtree twice with two different
% port bindings -- is just two legs with two different point(...)
% arguments:
%   plan(seq_node([planWith(astar,point(P1x,P1y),CP1), moveto_leg(CP1,[collision,battery]),
%                  planWith(astar,point(P2x,P2y),CP2), moveto_leg(CP2,[collision,battery])])).
%
% NOT YET IMPLEMENTED: reacting to planWith or the subsequent
% moveto_leg FAILING by re-planning (e.g.
% fallback_node([seq_node([planWith(astar,point(GX,GY),CP1),moveto_leg(CP1,...)]),
%                seq_node([planWith(straight,point(GX,GY),CP2),moveto_leg(CP2,...)])])
% as a first, coarse version of "try A*, fall back to a straight line
% if that doesn't work") -- flagged here as a natural next step, not
% built yet.
default_triggers([collision, battery]).

plan(seq_node([moveto_leg(CP,Triggers)])) :-
    control_points(CP),
    default_triggers(Triggers).

% ============================================================
% 10. QUERIES
% ============================================================
query(goal_reached).
query(plan_outcome(true)).
query(plan_outcome(false)).
query(any_collision).
query(any_battery_depletion).
query(sight_threshold_valid).
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