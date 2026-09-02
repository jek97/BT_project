% ============================================================
% ProbLog -- ATOMIC moveto() action theory (simplified branch)
%
% Rewrite of the durative, continuous-time/continuous-space theory
% (see git history on branch claude/project-review-179rmx for the
% pre-simplification version): every moveto is now a SINGLE atomic
% action whose full outcome distribution -- landing position, elapsed
% duration, battery drained, and the reason it stopped (success,
% crashed(Obstacle), battery_depleted, or a reactive trigger name) --
% is computed OFFLINE, once per (leg, incoming-branch) pair, by a
% Python calibration script that has access to the map/obstacle
% geometry this theory itself no longer needs to see. See
% FUTUREWORK.md and this file's own git history for the diagnostic
% trail that motivated this rewrite: the previous continuous-noise
% model (per-leg z/zt/zbatt draws, resolved via a live bracket-scan +
% bisection search inside ProbLog) made exact inference blow up
% combinatorially once more than ~2 stochastic axes were
% simultaneously active across more than one reactive redescend.
%
% The reactive-redescend ARCHITECTURE IS UNCHANGED: do_node still
% reports true/false/reactive for every node, seq_node/fallback_node
% still propagate `reactive` straight through untouched, and
% evaluate_plan/4 still re-descends the WHOLE tree from wherever a
% reactive outcome left the robot, exactly as before -- see Sections
% 6-7 below, copied verbatim from the pre-simplification theory. Only
% how a MOVETO LEAF decides its own outcome has changed: from a live,
% noise-resolved geometric simulation to a single lookup into a
% pre-computed table.
% ============================================================

% ---------------------------------------------------------------
% 0. PROBLEM-SPECIFIC DATA. Consulted via problem_data.pl, the same
%    auto-generated bootstrap main.py rewrites on every run (see that
%    file's own header). It still points at obstacles_generated.pl and
%    config_generated.pl for compatibility with the offline
%    calibration pipeline (which DOES need the map and the tunable
%    constants), but THIS file itself only actually consumes:
%      - battery_start/1              (config_generated.pl)
%      - plan/1                       (plan_generated.pl)
%      - goal_formula/1               (goal_formula.pl)
%      - moveto_outcome/7             (moveto_outcomes_generated.pl --
%                                       the calibrated per-leg outcome
%                                       tables; the offline Python
%                                       calibrator that produces this
%                                       file is NOT YET BUILT -- see
%                                       FUTUREWORK.md -- so every
%                                       problem using this theory
%                                       currently needs one hand-
%                                       written as a stub, the same
%                                       "translator gap, hand-written"
%                                       precedent problem3's own
%                                       plan_generated.pl already set.)
%    obstacle_polygon/2 and the noise/geometry config facts (sigma/1,
%    z/2, zt/2, zbatt/1, robot_radius/1, ...) get loaded too, harmlessly
%    unused, for backward compatibility with problem_data.pl's existing
%    four-file bootstrap; they play no part in this theory's own
%    reasoning any more.
% ---------------------------------------------------------------
:- consult('./problem_data.pl').

% ---------------------------------------------------------------
% 1. GEOMETRY HELPER -- kept for any query/diagnostic that still wants
%    plain Euclidean distance; none of this theory's own predicates
%    need it any more.
% ---------------------------------------------------------------
dist(X1,Y1,X2,Y2,D) :- D is sqrt((X2-X1)**2 + (Y2-Y1)**2).

% ---------------------------------------------------------------
% 2. THE moveto OUTCOME TABLE -- calibrated OFFLINE, once per (leg,
%    incoming-branch) pair, by a Python script with access to the map
%    and obstacle geometry this theory no longer sees directly (see
%    Section 0's own note). Declared here as the interface
%    moveto_outcomes_generated.pl (or a hand-written stub) must supply:
%
%      moveto_outcome(LegId, InBranch, BranchId, Reason, EndPoint, Duration, Drain)
%
%    LegId    -- static id for this leaf, taken from the BT node's own
%                name= attribute (e.g. leg_try_goal).
%    InBranch -- which upstream branch this variant was calibrated
%                for: `root` if this leg's own start position is fixed
%                (the plan's own initial position, or a branch whose
%                predecessor's landing position doesn't matter), or
%                the BranchId some earlier moveto resolved to, if this
%                leg's own outcome distribution genuinely depends on
%                where that left the robot -- see incoming_branch/2
%                below for how this gets looked up at run time.
%                Sequence-chained AND Fallback-redescended legs both
%                go through the exact same mechanism, no special-
%                casing needed for either.
%    BranchId -- this specific row's own id (so a LATER leg, if it
%                depends on where THIS one lands, can key off it in
%                turn).
%    Reason   -- success, crashed(ObstacleId), battery_depleted, or
%                any reactive trigger name (battery_below(Threshold),
%                obstacle_in_bound(Threshold,ObstacleId),
%                obstacle_on_path(Threshold,ObstacleId),
%                battery_equal(Threshold), battery_over(Threshold),
%                line_of_sight_clear(ObstacleId,GX,GY), or any future
%                trigger name not listed here -- outcome_status/2
%                below treats every Reason it doesn't recognize as a
%                hard success/failure as reactive, so a new trigger
%                type needs no change to this theory, only a new
%                Reason value appearing in a generated table).
%    EndPoint -- point(X,Y) this branch actually lands at, regardless
%                of Reason (a crashed/battery_depleted/reactive branch
%                still lands SOMEWHERE, and at/3 below reports it).
%    Duration -- elapsed time this action instance took.
%    Drain    -- battery percentage consumed by this action instance.
%
%    Every row's probabilities, for a fixed (LegId,InBranch) pair,
%    MUST sum to exactly 1.0 -- same convention, and same silent-
%    truncation risk if violated, as the old z/2 noise tables (see
%    config.yaml's own note on this).
% ---------------------------------------------------------------

% incoming_branch(+S, -InKey): which upstream branch variant this
% leg's own calibration table should be looked up under, given the
% situation S it's being attempted from. `root` if S's outermost
% action isn't a moveto_result at all (first leg of a branch whose own
% start is the plan's fixed initial position). Works identically
% whether S got here by plain Sequence-chaining or by evaluate_plan/4's
% reactive redescend landing on a DIFFERENT branch entirely -- both
% cases are just "look at S's own outermost layer", same principle
% last_halt/1 (Section 8) relies on.
incoming_branch(S, root) :- \+ (S = do(moveto_result(_,_,_,_,_,_), _)).
incoming_branch(do(moveto_result(_,B,_,_,_,_), _), B).

% outcome_status(+Reason, -Status): the ONLY place a Reason value gets
% mapped to the true/false/reactive vocabulary do_node/seq_node/
% fallback_node/evaluate_plan all share (Section 6-7). Everything not
% explicitly a hard success/failure is reactive BY DEFAULT -- a future
% trigger name needs no change here, only a moveto_outcome row using
% it.
outcome_status(success,          true).
outcome_status(crashed(_),       false).
outcome_status(battery_depleted, false).
outcome_status(Reason, reactive) :-
    Reason \= success,
    Reason \= crashed(_),
    Reason \= battery_depleted.

% ---------------------------------------------------------------
% 3. THE moveto ACTION -- ONE atomic leaf, replacing the old
%    planWith(Algorithm,Goal,CP) + moveto_leg(CP,Triggers) pair.
%    Algorithm is now just an argument (straight/astar/...), consumed
%    entirely by the offline calibrator when it built this leg's own
%    moveto_outcome rows -- this theory never dispatches on it itself.
%
%    COMPLETENESS OF moveto_outcomes_generated.pl is NOT enforced here.
%    An in-theory guard was tried and abandoned: ProbLog's own
%    grounder explores every (LegId,InBranch) combination reachable by
%    unification while building its Boolean formula -- including ones
%    a real resolved world can never take (e.g. "what if leg_try_goal's
%    Fallback branch had resolved to false", even though none of its
%    rows map to false) -- not only the combinations a specific
%    resolved world actually needs. A guard built from negation-as-
%    failure (the only mechanism ProbLog offers for this -- it
%    supports neither if-then-else, throw/1, nor format/2, all
%    confirmed unsupported) cannot tell "genuinely missing and
%    reachable" apart from "structurally explored but always
%    zero-weight", and false-positived on this very theory's own
%    complete, correct problem4 stub. The right place for this check
%    is a Python-side pre-flight validator, mirroring
%    goal_formula_check.py's existing pattern, that walks
%    plan_generated.pl's tree directly (real graph traversal, not
%    ProbLog's grounder) before problog ever runs -- not yet built,
%    see FUTUREWORK.md. Until then, an incomplete table just makes
%    do_node silently fail for the affected world, exactly as any
%    other missing fact would.
% ---------------------------------------------------------------
do_node(moveto(LegId,_Algorithm,_Goal), S,
        do(moveto_result(LegId,BranchId,Reason,EndPoint,Duration,Drain), S), Status) :-
    incoming_branch(S, InKey),
    moveto_outcome(LegId, InKey, BranchId, Reason, EndPoint, Duration, Drain),
    outcome_status(Reason, Status).

% ---------------------------------------------------------------
% 4. FLUENTS -- trivial Reiter-style regressions now: each is a plain
%    accumulator over the (already resolved, offline-computed)
%    Duration/Drain/EndPoint a moveto_result instance carries, not an
%    integral over a live noisy walk. The pass-through clauses are
%    dead code today (moveto is the only situation-extending action in
%    this theory) but kept for the same reason the old theory kept its
%    own equivalents: whichever future action type gets added next
%    (e.g. a wait/1) doesn't silently break these.
% ---------------------------------------------------------------
now(0.0, s0).
now(T, do(moveto_result(_,_,_,_,Duration,_), S)) :-
    now(T0, S), T is T0 + Duration.
now(T, do(A, S)) :-
    A \= moveto_result(_,_,_,_,_,_),
    now(T, S).

battery(Level, s0) :- battery_start(Level).
battery(Level, do(moveto_result(_,_,_,_,_,Drain), S)) :-
    battery(B0, S), Level is max(0, B0 - Drain).
battery(Level, do(A, S)) :-
    A \= moveto_result(_,_,_,_,_,_),
    battery(Level, S).

at(X,Y, s0) :- start(X,Y).
at(X,Y, do(moveto_result(_,_,_,point(X,Y),_,_), _)).
at(X,Y, do(A, S)) :-
    A \= moveto_result(_,_,_,_,_,_),
    at(X,Y, S).

% ---------------------------------------------------------------
% 5. CONDITIONS -- same names/semantics as the old theory's
%    battery_below/battery_equal/battery_over cond() leaves (unified
%    to a single battery_below name -- battery_under is gone, it was
%    only ever an artifact of the old implementation), now reading the
%    trivial battery/2 fluent above instead of a continuous one.
%    after_time(H) is new -- the "after X (hours/time-units) do that"
%    condition now/2 exists specifically to support.
% ---------------------------------------------------------------
holds(battery_below(Threshold), S) :- battery(Level,S), Level < Threshold.
holds(battery_equal(Threshold), S) :- battery(Level,S), Level =:= Threshold.
holds(battery_over(Threshold),  S) :- battery(Level,S), Level > Threshold.
holds(after_time(H), S) :- now(T,S), T >= H.

% ---------------------------------------------------------------
% 6. PRIMITIVE ACTION EXECUTION + BEHAVIOR-TREE INTERFACE.
%    UNCHANGED from the old theory -- do_node(Node,S,S1,Outcome) is
%    the same standard interface, seq_node/fallback_node don't inspect
%    what kind of thing a child is, and neither needed a single edit
%    for this rewrite.
% ---------------------------------------------------------------
do_node(cond(C), S, S, true)  :- holds(C, S).
do_node(cond(C), S, S, false) :- \+ holds(C, S).

do_node(seq_node([]), S, S, true).
do_node(seq_node([Child|Rest]), S, S1, Outcome) :-
    do_node(Child, S, S2, true),
    do_node(seq_node(Rest), S2, S1, Outcome).
do_node(seq_node([Child|_]), S, S1, false) :-
    do_node(Child, S, S1, false).
do_node(seq_node([Child|_]), S, S1, reactive) :-
    do_node(Child, S, S1, reactive).

do_node(fallback_node([]), S, S, false).
do_node(fallback_node([Child|_]), S, S1, true) :-
    do_node(Child, S, S1, true).
do_node(fallback_node([Child|Rest]), S, S1, Outcome) :-
    do_node(Child, S, S2, false),
    do_node(fallback_node(Rest), S2, S1, Outcome).
do_node(fallback_node([Child|_]), S, S1, reactive) :-
    do_node(Child, S, S1, reactive).

% ---------------------------------------------------------------
% 7. evaluate_plan/4 -- UNCHANGED redescend loop. See the old theory's
%    own extensive header comment (git history on
%    claude/project-review-179rmx) for the full rationale; nothing
%    about it needed to change for this rewrite.
% ---------------------------------------------------------------
evaluate_plan(S0, S, Outcome, Budget) :-
    Budget > 0,
    plan(Node),
    do_node(Node, S0, S1, reactive),
    Budget1 is Budget - 1,
    evaluate_plan(S1, S, Outcome, Budget1).
evaluate_plan(S0, S1, Outcome, _Budget) :-
    plan(Node),
    do_node(Node, S0, S1, Outcome),
    Outcome \== reactive.
evaluate_plan(S0, S0, world_too_large, 0) :-
    plan(Node),
    do_node(Node, S0, _, reactive).

replan_budget(1000).

final_situation(S) :- replan_budget(B), evaluate_plan(s0, S, _, B).

% plan_outcome(Outcome): the WHOLE tree's true/false/world_too_large
% outcome -- unchanged from the old theory.
plan_outcome(Outcome) :- replan_budget(B), evaluate_plan(s0, _, Outcome, B).

% ---------------------------------------------------------------
% 8. HISTORY-SEARCH ACCESSORS -- same shape as the old theory's
%    halted_with/2 family (search S's WHOLE action history, monotonic
%    since a situation only ever grows by appending do(...)), now
%    reading Reason off do(moveto_result(...),S) instead of
%    do(haltMoveto(...),S).
% ---------------------------------------------------------------
halted_with(Reason, do(moveto_result(_,_,Reason,_,_,_), _)).
halted_with(Reason, do(A, S)) :-
    A \= moveto_result(_,_,Reason,_,_,_),
    halted_with(Reason, S).

% visited(+Loc,+S): TRUE iff the robot actually landed at Loc with
% Reason=success (not merely "attempted") anywhere in S's history --
% success is already the tolerance-checked notion (Python only reports
% it for landings within goal_tolerance), so no separate distance
% check is needed here the way the old theory's leg_status/8 needed
% one.
visited(point(GX,GY), do(moveto_result(_,_,success,point(GX,GY),_,_), _)).
visited(Loc, do(_A, S)) :- visited(Loc, S).

crashed_in(S)          :- halted_with(crashed(_), S).
battery_depleted_in(S) :- halted_with(battery_depleted, S).

crashed_obstacle(ObstacleId, S)      :- halted_with(crashed(ObstacleId), S).
battery_below_threshold(Threshold,S) :- halted_with(battery_below(Threshold), S).
battery_equal_threshold(Threshold,S) :- halted_with(battery_equal(Threshold), S).
battery_over_threshold(Threshold,S)  :- halted_with(battery_over(Threshold), S).

any_collision         :- final_situation(S), crashed_in(S).
any_battery_depletion :- final_situation(S), battery_depleted_in(S).

% last_halt(-Reason): reads Reason off S's own OUTERMOST layer without
% searching -- same "no search, just look at the outermost do(...)"
% principle as incoming_branch/2 (Section 2).
holds(last_halt(Reason), do(moveto_result(_,_,Reason,_,_,_), _)).

% recover_obstacle(-ObstacleId): a cond() leaf reading which obstacle
% the branch that led here most recently reacted to -- built on
% last_halt/1, not halted_with/2, for the same reason the old theory's
% version was: it must report the obstacle THIS branch is reacting to,
% not any earlier one still present in S's history.
holds(recover_obstacle(ObstacleId), S) :-
    holds(last_halt(obstacle_on_path(_Threshold,ObstacleId)), S).

% ---------------------------------------------------------------
% 9. THE POLICY. plan/1 and goal_formula/1 both come from the
%    problem's own generated/hand-authored files (consulted via
%    problem_data.pl, see Section 0) -- see plan_generated.pl and
%    goal_formula.pl themselves for the specific problem's own policy
%    and mission goal. verify_goal_formula is the same "zero-arg
%    convenience wrapper hardwired to final_situation" shape any_
%    collision/plan_outcome above already use.
% ---------------------------------------------------------------
verify_goal_formula :- final_situation(S), goal_formula(S).

% ============================================================
% 10. QUERIES
% ============================================================
query(verify_goal_formula).
query(plan_outcome(true)).
query(plan_outcome(false)).
query(plan_outcome(world_too_large)).
query(any_collision).
query(any_battery_depletion).
debug_missing(_,_).
