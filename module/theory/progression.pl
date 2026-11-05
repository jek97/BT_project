% ============================================================
% PROGRESSION LAYER on top of basic_action_theory.pl -- driven by
% progression.py (project root), which holds the BT stack and the
% weighted frontier. This file answers one question: given a progressed
% state and one fixed set of noise draws, what does one BT LEAF do?
%
% basic_action_theory.pl is consulted UNCHANGED. Every fluent keeps its
% own successor-state clauses; this file only adds a second base case
% next to s0, the progressed situation
%
%     st(Phys, Mem, Draws)
%
% so a fluent regresses through one leaf's own do(...) terms and stops
% here instead of going back to s0:
%     Phys  = phys(X,Y,Now,AnchorT,AnchorB,Hitch,HitchId,Deployed,
%                  Tools,Cells,ReactiveCount)
%             the current value of every state fluent. (AnchorT,AnchorB)
%             is the last halt's time and battery level, the anchor
%             idle drain continues from.
%     Mem   = mem(Reasons,Outcomes,Checks,Planned,Arrivals,LastHalt)
%             what the history-reading predicates (halted_with/2,
%             visited/3, checked_with/4, ...) need once the do(...)
%             terms that used to answer them are gone.
%     Draws = [d(Name,Value),...] the noise fixed for this one leaf.
%
% No probabilistic fact is ever used here: progression.py loads this
% through DefaultEngine.query (plain SLD, no grounding, no compilation)
% and a generated config in which z/2, zt/2, sample_result/3, ... read
% Draws through fixed_draw/3 instead of being annotated disjunctions.
% zbatt/1 has no situation argument to carry a draw through, so it reads
% current_zb/1, a fact progression.py adds to one per-value database
% extension -- it is one draw for the whole mission, split at the root.
% ============================================================
:- consult('./basic_action_theory.pl').

zbatt(V) :- current_zb(V).
current_zb(no_zbatt_configured) :- fail.

prog_member(X, [X|_]).
prog_member(X, [_|T]) :- prog_member(X, T).

% fixed_draw(+Situation, +Name, ?Value): Value of table Name in the
% Draws at the bottom of Situation.
fixed_draw(st(_,_,Draws), Name, V) :- draw_value(Draws, Name, V).
fixed_draw(do(_,S), Name, V) :- fixed_draw(S, Name, V).

draw_value([d(Name,V)|_], Name, V).
draw_value([d(Other,_)|Rest], Name, V) :-
    Other \= Name,
    draw_value(Rest, Name, V).

% -- STATE FLUENTS: base case reading Phys ----------------------------
now(T, st(phys(_,_,T,_,_,_,_,_,_,_,_),_,_)).
at(X, Y, _, st(phys(X,Y,_,_,_,_,_,_,_,_,_),_,_)).
hitch(Hitch, st(phys(_,_,_,_,_,Hitch,_,_,_,_,_),_,_)).
hitch_id(HitchId, st(phys(_,_,_,_,_,_,HitchId,_,_,_,_),_,_)).
deployed(st(phys(_,_,_,_,_,_,_,true,_,_,_),_,_)).
tool_position(Id, GX, GY, st(phys(_,_,_,_,_,_,_,_,Tools,_,_),_,_)) :-
    prog_member(tool(Id,GX,GY), Tools).
ploughed(Cx, Cy, st(phys(_,_,_,_,_,_,_,_,_,Cells,_),_,_)) :-
    prog_member(cell(Cx,Cy), Cells).
reactive_halt_count(st(phys(_,_,_,_,_,_,_,_,_,_,Count),_,_), Count).

% The same idle formula as battery(Level,T,do(haltMoveto(T1,_,_),S)),
% anchored on the stored (AnchorT,AnchorB) instead of on that action.
% At s0 the anchor is (0,battery_start), which is battery(Level,T,s0).
battery(Level, T, st(phys(_,_,_,AnchorT,AnchorB,_,_,_,_,_,_),_,_)) :-
    idle_drain_rate(IdleRate),
    sigma_battery(SigmaB),
    zbatt(Zb),
    Elapsed is max(0.0, T - AnchorT),
    Deviation is Zb * SigmaB * sqrt(Elapsed),
    NominalDrain is IdleRate*Elapsed,
    noisy_drain(NominalDrain, Deviation, TotalDrain),
    Level is max(0, min(100, AnchorB - TotalDrain)).

% -- HISTORY-READING PREDICATES: base case reading Mem ----------------
% Their recursive clauses are untouched: they scan the current leaf's
% do(...) terms as usual and find the past here at the bottom. A missing
% base case would make one of these fail SILENTLY (\+ visited(...) would
% read true), which is why every one of them is listed explicitly.
situation_reasons(st(_,mem(Reasons,_,_,_,_,_),_), Reasons).
history_outcomes(st(_,mem(_,Outcomes,_,_,_,_),_), Outcomes).
checked_with(Code, C, Status, st(_,mem(_,_,Checks,_,_,_),_)) :-
    prog_member(chk(Code,C,Status), Checks).
planned_with(Algorithm, Reason, st(_,mem(_,_,_,Planned,_,_),_)) :-
    prog_member(pln(Algorithm,Reason), Planned).
visited(point(GX,GY), Tol, st(_,mem(_,_,_,_,Arrivals,_),_)) :-
    prog_member(arrival(GX,GY,X,Y), Arrivals),
    dist(X,Y,GX,GY,D),
    D =< Tol.
sample_success_at(point(GX,GY), Tol, st(_,mem(Reasons,_,_,_,_,_),_)) :-
    prog_member(sample_success(X,Y,_V,_SampleId,_ActionCode), Reasons),
    dist(X,Y,GX,GY,D),
    D =< Tol.
holds(last_halt(Reason), st(_,mem(_,_,_,_,_,last(Reason)),_)).

% -- INITIAL STATE ----------------------------------------------------
initial_state(phys(X,Y,0,0,B,free,free,false,Tools,[],0),
              mem([],[],[],[],[],none)) :-
    start(X, Y),
    battery_start(B),
    findall(tool(Id,GX,GY),
            (tool_instance(Id,_), tool_start_position(Id,GX,GY)),
            Tools).

% -- ONE LEAF ---------------------------------------------------------
% progress_leaf(+Leaf,+Phys0,+Mem0,+Choices,-Draws,-Outcome,-Phys,-Mem):
% one solution per joint draw from Choices = [choice(Name,Values),...]
% for which Leaf has a successor. A draw with no solution is a dead end,
% which progression.py keeps as an explicit stuck final state. Leaf may
% hold unbound blackboard variables, and comes back instantiated.
progress_leaf(Leaf, Phys0, Mem0, Choices, Draws, Outcome, Phys, Mem) :-
    pick_draws(Choices, Draws),
    S0 = st(Phys0, Mem0, Draws),
    do_node(Leaf, S0, S1, Outcome),
    next_state(Phys0, S1, Phys, Mem).

pick_draws([], []).
pick_draws([choice(Name,Values)|Rest], [d(Name,V)|Draws]) :-
    prog_member(V, Values),
    pick_draws(Rest, Draws).

% next_state(+Phys0,+S1,-Phys,-Mem): the progression itself. Reads every
% fluent at S1 through the ordinary axioms. The do(...) terms of S1 are
% then discarded, since Phys and Mem say everything the future can ask.
next_state(phys(_,_,_,AnchorT0,AnchorB0,_,_,_,_,_,_), S1,
           phys(X,Y,Now,AnchorT,AnchorB,Hitch,HitchId,Deployed,Tools,Cells,Count),
           mem(Reasons,Outcomes,Checks,Planned,Arrivals,LastHalt)) :-
    now(Now, S1),
    at(X, Y, Now, S1),
    anchor_after(S1, AnchorT0, AnchorB0, AnchorT, AnchorB),
    hitch(Hitch, S1),
    hitch_id(HitchId, S1),
    deployed_flag(S1, Deployed),
    findall(tool(Id,GX,GY),
            (tool_instance(Id,_), tool_position(Id,GX,GY,S1)),
            Tools),
    findall(cell(Cx,Cy), ploughed(Cx,Cy,S1), Cells0),
    sort(Cells0, Cells),
    reactive_halt_count(S1, Count),
    situation_reasons(S1, Reasons),
    history_outcomes(S1, Outcomes),
    all_checks(S1, Checks),
    all_planned(S1, Planned),
    all_arrivals(S1, Arrivals),
    last_halt_of(S1, LastHalt).

halt_time(haltMoveto(T,_,_), T).
halt_time(interrupt(T), T).
halt_time(halt_install_tool(T,_,_), T).
halt_time(halt_uninstall_tool(T,_,_), T).
halt_time(halt_deploy_tool(T,_,_), T).
halt_time(halt_retract_tool(T,_,_), T).

anchor_after(do(A,S), _, _, T, Level) :-
    halt_time(A, T),
    battery(Level, T, do(A,S)).
anchor_after(do(A,_), AnchorT, AnchorB, AnchorT, AnchorB) :-
    \+ halt_time(A, _).

deployed_flag(S, true) :- deployed(S).
deployed_flag(S, false) :- \+ deployed(S).

all_checks(st(_,mem(_,_,Checks,_,_,_),_), Checks).
all_checks(do(checked(Code,C,Status),S), [chk(Code,C,Status)|Rest]) :-
    all_checks(S, Rest).
all_checks(do(A,S), Rest) :-
    A \= checked(_,_,_),
    all_checks(S, Rest).

all_planned(st(_,mem(_,_,_,Planned,_,_),_), Planned).
all_planned(do(planned(Algorithm,Reason),S), [pln(Algorithm,Reason)|Rest]) :-
    all_planned(S, Rest).
all_planned(do(A,S), Rest) :-
    A \= planned(_,_),
    all_planned(S, Rest).

% One arrival per completed leg, with its exact position: visited/3's
% tolerance is only known when it is asked.
all_arrivals(st(_,mem(_,_,_,_,Arrivals,_),_), Arrivals).
all_arrivals(do(haltMoveto(T,completed(Code),true),S), [arrival(GX,GY,X,Y)|Rest]) :-
    current_walk(S, CP, _Triggers, _T0, _SPrev),
    leg_target(CP, GX, GY),
    at(X, Y, T, do(haltMoveto(T,completed(Code),true),S)),
    all_arrivals(S, Rest).
all_arrivals(do(A,S), Rest) :-
    A \= haltMoveto(_,completed(_),true),
    all_arrivals(S, Rest).

last_halt_of(S, last(Reason)) :- holds(last_halt(Reason), S).
last_halt_of(S, none) :- \+ holds(last_halt(_), S).

% walk_seed(+Phys0,+Mem,-Phys): what a moveto_leg about to start actually
% reads from Phys0. Its T0 and B0 are rounded by the merge grids and its
% path is already in the leaf, so exact position, time and battery stop
% mattering here. Two states that agree on the seed have the same future,
% which is the point where they can merge. Position is replaced by the
% atom dropped, so an unexpected read fails loudly. progression.py only
% calls this for legs whose triggers are collision/battery.
walk_seed(phys(X,Y,Now,AnchorT,AnchorB,Hitch,HitchId,Deployed,Tools,Cells,Count), Mem,
          phys(dropped,dropped,T0,T0,B0,Hitch,HitchId,Deployed,Tools,Cells,Count)) :-
    Phys0 = phys(X,Y,Now,AnchorT,AnchorB,Hitch,HitchId,Deployed,Tools,Cells,Count),
    disc_step_time(Grid),
    quantize_up(Now, Grid, T0),
    leg_start_battery(T0, st(Phys0,Mem,[]), B0).

% -- FINAL-STATE QUERIES ----------------------------------------------
% The queries of queries_generated.pl and config_generated.pl, read at
% a final state instead of through final_situation/1.
final_goal(Phys, Mem) :- goal_formula(st(Phys,Mem,[])).

final_signature(Phys, Mem, Signature) :-
    history_outcomes(st(Phys,Mem,[]), Signature).

final_answers(Queries, Phys, Mem, Answers) :-
    findall(Q, (prog_member(Q,Queries), final_holds(Q, st(Phys,Mem,[]))), Answers).

final_holds(any_reason_pattern(Pattern), S) :-
    situation_reasons(S, Reasons),
    reasons_pattern(Pattern, _ActionCode, Reasons).
final_holds(any_reason_pattern_by_action(Pattern,ActionCode), S) :-
    situation_reasons(S, Reasons),
    reasons_pattern(Pattern, ActionCode, Reasons).
final_holds(any_reason_pattern_detail_by_action(Pattern,ActionCode), S) :-
    situation_reasons(S, Reasons),
    reasons_pattern_detail(Pattern, ActionCode, Reasons).
final_holds(any_condition_status(Code,Status), S) :-
    checked_with(Code, _C, Status, S).
final_holds(any_battery_depletion, S) :-
    battery_depleted_in(S).
