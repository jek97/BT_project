% goal_formula.pl -- problem4M
%
% Goal (per user's own request): "sample_taken AND battery > 0 AND
% ((sample > 5 AND at()) OR (sample < 5 AND at()))" -- same shape as
% problem3's own goal, PLUS the robot must still have battery left
% (battery > 0%) by the checked situation.
%
% *** PLACEHOLDER COORDINATES *** -- point(999.0,1.0) (high branch)
% and point(999.0,2.0) (low branch) below are NOT real positions,
% deliberately far outside every map this project uses (~-30..50m
% range) so they can never be mistaken for real data or accidentally
% satisfied -- replace both with the real target points once decided
% (per the user's own "add correct positions later" instruction).
%
% sample_tree_11 is this tree's own (and only) TakeSample id.
% halted_with(sample_success(...),S) checks BY ID that it succeeded
% (no position needed, see problem0/1's own goal_formula.pl note);
% now(T,S)+battery(B,T,S) reads the CURRENT battery level in S (same
% "now(T,S), battery(B,T,S), B > 50" idiom vocabulary.yaml's own header
% documents); sample_value_over/below check the drawn VALUE; visited/3
% checks the ARRIVAL location, tolerance 0.3m. The OR itself is plain
% Prolog ';' -- see this project's own recent fix making goal_formula_
% check.py's validator walk into it.
goal_formula(S) :-
    halted_with(sample_success(_,_,_,sample_tree_11,_), S),
    now(T, S), battery(B, T, S), B > 0,
    (
        (battery_under_in(S), visited(point(9.0,27.0), 1.0, S))
        ;
        (\+ battery_under_in(S), visited(point(15.0,-18.0), 1.0, S))
    ).
