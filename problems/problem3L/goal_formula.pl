% goal_formula.pl -- problem3L
%
% Goal (per user's own request): "sample_taken AND ((sample > 5 AND
% at()) OR (sample < 5 AND at()))" -- the sample must have succeeded,
% AND, depending on whether its own drawn value came back ABOVE or
% BELOW 5, the robot must have visited a DIFFERENT target location (A
% if high, B if low).
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
% sample_value_over/below (vocabulary.yaml) check the drawn VALUE;
% visited/3 checks the ARRIVAL location, tolerance 0.3m (same
% convention every other visited/3 conjunct in this project uses).
% The OR itself is plain Prolog ';' -- see this project's own recent
% fix making goal_formula_check.py's validator walk into it.
goal_formula(S) :-
    halted_with(sample_success(_,_,_,sample_tree_11,_), S),
    visited(point(45.5,-36.0), 1.0, S).
