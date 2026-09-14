% goal_formula.pl -- problem1M
%
% Goal (per user's own request, same as problem0's): "sample_taken()
% AND sample_taken()" -- BOTH TakeSample occurrences in this tree
% (id="sample_tree_11" and id="sample_tree_53", see behavior_tree.xml)
% must have SUCCEEDED somewhere in the resolved plan's own history.
%
% halted_with/2 (vocabulary.yaml) is used here rather than
% sample_success_at/3 specifically because this check is BY SampleId,
% not by location -- no positions/coordinates needed for this goal at
% all (see basic_action_theory.pl's own halted_with(Reason,S) clause
% for take_sample: it matches do(take_sample(...),S) directly, not
% just haltMoveto). sample_success(_,_,_,SampleId,_) wildcards
% everything except which sample this is (X, Y, V, ActionCode).
%
% Conjunction here is plain Prolog comma (AND) -- goal_formula.pl is a
% plain Prolog file, not a BT.cpp tree, so there is no Sequence/and()/
% or() connector to reach for; see this project's own note on why.
goal_formula(S) :-
    halted_with(sample_success(_,_,_,sample_tree_11,_), S),
    halted_with(sample_success(_,_,_,sample_tree_53,_), S).
