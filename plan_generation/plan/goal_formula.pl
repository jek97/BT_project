% goal_formula.pl
%
% Hand-authored, tied to THIS PARTICULAR plan's own behavior_tree.xml
% -- same "must be kept in sync with whichever tree it verifies"
% convention already accepted for current_plan.pl's own goal/2 vs
% behavior_tree.xml's goal port (see moveto_continuous.pl's own note
% there); if the tree's own waypoints change, update this file too.
%
% goal_formula(S) is a UNIFORM formula in S (Reiter's own sense: S is
% the ONLY free situation term in the body, and every fluent that
% takes a situation argument is applied to exactly this S) -- it is
% NOT hardwired to final_situation itself, so it can in principle be
% checked at any situation; moveto_continuous.pl's own
% verify_goal_formula wrapper is what actually applies it at
% final_situation, matching every other top-level verification query
% (goal_reached, any_collision, plan_outcome, ...).
%
% Written directly in Prolog -- and(P,Q)/or(P,Q) (the mini-language
% cond() leaves use inside a BT tree) is a SEPARATE vocabulary for a
% SEPARATE purpose (serializable as BT.cpp XML attribute text); this
% file has no such constraint, so it just uses Prolog's own ','/2 for
% conjunction directly.
%
% Current formula: the shipped single-leg plan (behavior_tree.xml)
% has exactly one MoveTo, planned toward goal/2's own point
% (11.675,11.525) -- so "visited the goal" is the only waypoint to
% check today. A multi-leg A->B->C plan would list each leg's own
% endpoint here as its own visited(...) conjunct, in the SAME order
% the tree visits them (see visited/2's own note on why, for a plain
% linear Sequence, checking only the LAST waypoint already implies
% every earlier one -- listing them all here is for clarity/
% robustness against a Fallback being added later, not because it's
% strictly needed today).
goal_formula(S) :-
    visited(point(11.675,11.525), S).
