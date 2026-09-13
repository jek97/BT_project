% goal_formula.pl -- problem41L
%
% Tied to this problem's own behavior_tree.xml (problem6L's tree, reused
% verbatim): the shipped plan's last leg is exit_to_entrance_7 (its own
% cp8), whose PlanWith goal is point(11.3,-15.0) -- "visited it" is the
% one waypoint to check, same convention problem1/4/5/6's own
% goal_formula.pl already use (a single visited/3 conjunct for the
% plan's own final target). Written directly against the tree's own
% real coordinate, NOT copied from problem6L's own goal_formula.pl --
% that file still carries a stale point(0.0,0.0) placeholder left over
% from before problem6/7's waypoints were filled in (see problem6L's
% own file), not fixed there yet.
goal_formula(S) :-
    visited(point(11.3,-15.0), 0.3, S).
