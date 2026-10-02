% goal_formula.pl -- problem5L
%
% Goal: "visited()" -- the robot's single MoveTo leg must have
% completed and ended within tolerance of its own target point. No
% TakeSample in this tree (see behavior_tree.xml -- this problem is
% deliberately just PlanWithWaypoints+MoveTo, nothing else), so unlike
% every other problemN's goal_formula.pl there is no halted_with(
% sample_success(...),S) conjunct here -- visited/3 alone is the whole
% goal.
%
% point(32.0,-3.3) is this leg's own target (the last waypoint of the
% PlanWithWaypoints call in behavior_tree.xml, copied from problem3L's
% first leg, which this problem is a trimmed-down copy of). Tolerance
% 1.0m matches problem3L's own c1/c2 DistanceBelow threshold for the
% same point.
goal_formula(S) :-
    visited(point(32.0,-3.3), 1.0, S).
