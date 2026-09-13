% goal_formula.pl -- problem5L (formerly problem5)
%
% Tied to this problem's own behavior_tree.xml: the shipped plan's last
% leg returns to the start position (11.3,-15.0) -- "visited it" is the
% one waypoint to check, same convention problem1/problem4's own
% goal_formula.pl already use (a single visited/3 conjunct for the
% plan's own final target).
goal_formula(S) :-
    visited(point(11.3,-15.0), 0.3, S).
