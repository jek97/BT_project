% goal_formula.pl -- problem5S
%
% Tied to this problem's own behavior_tree.xml: the shipped plan's last
% leg returns to the start position (0,-13.5) -- "visited it" is the
% one waypoint to check, same convention problem1/problem4/problem5L's
% own goal_formula.pl already use (a single visited/3 conjunct for the
% plan's own final target).
goal_formula(S) :-
    visited(point(0,-13.5), 0.3, S).
