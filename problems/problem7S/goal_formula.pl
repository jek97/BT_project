% goal_formula.pl -- problem7S
%
% Tied to this problem's own behavior_tree.xml: the shipped plan's last
% leg is return_to_entrance_7 (the second branch's own final waypoint)
% -- "visited it" is the one waypoint to check, same convention
% problem1/4/5's own goal_formula.pl already use (a single visited/3
% conjunct for the plan's own final target).
%
% PLACEHOLDER: point(0.0,0.0) needs the same real (X,Y) as
% return_to_entrance_7's own PlanWith goal in behavior_tree.xml (its
% own cp10) once picked -- see that file's own header note.
goal_formula(S) :-
    visited(point(0.0,0.0), 0.3, S).
