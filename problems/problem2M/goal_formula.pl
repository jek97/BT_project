% goal_formula.pl -- problem2M
%
% Goal (per user's own request): "ploughed()" -- the swath THIS
% problem's own plowing SubTree call actually ploughs must be fully
% ploughed by the end of the plan. p1="5.0;-9.0" p2="25.0;-9.0" are
% copied directly from behavior_tree.xml's own <SubTree ID="plowing"
% p1="5.0;-9.0" p2="25.0;-9.0"/> -- the real coordinates already in
% this problem's own tree, not a placeholder.
%
% ploughed_between/5 (vocabulary.yaml) is the SAME check schema.yaml's
% own PloughedBetween condition uses inside the tree itself: TRUE iff
% EVERY cell the straight line between p1's own cell center and p2's
% touches (a Bresenham line over the discretized ploughing grid, NOT
% the full box spanned by p1/p2) is currently ploughed.
goal_formula(S) :-
    ploughed_between(5.0, -9.0, 25.0, -9.0, S).
