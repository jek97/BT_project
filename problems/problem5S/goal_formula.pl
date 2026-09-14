% goal_formula.pl -- problem5S
%
% Not specified by the user's own request for this problem -- chosen
% as the natural check for a minimal install/uninstall round trip:
% nothing left attached by the end of the mission, i.e. install_
% closest_cart's own cart was successfully installed AND later
% successfully uninstalled again (a failed InstallTool/UninstallTool
% attempt, or a crash/battery depletion partway through, would leave
% hitch/2 at a state other than free). Same idiom vocabulary.yaml's
% own hitch/2 entry documents ("goal_formula(S) :- hitch(plow,S)." for
% "the plow is attached by the end") -- this is its inverse.
goal_formula(S) :-
    visited(point(0.5,6.0), 1.0, S),
    hitch(free, S).
