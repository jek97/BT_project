% HAND-WRITTEN STUB -- the offline Python calibrator that should
% generate this file (see FUTUREWORK.md and basic_action_theory.pl
% Section 2's own note on moveto_outcome/7) is not built yet. Numbers
% below are illustrative, chosen to exercise every Reason category
% (success, a reactive trigger, battery_depleted) and the
% incoming-branch mechanism end to end -- NOT calibrated against real
% geometry. DO NOT treat this as auto-generated.
%
% leg_try_goal (InBranch=root -- this is the first leg attempted from
% the plan's own fixed initial position, battery_start=100):
%   b1/b2: reaches the far goal (22.275,2.075) without ever crossing
%          the 70% threshold in this resolved world (low drain).
%   b3:    crosses battery_below(70) partway along the nominal path
%          (drain=30 lands it exactly at the 70% boundary), causing a
%          reactive halt -- evaluate_plan/4 redescends the whole tree
%          from here, cond(battery_over(70)) now fails, and the
%          Fallback moves to leg_go_home.
0.5::moveto_outcome(leg_try_goal, root, b1, success, point(22.275,2.075), 20.1, 25.0) ;
0.3::moveto_outcome(leg_try_goal, root, b2, success, point(22.15,2.02),   19.8, 24.0) ;
0.2::moveto_outcome(leg_try_goal, root, b3, battery_below(70), point(15.4,2.06), 14.0, 30.0).

% leg_go_home (InBranch=b3 -- the ONLY branch of leg_try_goal that
% reaches leg_go_home in this plan's own tree, since b1/b2 succeed and
% the Fallback never tries its second branch at all in those worlds).
% No InBranch=root variant is supplied: with battery_start=100,
% cond(battery_over(70)) is always true on the very first descent, so
% leg_go_home is never reached without going through leg_try_goal's
% own b3 first -- if that ever changes (e.g. a lower battery_start),
% this stub would need a root variant too.
0.7::moveto_outcome(leg_go_home, b3, c1, success,          point(2.275,2.075), 13.0, 15.0) ;
0.3::moveto_outcome(leg_go_home, b3, c2, battery_depleted, point(6.0,2.05),     9.0,  70.0).
