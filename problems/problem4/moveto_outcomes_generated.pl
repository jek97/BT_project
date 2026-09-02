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
% No InBranch=root/b1/b2 variant is supplied: ProbLog's grounder does
% explore do_node(moveto(leg_go_home,...),...) from those situations
% too while building its formula (see basic_action_theory.pl Section
% 3's own note), but their contribution to every real query is exactly
% zero regardless, since the plan's own Fallback semantics never
% actually reaches leg_go_home from a resolved world where leg_try_goal
% succeeded -- confirmed empirically (adding placeholder rows for
% them changed no query's result). A real offline calibrator would
% still need to decide whether to over-provision these dead
% combinations or rely on a pre-flight validator that understands
% they're unreachable; this stub takes the latter approach.
0.7::moveto_outcome(leg_go_home, b3, c1, success,          point(2.275,2.075), 13.0, 15.0) ;
0.3::moveto_outcome(leg_go_home, b3, c2, battery_depleted, point(6.0,2.05),     9.0,  70.0).
