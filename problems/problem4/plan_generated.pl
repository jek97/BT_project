% HAND-WRITTEN STUB (simplified-branch atomic moveto shape) --
% bt_to_prolog.py does not yet translate behavior_tree.xml into this
% leaf shape (moveto(LegId,Algorithm,Goal), see basic_action_theory.pl
% Section 3), so this stands in until it does -- same "translator
% gap, hand-written" precedent problem3's own plan_generated.pl
% already set for the pre-simplification theory. DO NOT treat this as
% auto-generated; it will NOT be overwritten by main.py until the
% translator is updated to support atomic moveto legs.
%
% Same tree shape as behavior_tree.xml describes: try the far goal
% while battery allows it, otherwise go home. leg_try_goal and
% leg_go_home are the LegIds moveto_outcomes_generated.pl's own rows
% are keyed by.
plan(fallback_node([
    seq_node([cond(battery_over(70)),
              moveto(leg_try_goal, straight, point(22.275,2.075))]),
    seq_node([moveto(leg_go_home, straight, point(2.275,2.075))])
])).
