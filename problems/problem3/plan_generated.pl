% HAND-WRITTEN, not translator-generated -- module/translators/
% bt_to_prolog.py does not yet support the shape behavior_tree.xml
% describes for this problem (<Inverter>, ObstacleOnPath used as a
% plain Condition leaf, and dynamically binding obstacle_id from a
% trigger's own halt reason are all documented as KNOWN GAPS in that
% file's own header). This is the Golog-level term that tree is meant
% to translate to, written directly so the reactive-evaluation
% mechanism (leg_status/8's three-valued Status, evaluate_plan/5) can
% actually be tested end-to-end before the translator catches up.
%
% Two branches under one Fallback, matching behavior_tree.xml's own
% Bug0 shape, PLUS an explicit at_goal/3 early exit as the first
% branch (same idiom AtGoal already uses elsewhere in this project --
% "a fallback child that skips moveto entirely if already there"):
%
%   1. at_goal(11.675,11.525,0.3) -- already there, nothing to do.
%   2. Plan straight to goal; walk it, watching obstacle_on_path(0.6)
%      as a trigger.
%   3. Recover WHICH obstacle branch 2's own trigger fired against
%      (recover_obstacle/1, built on obstacle_on_path_obstacle/3 --
%      see basic_action_theory.pl's own note there for why this is
%      needed instead of a shared variable); follow its offset
%      boundary; walk it, watching line_of_sight_clear(Obst1,11.675,11.525)
%      as a trigger.
%
% No third "resume to goal" leg is hand-chained after branch 3 -- it
% doesn't need one. Every trigger above is classified `reactive` (see
% leg_status/8), so EITHER branch halting via its own trigger makes
% evaluate_plan/5 re-descend this SAME tree from the halted situation:
% once line of sight clears, branch 2 (straight to goal) gets tried
% again from the new position, this time (assuming the geometry
% allows it) reaching the goal without re-triggering obstacle_on_path
% at all -- resumption falls out of the reactive mechanism itself,
% for free.
%
% KNOWN LIMITATION: recover_obstacle/1 searches S's WHOLE history for
% an obstacle_on_path halt reason (via halted_with/2), not just the
% MOST RECENT one -- fine for THIS tree, where obstacle_on_path(0.6) is
% branch 2's only reactive trigger (so any reactive failure reaching
% branch 3 can only be that one), but not a safe pattern to copy
% verbatim onto a leg with more than one reactive trigger without
% adding a way to distinguish "most recent" from "any".
plan(fallback_node([
    cond(at_goal(11.675,11.525,0.3)),
    seq_node([
        planWith(straight, point(11.675,11.525), PathS),
        moveto_leg(PathS, [collision,battery,obstacle_on_path(0.6)])
    ]),
    seq_node([
        cond(recover_obstacle(Obst1)),
        planWith(follow_boarder(Obst1,0.6), point(0.0,0.0), PathFB),
        moveto_leg(PathFB, [collision,battery,line_of_sight_clear(Obst1,11.675,11.525)])
    ])
])).
