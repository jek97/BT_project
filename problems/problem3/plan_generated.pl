% HAND-WRITTEN, not translator-generated -- module/translators/
% bt_to_prolog.py does not yet support the shape behavior_tree.xml
% describes for this problem (<Inverter>, ObstacleOnPath used as a
% plain Condition leaf, and dynamically binding obstacle_id from a
% trigger's own halt reason are all documented as KNOWN GAPS in that
% file's own header). This is the Golog-level term that tree is meant
% to translate to, written directly so the reactive-evaluation
% mechanism (leg_status/8's three-valued Status, evaluate_plan/4) can
% actually be tested end-to-end before the translator catches up.
%
% Two branches under one Fallback, matching behavior_tree.xml's own
% Bug0 shape, PLUS an explicit at_goal/3 early exit as the first
% branch (same idiom AtGoal already uses elsewhere in this project --
% "a fallback child that skips moveto entirely if already there"):
%
%   1. at_goal(11.675,11.525,0.3) -- already there, nothing to do.
%   2. Guarded by cond(neg(last_halt(obstacle_on_path(_,_)))): plan
%      straight to goal; walk it, watching obstacle_on_path(0.6) as a
%      trigger.
%   3. Recover WHICH obstacle branch 2's own trigger fired against
%      (recover_obstacle/1, built on last_halt/1 -- see basic_action_
%      theory.pl's own note there); follow its offset boundary; walk
%      it, watching line_of_sight_clear(Obst1,11.675,11.525) as a
%      trigger.
%
% No third "resume to goal" leg is hand-chained after branch 3 -- it
% doesn't need one. Every trigger above is classified `reactive` (see
% leg_status/8), so EITHER branch halting via its own trigger makes
% evaluate_plan/4 re-descend this SAME tree from the halted situation:
% once line of sight clears, branch 2 (straight to goal) gets tried
% again from the new position, this time (assuming the geometry
% allows it) reaching the goal without re-triggering obstacle_on_path
% at all -- resumption falls out of the reactive mechanism itself,
% for free.
%
% Branch 2's guard, cond(neg(last_halt(obstacle_on_path(_,_)))), is
% what actually makes that resumption (and the initial descent) work,
% and it's worth spelling out why it's phrased this way rather than as
% a live geometric check:
%   - At s0, nothing has halted yet, so last_halt/1 has no solution,
%     neg(...) succeeds, and branch 2 is tried -- covering the "very
%     first attempt" case with no separate "is anything undefined yet"
%     guard needed.
%   - Right after branch 2's own obstacle_on_path trigger halts,
%     last_halt/1 reports EXACTLY that reason, neg(...) fails, so
%     branch 2 (as a whole) reports Status=false instead of looping on
%     `reactive` forever -- letting the enclosing fallback_node fall
%     through to branch 3 on THIS SAME descent.
%   - Right after branch 3's own line_of_sight_clear trigger halts,
%     last_halt/1 reports THAT reason (not obstacle_on_path), so
%     neg(...) succeeds again and branch 2 is retried -- this is why
%     the guard is checked against S's own recorded history rather
%     than current_walk/5's live geometry: current_walk/5 would still
%     be reporting branch 3's OWN boundary-following path (which is,
%     by construction, held within threshold of the obstacle for its
%     whole length), permanently blocking branch 2 from ever being
%     retried after a successful recovery.
plan(fallback_node([
    cond(at_goal(11.675,11.525,0.3)),
    seq_node([
        cond(neg(last_halt(obstacle_on_path(_,_)))),
        planWith(straight, point(11.675,11.525), PathS),
        moveto_leg(PathS, [collision,battery,obstacle_on_path(0.6)])
    ]),
    seq_node([
        cond(recover_obstacle(Obst1)),
        planWith(follow_boarder(Obst1,0.6), point(0.0,0.0), PathFB),
        moveto_leg(PathFB, [collision,battery,line_of_sight_clear(Obst1,11.675,11.525)])
    ])
])).
