"""
moveto_calibrator.py

Offline calibration for the simplified atomic moveto(LegId,Algorithm,Goal)
action theory (see basic_action_theory.pl's own header and Section 2's
moveto_outcome/7 interface documentation for the contract calibrate_moveto()'s
output must satisfy).

CURRENT SCOPE (first pass, position only): the noise model is the SAME
Wiener-process-consistent Gaussian model collision_geometry.py's own
_walk_noisy_point already used before this branch's simplification --
sigma scaled by sqrt(duration), Brownian-bridge-consistent (zero at the
start, growing with elapsed time) -- just evaluated ONCE, at the leg's
own completion, instead of resampled continuously along a walk. Battery
drain is a flat, DETERMINISTIC nominal value (duration * moving_drain_rate)
-- not yet stochastic, and every returned row has Reason=success. Both
are explicit placeholders: extending this to also classify
crashed(ObstacleId)/battery_depleted/reactive(battery_below(...))
branches (stochastic battery drain, obstacle clearance) is separate,
later work -- see FUTUREWORK.md and basic_action_theory.pl Section 2's
own full Reason vocabulary.

Using a genuine Wiener process INTERNALLY here does NOT reintroduce the
combinatorial blowup the simplified theory exists to avoid: the noise
is integrated/discretized HERE, offline, once per (leg, incoming-branch)
pair, and only the resulting small flat table -- a handful of
(probability, branch) rows -- ever reaches ProbLog. The blowup was never
caused by the noise model's own sophistication, it was caused by
exposing z/zt as ProbLog's OWN random variables, re-referenced through a
live bracket-scan search and accumulated across hops. Nothing here does
either: this function is called at most once per (LegId,InBranch) pair
(see moveto_outcomes_orchestrator.py's own memoization), and its output
is a static, flat table.

Currently only algorithm="straight" is implemented (the nominal path is
the direct segment from start to goal). astar/voronoi/follow_boarder
fall back to the straight-line distance/direction with a documented
TODO -- see _nominal_path_straight's own note.
"""

import math


def _nominal_path_straight(start, goal):
    """(distance, tangent_unit, perp_unit) for the direct segment from
    start to goal. astar/voronoi/follow_boarder would need their own
    nominal-path shape (via planners.py) to compute a genuine arc
    length and end tangent direction instead of the straight-line
    approximation used here -- not yet implemented; every algorithm
    currently falls back to this."""
    sx, sy = start
    gx, gy = goal
    dx, dy = gx - sx, gy - sy
    norm = math.hypot(dx, dy)
    if norm <= 1.0e-9:
        return 0.0, (0.0, 0.0), (0.0, 0.0)
    tan_x, tan_y = dx / norm, dy / norm
    perp_x, perp_y = -dy / norm, dx / norm
    return norm, (tan_x, tan_y), (perp_x, perp_y)


def calibrate_moveto(leg_id, in_branch, start, battery_in, goal, algorithm,
                      triggers, config):
    """calibrate_moveto(LegId, InBranch, Start, BatteryIn, Goal, Algorithm,
    Triggers, Config) -> list of row dicts, one per outcome branch,
    probabilities summing to 1.0. See this file's own header for current
    scope (position noise only, deterministic nominal drain, always
    Reason='success').

    LegId/InBranch: pure labels (see the interface discussion this was
    designed against) -- not used in the computation below, just
    threaded through by the caller for logging/error messages.
    BatteryIn: accepted per the interface contract (needed in general to
    decide whether a battery-crossing branch is even plausible for this
    leg) but not yet consumed, since no branch here is battery-crossing
    yet.
    Triggers: which reactive conditions this leg should watch -- not yet
    consumed either, since no trigger types besides plain arrival are
    modeled in this first pass.
    """
    del in_branch, battery_in, triggers  # not yet used, see docstring

    distance, (tan_x, tan_y), (perp_x, perp_y) = _nominal_path_straight(start, goal)

    speed = config["motion"]["speed"]
    duration = distance / speed if speed > 0 else 0.0

    moving_drain_rate = config["battery"]["moving_drain_rate"]
    drain = duration * moving_drain_rate

    sigma_pos = config["noise"]["position"]["sigma"]
    sigma_tan = config["noise"]["tangential"]["sigma"]
    pos_table = config["noise"]["position"]["discretized_gaussian"]
    tan_table = config["noise"]["tangential"]["discretized_gaussian"]

    gx, gy = goal
    rows = []
    n = 0
    for pos_entry in pos_table:
        for tan_entry in tan_table:
            n += 1
            z = pos_entry["value"]
            zt = tan_entry["value"]
            probability = pos_entry["weight"] * tan_entry["weight"]
            normal_dev = z * sigma_pos * math.sqrt(duration) if duration > 0 else 0.0
            tangent_dev = zt * sigma_tan * math.sqrt(duration) if duration > 0 else 0.0
            ex = gx + normal_dev * perp_x + tangent_dev * tan_x
            ey = gy + normal_dev * perp_y + tangent_dev * tan_y
            rows.append({
                "branch_id": f"p{n}",
                "probability": probability,
                "reason": "success",
                "end_point": (ex, ey),
                "duration": duration,
                "drain": drain,
            })
    return rows
