#!/usr/bin/env python3
"""
module/translators/config_to_prolog.py

Turns a problem's config.yaml into config_generated.pl -- plain Prolog
facts, same predicate names/arities basic_action_theory.pl used to
define inline (robot_radius/1, sigma/1, battery_start/1, the z/2 and
zbatt/1 annotated disjunctions, etc.) -- see config.yaml's own header
for the full rationale.

This mirrors an ALREADY-ESTABLISHED project pattern: occgrid_to_problog.py
generates obstacles_generated.pl from a map, and bt_to_prolog.py
generates plan_generated.pl from a BT.cpp XML tree -- both siblings of
this file in module/translators/, both writing into the SAME problem
directory this file does. config_generated.pl is a third instance of
the same "source data -> generated Prolog facts, consulted separately"
shape, just with config.yaml as the source instead of a map or a plan.

config.yaml's own top level is organized by PHYSICAL QUANTITY
(position:, battery:, ...) rather than by "noise vs. drain vs.
grounding" -- see that file's own header for why -- and the Prolog FACT
NAMES this emits mirror config.yaml's own key names directly (sigma/1,
sigma_tangential/1, battery_start/1, disc_step_position/1, ...) -- the
three discretization-step knobs (disc_step_position/1, disc_step_
battery/1, disc_step_time/1) are named identically to their own
config.yaml keys (position.disc_step_position, battery.disc_step_
battery, grounding.disc_step_time) specifically so the two stay
trivially greppable as the same knob.

battery.enabled (config.yaml) drives TWO things here:
  - battery_enabled/1: a plain fact basic_action_theory.pl doesn't
    itself read (nothing there branches on it) -- it exists so other
    generators/tooling can inspect it without re-parsing config.yaml.
  - whether query(any_battery_depletion) is emitted at all (see
    render_prolog's own note on this below) -- basic_action_theory.pl's
    own Section 10 no longer declares this query as a hardcoded fact,
    specifically so it can be conditional on a PER-PROBLEM config
    value instead of being the same for every problem.
Every OTHER battery-related fact (battery_start/1, idle_drain_rate/1,
moving_drain_rate/1, sigma_battery/1, disc_step_battery/1, zbatt/1's
own table) is ALWAYS emitted regardless of battery.enabled -- battery/3
(the fluent itself) is unconditional theory code, still tracking charge
level either way; only whether battery can ever be a HALTING cause
(handled in module/translators/bt_to_prolog.py, which strips battery-
related trigger names out of every leg's own Triggers list when
disabled) and whether depletion is queried change.

Usage:
    python3 module/translators/config_to_prolog.py
        (regenerates config_generated.pl from config.yaml, both in
        problems/problem0/ by default)

main.py calls generate() itself before every run, so you don't
normally need to run this by hand -- it's here mainly so
config_generated.pl can be regenerated/inspected on its own, and so the
generation logic has exactly one implementation.
"""
import os
import re
import sys

import yaml

# A valid unquoted Prolog atom -- same shape bt_to_prolog.py's own
# _VALID_PROLOG_ATOM_RE independently checks a BT tree's own tool="..."
# port against (these two files never share imports for tiny, stable
# checks like this -- same reasoning _TOOL_KINDS below is its own copy
# rather than a cross-file import).
_VALID_PROLOG_ATOM_RE = re.compile(r"^[a-z][a-zA-Z0-9_]*$")

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
_DEFAULT_PROBLEM_DIR = os.path.join(_PROJECT_ROOT, "problems", "problem0")
DEFAULT_CONFIG_PATH = os.path.join(_DEFAULT_PROBLEM_DIR, "config.yaml")
DEFAULT_OUTPUT_PATH = os.path.join(_DEFAULT_PROBLEM_DIR, "config_generated.pl")


def load_config(config_path=DEFAULT_CONFIG_PATH):
    with open(config_path) as f:
        return yaml.safe_load(f)


def _check_gaussian_weights(label, discretized_gaussian):
    total = sum(entry["weight"] for entry in discretized_gaussian)
    if abs(total - 1.0) > 1e-9:
        print(
            f"[warn] config.yaml's {label}.discretized_gaussian weights "
            f"sum to {total!r}, not 1.0 -- ProbLog silently treats the missing "
            f"mass as an implicit failure branch, which will cap every "
            f"downstream probability. Fix the weights in config.yaml.",
            file=sys.stderr,
        )


def _format_number(x):
    """Preserve int vs. float formatting from the YAML source (e.g.
    battery_start(100). stays an integer fact, sigma(0.15). stays a
    float) -- Prolog's own arithmetic treats the two interchangeably,
    this is purely for the generated file to read naturally."""
    return repr(x)


def _gaussian_disjunction(functor, args_prefix, discretized_gaussian):
    """Build one annotated-disjunction block, e.g.:
        0.0606::z(do(startMoveto(CP,Triggers,ActionCode,T0),S), -2.0) ;
        ...
        0.0606::z(do(startMoveto(CP,Triggers,ActionCode,T0),S),  2.0).
    or, for a zero-argument functor like zbatt/1:
        0.0606::zbatt(-2.0) ;
        ...
        0.0606::zbatt( 2.0).
    """
    lines = []
    n = len(discretized_gaussian)
    for i, entry in enumerate(discretized_gaussian):
        weight = _format_number(entry["weight"])
        value = _format_number(entry["value"])
        head = f"{functor}({args_prefix}{value})" if args_prefix else f"{functor}({value})"
        terminator = " ;" if i < n - 1 else "."
        lines.append(f"{weight}::{head}{terminator}")
    return "\n".join(lines)


def _binary_result_block(functor, success_probability):
    """Build a two-outcome annotated disjunction keyed by (S,ActionCode),
    e.g. for functor="sample_result":
        0.5::sample_result(S,ActionCode,true) ;
        0.5::sample_result(S,ActionCode,false).
    S and ActionCode are free variables in the head -- ProbLog grounds
    one instance per DISTINCT (S,ActionCode) pair actually encountered
    (the same "free variables in an annotated-disjunction head" idiom
    z/2's own key already relies on), so a genuinely new situation
    always gets an independent draw, while an identical situation term
    reached twice (via ProbLog's own proof-sharing) shares the SAME
    cached one -- see basic_action_theory.pl's own do_node(take_sample
    (...))/do_node(install_tool_leg(...)) notes, the three callers of
    this same shape (sample_result/3, install_tool_result/3,
    uninstall_tool_result/3). failure_probability is DERIVED as
    1-success_probability, never taken as a second config value, so
    the two outcomes can never fail to sum to 1.0 -- avoiding the
    exact missing-annotated-disjunction-mass bug _check_gaussian_
    weights above warns about, by construction rather than by
    validation."""
    p_success = _format_number(success_probability)
    p_failure = _format_number(1.0 - success_probability)
    return (f"{p_success}::{functor}(S,ActionCode,true) ;\n"
            f"{p_failure}::{functor}(S,ActionCode,false).")


# The only two tool kinds install_tool/uninstall_tool currently accept
# -- see module/translators/bt_to_prolog.py's own _TOOL_KINDS (the
# SAME set, validated at translation time against a BT XML's own
# tool="..." port).
_TOOL_KINDS = ("cart", "plow")
_DEFAULT_TOOL_DURATION_S = 10.0
_DEFAULT_TOOL_SUCCESS_PROBABILITY = 0.9


def _tool_moveto_param_facts(functor, param_key, equipped_cfg, base_value):
    """Build one MoveTo-parameter fact per tool kind PLUS free, e.g. for
    functor="tool_speed", param_key="speed":
        tool_speed(free, 1.0).
        tool_speed(cart, 1.0).
        tool_speed(plow, 1.0).
    free ALWAYS gets base_value (motion.speed or battery.moving_drain_
    rate -- the SAME value speed/1 or moving_drain_rate/1 itself
    already carries, see basic_action_theory.pl's own tool_speed/2
    note) -- there is no config.yaml key for "no tool equipped", since
    that case already has its own name (motion.speed/battery.moving_
    drain_rate). cart/plow each default INDEPENDENTLY to base_value too
    if config.yaml's tool.equipped.<tool>.<param_key> key is missing
    (same per-key-default style as _tool_duration_facts above), so a
    problem with no tool.equipped section at all -- or one that only
    overrides ONE tool -- behaves EXACTLY like the un-equipped case for
    whichever tool(s) it doesn't mention. This is what makes "if the
    robot is equipped with the cart or the plow, MoveTo uses these
    parameters instead" (this feature's own request) an OPT-IN override
    per tool, not a required one."""
    lines = [f"{functor}(free, {_format_number(base_value)})."]
    for tool in _TOOL_KINDS:
        value = float(equipped_cfg.get(tool, {}).get(param_key, base_value))
        lines.append(f"{functor}({tool}, {_format_number(value)}).")
    return "\n".join(lines)


def _tool_duration_facts(functor, duration_cfg):
    """Build one Duration fact per tool kind, e.g. for
    functor="install_tool_duration":
        install_tool_duration(cart, 10.0).
        install_tool_duration(plow, 10.0).
    duration_cfg is config.yaml's own tool.install.duration_seconds (or
    tool.uninstall.*) mapping -- EACH tool defaults independently to
    _DEFAULT_TOOL_DURATION_S if its own key is missing (same per-key-
    default style disc_step_position/disc_step_battery/disc_step_time
    already use above), so config.yaml never needs a tool.* section at
    all unless a problem actually wants to override it -- see
    basic_action_theory.pl's own install_tool_duration/2 note. Genuinely
    PER-TOOL from day one (not a single shared constant retrofitted
    later), per this feature's own request, even though both tools
    currently default to the SAME value."""
    lines = []
    for tool in _TOOL_KINDS:
        duration = float(duration_cfg.get(tool, _DEFAULT_TOOL_DURATION_S))
        lines.append(f"{functor}({tool}, {_format_number(duration)}).")
    return "\n".join(lines)


def render_prolog(config):
    position_cfg = config["position"]
    battery_cfg = config["battery"]

    _check_gaussian_weights("position.lateral", position_cfg["lateral"]["discretized_gaussian"])
    _check_gaussian_weights("position.tangential", position_cfg["tangential"]["discretized_gaussian"])
    _check_gaussian_weights("battery", battery_cfg["discretized_gaussian"])

    # position.disc_step_position/battery.disc_step_battery default to 0
    # if omitted; grounding.disc_step_time the same, via its own (much
    # smaller, single-purpose) grounding: section -- all three
    # "disabled, exact" at 0, per basic_action_theory.pl's own quantize/
    # quantize_down/quantize_up.
    disc_step_position = position_cfg.get("disc_step_position", 0.0)
    disc_step_battery = battery_cfg.get("disc_step_battery", 0.0)
    disc_step_time = config.get("grounding", {}).get("disc_step_time", 0.0)

    battery_enabled = battery_cfg.get("enabled", True)

    # ActionCode is a free variable here, same as CP/Triggers/T0 --
    # z/zt's own key doesn't need its ACTUAL value (CP+T0+S already
    # uniquely pin down "this leg"), it just has to be PRESENT so the
    # key's arity matches startMoveto/4 (see basic_action_theory.pl's
    # own poss(haltMoveto(...))/tag_reason note for why startMoveto
    # gained this 4th argument).
    z_block = _gaussian_disjunction(
        "z", "do(startMoveto(CP,Triggers,ActionCode,T0),S), ",
        position_cfg["lateral"]["discretized_gaussian"])
    zt_block = _gaussian_disjunction(
        "zt", "do(startMoveto(CP,Triggers,ActionCode,T0),S), ",
        position_cfg["tangential"]["discretized_gaussian"])
    zbatt_block = _gaussian_disjunction(
        "zbatt", "", battery_cfg["discretized_gaussian"])

    # sample.success_probability defaults to 0.5 if the whole `sample:`
    # section is omitted -- an OPTIONAL feature (not every problem's
    # tree uses <TakeSample/>), same "optional knob, sensible default"
    # treatment as disc_step_position/disc_step_battery/disc_step_time
    # above, unlike the core physical constants (robot_radius, sigma,
    # ...) which have no default and are required.
    sample_success_probability = config.get("sample", {}).get("success_probability", 0.5)
    sample_block = _binary_result_block("sample_result", float(sample_success_probability))

    # tool.install/tool.uninstall default entirely if the whole `tool:`
    # section is omitted -- same "optional feature" treatment as
    # sample: above (not every problem's tree uses <InstallTool/>/
    # <UninstallTool/>). SEPARATE success_probability for install vs
    # uninstall (this feature's own request: physically different
    # operations, no reason to share one number) -- both default to
    # _DEFAULT_TOOL_SUCCESS_PROBABILITY if their own key is missing.
    install_cfg = config.get("tool", {}).get("install", {})
    uninstall_cfg = config.get("tool", {}).get("uninstall", {})
    install_success_probability = float(
        install_cfg.get("success_probability", _DEFAULT_TOOL_SUCCESS_PROBABILITY))
    uninstall_success_probability = float(
        uninstall_cfg.get("success_probability", _DEFAULT_TOOL_SUCCESS_PROBABILITY))
    install_tool_result_block = _binary_result_block("install_tool_result", install_success_probability)
    uninstall_tool_result_block = _binary_result_block("uninstall_tool_result", uninstall_success_probability)
    install_tool_duration_facts = _tool_duration_facts(
        "install_tool_duration", install_cfg.get("duration_seconds", {}))
    uninstall_tool_duration_facts = _tool_duration_facts(
        "uninstall_tool_duration", uninstall_cfg.get("duration_seconds", {}))

    # install_tool_drain_rate/1, uninstall_tool_drain_rate/1: the
    # battery drain rate install_tool/uninstall_tool use for THEIR OWN
    # span (see basic_action_theory.pl's own tool_battery_at_leg/7 and
    # battery/3's do(start_install_tool(...),S)/do(start_uninstall_tool
    # (...),S) clauses) -- ONE value per action TYPE (not per tool,
    # matching success_probability's own shape above, per this
    # feature's own "specific for the action of installing or
    # uninstalling any tool" request), defaulting to battery.idle_
    # drain_rate (the SAME rate these two actions reused before this
    # feature existed) if config.yaml doesn't override it -- so a
    # problem with no tool: section at all keeps behaving exactly as
    # before.
    install_drain_rate = float(
        install_cfg.get("drain_rate", battery_cfg["idle_drain_rate"]))
    uninstall_drain_rate = float(
        uninstall_cfg.get("drain_rate", battery_cfg["idle_drain_rate"]))

    # tool.equipped.<cart|plow>.speed / .moving_drain_rate: the MoveTo
    # parameters used WHILE that tool is equipped (hitch(Tool,S) --
    # see basic_action_theory.pl's own walk_duration/3 and tool_
    # moving_drain_rate/2 notes), per this feature's own request.
    # tool_speed(free,_)/tool_moving_drain_rate(free,_) reuse motion.
    # speed/battery.moving_drain_rate directly -- there's no separate
    # config key for "no tool equipped", it's just the existing base
    # value under its own existing name.
    equipped_cfg = config.get("tool", {}).get("equipped", {})
    tool_speed_facts = _tool_moveto_param_facts(
        "tool_speed", "speed", equipped_cfg, config["motion"]["speed"])
    tool_moving_drain_rate_facts = _tool_moveto_param_facts(
        "tool_moving_drain_rate", "moving_drain_rate", equipped_cfg,
        battery_cfg["moving_drain_rate"])

    # tool.instances -- the tool INSTANCES this problem actually has, as
    # a list of {id, kind, x, y} entries, e.g.:
    #   tool:
    #     instances:
    #       - {id: cart1, kind: cart, x: 3.0, y: 2.075}
    #       - {id: cart2, kind: cart, x: 8.0, y: 4.0}
    #       - {id: plow1, kind: plow, x: 5.0, y: 2.075}
    # Multiple instances of the SAME kind are exactly the point (see
    # basic_action_theory.pl's own tool_instance/2, tool_position/4,
    # tools_of_kind/5, nearest_tool_of_kind/6) -- a BT tree's own
    # <InstallTool tool="..."> names ONE SPECIFIC instance id, not a
    # kind, so id must be unique per problem (kind need not be -- many
    # instances legitimately share one). OPTIONAL: a problem whose tree
    # never installs anything needs no tool.instances at all, same
    # "optional feature" treatment as tool.equipped/tool.install above.
    # Unlike every other tool.* numeric knob, there is NO sensible
    # default for a fixed physical location or a made-up id -- an
    # instance simply isn't emitted unless config.yaml states it in
    # full; a tree that references an unknown id just makes
    # tool_position/tool_instance fail for it, which makes
    # poss(start_install_tool(...)) fail too (the SAME "an unsatisfied
    # precondition, not a generation-time error" shape hitch(free,S)
    # already has).
    instances_cfg = config.get("tool", {}).get("instances", [])
    tool_instance_lines = []
    seen_ids = set()
    for entry in instances_cfg:
        tool_id = str(entry["id"]).strip()
        kind = str(entry["kind"]).strip()
        if not _VALID_PROLOG_ATOM_RE.match(tool_id):
            raise ValueError(
                f"tool.instances entry id {tool_id!r} is not a valid Prolog "
                f"atom -- must start with a lowercase letter, then letters/"
                f"digits/underscores only.")
        if tool_id in seen_ids:
            raise ValueError(f"tool.instances has more than one entry with id {tool_id!r}.")
        seen_ids.add(tool_id)
        if kind not in _TOOL_KINDS:
            raise ValueError(
                f"tool.instances entry {tool_id!r} has kind {kind!r}, not "
                f"one of {sorted(_TOOL_KINDS)}.")
        gx = _format_number(float(entry["x"]))
        gy = _format_number(float(entry["y"]))
        tool_instance_lines.append(f"tool_instance({tool_id}, {kind}).")
        tool_instance_lines.append(f"tool_start_position({tool_id}, {gx}, {gy}).")
    tool_position_facts = "\n".join(tool_instance_lines)

    # ploughing.cell_size -- the ONE knob basic_action_theory.pl's own
    # ploughed/3 fluent needs (cell_index/3's own CellSize argument),
    # deliberately its OWN independent grid, decoupled from disc_step_
    # position -- coarser on purpose (an approximation of the plow's
    # own physical width), which also means better cross-world proof
    # sharing, not worse, than disc_step_position's own finer grid would
    # give (more distinct noisy positions round to the SAME cell index).
    # OPTIONAL: a problem whose tree never installs a plow needs no
    # ploughing: section at all, same "optional feature" treatment as
    # tool.equipped/tool.install above -- NO sensible numeric default
    # though (there's nothing universal to default a physical width to,
    # same reasoning tool.instances above has for position), so a
    # ploughing: section with no cell_size key is a hard error rather
    # than silently picking a number.
    ploughing_cfg = config.get("ploughing", {})
    plough_cell_size_fact = (
        f"plough_cell_size({_format_number(float(ploughing_cfg['cell_size']))})."
        if "cell_size" in ploughing_cfg else "")

    # tool.install.range -- how close (metres) the robot's CURRENT
    # position must be to a tool's own tool_position(...) before
    # install_tool can start (basic_action_theory.pl reuses holds(
    # distance_below(GX,GY,Range),S) directly for this -- see that
    # predicate's own note and poss(start_install_tool(...))'s). ONE
    # value, not per-tool (matching install_tool_drain_rate/1's own
    # "specific to the ACTION, not the tool" shape above), defaulting to
    # safety_margin (robot_radius+safety_buffer) -- "close enough that
    # the robot's own body reaches it" is the least arbitrary default
    # available without a real robot/tool geometry model.
    install_range = float(install_cfg.get(
        "range", round(config["robot"]["radius"] + config["robot"]["safety_buffer"], 6)))

    lines = [
        "% AUTO-GENERATED by module/translators/config_to_prolog.py from",
        "% this problem's own config.yaml -- DO NOT HAND-EDIT, edit config.yaml",
        "% instead and regenerate (main.py does this automatically before",
        "% every run).",
        "",
        f"start({_format_number(config['initial_situation']['start_x'])},"
        f"{_format_number(config['initial_situation']['start_y'])}).",
        f"robot_radius({_format_number(config['robot']['radius'])}).",
        f"safety_buffer({_format_number(config['robot']['safety_buffer'])}).",
        f"speed({_format_number(config['motion']['speed'])}).",
        f"sigma({_format_number(position_cfg['lateral']['sigma'])}).",
        f"sigma_tangential({_format_number(position_cfg['tangential']['sigma'])}).",
        f"disc_step_position({_format_number(disc_step_position)}).",
        f"battery_enabled({str(bool(battery_enabled)).lower()}).",
        f"sigma_battery({_format_number(battery_cfg['sigma'])}).",
        f"battery_start({_format_number(battery_cfg['start'])}).",
        f"idle_drain_rate({_format_number(battery_cfg['idle_drain_rate'])}).",
        f"moving_drain_rate({_format_number(battery_cfg['moving_drain_rate'])}).",
        f"disc_step_battery({_format_number(disc_step_battery)}).",
        f"tolerance({_format_number(config['tolerances']['on_track'])}).",
        f"num_samples({_format_number(config['verification']['num_samples'])}).",
        f"bracket_samples({_format_number(config['verification']['bracket_samples'])}).",
        f"crossing_eps({_format_number(config['verification']['crossing_eps'])}).",
        f"disc_step_time({_format_number(disc_step_time)}).",
        "",
        z_block,
        "",
        zt_block,
        "",
        zbatt_block,
        "",
        sample_block,
        "",
        install_tool_duration_facts,
        uninstall_tool_duration_facts,
        f"install_tool_drain_rate({_format_number(install_drain_rate)}).",
        f"uninstall_tool_drain_rate({_format_number(uninstall_drain_rate)}).",
        f"install_tool_range({_format_number(install_range)}).",
        "",
        install_tool_result_block,
        "",
        uninstall_tool_result_block,
        "",
        tool_speed_facts,
        "",
        tool_moving_drain_rate_facts,
        "",
    ]
    # tool_position_facts is the one block above that can genuinely be
    # EMPTY (no tool.position.* in config.yaml at all) -- unlike its
    # siblings above (always non-empty, one line per _TOOL_KINDS entry
    # regardless of config), so it's appended separately, with its own
    # trailing blank line, ONLY when non-empty -- keeping this file's
    # existing "one block, one blank line" spacing convention instead of
    # emitting a stray double-blank when no tool position is configured.
    if tool_position_facts:
        lines += [tool_position_facts, ""]
    # plough_cell_size_fact is the OTHER block that can genuinely be
    # empty (no ploughing: section, or no cell_size key in it) --
    # same "append only when non-empty" treatment as tool_position_facts
    # just above, for the same spacing reason.
    if plough_cell_size_fact:
        lines += [plough_cell_size_fact, ""]

    # any_battery_depletion is basic_action_theory.pl's own ONLY query
    # about "finishing the battery" (see main.py's SUMMARY_QUERIES) --
    # emitted here, conditionally, rather than as a hardcoded fact in
    # Section 10 of basic_action_theory.pl, specifically so a problem
    # with battery.enabled: false can drop it without touching the
    # (problem-independent) theory file at all.
    if battery_enabled:
        lines += ["query(any_battery_depletion).", ""]

    return "\n".join(lines)


def generate(config_path=DEFAULT_CONFIG_PATH, output_path=DEFAULT_OUTPUT_PATH):
    config = load_config(config_path)
    text = render_prolog(config)
    with open(output_path, "w") as f:
        f.write(text)
    return output_path


if __name__ == "__main__":
    out = generate()
    print(f"Wrote {out}")
