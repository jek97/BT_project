#!/usr/bin/env bash
# perf_diag_two_hop/run.sh
#
# Reproduces the problem3 Bug0 performance investigation's key result:
# chaining TWO full evaluations of the tree (see two_hop_diag.pl)
# times out ProbLog's exact inference at the problem's default 5x5x5
# noise table (125 discrete noise combinations per hop), but succeeds
# at a reduced 3x1x1 table (3 combinations per hop) -- isolating the
# noise-table combinatorics, not the tree's own logic, as the driver.
#
# What this does, for EACH of the two noise configs in this directory:
#   1. Copies that config over problems/problem3/config.yaml.
#   2. Regenerates problems/problem3/config_generated.pl from it.
#   3. Strips module/theory/basic_action_theory.pl's own default
#      QUERIES section (its ~48 report queries pull in a different,
#      unrelated cost -- see two_hop_diag.pl's own header) and appends
#      two_hop_diag.pl in its place.
#   4. Runs `problog` against it with a 5-minute timeout.
#   5. Reverts all three files via `git checkout`, whatever happened.
#
# Usage (from anywhere):
#   bash perf_diag_two_hop/run.sh
#
# Refuses to run if basic_action_theory.pl / problem3's config.yaml /
# config_generated.pl already have uncommitted changes, so it can
# never clobber real work -- commit or stash first if it complains.

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."   # repo root

THEORY=module/theory/basic_action_theory.pl
CONFIG=problems/problem3/config.yaml
CONFIG_GEN=problems/problem3/config_generated.pl
DIAG_DIR=perf_diag_two_hop

for f in "$THEORY" "$CONFIG" "$CONFIG_GEN"; do
    if ! git diff --quiet -- "$f"; then
        echo "Refusing to run: $f has uncommitted changes -- commit or stash first." >&2
        exit 1
    fi
done

cleanup() {
    git checkout -- "$THEORY" "$CONFIG" "$CONFIG_GEN"
}
trap cleanup EXIT

run_one() {
    local label="$1" config_src="$2"

    echo "=================================================================="
    echo "  $label"
    echo "=================================================================="

    cp "$config_src" "$CONFIG"
    python3 -c "
import sys
sys.path.insert(0, 'module/translators')
from config_to_prolog import generate
generate('$CONFIG', '$CONFIG_GEN')
"

    # Replace the default QUERIES section (from its own "10. QUERIES"
    # header to end of file) with this diagnostic's single query.
    sed -i '/^% 10\. QUERIES$/,$d' "$THEORY"
    cat "$DIAG_DIR/two_hop_diag.pl" >> "$THEORY"

    set +e
    time BT_PROBLEM_DIR="$(pwd)/problems/problem3" timeout 300 problog "$THEORY"
    local code=$?
    set -e
    echo "---- exit code: $code (124 = hit the 5-minute timeout) ----"

    git checkout -- "$THEORY" "$CONFIG" "$CONFIG_GEN"
    echo
}

run_one "5x5x5 noise -- 125 combinations/hop (this problem's default)" "$DIAG_DIR/config_noise_555.yaml"
run_one "3x1x1 noise -- 3 combinations/hop (position only, rest deterministic)" "$DIAG_DIR/config_noise_3.yaml"
