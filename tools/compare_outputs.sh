#!/usr/bin/env bash
# compare_outputs.sh <build-dir> <out-dir>
#
# Runs the harnesses on fixed inputs and writes every output to <out-dir>, so
# two builds (before and after a refactoring that must not change behavior) can
# be compared byte for byte:
#
#   tools/compare_outputs.sh build/gcc-release /tmp/before    # on main
#   tools/compare_outputs.sh build/gcc-release /tmp/after     # on the branch
#   diff -r /tmp/before /tmp/after                            # must be empty
#
# Run from the source root. Used by the framework split (docs/plan/framework-split).
set -euo pipefail

if [ $# -ne 2 ]; then
    echo "usage: $0 <build-dir> <out-dir>" >&2
    exit 2
fi
B="$1/apps"
O="$2"
rm -rf "$O"
mkdir -p "$O"

# SITL logs, and their verified replays.
for run in null:null-smoke scripted-hop:scripted-hop b0-gt-pid:b0-determinism; do
    name=${run%%:*}
    scenario=${run##*:}
    "$B/sitl_$name" --scenario "scenarios/$scenario.yaml" --out "$O/$name.brlog" > "$O/$name.sitl.txt"
    "$B/replay_$name" --log "$O/$name.brlog" --out "$O/$name.replay.brlog" --verify > "$O/$name.replay.txt"
done

# The B0 log, scored and dumped.
"$B/score_log" --log "$O/b0-gt-pid.brlog" --speed-limit 5 > "$O/b0-gt-pid.score.txt"
"$B/log_dump" --log "$O/b0-gt-pid.brlog" --out-dir "$O/b0-gt-pid.dump" > "$O/b0-gt-pid.dump.txt"

# Many seeds, and the precision sweep.
"$B/sitl_batch_b0-gt-pid" --scenario scenarios/b0-nominal.yaml --seeds 20 > "$O/b0-gt-pid.batch.txt"
"$B/precision_sweep_b0-gt-pid" --scenario scenarios/b0-nominal.yaml --seeds 20 --out-dir "$O/b0-gt-pid.sweep" > "$O/b0-gt-pid.sweep.txt"

# The harnesses print their output paths; name the directory, so the text
# compares equal across runs into different directories.
sed -i "s|$O|<out>|g" "$O"/*.txt

echo "compare_outputs: wrote $(find "$O" -type f | wc -l) files to $O"
