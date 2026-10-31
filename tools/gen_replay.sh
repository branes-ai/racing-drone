#!/usr/bin/env bash
# gen_replay.sh [preset]
#
# Records the docs site's race replay (docs/guides/mission-replay.mdx): one
# SITL run of baseline B0 on scenarios/b0-nominal.yaml, exported by
# trace_export as reflex-trace/1 to docs-site/src/traces/b0-nominal.json,
# stamped with `git describe` of this tree. The preset defaults to
# gcc-release. Commit the trace with the change that produced it:
#
#   tools/gen_replay.sh
#   git add docs-site/src/traces/b0-nominal.json
#
# Run from the source root.
set -euo pipefail

preset=${1:-gcc-release}
scenario=scenarios/b0-nominal.yaml
out=docs-site/src/traces/b0-nominal.json

cmake --preset "$preset" >/dev/null
cmake --build --preset "$preset" --target sitl_b0-gt-pid trace_export
commit=$(git describe --always --dirty --abbrev=7 --exclude '*')
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
"build/$preset/apps/sitl_b0-gt-pid" --scenario "$scenario" --out "$work/run.brlog"
mkdir -p "$(dirname "$out")"
"build/$preset/apps/trace_export" --log "$work/run.brlog" --scenario "$scenario" --out "$out" --commit "$commit"
