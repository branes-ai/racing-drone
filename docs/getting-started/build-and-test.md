# Build & Test

`racing-drone` is C++20 and builds with CMake ≥ 4.0 and Ninja, on GCC, Clang, and MSVC, using the same presets as the sibling repos.

## Build and run the tests

```bash
cmake --preset gcc-debug              # also: gcc-release, clang-debug, clang-release, msvc, msvc-ninja, ...
cmake --build --preset gcc-debug
ctest --preset gcc-debug
```

## Fly a baseline

Every baseline in [`baselines/`](../../baselines/) becomes four executables: `flight_stack_<name>`, `replay_<name>`, `sitl_<name>`, and `sitl_batch_<name>` ([ADR 0001](../adr/0001-baseline-composition-framework.md)). B0 also gets `precision_sweep_b0-gt-pid`, which runs its stack in each scalar type of the [precision sweep](../plan/precision-sweep/README.md). With baseline B0:

```bash
B=build/gcc-debug/apps
$B/sitl_b0-gt-pid --scenario scenarios/b0-determinism.yaml --out run.brlog    # simulate, log every input and output
$B/score_log --log run.brlog --speed-limit 5                                    # score the run: laps, gate errors, failures
$B/replay_b0-gt-pid --log run.brlog --out replay.brlog --verify                 # reproduce the outputs record for record
$B/log_dump --log run.brlog --out-dir run                                       # every channel as CSV, for plotting
$B/sitl_batch_b0-gt-pid --scenario scenarios/b0-nominal.yaml --seeds 10 --min-success 1   # many seeds, as the gates run
$B/precision_sweep_b0-gt-pid --scenario scenarios/b0-nominal.yaml --seeds 10 --out-dir sweep  # the stack in float, posits, bfloat16, ...
$B/flight_stack_b0-gt-pid --course courses/a2rl-2025.yaml --vehicle vehicles/rp1.yaml --cycles 10
```

Run `sitl_<name>` first: `replay_<name>`, `score_log`, and `log_dump` read the log it writes.

## Visual Studio

With the `msvc` preset, the harnesses are under `Baselines/<name>` in the solution, and `score_log` and `log_dump` are under `Tools`. Each starts in the source root with debugger arguments filled in (the table in the [README](../../README.md#building-and-testing)). [Working through the stack in Visual Studio](../guides/visual-studio.md) walks through one closed loop, its log, many seeds, the precision sweep, and the tests.

## The docs site

This site is generated from [`docs/`](../README.md). To preview it:

```bash
cd docs-site
npm ci
npm run dev        # regenerates the pages from docs/, then serves them
```

Next: [repository layout](repository-layout.md).
