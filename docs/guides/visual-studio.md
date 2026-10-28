# Working through the stack in Visual Studio

This guide works through what the repo does today, in Visual Studio 2022 on Windows: one closed loop, its log, many seeds, the precision sweep, and the tests. Each step covers what to run, where to break, what to look for, and what it tells you.

The solution is set up for it. Every harness target starts in the source root, with debugger arguments that run something meaningful on F5 (the debugger-arguments table in the [README](../../README.md#building-and-testing)).

## 0. Configure and build

The project needs CMake ≥ 4.0, and the CMake bundled with Visual Studio 2022 is 3.x. So configure from a terminal with CMake 4 on `PATH`, then open the generated solution:

```
cmake --preset msvc
start build\msvc\branes_racing.sln
```

1. Build the `Debug` configuration: Build Solution, or `cmake --build --preset msvc-debug`.
2. Run `ctest --preset msvc-debug`. Every test should pass before you start; that is your baseline.

In Solution Explorer:
- the harnesses are under `Baselines/<name>`: `sitl_`, `sitl_batch_`, `replay_`, `flight_stack_`, and for B0 `precision_sweep_`;
- `score_log` and `log_dump` are under `Tools`;
- the tests are under `Tests/<layer>`.

## 1. One closed loop: `sitl_b0-gt-pid`

**Start here.** Set `sitl_b0-gt-pid` as the startup project and press F5.
- It flies baseline B0 (ground-truth state, waypoint planner, cascaded PID; [B0 plan](../plan/b0/README.md)) on `scenarios/b0-determinism.yaml`.
- That scenario is 8 s long, shorter than two laps, so it's quick to step through.
- It writes `build/msvc/b0-gt-pid.brlog`.

Breakpoints along one tick's path, in order:

| Where | What you see |
|---|---|
| `sim/include/branes/racing/sim/simulator.hpp`, `Simulator::next()` | The simulator, in `double`: the reflex dynamics, the IMU and the truth, on integer-nanosecond time |
| `sdk/include/branes/racing/harness/harness.hpp`, `Runner::feed` | The harness boundary. Records are decoded and cast to the stack's scalar type T, and outputs are cast back to `double` |
| `sdk/include/branes/racing/compose/stack.hpp`, `Stack::on_tick` | The composition, one call per slot: estimator → planner → controller → safety filter → FC bridge |
| `sdk/include/branes/racing/slots/ground_truth.hpp` | B0's estimator: the true state handed through, without a filter (SITL only) |
| `sdk/include/branes/racing/slots/waypoint.hpp`, `plan()` | The path through the gate centers: the lookahead point, and the corner slowdown |
| `baselines/glue/include/branes/racing/glue/cascaded_pid.hpp`, `step()` | The position and attitude loops. dt comes from `TimeBase::seconds` ([time-base plan](../plan/time-base/README.md)) |
| `sim/include/branes/racing/sim/simulator.hpp`, `command()` | The command going back to the simulated flight controller |

What to look for:
- the state `x.position` closing on the reference `r.position`;
- the velocity setpoint staying under `max_speed_m_s` (`baselines/b0-gt-pid.yaml`);
- the thrust staying inside the cascade's limits;
- `time_base().rel_tol()` holding the controller's dt accuracy, which it derives from its gains.

The classes check their invariants and preconditions with `assert`, so a Debug build stops where an asserted one fails. Setup is checked differently: a bad config, course or scalar type is a returned error, reported before the run starts. `TimeBase`'s type requirements are `static_assert`s, so they fail at compile time.

## 2. The run's result: `score_log`, `log_dump`, `replay_b0-gt-pid`

Run these on the log from step 1.

| Target | What it tells you |
|---|---|
| `score_log` | One JSON line: success, laps, gate errors (true and estimated), collision, geofence breach, max speed, and the first failure. Here the run fails because the scenario is short. Every baseline is scored the same way, so the scoreboard compares like with like |
| `log_dump` | CSV files in `build/msvc/b0-gt-pid-dump/`, one per channel: `truth`, `state`, `reference`, `command`, … Plot `truth.csv` x/y against `reference.csv` to see the path flown against the planner's target |
| `replay_b0-gt-pid` | With `--verify`, it re-runs the stack on the log's inputs and checks that the outputs match bit for bit. A SITL run can be replayed and inspected later from its log, without the simulator. Break in `Runner::feed` during a replay and you see the same ticks |

## 3. Many seeds: `sitl_batch_b0-gt-pid`

The default arguments run one seed on one thread, for stepping. For gate P1 ([test plan](../plan/test-plan.md) §6), select `sitl_batch_b0-gt-pid` in the Release configuration, and set its Debugger Command Arguments (project Properties → Debugging) to:

```
--scenario scenarios/b0-nominal.yaml --seeds 500 --min-success 0.99
```

Or run it from the source root in a terminal:

```
build\msvc\apps\Release\sitl_batch_b0-gt-pid.exe --scenario scenarios/b0-nominal.yaml --seeds 500 --min-success 0.99
```

Each seed randomizes mass, inertia and drag. The summary gives the success rate, the lap-time and gate-error percentiles, the max speed, and the failures. A batch writes no log: it is reproducible from its seeds and arguments instead. A run is reproducible on its own:
- `--first-seed <s> --seeds 1` reruns seed s;
- or set `seed:` in a scenario copy for `sitl_b0-gt-pid`, to step through it.

## 4. Precision: `precision_sweep_b0-gt-pid`

The sweep ([plan](../plan/precision-sweep/README.md)) runs the same stack in each type of `sim/include/branes/racing/sim/sweep_types.hpp`: `double`, `float`, `cfloat<32,8>`, `posit<32,2>`, bfloat16, binary16 and `posit<16,1>`. The simulator, FC, scorer and log stay in `double`.

- **To debug:** keep the 1-seed default, and break in `apps/precision_sweep/main.cpp`, `sweep_type<T>`, which is called once per type. Step into `run_one` to watch the stack in T. Universal's types show their values in the debugger.
- **To reproduce the results:** select `precision_sweep_b0-gt-pid` in the Release configuration, and set its Debugger Command Arguments to `--scenario scenarios/b0-nominal.yaml --seeds 500 --out-dir <dir>`. Or run `build\msvc\apps\Release\precision_sweep_b0-gt-pid.exe` with those arguments from the source root. The software types make this the slowest run in the repo.
  - **Comparing with the committed results:** the committed `docs/assessments/data/precision-sweep-b0.md` was generated with two more flags. `--commit` records the commit in the header, and `--timing` adds a wall-time column. Its "Command" and "Commit" lines name them.
  - Compare the result columns of `<dir>/summary.md`, and the files `gate_error.jsonl` and `lap_delta.jsonl`, with the committed ones. The header lines and wall times differ by construction.

What it tells you is in the [mixed-precision assessment](../assessments/mixed-precision.md) §9:
- which types construct the stack;
- which pass P1;
- how far each lands from `double`, per lap and per gate.

## 5. The tests: Test Explorer

The CMake integration lists every Catch2 case. Each `.cpp` under `tests/<layer>/` is its own executable. To debug one case, set the test's target as the startup project and give the case name, in quotes, as the debugger argument.

| Group | What it tells you |
|---|---|
| `tests/sdk/` | The message contracts, config loading, the log format, and the stack's composition |
| `tests/sim/` | The simulator, the scorer, and B0's controller: it holds on a bad dt, commands m·g at equilibrium, keeps thrust in its limits, and derives its dt accuracy from its gains |
| `tests/precision/` | Every sweep type: the casts round to nearest, the tolerances are derived from T, `TimeBase`'s bounds, and the ULP at each gate |
| `gate.*`, `sweep.b0.smoke`, `tools.log_dump` | End to end: logs and replays are deterministic, P1 passes in its smoke form, a sweep rerun is byte-identical |
| `compile_fail.time_base.narrow_wide` | Builds code that must not compile, and checks that it fails on the intended `static_assert` |

## 6. Experiments

- **The speed cap.** Change `max_speed_m_s` in `baselines/b0-gt-pid.yaml`, rebuild, and compare the `sitl_batch` summaries. Lap time against gate error is the basic trade-off of the controller.
- **The seed.** Rerun one failing or slow seed of a batch with `--first-seed`, and step through it in `sitl_b0-gt-pid`.
- **[#44](https://github.com/branes-ai/racing-drone/issues/44), open.** `posit<16,1>`'s gate error over `double` grows along each straight. To start on it:
  1. Run the sweep with one `b0-nominal` seed.
  2. Break in `WaypointPlanner::plan` and in the horizontal speed-cap scaling of reflex's `control/position.hpp` (fetched into `build/msvc/_deps/reflex-src`).
  3. Watch the lateral velocity setpoint, `v_sp(1)`, along the first straight.

## What to take away

- **Layers:** the simulator, FC, scorer and log run in `double` and are the ground truth. The stack runs in its scalar type T, behind casts at the harness boundary.
- **Reproducibility:** every run is deterministic. A SITL run is replayable from its log; batch and sweep runs write no log, and are reproducible from their seeds and arguments.
- **Contracts:** every layer has contracts you can watch hold, or fail, in the debugger.
