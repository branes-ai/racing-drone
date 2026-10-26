# racing-drone

Branes.AI mission and flight controller for an autonomous racing drone: the integration target that assembles perception, planning, and optimal control from the Branes.AI platform into a vehicle that flies a race course on its own, as fast as physics allows.

## Vision

An autonomous racing drone is one of the hardest real-time embodied-AI problems that fits on a desk. It has to see the gates, know where it is, plan a time-optimal path, and track that path at the edge of its actuator limits. All of this happens on a small, power-limited flight computer, with latency budgets measured in milliseconds and no human in the loop to save it.

That makes it the ideal forcing function for the Branes.AI platform. Every weakness in perception latency, state-estimation accuracy, control bandwidth, or compute efficiency shows up directly as lap time or as a crash.

## How the platform fits together

The Branes.AI stack has four tiers. This repo sits in the top one:

```text
+--------------+ +-------------------+ +-----------+ +------------+
| racing-drone | | interceptor-drone | | isr-drone | | nano-drone | ...  Mission
+--------------+ +-------------------+ +-----------+ +------------+
        |                  |                 |              |
        v                  v                 v              v
+---------------------+ +---------------------+ +---------------------+
|    Branes Cortex    | |   Branes Synapse    | |    Branes Reflex    |  Platform
| perception (VIO,    | | tensor graph engine | | stabilization, MPC, |
| SLAM), VLA, world   | | for GGUF models,    | | state propagation,  |
| model, planning     | | static memory plan  | | fault recovery      |
+---------------------+ +---------------------+ +---------------------+
           |                       |                       |
           v                       v                       v
+---------------------------------+ +---------------------------------+
|              MTL5               | |            Universal            |  Math
| high-performance linear algebra | |       computer arithmetic       |
+---------------------------------+ +---------------------------------+
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
+-----------+ +-------------------+ +-------------------+ +-----------+
|    CPU    | |     CPU + GPU     | |     CPU + KPU     | |    KPU    |  Hardware
+-----------+ +-------------------+ +-------------------+ +-----------+
```

- **Mission**: one repo per vehicle and mission (`racing-drone`, `interceptor-drone`, `isr-drone`, `nano-drone`, ...). It owns the mission capability and composes platform components; algorithms do not live there.
- **Platform**: three sister repos, `cortex`, `synapse`, and `reflex`, that provide the functionality and services a mission is built from. None of them sits above the others.
- **Math**: [MTL5](https://github.com/stillwater-sc/mtl5) provides high-performance linear algebra and [Universal](https://github.com/stillwater-sc/universal) provides computer arithmetic (posits, intervals, mixed precision, hardware-aligned types).
- **Hardware**: the configurations the stack targets: CPU, CPU + GPU, CPU + KPU, and KPU. Today the stack runs on CPU; GPU and KPU support are planned, and the KPU is a stub until the silicon ships.

Arrows point from a tier to the tier below it that it builds on. They show how the tiers relate, not the dependencies of individual repos: each repo's build declares the libraries it actually uses.

The platform is modeled on the mammalian nervous system. This repo is the whole organism; the sister repos provide its brain, its reflexes, and the synapses that run its learned models:

| Repo | Biological analogue | What it provides to the racing drone | Time scale |
|---|---|---|---|
| [`cortex`](https://github.com/branes-ai/cortex) | Cerebral cortex | Perception (visual-inertial odometry, gate detection, SLAM), world model, path planning | ~5–100 Hz |
| [`synapse`](https://github.com/branes-ai/synapse) | The synapses: where signals are transmitted and computed | Execution of learned models (for example gate-detection or policy networks) as static, allocation-free compute graphs | Deterministic |
| [`reflex`](https://github.com/branes-ai/reflex) | Brainstem, spinal cord, autonomic system | Stabilization, trajectory tracking (PID → MPC → non-linear MPC), fault detection and recovery, safety envelopes | ~100 Hz–8 kHz, hard real-time |
| **`racing-drone`** (this repo) | The whole organism | Mission logic, vehicle integration, airframe and sensor configuration, simulation and flight testing | — |

```text
            +---------------------------------------------+
            |                racing-drone                 |
            |  mission manager: arm, race, abort, land    |
            +----------------------+----------------------+
                                   |
         +-------------------------+-------------------------+
         |                                                   |
+--------v---------+   state estimate + planned    +---------v--------+
|      cortex      |   trajectory (setpoints)      |      reflex      |
|  perception, VIO,+------------------------------>+  control loops,  |
|  gate detection, |                               |  safety, fault   |
|  path planning   |                               |  recovery        |
+--------^---------+                               +---------+--------+
         |                                                   |
   cameras, IMU                                   motor commands (ESCs)
```

`cortex` decides *where to fly*. `reflex` makes the airframe *follow that path* and keeps it safe if anything goes wrong. `racing-drone` owns the mission and wires the two to a real vehicle.

## Scope of this repo

- **Mission controller:** the race state machine (pre-flight checks, arm, launch, gate sequence, finish, land) and abort/failsafe policy
- **Vehicle integration:** airframe, motor/propeller, and sensor configuration; calibration data; the glue between `cortex` and `reflex` components
- **Simulation:** race-course SITL with realistic multirotor dynamics, camera rendering, and sensor noise for closed-loop development
- **Hardware-in-the-loop and flight testing:** HIL rigs, flight logs, and lap-time / tracking-error benchmarks
- **Performance accounting:** latency, energy, and compute budgets per pipeline stage on the target flight computer

Algorithms do not live here. Perception and planning go in `cortex`, control and safety go in `reflex`, and this repo composes them.

## Roadmap

The detailed plan is in [`docs/plan/implementation-plan.md`](docs/plan/implementation-plan.md), with requirements in [`docs/requirements/mission-requirements.md`](docs/requirements/mission-requirements.md). Phases 1–7 each end in a *baseline* (a named composition of components; see [`docs/arch/framework-architecture.md`](docs/arch/framework-architecture.md)) that runs in the scenario suite. Phases 0 and 8 are infrastructure phases with their own gates:

0. **Framework skeleton:** slots, data contracts, vehicle/course/scenario/baseline files, logging and replay.
1. **Simulation and B0:** `reflex` dynamics model and SITL; ground-truth state with cascaded PID.
2. **Time-optimal control (B1):** time-optimal planning in `cortex`, MPCC + INDI in `reflex`, still on ground-truth state.
3. **Vision-only flight (B2):** gate detection (`synapse` vision ops + `cortex`), gate PnP fused with `cortex` VIO; no external pose.
4. **FPV and pilot:** FPV video and OSD, human pilot modes, bumpless takeover, failsafes, HIL.
5. **Real hardware:** a staged flight-test progression up to the qualification speed.
6. **Detect and avoid (B4):** drone detection and tracking, MPC separation constraints, a CBF safety filter in `reflex`.
7. **Learned-control baseline (B3), optional:** an RL policy behind the same controller slot, benchmarked head-to-head against MPCC.
8. **Platform hardening:** reflex firmware replaces Betaflight on the FC, the hot path moves to the KPU, and motor-failure recovery is added.

Every phase has an acceptance gate in [`docs/plan/test-plan.md`](docs/plan/test-plan.md) §6.

## Building and testing

`racing-drone` is C++20 and builds with CMake ≥ 4.0 and Ninja, on GCC, Clang, and MSVC, using the same presets as the sibling repos.

```bash
cmake --preset gcc-debug              # also: gcc-release, clang-debug, clang-release, msvc, msvc-ninja, ...
cmake --build --preset gcc-debug
ctest --preset gcc-debug
```

Every baseline in [`baselines/`](baselines/) becomes four executables: `flight_stack_<name>`, `replay_<name>`, `sitl_<name>`, and `sitl_batch_<name>` ([ADR 0001](docs/adr/0001-baseline-composition-framework.md)). With baseline B0:

```bash
B=build/gcc-debug/apps
$B/sitl_b0-gt-pid --scenario scenarios/b0-determinism.yaml --out run.brlog    # simulate, log every input and output
$B/score_log --log run.brlog --speed-limit 5                                    # score the run: laps, gate errors, failures
$B/replay_b0-gt-pid --log run.brlog --out replay.brlog --verify                 # reproduce the outputs record for record
$B/sitl_batch_b0-gt-pid --scenario scenarios/b0-nominal.yaml --seeds 10 --min-success 1   # many seeds, as the gates run
$B/flight_stack_b0-gt-pid --course courses/a2rl-2025.yaml --vehicle vehicles/rp1.yaml --cycles 10
```

In the Visual Studio solution (`msvc` preset), the harnesses are under `Baselines/<name>` and `score_log` is under `Tools`. Each starts in the source root, and the debugger arguments are filled in from the baseline's `DEBUG_SCENARIO` ([`apps/CMakeLists.txt`](apps/CMakeLists.txt)). They differ from the commands above:

| Target | Debugger arguments |
|---|---|
| `sitl_<name>` | `--scenario <DEBUG_SCENARIO> --out <build>/<name>.brlog` |
| `replay_<name>` | `--log <build>/<name>.brlog --out <build>/<name>-replay.brlog --verify` |
| `sitl_batch_<name>` | `--scenario <DEBUG_SCENARIO> --seeds 1 --jobs 1` (one seed on one thread, for stepping) |
| `flight_stack_<name>` | `--course courses/a2rl-2025.yaml --vehicle vehicles/rp1.yaml --cycles 10` |
| `score_log` | `--log <build>/b0-gt-pid.brlog` |

Run `sitl_<name>` first: `replay_<name>` and `score_log` read the log it writes.

## Repository layout

```text
racing-drone/
├── apps/         # harness sources (flight_stack, replay, sitl, sitl_batch, score), built once per baseline
├── baselines/    # one YAML per baseline: slot composition + parameters
├── cmake/        # deps, warnings, slot registry, baseline target generation
├── courses/      # course files (gates, sequence, laps, geofence)
├── docs/         # requirements, assessments, architecture, plans, ADRs
├── scenarios/    # SITL scenarios (course + vehicle + duration + seed)
├── sdk/          # branes::racing: data contracts, slots, Stack, config loaders, log, harness pieces
├── tests/        # Catch2 unit tests, baseline tests, phase gates
└── vehicles/     # vehicle files (mass, inertia, rotors, drag, IMU, camera)
```

## Status

Phase 0 (framework skeleton) is in place: the data contracts and the seven slot concepts; vehicle, course, scenario, and baseline files with validating loaders; the replay log; and the three harnesses, all running the `null` baseline. Gate P0 ([test plan §6](docs/plan/test-plan.md)) runs in CI: SITL → replay twice → byte-identical logs. Phase 1 has started: SITL now closes the loop on the `reflex` dynamics model, with a simulated flight controller, IMU and fault models, an oracle gate projector, and a scoreboard (`sim/`, [plan](docs/plan/sitl/README.md)). The first real baseline, B0 `gt-pid` (ground truth, waypoint planner, the reflex cascade; [plan](docs/plan/b0/README.md)), flies the course: gate P1 runs 10 seeds on every PR and 500 seeds nightly.

## License

MIT. See [LICENSE](LICENSE).
