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

The racing drone grows alongside the control stack in `reflex`:

1. **Simulated hover and waypoint flight:** cascaded PID from `reflex`, ground-truth state, closed loop in SITL.
2. **Estimated-state flight:** replace ground truth with `cortex` visual-inertial odometry; fly a fixed gate course in simulation.
3. **Gate-aware racing:** gate detection and minimum-time trajectory planning in `cortex`, linear MPC tracking in `reflex`.
4. **Aggressive racing:** non-linear MPC flying near actuator saturation to push lap times toward the time-optimal limit.
5. **Real hardware:** HIL, then tethered and free flight on a physical racing airframe with the Branes.AI flight computer.
6. **Robustness:** motor-loss and sensor-fault recovery from `reflex`, so the drone degrades gracefully instead of crashing.

## Status

Early stage. The repository has just been created. Build system, simulation harness, and first integration with `cortex` and `reflex` will follow.

## License

MIT. See [LICENSE](LICENSE).
