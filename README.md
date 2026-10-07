# racing-drone

Branes.AI mission and flight controller for an autonomous racing drone: the integration target that assembles perception, planning, and optimal control from the Branes.AI platform into a vehicle that flies a race course on its own, as fast as physics allows.

## Vision

An autonomous racing drone is one of the hardest real-time embodied-AI problems that fits on a desk. It has to see the gates, know where it is, plan a time-optimal path, and track that path at the edge of its actuator limits. All of this happens on a small, power-limited flight computer, with latency budgets measured in milliseconds and no human in the loop to save it.

That makes it the ideal forcing function for the Branes.AI platform. Every weakness in perception latency, state-estimation accuracy, control bandwidth, or compute efficiency shows up directly as lap time or as a crash.

## How the platform fits together

The Branes.AI platform is modeled on the mammalian nervous system. This repo is the whole organism; the sister repos provide its brain and its reflexes:

| Repo | Biological analogue | What it provides to the racing drone | Time scale |
|---|---|---|---|
| [`cortex`](https://github.com/branes-ai/cortex) | Cerebral cortex | Perception (visual-inertial odometry, gate detection, SLAM), world model, path planning | ~5–100 Hz |
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
