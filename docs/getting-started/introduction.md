# Introduction

`racing-drone` is the Branes.AI mission and flight controller for an autonomous racing drone: the integration target that assembles perception, planning, and optimal control from the Branes.AI platform into a vehicle that flies a race course on its own, as fast as physics allows.

## Why a racing drone

An autonomous racing drone is one of the hardest real-time embodied-AI problems that fits on a desk. It has to see the gates, know where it is, plan a time-optimal path, and track that path at the edge of its actuator limits. All of this happens on a small, power-limited flight computer, with latency budgets measured in milliseconds and no human in the loop to save it.

That makes it the ideal forcing function for the Branes.AI platform. Every weakness in perception latency, state-estimation accuracy, control bandwidth, or compute efficiency shows up directly as lap time or as a crash. The history of the field, and what it taught, is in [autonomous drone racing](../assessments/autonomous-drone-racing.md).

## How the platform fits together

The Branes.AI stack has five tiers. This repo sits in the top one:

- **Mission**: one repo per vehicle and mission. `racing-drone` and `scout-drone` are the two that stand up the architecture; more follow (`interceptor-drone`, `isr-drone`, `nano-drone`, `counter-uas`). A mission repo owns its task: its world, the slots that see it, its stack and scorer, and its data. Algorithms do not live there.
- **Mission framework**: [`anatomy`](https://github.com/branes-ai/anatomy), the body plan every mission composes its vehicle with: the data contracts, the slot concepts and their composition, configs, the replay log, the harness, and the closed-loop simulator. It holds no algorithms and no mission.
- **Platform**: three sister repos, `cortex`, `synapse`, and `reflex`, that provide the functionality and services a mission is built from. None of them sits above the others.
- **Math**: [MTL5](https://github.com/stillwater-sc/mtl5) provides high-performance linear algebra and [Universal](https://github.com/stillwater-sc/universal) provides computer arithmetic (posits, intervals, mixed precision, hardware-aligned types).
- **Hardware**: CPU, CPU + GPU, CPU + KPU, and KPU. Today the stack runs on CPU; GPU and KPU support are planned.

The platform is modeled on the mammalian nervous system. This repo is the whole organism; the sister repos provide its brain, its reflexes, and the synapses that run its learned models:

| Repo | Biological analogue | What it provides to the racing drone | Time scale |
|---|---|---|---|
| [`cortex`](https://github.com/branes-ai/cortex) | Cerebral cortex | Perception (visual-inertial odometry, gate detection, SLAM), world model, path planning | ~5–100 Hz |
| [`synapse`](https://github.com/branes-ai/synapse) | The synapses: where signals are transmitted and computed | Execution of learned models (for example gate-detection or policy networks) as static, allocation-free compute graphs | Deterministic |
| [`reflex`](https://github.com/branes-ai/reflex) | Brainstem, spinal cord, autonomic system | Stabilization, trajectory tracking (PID → MPC → non-linear MPC), fault detection and recovery, safety envelopes | ~100 Hz–8 kHz, hard real-time |
| **`racing-drone`** (this repo) | The whole organism | Mission logic, vehicle integration, airframe and sensor configuration, simulation and flight testing | — |

`cortex` decides *where to fly*. `reflex` makes the airframe *follow that path* and keeps it safe if anything goes wrong. `racing-drone` owns the mission and wires the two to a real vehicle. The full diagrams are in the [README](../../README.md#how-the-platform-fits-together).

## What this repo owns

- **Mission controller:** the race state machine (pre-flight checks, arm, launch, gate sequence, finish, land) and abort/failsafe policy.
- **Vehicle integration:** airframe, motor/propeller, and sensor configuration; calibration data; the glue between `cortex` and `reflex` components.
- **Simulation:** race-course SITL with realistic multirotor dynamics, camera rendering, and sensor noise for closed-loop development.
- **Hardware-in-the-loop and flight testing:** HIL rigs, flight logs, and lap-time / tracking-error benchmarks.
- **Performance accounting:** latency, energy, and compute budgets per pipeline stage on the target flight computer.

Algorithms do not live here. Perception and planning go in `cortex`, control and safety go in `reflex`, and this repo composes them.

## Baselines

A racing drone here is a *baseline*: a named composition of components in fixed slots ([ADR 0001](../adr/0001-baseline-composition-framework.md)). The [implementation plan](../plan/implementation-plan.md) builds them in phases, from B0 (ground-truth state, cascaded PID) to vision-only flight, detect-and-avoid, and a learned-control comparison. Each phase ends in a gate in the [test plan](../plan/test-plan.md) §6.

Next: [build and test](build-and-test.md).
