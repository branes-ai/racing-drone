# Racing drone mission requirements

**Status:** Draft
**Date:** 2026-10-23
**Related:** [state of the art](../assessments/state-of-the-art.md), [framework architecture](../arch/framework-architecture.md), [implementation plan](../plan/implementation-plan.md), [test plan](../plan/test-plan.md)

## 1. Mission statement

Fly a known gate course autonomously, as fast as a world-class human FPV pilot or faster, using only onboard sensing and compute. There is no motion capture, GNSS, UWB, or other external pose measurement. The same vehicle must also support:

- a live FPV video channel;
- a human pilot who can fly it manually, supervise autonomy, and take over at any time;
- detect-and-avoid (DAA) of other drones and unexpected obstacles on the course.

The mission is also a framework. Every algorithm sits behind an interface, so a *baseline* (one named composition of estimator, detector, planner, controller, DAA, and pilot arbitration) can be swapped, compared, and extended without touching the rest of the stack.

## 2. Reference mission profile

The profile is modeled on the Abu Dhabi Autonomous Racing League (A2RL) drone championships (2025 and 2026). These are the current public benchmark for vision-only racing.

| Parameter | Reference value | Source / rationale |
|---|---|---|
| Venue | Indoor hall, about 76 × 18 × 5.4 m flight volume | MonoRace paper, A2RL 2025 track ([survey \[2\]](../assessments/state-of-the-art.md#sources)) |
| Course | 11 gates, two laps, gate inner opening 1.5 m | MonoRace paper ([survey \[2\]](../assessments/state-of-the-art.md#sources)) |
| Gate map | Nominal gate poses known in advance. As-built poses may differ by up to ±0.5 m and ±10° | Assumption: real courses are set up by hand |
| Top speed | 28 m/s (≈100 km/h), the MonoRace peak | MonoRace paper ([survey \[2\]](../assessments/state-of-the-art.md#sources)) |
| Peak acceleration | ≈7 g during maneuvers | MonoRace paper ([survey \[2\]](../assessments/state-of-the-art.md#sources)). The ±16 g accelerometer still saturated, which points to vibration peaks on top of the maneuver load |
| Lap-time benchmark | Two-lap completion 16.56 s (MonoRace best, A2RL 2025); fastest lap 12.03 s (TII Racing, A2RL 2026) | MonoRace paper, results section ([survey \[2\]](../assessments/state-of-the-art.md#sources); <https://arxiv.org/html/2601.15222>, accessed 2026-10-23); A2RL 2026 coverage ([survey \[6\]](../assessments/state-of-the-art.md#sources)) |
| Allowed sensors (league rules) | One forward-facing monocular RGB camera and one IMU. No stereo, LiDAR, external compute, or human RC during autonomous heats | As reported in A2RL 2026 coverage ([survey \[6\]](../assessments/state-of-the-art.md#sources), accessed 2026-10-23). **The official rulebook has not been located; obtaining it is an open action (§7)** |
| Race formats | Single-drone time trial; multi-drone head-to-head; AI vs. human pilot | A2RL |
| Lighting | Indoor event lighting, with flicker, glare, and LED gate markers possible | Assumption |
| RF environment | Crowded 2.4 / 5.8 GHz spectrum; camera EMI observed (8–75 % of frames corrupted on some MonoRace flights) | MonoRace paper |

**Constraint for the framework:** the A2RL sensor rules are the *minimum* sensor set. Every baseline must be able to fly with one mono camera and an IMU. Baselines for other leagues or missions may add sensors (stereo, ToF, event camera, radar), but only through the same sensor interfaces.

## 3. Reference vehicle (RP-1, "A2RL-class")

Requirements are written against this vehicle. The framework must not hard-code it. Every value lives in a vehicle description file.

| Item | RP-1 value |
|---|---|
| Airframe | 5-inch-class quadrotor, ≈1 kg all-up (A2RL 2025 drone: 966 g, 5.1-inch props) |
| Flight controller (FC) | STM32H7-class MCU (A2RL 2025: STM32H743, 480 MHz) |
| Companion computer | NVIDIA Jetson Orin NX class (A2RL 2025), or a Branes KPU when available |
| Navigation camera | Rolling-shutter CMOS, about 820 × 616 at 90 Hz, wide FOV (A2RL 2025: IMX219, 155° × 115°) |
| IMU | Gyro ≥ 2 kHz, accelerometer ≥ 1 kHz. A ±16 g accelerometer saturates in flight, so saturation must be handled (EST-07) |
| ESCs | DShot 600 or faster, bidirectional (RPM telemetry) |
| FPV video | Separate low-latency digital or analog VTX with its own camera (see FPV-01) |
| RC link | ExpressLRS / CRSF |

## 4. Operating modes

| Mode | Who commands what | Safety layer active |
|---|---|---|
| `DISARMED` | Nothing spins | — |
| `MANUAL` (acro) | Pilot sticks → body rates and thrust → FC rate loop | FC failsafes |
| `ASSISTED` | Pilot sticks → setpoints; autonomy enforces the envelope and DAA | FC + reflex safety filter |
| `AUTONOMOUS` | Mission → planner → MPC → FC; the pilot supervises | FC + reflex safety filter + DAA |
| `HOLD` / `LAND` | Reflex degradation ladder | FC failsafes |
| `KILL` | Motors stop at once (hardware path on FC) | — |

## 5. Requirements

Priority: **M** = must, **S** = should, **C** = could. Verification: **T** = test, **A** = analysis, **I** = inspection, **D** = demonstration. *Owner* is the repo the capability belongs to; `racing-drone` composes and integrates it.

### 5.1 Mission and system (SYS)

| ID | Requirement | Pri | Ver | Owner |
|---|---|---|---|---|
| SYS-01 | Complete a known gate course autonomously, all gates in order, using only onboard sensors and compute | M | T, D | racing-drone |
| SYS-02 | Use no external pose source (mocap, GNSS, UWB, beacons) in any flight mode | M | I | all |
| SYS-03 | Fly with the A2RL minimum sensor set: one forward mono RGB camera and one IMU | M | T | cortex |
| SYS-04 | Load a course description (gate poses, sizes, order, laps, and the flight-volume geofence) from a file, with no recompile | M | T | racing-drone |
| SYS-05 | Load a vehicle description (mass, inertia, rotor and motor model, drag, camera intrinsics and extrinsics, IMU noise) from a file | M | T | racing-drone |
| SYS-06 | Select the active baseline (the composition of components) from a file | M | T | racing-drone |
| SYS-07 | Run the same baseline unchanged in SITL, HIL, and flight | M | T | all |
| SYS-08 | Record every sensor input, internal estimate, and command with timestamps, so any flight can be replayed deterministically | M | T | racing-drone |
| SYS-09 | Two-lap success rate ≥ 90 % in SITL under the nominal randomization set (TP §6), and ≥ 80 % in real flight at the qualification speed | M | T | racing-drone |
| SYS-10 | Lap time within 10 % of the time-optimal reference for the vehicle model (SITL), with a stretch goal of beating the A2RL benchmark on an equivalent course | S | T | racing-drone |

### 5.2 Perception and state estimation (EST)

| ID | Requirement | Pri | Ver | Owner |
|---|---|---|---|---|
| EST-01 | Estimate the full state (position, velocity, attitude, angular rate, IMU biases) in the course frame at ≥ the controller rate | M | T | cortex / reflex |
| EST-02 | Detect gates in the navigation camera and extract the four inner corners per visible gate at the camera frame rate | M | T | cortex |
| EST-03 | Correct VIO drift with gate observations (PnP against the course map, joint over multiple gates when visible) | M | T | cortex |
| EST-04 | Position error at the gate plane, 3σ ≤ 0.30 m, at speeds up to the qualification speed (budget in §6.2) | M | T, A | cortex |
| EST-05 | Estimator is consistent: average NEES within the 95 % χ² bounds on the replay suite | M | T | cortex |
| EST-06 | Compensate measurement delay. The estimate handed to the controller is valid at the current time, not the image timestamp | M | T | reflex |
| EST-07 | Survive accelerometer saturation, for example by switching to model-predicted specific force (drag and thrust model) when the IMU saturates | M | T | cortex / reflex |
| EST-08 | Survive camera corruption or dropout: at least 50 % corrupted frames over a lap, and ≥ 1 s total blackout, without losing the course | M | T | cortex |
| EST-09 | Model the rolling shutter, either in the gate PnP or through per-row timestamps | S | T | cortex |
| EST-10 | Refine camera–IMU extrinsics and estimator parameters offline from flight logs against the known gate geometry | S | T | cortex / racing-drone |
| EST-11 | Detect the race-start pose relative to the course, from the first gate in view or a known start pad | M | T | cortex |
| EST-12 | Run the gate-detection network on the companion within its latency budget (§6.1) | M | T | synapse |

### 5.3 Planning and control (CTL)

| ID | Requirement | Pri | Ver | Owner |
|---|---|---|---|---|
| CTL-01 | Generate a (near) time-optimal reference through the gate sequence that respects single-rotor thrust limits and body-rate limits | M | T | cortex |
| CTL-02 | Replan online when the estimated gate poses differ from the nominal map | S | T | cortex |
| CTL-03 | Track the reference with a nonlinear MPC on the full rigid-body model. The formulation is MPCC or tracking NMPC; actuator constraints are explicit | M | T | reflex |
| CTL-04 | MPC rate ≥ 100 Hz, with a horizon ≥ 0.5 s, on the companion computer | M | T | reflex |
| CTL-05 | MPC solve time is bounded (fixed iteration budget, real-time iteration). A deadline miss falls back to the previous solution shifted in time | M | T | reflex |
| CTL-06 | The inner loop runs on the FC at ≥ 2 kHz and takes collective thrust plus body rates (or angular acceleration via INDI) | M | T | reflex |
| CTL-07 | Use bidirectional DShot RPM telemetry for motor-level feedback (INDI, RPM notch filters) | S | T | reflex |
| CTL-08 | Tracking error at the gate plane, 3σ ≤ 0.25 m, at the qualification speed | M | T | reflex |
| CTL-09 | Provide a slot for a learned guidance-and-control policy (as in MonoRace or Swift) with the same inputs and outputs, so learned and MPC baselines can be compared head-to-head | S | T | racing-drone |

### 5.4 FPV, pilot interface, and modes (FPV)

**Scope.** League rules forbid human RC during autonomous heats (§2). FPV-03 and FPV-04 therefore apply in the **development profile**: development, testing, practice, and non-league missions. In the **league profile**, FPV-08 applies instead. The active profile is part of the mission configuration and is logged.

| ID | Requirement | Pri | Ver | Owner |
|---|---|---|---|---|
| FPV-01 | Stream live FPV video to the pilot, glass-to-glass latency ≤ 40 ms, separate from the navigation camera pipeline (navigation latency must not depend on the video link) | M | T | racing-drone |
| FPV-02 | Overlay an OSD with mode, battery, link quality, the autonomy state (next gate, estimator health), and DAA alerts | S | D | racing-drone |
| FPV-03 | Development profile: support a human pilot on an RC link (ELRS/CRSF, ≥ 250 Hz) in every mode in §4 | M | T | reflex |
| FPV-04 | Development profile: the pilot can switch between `AUTONOMOUS` and `MANUAL` at any time, with no thrust or attitude transient larger than one stick-induced transient (bumpless transfer) | M | T | reflex |
| FPV-05 | A kill switch on the RC link stops the motors within 50 ms through the FC alone, independent of the companion software | M | T | reflex |
| FPV-06 | Record the navigation camera onboard, and the FPV feed, for post-flight analysis | S | I | racing-drone |
| FPV-07 | Log the pilot's commands during `MANUAL`, so human laps can be used as reference data and compared with autonomous laps | C | T | racing-drone |
| FPV-08 | League profile: during an autonomous heat, RC input is limited to what the league rules allow (at minimum the FPV-05 kill switch). Stick takeover and `ASSISTED` are disabled, and the FC ignores other RC channels | M | T | reflex |

### 5.5 Detect and avoid (DAA)

| ID | Requirement | Pri | Ver | Owner |
|---|---|---|---|---|
| DAA-01 | Detect other drones and unexpected obstacles in the navigation camera's field of view with mono vision only (SYS-03) | M | T | cortex |
| DAA-02 | Track detected intruders and predict their trajectories with uncertainty over the MPC horizon | M | T | cortex |
| DAA-03 | Avoid predicted conflicts through constraints in the planner or MPC (preferred: keep racing) | M | T | cortex / reflex |
| DAA-04 | A last-resort safety filter in reflex acts on minimum separation independent of the MPC, for example with a control-barrier-function QP or a precomputed escape maneuver. Its assumptions and stale-data behavior are defined in the architecture (§8) | M | T | reflex |
| DAA-05 | Minimum separation ≥ 1.0 m center to center in ≥ 99 % of simulated in-envelope encounters in the DAA scenario matrix (TP §7). This is a statistical acceptance target measured in simulation, not a runtime guarantee | M | T | racing-drone |
| DAA-06 | Declared performance envelope, **applicable only while DAA is available** (fresh `ConflictSet`s, architecture §8): forward-hemisphere encounters at relative closing speed ≤ 10 m/s (overtaking and following), and crossings ≤ 20 m/s. While DAA is unavailable (startup, stale data, tracker failure), no envelope is claimed; the vehicle runs in conservative mode and reports DAA unavailable. Faster encounters, and encounters outside the camera FOV, are out of scope with the minimum sensor set and must be documented as such | M | A | racing-drone |
| DAA-07 | Right-of-way rule: the overtaking drone is responsible for separation, as in crewed aviation | M | A | racing-drone |
| DAA-08 | False alerts that cause an avoidance maneuver ≤ 1 per 10 laps on an empty course | S | T | cortex |
| DAA-09 | The DAA sensor interface accepts extra sensors (stereo, ToF, radar, event camera, cooperative position broadcasts) in baselines whose rules allow them | S | I | cortex |
| DAA-11 | Probability of a confirmed track ≥ 0.9 for a 0.3 m drone by the detection distances of §6.3 (≥ 5 m at 10 m/s closing speed, ≥ 10 m at 20 m/s), in the DAA-06 envelope under nominal lighting. P_d is measured per encounter: the fraction of encounters where the track is confirmed at or beyond that distance | M | T | cortex |
| DAA-10 | The DAA design can be traced to ASTM F3442/F3442M (DAA performance for small UAS), so it carries over to non-racing missions (ISR, interceptor) | C | A | racing-drone |

### 5.6 Safety and fault handling (SAF)

| ID | Requirement | Pri | Ver | Owner |
|---|---|---|---|---|
| SAF-01 | Losing the companion heartbeat (or command stream) for more than 50 ms puts the FC into a configurable failsafe (level-and-descend or disarm, per venue rules). This is the loss-of-companion behavior for every companion-side safety function (SAF-02, SAF-03, DAA-04) | M | T | reflex (FC) |
| SAF-02 | Geofence on the course flight volume. A breach forces `HOLD`, then `LAND`. Runs on the companion (it needs the position estimate). If the companion is lost, SAF-01 applies | M | T | reflex (companion) |
| SAF-03 | An estimator-health monitor (innovation / NIS gating, staleness) drives the degradation ladder: race → hold → land → disarm. Runs on the companion. If the companion is lost, SAF-01 applies | M | T | reflex (companion) |
| SAF-04 | RC link-loss behavior is configurable per mode | M | T | reflex (FC) |
| SAF-05 | Detect motor or ESC failure from RPM telemetry, and fly reduced-authority recovery when possible | C | T | reflex |
| SAF-06 | Preflight checks: sensors alive, calibration loaded, estimator converged, course loaded, link quality OK, battery OK, and (in baselines with DAA) 3 consecutive fresh `tracker_ok` conflict sets | M | T | racing-drone |

### 5.7 Compute, timing, and software (SW)

| ID | Requirement | Pri | Ver | Owner |
|---|---|---|---|---|
| SW-01 | No heap allocation and no blocking calls on the hot path (camera → estimator → MPC → FC command) after initialization | M | I, T | all |
| SW-02 | Every hot-path stage has a measured latency budget, checked in CI on the reference compute (§6.1) | M | T | all |
| SW-03 | Every module has a deterministic replay mode: same inputs, same outputs, bit for bit, on the same build | M | T | all |
| SW-04 | Components are C++20, follow the Branes repo conventions, and are pulled in through `cmake/deps.cmake` pins | M | I | all |
| SW-05 | Use permissive-licensed dependencies only. No GPL code, even as a test oracle (cortex ADR 0001/0002) | M | I | all |
| SW-06 | Run telemetry, logging, and the ground station over the Zenoh surface without slowing the hot path | S | T | cortex |

## 6. Derived budgets

### 6.1 Latency budget (pose-correction path)

At 28 m/s the vehicle moves 2.8 cm per millisecond. For a 30 ms stale estimate that is 0.84 m, more than the gate margin. Delay is therefore compensated (EST-06), and the budget below keeps the residual small.

| Stage | Budget (p99) | Notes |
|---|---|---|
| Exposure plus rolling-shutter readout | 11 ms | One frame at 90 Hz. Timestamp at mid-exposure |
| Transfer, undistort, crop | 2 ms | |
| Gate segmentation and corners | 6 ms | GPU, or a KPU once Synapse can run CNNs |
| PnP, EKF/VIO update, delay compensation | 2 ms | The VIO front end runs in parallel |
| MPC solve | 5 ms | Real-time iteration at ≥ 100 Hz |
| Safety filter (CBF, geofence, envelope) | 0.5 ms | Bounded-time QP or closed form |
| Queues, scheduling, and wake-up jitter (all hops) | 1.5 ms | Isolated cores, SCHED_FIFO, lock-free queues |
| Companion → FC link and FC loop | 1 ms | Serial at ≥ 1 Mbaud |
| **Total, camera → motor** | **≤ 29 ms** | MonoRace reports 17 ms image delay in its SITL model |

The per-stage values are *allocations*. Each is replaced by a measured p99 as the stage is built (SW-02). The acceptance quantity is the **measured end-to-end p99**, camera mid-exposure to motor command, at ≤ 29 ms. Stage p99s do not add up to the end-to-end p99, which is why it is measured directly.

IMU → motor through the FC rate loop: ≤ 1 ms.

**Gap today:** the cortex VIO front end measures about 38 ms median per frame (752×480, x86, `-O2`). The racing pipeline needs it under 8 ms per frame at 90 Hz. The main cost is full-frame FAST detection on every frame.

### 6.2 Gate-passing error budget

The gate inner opening is 1.5 m and the vehicle, props included, is about 0.35 m across. That leaves 0.575 m margin on each side.

| Contributor | 3σ allocation | Error model |
|---|---|---|
| Estimation error at the gate plane (EST-04) | 0.30 m | Zero-mean random, per crossing |
| Tracking error at the gate plane (CTL-08) | 0.25 m | Zero-mean random, per crossing |
| Gate map residual after online correction | 0.15 m | **Bias**: systematic per gate, and can correlate with estimation error, because the estimator is anchored to the same gates |

How the contributors combine:

- **Independent random terms:** root sum of squares, √(0.30² + 0.25²) = 0.39 m.
- **Map residual:** treated as a bias and added linearly, 0.39 + 0.15 = **0.54 m < 0.575 m**. That leaves a 0.035 m margin, which is thin.
- **All three fully correlated** (worst case): 0.70 m, which exceeds the margin.

The allocations are therefore design guidance only, not proof of clearance. The acceptance quantity is the **measured true gate-crossing error** (test plan §3 `gate_error`, true vs. gate center). It is measured over the randomized scenario suite, so all correlations are included: |bias| + 3σ must be ≤ 0.575 m per gate. If the margin proves too thin, the first lever is tighter online gate-map correction.

### 6.3 DAA detection-range budget

The time needed to resolve a conflict is:

| Contributor | Time |
|---|---|
| Lateral offset of 1.0 m at 5 g lateral acceleration | ≈ 0.20 s |
| Pipeline latency | 0.03 s |
| Track confirmation (3 frames at 90 Hz) | 0.03 s |
| Margin | 2× |
| **Total** | **≈ 0.5 s** |

At 10 m/s closing speed that needs detection at ≥ 5 m; at 20 m/s, ≥ 10 m.

A 0.3 m drone at 10 m subtends about 1.7°. The RP-1 camera has about 5.3 px/deg (820 px / 155°), so the target is only **about 9 px wide**. Detection at range with the wide-FOV navigation camera is therefore the hardest DAA problem. That is why DAA-06 limits the declared envelope, and DAA-09 lets other baselines add a narrow-FOV or ranging sensor.

## 7. Open questions

1. **Flight-controller firmware.** Run reflex on our own MCU firmware from the start, or start on Betaflight or PX4 (companion sends thrust and body rates) and replace the FC loop with reflex later? The plan assumes the second, to de-risk the first flights.
2. **Target compute.** Is it Jetson Orin NX first and KPU later? This sets whether Synapse needs CUDA or TensorRT interop, or only CPU/KPU kernels.
3. **Venue rules.** For the human-pilot channel: does the target league allow pilot takeover during an autonomous heat, or only a kill switch?
4. **Head-to-head DAA.** Are other teams' drones broadcasting position (cooperative DAA), or is DAA purely non-cooperative?
5. **Course knowledge.** Is the gate map exact (surveyed), or nominal only? This changes how much online map correction CTL-02 needs.
6. **Rulebook.** Obtain the official A2RL drone rulebook. The sensor and RC rules in §2 are taken from press coverage.
