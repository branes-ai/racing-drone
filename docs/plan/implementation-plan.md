# Racing drone implementation plan

**Status:** Draft
**Date:** 2026-10-23
**Related:** [requirements](../requirements/mission-requirements.md), [architecture](../arch/framework-architecture.md), [test plan](test-plan.md)

## 1. Starting point

What exists today (inventory of 2026-10-23):

| Repo | Usable now | Missing for racing |
|---|---|---|
| cortex | Mono MSCKF VIO (EuRoC V1_01 ATE ≈ 0.29 m), FAST + KLT front end, camera models (pinhole, Kannala-Brandt, unified), Lie groups, PnP, IMU preintegration, eval probes (NEES/NIS), benches | Gate detection, drone detection, planning, tracking, Zenoh daemons (placeholders), message schemas, YAML config (empty). VIO is **over-confident** (NEES ≈ 43 vs. 15, issue #212). Frame time **≈ 38 ms**, need < 8 ms |
| reflex | `Pid<Real>` and a gyro low-pass example | Dynamics model, SITL, mixer, attitude estimator, INDI, MPC / RTI solver, CBF, mode manager, FC firmware |
| synapse | Static-arena graph engine, LLM decode | conv2d / pooling / upsample / activation ops, image tensors, INT8 vision kernels, a non-token session API |
| racing-drone | README | Everything in the architecture doc |

None of the repos depends on another yet. racing-drone will be the first consumer of all three.

## 2. Phases

Phases 1–7 each end in a **baseline that runs in the scenario suite**, so progress is measured as lap time and success rate, never as "component done". Phase 0 (framework) and Phase 8 (hardening) are infrastructure phases, and end in explicit gates instead of a new baseline.

**One acceptance contract:** the exit gates are defined once, in [test plan §6](test-plan.md#6-acceptance-gates-per-phase). The *Exit* lines below are summaries of those gates. If the two ever disagree, the test plan wins.

Phases 1–3 and parts of 4 run in simulation only. Hardware starts in Phase 5.

### Phase 0: framework skeleton (racing-drone)

- Repo build: `cmake/deps.cmake` pinning cortex, reflex, synapse, MTL5, and Universal. Presets, CI (GCC, Clang, MSVC), commitlint, release-please, as in the siblings.
- `branes::racing::msg` data contracts and `slots` concepts (architecture §3–4).
- Loaders and schemas for vehicle, course, scenario, and baseline files. Includes `vehicles/rp1.yaml` and `courses/a2rl-2025.yaml` (an approximation from published figures).
- Log format and the `replay` app skeleton (SYS-08, SW-03).
- **Exit (gate P0):** CI is green on GCC, Clang, and MSVC. Config files round-trip. A `null` baseline (stub implementation in every slot) builds `flight_stack_null`, `replay_null`, and `sitl_null`. Replaying a synthetic log through it twice gives bit-identical output.

### Phase 1: simulation and B0 (reflex + racing-drone)

- **reflex:** rigid-body quadrotor model on SO(3), with rotor thrust and torque maps, a first-order motor model, quadratic drag (the MonoRace-style v|v| term), and battery sag. Deterministic fixed-step integrator. Mixer and allocation.
- **reflex:** cascaded PID rate → attitude → velocity → position, and gyro filters (roadmap Phase 1).
- **racing-drone `sim/`:** SITL harness that steps the dynamics with sensor models (IMU noise, bias, ±16 g saturation, latency) and an *oracle* gate projector (gate corners projected through the camera model, no rendering yet). Includes fault injection.
- **Baseline B0 `gt-pid`:** ground-truth state, waypoint planner, cascaded PID. Flies the course slowly.
- **Exit:** B0 completes two laps at ≤ 5 m/s on `a2rl-2025` in ≥ 99 % of seeds.

### Phase 2: time-optimal control with perfect state, B1 (reflex + cortex)

- **cortex:** point-mass time-optimal sampler with online replanning (CTL-01/02). Offline CPC tool for the lap-time bound.
- **reflex:** INDI + rate loop on the FC model. Nonlinear MPC with a real-time-iteration SQP and a bounded iteration count (roadmap Phase 3). MPCC formulation with a progress variable. Validate against acados/HPIPM (BSD-2) as an *oracle in tests only*.
- **Baseline B1 `gt-mpcc`:** ground-truth state and MPCC. Establishes the control upper bound.
- **Exit:** B1 two-lap time within 10 % of the CPC bound (SYS-10). MPC p99 solve ≤ 5 ms on x86 and on the target companion CPU. Tracking 3σ ≤ 0.25 m at gates (CTL-08).

### Phase 3: vision-only state estimation, B2 / B2m (cortex + synapse)

This is the critical-path phase. Steps 1–2 can start in parallel with Phase 1.

1. **Rendering bridge.** Camera images from an external renderer (Isaac Sim or Flightmare, after a license check) through the `CameraFrame` contract, with a rolling-shutter and motion-blur model. Domain randomization: lighting, textures, gate pose noise.
2. **synapse vision ops:** conv2d (with depthwise and stride variants), pooling, upsample/transpose-conv, ReLU/SiLU, concatenation, an image-tensor session API, and INT8 kernels. Target: a U-Net GateNet (384×384) in ≤ 6 ms on the companion.
3. **cortex GateNet + corners.** Segmentation network, line-based corner extraction with RANSAC, and multi-gate PnP (EST-02/03). Training data from the renderer plus labeled real images (MonoRace ratio: 3,500 synthetic to 500 real).
4. **cortex estimator `GateEkf`** (MonoRace-style). 16-state EKF with drag-model prediction, IMU-saturation switching (EST-07), and delay compensation (EST-06).
5. **cortex estimator `VioGateEkf`.** Fuse the MSCKF with gate PnP. Prerequisites:
   - fix the NEES over-confidence (#212);
   - cut the front-end frame time from 38 ms to < 8 ms (detect only in feature-starved cells, run detection on the accelerator);
   - add rolling-shutter support (EST-09).
6. **Baselines B2m `gate-ekf-mpcc` and B2 `vio-gate-mpcc`.**
- **Decision at P3, before the gate:** run the B2 vs. B2m comparison under [ADR 0004](../adr/0004-gate-relative-hybrid-estimation.md). It selects the **primary racing baseline**: B2 if `VioGateEkf` passes the promotion criterion, otherwise B2m. Record the result in a new ADR. From here on, "B2" in later phases means the selected primary baseline.
- **Exit (gate P3):** in rendered SITL (L3), the selected primary baseline (B2 or B2m) achieves ≥ 90 % two-lap success at the qualification speed on the nominal, course-uncertainty, and sensor-degradation groups. The other requirements:
  - measured camera-to-motor latency p99 ≤ 29 ms;
  - EST-04 / EST-05 met;
  - true gate-crossing error |bias| + 3σ ≤ 0.575 m.

### Phase 4: FPV, pilot, and the FC integration (reflex + racing-drone)

- **FC path, interim.** `BetaflightBridge`: the companion sends thrust and body rates over MSP or a custom serial protocol (open question 1).
- **reflex:** mode manager and pilot arbiter (development and league profiles), failsafes, bumpless takeover (FPV-04), and the estimator-health ladder (SAF-03). Proven in SITL first.
- **Where each safety function runs**, in both FC configurations ([ADR 0002](../adr/0002-companion-mpc-fc-rate-loop.md)):

| Function | Interim (Betaflight FC) | Final (`ReflexFc`) | If the companion link drops |
|---|---|---|---|
| Kill switch, arming, RC input (FPV-05, SAF-04) | FC: Betaflight | FC: reflex | Unaffected |
| Mode manager / pilot arbiter, takeover (FPV-04, FPV-08) | FC: Betaflight modes (switch-selected); autonomy sub-states on the companion | FC: reflex | Interim: Betaflight drops to its failsafe (or `MANUAL` if the pilot has the switch). Final: reflex `SAF-01` failsafe |
| Companion heartbeat and command-stream loss (SAF-01) | FC: Betaflight detects command-stream loss like RX loss, then its failsafe (*to verify in HIL*) | FC: reflex, 50 ms timeout | Final: configured failsafe within 50 ms. Interim: **unverified** until the HIL check passes; no Betaflight flight before it does |
| Geofence (SAF-02), estimator-health ladder (SAF-03) | Companion: reflex | Companion: reflex | Not enforceable; covered by the SAF-01 failsafe |
| DAA safety filter (DAA-04), envelope filter | Companion: reflex | Companion: reflex | Not enforceable; covered by the SAF-01 failsafe |

- **Expected fault outcomes** (test plan §5 faults group):
  - **Companion crash or FC link loss:** the FC reaches the configured failsafe within 50 ms in both configurations. A development-profile pilot can then take `MANUAL`.
  - **RC loss:** the per-mode SAF-04 action applies. In `AUTONOMOUS` under the league profile, the race continues and the kill path is lost, so the venue rules decide whether that is allowed.
  - The interim row for SAF-01 is the one unverified assumption about Betaflight. HIL must show it before any flight.
- **FPV.** Separate FPV camera and VTX. MSP DisplayPort OSD with the autonomy state (FPV-01/02). Onboard recording of the navigation camera.
- **HIL rig.** Real FC and companion with simulated sensors injected (IMU over the FC sensor bus, or replayed; rendered images into the companion), motors off.
- **Exit (gate P4):** every fault and pilot scenario passes in L2 and L4, with the outcomes above, in both profiles. Measured kill latency on hardware ≤ 50 ms. Command-stream-loss failsafe ≤ 50 ms.

### Phase 5: flight (racing-drone)

- Build the RP-1 airframe. Calibrate camera intrinsics, camera–IMU extrinsics, and the motor thrust map (static thrust stand).
- Flight progression per the test plan §8: tethered hover → manual → B0 / B1-style slow laps → B2 at increasing speed caps.
- Refine extrinsics and drag parameters offline from logs (EST-10). Close the sim-to-real gap by fitting the reflex dynamics model to flight data.
- **Exit (gate P5):** B2 completes two laps on a physical course at the qualification speed (initially 10 m/s, then raised) with ≥ 80 % success over ≥ 20 flights (SYS-09). Real lap time is within 5 % of the SITL lap time for the same course and speed cap, which shows the simulator is predictive enough to keep developing in.

### Phase 6: detect and avoid, B4 (cortex + reflex + synapse)

- **synapse/cortex:** DroneNet as a second head on the GateNet backbone. Training data: rendered drones from many viewpoints plus real footage.
- **cortex:** `ImmTracker` with bearing and apparent-size ranging, and conflict prediction over the MPC horizon (DAA-02).
- **reflex:** time-varying separation constraints in MPCC (DAA-03). The CBF safety filter with a bounded-time QP or closed form (DAA-04).
- **racing-drone `sim/`:** multi-drone scenarios. Intruders flown by B1 or B2 instances, or scripted (overtake, follow, cross, a drone stalled in a gate).
- **Baseline B4 `vio-gate-mpcc-daa`.**
- **Exit (gate P6):** DAA-05 (≥ 99 % of in-envelope simulated encounters with separation ≥ 1.0 m) across the DAA scenario matrix. DAA-08 false-alert rate. B4 lap time on an empty course within 2 % of B2's (DAA must cost almost nothing when there is no conflict).

### Phase 7: learned-control baseline B3 (synapse + racing-drone), optional and non-blocking

Phase 7 runs in parallel after Phase 3. No other phase waits for it, and the mission can complete without it. Its gate applies only if the phase is run.

- **Training.** RL (PPO) in a GPU-parallel simulator, with the reflex dynamics as the reference model and domain randomization at 30–50 % (MonoRace values).
- **Deployment.** `LearnedPolicy` through synapse on the companion. Optionally, a motor-command policy on the FC (MonoRace runs at 500 Hz on an STM32H7).
- **Exit (gate P7, non-blocking):** a head-to-head scoreboard of B2 vs. B3 on the full scenario suite. Decide (as an ADR) which controller is primary for racing, and whether MPCC remains the DAA-capable controller.

### Phase 8: platform hardening

- Move the FC loop from Betaflight to `ReflexFc` firmware (reflex: Phase 1 rate loop + Phase 4 safety).
- Move the hot path to the KPU (cortex `kpu_provider`, synapse KPU kernels). Compare energy and latency with the Orin.
- Motor-failure recovery (SAF-05).
- **Exit (gate P8):** with `ReflexFc` replacing Betaflight, B2 re-passes gates P4 and P5 with no regression (success rate within 2 percentage points, kill and failsafe latency ≤ 50 ms). On the KPU, the B2 L3 gate (P3) still passes, with a published latency and energy comparison against the Orin. The degraded-motor scenario (one motor at 70 %) ends in a controlled landing in ≥ 95 % of seeds.

## 3. Critical path and parallel tracks

```text
P0 skeleton --> P1 dynamics + SITL --> P2 MPCC (B1) -----------------------+
                     |                                                     |
                     +--> P3.1 renderer --> P3.3 GateNet --> P3.4/5 est --> B2 --> P4 HIL --> P5 flight
                                               ^                                                 |
             P3.2 synapse conv ops ------------+                                                 v
                                                                    P6 DAA (sim from P3) --> B4 in flight
             cortex: VIO NEES fix + front-end speedup (parallel, needed by P3.5 only)
             P7 learned baseline (optional, non-blocking, after P3)
```

The longest pole is **synapse vision ops → GateNet → vision-only estimation**. Two mitigations:

- Start the synapse conv work and the renderer bridge in parallel with Phase 1.
- Allow a temporary vendor-runtime GateNet behind the `GateDetector` slot, so Phase 3 isn't blocked on synapse.

## 4. Work allocation by repo

| Repo | Work items (in order) |
|---|---|
| **racing-drone** | Skeleton, contracts, slots, loaders; SITL harness, sensor models, fault injection; renderer bridge; baselines B0–B4; mission state machine; Betaflight bridge; FPV / OSD; HIL rig; flight tests; scoreboard |
| **reflex** | Dynamics model; mixer; cascaded PID; filters; INDI; RTI-SQP NMPC / MPCC; mode manager and arbiter; failsafes; health ladder; CBF safety filter; FC firmware |
| **cortex** | Course-frame types; time-optimal planner and CPC; GateNet integration and corners; multi-gate PnP; GateEkf; VIO fixes (NEES #212, latency, rolling shutter); VioGateEkf; DroneNet; IMM tracker; Zenoh daemons and schemas |
| **synapse** | Vision ops (conv2d, pool, upsample, activations); image-tensor session API; INT8 vision kernels; GateNet/DroneNet model loader; small-MLP policy runner; KPU backend |

## 5. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| VIO too slow or inconsistent at racing speed | B2 depends on gate-only fixes | B2m (MonoRace-style) needs no VIO. Use VIO only to bridge gaps |
| Sim-to-real gap in perception | GateNet fails on real images | Real-image fine-tuning (≈ 500 labeled images). Heavy augmentation. Log-driven relabeling |
| MPC slower than the learned policy | Lap time behind SOTA | B3 comparison (Phase 7). Keep MPCC for DAA and constraints, possibly as a supervisor over a learned policy |
| Camera EMI and corrupted frames | Estimator failure | Corruption detector, EST-08 tests, cable and shielding design |
| Accelerometer saturation | Estimator divergence | Model-based specific-force fallback (EST-07). Choose a ±32 g IMU if one is available |
| DAA detection range with a wide-FOV camera | DAA envelope too small | Declared envelope (DAA-06). An extra sensor where rules allow (DAA-09). Cooperative position sharing where available |
| Licensing of simulators or reference stacks | Clean-room violation | License check before any use. Architecture-level reference only (ADR 0002) |
| Three repos with different MTL5 / Universal pins | Diamond-dependency breakage | Align the pins before Phase 0 exits. racing-drone CI builds all three at the pinned versions |
