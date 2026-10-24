# Racing drone test plan

**Status:** Draft
**Date:** 2026-10-23
**Related:** [requirements](../requirements/mission-requirements.md), [implementation plan](implementation-plan.md)

## 1. Principles

1. **Every requirement has a test or an analysis.** The traceability table (§10) is the checklist.
2. **Measure the mission, not just the module.** The primary metrics are lap time, two-lap success rate, and minimum separation, run over many randomized seeds. Component metrics explain them.
3. **Same baseline, every level.** The same baseline file runs in replay, SITL, HIL, and flight (SYS-07).
4. **Deterministic and reproducible.** Every scenario has a seed, and every result records the build hash and baseline hash. A replay of a log through the same build gives bit-identical outputs (SW-03).
5. **Gates in CI.** A regression in lap time, success rate, or latency fails the build, as cortex does with its EuRoC ATE gate.

## 2. Test levels

| Level | What runs | Where | Cadence |
|---|---|---|---|
| L0 unit | Single functions and classes (Catch2) | Each platform repo + racing-drone | Every PR |
| L1 component | One slot implementation on recorded or synthetic data (estimator on replay, MPC on a reference, detector on labeled images) | racing-drone `tests/component` | Every PR (short set), nightly (full set) |
| L2 closed-loop SITL | A full baseline against the reflex dynamics model with oracle perception (no rendering) | racing-drone `sim/` | Every PR (smoke: 10 seeds), nightly (500 seeds) |
| L3 rendered SITL | A full baseline with rendered images and the real perception stack | GPU runner | Nightly / weekly (200 seeds) |
| L4 HIL | Real FC + companion, simulated sensors, motors off or on a thrust stand | Bench rig | Per release candidate |
| L5 flight | Real vehicle on a real course | Test venue with net enclosure | Per milestone |

## 3. Metrics (scoreboard schema)

| Metric | Definition |
|---|---|
| `success` | All gates passed in order, both laps, no collision, no geofence breach |
| `lap_time` | Per lap, and the two-lap total |
| `gap_to_optimal` | Two-lap time / CPC bound − 1 |
| `gate_error` | Distance from the gate center at the gate-plane crossing: true, estimated, and reference |
| `est_err` | ATE / RPE vs. ground truth (SITL, or mocap at the test venue for evaluation only), NEES, NIS |
| `latency` | Camera mid-exposure → motor command, p50 / p99 / max, per stage |
| `mpc_solve` | Solve time p50 / p99, iterations, deadline misses |
| `min_sep` | Minimum center-to-center distance to any intruder or obstacle |
| `daa_false_alert` | Avoidance maneuvers with no true conflict, per lap |
| `takeover_transient` | Peak attitude and thrust deviation in the 0.5 s after a mode switch |
| `cpu`, `gpu`, `energy` | Per-stage load and energy (cortex bench energy hooks) |

The test-venue mocap is used **only to score** flights, never as an input (SYS-02).

## 4. L0/L1 component tests, highlights

| Component | Test | Pass criterion |
|---|---|---|
| Dynamics model | Hover thrust, step responses, energy conservation with drag off; compare with real thrust-stand data | Analytic match to 1e-9; thrust map within 5 % |
| INDI + rate loop | Step and chirp in body rates; disturbance torque rejection | Rise time and overshoot per spec; disturbance rejected within 50 ms |
| MPCC | Solve on recorded states; compare with the acados oracle (tests only) | Cost within 1 % of the oracle; p99 ≤ 5 ms; zero deadline misses in 1e5 solves |
| Time-optimal planner | Gate sequences with known optima | Within 2 % of CPC |
| GateNet | Labeled real and synthetic sets, by distance, angle, lighting, blur | Corner RMS ≤ 2 px at ≤ 10 m; recall ≥ 98 % at ≤ 8 m |
| Gate PnP | Synthetic projections with noise; rolling shutter on/off | Pose error matches the predicted covariance (NEES) |
| Estimators | UZH-FPV racing sequences, EuRoC, own logs; synthetic world | EST-04 / EST-05 thresholds |
| IMU saturation | Synthetic 7 g maneuver with a ±16 g sensor clipped below truth | No divergence; position error ≤ 2× nominal |
| DroneNet + tracker | Rendered and real drones at 2–20 m | Detection range for P_d ≥ 0.9 per DAA-06; track continuity |
| CBF filter | Random states and conflicts; worst-case runtime | Never violates separation when feasible; bounded runtime |
| Mode manager | Exhaustive transition table, including invalid ones | Matches the state machine; no unhandled input |

## 5. Scenario suite (L2/L3)

Scenarios are YAML files in `scenarios/`, each run over N seeds.

| Group | Scenarios |
|---|---|
| Nominal | `a2rl-2025` two laps; three more course layouts (tight hairpin, split-S, long straight); speed caps 5 / 10 / 15 / 20 / 28 m/s |
| Course uncertainty | Gate poses perturbed ±0.25 / ±0.5 m and ±5 / ±10°; one gate missing from the map; gate moved mid-race |
| Sensor degradation | Camera corruption at 10 / 25 / 50 % of frames; 0.5 s and 1 s blackouts; motion blur; lighting change; LED flicker; IMU saturation; IMU bias step; camera-extrinsic error of 1° and 3° |
| Dynamics mismatch | Mass ±10 %, inertia ±20 %, drag ±30 %, motor time constant ±30 %, battery sag, ground effect at the start |
| Timing | Extra 5 / 10 / 20 ms latency; companion CPU contention; dropped FC packets |
| Faults | Companion crash; FC link loss; RC loss in each mode; geofence approach; one motor degraded at 70 % |
| Pilot | Takeover at each phase of the lap; ASSISTED laps with scripted stick inputs; kill switch at full speed |

**Randomization levels:** *nominal* (the expected real-world variation) for the SYS-09 gate, and *stress* (2× nominal) for reporting only.

## 6. Acceptance gates per phase

| Phase | Gate (must pass to exit) |
|---|---|
| P1 | B0: ≥ 99 % success at ≤ 5 m/s, nominal group, 500 seeds |
| P2 | B1: gap_to_optimal ≤ 10 %; mpc_solve p99 ≤ 5 ms; gate_error 3σ ≤ 0.25 m |
| P3 | B2: ≥ 90 % success at the qualification speed, nominal + course-uncertainty + sensor-degradation groups (L3); latency p99 ≤ 27 ms; EST-04 / EST-05 |
| P4 | All fault and pilot scenarios pass in L2 and L4; kill ≤ 50 ms measured on hardware |
| P5 | ≥ 80 % success over ≥ 20 flights at the qualification speed; sim-vs-real lap time within 5 % |
| P6 | DAA-05 over the DAA matrix (§7); DAA-08; B4 vs. B2 empty-course lap time ≤ +2 % |
| P7 | Scoreboard B2 vs. B3, all groups |

## 7. DAA scenario matrix

Encounters are run in L2 with oracle detections, to verify the avoidance logic. They are then run in L3 with DroneNet, to verify the whole chain. Each cell runs 200 seeds.

| Geometry | Relative speed | Intruder behavior | Notes |
|---|---|---|---|
| Overtake (ownship faster, same line) | 2, 5, 10 m/s | Steady; brakes suddenly; weaves | Ownship responsible (DAA-07) |
| Follow (intruder ahead, slower in a turn) | 2, 5 m/s | Steady | Typical head-to-head |
| Converge at a gate (both aim for the same gate) | 5, 10 m/s | Steady | Gate occupancy conflict |
| Crossing (course self-intersection) | 10, 20 m/s | Steady | Limit of the declared envelope |
| Head-on (wrong-way or crashed drone) | 20, 40 m/s | Steady | **Outside the envelope**: report only, documents the limit |
| Static obstacle (fallen gate, drone stalled in a gate, net sag) | Ownship speed | Static | Detection at ≥ 10 m required |
| Out-of-FOV intruder from behind | Any | Steady | Out of scope with the minimum sensor set; check that no unsafe action results |
| Empty course | — | — | False-alert rate (DAA-08) |

Pass: separation ≥ 1.0 m in ≥ 99 % of in-envelope encounters, and no collision caused by the avoidance maneuver itself (for example, being pushed into a gate frame).

## 8. Flight test progression (L5)

Each step needs the previous one passed and a test card signed off. A safety pilot holds the kill switch at all times. Flights run inside a net enclosure.

1. Bench: motor direction, mixer, failsafes with props off, kill latency.
2. Tethered hover in `MANUAL`; then untethered `MANUAL` acro by a human pilot (validates the airframe and the FC tune).
3. Estimator in the loop, controller *not* in the loop: a human flies laps while the estimator runs and logs. Score it against venue mocap (EST-04 on real data).
4. `AUTONOMOUS` hover and slow waypoints (B0-class, 3 m/s cap), with takeover drills.
5. B2 at speed caps of 5 → 10 → 15 → 20 m/s. Raise the cap only after ≥ 10 consecutive successful flights at the current one, and a sim-vs-real comparison review.
6. Fault drills in flight at low speed: companion kill, RC loss, camera cover.
7. Multi-drone: a second drone flown by a human, then by B2, at a 5 m/s relative-speed cap, then raised. DAA (B4).

## 9. Infrastructure

- **Log format.** Timestamped channels for all slot inputs and outputs. The `replay` app runs any baseline over any log.
- **Scoreboard.** One results table per run (CSV/JSON), with plots per metric and a baseline-vs-baseline diff. Published to the docs site like the cortex benchmark data.
- **CI.** L0 + L1-short + L2-smoke on every PR (GCC, Clang, MSVC). Nightly L1-full + L2-full. A weekly GPU runner for L3.
- **Ground-truth venue.** Mocap volume or total-station-surveyed gates, used for scoring only.

## 10. Traceability (excerpt)

| Requirement | Verified by |
|---|---|
| SYS-01 / SYS-09 | §5 nominal group, §6 P3 / P5 gates, §8 step 5 |
| SYS-02 | Inspection of the baseline files and build: no external-pose inputs exist in `msg/` |
| EST-04 / EST-05 | §4 estimators; §8 step 3 |
| EST-06 / SW-02 | §3 `latency`; §5 timing group |
| EST-07 / EST-08 | §4 IMU saturation; §5 sensor-degradation group |
| CTL-03..05 / CTL-08 | §4 MPCC; §6 P2 |
| FPV-04 / FPV-05 | §5 pilot group; §8 steps 1 and 4 |
| DAA-01..08 | §7 matrix; §6 P6 |
| SAF-01..04 | §5 faults group; §8 step 6 |

The full matrix is maintained as `docs/plan/traceability.csv` once the test IDs exist.
