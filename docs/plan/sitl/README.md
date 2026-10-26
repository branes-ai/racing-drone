# SITL harness on the reflex dynamics

**Status:** Approved 2026-10-26, in implementation
**Issue:** [racing-drone#16](https://github.com/branes-ai/racing-drone/issues/16), part of the Phase 1 epic [racing-drone#8](https://github.com/branes-ai/racing-drone/issues/8)
**Related:** [implementation plan](../implementation-plan.md) Phase 1, [test plan](../test-plan.md) §2–§6, [architecture](../../arch/framework-architecture.md), [ADR 0001](../../adr/0001-baseline-composition-framework.md)

This plan replaces Phase 0's `SyntheticSensors` (a vehicle at rest) with closed-loop simulation on the reflex dynamics model. It is the L2 level of the test plan, and the harness that baseline B0 ([racing-drone#17](https://github.com/branes-ai/racing-drone/issues/17)) flies in for gate P1.

## Issues and pull requests

### This plan

| Item | What | State |
|---|---|---|
| [racing-drone#16](https://github.com/branes-ai/racing-drone/issues/16) | This plan: SITL harness, sensor models, oracle gates, faults, scoreboard | Open |
| racing-drone PR (see below) | The implementation; this document lands with it | — |
| [racing-drone#17](https://github.com/branes-ai/racing-drone/issues/17) | Next: B0 `gt-pid` baseline and gate P1, which consume this harness | Open |

### What it builds on (merged)

| Item | What it provides here |
|---|---|
| [reflex#18](https://github.com/branes-ai/reflex/issues/18) / [reflex#24](https://github.com/branes-ai/reflex/pull/24) | `QuadrotorModel`: the plant |
| [reflex#30](https://github.com/branes-ai/reflex/pull/30) | The model at grade C: validity envelope, invariant diagnostics, operation-count contract |
| [reflex#28](https://github.com/branes-ai/reflex/pull/28) | The building-block standard and the model's V&V plan, which set what this evidence can claim |
| [reflex#19](https://github.com/branes-ai/reflex/issues/19) / [reflex#27](https://github.com/branes-ai/reflex/pull/27) | `Mixer`: wrench → rotor thrusts, with the saturation report (the simulated FC) |
| [reflex#20](https://github.com/branes-ai/reflex/issues/20) / [reflex#31](https://github.com/branes-ai/reflex/pull/31) | `RateController` and `cascade_config_from` (the simulated FC's rate loop); the full cascade is B0's controller in #17 |
| [reflex#21](https://github.com/branes-ai/reflex/issues/21) / [reflex#33](https://github.com/branes-ai/reflex/pull/33) | `dsp::Biquad`: the simulated FC's gyro low-pass |
| [reflex#17](https://github.com/branes-ai/reflex/issues/17) / [reflex#22](https://github.com/branes-ai/reflex/pull/22), [cortex#518](https://github.com/branes-ai/cortex/pull/518), [synapse#1](https://github.com/branes-ai/synapse/pull/1) | Pin alignment: MTL5 5.20.0 and Universal 5.11.0 across reflex, cortex, and synapse; reflex and synapse define `MTL5::mtl5` only if no one else has |

### Known limits, tracked upstream

| Item | Effect here |
|---|---|
| [stillwater-sc/mtl5#652](https://github.com/stillwater-sc/mtl5/issues/652) | reflex (through MTL5's fixed-size types) does not build with `-fno-exceptions`. No effect on SITL; it blocks flight-controller builds of the same code |
| [stillwater-sc/mp-dsp#217](https://github.com/stillwater-sc/mp-dsp/issues/217), [#218](https://github.com/stillwater-sc/mp-dsp/issues/218) | mp-dsp stays a test-only reference in reflex; no effect here |

## What the evidence can claim

The plant is reflex `QuadrotorModel`, at grade C under the [building block standard](https://github.com/branes-ai/reflex/blob/main/docs-site/src/content/docs/standards/building-blocks.md). Its independent references (RotorPy and PX4 SIH, then RP-1 flight data) are milestones of its [V&V plan](https://github.com/branes-ai/reflex/blob/main/docs-site/src/content/docs/model/quadrotor-vv.md). L2 SITL results are therefore **closed-loop design verification on that model, not validation**.

Two checks in this plan do not depend on the model's grade:
- the scripted-trajectory acceptance compares the harness with reflex itself and with analytic results;
- replay determinism is a property of the harness.

## Layout

A new top-level `sim/` directory, as the architecture's repo layout shows. It is a header-only target `branes::racing_sim`, in namespace `branes::racing::sim`, that links `branes::racing` and `branes::reflex`.

`cmake/deps.cmake` adds MTL5 5.20.0 and Universal 5.11.0 (aligned with the siblings), then reflex, pinned to a commit on its main. Reflex's tests and benches stay off, because it is not the top-level project.

## 1. `vehicle.hpp`: `reflex_params_from(VehicleConfig) -> result<QuadrotorParams<double>>`

- Maps mass, inertia, arm length, k_f, k_m, maximum thrust, motor τ, and drag.
- Rejects `rotor_count ≠ 4` and anything that fails `quadrotor_params_valid`.
- Uses the battery defaults for now.
- Sets `ground_z` to the course start height, so the vehicle starts at rest on the pad.

## 2. `imu.hpp`: `ImuModel`

**Model:**
- the vehicle's IMU parameters;
- `rng_` (SplitMix64, the only random source);
- gyro and accelerometer biases, each with an optional per-axis random walk;
- a fixed-capacity delay line of pending samples.

**Measurement:**
- gyro = ω + b_g + n;
- accel = specific force + b_a + n. Airborne, the specific force is Rᵀ(v̇ + g e₃), from the model's `derivative()`; on the ground it is Rᵀ g e₃.

Each axis is clipped at ±range, with its `ImuSaturation` bit set.

**Latency:** a sample measured at t carries `t` but is delivered at t + latency.

**Invariants:**
- I1: the biases are finite.
- I2: samples are delivered in non-decreasing delivery time.
- I3: every clipped axis has its bit set, and |value| ≤ range.

**Noise:** uniform with matched σ, as `SyntheticSensors` already uses. It needs no libm calls, so logs stay bit-reproducible.

**New optional vehicle-file fields** (default 0): `gyro_bias_random_walk`, `accel_bias_random_walk`, `initial_gyro_bias_sigma`, `initial_accel_bias_sigma`, `latency_s`.

## 3. `fc.hpp`: `SimFc`, the simulated flight controller

**Model:**
- the reflex `RateController` and `Mixer`, configured by `cascade_config_from(params).rate`;
- the FC's gyro low-pass (a `dsp::Biquad` Butterworth);
- the last accepted command and the time it was received;
- the mixer's saturation feedback.

**`step(now, gyro) -> RotorCommand`**, at the FC rate (1 kHz):
- `thrust_rates`: rate loop → wrench → mix → rotor speeds;
- `motors` (normalized thrust per rotor): ω = ω_max·√m, which bypasses the rate loop and drives the plant directly.

**Failsafe:** with no command yet, an invalid command, or no command for longer than `command_timeout_s` (default 0.1 s), every rotor goes to zero. This stands in for the reflex mode manager (P4).

**Invariants:**
- I1: the configuration is valid.
- I2: the held command passes `valid(ThrustRates)`.
- I3: every rotor command is in [0, ω_max].

## 4. `gates.hpp`: `project_gates(course, truth, camera) -> GateObservations<double>`

- Each gate's four inner corners go through the camera extrinsics and the equidistant (Kannala–Brandt) projection with the vehicle file's distortion.
- A gate is kept only if all four corners lie in front of the camera and inside the image.
- The nearest `kMaxVisibleGates` gates are kept, with `quality = 1` and a corner covariance from `oracle_corner_sigma_px` (default 1 px).
- Not modeled: rolling shutter and occlusion (non-goals).

## 5. Log and message additions

- **`msg/truth.hpp`:** `TruthState<T>` holds t, position, velocity, attitude, body rates, rotor speeds, and `on_ground`.
- **Sim channels** `truth` (4) and `oracle_gates` (5) are input-side: replay copies them through, and verification ignores them. In #16 they are not fed to the stack; #17 adds `Stack::on_truth` for `GroundTruthEstimator`.
- **`Runner`** accepts sim channels and skips them instead of reporting an error.
- **`codec`** encodes and decodes both types.

## 6. `simulator.hpp`: `Simulator`

It replaces `SyntheticSensors` in `sitl_<baseline>`.

**Model:**
- the plant state and model, `SimFc`, and `ImuModel`;
- the schedules, in integer nanoseconds: physics and FC at 1 kHz, IMU, camera, tick, and truth at 1 kHz;
- the fault schedule;
- the pending command.

**Event order at equal times:**
1. physics step;
2. IMU deliveries;
3. camera frame (test pattern), then its `oracle_gates` record;
4. truth;
5. tick.

**Command feedback:** the harness passes each tick's `TickOutput.command` back with `Simulator::command(u)`. It takes effect at the next FC step, plus any injected delay.

**Invariants:**
- I1: events are in non-decreasing time, in the order above.
- I2: the plant state passes `quadrotor_state_valid`.
- I3: every period is > 0.
- I4: an identical seed and identical inputs give identical events.

## 7. Faults and randomization (scenario file, drawn from the seed)

**`randomization:`** ±fractions on mass, inertia, drag, and motor τ, plus jitter on the start xy and yaw. Each run logs the values it drew.

**`faults:`** a list of timed events:
- `imu_bias_step`;
- `camera_corrupt` (probability; sets `header.corrupted`);
- `camera_blackout` [t₀, t₁];
- `command_delay_s`;
- `link_loss` [t₀, t₁] (commands dropped);
- `motor_scale` (rotor i delivers a fraction of its commanded thrust).

These are the hooks, and each fault type gets a unit test. The full scenario groups of test plan §5 come with the scenario suite, from #17 on.

## 8. Scoreboard: `sim/score.hpp` and the `score_log` tool

Computed from the logged truth and the course:
- **Gate passages:** a truth segment crosses the gate plane inside the inner rectangle, in sequence order.
- **`lap_time`**.
- **`gate_error`:** true, and estimated from the logged state.
- **`success`:** every gate in order on every lap, no geofence breach, and no collision. A collision is ground contact after takeoff, or a gate-plane crossing outside the opening within a 0.5 m frame margin.

The tool writes one JSON line per run.

## Acceptance

1. **Determinism.**
   - The P0 gate still passes with the dynamics in the loop (null baseline, vehicle at rest on the pad).
   - A new replay gate runs a scripted run with motion: two replays are bit-identical, and so is a rerun from the same seed.
2. **Scripted trajectories.** Scripted `motors` commands through `SimFc` reproduce reflex `QuadrotorModel` trajectories, run directly with the same dt and commands, **bit for bit**. The analytic reflex cases hold to reflex's own tolerances:
   - free fall with drag off (z = z₀ − ½gt²);
   - hover equilibrium at ω_hover;
   - the ground contact holds the vehicle on the pad below hover thrust.
3. **Closed-loop smoke through the rate loop.**
   - A `thrust_rates` script commands takeoff at T > mg, then hover at T = mg with zero rates. The vehicle leaves the pad, and its body rates settle to within 0.05 rad/s.
   - The command-timeout test shows the rotors going to 0 within one FC step after the 0.1 s timeout.

Every test runs in double. Unit and contract tests cover each invariant above. The gyro and accelerometer noise statistics are checked against σ = d·√f, within a derived sampling bound.

## Decisions (approved)

1. **Commands reach the simulated FC through the harness**, which taps `TickOutput.command`, not through a `SimFc` slot. The baseline's `fc_bridge` stays unchanged in SITL (SYS-07). `SimFc` becomes the simulated FC on the far side of the link, and the architecture's slot table is updated to say so.
2. **The FC reads the simulated IMU's gyro through a Butterworth low-pass**, not the true rates.
3. **Uniform matched-σ noise, with no libm calls**, so logs stay bit-reproducible.
4. **Truth is logged at 1 kHz**, about 1 cm of gate-crossing resolution at 10 m/s, and about 6 MB per minute.
5. **Sim channels are logged, not fed to the stack**, in #16. Feeding truth into slots is #17.
6. **Faults are hooks plus six types.** The scenario suite comes later.

## Implementation notes

Points where the implementation fills in or departs from the plan above, for review.

1. **Header-only camera records.** Camera records carry the frame header with no pixels by default, because the stack has no image consumer before rendering lands (L3). Test-pattern images at RP-1's 820 × 616 and 90 Hz would add about 45 MB per second of log. The scenario key `sim.test_pattern_frames: true` restores them. The oracle projection is on its own channel either way.
2. **Event-order wording.** The plant reaches t before any event at t is produced. The tick at t therefore drives the plant step from t, and the truth record at t is the state at t.
3. **A `sim_config` channel (6).** It holds the run's drawn randomization as YAML, at t = 0. The log header is unchanged, and so is the log format version (the new channels need no format change).
4. **No Universal pin.** racing-drone needs only MTL5, which reflex brings at 5.20.0. Universal joins when a component needs it.
5. **The HIL harness is unchanged.** `flight_stack --scenario` (HIL) still uses `SyntheticSensors`: in HIL the real FC is in the loop, and its simulated sensors arrive with the HIL rig (Phase 4).
6. **A determinism fixture.** The `scripted-hop` baseline (`ScriptedHopController`, an open-loop hop) is a test fixture, not a racing baseline. The new gate `gate.sitl.determinism` runs it on `scenarios/scripted-hop.yaml`, with randomization, IMU noise and bias, camera corruption and an IMU bias step. It checks that a rerun and two replays are byte-identical.
7. **Optional RP-1 IMU values.** The file gets bias and latency values marked as placeholders, like its other sensor values.
8. **Ground and collisions.** The ground plane is at the start pad's height. The scorer counts a crossing of any gate's plane outside the opening but within 0.5 m of it as a frame hit, and ground contact after takeoff, before the run is complete, as a crash.
