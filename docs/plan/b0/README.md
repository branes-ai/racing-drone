# Baseline B0 `gt-pid` and gate P1

**Status:** Approved 2026-10-26, implemented ([racing-drone#20](https://github.com/branes-ai/racing-drone/pull/20))
**Issue:** [racing-drone#17](https://github.com/branes-ai/racing-drone/issues/17), part of the Phase 1 epic [racing-drone#8](https://github.com/branes-ai/racing-drone/issues/8)
**Related:** [implementation plan](../implementation-plan.md) Phase 1, [test plan](../test-plan.md) §5–§6, [SITL plan](../sitl/README.md), [architecture](../../arch/framework-architecture.md), [ADR 0001](../../adr/0001-baseline-composition-framework.md)

B0 is the first baseline that flies the course. It answers "can the stack fly the course at all?" with perfect state from simulation. It is the upper bound for later baselines, and the scoreboard's first real row.

## Issues and pull requests

### This plan

| Item | What | State |
|---|---|---|
| [racing-drone#17](https://github.com/branes-ai/racing-drone/issues/17) | This plan: B0 `gt-pid` baseline and gate P1 | Closed |
| [racing-drone#20](https://github.com/branes-ai/racing-drone/pull/20) | The implementation; this document landed with it | Merged |

### What it builds on (merged)

| Item | What it provides here |
|---|---|
| [racing-drone#16](https://github.com/branes-ai/racing-drone/issues/16) / [racing-drone#18](https://github.com/branes-ai/racing-drone/pull/18) | The SITL harness, the truth channel, `SimFc`, faults and randomization, and the scoreboard ([SITL plan](../sitl/README.md)) |
| [reflex#20](https://github.com/branes-ai/reflex/issues/20) / [reflex#31](https://github.com/branes-ai/reflex/pull/31) | `PositionController`, `AttitudeController`, and `cascade_config_from`: B0's controller |
| [reflex#18](https://github.com/branes-ai/reflex/issues/18) / [reflex#24](https://github.com/branes-ai/reflex/pull/24), [reflex#30](https://github.com/branes-ai/reflex/pull/30) | `QuadrotorModel` at grade C: the plant |

### Known limits, tracked upstream

| Item | Effect here |
|---|---|
| [stillwater-sc/mtl5#652](https://github.com/stillwater-sc/mtl5/issues/652) | The reflex cascade does not build with `-fno-exceptions`; no effect on SITL |

## What the evidence can claim

Success on the reflex model (grade C) with perfect state, in simulation (test level L2). This is **design verification of the stack, not validation** against real flight.

## 0. Framework changes (`sdk/`)

**Truth input.**
- `Stack::on_truth(const TruthState<T>&)` passes the true state to any slot that declares `process_truth(...)`. An optional concept `TruthConsumer` detects it, so existing slots are unaffected.
- `Runner` feeds the `truth` channel to the stack, so replay reproduces B0 exactly.
- The flight harness has no truth channel, so a truth-based slot never receives data there.

**Slot construction from config.**
- Today `Stack(map)` default-constructs every slot. B0's controller needs the vehicle file (mass, thrust, gains).
- New: `SlotContext{const VehicleConfig&, const BaselineConfig&, const CourseMap<double>&}`. A slot type constructible from a `SlotContext` is built from it; any other slot is still default-constructed. The choice is made per slot at compile time.
- The `sitl`, `replay` and `flight_stack` harnesses build `Composition<double>(map, ctx)`. Replay takes the vehicle from the log header.

## 1. `GroundTruthEstimator<T>` (racing-drone; oracles live here, ADR 0001)

- **Model:** `last_`, an optional `TruthState`.
- **Invariant:** I1: `last_`, if set, passes `valid(TruthState)`.
- **`process_truth(x)`:** stores `x`.
- **`state_at(now)`:** with `last_` set, a `StateEstimate` with position, velocity, attitude and body rates from `last_`, `t_valid = last_.t`, zero biases and zero covariance. Otherwise the vehicle at rest, as `NullEstimator` reports. Post: `valid()`.
- **`health()`:** `ok` iff truth has been received; `staleness_ns` is the time since the last truth sample.
- **IMU and frame input:** ignored. SITL only (SYS-02).

## 2. `WaypointPlanner<T>`: carrot-following along a gate path

**Path.**
- The pad, then a climb point above the pad at gate height.
- For each lap and each gate in sequence: a pre-point (center − 2 m along the gate normal), the gate center, and a post-point (center + 2 m). Every crossing is straight along the gate normal, through its center, away from the frame.
- It ends holding at the final post-point.

**Model.**
- `path_`: a fixed array of waypoints, at most 2 + 3·kMaxGateSequence·laps.
- `segment_`: the index of the current segment.
- Parameters: `cruise_speed`, `lookahead`, `corner_speed`.

**Invariants.**
- I1: `segment_` lies in [0, path size − 1).
- I2: `segment_` never decreases.

**`plan(map, x)`.**
- Project x onto the current segment; past its end, advance `segment_` (monotonic, so it cannot skip back).
- The setpoint is the point `lookahead` along the path from the projection. The velocity feed-forward is the path direction × speed: `cruise_speed`, reduced to `corner_speed` within 1.5 m of a corner sharper than 60°, and 0 at the final hold.
- The yaw is the path heading.
- One `ReferenceSample`, with `progress` = arc length.

**Placement.** The architecture lists `WaypointPlanner` under cortex. It lives in racing-drone for B0 and moves to cortex when the planners land there (Phase 2).

## 3. `CascadedPid<T>`: the reflex cascade in the `Controller` slot

- **Adapts** reflex `PositionController` and `AttitudeController`, configured by `cascade_config_from(params)` from the vehicle file, with B0's speed limit.
- **Output:** collective thrust and body-rate setpoints. The rate loop belongs to the flight controller (`SimFc` in simulation), so the slot output is the thrust-and-rates command it already defines.
- **Model:** the two reflex controllers and the previous tick's time, for dt.
- **Invariants:**
  - I1: the cascade configuration is valid (needs thrust-to-weight > 1.25; RP-1 is about 5);
  - I2: thrust lies in [T_min, T_max].
- **`step(x, ref, conflicts)`:**
  - dt = x.t_valid − the previous tick's time. On the first tick or when dt ≤ 0, it holds the last command, as `Pid` does.
  - The position loop runs on the reference sample (position, velocity feed-forward, yaw); the attitude loop gives the rate setpoints.
  - It outputs `ThrustRates{thrust, rates}`.
  - `conflicts` is unused (no DAA in B0).
- **Parameters** in `baselines/b0-gt-pid.yaml`: `cruise_speed_m_s: 4.0`, `max_speed_m_s: 4.5`, `lookahead_m: 1.0`, `corner_speed_m_s: 2.5`, `control_rate_hz: 250`.

## 4. Baseline, scenario, and gate P1

- **`baselines/b0-gt-pid.yaml`:** `GroundTruthEstimator`, `NullGateDetector`, `WaypointPlanner`, `CascadedPid`, `NullSafetyFilter`, `NullDaaTracker`, `NullFcBridge`.
- **`scenarios/b0-nominal.yaml`:** `a2rl-2025`, two laps, 120 s. Nominal randomization:
  - mass ±5 %;
  - inertia ±10 %;
  - drag ±10 %;
  - motor τ ±10 %;
  - start ±0.2 m and ±0.05 rad.

  The test plan does not quantify the nominal group; these values are this plan's. "Stress" would double each.
- **Scoreboard:** a `max_speed_m_s` metric. `success` also requires the run's maximum true speed ≤ the scenario's speed cap (5 m/s for P1).
- **`gate_p1` tool:** runs N seeds in-process without writing logs. It prints a summary (success rate, lap time p50 and p95, gate error p50 and p99, max speed, failures by reason) and exits nonzero below the threshold.
- **CTest:**
  - `gate.p1.smoke`: 10 seeds, on every PR (the test plan's L2 PR tier);
  - `gate.p1.full`: 500 seeds, ≥ 99 %, label `nightly`, excluded from the default run, and run on main by a scheduled `nightly.yml` workflow.
- **Determinism:** one seed of B0 also runs through the SITL determinism gate (rerun and replay byte-identical), as the first stateful stack.

## Acceptance (stated before any test runs)

| Test | Pass if |
|---|---|
| Gate P1 (test plan §6) | B0 completes two laps on `a2rl-2025` at ≤ 5 m/s in ≥ 99 % of 500 nominal seeds |
| PR smoke | 10 of 10 seeds succeed |
| Determinism | A B0 run reruns and replays byte-identically |
| Unit and contract tests | Each invariant; the planner's monotonic progress and straight gate approach; the estimator's pass-through and health; the controller's dt ≤ 0 hold; slot construction from `SlotContext` |

Reported, not gated: lap times, true gate error, and failure reasons.

## Decisions (approved)

1. **Slots are built through `SlotContext`**, detected per slot at compile time. A parameters-only hook would still leave the controller without the vehicle mass.
2. **`WaypointPlanner` lives in racing-drone for now**, not in cortex as the architecture table lists it.
3. **The nominal randomization values** of §4.
4. **P1 gate tiers:** 10 seeds on every PR, 500 nightly, with a new nightly workflow.
5. **B0's position and attitude loops run at 250 Hz**; `SimFc`'s rate loop runs at 1 kHz.

## Implementation notes

Points where the implementation fills in or departs from the plan above, for review.

1. **A factory, not a constructor.** A slot is built from the context by `static result<S, std::string> from_context(const SlotContext&)` (concept `ContextBuilt`), so a bad vehicle or parameter is an error message, not an assert. `Stack::create(map, ctx)` returns `result<Stack, std::string>` with the first slot's error, and `harness::make_stack()` wraps it for the harnesses. They build the stack before opening `--out`, so a bad baseline file leaves an existing log untouched.
2. **Where the slots live.**
   - `GroundTruthEstimator` and `WaypointPlanner` are in `sdk/include/branes/racing/slots/` (`ground_truth.hpp`, `waypoint.hpp`). They need no platform library.
   - `CascadedPid` needs reflex, so it lives in a new header-only target `branes::racing_glue` (`baselines/glue/`), which every harness links.
   - The vehicle-file → `QuadrotorParams` mapping moved there too (`glue/reflex_params.hpp`, templated on the scalar type). `sim/vehicle.hpp` now calls it, so the plant and the controller read the vehicle file the same way.
3. **Planner parameters.** The plan's `lookahead_m: 1.0` became 0.3, and a `switch_radius_m` (1.0) was added. Both were derived before the first run:
   - The position loop adds K_p·L to the along-path velocity setpoint. For RP-1, K_p = 1/(32 τ) = 1.5625 s⁻¹ (reflex `cascade_config_from`, τ = 0.02 s). With L = 1 m, the 4 m/s cruise would ask for 5.56 m/s and run against the 4.5 m/s cap all the time. With L = 0.3 m it asks for 4.47 m/s.
   - The switch radius must exceed the lookahead. A vehicle converging on a carrot held at a segment's end stops `lookahead` short of it, and at a turn of 90° or more (the climb corner) it never passes the end either. The segment advances once the vehicle passes the end or comes within the switch radius.
   - The corner radius (1.5 m), corner angle (60°) and gate approach (2 m) are parameters too, at the plan's values.
4. **The path starts where the vehicle is.** The first `plan()` lays the path out from the estimate's position, so the climb is straight up from the actual (jittered) start, not from the course's nominal start pose.
5. **Estimator staleness.** `health()` takes no time argument, so `staleness_ns` is measured on the estimator's input clock: the latest IMU or truth time minus the latest truth time.
6. **Controller details.**
   - An empty reference also holds the last command, and a held command is re-stamped with the estimate's time.
   - The position loop gets no collective-saturation feedback, because the mixer's report stays on the FC side of the link.
   - `max_speed_m_s` caps the horizontal setpoint and lowers the up and down caps only where it is below reflex's (3 and 2 m/s).
7. **Truth through the Runner.** An undecodable or invalid truth record is now an error, as for every other input channel; before #17 the channel was skipped.
8. **The batch tool is `sitl_batch_<baseline>`, not `gate_p1`.**
   - It is a fourth harness, built for every baseline, so later gates reuse it.
   - Run i uses seed = the scenario's seed + i, so any run reproduces alone with `sitl_<baseline>` after setting that seed.
   - A run stops once it is complete or has failed.
   - `--jobs` runs seeds on worker threads; the scores do not depend on it.
9. **The speed limit is scenario data.** It is a new optional scenario key, `speed_limit_m_s`. `score_log` takes `--speed-limit`, because the log header does not hold the scenario. Max speed is measured from takeoff until the run is complete.
10. **Nightly registration.** `gate.p1.full` is registered only with `-DRACING_NIGHTLY_TESTS=ON`, which `nightly.yml` sets. It also carries the label `nightly`, so the default `ctest` never runs it.
11. **A short determinism scenario.** `gate.b0.determinism` runs `scenarios/b0-determinism.yaml`: 8 s through takeoff and the first gates, with the nominal randomization and a camera-corruption fault. It uses the same gate script as scripted-hop (`tests/gate/sitl_gate.cmake`, now with a `SCENARIO` input). A short run keeps the four compared logs small.
