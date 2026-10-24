# Racing drone framework architecture

**Status:** Draft
**Date:** 2026-10-23
**Related:** [requirements](../requirements/mission-requirements.md), [state of the art](../assessments/state-of-the-art.md), [ADRs](../adr/README.md)

## 1. Idea: a racing drone is a *baseline* plugged into fixed slots

`racing-drone` contains no algorithms (README: "Algorithms do not live here"). It defines:

1. **Slots**: C++20 concepts for each component role (estimator, gate detector, planner, controller, DAA, pilot arbiter).
2. **Data contracts**: the plain structs passed between slots, with timestamps and frames defined once.
3. **Descriptions**: vehicle, course, and scenario files.
4. **Baselines**: named compositions that fill each slot with an implementation from cortex, reflex, or synapse, plus their parameters.
5. **Harnesses**: SITL, replay, HIL, and the flight executable. They run *any* baseline unchanged (SYS-07).
6. **Scoreboard**: one metrics schema, so every baseline is compared on the same scenarios.

A new racing baseline (a new estimator, a learned controller, an extra sensor) is a new composition plus, where needed, a new component in a platform repo. Nothing else changes.

## 2. Runtime data flow

```text
            nav camera 90 Hz        IMU 1-2 kHz            RC (CRSF)     FPV cam
                 |                      |                      |            |
   COMPANION     v                      |                      |            v
  +--------------+-----------+          |                      |      +-----------+
  | perception (cortex +     |          |                      |      | FPV VTX   |--> pilot
  | synapse)                 |          |                      |      | (+OSD)    |    goggles
  |  GateNet + DroneNet      |          |                      |      +-----^-----+
  |  -> gate corners         |          |                      |            | MSP
  |  -> intruder detections  |          |                      |            | DisplayPort
  +-----+--------------+-----+          |                      |            |
        |              |                |                      |            |
        v              v                v                      |            |
  +-----+-----+  +-----+------+   IMU (relayed by FC)          |            |
  | estimator |  | DAA tracker|<------+                        |            |
  | VIO + gate|  | IMM-EKF    |       |                        |            |
  | PnP + EKF |  +-----+------+       |                        |            |
  +-----+-----+        | intruder     |                        |            |
        | state        | predictions  |                        |            |
        v              v              |                        |            |
  +-----+--------------+------+       |                        |            |
  | planner (cortex)          |       |                        |            |
  | time-optimal reference,   |       |                        |            |
  | perception-aware          |       |                        |            |
  +-----+---------------------+       |                        |            |
        | reference                   |                        |            |
        v                             |                        |            |
  +-----+---------------------+       |                        |            |
  | MPCC (reflex) 100-200 Hz  |       |                        |            |
  | + DAA separation          |       |                        |            |
  |   constraints             |       |                        |            |
  +-----+---------------------+       |                        |            |
        | thrust + body rates         |                        |            |
        v                             |                        |            |
  +-----+---------------------+       |                        |            |
  | safety filter (reflex)    |       |                        |            |
  | CBF, geofence, envelope   |       |                        |            |
  +-----+---------------------+       |                        |            |
        | serial >= 500 Hz            |                        |            |
  ======|=============================|========================|============|=====
   FC   v (MCU, hard real time)       |                        |            |
  +-----+-----------------------------+------------------------+------------+--+
  | mode manager / pilot arbiter (reflex): MANUAL | ASSISTED | AUTONOMOUS | KILL|
  | attitude estimate, INDI + rate loop 2-8 kHz, mixer, DShot (RPM telemetry)   |
  | failsafes: companion heartbeat, RC loss, kill switch                        |
  +-----------------------------------------------------------------------------+
                                       |
                                    4 x ESC
```

**Rules:**

- **Hot path.** Camera → perception → estimator → MPC → safety filter → FC. It runs in one companion process with pre-allocated, lock-free single-producer queues. There is no middleware hop on the hot path ([ADR 0006](../adr/0006-hot-path-in-process-zenoh-off-path.md)).
- **Off the hot path.** Telemetry, logs, OSD content, the ground station, and parameter upload go over Zenoh (cortex ADR 0014).
- **The FC never depends on the companion to stay safe** (reflex principle). Kill, RC takeover, and failsafes are FC-only paths.

## 3. Slots

Every slot is a concept in `branes::racing`, templated on the scalar type (cortex ADR 0015). Implementations live in platform repos. A sketch:

```cpp
namespace branes::racing {

template <class E, class T>
concept Estimator = requires(E e, const ImuSample<T>& imu, const GateObservations<T>& gates,
                             const CameraFrame& frame, Timestamp now) {
    e.process_imu(imu);
    e.process_frame(frame, gates);        // gates may be empty (no detection / corrupted frame)
    { e.state_at(now) } -> std::same_as<StateEstimate<T>>;   // delay-compensated (EST-06)
    { e.health() } -> std::same_as<EstimatorHealth>;         // NIS, staleness (SAF-03)
};

template <class D, class T>
concept GateDetector = requires(D d, const CameraFrame& frame, const CourseMap<T>& map,
                                const StateEstimate<T>& prior) {
    { d.detect(frame, map, prior) } -> std::same_as<GateObservations<T>>;
};

template <class P, class T>
concept Planner = requires(P p, const CourseMap<T>& map, const StateEstimate<T>& x) {
    { p.plan(map, x) } -> std::same_as<Reference<T>>;        // may reuse the previous plan
};

template <class C, class T>
concept Controller = requires(C c, const StateEstimate<T>& x, const Reference<T>& ref,
                              const ConflictSet<T>& conflicts) {
    { c.step(x, ref, conflicts) } -> std::same_as<ThrustRates<T>>;   // MPCC or learned policy
};

template <class F, class T>
concept SafetyFilter = requires(F f, const StateEstimate<T>& x, const ThrustRates<T>& u,
                                const ConflictSet<T>& conflicts, const Geofence<T>& fence) {
    { f.filter(x, u, conflicts, fence) } -> std::same_as<ThrustRates<T>>;
};

template <class A, class T>
concept DaaTracker = requires(A a, const IntruderDetections<T>& det, const StateEstimate<T>& x) {
    a.update(det, x);
    { a.conflicts(x) } -> std::same_as<ConflictSet<T>>;      // predictions over the horizon
};

}  // namespace branes::racing
```

| Slot | First implementations | Repo |
|---|---|---|
| `Estimator` | `GroundTruth` (SITL only), `VioGateEkf` (MSCKF + gate PnP + drag model), `GateEkf` (MonoRace-style) | racing-drone (GT), cortex |
| `GateDetector` | `OracleGates` (SITL projection), `GateNetCorners` | racing-drone, cortex + synapse |
| `Planner` | `WaypointPlanner`, `TimeOptimalSampler`, `OfflineCpc` | cortex |
| `Controller` | `CascadedPid`, `Mpcc`, `TrackingNmpc`, `LearnedPolicy` | reflex (+ synapse for the policy) |
| `SafetyFilter` | `PassThrough`, `CbfFilter` | reflex |
| `DaaTracker` | `None`, `OracleIntruders` (SITL), `ImmTracker` | racing-drone, cortex |
| FC firmware | `BetaflightBridge` (interim), `ReflexFc` | racing-drone, reflex |

A **learned controller** is a `Controller`, so it receives the same state and reference and is scored the same way. MonoRace's motor-level policy bypasses the FC rate loop. It is supported as a `Controller` with a `motor_commands` output variant that the FC accepts only in a configured mode.

## 4. Data contracts

Defined once in `branes::racing::msg`. They are plain `repr(C)`-compatible structs, so they can cross the Rust core and Zenoh unchanged (cortex ADR 0010).

| Struct | Contents | Frame / time |
|---|---|---|
| `ImuSample` | Gyro, accelerometer, saturation flags, temperature | Body frame. FC timestamp, mapped to companion time |
| `CameraFrame` | Image view (non-owning), exposure start and duration, per-row readout time, corruption flag | Sensor timestamp at mid-exposure |
| `GateObservations` | Per gate: id hypothesis, 4 corners (px) + covariance, mask quality | Image frame |
| `StateEstimate` | p, v, q, ω, biases, covariance, validity time | Course frame (gate-map origin) |
| `Reference` | Time- or progress-parameterized states, gate passing times | Course frame |
| `ThrustRates` | Collective thrust, body rates (or per-motor commands) | Body frame |
| `IntruderDetections` / `ConflictSet` | Bearing, size, range estimate; predicted tubes over the horizon | Course frame |

## 5. Descriptions (files)

```text
vehicles/rp1.yaml        mass, inertia, arm geometry, rotor thrust/torque maps, motor time constant,
                         drag coefficients, IMU noise and ranges, camera intrinsics, extrinsics, shutter
courses/a2rl-2025.yaml   gates (pose, inner size, order), laps, start pad, flight volume, geofence
scenarios/*.yaml         course + vehicle + randomization (gate pose noise, lighting, wind, sensor
                         faults, intruder trajectories) + seed
baselines/b2-vio-gate-mpcc.yaml
                         slot -> implementation + parameter files
```

One loader validates every file against a schema at startup. A config change means a restart, with no live reconfiguration (cortex ADR 0005).

## 6. Compute and timing map

| Domain | Task | Rate | Hardware |
|---|---|---|---|
| FC | IMU sampling, gyro filtering, INDI + rate loop, mixer, DShot, RC, failsafes, attitude estimate | 2–8 kHz | STM32H7 |
| Companion RT core | IMU propagation, estimator, MPC, safety filter, FC link | IMU rate / 100–200 Hz | CPU (isolated core, SCHED_FIFO) |
| Companion accelerator | GateNet + DroneNet (shared backbone) | 90 Hz | GPU / KPU via synapse |
| Companion best effort | VIO front end, planner replan, DAA tracker | 30–90 Hz | CPU |
| Companion background | Logging (MCAP-style), Zenoh telemetry, OSD composition | 10–50 Hz | CPU |

## 7. Mode manager and pilot arbitration

The arbiter lives on the FC (reflex), because it must work when the companion is dead.

```text
           arm switch                   autonomy switch + preflight OK
DISARMED ----------------> MANUAL ------------------------------------> AUTONOMOUS
   ^                         ^  ^                                          |  |
   | kill (any state)        |  +---- pilot takeover (switch or stick) ----+  |
   +-------------------------+                                                |
                             +--- HOLD <--- estimator unhealthy / geofence ---+
                                   |
                                   v
                                  LAND --> DISARMED
```

- **Takeover.** Moving the stick beyond the deadband in `AUTONOMOUS` hands control to the pilot (configurable). The rate-loop integrators are carried over, so the transfer is bumpless (FPV-04).
- **ASSISTED.** Pilot setpoints pass through the companion's safety filter, so the envelope and DAA apply to human flight.

## 8. DAA layering

```text
camera --> DroneNet --> ImmTracker --> ConflictSet --+--> MPCC constraints     (strategic: keep racing)
                                                     |
                                                     +--> CBF safety filter    (tactical: guaranteed separation)
                                                     |
                                                     +--> OSD alert to pilot   (situational awareness)
```

The CBF filter runs every control cycle in bounded time. It changes the MPC output only when the predicted separation would fall below the threshold, so a stalled or diverged MPC cannot remove the DAA guarantee.

## 9. Repository layout

```text
racing-drone/
├── cmake/deps.cmake          # pins: cortex, reflex, synapse, MTL5, Universal, Catch2 (same pattern as siblings)
├── sdk/include/branes/racing/
│   ├── slots/                # the concepts in §3
│   ├── msg/                  # the data contracts in §4
│   ├── config/               # vehicle / course / scenario / baseline loaders
│   ├── mission/              # race state machine: preflight, arm, start, gate sequence, finish, land
│   └── compose/              # baseline wiring (compile-time composition)
├── baselines/                # one YAML + optional glue per baseline (b0 … b4)
├── vehicles/  courses/  scenarios/
├── sim/                      # SITL harness: reflex dynamics + renderer bridge + sensor models + fault injection
├── apps/
│   ├── flight_stack/         # companion executable (runs any baseline)
│   ├── replay/               # log replay through any baseline
│   └── ground_station/       # Zenoh telemetry, parameter upload
├── fc/                       # FC integration: Betaflight bridge first, reflex firmware later
├── tools/scoreboard/         # metrics → comparison tables / plots
├── tests/                    # unit, component, scenario (Catch2; scenario tests drive the SITL harness)
└── docs/                     # requirements, assessments, arch, plan, adr, sessions
```

Namespace `branes::racing`, CMake target `branes::racing`. C++20, the same CMake presets and CI pattern as the sibling repos.
