# Mixed precision in the racing stack

**Date:** 2026-10-26
**Status:** Analysis and hypotheses. The first measurement is [racing-drone#30](https://github.com/branes-ai/racing-drone/issues/30), the precision sweep of baseline B0. This page gains a Results section when the sweep runs.
**Related:** [cortex ADR 0015](https://github.com/branes-ai/cortex/blob/main/docs/adr/0015-type-generic-arithmetic.md) (the arithmetic type is a template parameter), [architecture](../arch/framework-architecture.md) §3 (slots templated on the scalar type), [B0 plan](../plan/b0/README.md), [test plan](../plan/test-plan.md)

## 1. The question

Most sensor measurements carry 3–4 significant digits, about 1 part in 1,000 to 10,000. Actuators resolve no better: DShot's throttle is an 11-bit value, about 2,000 steps. If the inputs and outputs of estimation and control carry only three or four digits, do we need to compute in `double` precision, with its 53-bit significand? Are there opportunities to improve performance and/or energy efficiency by using mixed precision?

The short answer: a computation's required precision isn't set by its data precision. For most of the flight stack, the computational precision requirements rule out existing 16-bit formats like fp16 and bf16, which are now readily available in many GPUs and NPUs. Furthermore, these requirements don't apply equally to every component. That unevenness is the case for choosing the arithmetic **per component**, and for building on libraries that make that choice cheap to study.

Statements that are our own judgment, not a derivation or a cited fact, are marked *(assessment)*.

## 2. What sets the precision a computation needs

A rough budget for the digits a computation needs:

> digits needed ≈ digits in the data + log₁₀(condition number of the operations) + digits lost to accumulation, with exponent range enough for every intermediate value

### 2.1 Representation: the format must hold the data

A format with fewer significand digits than the measurement throws away information on the first conversion.

- bfloat16 has an 8-bit significand: 1 part in 256, or 2.4 digits. That is **below** the 3–4 digits the question assumes.
- binary16 has 11 bits: 1 part in 2,048, or 3.3 digits. That is at the low end of the range.
- MEMS IMUs typically digitize to 16 bits *(assessment)*. Even the DShot throttle, an 11-bit value inside a 16-bit frame, needs all 11 significand bits of binary16.

### 2.2 Accumulation: small increments into large totals

Most of a control stack's transformations are integration operators: positions from velocities, integrators from errors, attitudes from rates, progress along a path. When an increment `d` is added to a total `x`, round-to-nearest returns `x` unchanged if `|d| < ½ ulp(x)`. The increment is lost, and so is the information it carried. What matters is the ratio of the increment to the total, not the measurement precision.

Examples at this stack's magnitudes. The physics step is 1 kHz and the control step is 250 Hz (`ScenarioConfig::physics_rate_hz`, `baselines/b0-gt-pid.yaml`). The A2RL course spans x ∈ [0, 76] m (`courses/a2rl-2025.yaml`). ULPs are from Table 2.

- **Position at 4 m/s, integrated at 1 kHz,** grows by 4 mm per step.
  - In bfloat16 at B0's start position (x ≈ 3.2 m, y ≈ 4.0 m), the ULP is 15.6–31.3 mm. The integration step is less than half an ULP, so the integrated position stalls.
  - In binary16 between 32 and 64 m, the ULP is 31.3 mm, and the same accumulation stall occurs.
- **Position at a 250 Hz control step** grows by 16 mm at the 4 m/s cruise and 10 mm at the 2.5 m/s corner speed.
  - In binary16 between 32 and 64 m, ½ ULP is 15.6 mm.
  - At corner speed, the 10 mm step is lost entirely.
  - At cruise speed, the 16 mm step rounds **up** to a full 31.3 mm ULP, so the integrated speed is 1.95× the true one.
  - Both errors come from rounding alone.
- **A controller integrator** with a 1 cm error, k_i = 1 and dt = 4 ms gains 4e-5 per tick *(illustrative values)*. Near 0.5 the ULP is 4.9e-4 in binary16, 1.2e-4 in `posit<16,1>`, and 3.9e-3 in bfloat16. The increments vanish in all three; the integrator stops integrating, and the steady-state error it exists to remove comes back. reflex's cascade gives each velocity loop an integral bound of g/2 for a 33 % hover-thrust error (`cascade.hpp`), so the integral term is large precisely when it matters.

Rounding errors that aren't lost compound instead. Re-normalizing a quaternion every step with relative error ε random-walks the attitude by about √N·ε. For binary16 (ε ≈ 1e-3), that is about 0.3 rad after 100,000 steps, or 0.3 rad error in under two minutes of flight (100k samples at 1 kHz = 100 s).

### 2.3 Conditioning: steps that magnify rounding error

An operation with condition number κ can magnify relative input error by κ, costing about log₁₀κ digits. The worst place in this stack will be the estimator's covariance, once B1 replaces ground truth with an EKF ([ADR 0004](../adr/0004-gate-relative-hybrid-estimation.md)):
- Its diagonal spans eight orders of magnitude or more. Position variance is around 1e-2 m², and gyro-bias variance around 1e-10 (rad/s)² *(typical values, assessment)*.
- The update `P = (I − KH)P` subtracts nearly equal quantities.
- Even in `double`, practical filters use the Joseph form or square-root forms to keep `P` symmetric positive definite (SPD).
- Single-precision filters exist and fly: PX4's EKF2 is written in single precision *(assessment)*. They survive only because they were formulated carefully, and that care is part of the precision budget.

Geometry solvers in the gate detector (multi-gate PnP) and the QP or NLP solvers of an MPC planner ([ADR 0002](../adr/0002-companion-mpc-fc-rate-loop.md), [ADR 0003](../adr/0003-mpc-primary-learned-policy-slot.md)) have the same character.

### 2.4 Range: every intermediate value must fit

Precision and range are separate failures:
- **binary16 fails on range.** Its smallest normal is 6.1e-5 and its largest value 65,504.
  - The bias variances above fall below the smallest positive subnormal, 6.0e-8, and underflow to zero. The filter becomes certain of its biases and stops learning them.
  - RP-1's thrust coefficient, k_f = 8.9e-7 N/(rad/s)², is below the smallest normal. It survives only as a subnormal with about 4 significant bits (8.9e-7 / 2⁻²⁴ ≈ 15).
    - The value is the current placeholder in `vehicles/rp1.yaml`, chosen for about 12 N at 3,665 rad/s, not a measured one.
    - The conclusion doesn't hinge on it. In SI units, a rotor this size has k_f = T/ω² on the order of 1e-6 to 1e-7, because thrusts in newtons come from speeds of thousands of rad/s *(assessment)*. That is well below binary16's normal range.
  - reflex's parameter validation (`quadrotor_params_valid`, invariant I1) rejects subnormal parameters because their reciprocals overflow. So RP-1 can't be configured in binary16, and reflex's `tests/sdk/quadrotor_model.cpp` checks exactly that.
  - The battery capacity, 1e5 J, overflows.
- **bfloat16 fails on precision.** It has binary32's range but only an 8-bit significand.
- **Posits trade one for the other.** They taper: more fraction bits near 1 and fewer at the extremes. The trade suits values that cluster near 1, and penalizes large course-frame coordinates (Table 2).

Time is the only quantity this repo fixes. `Timestamp` is int64 nanoseconds. As binary16 seconds, t = 120 s has an ULP of 62.5 ms, so a 4 ms tick couldn't be represented. As binary32 seconds, t = 3,600 s has an ULP of 0.24 ms.

## 3. The formats

**Table 1.** Format definitions: significand bits include the hidden bit; posit fraction bits vary with the value. The smallest-positive column is the smallest normal for the IEEE-layout `cfloat` formats, and the smallest positive value for posits (minpos), lns and fixpnt, which have no normal/subnormal split.

| Format | Bits | Significand | ε (relative step at 1) | Smallest positive | Largest |
|---|---|---|---|---|---|
| binary64 (cfloat<64,11> or double) | 64 | 53 | 2.220446e-16 | 2.225074e-308 | 1.797693e+308 |
| binary48 (cfloat<48,8>) | 48 | 40 | 1.818989e-12 | 1.175494e-38 | 3.402824e+38 |
| binary40 (cfloat<40,8>) | 40 | 32 | 4.656613e-10 | 1.175494e-38 | 3.402824e+38 |
| binary32 (cfloat<32,8> or float) | 32 | 24 | 1.192093e-07 | 1.175494e-38 | 3.402823e+38 |
| binary24 (cfloat<24,8>) | 24 | 16 | 3.051758e-05 | 1.175494e-38 | 3.402772e+38 |
| bfloat16 (cfloat<16,8> or bfloat) | 16 | 8 | 7.812500e-03 | 1.175494e-38 | 3.389531e+38 |
| binary16 (cfloat<16,5> or FP16) | 16 | 11 | 9.765625e-04 | 6.103516e-05 | 6.550400e+04 |
| posit<48,2> | 48 | up to 44 | 1.136868e-13 | 4.078315e-56 | 2.451993e+55 |
| posit<32,2> | 32 | up to 28 | 7.450581e-09 | 7.523164e-37 | 1.329228e+36 |
| posit<24,2> | 24 | up to 20 | 1.907349e-06 | 3.231174e-27 | 3.094850e+26 |
| posit<16,2> | 16 | up to 12 | 4.882813e-04 | 1.387779e-17 | 7.205759e+16 |
| posit<16,1> | 16 | up to 13 | 2.441406e-04 | 3.725290e-09 | 2.684355e+08 |
| lns<32,24> | 32 | 24 | 4.13148e-08 | 5.42101e-20 | 1.84467e+19 |
| lns<16,8> | 16 | 8 | 0.00271028 | 5.43571e-20 | 1.83969e+19 |
| fixpnt<32,16> | 32 | 16 | 0.0000152587890625 | 0.0000152587890625 | 32767.9999847412109375 |
| fixpnt<16,8> | 16 | 8 | 0.00390625 | 0.00390625 | 127.99609375 |

**Table 2.** ULP at the stack's magnitudes, derived from the format definitions. For posits, the fraction-bit count is in parentheses.

| Value | binary32 | bfloat16 | binary16 | `posit<32,2>` | `posit<16,1>` |
|---|---|---|---|---|---|
| 0.5 (an integrator) | 6.0e-8 | 3.9e-3 | 4.9e-4 | 3.7e-9 (27) | 1.2e-4 (12) |
| 4 m (B0's start) | 4.8e-7 | 3.1e-2 | 3.9e-3 | 3.0e-8 (27) | 2.0e-3 (11) |
| 32 m | 3.8e-6 | 0.25 | 3.1e-2 | 4.8e-7 (26) | 3.1e-2 (10) |
| 64–76 m (course end) | 7.6e-6 | 0.5 | 6.3e-2 | 9.5e-7 (26) | 0.125 (9) |

For scale: a gate's inner opening is 1.5 m, a half-width of 0.75 m.

## 4. The stack, component by component

**Table 3.** What dominates each component's numerics, and the lowest format we expect to be sufficient *(assessment, to be tested)*.

| Component | Numeric character | Dominant mechanism | Expected floor |
|---|---|---|---|
| Simulator, simulated FC, scorer | The reference trajectory and the score | Accumulation (1 kHz integration) | **binary64**, by policy: a reference must be more precise than the system under test |
| Log and replay | Exact reproduction of a run | None: values are stored, not computed | **binary64**, so replay is exact whatever T the stack runs in |
| Time | Ticks and deadlines over a whole flight | Accumulation | **int64 ns** (already) |
| Estimator (B1+ EKF, VIO) | Covariance over ten decades; IMU integration | Conditioning, range, accumulation | binary32 with a careful formulation; `posit<32,2>` is a candidate |
| Gate detector: network | Convolutions over 8-bit pixels | None dominant; trained to tolerate noise | **16-bit or int8**, with 32-bit accumulation (the standard tensor-core design) |
| Gate detector: PnP and corner geometry | Small nonlinear solves | Conditioning | binary32 |
| Planner (waypoints; MPCC) | Course-frame geometry to 76 m; QP solves | Range × precision (positions); conditioning (solver) | binary32. Planner geometry in a gate-relative frame might go lower |
| Controller (cascaded PID; INDI) | Integrators, gains, quaternions | Accumulation | binary32 for integrators and attitude; 16-bit for stateless gain products is plausible |
| Safety filter | Geofence and margin comparisons | Representation: position resolution against the margin | The format of the position it checks |
| Mixer and FC output | Thrust allocation into 11-bit DShot | Representation of the output only | 16-bit is enough for the final stage: nothing accumulates downstream |

Two conclusions follow:
1. **No single format is right for the whole stack.** The floor ranges from int8 in the network to binary64 in the reference. A uniform `T` either wastes energy and bandwidth where the numerics are easy, or fails where they are hard.
2. **The hard spots are few and identifiable:** accumulators, ill-conditioned updates, and wide-range quantities. The hypothesis behind per-component precision is that keeping only those in 32 bits lets most of the arithmetic run at lower precision.

## 5. Hypotheses

Each hypothesis comes with a prediction stated before any measurement. [#30](https://github.com/branes-ai/racing-drone/issues/30) tests H1–H4 on B0 and gate P1, with 500 `b0-nominal` seeds and ≥ 99 % success. The simulator stays in binary64, and only the stack runs in T.

- **H1: 32 bits suffice for B0.** `float`, `cfloat<32,8>` and `posit<32,2>` pass P1.
  - *Basis:* the ULP at 76 m is ≤ 7.6e-6 m, five orders below the gate half-width. Integrator increments of order 1e-5 are two orders above binary32's ½ ULP at 0.5 (3e-8).
  - *What would refute it:* any 32-bit type failing P1.
- **H2: bfloat16 fails on precision.** The stack constructs, but fails P1 by tracking error.
  - *Basis:* position ULPs of 0.25–0.5 m on most of the course, against a 0.75 m gate half-width. Integrators stall below ½ ULP (1.95e-3 at 0.5).
- **H3: binary16 fails on range, before flight.** `reflex_params_from<binary16>` rejects RP-1, so the stack doesn't construct.
  - *Basis:* k_f = 8.9e-7 (the placeholder in `vehicles/rp1.yaml`) is below the smallest normal, 6.1e-5, and so it's subnormal. reflex's validator rejects subnormal parameters; the check reflex already makes for its model.
  - *What would refute it:* the stack constructing. That alone wouldn't say why: whether k_f was represented or validated differently than predicted needs its own check.
- **H4: `posit<16,1>` fails, and the failure grows with distance from the origin.**
  - *Basis:* 12 fraction bits near 1 is better than binary16's 10. But at 64 m it has 9 bits, an ULP of 0.125 m, and its integrator ULP at 0.5 is 1.2e-4.
  - *Prediction:* it constructs (its range covers k_f), then fails P1. Gate errors are worse at the far end of the course (x > 32 m) than near the start.
- **H5: mixed precision recovers 16 bits.** The configuration:
  - **binary32:** the accumulators (controller integrators, course-frame positions) and the vehicle parameters and constants (k_f among them);
  - **bfloat16:** all other arithmetic, the stateless products and the gain multiplications.

  Prediction: this B0 passes P1.
  - *Basis:* §2–§4. The failures predicted in H2–H4 come from accumulation and range, not stateless arithmetic.
  - bfloat16 is the 16-bit format here because its range covers every intermediate value. A binary16 variant would also need every small constant kept in 32 bits (H3).
  - Testing it needs precision chosen within and across slots, which is [#36](https://github.com/branes-ai/racing-drone/issues/36) (§6).

The hypotheses can fail in informative ways. If `float` fails H1, the stack has an unidentified conditioning or accumulation problem, and it would hurt binary32 flight hardware too. A failure confined to `cfloat<32,8>` or `posit<32,2>` points at that format, and needs its own analysis. If H2–H4 *pass*, the mechanisms in §2 matter less in closed loop than this analysis predicts, because feedback corrects the error. That is worth knowing on its own.

## 6. Why articulate precision per component

Precision is a cost, not only a correctness question *(assessment)*:
- A multiplier's area and energy grow roughly with the square of its significand width.
- Memory traffic, and so cache footprint and bandwidth, grows linearly with the total width.
- The stack's target hardware runs CPU + GPU today, and CPU + KPU later. The KPU is designed around number formats optimized per operator for energy and accuracy (Cortex ADR 0015).
- A stack locked to binary64 can't use that, and a stack locked to one 16-bit format can't fly (§5).

The framework has the compile-time seams to investigate this, but per-slot precision is a future change to the slot contracts:
- **Slots are templated on the scalar type** (architecture §3, cortex ADR 0015), so a slot's numerics are a compile-time parameter, not a rewrite.
- **Slot boundaries are message contracts** (`StateEstimate`, `Reference`, `ThrustRates`, …). Today one `T` runs through the whole `Stack`, so connected slots share it. Per-slot types therefore need either explicit adapters or cross-type slot contracts at those boundaries, with conversion and rounding rules the follow-up design must define. #30 adds the first such conversions, at the harness boundary, with a ½-ULP round-to-nearest bound.
- **The baseline file names the composition** ([ADR 0001](../adr/0001-baseline-composition-framework.md)). A per-slot scalar type is one more field per slot, giving a baseline such as "B1 with the estimator in `posit<32,2>` and the controller's stateless path in bfloat16" as its own scoreboard row.
- **Replay is exact and the log stays binary64.** Any precision configuration can be compared with any other on the same recorded inputs.

## 7. Why MTL5 and Universal

The experiment and the product share a requirement: the same algorithm source must run unchanged across many number systems, including formats with no hardware support on the development host. For the scalar types and the linear algebra over them, the platform uses two Stillwater libraries (cortex ADR 0015):

- **[Universal](https://github.com/stillwater-sc/universal)** supplies the number systems.
  - **Coverage beyond IEEE.** One parameterized `cfloat<nbits, es>` gives IEEE-layout formats of any width: binary32, bfloat16, binary16 and others. Universal also provides posits with their quire (an exact dot-product accumulator), fixed-point, logarithmic number systems, and intervals. You can test a hypothesis such as "accumulation is the bottleneck" by switching only the accumulator to a quire or a wider format.
  - **No hardware needed.** The types are emulated in software, so a 16-bit experiment runs on any CI host. Results don't depend on whether a CPU has native FP16, on FPU modes, or on compiler flags such as FMA contraction *(assessment)*.
  - **Already in use across the platform.** reflex instantiates its PID, quadrotor model, and biquads in `posit<32,2>`, `posit<16,1>`, `cfloat<32,8>`, bfloat16, and binary16 in its own tests. cortex runs a Kalman filter over posits in its type-smoke tests.
- **[MTL5](https://github.com/stillwater-sc/mtl5)** supplies the linear algebra.
  - **Generic over the value type.** Dense, sparse, and fixed-size matrices, Krylov solvers, and sparse direct solvers are templated on the scalar, so an EKF or QP written once runs in every Universal type. reflex's quadrotor model already runs on MTL5's fixed-size types.
  - **Co-developed with Universal.** Genericity gaps get fixed upstream rather than worked around, for example stillwater-sc/mtl5#538 (cortex ADR 0015).
- **Alternatives considered** *(assessment)*:
  - *Native `float`/`double` only.* This can't test 16-bit formats or posits, which is the question here.
  - *Compiler extended types* (`_Float16`, `__bf16`). Availability depends on the compiler and target; they are IEEE layouts only, with no posits, fixed-point, or quire.
  - *Eigen with custom scalar types.* It's possible, but the platform repos standardized on MTL5. A second linear-algebra library would split the generic-code effort and the validated-type set.
- **Costs we accept.**
  - Emulated types are much slower than hardware floats, so the sweep is an offline or nightly run, not a per-PR one.
  - Generic code compiles more slowly (cortex ADR 0003 and 0015).
  - Generic code may only use operations the scalar concepts guarantee.

## 8. Evidence so far, from the sibling repos

These are properties that tests in sibling repos already check. They aren't new measurements.
- **The reflex PID closed loop** (`tests/sdk/pid.cpp`) runs for 4 s at 1 kHz. It is required to match the `double` result within 1e-5 for the 32-bit types (`float`, `posit<32,2>`, `cfloat<32,8>`), and within 1e-2 for `posit<16,1>`. Those tolerances encode the expected 32-bit/16-bit gap of about three orders of magnitude.
- **The reflex quadrotor model** (`tests/sdk/quadrotor_model.cpp`) holds its state invariants (unit attitude, finite state, energy and ground) in `posit<16,1>` and bfloat16 on a spinning, climbing vehicle. Its parameter validation rejects RP-1 in binary16 because k_f underflows. The invariants show the model stays *well-formed* in 16 bits. They say nothing about *accuracy*, which is what P1 measures.
- **The reflex biquads** (`tests/sdk/dsp_biquad.cpp`) report notch depth for `posit<16,1>` and binary16 as a characterization, without gating it. The low-pass tests run in `double` and `float` only.

## 9. Results: the uniform-T sweep of B0

[#30](https://github.com/branes-ai/racing-drone/issues/30) ran B0 on gate P1 in each type, with the simulator, FC, scorer and log in `double` ([plan](../plan/precision-sweep/README.md)). Every number is in the generated table, [`data/precision-sweep-b0.md`](data/precision-sweep-b0.md), with the command line and commit that reproduce it; the per-type summaries and per-seed lap differences are beside it. This section gives the verdicts and doesn't copy the numbers.

- **H1 holds.** `float`, `cfloat<32,8>` and `posit<32,2>` construct and pass P1. Their lap-time, gate-error and speed percentiles equal `double`'s at the table's precision, and the Lap Δ column bounds the per-seed differences.
- **H2 is refuted.** bfloat16 constructs and passes P1. Its precision shows, but stays inside the gate: slower laps, a larger median gate error and a lower top speed than `double`.
- **H3 holds.** binary16 fails construction, and the reason names `rotor.thrust_coefficient` as subnormal (the table's Failures section), as predicted.
- **H4 is refuted on P1.** `posit<16,1>` constructs and passes P1, with laps faster than `double`'s and a top speed above the controller's velocity cap (`max_speed_m_s` in `baselines/b0-gt-pid.yaml`), though under the scenario's speed limit. Its second prediction, gate errors growing with distance from the origin, isn't measured: the table doesn't split gate error by position.

Why H2 and H4 were wrong, as far as it is known:
- **The predicted accumulation isn't in B0's stack.** §2.2's position examples integrate velocity into position in T. In B0, the simulator does that integration in `double`. The ground-truth estimator then hands the stack the true state each tick, cast to T: one rounding of at most ½ ULP, not an accumulated stall.
- **What B0 does accumulate is fed back.** The stack's accumulators are the controllers' integrators. Their lost increments (§2.2) leave errors that the proportional terms correct, the closed-loop outcome §5 named as informative.
- **Open: why `posit<16,1>` flies faster than `double`** and above the velocity cap. The cause isn't known yet (§10, [#35](https://github.com/branes-ai/racing-drone/issues/35)).

What this does and doesn't show:
- It shows that B0, a stack without a filter or optimizer, flies P1 in bfloat16 and `posit<16,1>`, the two 16-bit formats tested whose range holds its parameters.
- It doesn't overturn §2's mechanisms for the components that have them. B1's EKF integrates in T and carries the ill-conditioned covariance of §2.3, and B1's MPC solves a QP. B0 has neither, so this sweep doesn't test those predictions (§4, Table 3).

## 10. Next steps

1. Find why `posit<16,1>` flies B0 faster than `double` and above the velocity cap (§9), and measure H4's distance prediction with gate error split by position ([#35](https://github.com/branes-ai/racing-drone/issues/35)).
2. Per-slot scalar types in the composition and the baseline file, the cross-type slot contracts they need (§6), and H5 ([#36](https://github.com/branes-ai/racing-drone/issues/36)).
3. B1's EKF and MPC in binary32, `posit<32,2>` and the 16-bit formats, with the covariance form chosen for the precision (Joseph or square-root). That is where §2.2–§2.3 apply in earnest, and the sweep tool runs B1 once it has a baseline file ([#37](https://github.com/branes-ai/racing-drone/issues/37)).
