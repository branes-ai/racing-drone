# A checked time base for the controller's dt

**Status:** Approved 2026-10-27, in implementation
**Issue:** [racing-drone#40](https://github.com/branes-ai/racing-drone/issues/40), part of the Phase 2 epic [racing-drone#9](https://github.com/branes-ai/racing-drone/issues/9)
**Related:** [precision sweep plan](../precision-sweep/README.md), [mixed-precision assessment](../../assessments/mixed-precision.md) §9, [B0 plan](../b0/README.md)

## Issues and pull requests

| Item | What | State |
|---|---|---|
| [racing-drone#40](https://github.com/branes-ai/racing-drone/issues/40) | This plan | Open |
| PR (this document lands with it) | The implementation and the regenerated sweep | Open |
| [racing-drone#35](https://github.com/branes-ai/racing-drone/issues/35) | Found the defect; measures gate error by position after this lands | Open |
| [racing-drone#33](https://github.com/branes-ai/racing-drone/pull/33) | The precision sweep whose `posit<16,1>` result this explains | Merged |

## Problem

`CascadedPid::step` computed `dt = T(Δns) * T(1e-9)`, rounding each factor into `T` before the product. In `posit<16,1>`, 4,000,000 rounds to 2²² and 1e-9 rounds up to minpos, 2⁻²⁸, because posits don't underflow. A 4 ms tick became 2⁻⁶ s = 15.6 ms, 3.9× too long. The velocity PIDs' integral term then grew 3.9× too fast. Nothing reported it: the stack flew, faster than `double` and above its own velocity cap ([assessment §9](../../assessments/mixed-precision.md#9-results-the-uniform-t-sweep-of-b0)).

Rounding once, `T(double(Δns) * 1e-9)`, would repair B0 but hide `double` as the type the conversion is computed in. That would cap every later stack at double's time resolution, and it still wouldn't fail when a type can't resolve a loop's dynamics.

## Plan

### 1. `TimeBase<T, Wide>` (`msg/time.hpp`)

It converts a tick interval in integer nanoseconds to seconds in `T`, computed in `Wide`, a type the slot names. It checks the result against the slot's accuracy requirement, and fails construction when `T` can't meet it.

### 2. `CascadedPid` (`glue/cascaded_pid.hpp`)

- It names `time_wide_type = double` and reads `control_rate_hz` with the apps' default, `config::kDefaultControlRateHz`.
- It derives ε_max from its velocity gains (below) and builds its `TimeBase` in `from_context`.
- `step` takes dt from `TimeBase::seconds`. A tick whose dt is rejected is held, like dt ≤ 0, and counted in `timing_faults()`.

### 3. The dt accuracy, derived from the gains

dt enters B0's position loop only through the velocity PIDs' integral, k_i·e·dt. Their k_d is 0, and the attitude loop has no dt. So a relative dt error ε is a relative k_i error ε.

reflex's design (`cascade.hpp`) puts the velocity loop's crossover at ω_c ≈ k_v and the integral zero at r·ω_c, with r = k_i/k_v² = 0.1, "a decade below". The PI zero costs phase atan((1+ε)·r) at crossover. So, to first order, a dt error ε costs ε·r/(1+r²) rad of phase margin.

The budget is that a dt error may cost no more phase margin than one control period of delay already does, ω_c·T_s. The stack accepts that cost by running at its control rate.

  ε_max = ω_c·T_s·(1 + r²)/r, the smallest over the three axes

For RP-1 at 250 Hz (τ = 0.02 s, so k_v = 50/14 s⁻¹) this is ≈ 0.144. It's computed at construction from the configuration and the control rate.

Two approximations are disclosed: the analysis is first order in ε, and ω_c ≈ k_v treats the inner loops as ideal. `dt_tolerance` refuses a configuration outside the derivation, one whose velocity loops aren't PI (k_p, k_i > 0, k_d = 0).

## Model and contracts

### `TimeBase<T, Wide>` (value type, owned by the slot)

- **Members:**
  - `nominal_ns_`: the period the slot expects;
  - `max_ns_`: 2 × nominal, one held tick;
  - `rel_tol_`: the relative accuracy the slot requires of dt.
- **I1:** `Wide` has at least `T`'s significand digits, and at least 30 so that 10⁹ is exact (both are `static_assert`s). It holds every interval up to `max_ns_` exactly.
- **I2:** 0 < `nominal_ns_` ≤ `max_ns_`; `rel_tol_` > 0 and finite.
- **I3:** the nominal and the longest interval convert to a positive, finite, normal `T`, within `rel_tol_` of their value in `Wide`.
- **The conversion:** dt = T(Wide(Δns) / Wide(10⁹)). Both operands are exact (I1), so the value in `Wide` is the exact Δns·10⁻⁹ correctly rounded. The errors above are relative to it.
- **`make_time_base<T, Wide>(nominal_ns, rel_tol)`:**
  - Post: on success, I1–I3.
  - Otherwise it returns a reason naming the interval, the error and the bound. It reaches the user as a stack-construction error, before flight.
- **`seconds(dt_ns)`:**
  - Post: the dt within `rel_tol_` if 0 < dt_ns ≤ `max_ns_`, and `T` resolves it to that bound.
  - Otherwise nullopt: a timing fault, or an interval `T` can't resolve.

## Acceptance criteria (stated in the issue before implementation)

1. **Unit:** for every sweep type, `seconds(4 ms)` is within ½ ULP_T(0.004) + ½ ε_double·0.004 of 0.004. Main's two-rounding formula misses that bound for `posit<16,1>`.
2. **Unit:** `make_time_base` fails, with a message naming the period, when the bound can't be met.
3. **Compile:** a `Wide` narrower than `T` is rejected by a `static_assert`.
4. **Regression:**
   - The `double` logs and sitl_batch output are byte-identical to main's.
   - For `float`, `cfloat<32,8>` and `posit<32,2>`, any change is reported, not required to be zero.
5. **Sweep:** the data in `docs/assessments/data/precision-sweep-b0.*` is regenerated, and assessment §9 reports `posit<16,1>`'s change with its cause.

## Implementation notes (departures from the issue's plan, disclosed)

- **I1 uses ≥, not >.** B0's `double` stack names `Wide = double`, so the digits are equal. When `T` is `Wide` there is no second rounding, so equal digits cost nothing. Criterion 3, "narrower", already said ≥.
- **Division by 10⁹, not multiplication by 1e-9.** 1e-9 isn't exact in any binary `Wide`, while 10⁹ is, so the quotient is correctly rounded.
  - For B0's 4 ms tick in `double`, the quotient equals main's `4e6 * 1e-9` bit for bit, which keeps criterion 4.
  - The two differ at other intervals, which no B0 run produces.
- **I3 is checked at construction at the nominal and the longest interval, and in every `seconds()` call.** The issue's plan sampled the binade boundaries of `T`. But the value at a boundary converts exactly, and the error bound peaks just above it, so a sample there proves nothing. Instead, every dt is checked where it's used. A failure holds the tick and counts it in `timing_faults()`, which wasn't in the plan.
- **The criterion 2 test** takes as its requirement half of `T`'s actual error at 4 ms. The actual error can be far below ½ ULP (bfloat16's is), so a fixed fraction of ε doesn't fail every type. For `T = double` there's no second rounding, so no requirement can fail. The test checks that instead.
- **The compile-fail check** (`compile_fail.time_base.narrow_wide`) builds an object library outside `all`, on the header path only. ctest builds it and requires the `static_assert`'s message, so an unrelated build failure doesn't pass it.
- **The default control rate** is one constant, `config::kDefaultControlRateHz`, which the four apps and `CascadedPid` use.

## Out of scope

- The harness's message casts, which go through `double` because the simulator's truth is `double`. That is the simulator's declared type, not a hidden one.
- Gate error by position and H4's distance prediction: [#35](https://github.com/branes-ai/racing-drone/issues/35), measured after this lands.
