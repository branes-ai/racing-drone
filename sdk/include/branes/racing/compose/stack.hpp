#pragma once
/// @file stack.hpp
/// @brief Stack<T, ...>: one baseline's slot composition, driven event by event.
///
/// The harnesses (SITL, replay, flight) feed the stack the same three events —
/// an IMU sample, a camera frame, a control tick — so a log of the events
/// replays to the same outputs (SYS-08, SW-03). The stack itself reads no
/// clock and draws no random numbers: time arrives with the events.
///
/// Model
/// -----
///   Stack<T, Est, Det, Plan, Ctl, Safe, Daa, Fc>
///     map_       : CourseMap<T> [owned, immutable] — the course
///     est_ … fc_ : one instance per slot [owned]
///     last_tick_ : std::optional<Timestamp> — time of the previous on_tick
///
/// Invariants
/// ----------
///   I1: valid(map_)                                   — constrains map_
///   I2: ticks arrive in non-decreasing time order     — constrains last_tick_
///
/// Intruder detections have no producing slot yet; on_frame() hands the DAA
/// tracker an empty set until the DroneNet detector lands (Phase 6).

#include <branes/racing/msg/course.hpp>
#include <branes/racing/slots/slots.hpp>

#include <cassert>
#include <optional>
#include <utility>

namespace branes::racing {

/// What one control tick produces.
template <typename T>
struct TickOutput {
    StateEstimate<T> state;
    ThrustRates<T> command;

    friend constexpr bool operator==(const TickOutput&, const TickOutput&) = default;
};

template <typename T, class Est, class Det, class Plan, class Ctl, class Safe, class Daa, class Fc>
    requires Estimator<Est, T> && GateDetector<Det, T> && Planner<Plan, T> && Controller<Ctl, T> &&
             SafetyFilter<Safe, T> && DaaTracker<Daa, T> && FcBridge<Fc, T>
class Stack {
public:
    using scalar_type = T;

    /// Pre: valid(map). Post: I1, I2 (no tick yet).
    explicit Stack(const CourseMap<T>& map,
                   Est est = {},
                   Det det = {},
                   Plan plan = {},
                   Ctl ctl = {},
                   Safe safe = {},
                   Daa daa = {},
                   Fc fc = {})
        : map_(map), est_(std::move(est)), det_(std::move(det)), plan_(std::move(plan)), ctl_(std::move(ctl)),
          safe_(std::move(safe)), daa_(std::move(daa)), fc_(std::move(fc)) {
        assert(valid(map_));  // Pre / I1
        assert(check_invariant());
    }

    /// Feed one IMU sample to the estimator. Pre: valid(s).
    void on_imu(const ImuSample<T>& s) {
        assert(valid(s));  // Pre
        est_.process_imu(s);
    }

    /// Detect gates in a frame and fuse them. Pre: valid(f).
    void on_frame(const CameraFrame& f) {
        assert(valid(f));  // Pre
        const StateEstimate<T> prior = est_.state_at(f.header.t_mid);
        const GateObservations<T> gates = f.header.corrupted ? GateObservations<T>{} : det_.detect(f, map_, prior);
        est_.process_frame(f, gates);
        daa_.update(IntruderDetections<T>{}, prior);
    }

    /// Run one control cycle at `now` and send the command to the FC.
    /// Pre: now ≥ the previous tick (I2). Post: I2 with last_tick_ == now.
    TickOutput<T> on_tick(Timestamp now) {
        assert(!last_tick_ || *last_tick_ <= now);  // Pre / I2
        const StateEstimate<T> x = est_.state_at(now);
        const Reference<T> ref = plan_.plan(map_, x);
        const ConflictSet<T> conflicts = daa_.conflicts(x);
        const ThrustRates<T> u = safe_.filter(x, ctl_.step(x, ref, conflicts), conflicts, map_.geofence);
        fc_.send(u, now);
        last_tick_ = now;
        assert(check_invariant());
        return TickOutput<T>{x, u};
    }

    [[nodiscard]] const CourseMap<T>& course() const noexcept {
        return map_;
    }
    [[nodiscard]] Fc& fc_bridge() noexcept {
        return fc_;
    }

private:
    [[nodiscard]] bool check_invariant() const noexcept {
        return valid(map_);  // I1 (I2 is checked at the tick entry)
    }

    CourseMap<T> map_;
    Est est_;
    Det det_;
    Plan plan_;
    Ctl ctl_;
    Safe safe_;
    Daa daa_;
    Fc fc_;
    std::optional<Timestamp> last_tick_;
};

}  // namespace branes::racing
