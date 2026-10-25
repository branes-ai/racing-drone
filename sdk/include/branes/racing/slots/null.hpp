#pragma once
/// @file null.hpp
/// @brief Null implementations of every slot, for the `null` baseline.
///
/// Stateless and deterministic: each returns a valid, fixed output derived only
/// from its inputs. They exercise the framework end to end (composition,
/// harnesses, log, replay) before any real component exists.

#include <branes/racing/slots/slots.hpp>

#include <optional>

namespace branes::racing {

/// Reports the vehicle at rest on the start pad, level, at the requested time.
template <typename T>
class NullEstimator {
public:
    void process_imu(const ImuSample<T>&) noexcept {}
    void process_frame(const CameraFrame&, const GateObservations<T>&) noexcept {}

    [[nodiscard]] StateEstimate<T> state_at(Timestamp now) const noexcept {
        StateEstimate<T> x{};
        x.t_valid = now;  // attitude defaults to identity, covariance to zero: valid()
        return x;
    }
    [[nodiscard]] EstimatorHealth<T> health() const noexcept {
        return EstimatorHealth<T>{T(0), 0, true};
    }
};

/// Sees no gates.
template <typename T>
class NullGateDetector {
public:
    [[nodiscard]] GateObservations<T>
    detect(const CameraFrame&, const CourseMap<T>&, const StateEstimate<T>&) const noexcept {
        return GateObservations<T>{};
    }
};

/// Plans "stay where you are": one sample at the current state.
template <typename T>
class NullPlanner {
public:
    [[nodiscard]] Reference<T> plan(const CourseMap<T>&, const StateEstimate<T>& x) const noexcept {
        Reference<T> r{};
        r.count = 1;
        r.samples[0].t = x.t_valid;
        r.samples[0].position = x.position;
        return r;
    }
};

/// Commands zero thrust and zero body rates (motors stay idle).
template <typename T>
class NullController {
public:
    [[nodiscard]] ThrustRates<T>
    step(const StateEstimate<T>& x, const Reference<T>&, const ConflictSet<T>&) const noexcept {
        ThrustRates<T> u{};
        u.t = x.t_valid;
        return u;
    }
};

/// Passes the command through unchanged.
template <typename T>
class NullSafetyFilter {
public:
    [[nodiscard]] ThrustRates<T>
    filter(const StateEstimate<T>&, const ThrustRates<T>& u, const ConflictSet<T>&, const Geofence<T>&) const noexcept {
        return u;
    }
};

/// No DAA (DaaTracker "None"): never reports a conflict. A fresh, empty set
/// with tracker_ok, so the safety filter sees "no known intruders".
template <typename T>
class NullDaaTracker {
public:
    void update(const IntruderDetections<T>&, const StateEstimate<T>&) noexcept {}
    [[nodiscard]] ConflictSet<T> conflicts(const StateEstimate<T>& x) const noexcept {
        ConflictSet<T> c{};
        c.valid_at = x.t_valid;
        c.tracker_ok = true;
        return c;
    }
};

/// Discards commands and relays no IMU samples; reports a disarmed FC.
template <typename T>
class NullFcBridge {
public:
    void send(const ThrustRates<T>&, Timestamp) noexcept {}
    [[nodiscard]] std::optional<ImuSample<T>> poll_imu() noexcept {
        return std::nullopt;
    }
    [[nodiscard]] FcStatus fc_status() const noexcept {
        return FcStatus{};
    }
};

}  // namespace branes::racing
