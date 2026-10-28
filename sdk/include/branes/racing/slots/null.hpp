#pragma once
/// @file null.hpp
/// @brief Null implementations of racing's slots, for the `null` baseline.
///
/// Racing-only (docs/assessments/framework-boundary.md): the nulls of the
/// slots that see the course or the gates. The nulls of the mission-neutral
/// slots are in neutral_nulls.hpp, included here.
///
/// Stateless and deterministic: each returns a valid, fixed output derived only
/// from its inputs. They exercise the framework end to end (composition,
/// harnesses, log, replay) before any real component exists.

#include <branes/racing/slots/neutral_nulls.hpp>
#include <branes/racing/slots/slots.hpp>

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

}  // namespace branes::racing
