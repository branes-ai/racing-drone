#pragma once
/// @file slots.hpp
/// @brief Racing's seven slots (docs/adr/0001-baseline-composition-framework.md).
///
/// Racing-only (docs/assessments/framework-boundary.md): the three slots that
/// see the course or the gates are defined here; the four mission-neutral ones
/// (Controller, SafetyFilter, DaaTracker, FcBridge) are in neutral.hpp.
///
/// A baseline fills every slot with one implementation, chosen at compile time
/// (cmake/baselines.cmake). The mode manager and pilot arbiter is not a slot:
/// it is fixed FC-resident safety logic in reflex.
///
/// Every slot is templated on the scalar type T (cortex ADR 0015). The data
/// contracts it exchanges live in branes/racing/msg/.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/perception.hpp>

#include <concepts>

namespace branes::racing {

/// Fuses IMU and vision into the course-frame state.
template <class E, class T>
concept Estimator = requires(E e,
                             const E& ce,
                             const ImuSample<T>& imu,
                             const CameraFrame& frame,
                             const GateObservations<T>& gates,
                             Timestamp now) {
    e.process_imu(imu);
    e.process_frame(frame, gates);                           // gates may be empty (no detection / corrupted frame)
    { ce.state_at(now) } -> std::same_as<StateEstimate<T>>;  // delay-compensated (EST-06)
    { ce.health() } -> std::same_as<EstimatorHealth<T>>;     // NIS, staleness (SAF-03)
};

/// Finds gates in a camera frame.
template <class D, class T>
concept GateDetector = requires(D d, const CameraFrame& frame, const CourseMap<T>& map, const StateEstimate<T>& prior) {
    { d.detect(frame, map, prior) } -> std::same_as<GateObservations<T>>;
};

/// Plans the reference through the course.
template <class P, class T>
concept Planner = requires(P p, const CourseMap<T>& map, const StateEstimate<T>& x) {
    { p.plan(map, x) } -> std::same_as<Reference<T>>;  // may reuse the previous plan
};

}  // namespace branes::racing
