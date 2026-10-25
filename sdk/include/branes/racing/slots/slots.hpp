#pragma once
/// @file slots.hpp
/// @brief The seven slot concepts (docs/adr/0001-baseline-composition-framework.md).
///
/// A baseline fills every slot with one implementation, chosen at compile time
/// (cmake/baselines.cmake). The mode manager and pilot arbiter is not a slot:
/// it is fixed FC-resident safety logic in reflex.
///
/// Every slot is templated on the scalar type T (cortex ADR 0015). The data
/// contracts it exchanges live in branes/racing/msg/.

#include <branes/racing/msg/control.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/daa.hpp>
#include <branes/racing/msg/estimate.hpp>
#include <branes/racing/msg/fc.hpp>
#include <branes/racing/msg/perception.hpp>
#include <branes/racing/msg/sensors.hpp>

#include <concepts>
#include <optional>

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

/// Tracks the reference: MPCC, a learned policy, or PID.
template <class C, class T>
concept Controller =
    requires(C c, const StateEstimate<T>& x, const Reference<T>& ref, const ConflictSet<T>& conflicts) {
        { c.step(x, ref, conflicts) } -> std::same_as<ThrustRates<T>>;
    };

/// Last-resort command filter: CBF separation, geofence, envelope.
template <class F, class T>
concept SafetyFilter = requires(F f,
                                const StateEstimate<T>& x,
                                const ThrustRates<T>& u,
                                const ConflictSet<T>& conflicts,
                                const Geofence<T>& fence) {
    { f.filter(x, u, conflicts, fence) } -> std::same_as<ThrustRates<T>>;
};

/// Tracks intruders and predicts conflicts over the control horizon.
template <class A, class T>
concept DaaTracker = requires(A a, const A& ca, const IntruderDetections<T>& det, const StateEstimate<T>& x) {
    a.update(det, x);
    { ca.conflicts(x) } -> std::same_as<ConflictSet<T>>;
};

/// The companion's link to the flight controller.
template <class B, class T>
concept FcBridge = requires(B b, const B& cb, const ThrustRates<T>& u, Timestamp now) {
    b.send(u, now);                                                 // also serves as the companion heartbeat (SAF-01)
    { b.poll_imu() } -> std::same_as<std::optional<ImuSample<T>>>;  // FC-relayed, FC-timestamped
    { cb.fc_status() } -> std::same_as<FcStatus>;
};

}  // namespace branes::racing
