#pragma once
/// @file domains.hpp
/// @brief The domain of each of racing's messages: the course and the gate
/// observations (anatomy docs/adr/0004, step 3; docs/plan/flow-graph/).
///
/// Racing-only (docs/assessments/framework-boundary.md). One domain_of per
/// message struct in msg/, field by field, in declaration order, with the
/// struct's member names, as anatomy's are (mobility/flow/domains.hpp). A count
/// and the fixed array it counts are one bounded vector, named after the
/// array: the course's `gates` covers `gate_count`, its `sequence` covers
/// `sequence_length`, and the observations' `gates` covers `count`. A gate
/// observation's corner covariance is a symmetric 8 × 8 matrix, stored as the
/// struct's 64 elements.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/anatomy_mobility.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/perception.hpp>

#include <cstdint>

namespace branes::anatomy::flow {

// ── course.hpp ──────────────────────────────────────────────────────

template <class T>
struct domain_of<racing::Gate<T>> {
    using type = record<field<"id", scalar<std::int32_t>>,
                        field<"center", vec3<T>>,
                        field<"orientation", quat<T>>,
                        field<"inner_width", scalar<T>>,
                        field<"inner_height", scalar<T>>>;
};

template <class T>
struct domain_of<racing::CourseMap<T>> {
    using type = record<field<"gates", vector<domain_of_t<racing::Gate<T>>, bounded<racing::kMaxGates>>>,
                        field<"sequence", vector<std::int32_t, bounded<racing::kMaxGateSequence>>>,
                        field<"laps", scalar<std::uint32_t>>,
                        field<"geofence", domain_of_t<Geofence<T>>>,
                        field<"start", domain_of_t<StartPose<T>>>>;
};

// ── perception.hpp ──────────────────────────────────────────────────

template <class T>
struct domain_of<racing::GateObservation<T>> {
    using type = record<field<"gate_id", scalar<std::int32_t>>,
                        field<"corners", matrix<T, dim<4>, dim<2>>>,
                        field<"covariance", matrix<T, dim<8, "c">, dim<8, "c">, structure::symmetric>>,
                        field<"quality", scalar<T>>>;
};

template <class T>
struct domain_of<racing::GateObservations<T>> {
    using type = vector<domain_of_t<racing::GateObservation<T>>, bounded<racing::kMaxVisibleGates>>;
};

}  // namespace branes::anatomy::flow
