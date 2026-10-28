#pragma once
/// @file course.hpp
/// @brief CourseMap: the course as the slots see it.
///
/// Loaded from courses/<name>.yaml by config::load_course (load-time, double)
/// and converted to the stack's scalar type with cast<T>().

#include <branes/racing/msg/geometry.hpp>
#include <branes/racing/msg/region.hpp>

#include <array>
#include <cstdint>

namespace branes::racing {

inline constexpr std::uint32_t kMaxGates = 64;
inline constexpr std::uint32_t kMaxGateSequence = 64;

/// A gate: center and orientation in the course frame. The gate's x axis is
/// its normal, pointing in the direction of travel.
template <typename T>
struct Gate {
    std::int32_t id{0};
    Vec3<T> center{};
    Quat<T> orientation{};
    T inner_width{0};   ///< [m]
    T inner_height{0};  ///< [m]

    friend constexpr bool operator==(const Gate&, const Gate&) = default;
};

/// The course.
///
/// Invariants:
///   I1: 1 ≤ gate_count ≤ kMaxGates; gate ids are unique
///   I2: every gate has inner_width > 0, inner_height > 0, and a unit orientation
///   I3: 1 ≤ sequence_length ≤ kMaxGateSequence; every sequence entry is a gate id
///   I4: laps ≥ 1
///   I5: geofence.min < geofence.max on every axis, and the geofence contains every
///       gate center and the start position
template <typename T>
struct CourseMap {
    std::uint32_t gate_count{0};
    std::array<Gate<T>, kMaxGates> gates{};
    std::uint32_t sequence_length{0};
    std::array<std::int32_t, kMaxGateSequence> sequence{};  ///< gate ids in passing order, one lap
    std::uint32_t laps{0};
    Geofence<T> geofence{};
    StartPose<T> start{};

    friend constexpr bool operator==(const CourseMap&, const CourseMap&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool valid(const CourseMap<T>& m) noexcept {
    if (m.gate_count < 1 || m.gate_count > kMaxGates) {
        return false;  // I1
    }
    for (std::uint32_t i = 0; i < m.gate_count; ++i) {
        const Gate<T>& g = m.gates[i];
        for (std::uint32_t j = i + 1; j < m.gate_count; ++j) {
            if (m.gates[j].id == g.id) {
                return false;  // I1: unique ids
            }
        }
        if (!(g.inner_width > T(0) && g.inner_height > T(0) && is_unit(g.orientation))) {
            return false;  // I2
        }
        if (!contains(m.geofence, g.center)) {
            return false;  // I5
        }
    }
    if (m.sequence_length < 1 || m.sequence_length > kMaxGateSequence) {
        return false;  // I3
    }
    for (std::uint32_t s = 0; s < m.sequence_length; ++s) {
        bool found = false;
        for (std::uint32_t i = 0; i < m.gate_count; ++i) {
            found = found || m.gates[i].id == m.sequence[s];
        }
        if (!found) {
            return false;  // I3
        }
    }
    if (m.laps < 1) {
        return false;  // I4
    }
    for (std::size_t a = 0; a < 3; ++a) {
        if (!(m.geofence.min[a] < m.geofence.max[a])) {
            return false;  // I5
        }
    }
    return contains(m.geofence, m.start.position);  // I5
}

/// Convert a course to another scalar type, element by element.
template <typename To, typename From>
[[nodiscard]] constexpr CourseMap<To> cast(const CourseMap<From>& m) noexcept {
    const auto v = [](const Vec3<From>& a) { return Vec3<To>{To(a[0]), To(a[1]), To(a[2])}; };
    CourseMap<To> out{};
    out.gate_count = m.gate_count;
    for (std::uint32_t i = 0; i < kMaxGates; ++i) {
        const Gate<From>& g = m.gates[i];
        out.gates[i] =
            Gate<To>{g.id,
                     v(g.center),
                     Quat<To>{To(g.orientation.w), To(g.orientation.x), To(g.orientation.y), To(g.orientation.z)},
                     To(g.inner_width),
                     To(g.inner_height)};
    }
    out.sequence_length = m.sequence_length;
    out.sequence = m.sequence;
    out.laps = m.laps;
    out.geofence = Geofence<To>{v(m.geofence.min), v(m.geofence.max)};
    out.start = StartPose<To>{v(m.start.position), To(m.start.yaw)};
    return out;
}

}  // namespace branes::racing
