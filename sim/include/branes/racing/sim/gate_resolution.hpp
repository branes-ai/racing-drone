#pragma once
/// @file gate_resolution.hpp
/// @brief A scalar type's resolution at a gate (docs/plan/gate-error-by-gate/).
///
/// Racing-only (docs/assessments/framework-boundary.md); ulp_at, the
/// resolution at a value, is mission-neutral (sweep_types.hpp).

#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/geometry.hpp>
#include <branes/racing/sim/sweep_types.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace branes::racing::sim {

/// A gate's in-plane resolution in T: the largest ulp_at<T> of its center's
/// coordinates on the world axes that span its plane. The plane is spanned by
/// the gate's y and z axes (its x axis is the normal); a world axis spans it
/// if either has a component on it. A gate's error is measured in its plane,
/// so this is the finest error the stack's position can resolve there.
template <typename T>
[[nodiscard]] double in_plane_ulp(const Gate<double>& g) {
    const Quat<double>& q = g.orientation;
    // The gate's y and z axes in the world frame: R(q)·e_y and R(q)·e_z.
    const std::array<double, 3> gy{
        2 * (q.x * q.y - q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z + q.w * q.x)};
    const std::array<double, 3> gz{
        2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y)};
    constexpr double negligible = 1e-9;  // a rounded zero component of a unit axis
    double u = 0;
    for (std::size_t i = 0; i < 3; ++i) {
        if (std::fabs(gy[i]) > negligible || std::fabs(gz[i]) > negligible) {
            u = std::max(u, ulp_at<T>(g.center[i]));
        }
    }
    return u;
}

}  // namespace branes::racing::sim
