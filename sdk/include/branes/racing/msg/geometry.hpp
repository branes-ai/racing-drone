#pragma once
/// @file geometry.hpp
/// @brief Vec3 and Quat: the geometric value types of the data contracts.
///
/// Both are plain aggregates over std::array / named scalars, so every contract
/// that holds them stays a repr(C)-compatible value type (cortex ADR 0010).
/// The arithmetic lives in the components (MTL5 from Phase 1); the contracts
/// only carry values.

#include <array>

namespace branes::racing {

/// A 3-vector (x, y, z).
template <typename T>
using Vec3 = std::array<T, 3>;

/// A rotation quaternion, Hamilton convention, scalar first (w, x, y, z).
/// The default is the identity rotation.
template <typename T>
struct Quat {
    T w{1};
    T x{0};
    T y{0};
    T z{0};

    friend constexpr bool operator==(const Quat&, const Quat&) = default;
};

/// Squared norm of a quaternion.
template <typename T>
[[nodiscard]] constexpr T norm_squared(const Quat<T>& q) noexcept {
    return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
}

/// True if q is a unit quaternion within `tol` on the squared norm
/// (|‖q‖² − 1| ≤ tol). The contracts use tol = 1e-6.
template <typename T>
[[nodiscard]] constexpr bool is_unit(const Quat<T>& q, T tol = T(1e-6)) noexcept {
    const T d = norm_squared(q) - T(1);
    return d <= tol && -d <= tol;
}

}  // namespace branes::racing
