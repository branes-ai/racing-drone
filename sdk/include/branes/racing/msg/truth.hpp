#pragma once
/// @file truth.hpp
/// @brief TruthState: the simulated vehicle's true state (SITL only).
///
/// The SITL harness logs it on the `truth` channel for scoring. It is never a
/// flight input (SYS-02): only SITL-only slots, such as the ground-truth
/// estimator of baseline B0, may read it.

#include <branes/racing/msg/geometry.hpp>
#include <branes/racing/msg/time.hpp>

#include <array>

namespace branes::racing {

/// The true vehicle state at `t`, in the course frame (z up).
///
/// Invariants: I1 attitude is a unit quaternion (is_unit); I2 rotor speeds ≥ 0.
template <typename T>
struct TruthState {
    Timestamp t;
    Vec3<T> position{};               ///< course frame [m]
    Vec3<T> velocity{};               ///< course frame [m/s]
    Quat<T> attitude{};               ///< body → course
    Vec3<T> body_rates{};             ///< body frame [rad/s]
    std::array<T, 4> rotor_speeds{};  ///< [rad/s]
    bool on_ground{false};

    friend constexpr bool operator==(const TruthState&, const TruthState&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool valid(const TruthState<T>& x) noexcept {
    if (!is_unit(x.attitude)) {
        return false;  // I1
    }
    for (const T w : x.rotor_speeds) {
        if (!(w >= T(0))) {
            return false;  // I2
        }
    }
    return true;
}

}  // namespace branes::racing
