#pragma once
/// @file sweep_types.hpp
/// @brief The scalar types of the precision sweep (docs/plan/precision-sweep/).
///
/// One list for precision_sweep_<baseline> and the tests of the casts and
/// tolerances, so both cover the same types. Including it needs Universal
/// (universal::universal); the rest of sim/ does not. Also the resolution of
/// a type at a value and at a gate (docs/plan/gate-error-by-gate/).

#include <branes/racing/msg/course.hpp>

#include <universal/number/cfloat/cfloat.hpp>
#include <universal/number/posit/posit.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>
#include <tuple>
#include <type_traits>

namespace branes::racing::sim {

// IEEE layouts in software: subnormals, no max-exponent values, no saturation.
using cfloat32 = sw::universal::cfloat<32, 8, std::uint32_t, true, false, false>;
using bfloat16 = sw::universal::cfloat<16, 8, std::uint16_t, true, false, false>;
using binary16 = sw::universal::cfloat<16, 5, std::uint16_t, true, false, false>;
using posit32 = sw::universal::posit<32, 2>;
using posit16 = sw::universal::posit<16, 1>;

/// The sweep's types, in order; double first, the reference.
using SweepTypes = std::tuple<double, float, cfloat32, posit32, bfloat16, binary16, posit16>;

/// A sweep type's file name (slug) and its label in the results table.
template <typename T>
struct SweepType {
    std::string_view slug;
    std::string_view label;
};

/// The names of SweepTypes, element for element.
inline const auto kSweepTypes = std::tuple{SweepType<double>{"double", "double"},
                                           SweepType<float>{"float", "float"},
                                           SweepType<cfloat32>{"cfloat32_8", "cfloat<32,8>"},
                                           SweepType<posit32>{"posit32_2", "posit<32,2>"},
                                           SweepType<bfloat16>{"bfloat16", "bfloat16 (cfloat<16,8>)"},
                                           SweepType<binary16>{"binary16", "binary16 (cfloat<16,5>)"},
                                           SweepType<posit16>{"posit16_1", "posit<16,1>"}};

/// The gap from T(|x|) to the next value of T above it [same unit as x]: the
/// type's resolution at x. Built-in floats step with std::nextafter; Universal
/// types step to the next encoding, which they order by value.
template <typename T>
[[nodiscard]] double ulp_at(double x) {
    const T t = static_cast<T>(std::fabs(x));
    if constexpr (std::is_floating_point_v<T>) {
        return double(std::nextafter(t, std::numeric_limits<T>::infinity())) - double(t);
    } else {
        T above = t;
        ++above;
        return double(above) - double(t);
    }
}

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
