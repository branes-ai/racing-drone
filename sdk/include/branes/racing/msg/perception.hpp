#pragma once
/// @file perception.hpp
/// @brief GateObservations: what gate detection hands the estimator.
///
/// Fixed capacity, so a cycle never allocates (SW-01).

#include <branes/racing/msg/geometry.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace branes::racing {

inline constexpr std::uint32_t kMaxVisibleGates = 4;

/// One gate seen in one image: its four inner corners in pixels.
template <typename T>
struct GateObservation {
    std::int32_t gate_id{-1};                   ///< course gate id hypothesis, −1 = unknown
    std::array<std::array<T, 2>, 4> corners{};  ///< (u, v) [px]: top-left, top-right, bottom-right, bottom-left
    std::array<T, 64> covariance{};             ///< 8×8 row-major covariance of the corners [px²]
    T quality{0};                               ///< detector confidence ∈ [0, 1]

    friend constexpr bool operator==(const GateObservation&, const GateObservation&) = default;
};

/// The gates seen in one image.
///
/// Invariants: I1 count ≤ kMaxVisibleGates; I2 every used entry has quality ∈ [0, 1].
template <typename T>
struct GateObservations {
    std::uint32_t count{0};
    std::array<GateObservation<T>, kMaxVisibleGates> gates{};

    friend constexpr bool operator==(const GateObservations&, const GateObservations&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool valid(const GateObservations<T>& g) noexcept {
    if (g.count > kMaxVisibleGates) {
        return false;  // I1
    }
    for (std::uint32_t i = 0; i < g.count; ++i) {
        if (!(g.gates[i].quality >= T(0) && g.gates[i].quality <= T(1))) {
            return false;  // I2
        }
    }
    return true;
}

/// Gate observations in another scalar type, element by element (all entries, used or not).
template <typename To, typename From>
[[nodiscard]] constexpr GateObservations<To> cast(const GateObservations<From>& g) noexcept {
    GateObservations<To> out{};
    out.count = g.count;
    for (std::uint32_t i = 0; i < kMaxVisibleGates; ++i) {
        const GateObservation<From>& o = g.gates[i];
        GateObservation<To>& c = out.gates[i];
        c.gate_id = o.gate_id;
        for (std::size_t k = 0; k < o.corners.size(); ++k) {
            c.corners[k] = cast<To>(o.corners[k]);
        }
        c.covariance = cast<To>(o.covariance);
        c.quality = To(o.quality);
    }
    return out;
}

}  // namespace branes::racing
