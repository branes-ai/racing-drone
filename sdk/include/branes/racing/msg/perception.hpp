#pragma once
/// @file perception.hpp
/// @brief GateObservations and IntruderDetections: what perception hands the estimator and the DAA tracker.
///
/// Fixed capacity, so a cycle never allocates (SW-01).

#include <branes/racing/msg/geometry.hpp>

#include <array>
#include <cstdint>

namespace branes::racing {

inline constexpr std::uint32_t kMaxVisibleGates = 4;
inline constexpr std::uint32_t kMaxDetections = 8;

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

/// One intruder (another drone or obstacle) seen in one image.
template <typename T>
struct IntruderDetection {
    Vec3<T> bearing{};  ///< unit direction in the camera frame
    T angular_size{0};  ///< apparent size [rad]
    T range{0};         ///< range estimate [m], from the known intruder size
    T range_sigma{0};   ///< 1σ range uncertainty [m]

    friend constexpr bool operator==(const IntruderDetection&, const IntruderDetection&) = default;
};

/// The intruders seen in one image. Invariants: I1 count ≤ kMaxDetections.
template <typename T>
struct IntruderDetections {
    std::uint32_t count{0};
    std::array<IntruderDetection<T>, kMaxDetections> detections{};

    friend constexpr bool operator==(const IntruderDetections&, const IntruderDetections&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool valid(const IntruderDetections<T>& d) noexcept {
    return d.count <= kMaxDetections;  // I1
}

}  // namespace branes::racing
