#pragma once
/// @file gates.hpp
/// @brief The oracle gate projector: gate corners through the camera model, no rendering.
///
/// project_gates() projects each gate's four inner corners from the course
/// frame through the true vehicle pose and the camera's extrinsics and
/// intrinsics. It is what a perfect gate detector would report, for the
/// OracleGates detector and for scoring perception later.
///
/// Conventions:
///  - Gate frame: x is the gate's normal (direction of travel), y left, z up.
///    Seen from the approach side, the corners are, in order, top-left
///    (+y, +z), top-right (−y, +z), bottom-right (−y, −z), bottom-left (+y, −z),
///    at ±inner_width/2 and ±inner_height/2.
///  - Camera frame and models: camera_model.hpp (mission-neutral).
///
/// A gate is reported only if all four corners lie in front of the camera
/// (z > 0) and inside the image. At most kMaxVisibleGates are reported,
/// nearest gate center first. Each has quality 1 and a diagonal corner
/// covariance of σ² per coordinate.
///
/// Not modeled: occlusion by other gates, rolling shutter (the projection is
/// at the frame's mid-exposure pose), motion blur.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/anatomy_sim.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/perception.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace branes::racing::sim {

/// The four inner corners of `g` in the course frame, in the documented order.
[[nodiscard]] inline std::array<Vec3<double>, 4> gate_corners(const Gate<double>& g) noexcept {
    const double w = g.inner_width / 2;
    const double h = g.inner_height / 2;
    const std::array<Vec3<double>, 4> local = {
        Vec3<double>{0, w, h}, Vec3<double>{0, -w, h}, Vec3<double>{0, -w, -h}, Vec3<double>{0, w, -h}};
    std::array<Vec3<double>, 4> out{};
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec3<double> r = detail::rotate(g.orientation, local[i]);
        out[i] = {g.center[0] + r[0], g.center[1] + r[1], g.center[2] + r[2]};
    }
    return out;
}

/// The gates a perfect detector sees from `x`. Pre: valid(map), valid(x).
/// Post: valid(result); every reported corner lies inside the image.
[[nodiscard]] inline GateObservations<double> project_gates(const CourseMap<double>& map,
                                                            const TruthState<double>& x,
                                                            const config::CameraParams& cam,
                                                            double corner_sigma_px) {
    struct Seen {
        double distance;
        GateObservation<double> obs;
    };
    std::array<Seen, kMaxGates> seen{};
    std::size_t n = 0;
    for (std::uint32_t i = 0; i < map.gate_count; ++i) {
        const Gate<double>& g = map.gates[i];
        GateObservation<double> o{};
        o.gate_id = g.id;
        o.quality = 1;
        bool visible = true;
        const auto corners = gate_corners(g);
        for (std::size_t c = 0; c < 4 && visible; ++c) {
            const auto px = detail::project(cam, to_camera(cam, x, corners[c]));
            visible = px && (*px)[0] >= 0 && (*px)[0] < static_cast<double>(cam.width) && (*px)[1] >= 0 &&
                      (*px)[1] < static_cast<double>(cam.height);
            if (visible) {
                o.corners[c] = *px;
            }
        }
        if (!visible) {
            continue;
        }
        for (std::size_t k = 0; k < 8; ++k) {
            o.covariance[k * 8 + k] = corner_sigma_px * corner_sigma_px;
        }
        const Vec3<double> c = to_camera(cam, x, g.center);
        seen[n++] = Seen{std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]), o};
    }
    // Nearest first; ties keep course order (stable), so the result is deterministic.
    std::stable_sort(seen.begin(), seen.begin() + static_cast<std::ptrdiff_t>(n), [](const Seen& a, const Seen& b) {
        return a.distance < b.distance;
    });
    GateObservations<double> out{};
    out.count = static_cast<std::uint32_t>(std::min<std::size_t>(n, kMaxVisibleGates));
    for (std::uint32_t i = 0; i < out.count; ++i) {
        out.gates[i] = seen[i].obs;
    }
    return out;
}

}  // namespace branes::racing::sim
