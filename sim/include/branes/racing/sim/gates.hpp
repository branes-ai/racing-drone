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
///  - Camera frame: z forward, x right, y down (the vehicle file's convention);
///    rotation_body maps camera to body, translation_body is the camera origin
///    in the body frame.
///  - Equidistant (Kannala–Brandt): θ = atan2(r, z), r = √(x² + y²),
///    θ_d = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸), (u, v) = (fx θ_d x/r + cx, fy θ_d y/r + cy).
///    Pinhole–radtan: (x', y') = (x/z, y/z) with radial k1, k2 and tangential
///    p1, p2 distortion.
///
/// A gate is reported only if all four corners lie in front of the camera
/// (z > 0) and inside the image. At most kMaxVisibleGates are reported,
/// nearest gate center first. Each has quality 1 and a diagonal corner
/// covariance of σ² per coordinate.
///
/// Not modeled: occlusion by other gates, rolling shutter (the projection is
/// at the frame's mid-exposure pose), motion blur.

#include <branes/racing/config/config.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/geometry.hpp>
#include <branes/racing/msg/perception.hpp>
#include <branes/racing/msg/truth.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace branes::racing::sim {

namespace detail {

/// v rotated by unit quaternion q (Hamilton, scalar first).
[[nodiscard]] inline Vec3<double> rotate(const Quat<double>& q, const Vec3<double>& v) noexcept {
    // t = 2 q_v × v;  v' = v + w t + q_v × t
    const Vec3<double> u{q.x, q.y, q.z};
    const auto cross = [](const Vec3<double>& a, const Vec3<double>& b) {
        return Vec3<double>{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    };
    Vec3<double> t = cross(u, v);
    for (double& c : t) {
        c *= 2;
    }
    const Vec3<double> ut = cross(u, t);
    return {v[0] + q.w * t[0] + ut[0], v[1] + q.w * t[1] + ut[1], v[2] + q.w * t[2] + ut[2]};
}

[[nodiscard]] inline Quat<double> conjugate(const Quat<double>& q) noexcept {
    return {q.w, -q.x, -q.y, -q.z};
}

/// Pixel coordinates of camera-frame point `p`, or nullopt behind the camera.
[[nodiscard]] inline std::optional<std::array<double, 2>> project(const config::CameraParams& cam,
                                                                  const Vec3<double>& p) noexcept {
    if (!(p[2] > 0)) {
        return std::nullopt;
    }
    const auto& k = cam.distortion;
    if (cam.model == config::CameraModel::equidistant) {
        const double r = std::sqrt(p[0] * p[0] + p[1] * p[1]);
        if (r == 0) {
            return std::array<double, 2>{cam.cx, cam.cy};
        }
        const double th = std::atan2(r, p[2]);
        const double th2 = th * th;
        const double thd = th * (1 + th2 * (k[0] + th2 * (k[1] + th2 * (k[2] + th2 * k[3]))));
        return std::array<double, 2>{cam.fx * thd * p[0] / r + cam.cx, cam.fy * thd * p[1] / r + cam.cy};
    }
    const double x = p[0] / p[2];
    const double y = p[1] / p[2];
    const double r2 = x * x + y * y;
    const double radial = 1 + r2 * (k[0] + r2 * k[1]);
    const double xd = x * radial + 2 * k[2] * x * y + k[3] * (r2 + 2 * x * x);
    const double yd = y * radial + k[2] * (r2 + 2 * y * y) + 2 * k[3] * x * y;
    return std::array<double, 2>{cam.fx * xd + cam.cx, cam.fy * yd + cam.cy};
}

}  // namespace detail

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

/// Course-frame point → camera frame, for the vehicle at `x`.
[[nodiscard]] inline Vec3<double>
to_camera(const config::CameraParams& cam, const TruthState<double>& x, const Vec3<double>& p) noexcept {
    const Vec3<double> d{p[0] - x.position[0], p[1] - x.position[1], p[2] - x.position[2]};
    const Vec3<double> b = detail::rotate(detail::conjugate(x.attitude), d);
    const Vec3<double> bc{
        b[0] - cam.translation_body[0], b[1] - cam.translation_body[1], b[2] - cam.translation_body[2]};
    return detail::rotate(detail::conjugate(cam.rotation_body), bc);
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
