#pragma once
/// @file waypoint.hpp
/// @brief WaypointPlanner: carrot-following along a straight-line path through the gates.
///
/// Baseline B0's planner (docs/plan/b0/). On the first plan() it lays out the
/// path from the vehicle's position:
///
///   start → climb (straight up, to the first gate's height)
///         → for each lap, for each gate in sequence:
///             pre (center − d·n) → center → post (center + d·n)
///
/// where n is the gate normal and d the approach distance, so every crossing
/// is a straight line along the normal through the center. The path ends at
/// the last gate's post-point, where the planner holds.
///
/// Each plan() projects the position onto the current segment and advances to
/// the next segment once the projection passes the end or the vehicle is
/// within the switch radius of it. The reference is one sample:
///
///  - position: the point `lookahead` further along the path (the carrot);
///  - velocity: the segment direction × speed, where speed is the cruise
///    speed, the corner speed within the corner radius of a vertex that turns
///    by more than the corner angle, and 0 on the final segment;
///  - yaw: the segment heading (the next segment's, if this one is vertical);
///  - progress: the arc length of the projection.
///
/// The waypoints are not stored per lap: one lap's 3·sequence_length points
/// are, and waypoint k is computed from them, so the storage is fixed.
///
/// Model
/// -----
///   p_        : Params [immutable] — speeds and distances
///   built_    : bool — the path has been laid out (first plan())
///   start_, climb_ : Vec3 — the first two waypoints
///   lap_      : array<Vec3, 3·kMaxGateSequence> — one lap's pre/center/post points
///   n_        : uint64 — waypoint count, 2 + 3·sequence_length·laps
///   segment_  : uint64 — current segment (waypoint segment_ → segment_ + 1)
///   arc_base_ : T — path length before the current segment
///
/// Invariants
/// ----------
///   I1: built_ ⇒ n_ ≥ 5 and segment_ < n_ − 1      — constrains n_, segment_
///   I2: segment_ never decreases                  — constrains segment_
///   I3: the parameters satisfy params_valid()     — constrains p_

#include <branes/racing/anatomy_mobility.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/slots/context.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <string>

namespace branes::racing {

template <typename T>
class WaypointPlanner {
public:
    struct Params {
        T cruise_speed{4};        ///< [m/s]
        T corner_speed{T(2.5)};   ///< near sharp corners [m/s], ≤ cruise_speed
        T lookahead{T(0.3)};      ///< carrot distance along the path [m]
        T switch_radius{1};       ///< advance within this distance of a segment end [m], > lookahead
        T gate_approach{2};       ///< pre/post-point distance from a gate center [m]
        T corner_radius{T(1.5)};  ///< slow-down distance around a sharp corner [m]
        T corner_cos{T(0.5)};     ///< a vertex is sharp if the turn's cosine is below this (60°)
    };

    /// I3: every value finite; 0 < corner_speed ≤ cruise_speed; 0 ≤ lookahead <
    /// switch_radius (so a vehicle converging on a carrot held at a segment end
    /// comes within the switch radius and advances); gate_approach > 0;
    /// corner_radius ≥ 0; corner_cos ∈ [−1, 1].
    [[nodiscard]] static bool params_valid(const Params& p) noexcept {
        using std::isfinite;
        return isfinite(p.cruise_speed) && isfinite(p.corner_speed) && isfinite(p.lookahead) &&
               isfinite(p.switch_radius) && isfinite(p.gate_approach) && isfinite(p.corner_radius) &&
               isfinite(p.corner_cos) && p.corner_speed > T(0) && p.corner_speed <= p.cruise_speed &&
               p.lookahead >= T(0) && p.lookahead < p.switch_radius && p.gate_approach > T(0) &&
               p.corner_radius >= T(0) && p.corner_cos >= T(-1) && p.corner_cos <= T(1);
    }

    /// Default parameters. Post: I1–I3, no path yet.
    WaypointPlanner() = default;

    /// Pre: params_valid(p). Post: I1–I3, no path yet.
    explicit WaypointPlanner(const Params& p) : p_(p) {
        assert(params_valid(p_));  // Pre / I3
    }

    /// From the baseline parameters cruise_speed_m_s, corner_speed_m_s,
    /// lookahead_m, switch_radius_m, gate_approach_m, corner_radius_m, and
    /// corner_angle_deg (each optional, defaulting to Params{}).
    [[nodiscard]] static result<WaypointPlanner, std::string> from_context(const SlotContext& ctx) {
        const Params d{};
        const auto& b = ctx.baseline;
        const auto cruise = parameter_in(b, "cruise_speed_m_s", double(d.cruise_speed), 0.1, 50.0);
        const auto corner = parameter_in(b, "corner_speed_m_s", double(d.corner_speed), 0.1, 50.0);
        const auto look = parameter_in(b, "lookahead_m", double(d.lookahead), 0.0, 10.0);
        const auto sw = parameter_in(b, "switch_radius_m", double(d.switch_radius), 0.01, 10.0);
        const auto approach = parameter_in(b, "gate_approach_m", double(d.gate_approach), 0.1, 20.0);
        const auto radius = parameter_in(b, "corner_radius_m", double(d.corner_radius), 0.0, 20.0);
        const auto angle = parameter_in(b, "corner_angle_deg", 60.0, 0.0, 180.0);
        for (const auto* r : {&cruise, &corner, &look, &sw, &approach, &radius, &angle}) {
            if (!*r) {
                return r->error();
            }
        }
        Params p;
        p.cruise_speed = T(*cruise);
        p.corner_speed = T(*corner);
        p.lookahead = T(*look);
        p.switch_radius = T(*sw);
        p.gate_approach = T(*approach);
        p.corner_radius = T(*radius);
        p.corner_cos = T(std::cos(*angle * 3.141592653589793 / 180.0));
        if (!params_valid(p)) {
            return std::string("WaypointPlanner: need corner_speed_m_s ≤ cruise_speed_m_s and "
                               "lookahead_m < switch_radius_m");
        }
        return WaypointPlanner(p);
    }

    /// One reference sample for state x.
    /// Pre: valid(map); x.position finite; on later calls, the same map.
    /// Post: I1–I3; valid(result), result.count == 1.
    [[nodiscard]] Reference<T> plan(const CourseMap<T>& map, const StateEstimate<T>& x) {
        if (!built_) {
            build(map, x.position);
        }
        [[maybe_unused]] const std::uint64_t before = segment_;
        const Vec3<T>& p = x.position;

        // Advance while past the end or within the switch radius (never onto
        // the hold point itself: segment_ < n_ − 1, I1).
        T s = projection(p);
        while (segment_ + 2 < n_ && (s >= T(1) || distance(p, waypoint(segment_ + 1)) < p_.switch_radius)) {
            arc_base_ += distance(waypoint(segment_), waypoint(segment_ + 1));
            ++segment_;
            s = projection(p);
        }
        s = s < T(0) ? T(0) : (s > T(1) ? T(1) : s);

        const Vec3<T> a = waypoint(segment_);
        const Vec3<T> b = waypoint(segment_ + 1);
        const T len = distance(a, b);
        const Vec3<T> dir = len > T(0) ? scaled(sub(b, a), T(1) / len) : Vec3<T>{};
        const bool final_segment = segment_ + 2 == n_;
        T speed = p_.cruise_speed;
        if (final_segment) {
            speed = T(0);
        } else if (((T(1) - s) * len < p_.corner_radius && sharp(segment_ + 1)) ||
                   (s * len < p_.corner_radius && sharp(segment_))) {
            speed = p_.corner_speed;
        }

        Reference<T> r{};
        r.count = 1;
        ReferenceSample<T>& out = r.samples[0];
        out.t = x.t_valid;
        out.position = carrot(add(a, scaled(sub(b, a), s)), (T(1) - s) * len);
        out.velocity = scaled(dir, speed);
        out.yaw = heading(map);
        out.progress = arc_base_ + s * len;

        assert(segment_ >= before);  // I2
        assert(check_invariant());
        return r;
    }

    /// Path inspection, for tests and tools. Pre for waypoint(): k < waypoint_count().
    [[nodiscard]] bool built() const noexcept {
        return built_;
    }
    [[nodiscard]] std::uint64_t waypoint_count() const noexcept {
        return n_;
    }
    [[nodiscard]] std::uint64_t segment() const noexcept {
        return segment_;
    }
    [[nodiscard]] Vec3<T> waypoint(std::uint64_t k) const noexcept {
        assert(built_ && k < n_);  // Pre
        if (k == 0) {
            return start_;
        }
        if (k == 1) {
            return climb_;
        }
        return lap_[(k - 2) % lap_points_];
    }
    [[nodiscard]] const Params& params() const noexcept {
        return p_;
    }

private:
    [[nodiscard]] static Vec3<T> add(const Vec3<T>& a, const Vec3<T>& b) noexcept {
        return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }
    [[nodiscard]] static Vec3<T> sub(const Vec3<T>& a, const Vec3<T>& b) noexcept {
        return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }
    [[nodiscard]] static Vec3<T> scaled(const Vec3<T>& a, T k) noexcept {
        return {k * a[0], k * a[1], k * a[2]};
    }
    [[nodiscard]] static T dot(const Vec3<T>& a, const Vec3<T>& b) noexcept {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }
    [[nodiscard]] static T distance(const Vec3<T>& a, const Vec3<T>& b) noexcept {
        using std::sqrt;
        const Vec3<T> d = sub(b, a);
        return sqrt(dot(d, d));
    }

    /// The gate's x axis (its normal) in the course frame.
    [[nodiscard]] static Vec3<T> normal(const Quat<T>& q) noexcept {
        return {T(1) - T(2) * (q.y * q.y + q.z * q.z), T(2) * (q.x * q.y + q.w * q.z), T(2) * (q.x * q.z - q.w * q.y)};
    }

    void build(const CourseMap<T>& map, const Vec3<T>& from) {
        assert(valid(map));  // Pre
        lap_points_ = 0;
        for (std::uint32_t i = 0; i < map.sequence_length; ++i) {
            const Gate<T>* g = &map.gates[0];
            for (std::uint32_t k = 0; k < map.gate_count; ++k) {
                if (map.gates[k].id == map.sequence[i]) {
                    g = &map.gates[k];
                }
            }
            const Vec3<T> n = scaled(normal(g->orientation), p_.gate_approach);
            lap_[lap_points_++] = sub(g->center, n);
            lap_[lap_points_++] = g->center;
            lap_[lap_points_++] = add(g->center, n);
        }
        start_ = from;
        climb_ = {from[0], from[1], lap_[1][2]};  // the first gate center's height
        n_ = 2 + std::uint64_t{lap_points_} * map.laps;
        segment_ = 0;
        arc_base_ = T(0);
        built_ = true;
    }

    /// The projection parameter of p on the current segment (1 for a zero-length one).
    [[nodiscard]] T projection(const Vec3<T>& p) const noexcept {
        const Vec3<T> a = waypoint(segment_);
        const Vec3<T> ab = sub(waypoint(segment_ + 1), a);
        const T l2 = dot(ab, ab);
        return l2 > T(0) ? dot(sub(p, a), ab) / l2 : T(1);
    }

    /// True if the path turns at waypoint k by more than the corner angle
    /// (or a neighbouring segment has zero length). The ends never turn.
    [[nodiscard]] bool sharp(std::uint64_t k) const noexcept {
        if (k == 0 || k + 1 >= n_) {
            return false;
        }
        const Vec3<T> u = sub(waypoint(k), waypoint(k - 1));
        const Vec3<T> v = sub(waypoint(k + 1), waypoint(k));
        using std::sqrt;
        const T uv = sqrt(dot(u, u) * dot(v, v));
        return !(uv > T(0)) || dot(u, v) < p_.corner_cos * uv;
    }

    /// The point `lookahead` along the path from `from`, which lies on the
    /// current segment `remaining` before its end; it stops at the path's end.
    [[nodiscard]] Vec3<T> carrot(const Vec3<T>& from, T remaining) const noexcept {
        T left = p_.lookahead;
        if (left <= remaining) {
            const Vec3<T> a = waypoint(segment_);
            const Vec3<T> b = waypoint(segment_ + 1);
            const T len = distance(a, b);
            return len > T(0) ? add(from, scaled(sub(b, a), left / len)) : from;
        }
        left -= remaining;
        for (std::uint64_t k = segment_ + 1; k + 1 < n_; ++k) {
            const Vec3<T> a = waypoint(k);
            const Vec3<T> b = waypoint(k + 1);
            const T len = distance(a, b);
            if (left <= len) {
                return add(a, scaled(sub(b, a), left / len));
            }
            left -= len;
        }
        return waypoint(n_ - 1);
    }

    /// The heading of the current segment, or of the next if this one is
    /// (nearly) vertical, or the start yaw.
    [[nodiscard]] T heading(const CourseMap<T>& map) const noexcept {
        using std::atan2;
        using std::sqrt;
        for (std::uint64_t k = segment_; k <= segment_ + 1 && k + 1 < n_; ++k) {
            const Vec3<T> d = sub(waypoint(k + 1), waypoint(k));
            const T horizontal = sqrt(d[0] * d[0] + d[1] * d[1]);
            if (horizontal > T(0.1) * sqrt(dot(d, d))) {
                return atan2(d[1], d[0]);
            }
        }
        return map.start.yaw;
    }

    [[nodiscard]] bool check_invariant() const noexcept {
        return params_valid(p_) && (!built_ || (n_ >= 5 && segment_ + 1 < n_));  // I3, I1
    }

    Params p_{};
    bool built_{false};
    Vec3<T> start_{};
    Vec3<T> climb_{};
    std::array<Vec3<T>, 3 * kMaxGateSequence> lap_{};
    std::uint32_t lap_points_{0};
    std::uint64_t n_{0};
    std::uint64_t segment_{0};
    T arc_base_{0};
};

}  // namespace branes::racing
