#pragma once
/// @file score.hpp
/// @brief Scorer: the test plan §3 scoreboard metrics from a run's truth and estimates.
///
/// Fed the truth stream in time order (and the logged state estimates), it
/// tracks the course:
///
///  - **Gate passage.** Between consecutive truth samples the vehicle crosses
///    the next expected gate's plane in the direction of travel (gate-frame x
///    goes from < 0 to ≥ 0). The crossing point is interpolated linearly; it
///    is a passage if it lies within the inner opening (|y| ≤ w/2, |z| ≤ h/2).
///  - **Collision.** A crossing of any gate's plane, in either direction,
///    outside its opening but within kFrameMargin of it hits the frame. Ground
///    contact after the vehicle has left the pad, before the run is complete,
///    is a crash.
///  - **Geofence.** Any truth position outside the course geofence.
///  - **Speed.** max_speed is the largest true speed from takeoff until the
///    run is complete. With a speed limit (the scenario's speed_limit_m_s),
///    exceeding it fails the run.
///  - **Laps.** A lap is the course sequence passed in order. Lap 1 is timed
///    from takeoff (the first truth sample off the ground); each later lap
///    from the end of the one before.
///  - **gate_error.** At each passage, the true crossing point's distance from
///    the gate center, and the latest logged estimate's (t_valid ≤ crossing
///    time) distance from it.
///
/// success = every lap complete, no collision, no geofence breach, and no
/// speed above the limit.
///
/// Model
/// -----
///   map_      : CourseMap [owned, immutable]
///   limit_    : optional speed limit [m/s] [immutable]
///   prev_     : last truth sample
///   est_      : latest state estimate
///   next_     : index into the sequence of the next expected gate; lap_ the lap
///   passages_ : every passage so far, in order
///
/// Invariants
/// ----------
///   I1: truth samples arrive in non-decreasing time
///   I2: passages_ follows the course sequence, lap after lap
///   I3: complete() ⇔ lap_ == map_.laps

#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/estimate.hpp>
#include <branes/racing/msg/truth.hpp>
#include <branes/racing/sim/gates.hpp>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace branes::racing::sim {

/// How far around a gate's inner opening its frame reaches [m].
inline constexpr double kFrameMargin = 0.5;

struct GatePassage {
    std::int32_t gate_id{0};
    std::uint32_t lap{0};          ///< 1-based
    Timestamp t;                   ///< interpolated crossing time
    double true_error_m{0};        ///< crossing point to gate center
    double estimated_error_m{-1};  ///< estimate at the crossing to gate center; −1 = no estimate
};

struct Score {
    bool success{false};
    std::uint32_t laps_completed{0};
    std::uint32_t gates_passed{0};
    std::vector<double> lap_times_s;
    double total_time_s{0};
    std::vector<GatePassage> passages;
    bool collision{false};
    bool geofence_breach{false};
    double max_speed_m_s{0};  ///< from takeoff to completion
    bool speed_limit_exceeded{false};
    std::string failure;  ///< the first failure, if any
};

class Scorer {
public:
    /// Pre: valid(map); speed_limit, if given, > 0.
    explicit Scorer(const CourseMap<double>& map, std::optional<double> speed_limit = std::nullopt)
        : map_(map), limit_(speed_limit) {
        assert(valid(map_));             // Pre
        assert(!limit_ || *limit_ > 0);  // Pre
    }

    void on_estimate(const StateEstimate<double>& x) {
        est_ = x;
    }

    /// Pre: t ≥ the previous sample's t (I1).
    void on_truth(const TruthState<double>& x) {
        assert(!prev_ || prev_->t <= x.t);  // Pre / I1
        if (!contains(map_.geofence, x.position)) {
            fail_once(score_.geofence_breach, "geofence breach at t=" + seconds(x.t));
        }
        if (!x.on_ground && !took_off_) {
            took_off_ = true;
            lap_start_ = x.t;
            start_ = x.t;
        }
        if (took_off_ && !complete()) {
            const double v = std::sqrt(x.velocity[0] * x.velocity[0] + x.velocity[1] * x.velocity[1] +
                                       x.velocity[2] * x.velocity[2]);
            if (v > score_.max_speed_m_s) {
                score_.max_speed_m_s = v;
            }
            if (limit_ && v > *limit_) {
                fail_once(score_.speed_limit_exceeded, "speed limit exceeded at t=" + seconds(x.t));
            }
        }
        if (took_off_ && x.on_ground && !complete()) {
            fail_once(score_.collision, "ground contact at t=" + seconds(x.t));
        }
        if (prev_ && took_off_ && !complete()) {
            crossings(*prev_, x);
        }
        prev_ = x;
    }

    [[nodiscard]] bool complete() const noexcept {
        return lap_ == map_.laps;  // I3
    }

    /// True once the run has failed (collision, geofence, speed); it cannot recover.
    [[nodiscard]] bool failed() const noexcept {
        return !score_.failure.empty();
    }

    [[nodiscard]] Score score() const {
        Score s = score_;
        s.success = complete() && !s.collision && !s.geofence_breach && !s.speed_limit_exceeded;
        if (!complete() && s.failure.empty()) {
            s.failure = "incomplete: " + std::to_string(s.gates_passed) + " gates passed";
        }
        return s;
    }

private:
    static std::string seconds(Timestamp t) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%.3f s", static_cast<double>(t.ns) * 1e-9);
        return buf;
    }

    void fail_once(bool& flag, const std::string& why) {
        if (!flag && score_.failure.empty()) {
            score_.failure = why;
        }
        flag = true;
    }

    [[nodiscard]] const Gate<double>& gate(std::int32_t id) const {
        for (std::uint32_t i = 0; i < map_.gate_count; ++i) {
            if (map_.gates[i].id == id) {
                return map_.gates[i];
            }
        }
        assert(false && "sequence names a gate id the map does not have");  // valid(map) excludes it
        return map_.gates[0];
    }

    /// p in gate g's frame (x along the normal).
    [[nodiscard]] static Vec3<double> in_gate(const Gate<double>& g, const Vec3<double>& p) noexcept {
        const Vec3<double> d{p[0] - g.center[0], p[1] - g.center[1], p[2] - g.center[2]};
        return detail::rotate(detail::conjugate(g.orientation), d);
    }

    void crossings(const TruthState<double>& a, const TruthState<double>& b) {
        const std::int32_t expected = map_.sequence[next_];
        for (std::uint32_t i = 0; i < map_.gate_count; ++i) {
            const Gate<double>& g = map_.gates[i];
            const Vec3<double> pa = in_gate(g, a.position);
            const Vec3<double> pb = in_gate(g, b.position);
            const bool forward = pa[0] < 0 && pb[0] >= 0;
            const bool backward = pa[0] >= 0 && pb[0] < 0;
            if (!forward && !backward) {
                continue;
            }
            const double s = pa[0] / (pa[0] - pb[0]);  // crossing fraction along a → b
            const double y = pa[1] + s * (pb[1] - pa[1]);
            const double z = pa[2] + s * (pb[2] - pa[2]);
            const double hw = g.inner_width / 2;
            const double hh = g.inner_height / 2;
            const bool inside = std::abs(y) <= hw && std::abs(z) <= hh;
            const bool near = std::abs(y) <= hw + kFrameMargin && std::abs(z) <= hh + kFrameMargin;
            if (near && !inside) {
                fail_once(score_.collision, "hit the frame of gate " + std::to_string(g.id));
                continue;
            }
            if (!inside || !forward || g.id != expected) {
                continue;  // through an opening, but not the next gate in order
            }
            GatePassage p;
            p.gate_id = g.id;
            p.lap = lap_ + 1;
            p.t = Timestamp{a.t.ns + static_cast<std::int64_t>(std::llround(s * static_cast<double>(b.t.ns - a.t.ns)))};
            p.true_error_m = std::sqrt(y * y + z * z);
            if (est_ && est_->t_valid <= p.t) {
                const Vec3<double> e = in_gate(g, est_->position);
                p.estimated_error_m = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
            }
            score_.passages.push_back(p);
            ++score_.gates_passed;
            if (++next_ == map_.sequence_length) {
                next_ = 0;
                ++lap_;
                score_.laps_completed = lap_;
                score_.lap_times_s.push_back(static_cast<double>(p.t.ns - lap_start_.ns) * 1e-9);
                score_.total_time_s = static_cast<double>(p.t.ns - start_.ns) * 1e-9;
                lap_start_ = p.t;
            }
            return;  // one passage per segment
        }
    }

    CourseMap<double> map_;
    std::optional<double> limit_;
    std::optional<TruthState<double>> prev_;
    std::optional<StateEstimate<double>> est_;
    std::uint32_t next_{0};
    std::uint32_t lap_{0};
    bool took_off_{false};
    Timestamp start_{};
    Timestamp lap_start_{};
    Score score_;
};

/// One JSON line for a score (the scoreboard row).
[[nodiscard]] inline std::string to_json(const Score& s) {
    std::string out = "{\"success\":";
    out += s.success ? "true" : "false";
    out += ",\"laps_completed\":" + std::to_string(s.laps_completed);
    out += ",\"gates_passed\":" + std::to_string(s.gates_passed);
    char buf[64];
    out += ",\"lap_time_s\":[";
    for (std::size_t i = 0; i < s.lap_times_s.size(); ++i) {
        std::snprintf(buf, sizeof buf, "%s%.6f", i ? "," : "", s.lap_times_s[i]);
        out += buf;
    }
    std::snprintf(buf, sizeof buf, "],\"total_time_s\":%.6f", s.total_time_s);
    out += buf;
    out += ",\"gate_error_true_m\":[";
    for (std::size_t i = 0; i < s.passages.size(); ++i) {
        std::snprintf(buf, sizeof buf, "%s%.6f", i ? "," : "", s.passages[i].true_error_m);
        out += buf;
    }
    out += "],\"gate_error_estimated_m\":[";
    for (std::size_t i = 0; i < s.passages.size(); ++i) {
        std::snprintf(buf, sizeof buf, "%s%.6f", i ? "," : "", s.passages[i].estimated_error_m);
        out += buf;
    }
    out += "],\"collision\":";
    out += s.collision ? "true" : "false";
    out += ",\"geofence_breach\":";
    out += s.geofence_breach ? "true" : "false";
    std::snprintf(buf, sizeof buf, ",\"max_speed_m_s\":%.6f", s.max_speed_m_s);
    out += buf;
    out += ",\"speed_limit_exceeded\":";
    out += s.speed_limit_exceeded ? "true" : "false";
    out += ",\"failure\":\"";
    for (const char c : s.failure) {
        if (c == '"' || c == '\\') {
            out += '\\';
        }
        out += c;
    }
    out += "\"}";
    return out;
}

}  // namespace branes::racing::sim
