#pragma once
/// @file replay_trace.hpp
/// @brief A SITL log as a reflex-trace/1 replay, for the docs site's trace player.
///
/// The docs site plays mission replays in the player reflex publishes,
/// `@branes-ai/trace-player` (reflex docs/standards/animation, "Boundary with
/// mission SITL"). replay_trace() reads a log once and samples its truth
/// stream on a uniform grid:
///
///  - **Channels**, the format's reserved names: position and velocity
///    [m, m/s, world], attitude [w, x, y, z, body → world], body_rates
///    [rad/s, body], rotor_speeds [rad/s], and rotor_thrusts [N], k_f·ω² with
///    the vehicle's thrust coefficient. With references in the log, also
///    position_setpoint [m]: the latest reference sample logged before the
///    trace sample. Frames are the format's: z-up world, x-forward y-left
///    z-up body, and PX4 Quad X rotor order, which is the order of reflex's
///    plant, which SITL runs.
///  - **Events**: takeoff, and each gate passage the Scorer finds (sim/score.hpp),
///    fed every truth sample, not only the sampled ones.
///  - **Span**: from the first truth sample to the end of the log, or, once
///    the course is complete, to `tail_s` after the last passage.
///
/// The JSON follows the rules of reflex's writer (branes::reflex_trace),
/// which racing-drone's reflex pin does not have yet; once it does, the
/// writer below can give way to it.
///
/// Model (ReplayTrace)
/// -------------------
///   prov_    : provenance [immutable]
///   names_   : channel names, in the order they are written; widths_ their widths
///   t_       : sample times [s]
///   rows_    : rows_[c][k·widths_[c] + j] is component j of channel c at t_[k]
///   events_  : (time [s], label)
///
/// Invariants
/// ----------
///   T1: t_[k] = k·prov_.dt_trace, so t_ starts at 0 and increases strictly
///   T2: every channel holds one entry of its fixed width per sample; every value is finite
///   T3: event times lie within [0, t_.back()]

#include <branes/racing/anatomy.hpp>
#include <branes/racing/anatomy_mobility.hpp>
#include <branes/racing/config/config.hpp>
#include <branes/racing/log/channels.hpp>
#include <branes/racing/sim/score.hpp>

#include <array>
#include <cassert>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace branes::racing::sim {

/// What a replay is (reflex-trace/1 provenance).
struct ReplayProvenance {
    std::string commit;    ///< short git hash of the tree that recorded and exported the run
    std::string baseline;  ///< the log's baseline, e.g. "b0-gt-pid" (the trace's block)
    std::string scenario;  ///< repo-relative path of the scenario file
    std::uint64_t seed{0};
    double dt_sim{0};    ///< physics step [s]
    double dt_trace{0};  ///< sample spacing [s]
};

class ReplayTrace {
public:
    /// Pre: dt_sim > 0; dt_trace > 0.
    explicit ReplayTrace(ReplayProvenance p) : prov_(std::move(p)) {
        assert(prov_.dt_sim > 0 && prov_.dt_trace > 0);  // Pre
    }

    /// Declares a channel. Pre: no samples yet; width > 0.
    void channel(std::string name, std::size_t width) {
        assert(t_.empty() && width > 0);  // Pre
        names_.push_back(std::move(name));
        widths_.push_back(width);
        rows_.emplace_back();
    }

    /// Appends sample k = size() at t = k·dt_trace (T1), one value span per
    /// channel in declaration order. Returns false, and leaves the trace as
    /// it was, if a span has the wrong width or a value is not finite (T2).
    bool sample(std::span<const std::span<const double>> values) {
        if (values.size() != names_.size()) {
            return false;
        }
        for (std::size_t c = 0; c < values.size(); ++c) {
            if (values[c].size() != widths_[c]) {
                return false;
            }
            for (const double v : values[c]) {
                if (!std::isfinite(v)) {
                    return false;
                }
            }
        }
        for (std::size_t c = 0; c < values.size(); ++c) {
            rows_[c].insert(rows_[c].end(), values[c].begin(), values[c].end());
        }
        t_.push_back(static_cast<double>(t_.size()) * prov_.dt_trace);
        return true;
    }

    /// Drops the samples after the first n (n ≤ size()), and any event after the new end (T3).
    void truncate(std::size_t n) {
        assert(n <= t_.size());  // Pre
        t_.resize(n);
        for (std::size_t c = 0; c < rows_.size(); ++c) {
            rows_[c].resize(n * widths_[c]);
        }
        std::erase_if(events_, [&](const auto& e) { return n == 0 || e.first > t_.back(); });
    }

    /// Adds an event; one outside [0, t_.back()] is dropped (T3), as is one before any sample.
    void event(double t, std::string label) {
        if (!t_.empty() && t >= 0 && t <= t_.back() && !label.empty()) {
            events_.emplace_back(t, std::move(label));
        }
    }

    [[nodiscard]] std::size_t size() const noexcept {
        return t_.size();
    }
    [[nodiscard]] const std::vector<std::pair<double, std::string>>& events() const noexcept {
        return events_;
    }

    /// The trace as reflex-trace/1 JSON. Values carry 7 significant digits,
    /// ample for display and within the format's 1e-6 quaternion-norm
    /// tolerance. Pre: size() > 0.
    [[nodiscard]] std::string to_json(double arm_length) const {
        assert(!t_.empty());  // Pre
        std::string out = "{\"schema\":\"reflex-trace/1\",\"provenance\":{";
        out += "\"commit\":" + quote(prov_.commit.empty() ? "unknown" : prov_.commit);
        out += ",\"block\":" + quote(prov_.baseline);
        out += ",\"scenario\":" + quote(prov_.scenario);
        out += ",\"seed\":" + std::to_string(prov_.seed);
        out += ",\"compiler\":" + quote(compiler_id());
        out += ",\"dt_sim\":" + number(prov_.dt_sim);
        out += ",\"dt_trace\":" + number(prov_.dt_trace);
        out += "},\"frames\":{\"world\":\"z-up\",\"body\":\"x-forward y-left z-up\",\"rotors\":\"px4-quad-x\"}";
        if (arm_length > 0) {
            out += ",\"vehicle\":{\"arm_length\":" + number(arm_length) + "}";
        }
        out += ",\n\"t\":[";
        for (std::size_t k = 0; k < t_.size(); ++k) {
            out += (k ? "," : "") + number(t_[k]);
        }
        out += "],\n\"channels\":{";
        for (std::size_t c = 0; c < names_.size(); ++c) {
            out += (c ? ",\n" : "\n") + quote(names_[c]) + ":[";
            const std::size_t w = widths_[c];
            for (std::size_t k = 0; k < t_.size(); ++k) {
                out += k ? "," : "";
                if (w == 1) {
                    out += number(rows_[c][k]);
                    continue;
                }
                out += "[";
                for (std::size_t j = 0; j < w; ++j) {
                    out += (j ? "," : "") + number(rows_[c][k * w + j]);
                }
                out += "]";
            }
            out += "]";
        }
        out += "},\n\"events\":[";
        for (std::size_t i = 0; i < events_.size(); ++i) {
            out += (i ? ",\n" : "\n") + std::string("{\"t\":") + number(events_[i].first) +
                   ",\"label\":" + quote(events_[i].second) + "}";
        }
        out += "]}\n";
        return out;
    }

private:
    static std::string number(double v) {
        if (v == 0) {
            return "0";  // also folds −0
        }
        std::array<char, 32> buf{};
        const auto [end, ec] = std::to_chars(buf.data(), buf.data() + buf.size(), v, std::chars_format::general, 7);
        assert(ec == std::errc{});
        return std::string(buf.data(), end);
    }

    static std::string quote(std::string_view s) {
        std::string out = "\"";
        for (const char c : s) {
            if (c == '"' || c == '\\') {
                out += '\\';
                out += c;
            } else if (static_cast<unsigned char>(c) < 0x20) {
                char esc[8];
                std::snprintf(esc, sizeof esc, "\\u%04x", static_cast<unsigned>(c));
                out += esc;
            } else {
                out += c;
            }
        }
        return out + "\"";
    }

    static std::string compiler_id() {
        std::string id;
#if defined(__clang__)
        id = "clang " + std::to_string(__clang_major__) + "." + std::to_string(__clang_minor__);
#elif defined(__GNUC__)
        id = "gcc " + std::to_string(__GNUC__) + "." + std::to_string(__GNUC_MINOR__);
#elif defined(_MSC_VER)
        id = "msvc " + std::to_string(_MSC_VER);
#else
        id = "unknown";
#endif
#if defined(NDEBUG)
        id += " optimized";
#else
        id += " debug";
#endif
        return id;
    }

    ReplayProvenance prov_;
    std::vector<std::string> names_;
    std::vector<std::size_t> widths_;
    std::vector<double> t_;
    std::vector<std::vector<double>> rows_;
    std::vector<std::pair<double, std::string>> events_;
};

/// How to sample a run.
struct ReplayOptions {
    double rate_hz{20};  ///< sample rate; 1e9 / rate_hz must be a multiple of the physics step [ns]
    double tail_s{2};    ///< kept after the last passage of a complete course [s]
};

/// A replay and its vehicle's arm length, for ReplayTrace::to_json().
struct Replay {
    ReplayTrace trace;
    double arm_length{0};
};

/// Reads every record of `reader` and samples it as a replay. `prov` names
/// the run; dt_trace is set from `options`. Fails on a log with no truth,
/// with truth samples off the sample grid, or with an undecodable record.
/// Pre: options.rate_hz > 0; options.tail_s ≥ 0.
[[nodiscard]] inline result<Replay, std::string>
replay_trace(log::LogReader& reader, ReplayProvenance prov, const ReplayOptions& options) {
    assert(options.rate_hz > 0 && options.tail_s >= 0);  // Pre
    const log::LogHeader& h = reader.header();
    auto course = config::parse_course(h.world_yaml, "course (from the log header)");
    if (!course) {
        return course.error().to_string();
    }
    auto vehicle = config::parse_vehicle(h.vehicle_yaml, "vehicle (from the log header)");
    if (!vehicle) {
        return vehicle.error().to_string();
    }
    const double k_f = vehicle->rotor.thrust_coefficient;
    const auto step_ns = static_cast<std::int64_t>(std::llround(1e9 / options.rate_hz));
    if (step_ns <= 0) {
        return std::string("the sample rate is too high");
    }
    prov.dt_trace = static_cast<double>(step_ns) * 1e-9;

    // The reference channel is declared only if the log has one, so a pass
    // over the log decides the channels before the first sample is added.
    struct Sample {
        TruthState<double> x;
        std::optional<Vec3<double>> setpoint;
    };
    std::vector<Sample> samples;
    Scorer scorer(course->map);
    std::optional<Timestamp> t0;
    std::optional<Timestamp> takeoff;
    std::optional<Vec3<double>> setpoint;
    bool any_reference = false;
    for (;;) {
        auto rec = reader.next();
        if (!rec) {
            return rec.error().path + ": " + rec.error().message;
        }
        if (!rec->has_value()) {
            break;
        }
        const log::LogRecord& r = **rec;
        if (r.channel == log::Channel::reference) {
            const auto ref = codec::decode_reference(r.payload);
            if (!ref) {
                return "undecodable reference record at t=" + std::to_string(r.t.ns);
            }
            if (ref->count > 0) {
                setpoint = ref->samples[0].position;
                any_reference = true;
            }
        } else if (r.channel == log::Channel::state) {
            const auto x = codec::decode_state(r.payload);
            if (!x) {
                return "undecodable state record at t=" + std::to_string(r.t.ns);
            }
            scorer.on_estimate(*x);
        } else if (r.channel == log::Channel::truth) {
            const auto x = codec::decode_truth(r.payload);
            if (!x) {
                return "undecodable truth record at t=" + std::to_string(r.t.ns);
            }
            if (!samples.empty() && x->t < samples.back().x.t) {
                return "truth record at t=" + std::to_string(x->t.ns) + " is earlier than the one before it";
            }
            scorer.on_truth(*x);
            t0 = t0.value_or(x->t);
            if (!takeoff && !x->on_ground) {
                takeoff = x->t;
            }
            const std::int64_t since = x->t.ns - t0->ns;
            if (since % step_ns != 0) {
                continue;
            }
            if (static_cast<std::size_t>(since / step_ns) != samples.size()) {
                return "the truth stream has no sample at t=" +
                       std::to_string(t0->ns + static_cast<std::int64_t>(samples.size()) * step_ns) +
                       " ns: choose a rate whose period is a multiple of the physics step";
            }
            samples.push_back({*x, setpoint});
        }
    }
    if (samples.empty()) {
        return std::string("the log has no truth channel: export SITL logs only");
    }

    ReplayTrace trace(prov);
    trace.channel("position", 3);
    trace.channel("velocity", 3);
    trace.channel("attitude", 4);
    trace.channel("body_rates", 3);
    trace.channel("rotor_speeds", 4);
    trace.channel("rotor_thrusts", 4);
    if (any_reference) {
        trace.channel("position_setpoint", 3);
    }
    for (const Sample& s : samples) {
        const TruthState<double>& x = s.x;
        const std::array<double, 4> q{x.attitude.w, x.attitude.x, x.attitude.y, x.attitude.z};
        std::array<double, 4> thrust{};
        for (std::size_t i = 0; i < 4; ++i) {
            thrust[i] = k_f * x.rotor_speeds[i] * x.rotor_speeds[i];
        }
        // Before the first reference, the setpoint holds the vehicle's own position.
        const Vec3<double> sp = s.setpoint.value_or(x.position);
        std::vector<std::span<const double>> row{x.position, x.velocity, q, x.body_rates, x.rotor_speeds, thrust};
        if (any_reference) {
            row.emplace_back(sp);
        }
        if (!trace.sample(row)) {
            return "a value at t=" + std::to_string(x.t.ns) + " ns is not finite";
        }
    }

    const auto seconds = [&](Timestamp t) { return static_cast<double>(t.ns - t0->ns) * 1e-9; };
    const Score score = scorer.score();
    if (scorer.complete() && !score.passages.empty()) {
        const double end = seconds(score.passages.back().t) + options.tail_s;
        const auto keep = static_cast<std::size_t>(std::floor(end / prov.dt_trace)) + 1;
        if (keep < trace.size()) {
            trace.truncate(keep);
        }
    }
    if (takeoff) {
        trace.event(seconds(*takeoff), "takeoff");
    }
    for (const GatePassage& p : score.passages) {
        trace.event(seconds(p.t), "gate " + std::to_string(p.gate_id) + ", lap " + std::to_string(p.lap));
    }
    return Replay{std::move(trace), vehicle->arm_length};
}

}  // namespace branes::racing::sim
