#pragma once
/// @file harness.hpp
/// @brief Pieces shared by the harness executables (flight_stack, replay, sitl).
///
///  - Args: `--key value` / `--flag` command-line parsing.
///  - check_baseline(): the startup check of ADR 0001 — the baseline file
///    matches the name and composition hash compiled into the binary.
///  - Runner: feeds log records to a Stack and encodes its outputs. SITL and
///    replay both go through it, so a replay of a SITL log reproduces the
///    SITL outputs record for record.
///  - SyntheticSensors: the null SITL's deterministic IMU / camera / tick
///    event source.

#include <branes/racing/compose/stack.hpp>
#include <branes/racing/config/config.hpp>
#include <branes/racing/log/log.hpp>
#include <branes/racing/msg/codec.hpp>
#include <branes/racing/msg/time.hpp>
#include <branes/racing/result.hpp>
#include <branes/racing/util/sha256.hpp>
#include <branes/racing/util/splitmix64.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace branes::racing::harness {

// ── command line ────────────────────────────────────────────────────

/// Parsed `--key value` options and `--flag` switches.
class Args {
public:
    /// `flags` are the switches that take no value; every other `--name`
    /// takes the next argument as its value. Unknown names are accepted here
    /// and rejected by only().
    [[nodiscard]] static result<Args, std::string>
    parse(int argc, const char* const* argv, const std::set<std::string>& flags) {
        Args a;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg.rfind("--", 0) != 0 || arg.size() < 3) {
                return std::string("unexpected argument '" + arg + "'");
            }
            const std::string name = arg.substr(2);
            if (flags.count(name) != 0) {
                a.flags_.insert(name);
            } else if (i + 1 < argc) {
                a.values_[name] = argv[++i];
            } else {
                return std::string("option --" + name + " needs a value");
            }
        }
        return a;
    }

    /// Error text naming the first option outside `allowed`, if any.
    [[nodiscard]] std::optional<std::string> only(const std::set<std::string>& allowed) const {
        for (const auto& [k, v] : values_) {
            if (allowed.count(k) == 0) {
                return "option --" + k + " is not accepted here";
            }
        }
        for (const auto& f : flags_) {
            if (allowed.count(f) == 0) {
                return "option --" + f + " is not accepted here";
            }
        }
        return std::nullopt;
    }

    [[nodiscard]] std::optional<std::string> value(const std::string& name) const {
        const auto it = values_.find(name);
        return it == values_.end() ? std::nullopt : std::optional<std::string>(it->second);
    }
    [[nodiscard]] bool has(const std::string& name) const {
        return values_.count(name) != 0 || flags_.count(name) != 0;
    }

private:
    std::map<std::string, std::string> values_;
    std::set<std::string> flags_;
};

// ── baseline check ──────────────────────────────────────────────────

/// Load `<dir>/<name>.yaml` and confirm it matches the binary (ADR 0001):
/// same name, same composition hash. The binary refuses to start otherwise.
[[nodiscard]] inline result<config::BaselineConfig, std::string>
check_baseline(const std::filesystem::path& dir, std::string_view name, std::string_view compiled_hash) {
    const auto path = dir / (std::string(name) + ".yaml");
    auto b = config::load_baseline(path);
    if (!b) {
        return b.error().to_string();
    }
    if (b->name != name) {
        return std::string(path.string() + ": baseline name '" + b->name + "' does not match the binary ('" +
                           std::string(name) + "')");
    }
    if (const std::string h = config::composition_hash(*b); h != compiled_hash) {
        return std::string(path.string() + ": composition changed since this binary was built (hash " + h +
                           ", built with " + std::string(compiled_hash) + "); rebuild");
    }
    return std::move(b).value();
}

/// A file's text and SHA-256, for embedding in a log header.
struct FileContent {
    std::string text;
    std::string hash;
};

[[nodiscard]] inline std::optional<FileContent> read_with_hash(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return std::nullopt;
    }
    std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    std::string hash = sha256_hex(text);
    return FileContent{std::move(text), std::move(hash)};
}

// ── runner ──────────────────────────────────────────────────────────

/// One record the stack produced.
struct Output {
    log::Channel channel;
    Timestamp t;
    std::vector<std::byte> payload;
};

/// Feeds input records to a Stack and returns the encoded outputs.
///
/// The stack runs in double for the harness logs. Inputs that do not decode
/// are reported, never silently skipped: a log that does not replay cleanly is
/// not replay-complete (ADR 0006).
template <class StackT>
class Runner {
public:
    explicit Runner(StackT& stack) noexcept : stack_(stack) {}

    /// Process one input record. Pre: !log::is_output(rec.channel).
    [[nodiscard]] result<std::vector<Output>, std::string> feed(const log::LogRecord& rec) {
        assert(!log::is_output(rec.channel));  // Pre
        std::vector<Output> out;
        switch (rec.channel) {
        case log::Channel::imu: {
            const auto s = codec::decode_imu(rec.payload);
            if (!s) {
                return std::string("undecodable IMU record at t=" + std::to_string(rec.t.ns));
            }
            stack_.on_imu(*s);
            break;
        }
        case log::Channel::camera: {
            const auto f = codec::decode_frame(rec.payload);
            if (!f) {
                return std::string("undecodable camera record at t=" + std::to_string(rec.t.ns));
            }
            stack_.on_frame(view_of(*f));
            break;
        }
        case log::Channel::tick: {
            const TickOutput<double> y = stack_.on_tick(rec.t);
            out.push_back(Output{log::Channel::state, rec.t, codec::encode(y.state)});
            out.push_back(Output{log::Channel::command, rec.t, codec::encode(y.command)});
            break;
        }
        default:
            return std::string("not an input channel");
        }
        return out;
    }

private:
    StackT& stack_;
};

// ── synthetic sensors (null SITL) ───────────────────────────────────

/// One input event of the synthetic sensor stream.
struct SensorEvent {
    log::Channel channel;
    Timestamp t;
    std::vector<std::byte> payload;
};

/// Deterministic IMU / camera / tick stream for a vehicle at rest: gravity on
/// the accelerometer, white noise from the vehicle's noise densities, and a
/// test-pattern image. The same seed gives the same bytes on every platform
/// that rounds IEEE-754 the same way.
///
/// Model
/// -----
///   rng_          : SplitMix64 — the only random source
///   next_imu_, next_frame_, next_tick_ : int64 ns — next event time per stream
///   end_          : int64 ns — events at t ≥ end_ are not produced
///   frame_index_  : uint32 — frames produced so far
///
/// Invariants
/// ----------
///   I1: events come out in non-decreasing time; at equal times the order is
///       IMU, then camera, then tick                  — constrains next_*
///   I2: every period is > 0                          — constrains *_period_
class SyntheticSensors {
public:
    /// Pre: vehicle loaded (its invariants hold); duration_s > 0; control_hz > 0.
    SyntheticSensors(const config::VehicleConfig& vehicle, double duration_s, std::uint64_t seed, double control_hz)
        : vehicle_(vehicle), rng_(seed), imu_period_(period_ns(vehicle.imu.rate_hz)),
          frame_period_(period_ns(vehicle.camera.rate_hz)), tick_period_(period_ns(control_hz)),
          end_(static_cast<std::int64_t>(duration_s * 1e9)) {
        assert(imu_period_ > 0 && frame_period_ > 0 && tick_period_ > 0);  // I2
        // White noise of density d sampled at rate f has σ = d·√f; a uniform
        // variable on [−a, a) has σ = a/√3.
        gyro_amp_ = vehicle.imu.gyro_noise_density * std::sqrt(vehicle.imu.rate_hz) * std::sqrt(3.0);
        accel_amp_ = vehicle.imu.accel_noise_density * std::sqrt(vehicle.imu.rate_hz) * std::sqrt(3.0);
    }

    /// The next event, or nullopt once every stream has passed the end.
    [[nodiscard]] std::optional<SensorEvent> next() {
        const std::int64_t t = std::min({next_imu_, next_frame_, next_tick_});
        if (t >= end_) {
            return std::nullopt;
        }
        if (next_imu_ == t) {  // I1: IMU first at equal times
            next_imu_ += imu_period_;
            return SensorEvent{log::Channel::imu, Timestamp{t}, codec::encode(imu_at(t))};
        }
        if (next_frame_ == t) {
            next_frame_ += frame_period_;
            return SensorEvent{log::Channel::camera, Timestamp{t}, codec::encode(frame_at(t))};
        }
        next_tick_ += tick_period_;
        return SensorEvent{log::Channel::tick, Timestamp{t}, {}};
    }

private:
    ImuSample<double> imu_at(std::int64_t t) {
        constexpr double g = 9.80665;
        ImuSample<double> s{};
        s.t = Timestamp{t};
        for (double& w : s.gyro) {
            w = rng_.symmetric(gyro_amp_);
        }
        s.accel = {rng_.symmetric(accel_amp_), rng_.symmetric(accel_amp_), g + rng_.symmetric(accel_amp_)};
        s.temperature = 25.0;
        return s;
    }

    CameraFrameRecord frame_at(std::int64_t t) {
        const auto& cam = vehicle_.camera;
        CameraFrameRecord f{};
        f.header.t_mid = Timestamp{t};
        f.header.exposure_ns = 2'000'000;
        f.header.row_readout_ns = cam.row_readout_ns;
        f.header.width = cam.width;
        f.header.height = cam.height;
        f.header.format = cam.format;
        f.header.stride = cam.width * bytes_per_pixel(cam.format);
        f.bytes.resize(image_bytes(f.header));
        // A moving diagonal gradient: cheap, deterministic, and different per frame.
        for (std::uint32_t y = 0; y < f.header.height; ++y) {
            for (std::uint32_t x = 0; x < f.header.stride; ++x) {
                f.bytes[std::size_t{y} * f.header.stride + x] = static_cast<std::byte>((x + y + frame_index_) & 0xFFu);
            }
        }
        ++frame_index_;
        return f;
    }

    config::VehicleConfig vehicle_;
    SplitMix64 rng_;
    std::int64_t imu_period_;
    std::int64_t frame_period_;
    std::int64_t tick_period_;
    std::int64_t end_;
    std::int64_t next_imu_{0};
    std::int64_t next_frame_{0};
    std::int64_t next_tick_{0};
    std::uint32_t frame_index_{0};
    double gyro_amp_{0};
    double accel_amp_{0};
};

}  // namespace branes::racing::harness
