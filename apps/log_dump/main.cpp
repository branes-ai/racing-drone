// log_dump: write a log's contents as CSV and YAML files, for plotting.
//
//   log_dump --log <run.brlog> --out-dir <dir>
//
// Writes into <dir> (created if needed, files overwritten):
//   header.yaml          provenance: baseline, hashes, reference flag
//   course.yaml          the course file embedded in the log, verbatim
//   vehicle.yaml         the vehicle file embedded in the log, verbatim
//   sim_config.yaml      the run's drawn randomization (SITL logs)
//   <channel>.csv        one per channel present: imu, camera, tick, truth,
//                        oracle_gates, state, reference, command
//
// Every CSV row starts with t_ns (the record time, exact) and t_s (the same
// in seconds). Doubles are printed in shortest round-trip form, so a value
// read back is the logged value. Attitudes are the logged quaternion (body →
// course) plus roll, pitch, yaw [rad] in the Z-Y-X convention; state.csv
// carries the covariance diagonal only, and camera.csv the frame header
// without pixels. oracle_gates.csv has one row per observed gate, and
// reference.csv one row per reference sample.

#include <branes/racing/harness/harness.hpp>
#include <branes/racing/log/channels.hpp>
#include <branes/racing/log/log.hpp>
#include <branes/racing/msg/codec.hpp>
#include <branes/racing/msg/gate_codec.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace racing = branes::racing;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "log_dump: %s\n", message.c_str());
    return 1;
}

// ── formatting ──────────────────────────────────────────────────────

void put(std::string& row, double v) {
    char buf[32];
    const auto [end, ec] = std::to_chars(buf, buf + sizeof buf, v);
    row.push_back(',');
    row.append(buf, ec == std::errc{} ? end : buf);
}

void put(std::string& row, std::int64_t v) {
    row.push_back(',');
    row += std::to_string(v);
}

void put(std::string& row, std::uint64_t v) {
    row.push_back(',');
    row += std::to_string(v);
}

void put(std::string& row, const racing::Vec3<double>& v) {
    for (const double c : v) {
        put(row, c);
    }
}

/// The quaternion, then roll, pitch, yaw (Z-Y-X) of the body → course rotation.
void put(std::string& row, const racing::Quat<double>& q) {
    put(row, q.w);
    put(row, q.x);
    put(row, q.y);
    put(row, q.z);
    put(row, std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)));
    put(row, std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0)));
    put(row, std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)));
}

/// t_ns, t_s: the record time.
std::string start(racing::Timestamp t) {
    std::string row = std::to_string(t.ns);
    put(row, static_cast<double>(t.ns) / 1e9);
    return row;
}

std::string vec3(std::string_view name) {
    std::string s;
    for (const char axis : {'x', 'y', 'z'}) {
        s += ',';
        s += name;
        s += '_';
        s += axis;
    }
    return s;
}

std::string quat(std::string_view name) {
    std::string s;
    for (const char* part : {"_w", "_x", "_y", "_z"}) {
        s += ',';
        s += name;
        s += part;
    }
    return s + ",roll,pitch,yaw";
}

// ── output files ────────────────────────────────────────────────────

struct Sink {
    std::filesystem::path path;
    std::ofstream out;
    std::uint64_t records{0};
    std::uint64_t rows{0};
};

const char* file_name(racing::log::Channel c) {
    using racing::log::Channel;
    if (c == racing::log::oracle_gates) {
        return "oracle_gates.csv";
    }
    switch (c) {
    case Channel::imu:
        return "imu.csv";
    case Channel::camera:
        return "camera.csv";
    case Channel::tick:
        return "tick.csv";
    case Channel::truth:
        return "truth.csv";
    case Channel::sim_config:
        return "sim_config.yaml";
    case Channel::state:
        return "state.csv";
    case Channel::reference:
        return "reference.csv";
    case Channel::command:
        return "command.csv";
    }
    return "unknown.csv";
}

std::string columns(racing::log::Channel c) {
    using racing::log::Channel;
    const std::string t = "t_ns,t_s";
    if (c == racing::log::oracle_gates) {
        std::string s = t + ",count,index,gate_id";
        for (int i = 0; i < 4; ++i) {
            s += ",u" + std::to_string(i) + ",v" + std::to_string(i);
        }
        return s + ",quality";
    }
    switch (c) {
    case Channel::imu:
        return t + ",stamp_ns" + vec3("gyro") + vec3("accel") + ",saturated,temperature";
    case Channel::camera:
        return t + ",t_mid_ns,exposure_ns,row_readout_ns,width,height,stride,format,corrupted,bytes";
    case Channel::tick:
        return t;
    case Channel::truth:
        return t + ",stamp_ns" + vec3("p") + vec3("v") + quat("q") + vec3("w") +
               ",rotor_0,rotor_1,rotor_2,rotor_3,on_ground";
    case Channel::state: {
        std::string s =
            t + ",t_valid_ns" + vec3("p") + vec3("v") + quat("q") + vec3("w") + vec3("accel_bias") + vec3("gyro_bias");
        for (std::size_t i = 0; i < racing::kErrorStateDim; ++i) {
            s += ",var_" + std::to_string(i);
        }
        return s;
    }
    case Channel::reference:
        return t + ",count,index,sample_t_ns" + vec3("p") + vec3("v") + vec3("a") + ",yaw,progress";
    case Channel::command:
        return t + ",stamp_ns,kind,thrust" + vec3("w") + ",motor_0,motor_1,motor_2,motor_3";
    case Channel::sim_config:
        break;
    }
    return {};
}

/// The rows one record contributes to its channel's file, or nullopt when the
/// payload does not decode.
std::optional<std::string> rows_of(const racing::log::LogRecord& r, std::uint64_t& count) {
    using racing::log::Channel;
    namespace codec = racing::codec;
    std::string row = start(r.t);
    count = 1;
    if (r.channel == racing::log::oracle_gates) {
        const auto x = codec::decode_gates(r.payload);
        if (!x) {
            return std::nullopt;
        }
        // One row per observed gate; a frame with none contributes no row.
        const std::string time = row;
        row.clear();
        for (std::uint32_t i = 0; i < x->count; ++i) {
            const racing::GateObservation<double>& o = x->gates[i];
            row += time;
            put(row, std::uint64_t{x->count});
            put(row, std::uint64_t{i});
            put(row, std::int64_t{o.gate_id});
            for (const auto& c : o.corners) {
                put(row, c[0]);
                put(row, c[1]);
            }
            put(row, o.quality);
            row += '\n';
        }
        count = x->count;
        return row;
    }
    switch (r.channel) {
    case Channel::imu: {
        const auto x = codec::decode_imu(r.payload);
        if (!x) {
            return std::nullopt;
        }
        put(row, x->t.ns);
        put(row, x->gyro);
        put(row, x->accel);
        put(row, std::uint64_t{x->saturated});
        put(row, x->temperature);
        break;
    }
    case Channel::camera: {
        const auto x = codec::decode_frame(r.payload);
        if (!x) {
            return std::nullopt;
        }
        const racing::FrameHeader& h = x->header;
        put(row, h.t_mid.ns);
        put(row, h.exposure_ns);
        put(row, h.row_readout_ns);
        put(row, std::uint64_t{h.width});
        put(row, std::uint64_t{h.height});
        put(row, std::uint64_t{h.stride});
        row += h.format == racing::PixelFormat::rgb8 ? ",rgb8" : ",mono8";
        put(row, std::uint64_t{h.corrupted});
        put(row, std::uint64_t{x->bytes.size()});
        break;
    }
    case Channel::tick:
        break;
    case Channel::truth: {
        const auto x = codec::decode_truth(r.payload);
        if (!x) {
            return std::nullopt;
        }
        put(row, x->t.ns);
        put(row, x->position);
        put(row, x->velocity);
        put(row, x->attitude);
        put(row, x->body_rates);
        for (const double w : x->rotor_speeds) {
            put(row, w);
        }
        put(row, std::uint64_t{x->on_ground});
        break;
    }
    case Channel::state: {
        const auto x = codec::decode_state(r.payload);
        if (!x) {
            return std::nullopt;
        }
        put(row, x->t_valid.ns);
        put(row, x->position);
        put(row, x->velocity);
        put(row, x->attitude);
        put(row, x->body_rates);
        put(row, x->accel_bias);
        put(row, x->gyro_bias);
        for (std::size_t i = 0; i < racing::kErrorStateDim; ++i) {
            put(row, x->covariance[i * racing::kErrorStateDim + i]);
        }
        break;
    }
    case Channel::reference: {
        const auto x = codec::decode_reference(r.payload);
        if (!x) {
            return std::nullopt;
        }
        // One row per sample; an empty reference contributes no row.
        const std::string time = row;
        row.clear();
        for (std::uint32_t i = 0; i < x->count; ++i) {
            const racing::ReferenceSample<double>& s = x->samples[i];
            row += time;
            put(row, std::uint64_t{x->count});
            put(row, std::uint64_t{i});
            put(row, s.t.ns);
            put(row, s.position);
            put(row, s.velocity);
            put(row, s.acceleration);
            put(row, s.yaw);
            put(row, s.progress);
            row += '\n';
        }
        count = x->count;
        return row;
    }
    case Channel::command: {
        const auto x = codec::decode_command(r.payload);
        if (!x) {
            return std::nullopt;
        }
        put(row, x->t.ns);
        row += x->kind == racing::CommandKind::motors ? ",motors" : ",thrust_rates";
        put(row, x->thrust);
        put(row, x->body_rates);
        for (const double m : x->motors) {
            put(row, m);
        }
        break;
    }
    case Channel::sim_config:
        // YAML text: one document per record.
        return std::string("---\n") + std::string(reinterpret_cast<const char*>(r.payload.data()), r.payload.size()) +
               '\n';
    }
    return row + '\n';
}

bool write_text(const std::filesystem::path& path, std::string_view text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    out.close();
    return !out.fail();
}

}  // namespace

int main(int argc, char** argv) {
    auto args = racing::harness::Args::parse(argc, argv, {});
    if (!args) {
        return fail(args.error());
    }
    if (auto bad = args->only({"log", "out-dir"})) {
        return fail(*bad);
    }
    const auto in_path = args->value("log");
    const auto out_dir_arg = args->value("out-dir");
    if (!in_path || !out_dir_arg) {
        return fail("usage: --log <run.brlog> --out-dir <dir>");
    }
    const std::filesystem::path out_dir(*out_dir_arg);
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
        return fail(out_dir.string() + ": " + ec.message());
    }

    auto reader = racing::log::LogReader::open(*in_path, racing::log::kRacingChannels);
    if (!reader) {
        return fail(reader.error().path + ": " + reader.error().message);
    }
    const racing::log::LogHeader& h = reader->header();
    const std::string provenance = "baseline: " + h.baseline_name + "\ncomposition_hash: " + h.composition_hash +
                                   "\ncourse_hash: " + h.world_hash + "\nvehicle_hash: " + h.vehicle_hash +
                                   "\nscenario_hash: " + h.scenario_hash +
                                   "\nreference: " + (h.reference ? "true" : "false") + "\n";
    for (const auto& [name, text] : {std::pair<const char*, const std::string&>{"header.yaml", provenance},
                                     {"course.yaml", h.world_yaml},
                                     {"vehicle.yaml", h.vehicle_yaml}}) {
        if (!write_text(out_dir / name, text)) {
            return fail((out_dir / name).string() + ": cannot write the file");
        }
    }

    std::map<racing::log::Channel, Sink> sinks;
    std::uint64_t records = 0;
    for (;;) {
        auto rec = reader->next();
        if (!rec) {
            return fail(rec.error().path + ": " + rec.error().message);
        }
        if (!rec->has_value()) {
            break;
        }
        const racing::log::LogRecord& r = **rec;
        std::uint64_t n = 0;
        const auto text = rows_of(r, n);
        if (!text) {
            return fail("undecodable " + std::string(file_name(r.channel)) + " record at t=" + std::to_string(r.t.ns));
        }
        auto [it, inserted] = sinks.try_emplace(r.channel);
        Sink& sink = it->second;
        if (inserted) {
            sink.path = out_dir / file_name(r.channel);
            sink.out.open(sink.path, std::ios::binary | std::ios::trunc);
            if (const std::string header = columns(r.channel); !header.empty()) {
                sink.out << header << '\n';
            }
        }
        sink.out << *text;
        ++sink.records;
        sink.rows += n;
        ++records;
    }

    for (auto& [channel, sink] : sinks) {
        sink.out.close();
        if (sink.out.fail()) {
            return fail(sink.path.string() + ": cannot write the file");
        }
        std::printf("log_dump: %-12s %llu records, %llu rows -> %s\n",
                    file_name(channel),
                    static_cast<unsigned long long>(sink.records),
                    static_cast<unsigned long long>(sink.rows),
                    sink.path.string().c_str());
    }
    std::printf("log_dump: %llu records -> %s\n", static_cast<unsigned long long>(records), out_dir.string().c_str());
    return 0;
}
