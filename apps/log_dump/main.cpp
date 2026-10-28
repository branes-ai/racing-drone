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
// The records are dumped by the mission-neutral log::dump_records()
// (log/dump.hpp), which documents the CSV formats; this file adds racing's
// own channel, oracle_gates (one row per observed gate), and the header files.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/log/channels.hpp>
#include <branes/racing/msg/gate_codec.hpp>

#include <array>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <utility>

namespace racing = branes::racing;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "log_dump: %s\n", message.c_str());
    return 1;
}

// ── racing's channel ────────────────────────────────────────────────

/// oracle_gates.csv: one row per observed gate; a frame with none contributes no row.
std::optional<std::string> oracle_rows(const racing::log::LogRecord& r, std::uint64_t& count) {
    using namespace racing::log::csv;
    namespace codec = racing::codec;
    std::string row = start(r.t);
    count = 1;
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

std::string oracle_columns() {
    const std::string t = "t_ns,t_s";
    std::string s = t + ",count,index,gate_id";
    for (int i = 0; i < 4; ++i) {
        s += ",u" + std::to_string(i) + ",v" + std::to_string(i);
    }
    return s + ",quality";
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
        if (!racing::log::write_text(out_dir / name, text)) {
            return fail((out_dir / name).string() + ": cannot write the file");
        }
    }

    const std::array<racing::log::ChannelFormat, 1> formats = {
        racing::log::ChannelFormat{racing::log::oracle_gates, "oracle_gates.csv", oracle_columns(), &oracle_rows}};
    const auto dumped = racing::log::dump_records(*reader, out_dir, formats);
    if (!dumped) {
        return fail(dumped.error());
    }
    for (const racing::log::DumpedFile& f : dumped->files) {
        std::printf("log_dump: %-12s %llu records, %llu rows -> %s\n",
                    f.file_name.c_str(),
                    static_cast<unsigned long long>(f.records),
                    static_cast<unsigned long long>(f.rows),
                    f.path.string().c_str());
    }
    std::printf(
        "log_dump: %llu records -> %s\n", static_cast<unsigned long long>(dumped->records), out_dir.string().c_str());
    return 0;
}
