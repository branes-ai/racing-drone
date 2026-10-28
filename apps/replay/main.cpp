// replay_<baseline>: run a baseline over a recorded log.
//
//   replay_<baseline> --log <in.brlog> --out <out.brlog>
//                     [--course <course.yaml>] [--vehicle <vehicle.yaml>] [--verify]
//
// The course and vehicle come from the log header (the files are embedded), so
// a log replays on its own. --course / --vehicle override them for a what-if
// replay; the output is then marked non-reference (ADR 0001).
//
// The output log holds the input events and this baseline's outputs. --verify
// also checks that the outputs equal the ones recorded in the input log, record
// for record; it requires the log to have been recorded by this same baseline.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/log/channels.hpp>

#include "racing_baseline.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace racing = branes::racing;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "replay_%s: %s\n", std::string(racing::baseline::name).c_str(), message.c_str());
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = racing::harness::Args::parse(argc, argv, {"verify"});
    if (!args) {
        return fail(args.error());
    }
    if (auto bad = args->only({"log", "out", "course", "vehicle", "verify"})) {
        return fail(*bad);
    }
    const auto in_path = args->value("log");
    const auto out_path = args->value("out");
    if (!in_path || !out_path) {
        return fail("usage: --log <in.brlog> --out <out.brlog> [--course <yaml>] [--vehicle <yaml>] [--verify]");
    }
    const bool verify = args->has("verify");

    auto baseline = racing::harness::check_baseline(
        RACING_BASELINES_DIR, racing::baseline::name, racing::baseline::composition_hash);
    if (!baseline) {
        return fail(baseline.error());
    }

    auto reader = racing::log::LogReader::open(*in_path, racing::log::kRacingChannels);
    if (!reader) {
        return fail(reader.error().path + ": " + reader.error().message);
    }
    const racing::log::LogHeader& in_header = reader->header();
    if (verify && (in_header.baseline_name != racing::baseline::name ||
                   in_header.composition_hash != racing::baseline::composition_hash)) {
        return fail("--verify needs a log recorded by this baseline (log: " + in_header.baseline_name + ")");
    }

    racing::log::LogHeader header = in_header;
    header.baseline_name = std::string(racing::baseline::name);
    header.composition_hash = std::string(racing::baseline::composition_hash);
    if (const auto c = args->value("course")) {
        const auto f = racing::harness::read_with_hash(*c);
        if (!f) {
            return fail("cannot read " + *c);
        }
        header.world_yaml = f->text;
        header.world_hash = f->hash;
        header.reference = false;
    }
    if (const auto v = args->value("vehicle")) {
        const auto f = racing::harness::read_with_hash(*v);
        if (!f) {
            return fail("cannot read " + *v);
        }
        header.vehicle_yaml = f->text;
        header.vehicle_hash = f->hash;
        header.reference = false;
    }
    if (verify && !header.reference) {
        return fail("--verify cannot be combined with --course / --vehicle overrides");
    }

    auto course = racing::config::parse_course(header.world_yaml, "course (from " + *in_path + ")");
    if (!course) {
        return fail(course.error().to_string());
    }
    auto vehicle = racing::config::parse_vehicle(header.vehicle_yaml, "vehicle (from " + *in_path + ")");
    if (!vehicle) {
        return fail(vehicle.error().to_string());
    }

    auto built = racing::harness::make_stack<racing::baseline::Composition<double>>(course->map, *vehicle, *baseline);
    if (!built) {
        return fail(built.error());
    }
    auto& stack = *built;

    auto writer = racing::log::LogWriter::open(*out_path, header);
    if (!writer) {
        return fail(writer.error().path + ": " + writer.error().message);
    }

    const auto kept = racing::harness::replay_log(*reader, stack, racing::log::kRacingChannels, *writer, verify);
    if (!kept) {
        return fail(kept.error());
    }
    const auto records = writer->record_count();
    if (auto closed = writer->close(); !closed) {
        return fail(closed.error().path + ": " + closed.error().message);
    }

    if (verify) {
        const auto verified = racing::harness::verify_outputs(*kept);
        if (!verified) {
            return fail(verified.error());
        }
        std::printf("replay_%s: verified %zu outputs\n", std::string(racing::baseline::name).c_str(), *verified);
    }
    std::printf("replay_%s: %llu records -> %s%s\n",
                std::string(racing::baseline::name).c_str(),
                static_cast<unsigned long long>(records),
                out_path->c_str(),
                header.reference ? "" : " (non-reference)");
    return 0;
}
