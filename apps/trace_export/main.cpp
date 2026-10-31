// trace_export: write a SITL log as a reflex-trace/1 replay, for the docs site.
//
//   trace_export --log <run.brlog> --scenario <scenario.yaml> --out <replay.json>
//                [--rate <Hz>] [--commit <hash>]
//
// The replay plays in reflex's trace player (@branes-ai/trace-player), as on
// docs/guides/mission-replay.mdx. --scenario is the file the log was recorded
// from, as a repo-relative path: it must hash to the log's scenario_hash, and
// it gives the trace its scenario, seed, and physics step. The trace samples
// the truth stream at --rate (default 20 Hz), with takeoff and every gate
// passage as events (sim/replay_trace.hpp). --commit names the tree that
// recorded the run (tools/gen_replay.sh passes `git describe`).

#include <branes/racing/anatomy.hpp>
#include <branes/racing/anatomy_mobility.hpp>
#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/log/channels.hpp>
#include <branes/racing/sim/replay_trace.hpp>

#include <charconv>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

namespace racing = branes::racing;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "trace_export: %s\n", message.c_str());
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = racing::harness::Args::parse(argc, argv, {});
    if (!args) {
        return fail(args.error());
    }
    if (auto bad = args->only({"log", "scenario", "out", "rate", "commit"})) {
        return fail(*bad);
    }
    const auto in_path = args->value("log");
    const auto scenario_path = args->value("scenario");
    const auto out_path = args->value("out");
    if (!in_path || !scenario_path || !out_path) {
        return fail("usage: --log <run.brlog> --scenario <scenario.yaml> --out <replay.json> [--rate <Hz>] "
                    "[--commit <hash>]");
    }
    racing::sim::ReplayOptions options;
    if (const auto r = args->value("rate")) {
        double v = 0;
        const auto [end, ec] = std::from_chars(r->data(), r->data() + r->size(), v);
        if (ec != std::errc{} || end != r->data() + r->size() || !std::isfinite(v) || !(v > 0)) {
            return fail("--rate must be a positive number");
        }
        options.rate_hz = v;
    }

    auto scenario = racing::config::load_scenario(*scenario_path);
    if (!scenario) {
        return fail(scenario.error().to_string());
    }
    const auto scenario_hash = racing::sha256_file_hex(*scenario_path);
    if (!scenario_hash) {
        return fail("cannot read " + *scenario_path);
    }
    auto reader = racing::log::LogReader::open(*in_path, racing::log::kRacingChannels);
    if (!reader) {
        return fail(reader.error().path + ": " + reader.error().message);
    }
    const racing::log::LogHeader& h = reader->header();
    if (!h.reference) {
        return fail(*in_path + " is a what-if replay (its inputs were overridden): export reference runs only");
    }
    if (h.scenario_hash != *scenario_hash) {
        return fail(*in_path + " was not recorded from " + *scenario_path + " (scenario_hash differs)");
    }

    racing::sim::ReplayProvenance prov;
    prov.commit = args->value("commit").value_or("unknown");
    prov.baseline = h.baseline_name;
    prov.scenario = *scenario_path;
    prov.seed = scenario->seed;
    prov.dt_sim = 1 / scenario->sim.physics_rate_hz;
    auto replay = racing::sim::replay_trace(*reader, prov, options);
    if (!replay) {
        return fail(*in_path + ": " + replay.error());
    }

    std::ofstream out(*out_path, std::ios::binary | std::ios::trunc);
    out << replay->trace.to_json(replay->arm_length);
    if (!out.flush()) {
        return fail("cannot write " + *out_path);
    }
    std::printf("trace_export: %zu samples, %zu events -> %s\n",
                replay->trace.size(),
                replay->trace.events().size(),
                out_path->c_str());
    return 0;
}
