// sitl_batch_<baseline>: run a baseline over many seeds of a scenario and score them.
//
//   sitl_batch_<baseline> --scenario <scenario.yaml> --seeds <n>
//                         [--first-seed <s>] [--min-success <fraction>]
//                         [--jobs <j>] [--jsonl <scores.jsonl>]
//
// The phase gates' runner (test plan §6). Run i uses seed first-seed + i
// (default: the scenario's seed), so any run reproduces on its own with
// sitl_<baseline> and that seed. Each run is the SITL closed loop of
// sitl_<baseline>, through the same Runner, but writes no log; it is scored
// as it goes (sim::Scorer, with the scenario's speed limit) and stops once it
// is complete or has failed. The runs are independent, so --jobs changes the
// wall time, never the scores.
//
// It prints a summary (success rate, lap time and gate error percentiles,
// max speed, failures), optionally one JSON line per run, and exits nonzero
// if the success fraction is below --min-success (default 1).

#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/log/log.hpp>
#include <branes/racing/msg/codec.hpp>
#include <branes/racing/sim/score.hpp>
#include <branes/racing/sim/simulator.hpp>

#include "racing_baseline.hpp"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace racing = branes::racing;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "sitl_batch_%s: %s\n", std::string(racing::baseline::name).c_str(), message.c_str());
    return 1;
}

template <typename N>
std::optional<N> number(const std::string& s) {
    N v{};
    const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (ec != std::errc{} || end != s.data() + s.size()) {
        return std::nullopt;
    }
    return v;
}

struct Run {
    std::uint64_t seed{0};
    racing::sim::Score score;
    std::string error;  ///< set if the run could not be set up or fed
};

/// One closed-loop run, scored. Mirrors apps/sitl/main.cpp without the log.
Run run_one(const racing::config::BaselineConfig& baseline,
            const racing::config::VehicleConfig& vehicle,
            const racing::CourseMap<double>& map,
            racing::config::ScenarioConfig scenario,
            double control_hz,
            std::uint64_t seed) {
    Run out;
    out.seed = seed;
    scenario.seed = seed;
    auto sim = racing::sim::Simulator::create(vehicle, map, scenario, control_hz);
    if (!sim) {
        out.error = sim.error();
        return out;
    }
    auto stack = racing::harness::make_stack<racing::baseline::Composition<double>>(map, vehicle, baseline);
    if (!stack) {
        out.error = stack.error();
        return out;
    }
    racing::harness::Runner runner(*stack);
    racing::sim::Scorer scorer(map, scenario.speed_limit_m_s);
    while (auto ev = sim->next()) {
        if (ev->channel == racing::log::Channel::truth) {
            const auto x = racing::codec::decode_truth(ev->payload);
            if (!x) {
                out.error = "undecodable truth record";
                return out;
            }
            scorer.on_truth(*x);
        }
        auto outputs = runner.feed(racing::log::LogRecord{ev->channel, ev->t, std::move(ev->payload)});
        if (!outputs) {
            out.error = outputs.error();
            return out;
        }
        for (const auto& o : *outputs) {
            if (o.channel == racing::log::Channel::command) {
                const auto u = racing::codec::decode_command(o.payload);
                if (!u) {
                    out.error = "the stack produced a command that does not decode";
                    return out;
                }
                sim->command(*u);
            } else if (o.channel == racing::log::Channel::state) {
                if (const auto x = racing::codec::decode_state(o.payload)) {
                    scorer.on_estimate(*x);
                }
            }
        }
        if (scorer.complete() || scorer.failed()) {
            break;
        }
    }
    out.score = scorer.score();
    return out;
}

/// Nearest-rank percentile of `v` (sorted in place); NaN if empty.
double percentile(std::vector<double>& v, double p) {
    if (v.empty()) {
        return std::nan("");
    }
    std::sort(v.begin(), v.end());
    const auto rank = static_cast<std::size_t>(std::ceil(p * static_cast<double>(v.size())));
    return v[rank == 0 ? 0 : rank - 1];
}

/// A failure message without its time or gate count, for grouping.
std::string reason(const std::string& failure) {
    for (const char* cut : {" at t=", ":"}) {
        if (const auto i = failure.find(cut); i != std::string::npos) {
            return failure.substr(0, i);
        }
    }
    return failure;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = racing::harness::Args::parse(argc, argv, {});
    if (!args) {
        return fail(args.error());
    }
    if (auto bad = args->only({"scenario", "seeds", "first-seed", "min-success", "jobs", "jsonl"})) {
        return fail(*bad);
    }
    const auto scenario_path = args->value("scenario");
    const auto seeds_arg = args->value("seeds");
    if (!scenario_path || !seeds_arg) {
        return fail("usage: --scenario <yaml> --seeds <n> [--first-seed <s>] [--min-success <f>] [--jobs <j>] "
                    "[--jsonl <out>]");
    }
    const auto seeds = number<std::uint64_t>(*seeds_arg);
    if (!seeds || *seeds == 0 || *seeds > 1'000'000) {
        return fail("--seeds must be an integer in [1, 1000000]");
    }
    double min_success = 1.0;
    if (const auto m = args->value("min-success")) {
        const auto v = number<double>(*m);
        if (!v || !(*v >= 0 && *v <= 1)) {
            return fail("--min-success must be a fraction in [0, 1]");
        }
        min_success = *v;
    }
    unsigned jobs = std::max(1u, std::thread::hardware_concurrency());
    if (const auto j = args->value("jobs")) {
        const auto v = number<unsigned>(*j);
        if (!v || *v == 0 || *v > 1024) {
            return fail("--jobs must be an integer in [1, 1024]");
        }
        jobs = *v;
    }

    auto baseline = racing::harness::check_baseline(
        RACING_BASELINES_DIR, racing::baseline::name, racing::baseline::composition_hash);
    if (!baseline) {
        return fail(baseline.error());
    }
    const double control_hz = racing::config::numeric_parameter(*baseline, "control_rate_hz").value_or(100.0);
    auto scenario = racing::config::load_scenario(*scenario_path);
    if (!scenario) {
        return fail(scenario.error().to_string());
    }
    std::uint64_t first = scenario->seed;
    if (const auto f = args->value("first-seed")) {
        const auto v = number<std::uint64_t>(*f);
        if (!v) {
            return fail("--first-seed must be a non-negative integer");
        }
        first = *v;
    }
    if (first > UINT64_MAX - (*seeds - 1)) {
        return fail("--first-seed + --seeds overflows the seed range");
    }
    auto course = racing::config::load_course(scenario->course_path);
    if (!course) {
        return fail(course.error().to_string());
    }
    auto vehicle = racing::config::load_vehicle(scenario->vehicle_path);
    if (!vehicle) {
        return fail(vehicle.error().to_string());
    }

    std::vector<Run> runs(static_cast<std::size_t>(*seeds));
    std::atomic<std::uint64_t> next{0};
    const auto worker = [&] {
        for (std::uint64_t i = next++; i < *seeds; i = next++) {
            runs[i] = run_one(*baseline, *vehicle, course->map, *scenario, control_hz, first + i);
        }
    };
    {
        std::vector<std::jthread> pool;
        for (unsigned k = 0; k < std::min<std::uint64_t>(jobs, *seeds); ++k) {
            pool.emplace_back(worker);
        }
    }

    std::ofstream jsonl;
    if (const auto path = args->value("jsonl")) {
        jsonl.open(*path, std::ios::binary);
        if (!jsonl) {
            return fail("cannot write " + *path);
        }
    }
    std::uint64_t successes = 0;
    std::vector<double> laps;
    std::vector<double> errors;
    std::vector<double> speeds;
    std::map<std::string, std::uint64_t> failures;
    for (const Run& r : runs) {
        if (!r.error.empty()) {
            return fail("seed " + std::to_string(r.seed) + ": " + r.error);
        }
        const racing::sim::Score& s = r.score;
        if (jsonl.is_open()) {
            jsonl << "{\"seed\":" << r.seed << ",\"score\":" << racing::sim::to_json(s) << "}\n";
        }
        successes += s.success ? 1 : 0;
        laps.insert(laps.end(), s.lap_times_s.begin(), s.lap_times_s.end());
        for (const auto& p : s.passages) {
            errors.push_back(p.true_error_m);
        }
        speeds.push_back(s.max_speed_m_s);
        if (!s.success) {
            ++failures[reason(s.failure)];
            std::printf("  seed %llu: %s\n", static_cast<unsigned long long>(r.seed), s.failure.c_str());
        }
    }
    const double rate = static_cast<double>(successes) / static_cast<double>(*seeds);
    std::printf("sitl_batch_%s: %s, seeds %llu..%llu\n",
                std::string(racing::baseline::name).c_str(),
                scenario->name.c_str(),
                static_cast<unsigned long long>(first),
                static_cast<unsigned long long>(first + *seeds - 1));
    std::printf("  success       %llu / %llu = %.2f %% (need >= %.2f %%)\n",
                static_cast<unsigned long long>(successes),
                static_cast<unsigned long long>(*seeds),
                100.0 * rate,
                100.0 * min_success);
    std::printf("  lap time      p50 %.3f s, p95 %.3f s (%zu laps)\n",
                percentile(laps, 0.5),
                percentile(laps, 0.95),
                laps.size());
    std::printf("  gate error    p50 %.3f m, p99 %.3f m (true, %zu passages)\n",
                percentile(errors, 0.5),
                percentile(errors, 0.99),
                errors.size());
    const double speed_p50 = percentile(speeds, 0.5);  // sorts speeds
    std::printf("  max speed     p50 %.3f m/s, max %.3f m/s", speed_p50, speeds.back());
    if (scenario->speed_limit_m_s) {
        std::printf(" (limit %.3f m/s)", *scenario->speed_limit_m_s);
    }
    std::printf("\n");
    for (const auto& [why, n] : failures) {
        std::printf("  failures      %llu x %s\n", static_cast<unsigned long long>(n), why.c_str());
    }
    return rate >= min_success ? 0 : 1;
}
