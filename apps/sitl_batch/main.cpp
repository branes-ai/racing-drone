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
// is complete or has failed (sim/batch.hpp, shared with precision_sweep). The
// runs are independent, so --jobs changes the wall time, never the scores.
//
// It prints a summary (success rate, lap time and gate error percentiles,
// max speed, failures), optionally one JSON line per run, and exits nonzero
// if the success fraction is below --min-success (default 1).

#include <branes/racing/anatomy_mobility.hpp>
#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/sim/batch.hpp>
#include <branes/racing/sim/score.hpp>

#include "racing_baseline.hpp"

#include <algorithm>
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
    const auto seeds = racing::harness::parse_number<std::uint64_t>(*seeds_arg);
    if (!seeds || *seeds == 0 || *seeds > 1'000'000) {
        return fail("--seeds must be an integer in [1, 1000000]");
    }
    double min_success = 1.0;
    if (const auto m = args->value("min-success")) {
        const auto v = racing::harness::parse_number<double>(*m);
        if (!v || !(*v >= 0 && *v <= 1)) {
            return fail("--min-success must be a fraction in [0, 1]");
        }
        min_success = *v;
    }
    unsigned jobs = std::max(1u, std::thread::hardware_concurrency());
    if (const auto j = args->value("jobs")) {
        const auto v = racing::harness::parse_number<unsigned>(*j);
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
    const double control_hz =
        racing::config::numeric_parameter(*baseline, "control_rate_hz").value_or(racing::config::kDefaultControlRateHz);
    auto scenario = racing::config::load_scenario(*scenario_path);
    if (!scenario) {
        return fail(scenario.error().to_string());
    }
    std::uint64_t first = scenario->seed;
    if (const auto f = args->value("first-seed")) {
        const auto v = racing::harness::parse_number<std::uint64_t>(*f);
        if (!v) {
            return fail("--first-seed must be a non-negative integer");
        }
        first = *v;
    }
    if (first > UINT64_MAX - (*seeds - 1)) {
        return fail("--first-seed + --seeds overflows the seed range");
    }
    auto course = racing::config::load_course(scenario->world_path);
    if (!course) {
        return fail(course.error().to_string());
    }
    auto vehicle = racing::config::load_vehicle(scenario->vehicle_path);
    if (!vehicle) {
        return fail(vehicle.error().to_string());
    }

    std::vector<racing::sim::Run> runs(static_cast<std::size_t>(*seeds));
    racing::harness::for_each_index(*seeds, jobs, [&](std::uint64_t i) {
        runs[i] = racing::sim::run_one<racing::baseline::Composition<double>>(
            *baseline, *vehicle, course->map, *scenario, control_hz, first + i);
    });

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
    for (const racing::sim::Run& r : runs) {
        if (!r.error.empty()) {
            return fail("seed " + std::to_string(r.seed) + ": " + r.error);
        }
        const racing::sim::Score& s = r.score;
        if (jsonl.is_open()) {
            jsonl << racing::sim::to_json(r) << "\n";
        }
        successes += s.success ? 1 : 0;
        laps.insert(laps.end(), s.lap_times_s.begin(), s.lap_times_s.end());
        for (const auto& p : s.passages) {
            errors.push_back(p.true_error_m);
        }
        speeds.push_back(s.max_speed_m_s);
        if (!s.success) {
            ++failures[racing::sim::failure_reason(s.failure)];
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
                racing::sim::percentile(laps, 0.5),
                racing::sim::percentile(laps, 0.95),
                laps.size());
    std::printf("  gate error    p50 %.3f m, p99 %.3f m (true, %zu passages)\n",
                racing::sim::percentile(errors, 0.5),
                racing::sim::percentile(errors, 0.99),
                errors.size());
    const double speed_p50 = racing::sim::percentile(speeds, 0.5);  // sorts speeds
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
