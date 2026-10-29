// precision_sweep_<baseline>: the baseline's stack in each scalar type, over many seeds.
//
//   precision_sweep_<baseline> --scenario <scenario.yaml> --seeds <n> --out-dir <dir>
//                              [--first-seed <s>] [--jobs <j>] [--min-success <f>]
//                              [--commit <text>] [--timing]
//
// docs/plan/precision-sweep/. The simulator, simulated FC, scorer, and log stay
// in double; only the stack runs in T, through the Runner's casts. For each
// type in sim::kSweepTypes (sim/sweep_types.hpp), in order, the stack is built once: a failure is that type's
// result (construction_failed, with the reason), and the sweep moves on.
// Otherwise every seed runs as in sitl_batch_<baseline> (sim/batch.hpp), and
// a run that errors is that seed's result.
//
// Files in --out-dir:
//   <type>.jsonl     one line per seed, in sitl_batch's --jsonl format (or
//                    {"seed":…,"run_error":"…"}); empty if the stack didn't build
//   lap_delta.jsonl  per type and seed, each lap's time minus double's, for
//                    seeds where both succeeded
//   summary.jsonl    one line per type: construction, success, failures,
//                    percentiles, and the lap-time difference from double
//   gate_error.jsonl per built type and gate: passages, true gate error p50
//                    and p99, the paired excess over double, and the type's
//                    in-plane resolution at the gate (docs/plan/gate-error-by-gate/)
//   summary.md       the same as tables, with the command line and commit
//
// The files depend only on the arguments (not --jobs, --out-dir, or the wall
// time), so a rerun is byte-identical; --timing adds each type's wall time.
// --min-success (default 0.99, gate P1) only marks the table's P1 column.
// Exits 0 once every type has a result; nonzero on usage or setup errors.

#include <branes/racing/anatomy_mobility.hpp>
#include <branes/racing/anatomy_sweep.hpp>
#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/sim/batch.hpp>
#include <branes/racing/sim/gate_resolution.hpp>
#include <branes/racing/sim/score.hpp>

#include "racing_baseline.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
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
#include <thread>
#include <tuple>
#include <vector>

namespace racing = branes::racing;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "precision_sweep_%s: %s\n", std::string(racing::baseline::name).c_str(), message.c_str());
    return 1;
}

/// A JSON number in shortest round-trip form; null if not finite.
std::string num(double v) {
    if (!std::isfinite(v)) {
        return "null";
    }
    char buf[32];
    const auto [end, ec] = std::to_chars(buf, buf + sizeof buf, v);
    return ec == std::errc{} ? std::string(buf, end) : std::string("null");
}

/// A table cell: `v` with `digits` decimals, or an em dash if not finite.
std::string cell(double v, int digits) {
    if (!std::isfinite(v)) {
        return "—";
    }
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.*f", digits, v);
    return buf;
}

/// What one type gave.
struct TypeResult {
    std::string slug;
    std::string label;
    std::string construction_error;  ///< empty if the stack built
    std::vector<racing::sim::Run> runs;
    std::vector<double> in_plane_ulp;  ///< per course gate, sim::in_plane_ulp<T>
    double wall_s{0};
};

/// One gate's statistics in one type. Passages are those of runs without a
/// run error, as in the summary's gate error; a pair is the same seed, lap and
/// gate passed in this type and in double.
struct GateStats {
    std::int32_t id{0};
    std::uint64_t passages{0};
    double err_p50{0}, err_p99{0};
    std::uint64_t pairs{0};
    double excess_p50{0};  ///< median of (this type's true error − double's) over the pairs [m]
    double ulp{0};         ///< in-plane resolution [m]
};

std::vector<GateStats>
gate_stats(const TypeResult& t, const TypeResult& reference, const racing::CourseMap<double>& map) {
    std::vector<GateStats> out;
    for (std::uint32_t g = 0; g < map.gate_count; ++g) {
        GateStats s;
        s.id = map.gates[g].id;
        s.ulp = t.in_plane_ulp[g];
        std::vector<double> errors, excess;
        for (std::size_t i = 0; i < t.runs.size(); ++i) {
            const racing::sim::Run& r = t.runs[i];
            if (!r.error.empty()) {
                continue;
            }
            const racing::sim::Run* ref =
                i < reference.runs.size() && reference.runs[i].error.empty() ? &reference.runs[i] : nullptr;
            for (const auto& p : r.score.passages) {
                if (p.gate_id != s.id) {
                    continue;
                }
                errors.push_back(p.true_error_m);
                if (ref == nullptr) {
                    continue;
                }
                for (const auto& q : ref->score.passages) {
                    if (q.gate_id == p.gate_id && q.lap == p.lap) {
                        excess.push_back(p.true_error_m - q.true_error_m);
                        break;
                    }
                }
            }
        }
        s.passages = errors.size();
        s.err_p50 = racing::sim::percentile(errors, 0.5);
        s.err_p99 = racing::sim::percentile(errors, 0.99);
        s.pairs = excess.size();
        s.excess_p50 = racing::sim::percentile(excess, 0.5);
        out.push_back(s);
    }
    return out;
}

/// The seed-level and type-level statistics of one TypeResult.
struct Stats {
    std::uint64_t seeds{0};
    std::uint64_t successes{0};
    std::uint64_t run_errors{0};
    std::map<std::string, std::uint64_t> failures;  ///< failure reason (or "run_error: …") → count
    double lap_p50{0}, lap_p95{0}, err_p50{0}, err_p99{0}, speed_p50{0}, speed_max{0};
    std::uint64_t delta_seeds{0};                     ///< seeds where this type and double both succeeded
    double delta_p50{0}, delta_p95{0}, delta_max{0};  ///< |lap time − double's| over their laps
};

/// Each lap's time minus the reference's, if both runs succeeded with the same lap count.
std::optional<std::vector<double>> lap_delta(const racing::sim::Run& r, const racing::sim::Run& ref) {
    if (!r.error.empty() || !ref.error.empty() || !r.score.success || !ref.score.success ||
        r.score.lap_times_s.size() != ref.score.lap_times_s.size()) {
        return std::nullopt;
    }
    std::vector<double> d;
    for (std::size_t i = 0; i < r.score.lap_times_s.size(); ++i) {
        d.push_back(r.score.lap_times_s[i] - ref.score.lap_times_s[i]);
    }
    return d;
}

Stats stats_of(const TypeResult& t, const TypeResult* reference) {
    Stats s;
    s.seeds = t.runs.size();
    std::vector<double> laps, errors, speeds, deltas;
    for (std::size_t i = 0; i < t.runs.size(); ++i) {
        const racing::sim::Run& r = t.runs[i];
        if (!r.error.empty()) {
            ++s.run_errors;
            ++s.failures["run_error: " + racing::sim::failure_reason(r.error)];
            continue;
        }
        s.successes += r.score.success ? 1 : 0;
        if (!r.score.success) {
            ++s.failures[racing::sim::failure_reason(r.score.failure)];
        }
        laps.insert(laps.end(), r.score.lap_times_s.begin(), r.score.lap_times_s.end());
        for (const auto& p : r.score.passages) {
            errors.push_back(p.true_error_m);
        }
        speeds.push_back(r.score.max_speed_m_s);
        if (reference != nullptr && i < reference->runs.size()) {
            if (const auto d = lap_delta(r, reference->runs[i])) {
                ++s.delta_seeds;
                for (const double x : *d) {
                    deltas.push_back(std::fabs(x));
                }
            }
        }
    }
    s.lap_p50 = racing::sim::percentile(laps, 0.5);
    s.lap_p95 = racing::sim::percentile(laps, 0.95);
    s.err_p50 = racing::sim::percentile(errors, 0.5);
    s.err_p99 = racing::sim::percentile(errors, 0.99);
    s.speed_p50 = racing::sim::percentile(speeds, 0.5);  // sorts speeds
    s.speed_max = speeds.empty() ? std::nan("") : speeds.back();
    s.delta_p50 = racing::sim::percentile(deltas, 0.5);
    s.delta_p95 = racing::sim::percentile(deltas, 0.95);
    s.delta_max = deltas.empty() ? std::nan("") : deltas.back();
    return s;
}

std::string summary_line(const TypeResult& t, const Stats& s, bool timing) {
    namespace sim = racing::sim;
    std::string out = "{\"type\":\"" + sim::json_escape(t.label) + "\",\"slug\":\"" + t.slug + "\"";
    if (!t.construction_error.empty()) {
        out += ",\"status\":\"construction_failed\",\"reason\":\"" + sim::json_escape(t.construction_error) + "\"";
    } else {
        out += ",\"status\":\"constructed\",\"seeds\":" + std::to_string(s.seeds) +
               ",\"successes\":" + std::to_string(s.successes) +
               ",\"success_rate\":" + num(double(s.successes) / double(s.seeds)) +
               ",\"run_errors\":" + std::to_string(s.run_errors) + ",\"failures\":{";
        bool first = true;
        for (const auto& [why, n] : s.failures) {
            out += (first ? "\"" : ",\"") + sim::json_escape(why) + "\":" + std::to_string(n);
            first = false;
        }
        out += "},\"lap_time_s\":{\"p50\":" + num(s.lap_p50) + ",\"p95\":" + num(s.lap_p95) +
               "},\"gate_error_true_m\":{\"p50\":" + num(s.err_p50) + ",\"p99\":" + num(s.err_p99) +
               "},\"max_speed_m_s\":{\"p50\":" + num(s.speed_p50) + ",\"max\":" + num(s.speed_max) +
               "},\"lap_delta_abs_s\":{\"seeds\":" + std::to_string(s.delta_seeds) + ",\"p50\":" + num(s.delta_p50) +
               ",\"p95\":" + num(s.delta_p95) + ",\"max\":" + num(s.delta_max) + "}";
    }
    if (timing) {
        out += ",\"wall_s\":" + num(t.wall_s);
    }
    return out + "}";
}

/// Build the stack in T once; if it builds, run every seed.
template <typename T>
TypeResult sweep_type(const racing::sim::SweepType<T>& type,
                      const racing::config::BaselineConfig& baseline,
                      const racing::config::VehicleConfig& vehicle,
                      const racing::CourseMap<double>& map,
                      const racing::config::ScenarioConfig& scenario,
                      double control_hz,
                      std::uint64_t first,
                      std::uint64_t seeds,
                      unsigned jobs) {
    using StackT = racing::baseline::Composition<T>;
    TypeResult out;
    out.slug = type.slug;
    out.label = type.label;
    for (std::uint32_t g = 0; g < map.gate_count; ++g) {
        out.in_plane_ulp.push_back(racing::sim::in_plane_ulp<T>(map.gates[g]));
    }
    const auto start = std::chrono::steady_clock::now();
    if (auto probe = racing::harness::make_stack<StackT>(map, vehicle, baseline); !probe) {
        out.construction_error = probe.error();
        return out;
    }
    out.runs.resize(static_cast<std::size_t>(seeds));
    racing::harness::for_each_index(seeds, jobs, [&](std::uint64_t i) {
        out.runs[i] = racing::sim::run_one<StackT>(baseline, vehicle, map, scenario, control_hz, first + i);
    });
    out.wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = racing::harness::Args::parse(argc, argv, {"timing"});
    if (!args) {
        return fail(args.error());
    }
    if (auto bad =
            args->only({"scenario", "seeds", "first-seed", "jobs", "out-dir", "min-success", "commit", "timing"})) {
        return fail(*bad);
    }
    const auto scenario_path = args->value("scenario");
    const auto seeds_arg = args->value("seeds");
    const auto out_dir = args->value("out-dir");
    if (!scenario_path || !seeds_arg || !out_dir) {
        return fail("usage: --scenario <yaml> --seeds <n> --out-dir <dir> [--first-seed <s>] [--jobs <j>] "
                    "[--min-success <f>] [--commit <text>] [--timing]");
    }
    const auto seeds = racing::harness::parse_number<std::uint64_t>(*seeds_arg);
    if (!seeds || *seeds == 0 || *seeds > 1'000'000) {
        return fail("--seeds must be an integer in [1, 1000000]");
    }
    unsigned jobs = std::max(1u, std::thread::hardware_concurrency());
    if (const auto j = args->value("jobs")) {
        const auto v = racing::harness::parse_number<unsigned>(*j);
        if (!v || *v == 0 || *v > 1024) {
            return fail("--jobs must be an integer in [1, 1024]");
        }
        jobs = *v;
    }
    double min_success = 0.99;
    if (const auto m = args->value("min-success")) {
        const auto v = racing::harness::parse_number<double>(*m);
        if (!v || !(*v >= 0 && *v <= 1)) {
            return fail("--min-success must be a fraction in [0, 1]");
        }
        min_success = *v;
    }
    const bool timing = args->has("timing");

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
    std::error_code ec;
    std::filesystem::create_directories(*out_dir, ec);
    if (ec) {
        return fail("cannot create " + *out_dir + ": " + ec.message());
    }

    std::vector<TypeResult> results;
    std::apply(
        [&](const auto&... type) {
            const auto one = [&](const auto& t) {
                results.push_back(
                    sweep_type(t, *baseline, *vehicle, course->map, *scenario, control_hz, first, *seeds, jobs));
                const TypeResult& r = results.back();
                std::printf("  %-26s %s (%.1f s)\n",
                            r.label.c_str(),
                            r.construction_error.empty() ? "ran" : "construction failed",
                            r.wall_s);
            };
            (one(type), ...);
        },
        racing::sim::kSweepTypes);
    const TypeResult& reference = results.front();  // double

    const std::filesystem::path dir(*out_dir);
    const auto open = [&](const std::string& name, std::ofstream& f) {
        f.open(dir / name, std::ios::binary);
        return static_cast<bool>(f);
    };
    std::ofstream summary, deltas, gates, table;
    if (!open("summary.jsonl", summary) || !open("lap_delta.jsonl", deltas) || !open("gate_error.jsonl", gates) ||
        !open("summary.md", table)) {
        return fail("cannot write to " + *out_dir);
    }

    // The command line, without --out-dir and --jobs: neither changes a result.
    std::string command = "precision_sweep_" + std::string(racing::baseline::name);
    for (int i = 1; i < argc; ++i) {
        if (const std::string_view a(argv[i]); a == "--out-dir" || a == "--jobs") {
            ++i;
            continue;
        }
        command += " " + std::string(argv[i]);
    }
    char seeds_line[96];
    std::snprintf(seeds_line,
                  sizeof seeds_line,
                  "%llu..%llu (%llu seeds)",
                  static_cast<unsigned long long>(first),
                  static_cast<unsigned long long>(first + *seeds - 1),
                  static_cast<unsigned long long>(*seeds));
    table << "# Precision sweep: " << racing::baseline::name << " on " << scenario->name << "\n\n"
          << "Generated by `precision_sweep_" << racing::baseline::name << "` (docs/plan/precision-sweep/). "
          << "Do not edit; rerun the command instead.\n\n"
          << "- Command: `" << command << "` (`--jobs` and `--out-dir` omitted; they change no result)\n"
          << "- Commit: " << args->value("commit").value_or("unrecorded") << "\n"
          << "- Seeds: " << seeds_line << "\n"
          << "- The simulator, FC, scorer and log run in double; only the stack runs in the listed type.\n"
          << "- P1: success rate ≥ " << cell(100.0 * min_success, 1) << " %.\n"
          << "- Lap Δ: |lap time − double's| over the laps of seeds where both succeeded.\n\n"
          << "| Type | Stack | Success | P1 | Lap time p50 / p95 [s] | True gate error p50 / p99 [m] "
          << "| Max speed p50 / max [m/s] | Lap Δ p50 / p95 / max [s] |" << (timing ? " Wall time [s] |" : "") << "\n"
          << "|---|---|---|---|---|---|---|---|" << (timing ? "---|" : "") << "\n";
    std::string notes;
    std::string by_gate;
    for (const TypeResult& t : results) {
        const bool is_reference = &t == &reference;
        const Stats s = stats_of(t, is_reference ? nullptr : &reference);
        summary << summary_line(t, s, timing) << "\n";

        std::ofstream lines;
        if (!open(t.slug + ".jsonl", lines)) {
            return fail("cannot write to " + *out_dir);
        }
        for (const racing::sim::Run& r : t.runs) {
            lines << racing::sim::to_json(r) << "\n";
        }
        if (!is_reference) {
            for (std::size_t i = 0; i < t.runs.size() && i < reference.runs.size(); ++i) {
                if (const auto d = lap_delta(t.runs[i], reference.runs[i])) {
                    deltas << "{\"type\":\"" << t.slug << "\",\"seed\":" << t.runs[i].seed << ",\"lap_delta_s\":[";
                    for (std::size_t k = 0; k < d->size(); ++k) {
                        deltas << (k ? "," : "") << num((*d)[k]);
                    }
                    deltas << "]}\n";
                }
            }
        }

        if (t.construction_error.empty()) {
            by_gate += "\n### " + t.label +
                       "\n\n| Gate | Center [m] | Passages | True error p50 / p99 [m] "
                       "| Paired excess p50 [m] | In-plane ULP [m] | ¼ in-plane ULP [m] |\n"
                       "|---|---|---|---|---|---|---|\n";
            const std::vector<GateStats> per_gate = gate_stats(t, reference, course->map);
            for (std::uint32_t g = 0; g < course->map.gate_count; ++g) {
                const GateStats& gs = per_gate[g];
                const auto& c = course->map.gates[g].center;
                gates << "{\"type\":\"" << t.slug << "\",\"gate\":" << gs.id << ",\"passages\":" << gs.passages
                      << ",\"true_error_m\":{\"p50\":" << num(gs.err_p50) << ",\"p99\":" << num(gs.err_p99)
                      << "},\"pairs\":" << gs.pairs << ",\"paired_excess_m\":{\"p50\":" << num(gs.excess_p50)
                      << "},\"in_plane_ulp_m\":" << num(gs.ulp) << ",\"quarter_ulp_m\":" << num(gs.ulp / 4) << "}\n";
                char buf[256];
                std::snprintf(buf,
                              sizeof buf,
                              "| %d | (%g, %g, %g) | %llu | %s / %s | ",
                              gs.id,
                              c[0],
                              c[1],
                              c[2],
                              static_cast<unsigned long long>(gs.passages),
                              cell(gs.err_p50, 3).c_str(),
                              cell(gs.err_p99, 3).c_str());
                by_gate += buf;
                if (is_reference) {
                    by_gate += "reference";
                } else if (gs.pairs == 0) {
                    by_gate += "—";
                } else {
                    std::snprintf(
                        buf, sizeof buf, "%.2e (%llu pairs)", gs.excess_p50, static_cast<unsigned long long>(gs.pairs));
                    by_gate += buf;
                }
                std::snprintf(buf, sizeof buf, " | %.3g | %.3g |\n", gs.ulp, gs.ulp / 4);
                by_gate += buf;
            }
        }

        table << "| " << t.label << " | ";
        if (!t.construction_error.empty()) {
            table << "failed (note) | — | no | — | — | — | — |" << (timing ? " " + cell(t.wall_s, 1) + " |" : "")
                  << "\n";
            notes += "- **" + t.label + "**: the stack did not build: " + t.construction_error + "\n";
            continue;
        }
        const double rate = double(s.successes) / double(s.seeds);
        table << "built | " << s.successes << " / " << s.seeds << " (" << cell(100.0 * rate, 1) << " %) | "
              << (rate >= min_success ? "yes" : "no") << " | " << cell(s.lap_p50, 3) << " / " << cell(s.lap_p95, 3)
              << " | " << cell(s.err_p50, 3) << " / " << cell(s.err_p99, 3) << " | " << cell(s.speed_p50, 3) << " / "
              << cell(s.speed_max, 3) << " | ";
        if (is_reference) {
            table << "reference";
        } else if (s.delta_seeds == 0) {
            table << "—";
        } else {
            char buf[96];
            std::snprintf(buf, sizeof buf, "%.2e / %.2e / %.2e", s.delta_p50, s.delta_p95, s.delta_max);
            table << buf;
        }
        table << " |" << (timing ? " " + cell(t.wall_s, 1) + " |" : "") << "\n";
        for (const auto& [why, n] : s.failures) {
            notes += "- **" + t.label + "**: " + std::to_string(n) + " × " + why + "\n";
        }
    }
    table << "\n## Failures\n\n" << (notes.empty() ? std::string("None.\n") : notes);
    table << "\n## Gate error by gate\n\n"
          << "- Per built type and gate, over the passages of runs without a run error, as in the table above.\n"
          << "- Paired excess: the median of (this type's true gate error − double's) over the passages of the "
          << "same seed, lap and gate in both.\n"
          << "- In-plane ULP: the type's resolution at the gate's center, on the world axes that span the gate's "
          << "plane (its error is measured in that plane).\n"
          << by_gate;
    std::printf("precision_sweep_%s: %zu types, seeds %s, files in %s\n",
                std::string(racing::baseline::name).c_str(),
                results.size(),
                seeds_line,
                out_dir->c_str());
    return 0;
}
