#pragma once
/// @file batch.hpp
/// @brief One scored SITL run without a log, and the summaries over many.
///
/// The batch runners (sitl_batch_<baseline>, precision_sweep_<baseline>) share
/// it, so a run, its JSON line, and its statistics mean the same in both. A
/// run is the SITL closed loop of sitl_<baseline>, through the same Runner,
/// scored as it goes (Scorer, with the scenario's speed limit), and stopped
/// once it is complete or has failed.

#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/log/log.hpp>
#include <branes/racing/msg/codec.hpp>
#include <branes/racing/sim/score.hpp>
#include <branes/racing/sim/simulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace branes::racing::sim {

/// One seed's run.
struct Run {
    std::uint64_t seed{0};
    Score score;
    std::string error;  ///< set if the run could not be set up or fed
};

/// One closed-loop run of StackT, scored. Mirrors apps/sitl/main.cpp without the log.
template <class StackT>
[[nodiscard]] Run run_one(const config::BaselineConfig& baseline,
                          const config::VehicleConfig& vehicle,
                          const CourseMap<double>& map,
                          config::ScenarioConfig scenario,
                          double control_hz,
                          std::uint64_t seed) {
    Run out;
    out.seed = seed;
    scenario.seed = seed;
    auto sim = Simulator::create(vehicle, map, scenario, control_hz);
    if (!sim) {
        out.error = sim.error();
        return out;
    }
    auto stack = harness::make_stack<StackT>(map, vehicle, baseline);
    if (!stack) {
        out.error = stack.error();
        return out;
    }
    harness::Runner runner(*stack);
    Scorer scorer(map, scenario.speed_limit_m_s);
    while (auto ev = sim->next()) {
        if (ev->channel == log::Channel::truth) {
            const auto x = codec::decode_truth(ev->payload);
            if (!x) {
                out.error = "undecodable truth record";
                return out;
            }
            scorer.on_truth(*x);
        }
        auto outputs = runner.feed(log::LogRecord{ev->channel, ev->t, std::move(ev->payload)});
        if (!outputs) {
            out.error = outputs.error();
            return out;
        }
        for (const auto& o : *outputs) {
            if (o.channel == log::Channel::command) {
                const auto u = codec::decode_command(o.payload);
                if (!u) {
                    out.error = "the stack produced a command that does not decode";
                    return out;
                }
                sim->command(*u);
            } else if (o.channel == log::Channel::state) {
                if (const auto x = codec::decode_state(o.payload)) {
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

/// A string as a JSON string literal's body: quotes and backslashes escaped,
/// as in to_json(Score).
[[nodiscard]] inline std::string json_escape(const std::string& s) {
    std::string out;
    for (const char c : s) {
        if (c == '"' || c == '\\') {
            out += '\\';
        }
        out += c;
    }
    return out;
}

/// A run's JSON line, without the newline: {"seed":…,"score":{…}}, or
/// {"seed":…,"run_error":"…"} for a run that could not be set up or fed.
[[nodiscard]] inline std::string to_json(const Run& r) {
    std::string out = "{\"seed\":" + std::to_string(r.seed);
    if (!r.error.empty()) {
        return out + ",\"run_error\":\"" + json_escape(r.error) + "\"}";
    }
    return out + ",\"score\":" + to_json(r.score) + "}";
}

/// Nearest-rank percentile of `v` (sorted in place); NaN if empty.
[[nodiscard]] inline double percentile(std::vector<double>& v, double p) {
    if (v.empty()) {
        return std::nan("");
    }
    std::sort(v.begin(), v.end());
    const auto rank = static_cast<std::size_t>(std::ceil(p * static_cast<double>(v.size())));
    return v[rank == 0 ? 0 : rank - 1];
}

/// A failure message without its time or gate count, for grouping.
[[nodiscard]] inline std::string failure_reason(const std::string& failure) {
    for (const char* cut : {" at t=", ":"}) {
        if (const auto i = failure.find(cut); i != std::string::npos) {
            return failure.substr(0, i);
        }
    }
    return failure;
}

}  // namespace branes::racing::sim
