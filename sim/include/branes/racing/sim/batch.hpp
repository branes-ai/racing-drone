#pragma once
/// @file batch.hpp
/// @brief One scored racing SITL run without a log (run_one), for the batch runners.
///
/// Racing-only (docs/assessments/framework-boundary.md). The batch runners
/// (sitl_batch_<baseline>, precision_sweep_<baseline>) share it, so a run, its
/// JSON line, and its statistics mean the same in both. The closed loop, the
/// run's JSON line, and the statistics are mission-neutral (closed_loop.hpp);
/// racing supplies its simulator, its stack, its scorer (Scorer, with the
/// scenario's speed limit), and its channels.

#include <branes/racing/anatomy_sim.hpp>
#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/log/channels.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/sim/score.hpp>
#include <branes/racing/sim/simulator.hpp>

#include <cstdint>

namespace branes::racing::sim {

/// One seed's racing run.
using Run = BasicRun<Score>;

static_assert(RunScorer<Scorer>);

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
    Scorer scorer(map, scenario.speed_limit_m_s);
    return run_closed_loop<Score>(*sim, *stack, log::kRacingChannels, scorer, seed);
}

}  // namespace branes::racing::sim
