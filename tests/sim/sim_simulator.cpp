// Racing's Simulator (sim/simulator.hpp): BasicSimulator with the gate
// oracle emits oracle_gates after every camera frame, in the documented order,
// and deterministically. The closed loop itself (event order, physics
// acceptance, faults) is tested in anatomy.

#include <branes/racing/log/channels.hpp>
#include <branes/racing/sim/simulator.hpp>

#include "sim_fixtures.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

using namespace branes::racing;
using namespace branes::racing::sim;
using Catch::Matchers::WithinAbs;
using sim_fixtures::course;
using sim_fixtures::motors;
using sim_fixtures::run;
using sim_fixtures::scenario;
using sim_fixtures::thrust_rates;

namespace {

Simulator make(const config::VehicleConfig& v, const config::ScenarioConfig& s, const SimOptions& o = {}) {
    auto sim = Simulator::create(v, course().map, s, 100.0, o);
    REQUIRE(sim);
    return std::move(sim).value();
}

}  // namespace

// ── event stream ────────────────────────────────────────────────────

TEST_CASE("simulator: events come out in time order, in the documented order at equal times", "[sim][simulator]") {
    const auto events = [] {
        Simulator sim = make(sim_fixtures::rp1(), scenario(0.3));
        return run(sim, [](Timestamp) { return thrust_rates(5.0); });
    }();
    const std::map<log::Channel, int> rank = {{log::Channel::sim_config, 0},
                                              {log::Channel::imu, 1},
                                              {log::Channel::camera, 2},
                                              {log::oracle_gates, 3},
                                              {log::Channel::truth, 4},
                                              {log::Channel::tick, 5}};
    REQUIRE(events.front().channel == log::Channel::sim_config);
    std::map<log::Channel, int> count;
    for (std::size_t i = 1; i < events.size(); ++i) {
        const auto& a = events[i - 1];
        const auto& b = events[i];
        INFO("event " << i);
        REQUIRE(a.t <= b.t);
        if (a.t == b.t && a.channel != log::Channel::imu) {
            REQUIRE(rank.at(a.channel) <= rank.at(b.channel));
        }
        ++count[b.channel];
    }
    CHECK(count[log::Channel::truth] == 300);  // 1 kHz for 0.3 s
    CHECK(count[log::Channel::tick] == 30);    // 100 Hz
    CHECK(count[log::Channel::camera] == 28);  // 90 Hz (11 111 111 ns): k = 0 … 27, the last at 299.99997 ms
    CHECK(count[log::oracle_gates] == count[log::Channel::camera]);
    CHECK(count[log::Channel::imu] == 299);  // 1 kHz, the last one still in flight (1 ms latency)
}

TEST_CASE("simulator: the same seed gives the same events; another seed changes the noise", "[sim][simulator]") {
    auto s = scenario(0.2);
    s.randomization.mass = 0.1;
    s.faults.push_back(config::FaultEvent{config::FaultType::camera_corrupt, 0, 1e6, {}, {}, 0.5, 0, 0, 1});
    const auto script = [](Timestamp) { return thrust_rates(11.0, 0.1); };
    Simulator a = make(sim_fixtures::rp1(), s);
    Simulator b = make(sim_fixtures::rp1(), s);
    const auto ea = run(a, script);
    const auto eb = run(b, script);
    REQUIRE(ea.size() == eb.size());
    for (std::size_t i = 0; i < ea.size(); ++i) {
        REQUIRE(ea[i].channel == eb[i].channel);
        REQUIRE(ea[i].t == eb[i].t);
        REQUIRE(ea[i].payload == eb[i].payload);
    }
    s.seed += 1;
    Simulator c = make(sim_fixtures::rp1(), s);
    const auto ec = run(c, script);
    bool imu_differs = false;
    for (std::size_t i = 0; i < ec.size() && i < ea.size(); ++i) {
        imu_differs = imu_differs || (ea[i].channel == log::Channel::imu && ea[i].payload != ec[i].payload);
    }
    CHECK(imu_differs);
    CHECK(c.draws().mass_scale != a.draws().mass_scale);
}

// ── acceptance 2: scripted trajectories ─────────────────────────────

// ── acceptance 3: closed loop through the rate loop ─────────────────

// ── faults ──────────────────────────────────────────────────────────

TEST_CASE("simulator: create rejects a scenario without racing's sim parameters", "[sim][simulator]") {
    auto s = scenario(0.1);
    s.world_sim.clear();  // a scenario built in code without kRacingScenario's parameters
    const auto sim = Simulator::create(sim_fixtures::rp1(), course().map, s, 100.0);
    REQUIRE_FALSE(sim);
    CHECK(sim.error().find("racing sim parameters") != std::string::npos);
}
