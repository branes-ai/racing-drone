// Shared fixtures for the sim/ tests: the repo's RP-1 vehicle and A2RL course,
// and a quiet scenario (no randomization, no faults) to edit per test.
#pragma once

#include <branes/racing/config/config.hpp>
#include <branes/racing/sim/simulator.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <functional>
#include <optional>
#include <vector>

namespace sim_fixtures {

namespace racing = branes::racing;

inline const std::filesystem::path src = RACING_SOURCE_DIR;

inline racing::config::VehicleConfig rp1() {
    auto v = racing::config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(v);
    return *v;
}

/// RP-1 with no sensor noise, bias, or latency: measurements equal the truth.
inline racing::config::VehicleConfig rp1_ideal_imu() {
    auto v = rp1();
    v.imu.gyro_noise_density = 0;
    v.imu.accel_noise_density = 0;
    v.imu.gyro_bias_random_walk = 0;
    v.imu.accel_bias_random_walk = 0;
    v.imu.initial_gyro_bias_sigma = 0;
    v.imu.initial_accel_bias_sigma = 0;
    v.imu.latency_s = 0;
    return v;
}

inline racing::config::CourseConfig course() {
    auto c = racing::config::load_course(src / "courses/a2rl-2025.yaml");
    REQUIRE(c);
    return *c;
}

inline racing::config::ScenarioConfig scenario(double duration_s, std::uint64_t seed = 20261026) {
    racing::config::ScenarioConfig s;
    s.name = "test";
    s.duration_s = duration_s;
    s.seed = seed;
    s.world_sim = racing::config::default_world_sim(racing::config::kRacingScenario);
    return s;
}

/// A command for tick time t.
using Script = std::function<std::optional<racing::ThrustRates<double>>(racing::Timestamp)>;

/// Run `sim` to the end, answering every tick with `script` (nullopt: no
/// command), and return every event.
inline std::vector<racing::sim::SimEvent> run(racing::sim::Simulator& sim, const Script& script) {
    std::vector<racing::sim::SimEvent> out;
    while (auto e = sim.next()) {
        if (e->channel == racing::log::Channel::tick) {
            if (const auto u = script(e->t)) {
                sim.command(*u);
            }
        }
        out.push_back(std::move(*e));
    }
    return out;
}

inline racing::ThrustRates<double> motors(double m0, double m1, double m2, double m3) {
    racing::ThrustRates<double> u{};
    u.kind = racing::CommandKind::motors;
    u.motors = {m0, m1, m2, m3};
    return u;
}

inline racing::ThrustRates<double> thrust_rates(double thrust, double p = 0, double q = 0, double r = 0) {
    racing::ThrustRates<double> u{};
    u.thrust = thrust;
    u.body_rates = {p, q, r};
    return u;
}

}  // namespace sim_fixtures
