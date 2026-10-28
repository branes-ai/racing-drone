#pragma once
/// @file simulator.hpp
/// @brief Simulator: racing's SITL event source, BasicSimulator with the gate oracle.
///
/// Racing-only (docs/assessments/framework-boundary.md). The closed loop (the
/// reflex plant, the simulated FC, the IMU, faults, camera timing, truth) is
/// mission-neutral, in basic_simulator.hpp; racing adds its world (the course:
/// the start pose and the gates) and its oracle sensor (GateOracle), which
/// projects the gates at every camera frame onto the oracle_gates channel.
///
/// Events come out as BasicSimulator documents, with oracle_gates as the
/// oracle event:
///
///   sim_config (t = 0 only) → IMU deliveries → camera → oracle_gates → truth → tick

#include <branes/racing/config/config.hpp>
#include <branes/racing/log/channels.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/gate_codec.hpp>
#include <branes/racing/msg/time.hpp>
#include <branes/racing/msg/truth.hpp>
#include <branes/racing/result.hpp>
#include <branes/racing/sim/basic_simulator.hpp>
#include <branes/racing/sim/gates.hpp>

#include <optional>
#include <string>
#include <utility>

namespace branes::racing::sim {

/// Racing's oracle sensor: the gates a perfect detector sees at each frame
/// (project_gates), on the oracle_gates channel.
///
/// Model: map_ — the course [owned]; cam_ — the camera; sigma_ — the corner
/// noise, 1σ per coordinate [px]. Invariants: I1 valid(map_); I2 sigma_ ≥ 0.
class GateOracle {
public:
    /// Pre: valid(map), corner_sigma_px ≥ 0.
    GateOracle(const CourseMap<double>& map, const config::CameraParams& cam, double corner_sigma_px)
        : map_(map), cam_(cam), sigma_(corner_sigma_px) {}

    [[nodiscard]] std::optional<SimEvent> observe(const TruthState<double>& x, Timestamp t) const {
        return SimEvent{log::oracle_gates, t, codec::encode(project_gates(map_, x, cam_, sigma_))};
    }

private:
    CourseMap<double> map_;
    config::CameraParams cam_;
    double sigma_;
};

static_assert(OracleSensor<GateOracle>);

/// Racing's simulator: the course's start pose and the gate oracle.
class Simulator : public BasicSimulator<GateOracle> {
public:
    using Base = BasicSimulator<GateOracle>;

    /// Validate the inputs and build the run. Pre: `course` is valid (loader).
    /// Fails, besides BasicSimulator's checks, if the scenario does not carry
    /// racing's sim parameters (config::follows_racing_schema).
    [[nodiscard]] static result<Simulator, std::string> create(const config::VehicleConfig& vehicle,
                                                               const CourseMap<double>& course,
                                                               const config::ScenarioConfig& scenario,
                                                               double control_hz,
                                                               const SimOptions& options = {}) {
        if (!config::follows_racing_schema(scenario)) {
            return std::string(
                "the scenario has no racing sim parameters (set world_sim, see config::kRacingScenario)");
        }
        const double sigma = config::oracle_corner_sigma_px(scenario);
        auto base = Base::create(
            vehicle, course.start, scenario, control_hz, GateOracle(course, vehicle.camera, sigma), options);
        if (!base) {
            return base.error();
        }
        return Simulator(std::move(base).value());
    }

private:
    explicit Simulator(Base&& base) : Base(std::move(base)) {}
};

}  // namespace branes::racing::sim
