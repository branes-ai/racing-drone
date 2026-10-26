#pragma once
/// @file vehicle.hpp
/// @brief The reflex plant parameters for a vehicle file, and the per-run randomization.
///
/// reflex_params_from() maps a VehicleConfig onto reflex's QuadrotorParams.
/// The vehicle file has no battery model yet, so the reflex defaults (a large
/// battery, no sag) stay. The ground is a plane at the start pad's height, so
/// a vehicle starts at rest on the pad.
///
/// randomize() applies a scenario's `randomization:` block: each parameter p
/// becomes p·(1 + u) with u uniform in [−f, f], drawn from the run's
/// randomization stream, and the start pose is jittered. The draws are
/// returned so the harness can log them.

#include <branes/racing/config/config.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/result.hpp>
#include <branes/racing/util/splitmix64.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <cstdio>
#include <string>

namespace branes::racing::sim {

namespace rx = branes::reflex;

using PlantParams = rx::model::QuadrotorParams<double>;

/// reflex QuadrotorParams for `v`, with the ground at `ground_z` [m].
/// Fails unless the result passes reflex's quadrotor_params_valid.
[[nodiscard]] inline result<PlantParams, std::string> reflex_params_from(const config::VehicleConfig& v,
                                                                         double ground_z) {
    if (v.rotor_count != rx::model::kRotors) {
        return std::string("the reflex model simulates quadrotors only (rotor_count 4)");
    }
    PlantParams p;
    p.mass = v.mass;
    p.inertia = {v.inertia[0], v.inertia[1], v.inertia[2]};
    p.arm_length = v.arm_length;
    p.thrust_coefficient = v.rotor.thrust_coefficient;
    p.torque_coefficient = v.rotor.torque_coefficient;
    p.max_thrust = v.rotor.max_thrust;
    p.motor_time_constant = v.rotor.time_constant;
    p.drag_linear = {v.drag.linear[0], v.drag.linear[1], v.drag.linear[2]};
    p.drag_quadratic = {v.drag.quadratic[0], v.drag.quadratic[1], v.drag.quadratic[2]};
    p.ground_z = ground_z;
    if (!rx::model::quadrotor_params_valid(p)) {
        return std::string("vehicle '" + v.name + "' does not give valid reflex model parameters");
    }
    return p;
}

/// The values one run drew from its scenario's randomization block.
struct Draws {
    double mass_scale{1};
    double inertia_scale{1};
    double drag_scale{1};
    double motor_time_constant_scale{1};
    double start_dx{0};    ///< [m]
    double start_dy{0};    ///< [m]
    double start_dyaw{0};  ///< [rad]
};

/// The draws as YAML text, for the log's sim_config record.
[[nodiscard]] inline std::string to_yaml(const Draws& d) {
    char buf[512];
    std::snprintf(buf,
                  sizeof buf,
                  "mass_scale: %.17g\ninertia_scale: %.17g\ndrag_scale: %.17g\nmotor_time_constant_scale: %.17g\n"
                  "start_dx_m: %.17g\nstart_dy_m: %.17g\nstart_dyaw_rad: %.17g\n",
                  d.mass_scale,
                  d.inertia_scale,
                  d.drag_scale,
                  d.motor_time_constant_scale,
                  d.start_dx,
                  d.start_dy,
                  d.start_dyaw);
    return buf;
}

/// Draw a run's randomization from `rng`, always in the same order (so a
/// zero-width field still consumes its draw and the other draws do not shift).
[[nodiscard]] inline Draws draw(const config::Randomization& r, SplitMix64& rng) {
    Draws d;
    d.mass_scale = 1 + rng.symmetric(r.mass);
    d.inertia_scale = 1 + rng.symmetric(r.inertia);
    d.drag_scale = 1 + rng.symmetric(r.drag);
    d.motor_time_constant_scale = 1 + rng.symmetric(r.motor_time_constant);
    d.start_dx = rng.symmetric(r.start_xy_m);
    d.start_dy = rng.symmetric(r.start_xy_m);
    d.start_dyaw = rng.symmetric(r.start_yaw_rad);
    return d;
}

/// `p` with the draws applied. Randomization fractions are < 1 (the scenario
/// loader checks), so every scaled parameter stays positive.
[[nodiscard]] inline PlantParams apply(PlantParams p, const Draws& d) {
    p.mass *= d.mass_scale;
    for (double& j : p.inertia) {
        j *= d.inertia_scale;
    }
    for (std::size_t i = 0; i < 3; ++i) {
        p.drag_linear[i] *= d.drag_scale;
        p.drag_quadratic[i] *= d.drag_scale;
    }
    p.motor_time_constant *= d.motor_time_constant_scale;
    return p;
}

/// The start pose with the draws applied.
[[nodiscard]] inline StartPose<double> apply(StartPose<double> s, const Draws& d) {
    s.position[0] += d.start_dx;
    s.position[1] += d.start_dy;
    s.yaw += d.start_dyaw;
    return s;
}

}  // namespace branes::racing::sim
