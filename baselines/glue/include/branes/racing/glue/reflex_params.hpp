#pragma once
/// @file reflex_params.hpp
/// @brief reflex QuadrotorParams for a vehicle file.
///
/// The one mapping from a VehicleConfig onto reflex's QuadrotorParams, shared
/// by the SITL plant (sim/vehicle.hpp) and the controllers built from the
/// vehicle file (CascadedPid). The vehicle file has no battery model yet, so
/// the reflex defaults (a large battery, no sag) stay.

#include <branes/racing/config/config.hpp>
#include <branes/racing/result.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <string>

namespace branes::racing::glue {

namespace rx = branes::reflex;

/// reflex QuadrotorParams for `v` in scalar type T, with the ground at
/// `ground_z` [m]. Fails unless the result passes quadrotor_params_valid.
template <typename T = double>
[[nodiscard]] result<rx::model::QuadrotorParams<T>, std::string> reflex_params_from(const config::VehicleConfig& v,
                                                                                    double ground_z) {
    if (v.rotor_count != rx::model::kRotors) {
        return std::string("the reflex model simulates quadrotors only (rotor_count 4)");
    }
    rx::model::QuadrotorParams<T> p;
    p.mass = T(v.mass);
    p.inertia = {T(v.inertia[0]), T(v.inertia[1]), T(v.inertia[2])};
    p.arm_length = T(v.arm_length);
    p.thrust_coefficient = T(v.rotor.thrust_coefficient);
    p.torque_coefficient = T(v.rotor.torque_coefficient);
    p.max_thrust = T(v.rotor.max_thrust);
    p.motor_time_constant = T(v.rotor.time_constant);
    p.drag_linear = {T(v.drag.linear[0]), T(v.drag.linear[1]), T(v.drag.linear[2])};
    p.drag_quadratic = {T(v.drag.quadratic[0]), T(v.drag.quadratic[1]), T(v.drag.quadratic[2])};
    p.ground_z = T(ground_z);
    if (!rx::model::quadrotor_params_valid(p)) {
        return std::string("vehicle '" + v.name + "' does not give valid reflex model parameters");
    }
    return p;
}

}  // namespace branes::racing::glue
