#pragma once
/// @file cascaded_pid.hpp
/// @brief CascadedPid: the reflex position and attitude loops in the Controller slot.
///
/// Baseline B0's controller (docs/plan/b0/). Each tick runs the reflex
/// PositionController on the reference's first sample (position, velocity
/// and acceleration feed-forward, yaw), then the AttitudeController on its
/// attitude target. The output is the collective thrust and the body-rate
/// setpoint: the rate loop belongs to the flight controller (SimFc in SITL),
/// so the slot sends exactly the thrust-and-rates command it defines.
///
/// The gains are reflex's cascade_config_from() for the vehicle file; the
/// baseline parameter max_speed_m_s caps the position loop's velocity
/// setpoint (horizontal, up, and down). Conflicts are not used (B0 has no DAA).
///
/// dt is the time between the estimates of consecutive ticks. On the first
/// tick, when dt ≤ 0, or with an empty reference, the last command is held,
/// re-stamped with the estimate's time.
///
/// Model
/// -----
///   position_ : reflex PositionController [owned]
///   attitude_ : reflex AttitudeController [owned]
///   last_t_   : optional<Timestamp> — t_valid of the previous tick's estimate
///   last_     : ThrustRates — the last command (hover thrust, zero rates at first)
///
/// Invariants
/// ----------
///   I1: the position and attitude configs are valid (reflex validators); the
///       vehicle's thrust-to-weight exceeds 1.25 (cascade_config_from)
///                                            — constrains position_, attitude_
///   I2: T_min ≤ last_.thrust ≤ T_max; last_.kind == thrust_rates
///                                            — constrains last_
///   I3: last_t_ never decreases               — constrains last_t_

#include <branes/racing/glue/reflex_params.hpp>
#include <branes/racing/msg/control.hpp>
#include <branes/racing/msg/daa.hpp>
#include <branes/racing/msg/estimate.hpp>
#include <branes/racing/result.hpp>
#include <branes/racing/slots/context.hpp>
#include <branes/reflex/control/attitude.hpp>
#include <branes/reflex/control/cascade.hpp>
#include <branes/reflex/control/position.hpp>
#include <branes/reflex/math/quat.hpp>
#include <branes/reflex/math/vec3.hpp>

#include <cassert>
#include <optional>
#include <string>

namespace branes::racing::glue {

namespace rx = branes::reflex;

template <typename T>
class CascadedPid {
public:
    /// Pre: position_config_valid(pos), attitude_config_valid(att).
    /// Post: I1–I3; no tick yet; last_ = hover thrust, zero rates.
    CascadedPid(const rx::control::PositionConfig<T>& pos, const rx::control::AttitudeConfig<T>& att)
        : position_(pos), attitude_(att) {
        last_.thrust = position_.target().thrust;
        assert(check_invariant());
    }

    /// From the vehicle file (cascade_config_from) and the baseline parameter
    /// max_speed_m_s (default 4.5). Fails if the vehicle cannot hover with
    /// margin (thrust-to-weight ≤ 1.25) or the parameter is out of range.
    [[nodiscard]] static result<CascadedPid, std::string> from_context(const SlotContext& ctx) {
        const auto p = reflex_params_from<T>(ctx.vehicle, ctx.course.start.position[2]);
        if (!p) {
            return p.error();
        }
        const auto max_speed = parameter_in(ctx.baseline, "max_speed_m_s", 4.5, 0.1, 50.0);
        if (!max_speed) {
            return max_speed.error();
        }
        rx::control::CascadeConfig<T> c = rx::control::cascade_config_from(*p);
        const T v = T(*max_speed);
        c.position.max_speed_xy = v;
        c.position.max_speed_up = v < c.position.max_speed_up ? v : c.position.max_speed_up;
        c.position.max_speed_down = v < c.position.max_speed_down ? v : c.position.max_speed_down;
        if (!rx::control::position_config_valid(c.position) || !rx::control::attitude_config_valid(c.attitude)) {
            return std::string("CascadedPid: vehicle '" + ctx.vehicle.name +
                               "' cannot hover inside the cascade's thrust limit (thrust-to-weight must exceed 1.25)");
        }
        return CascadedPid(c.position, c.attitude);
    }

    /// One control tick. Pre: valid(x); x and ref finite.
    /// Post: I1–I3; valid(result), result.kind == thrust_rates, result.t == x.t_valid.
    [[nodiscard]] ThrustRates<T> step(const StateEstimate<T>& x, const Reference<T>& ref, const ConflictSet<T>&) {
        assert(valid(x));  // Pre
        const std::optional<Timestamp> prev = last_t_;
        if (!prev || *prev < x.t_valid) {
            last_t_ = x.t_valid;  // I3
        }
        if (!prev || !(*prev < x.t_valid) || ref.count == 0) {
            last_.t = x.t_valid;  // hold
            assert(check_invariant());
            return last_;
        }
        const T dt = T(x.t_valid.ns - prev->ns) * T(1e-9);
        const ReferenceSample<T>& r = ref.samples[0];
        rx::control::PositionSetpoint<T> sp;
        sp.position = v3(r.position);
        sp.velocity = v3(r.velocity);
        sp.acceleration = v3(r.acceleration);
        sp.yaw = r.yaw;
        const rx::Quat<T> q{x.attitude.w, x.attitude.x, x.attitude.y, x.attitude.z};
        const rx::control::AttitudeTarget<T> target = position_.update(v3(x.position), v3(x.velocity), q, sp, dt);
        const rx::Vec3<T> rates = attitude_.update(q, target.attitude);

        last_.t = x.t_valid;
        last_.thrust = target.thrust;
        last_.body_rates = {rates(0), rates(1), rates(2)};
        assert(check_invariant());
        return last_;
    }

    [[nodiscard]] const rx::control::PositionController<T>& position_loop() const noexcept {
        return position_;
    }

private:
    [[nodiscard]] static rx::Vec3<T> v3(const Vec3<T>& a) {
        return rx::vec3(a[0], a[1], a[2]);
    }

    [[nodiscard]] bool check_invariant() const noexcept {
        const auto& c = position_.config();
        bool good = rx::control::position_config_valid(c) && rx::control::attitude_config_valid(attitude_.config());
        good = good && !(last_.thrust < c.min_thrust) && !(c.max_thrust < last_.thrust);  // I2
        return good && last_.kind == CommandKind::thrust_rates && valid(last_);           // I2
    }

    rx::control::PositionController<T> position_;
    rx::control::AttitudeController<T> attitude_;
    std::optional<Timestamp> last_t_;
    ThrustRates<T> last_{};
};

}  // namespace branes::racing::glue
