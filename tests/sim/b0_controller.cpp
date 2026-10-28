// Baseline B0's controller (baselines/glue/cascaded_pid.hpp): built from the
// vehicle file with the speed cap, it holds on the first tick and on dt ≤ 0,
// commands m·g and zero rates at equilibrium, and keeps the thrust within the
// cascade's limits. Its dt accuracy is ε_max from the velocity gains
// (docs/plan/time-base/), and a dt its TimeBase rejects is held and counted.

#include <branes/racing/glue/cascaded_pid.hpp>
#include <branes/racing/slots/context.hpp>
#include <branes/racing/slots/slots.hpp>

#include "sim_fixtures.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <limits>
#include <map>
#include <string>

using namespace branes::racing;
using branes::racing::glue::CascadedPid;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

static_assert(Controller<CascadedPid<double>, double>);
static_assert(ContextBuilt<CascadedPid<double>, SlotContext>);

namespace {

config::BaselineConfig baseline_with(std::map<std::string, std::string> params) {
    config::BaselineConfig b;
    b.name = "t";
    b.parameters = std::move(params);
    return b;
}

CascadedPid<double> make(const config::VehicleConfig& v, std::map<std::string, std::string> params = {}) {
    const auto course = sim_fixtures::course();
    const auto b = baseline_with(std::move(params));
    auto c = CascadedPid<double>::from_context(SlotContext{v, b, course.map});
    REQUIRE(c);
    return std::move(c).value();
}

StateEstimate<double> state(std::int64_t t_ns, Vec3<double> p, Vec3<double> v = {}) {
    StateEstimate<double> x{};
    x.t_valid = Timestamp{t_ns};
    x.position = p;
    x.velocity = v;
    return x;
}

Reference<double> hold_at(Vec3<double> p, double yaw = 0) {
    Reference<double> r{};
    r.count = 1;
    r.samples[0].position = p;
    r.samples[0].yaw = yaw;
    return r;
}

const ConflictSet<double> none{};

}  // namespace

TEST_CASE("cascaded pid: from_context applies the speed cap and rejects a weak vehicle", "[b0][controller]") {
    const auto v = sim_fixtures::rp1();
    const auto c = make(v);
    const auto& cfg = c.position_loop().config();
    CHECK(cfg.max_speed_xy == 4.5);  // the default cap
    CHECK(cfg.max_speed_up == 3.0);  // cascade_config_from's, below the cap
    CHECK(cfg.max_speed_down == 2.0);
    CHECK(cfg.mass == v.mass);

    const auto slow = make(v, {{"max_speed_m_s", "1.5"}});
    CHECK(slow.position_loop().config().max_speed_xy == 1.5);
    CHECK(slow.position_loop().config().max_speed_up == 1.5);
    CHECK(slow.position_loop().config().max_speed_down == 1.5);

    // 0.8 · 4 · T_max ≤ m·g: the cascade cannot hover with margin.
    auto weak = v;
    weak.rotor.max_thrust = 0.25 * v.mass * 9.80665 / 0.8 * 0.99;
    const auto course = sim_fixtures::course();
    const auto b = baseline_with({});
    const auto r = CascadedPid<double>::from_context(SlotContext{weak, b, course.map});
    REQUIRE_FALSE(r);
    CHECK(r.error().find("thrust-to-weight") != std::string::npos);
    CHECK_FALSE(CascadedPid<double>::from_context(SlotContext{v, baseline_with({{"max_speed_m_s", "0"}}), course.map}));
}

TEST_CASE("cascaded pid: the first tick and a non-positive dt hold the last command", "[b0][controller]") {
    const auto v = sim_fixtures::rp1();
    auto c = make(v);
    const ThrustRates<double> first = c.step(state(0, {3, 4, 0.1}), hold_at({3, 4, 2}), none);
    CHECK(valid(first));
    CHECK(first.kind == CommandKind::thrust_rates);
    CHECK(first.t == Timestamp{0});
    CHECK(first.thrust == v.mass * 9.80665);  // the position loop's reset target: hover
    CHECK(first.body_rates == Vec3<double>{});

    const ThrustRates<double> moved = c.step(state(4'000'000, {3, 4, 0.1}), hold_at({3, 4, 2}), none);
    CHECK(moved.thrust > first.thrust);  // the setpoint is 1.9 m up
    const ThrustRates<double> again = c.step(state(4'000'000, {3, 4, 0.1}), hold_at({9, 9, 9}), none);
    CHECK(again == moved);  // dt = 0: held, whatever the reference
    const ThrustRates<double> back = c.step(state(2'000'000, {3, 4, 0.1}), hold_at({9, 9, 9}), none);
    CHECK(back.thrust == moved.thrust);  // dt < 0: held, re-stamped
    CHECK(back.body_rates == moved.body_rates);
    CHECK(back.t == Timestamp{2'000'000});
}

TEST_CASE("cascaded pid: at the setpoint and level it commands m*g and zero rates", "[b0][controller]") {
    const auto v = sim_fixtures::rp1();
    auto c = make(v);
    const Vec3<double> p{10, 4, 2};
    (void)c.step(state(0, p), hold_at(p), none);
    const ThrustRates<double> u = c.step(state(4'000'000, p), hold_at(p), none);
    // Zero error in every loop: f = g·e₃ exactly, and the attitude target is
    // level up to the rounding of the quaternion-from-matrix conversion.
    CHECK(u.thrust == v.mass * 9.80665);
    for (const double w : u.body_rates) {
        CHECK_THAT(w, WithinAbs(0.0, 1e-12));
    }
}

TEST_CASE("cascaded pid: the thrust stays within the cascade limits for any setpoint", "[b0][controller]") {
    const auto v = sim_fixtures::rp1();
    for (const double dz : {-100.0, -2.0, 2.0, 100.0}) {
        auto c = make(v);
        const auto& cfg = c.position_loop().config();
        (void)c.step(state(0, {10, 4, 2}), hold_at({10, 4, 2 + dz}), none);
        for (std::int64_t k = 1; k <= 50; ++k) {
            const ThrustRates<double> u = c.step(state(k * 4'000'000, {10, 4, 2}), hold_at({30, 4, 2 + dz}), none);
            INFO("dz " << dz << " tick " << k);
            REQUIRE(valid(u));
            REQUIRE(u.thrust >= cfg.min_thrust);
            REQUIRE(u.thrust <= cfg.max_thrust);
        }
    }
}

TEST_CASE("cascaded pid: the dt accuracy is eps_max from the velocity gains", "[b0][controller]") {
    const auto v = sim_fixtures::rp1();
    const auto c = make(v, {{"control_rate_hz", "250"}});
    CHECK(c.time_base().nominal_ns() == 4'000'000);
    // cascade_config_from: k_v = (1/τ)/14 and k_i = k_v²/10, so r = 0.1 and
    // ε_max = k_v·T_s·(1 + r²)/r. Both sides take a handful of roundings
    // (≤ 6 each, ½ ε apiece), so they agree to 8 ε.
    const double kv = (1.0 / v.rotor.time_constant) / 14.0;
    const double r = 0.1;
    const double expected = kv * 0.004 * (1.0 + r * r) / r;
    CHECK_THAT(c.time_base().rel_tol(), WithinRel(expected, 8 * std::numeric_limits<double>::epsilon()));

    auto pd = c.position_loop().config();
    pd.velocity[1].kd = 0.1;
    const auto rejected = CascadedPid<double>::dt_tolerance(pd, 0.004);
    REQUIRE_FALSE(rejected);
    CHECK(rejected.error().find("PI velocity loops") != std::string::npos);
}

TEST_CASE("cascaded pid: a dt longer than two periods is held and counted", "[b0][controller]") {
    const auto v = sim_fixtures::rp1();
    auto c = make(v);  // the default 100 Hz: 10 ms period, 20 ms longest interval
    REQUIRE(c.time_base().max_ns() == 20'000'000);
    (void)c.step(state(0, {3, 4, 0.1}), hold_at({3, 4, 2}), none);
    const ThrustRates<double> first = c.step(state(10'000'000, {3, 4, 0.1}), hold_at({3, 4, 2}), none);
    CHECK(c.timing_faults() == 0);
    const ThrustRates<double> late = c.step(state(40'000'000, {3, 4, 0.1}), hold_at({9, 9, 9}), none);
    CHECK(c.timing_faults() == 1);
    CHECK(late.thrust == first.thrust);  // held, re-stamped
    CHECK(late.body_rates == first.body_rates);
    CHECK(late.t == Timestamp{40'000'000});
    (void)c.step(state(50'000'000, {3, 4, 0.1}), hold_at({3, 4, 2}), none);
    CHECK(c.timing_faults() == 1);  // back on the period
}
