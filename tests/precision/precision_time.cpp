// TimeBase in every sweep type (docs/plan/time-base/, acceptance criteria 1–2):
//  - a 4 ms tick converts to within ½ ULP_T(0.004) + ½ ε_double·0.004 of 0.004
//    s, the bound of one correctly rounded division in double and one rounding
//    into T (the ULP from T's actual neighbors, so no tolerance is chosen);
//    main's T(Δns) · T(1e-9), which rounds each factor into T, misses it in
//    posit<16,1>;
//  - make_time_base fails, naming the period, the error and the bound, when
//    the requirement is tighter than T's actual error at 4 ms (for double
//    there is none: T is the conversion type); seconds() rejects intervals
//    outside (0, 2 periods], and ones T can't resolve (1 ns in the 16-bit
//    formats with the narrowest range);
//  - CascadedPid builds at 250 Hz in every type whose vehicle parameters
//    construct (binary16's don't, before the controller is reached).

#include <branes/racing/config/config.hpp>
#include <branes/racing/glue/cascaded_pid.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/time.hpp>
#include <branes/racing/sim/sweep_types.hpp>
#include <branes/racing/slots/context.hpp>

#include "precision_fixtures.hpp"

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <type_traits>

using namespace branes::racing;
using precision_fixtures::neighbors;

namespace {

const std::filesystem::path src = RACING_SOURCE_DIR;

constexpr std::int64_t kTick = 4'000'000;  // 250 Hz
constexpr double kTickS = 0.004;

/// ½ ULP_T(0.004) + ½ ε_double·0.004: the farthest a correctly rounded
/// division in double, then a rounding into T, can land from 0.004.
template <typename T>
double tick_bound() {
    const T t = static_cast<T>(kTickS);
    const auto [below, above] = neighbors(t);
    const double gap = double(t) < kTickS ? above - double(t) : double(t) - below;
    return 0.5 * gap + 0.5 * std::numeric_limits<double>::epsilon() * kTickS;
}

}  // namespace

TEMPLATE_LIST_TEST_CASE("time base: a 4 ms tick is within half an ULP of 0.004 s",
                        "[precision][time]",
                        sim::SweepTypes) {
    using T = TestType;
    const auto b = make_time_base<T, double>(kTick, 0.5);
    REQUIRE(b);
    const auto dt = b->seconds(kTick);
    REQUIRE(dt);
    INFO("dt = " << double(*dt) << ", bound " << tick_bound<T>());
    CHECK(std::fabs(double(*dt) - kTickS) <= tick_bound<T>());
}

TEST_CASE("time base: rounding each factor into posit<16,1> misses the bound", "[precision][time]") {
    using T = sim::posit16;
    const T two_roundings = T(double(kTick)) * T(1e-9);  // main's CascadedPid formula
    INFO("T(4e6) * T(1e-9) = " << double(two_roundings));
    CHECK(std::fabs(double(two_roundings) - kTickS) > tick_bound<T>());
    CHECK(double(two_roundings) > 3.0 * kTickS);  // 0.0156 s: 2^22 · 2^-28
}

TEMPLATE_LIST_TEST_CASE("time base: a requirement tighter than T resolves fails construction",
                        "[precision][time]",
                        sim::SweepTypes) {
    using T = TestType;
    const double w = double(kTick) / 1e9;
    const double error = std::fabs(double(static_cast<T>(w)) - w) / w;
    if constexpr (std::is_same_v<T, double>) {
        CHECK(error == 0);  // T is the conversion type: no second rounding, nothing to miss
    } else {
        REQUIRE(error > 0);  // 0.004 is not a binary fraction
        const auto b = make_time_base<T, double>(kTick, 0.5 * error);
        REQUIRE_FALSE(b);
        INFO(b.error());
        CHECK(b.error().find("4 ms interval converts with relative error") != std::string::npos);
        CHECK(b.error().find("required") != std::string::npos);
        CHECK(make_time_base<T, double>(kTick, error));  // the bound is inclusive
    }
}

TEMPLATE_LIST_TEST_CASE("time base: intervals outside (0, 2 periods] are rejected",
                        "[precision][time]",
                        sim::SweepTypes) {
    using T = TestType;
    const auto b = make_time_base<T, double>(kTick, 0.5);
    REQUIRE(b);
    CHECK(b->max_ns() == 2 * kTick);
    CHECK_FALSE(b->seconds(0));
    CHECK_FALSE(b->seconds(-kTick));
    CHECK_FALSE(b->seconds(2 * kTick + 1));
    CHECK(b->seconds(2 * kTick));
    CHECK(b->seconds(kTick / 2));  // shorter than the period: allowed, and checked
    // 1 ns is below binary16's normals and posit<16,1>'s minpos: rejected there.
    const bool resolves_1ns = !std::is_same_v<T, sim::binary16> && !std::is_same_v<T, sim::posit16>;
    CHECK(b->seconds(1).has_value() == resolves_1ns);
}

TEMPLATE_LIST_TEST_CASE("cascaded pid: builds at 250 Hz where the vehicle does", "[precision][time]", sim::SweepTypes) {
    using T = TestType;
    const auto course = config::load_course(src / "courses/a2rl-2025.yaml");
    const auto vehicle = config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(course);
    REQUIRE(vehicle);
    config::BaselineConfig b;
    b.parameters["control_rate_hz"] = "250";
    const auto c = glue::CascadedPid<T>::from_context(SlotContext{*vehicle, b, course->map});
    if constexpr (std::is_same_v<T, sim::binary16>) {
        REQUIRE_FALSE(c);
        CHECK(c.error().find("thrust_coefficient") != std::string::npos);  // before the controller
    } else {
        INFO((c ? std::string() : c.error()));
        REQUIRE(c);
        CHECK(c->time_base().nominal_ns() == kTick);
    }
}
