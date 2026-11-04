// SetpointShaper and ShapedPid (slots/shaped_pid.hpp; anatomy
// docs/plan/flow-testing, T5): the shaper scales the feed-forward of every
// sample and nothing else, by feedforward_gain from the baseline; ShapedPid is
// CascadedPid on the shaped reference, so with a gain of 1 it is CascadedPid.

#include <branes/racing/anatomy_glue.hpp>
#include <branes/racing/slots/context.hpp>
#include <branes/racing/slots/shaped_pid.hpp>
#include <branes/racing/slots/slots.hpp>

#include "sim_fixtures.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <map>
#include <string>

using namespace branes::racing;
using branes::racing::glue::CascadedPid;

static_assert(Controller<ShapedPid<double>, double>);
static_assert(ContextBuilt<ShapedPid<double>, SlotContext>);
static_assert(ContextBuilt<SetpointShaper<double>, SlotContext>);

namespace {

config::BaselineConfig baseline_with(std::map<std::string, std::string> params) {
    config::BaselineConfig b;
    b.name = "t";
    b.parameters = std::move(params);
    return b;
}

/// A reference of three samples, each with a velocity and an acceleration.
Reference<double> moving() {
    Reference<double> r{};
    r.count = 3;
    for (std::uint32_t i = 0; i < r.count; ++i) {
        ReferenceSample<double>& s = r.samples[i];
        s.t = Timestamp{std::int64_t(i) * 1'000'000};
        s.position = {1.0 + i, 2.0, 3.0};
        s.velocity = {2.0, -1.0, 0.5 * i};
        s.acceleration = {0.25, 0.5, -1.0};
        s.yaw = 0.1 * i;
        s.progress = 1.0 * i;
    }
    return r;
}

StateEstimate<double> state(std::int64_t t_ns, Vec3<double> p, Vec3<double> v = {}) {
    StateEstimate<double> x{};
    x.t_valid = Timestamp{t_ns};
    x.position = p;
    x.velocity = v;
    return x;
}

}  // namespace

TEST_CASE("setpoint shaper: scales each sample's feed-forward, and nothing else", "[shaped][controller]") {
    const Reference<double> r = moving();
    CHECK(SetpointShaper<double>(1.0).shape(r) == r);

    const Reference<double> half = SetpointShaper<double>(0.5).shape(r);
    REQUIRE(half.count == r.count);
    for (std::uint32_t i = 0; i < r.count; ++i) {
        const ReferenceSample<double>& a = r.samples[i];
        const ReferenceSample<double>& b = half.samples[i];
        CHECK(b.t == a.t);
        CHECK(b.position == a.position);
        CHECK(b.yaw == a.yaw);
        CHECK(b.progress == a.progress);
        for (std::size_t k = 0; k < 3; ++k) {
            CHECK(b.velocity[k] == 0.5 * a.velocity[k]);
            CHECK(b.acceleration[k] == 0.5 * a.acceleration[k]);
        }
    }
    CHECK(valid(half));
    // A sample past the count is not the reference's, and is left as it is.
    CHECK(half.samples[r.count] == r.samples[r.count]);
}

TEST_CASE("setpoint shaper: feedforward_gain from the baseline, in [0, 1]", "[shaped][controller]") {
    const auto v = sim_fixtures::rp1();
    const auto course = sim_fixtures::course();
    const auto make = [&](std::map<std::string, std::string> params) {
        return SetpointShaper<double>::from_context(SlotContext{v, baseline_with(std::move(params)), course.map});
    };
    const auto d = make({});
    REQUIRE(d);
    CHECK(d->gain() == 1.0);
    const auto g = make({{"feedforward_gain", "0.8"}});
    REQUIRE(g);
    CHECK(g->gain() == 0.8);
    for (const char* bad : {"1.5", "-0.1", "fast"}) {
        const auto r = make({{"feedforward_gain", bad}});
        REQUIRE_FALSE(r);
        CHECK(r.error().find("SetpointShaper: baseline parameter feedforward_gain") == 0);
    }
}

TEST_CASE("shaped pid: CascadedPid on the shaped reference", "[shaped][controller]") {
    const auto v = sim_fixtures::rp1();
    const auto course = sim_fixtures::course();
    for (const char* gain : {"1", "0.8"}) {
        const auto b = baseline_with({{"feedforward_gain", gain}});
        const SlotContext ctx{v, b, course.map};
        auto shaped = ShapedPid<double>::from_context(ctx);
        auto pid = CascadedPid<double>::from_context(ctx);
        auto shaper = SetpointShaper<double>::from_context(ctx);
        REQUIRE(shaped);
        REQUIRE(pid);
        REQUIRE(shaper);
        const Reference<double> r = moving();
        for (std::int64_t k = 0; k < 50; ++k) {
            const auto x = state(k * 4'000'000, {0.01 * double(k), 0.0, 0.1}, {0.2, 0.0, 0.0});
            const ThrustRates<double> u = shaped->step(x, r, ConflictSet<double>{});
            CHECK(u == pid->step(x, shaper->shape(r), ConflictSet<double>{}));
        }
    }
    // With a gain below 1 it commands otherwise than CascadedPid on the reference.
    const auto b = baseline_with({{"feedforward_gain", "0.5"}});
    const SlotContext ctx{v, b, course.map};
    auto shaped = ShapedPid<double>::from_context(ctx);
    auto pid = CascadedPid<double>::from_context(ctx);
    REQUIRE(shaped);
    REQUIRE(pid);
    bool differs = false;
    for (std::int64_t k = 0; k < 10; ++k) {
        const auto x = state(k * 4'000'000, {0.0, 0.0, 0.1});
        differs = differs ||
                  !(shaped->step(x, moving(), ConflictSet<double>{}) == pid->step(x, moving(), ConflictSet<double>{}));
    }
    CHECK(differs);
}
