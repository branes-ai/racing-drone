// Unit tests for the composition and the harness pieces: the null slots model
// their concepts, a null Stack produces valid outputs, SyntheticSensors is
// deterministic and time-ordered, and Args enforces the harness inputs.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/compose/stack.hpp>
#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/slots/null.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <string>
#include <vector>

using namespace branes::racing;

static_assert(Estimator<NullEstimator<double>, double>);
static_assert(GateDetector<NullGateDetector<double>, double>);
static_assert(Planner<NullPlanner<double>, double>);
static_assert(Controller<NullController<double>, double>);
static_assert(SafetyFilter<NullSafetyFilter<double>, double>);
static_assert(DaaTracker<NullDaaTracker<double>, double>);
static_assert(FcBridge<NullFcBridge<double>, double>);
static_assert(Estimator<NullEstimator<float>, float>);  // scalar-generic

namespace {

template <typename T>
using NullStack = Stack<T,
                        NullEstimator<T>,
                        NullGateDetector<T>,
                        NullPlanner<T>,
                        NullController<T>,
                        NullSafetyFilter<T>,
                        NullDaaTracker<T>,
                        NullFcBridge<T>>;

const std::filesystem::path src = RACING_SOURCE_DIR;

}  // namespace

TEST_CASE("Stack: null composition produces valid, time-stamped outputs", "[stack]") {
    const auto course = config::load_course(src / "courses/a2rl-2025.yaml");
    REQUIRE(course);
    NullStack<double> stack(course->map);
    stack.on_imu(ImuSample<double>{});
    const TickOutput<double> y = stack.on_tick(Timestamp{1'000'000});
    CHECK(valid(y.state));
    CHECK(valid(y.command));
    CHECK(y.state.t_valid == Timestamp{1'000'000});
    CHECK(y.command.t == Timestamp{1'000'000});
    CHECK(valid(y.reference));
    CHECK(y.reference == stack.planner().plan(stack.course(), y.state));  // the planner's output, as computed

    // Same scalar-generic composition in float.
    NullStack<float> fstack(cast<float>(course->map));
    CHECK(valid(fstack.on_tick(Timestamp{0}).command));
}

TEST_CASE("SyntheticSensors: deterministic, time-ordered, IMU before camera before tick", "[stack]") {
    const auto vehicle = config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(vehicle);
    auto a = harness::SyntheticSensors::create(*vehicle, 0.05, 7, 100.0);
    auto b = harness::SyntheticSensors::create(*vehicle, 0.05, 7, 100.0);
    REQUIRE(a);
    REQUIRE(b);
    std::int64_t last = -1;
    int last_rank = 0;
    std::size_t imu = 0, frames = 0, ticks = 0;
    while (auto ea = a->next()) {
        const auto eb = b->next();
        REQUIRE(eb);
        REQUIRE(ea->channel == eb->channel);
        REQUIRE(ea->t == eb->t);
        REQUIRE(ea->payload == eb->payload);
        const int rank = static_cast<int>(ea->channel);  // imu 1, camera 2, tick 3
        REQUIRE(ea->t.ns >= last);
        if (ea->t.ns == last) {
            REQUIRE(rank > last_rank);
        }
        last = ea->t.ns;
        last_rank = rank;
        if (ea->channel == log::Channel::imu) {
            ++imu;
        } else if (ea->channel == log::Channel::camera) {
            ++frames;
        } else {
            ++ticks;
        }
    }
    REQUIRE_FALSE(b->next());
    CHECK(imu == 50u);    // 1 kHz × 50 ms
    CHECK(frames == 5u);  // 90 Hz × 50 ms, frames at 0, 11.1, ..., 44.4 ms
    CHECK(ticks == 5u);   // 100 Hz × 50 ms
}

TEST_CASE("SyntheticSensors: a different seed changes the IMU noise", "[stack]") {
    const auto vehicle = config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(vehicle);
    auto a = harness::SyntheticSensors::create(*vehicle, 0.01, 1, 100.0);
    auto b = harness::SyntheticSensors::create(*vehicle, 0.01, 2, 100.0);
    REQUIRE(a);
    REQUIRE(b);
    const auto ea = a->next();
    const auto eb = b->next();
    REQUIRE(ea);
    REQUIRE(eb);
    REQUIRE(ea->channel == log::Channel::imu);
    REQUIRE(ea->payload != eb->payload);
}

TEST_CASE("Args: values, flags, and the allowed set", "[stack]") {
    const char* argv[] = {"prog", "--log", "in.brlog", "--verify", "--out", "o"};
    const auto a = harness::Args::parse(6, argv, {"verify"});
    REQUIRE(a);
    CHECK(a->value("log") == "in.brlog");
    CHECK(a->has("verify"));
    CHECK_FALSE(a->only({"log", "out", "verify"}));
    CHECK(a->only({"log", "out"}));  // --verify not allowed here

    const char* missing[] = {"prog", "--log"};
    CHECK_FALSE(harness::Args::parse(2, missing, {}));
    const char* positional[] = {"prog", "stray"};
    CHECK_FALSE(harness::Args::parse(2, positional, {}));
}

TEST_CASE("SyntheticSensors::create rejects rates and durations with no representable period", "[stack]") {
    const auto vehicle = config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(vehicle);
    CHECK(harness::SyntheticSensors::create(*vehicle, 0.1, 1, 100.0));
    CHECK_FALSE(harness::SyntheticSensors::create(*vehicle, 0.1, 1, 1e12));  // period would round to 0 ns
    CHECK_FALSE(harness::SyntheticSensors::create(*vehicle, 0.1, 1, 1e-9));  // period would overflow int64
    CHECK_FALSE(harness::SyntheticSensors::create(*vehicle, 0.0, 1, 100.0));
    CHECK_FALSE(harness::SyntheticSensors::create(*vehicle, 1e300, 1, 100.0));  // end time would overflow
    auto fast = *vehicle;
    fast.imu.rate_hz = 1e12;
    CHECK_FALSE(harness::SyntheticSensors::create(fast, 0.1, 1, 100.0));
}

TEST_CASE("Runner rejects a tick earlier than the previous one", "[stack]") {
    const auto course = config::load_course(src / "courses/a2rl-2025.yaml");
    REQUIRE(course);
    NullStack<double> stack(course->map);
    harness::Runner runner(stack, log::kRacingChannels);
    REQUIRE(runner.feed(log::LogRecord{log::Channel::tick, Timestamp{100}, {}}));
    REQUIRE(runner.feed(log::LogRecord{log::Channel::tick, Timestamp{100}, {}}));  // equal is fine
    const auto back = runner.feed(log::LogRecord{log::Channel::tick, Timestamp{50}, {}});
    REQUIRE_FALSE(back);
    CHECK(back.error().find("earlier than the previous tick") != std::string::npos);
}

TEST_CASE("Runner: each tick yields state, reference, and command, in that order", "[stack]") {
    const auto course = config::load_course(src / "courses/a2rl-2025.yaml");
    REQUIRE(course);
    NullStack<double> stack(course->map);
    harness::Runner runner(stack, log::kRacingChannels);
    const Timestamp t{4'000'000};
    const auto out = runner.feed(log::LogRecord{log::Channel::tick, t, {}});
    REQUIRE(out);
    REQUIRE(out->size() == 3);
    CHECK((*out)[0].channel == log::Channel::state);
    CHECK((*out)[1].channel == log::Channel::reference);
    CHECK((*out)[2].channel == log::Channel::command);
    for (const auto& o : *out) {
        CHECK(o.t == t);
    }
    const auto ref = codec::decode_reference((*out)[1].payload);
    REQUIRE(ref);
    const auto state = codec::decode_state((*out)[0].payload);
    REQUIRE(state);
    CHECK(*ref == stack.planner().plan(stack.course(), *state));
}
