// ReplayTrace (sim/replay_trace.hpp), the reflex-trace/1 writer of trace_export:
// samples sit on the k·dt_trace grid (T1); a row with a wrong width or a
// non-finite value is refused and leaves the trace as it was (T2); events
// outside the samples are dropped, and truncate() drops the samples and
// events after the new end (T3). The JSON carries the schema, provenance,
// frames, and channels the player reads, with 7 significant digits.

#include <branes/racing/sim/replay_trace.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <limits>
#include <span>
#include <string>
#include <vector>

using branes::racing::sim::ReplayProvenance;
using branes::racing::sim::ReplayTrace;

namespace {

ReplayProvenance provenance() {
    ReplayProvenance p;
    p.commit = "abc1234";
    p.baseline = "b0-gt-pid";
    p.scenario = "scenarios/b0-nominal.yaml";
    p.seed = 1700;
    p.dt_sim = 0.001;
    p.dt_trace = 0.05;
    return p;
}

// One sample of a trace with position (3) and rotor_speeds (4).
bool add(ReplayTrace& trace, std::array<double, 3> p, std::array<double, 4> w) {
    const std::array<std::span<const double>, 2> row{std::span<const double>(p), std::span<const double>(w)};
    return trace.sample(row);
}

ReplayTrace two_channels() {
    ReplayTrace trace(provenance());
    trace.channel("position", 3);
    trace.channel("rotor_speeds", 4);
    return trace;
}

}  // namespace

TEST_CASE("replay trace: samples sit on the grid (T1)", "[sim][replay]") {
    ReplayTrace trace = two_channels();
    for (int k = 0; k < 4; ++k) {
        REQUIRE(add(trace, {1.0 * k, 0, 2}, {100, 100, 100, 100}));
    }
    const std::string json = trace.to_json(0.11);
    CHECK(json.find("\"t\":[0,0.05,0.1,0.15]") != std::string::npos);
    CHECK(json.find("\"schema\":\"reflex-trace/1\"") != std::string::npos);
    CHECK(json.find("\"block\":\"b0-gt-pid\"") != std::string::npos);
    CHECK(json.find("\"seed\":1700") != std::string::npos);
    CHECK(json.find("\"rotors\":\"px4-quad-x\"") != std::string::npos);
    CHECK(json.find("\"vehicle\":{\"arm_length\":0.11}") != std::string::npos);
    CHECK(json.find("\"position\":[[0,0,2],[1,0,2],[2,0,2],[3,0,2]]") != std::string::npos);
}

TEST_CASE("replay trace: a bad row is refused and changes nothing (T2)", "[sim][replay]") {
    ReplayTrace trace = two_channels();
    REQUIRE(add(trace, {0, 0, 0}, {1, 2, 3, 4}));
    CHECK_FALSE(add(trace, {0, std::numeric_limits<double>::quiet_NaN(), 0}, {1, 2, 3, 4}));
    CHECK_FALSE(add(trace, {0, 0, 0}, {1, 2, 3, std::numeric_limits<double>::infinity()}));
    const std::array<double, 3> p{0, 0, 0};
    const std::array<double, 3> short_w{1, 2, 3};
    const std::array<std::span<const double>, 2> narrow{std::span<const double>(p), std::span<const double>(short_w)};
    CHECK_FALSE(trace.sample(narrow));
    const std::array<std::span<const double>, 1> missing{std::span<const double>(p)};
    CHECK_FALSE(trace.sample(missing));
    CHECK(trace.size() == 1);
    CHECK(trace.to_json(0).find("\"rotor_speeds\":[[1,2,3,4]]") != std::string::npos);
}

TEST_CASE("replay trace: events stay within the samples (T3)", "[sim][replay]") {
    ReplayTrace trace = two_channels();
    trace.event(0, "before any sample");
    for (int k = 0; k < 5; ++k) {
        REQUIRE(add(trace, {0, 0, 0}, {0, 0, 0, 0}));
    }
    trace.event(-0.01, "before the first");
    trace.event(0.03, "takeoff");
    trace.event(0.2, "at the end");
    trace.event(0.21, "after the end");
    trace.event(0.1, "");
    REQUIRE(trace.events().size() == 2);
    CHECK(trace.events()[0].second == "takeoff");

    trace.truncate(3);  // t = 0, 0.05, 0.1
    CHECK(trace.size() == 3);
    REQUIRE(trace.events().size() == 1);
    CHECK(trace.events()[0].second == "takeoff");
    CHECK(trace.to_json(0).find("\"position\":[[0,0,0],[0,0,0],[0,0,0]]") != std::string::npos);
}

TEST_CASE("replay trace: values carry 7 significant digits, and strings are escaped", "[sim][replay]") {
    ReplayProvenance p = provenance();
    p.scenario = "a \"quoted\" path";
    ReplayTrace trace(p);
    trace.channel("rotor_speeds", 4);
    const std::array<double, 4> w{1234.56789, -0.0, 1e-9, 2.0 / 3.0};
    const std::array<std::span<const double>, 1> row{std::span<const double>(w)};
    REQUIRE(trace.sample(row));
    const std::string json = trace.to_json(0);
    CHECK(json.find("[1234.568,0,1e-09,0.6666667]") != std::string::npos);
    CHECK(json.find("\"scenario\":\"a \\\"quoted\\\" path\"") != std::string::npos);
    CHECK(json.find("\"vehicle\"") == std::string::npos);  // no arm length given
}
