// The scoreboard (sim/score.hpp) on hand-built truth trajectories: gate
// passages in order, laps and lap times, gate errors, frame hits, ground
// contact, and geofence breaches.

#include <branes/racing/sim/score.hpp>

#include "sim_fixtures.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <string>
#include <vector>

using namespace branes::racing;
using namespace branes::racing::sim;
using Catch::Matchers::WithinAbs;

namespace {

/// A two-gate course along x, one lap or two.
CourseMap<double> line(std::uint32_t laps) {
    CourseMap<double> m{};
    m.gate_count = 2;
    m.gates[0] = Gate<double>{1, {10, 0, 2}, {}, 1.5, 1.5};
    m.gates[1] = Gate<double>{2, {20, 0, 2}, {}, 1.5, 1.5};
    m.sequence_length = 2;
    m.sequence[0] = 1;
    m.sequence[1] = 2;
    m.laps = laps;
    m.geofence = Geofence<double>{{-5, -5, 0}, {30, 5, 5}};
    m.start = StartPose<double>{{0, 0, 0.1}, 0};
    REQUIRE(valid(m));
    return m;
}

TruthState<double> at(double t_s, double x, double y, double z, bool ground = false) {
    TruthState<double> s{};
    s.t = Timestamp{static_cast<std::int64_t>(t_s * 1e9)};
    s.position = {x, y, z};
    s.on_ground = ground;
    return s;
}

/// Fly along x at 10 m/s at (y, z), sampled every 10 ms, from x0 to x1.
void fly(Scorer& s, double& t, double x0, double x1, double y, double z) {
    for (double x = x0; x <= x1 + 1e-9; x += 0.1) {
        s.on_truth(at(t, x, y, z));
        t += 0.01;
    }
}

}  // namespace

TEST_CASE("score: one lap through both gates, timed from takeoff", "[sim][score]") {
    Scorer s(line(1));
    s.on_truth(at(0, 0, 0, 0.1, true));
    double t = 0.5;
    fly(s, t, 0, 25, 0.3, 2.4);
    const Score r = s.score();
    INFO(to_json(r));
    CHECK(r.success);
    CHECK(r.laps_completed == 1);
    CHECK(r.gates_passed == 2);
    REQUIRE(r.lap_times_s.size() == 1);
    CHECK_THAT(r.lap_times_s[0], WithinAbs(2.0, 0.011));  // 20 m at 10 m/s, interpolated crossing
    REQUIRE(r.passages.size() == 2);
    CHECK_THAT(r.passages[0].true_error_m, WithinAbs(0.5, 1e-9));  // (0.3, 0.4) off center
    CHECK(r.passages[0].estimated_error_m == -1);                  // no estimate logged
}

TEST_CASE("score: the estimated gate error uses the latest estimate", "[sim][score]") {
    Scorer s(line(1));
    double t = 0;
    s.on_truth(at(t, 0, 0, 2));
    StateEstimate<double> e{};
    e.t_valid = Timestamp{0};
    e.position = {10, 1, 2};  // 1 m to the side at the crossing
    s.on_estimate(e);
    fly(s, t, 0, 15, 0, 2);
    const Score r = s.score();
    REQUIRE(r.passages.size() == 1);
    CHECK_THAT(r.passages[0].estimated_error_m, WithinAbs(1.0, 1e-9));
    CHECK_FALSE(r.success);  // only one of two gates
}

TEST_CASE("score: two laps need the sequence twice", "[sim][score]") {
    Scorer s(line(2));
    double t = 0;
    fly(s, t, 0, 25, 0, 2);
    CHECK_FALSE(s.complete());
    // Loop back outside the gates (y = 4), then through again.
    for (double x = 25; x >= -2; x -= 0.5) {
        s.on_truth(at(t, x, 4, 2));
        t += 0.05;
    }
    fly(s, t, -2, 25, 0, 2);
    const Score r = s.score();
    CHECK(r.success);
    CHECK(r.laps_completed == 2);
    CHECK(r.lap_times_s.size() == 2);
    CHECK(r.passages.size() == 4);
    CHECK(r.passages[3].lap == 2);
}

TEST_CASE("score: skipping a gate does not count the next one", "[sim][score]") {
    Scorer s(line(1));
    double t = 0;
    fly(s, t, 0, 25, 3, 2);  // passes beside gate 1 (y = 3, beyond its frame), through nothing
    CHECK(s.score().gates_passed == 0);
}

TEST_CASE("score: hitting a frame, landing early, and leaving the geofence fail the run", "[sim][score]") {
    {
        Scorer s(line(1));
        double t = 0;
        fly(s, t, 0, 25, 1.0, 2);  // 0.25 m outside the opening, inside the 0.5 m frame
        const Score r = s.score();
        CHECK(r.collision);
        CHECK_FALSE(r.success);
        CHECK(r.failure.find("frame of gate 1") != std::string::npos);
    }
    {
        Scorer s(line(1));
        s.on_truth(at(0, 0, 0, 1));
        s.on_truth(at(0.1, 1, 0, 0.1, true));
        CHECK(s.score().collision);
    }
    {
        Scorer s(line(1));
        s.on_truth(at(0, 0, 0, 2));
        s.on_truth(at(0.1, 0, 0, 6));  // above the 5 m ceiling
        const Score r = s.score();
        CHECK(r.geofence_breach);
        CHECK(r.failure.find("geofence") != std::string::npos);
    }
}

TEST_CASE("score: the JSON line names every field", "[sim][score]") {
    Scorer s(line(1));
    const std::string j = to_json(s.score());
    for (const char* key : {"\"success\":false",
                            "\"laps_completed\":0",
                            "\"lap_time_s\":[]",
                            "\"gate_error_true_m\":[]",
                            "\"collision\":false",
                            "\"failure\":\"incomplete: 0 gates passed\""}) {
        INFO(j);
        CHECK(j.find(key) != std::string::npos);
    }
}
