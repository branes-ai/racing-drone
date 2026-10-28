// Unit tests for racing's data contracts' invariants (msg/course.hpp,
// msg/perception.hpp): each valid() accepts a hand-built good value and
// rejects one violation per invariant. The framework's contracts are tested in
// anatomy.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/perception.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <vector>

using namespace branes::racing;

namespace {

CourseMap<double> one_gate_course() {
    CourseMap<double> m{};
    m.gate_count = 1;
    m.gates[0] = Gate<double>{7, {5.0, 5.0, 2.0}, Quat<double>{}, 1.5, 1.5};
    m.sequence_length = 1;
    m.sequence[0] = 7;
    m.laps = 1;
    m.geofence = Geofence<double>{{0.0, 0.0, 0.0}, {10.0, 10.0, 5.0}};
    m.start = StartPose<double>{{1.0, 1.0, 0.125}, 0.0};
    return m;
}

}  // namespace

TEST_CASE("GateObservations: I1 count bound, I2 quality in [0, 1]", "[msg]") {
    GateObservations<double> g{};
    REQUIRE(valid(g));
    g.count = 1;
    g.gates[0].quality = 0.9;
    REQUIRE(valid(g));
    g.gates[0].quality = 1.5;
    REQUIRE_FALSE(valid(g));  // I2
    g.gates[0].quality = 0.5;
    g.count = kMaxVisibleGates + 1;
    REQUIRE_FALSE(valid(g));  // I1
}

// Test names stay ASCII: CTest passes them back to Catch2 as filters, and
// Windows mangles non-ASCII command-line characters (≥ became =).

TEST_CASE("CourseMap: every invariant", "[msg]") {
    const CourseMap<double> good = one_gate_course();
    REQUIRE(valid(good));

    auto m = good;
    m.gate_count = 0;
    REQUIRE_FALSE(valid(m));  // I1

    m = good;
    m.gate_count = 2;
    m.gates[1] = m.gates[0];
    REQUIRE_FALSE(valid(m));  // I1: duplicate id

    m = good;
    m.gates[0].inner_width = 0.0;
    REQUIRE_FALSE(valid(m));  // I2

    m = good;
    m.sequence[0] = 99;
    REQUIRE_FALSE(valid(m));  // I3: unknown id

    m = good;
    m.laps = 0;
    REQUIRE_FALSE(valid(m));  // I4

    m = good;
    m.gates[0].center = {20.0, 5.0, 2.0};
    REQUIRE_FALSE(valid(m));  // I5: gate outside the fence

    m = good;
    m.start.position = {-1.0, 1.0, 0.1};
    REQUIRE_FALSE(valid(m));  // I5: start outside the fence
}

TEST_CASE("CourseMap: cast to float and back keeps a valid course", "[msg]") {
    const CourseMap<double> m = one_gate_course();
    const CourseMap<float> f = cast<float>(m);
    REQUIRE(valid(f));
    REQUIRE(cast<double>(f) == m);  // every value here is exact in float
}
