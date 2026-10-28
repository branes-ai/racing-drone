// A type's resolution at a value and at a gate (docs/plan/gate-error-by-gate/):
//  - ulp_at<T>(x) is the gap from T(|x|) to its upper neighbor, checked
//    against the neighbors themselves at the course's magnitudes, in every
//    sweep type; in posit<16,1> it is 2^-3 at 72 m (9 fraction bits in [64, 128));
//  - in_plane_ulp<T> takes the world axes spanning each a2rl-2025 gate's
//    plane: y and z for the gates facing ±x, x and z for gates 6 and 11.

#include <branes/racing/config/config.hpp>
#include <branes/racing/sim/sweep_types.hpp>

#include "precision_fixtures.hpp"

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>

using namespace branes::racing;
using precision_fixtures::neighbors;

namespace {

const std::filesystem::path src = RACING_SOURCE_DIR;

}  // namespace

TEMPLATE_LIST_TEST_CASE("ulp_at: the gap to the upper neighbor", "[precision][resolution]", sim::SweepTypes) {
    using T = TestType;
    for (const double x : {0.004, 0.5, 1.0, 2.0, 4.0, 5.0, 9.0, 14.0, 25.0, 32.0, 40.0, 55.0, 64.0, 68.0, 72.0}) {
        const T t = static_cast<T>(x);
        const auto [below, above] = neighbors(t);
        INFO("x = " << x << ", T(x) = " << double(t));
        CHECK(sim::ulp_at<T>(x) == above - double(t));
        CHECK(sim::ulp_at<T>(-x) == sim::ulp_at<T>(x));
    }
}

TEST_CASE("ulp_at: posit<16,1> resolves 0.125 m at 72 m", "[precision][resolution]") {
    CHECK(sim::ulp_at<sim::posit16>(72.0) == 0.125);
}

TEMPLATE_LIST_TEST_CASE("in_plane_ulp: the axes spanning each a2rl-2025 gate's plane",
                        "[precision][resolution]",
                        sim::SweepTypes) {
    using T = TestType;
    const auto course = config::load_course(src / "courses/a2rl-2025.yaml");
    REQUIRE(course);
    REQUIRE(course->map.gate_count == 11);
    for (std::uint32_t g = 0; g < course->map.gate_count; ++g) {
        const Gate<double>& gate = course->map.gates[g];
        const bool faces_y = gate.id == 6 || gate.id == 11;  // yaw ±π/2: the plane holds world x
        const double lateral = sim::ulp_at<T>(gate.center[faces_y ? 0 : 1]);
        const double up = sim::ulp_at<T>(gate.center[2]);
        INFO("gate " << gate.id);
        CHECK(sim::in_plane_ulp<T>(gate) == std::max(lateral, up));
    }
}
