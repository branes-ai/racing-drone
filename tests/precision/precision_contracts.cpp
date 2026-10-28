// The contracts in every sweep type (docs/plan/precision-sweep/):
//  - unit_tolerance<T>() is max(1e-6, 4ε_T), so double and float keep 1e-6;
//  - every unit quaternion cast to T passes is_unit in T (§2's bound), and a
//    norm off by 8 tolerances fails it;
//  - the course, cast to T, is valid in every type, and make_stack reports a
//    course that isn't valid in T as an error;
//  - reflex_params_from<T> names the vehicle parameters T cannot represent.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/anatomy_glue.hpp>
#include <branes/racing/anatomy_sweep.hpp>
#include <branes/racing/compose/stack.hpp>
#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/slots/null.hpp>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <limits>
#include <string>

using namespace branes::racing;

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

TEST_CASE("unit_tolerance: 1e-6 for double and float", "[precision][contracts]") {
    CHECK(unit_tolerance<double>() == 1e-6);
    CHECK(unit_tolerance<float>() == 1e-6f);
}

TEMPLATE_LIST_TEST_CASE("unit_tolerance: max(1e-6, 4 epsilon)", "[precision][contracts]", sim::SweepTypes) {
    using T = TestType;
    const double four_eps = 4.0 * double(std::numeric_limits<T>::epsilon());
    const double expected = four_eps > double(T(1e-6)) ? four_eps : double(T(1e-6));
    CHECK(double(unit_tolerance<T>()) == expected);
}

TEMPLATE_LIST_TEST_CASE("is_unit: every unit quaternion cast to T is unit in T",
                        "[precision][contracts]",
                        sim::SweepTypes) {
    using T = TestType;
    SplitMix64 rng(30);
    int rejected = 0;
    for (int i = 0; i < 10'000; ++i) {
        const double w = rng.symmetric(1.0), x = rng.symmetric(1.0), y = rng.symmetric(1.0), z = rng.symmetric(1.0);
        const double n = std::sqrt(w * w + x * x + y * y + z * z);
        if (!(n > 1e-3)) {
            continue;
        }
        const Quat<double> q{w / n, x / n, y / n, z / n};
        if (!is_unit(cast<T>(q))) {
            ++rejected;
            UNSCOPED_INFO("q = (" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")");
        }
    }
    CHECK(rejected == 0);

    // A norm off by well over the tolerance is still rejected: (1 + 4 tol)² ≈ 1 + 8 tol.
    const T tol = unit_tolerance<T>();
    CHECK_FALSE(is_unit(Quat<T>{T(1) + T(4) * tol, T(0), T(0), T(0)}));
    CHECK_FALSE(is_unit(cast<T>(Quat<double>{1.1, 0.0, 0.0, 0.0})));
}

TEMPLATE_LIST_TEST_CASE("course: a2rl-2025 is valid in T", "[precision][contracts]", sim::SweepTypes) {
    using T = TestType;
    const auto course = config::load_course(src / "courses/a2rl-2025.yaml");
    REQUIRE(course);
    CHECK(valid(cast<T>(course->map)));
}

TEST_CASE("make_stack: a course that isn't valid in T is an error, not an assert", "[precision][contracts]") {
    const auto course = config::load_course(src / "courses/a2rl-2025.yaml");
    const auto vehicle = config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(course);
    REQUIRE(vehicle);
    CourseMap<double> m = course->map;
    m.gates[0].inner_width = 1e-9;  // positive in double, zero in binary16 (I2 needs > 0)
    REQUIRE(valid(m));
    const config::BaselineConfig b{};
    CHECK(harness::make_stack<NullStack<double>>(m, *vehicle, b));
    const auto s = harness::make_stack<NullStack<sim::binary16>>(m, *vehicle, b);
    REQUIRE_FALSE(s);
    CHECK(s.error().find("course is not valid") != std::string::npos);
}

TEST_CASE("reflex_params_from: the reason names what T cannot represent", "[precision][contracts]") {
    using glue::unrepresentable;
    CHECK(unrepresentable<sim::binary16>("x", 0.0).empty());
    CHECK(unrepresentable<sim::binary16>("x", 1.0).empty());
    CHECK(unrepresentable<sim::binary16>("x", 1e-6).find("x = 1e-06 is subnormal") != std::string::npos);
    CHECK(unrepresentable<sim::binary16>("x", 1e-9).find("is zero") != std::string::npos);
    CHECK(unrepresentable<sim::binary16>("x", 1e5).find("is not finite") != std::string::npos);
    CHECK(unrepresentable<sim::posit16>("x", 8.9e-7).empty());  // posits have no subnormals

    const auto vehicle = config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(vehicle);
    CHECK(glue::reflex_params_from<double>(*vehicle, 0.0));
    const auto p = glue::reflex_params_from<sim::binary16>(*vehicle, 0.0);
    REQUIRE_FALSE(p);
    INFO(p.error());
    CHECK(p.error().find("rotor.thrust_coefficient = 8.9e-07 is subnormal") != std::string::npos);
}
