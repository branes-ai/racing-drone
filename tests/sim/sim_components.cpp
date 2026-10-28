// Unit tests for racing's oracle gate projector (sim/gates.hpp). The
// simulator's components (vehicle, IMU, FC, camera model) are tested in
// anatomy.

#include <branes/racing/anatomy_sim.hpp>
#include <branes/racing/sim/gates.hpp>

#include "sim_fixtures.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>
#include <numbers>

using namespace branes::racing;
using namespace branes::racing::sim;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using sim_fixtures::rp1;

// ── vehicle ─────────────────────────────────────────────────────────

// ── IMU ─────────────────────────────────────────────────────────────

// ── simulated FC ────────────────────────────────────────────────────

// ── oracle gates ────────────────────────────────────────────────────

namespace {

/// The vehicle level at `p`, yawed by `yaw`, at rest.
TruthState<double> pose(double x, double y, double z, double yaw) {
    TruthState<double> s{};
    s.position = {x, y, z};
    s.attitude = Quat<double>{std::cos(yaw / 2), 0, 0, std::sin(yaw / 2)};
    return s;
}

}  // namespace

TEST_CASE("gates: a gate straight ahead projects symmetrically about the principal point", "[sim][gates]") {
    const auto cam = rp1().camera;
    const auto map = sim_fixtures::course().map;
    const Gate<double>& g1 = map.gates[0];  // id 1 at (10, 4, 2), facing +x
    // Camera center 5 m before the gate on its axis.
    const auto x =
        pose(g1.center[0] - 5 - cam.translation_body[0], g1.center[1], g1.center[2] - cam.translation_body[2], 0);
    const auto obs = project_gates(map, x, cam, 1.5);
    REQUIRE(obs.count >= 1);
    const auto& o = obs.gates[0];
    CHECK(o.gate_id == 1);
    CHECK(o.quality == 1);
    CHECK(o.covariance[0] == 2.25);
    CHECK(o.covariance[63] == 2.25);
    CHECK(o.covariance[1] == 0);
    // TL, TR, BR, BL: left/right and top/bottom in the image.
    CHECK(o.corners[0][0] < o.corners[1][0]);
    CHECK(o.corners[0][1] < o.corners[3][1]);
    double mu = 0, mv = 0;
    for (const auto& c : o.corners) {
        mu += c[0] / 4;
        mv += c[1] / 4;
    }
    CHECK_THAT(mu, WithinAbs(cam.cx, 1e-9));
    CHECK_THAT(mv, WithinAbs(cam.cy, 1e-9));
    CHECK(valid(obs));
}

TEST_CASE("gates: at most four gates, nearest first; none behind the camera", "[sim][gates]") {
    const auto cam = rp1().camera;
    const auto map = sim_fixtures::course().map;
    // On the start pad's line, looking down the row of gates 1–5. The 155°
    // fisheye also sees gates 10 and 11 off to the left; more than four are
    // in view, so the four nearest are reported, nearest first.
    const auto x = pose(3, 4, 2, 0);
    const auto ahead = project_gates(map, x, cam, 1);
    REQUIRE(ahead.count == kMaxVisibleGates);
    const auto distance = [&](std::int32_t id) {
        double d = -1;
        for (std::uint32_t g = 0; g < map.gate_count; ++g) {
            if (map.gates[g].id == id) {
                const auto c = to_camera(cam, x, map.gates[g].center);
                d = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
            }
        }
        REQUIRE(d >= 0);  // a known gate id
        return d;
    };
    CHECK(ahead.gates[1].gate_id == 1);  // 7 m ahead; gate 11 (5.4 m, to the left) is nearer
    for (std::uint32_t i = 0; i < ahead.count; ++i) {
        if (i > 0) {
            CHECK(distance(ahead.gates[i - 1].gate_id) <= distance(ahead.gates[i].gate_id));
        }
        for (const auto& c : ahead.gates[i].corners) {
            CHECK(c[0] >= 0);
            CHECK(c[0] < cam.width);
            CHECK(c[1] >= 0);
            CHECK(c[1] < cam.height);
        }
    }
    // Facing the hall's end wall at x = 0: every gate is behind.
    CHECK(project_gates(map, pose(3, 4, 2, std::numbers::pi), cam, 1).count == 0);
}
