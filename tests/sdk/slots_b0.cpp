// Baseline B0's SDK slots (docs/plan/b0/): GroundTruthEstimator passes the
// truth through and reports its health; WaypointPlanner lays out the gate path,
// approaches every gate straight along its normal, only moves forward, and
// holds at the end; SlotContext builds slots that support it; the stack and
// the Runner deliver truth to TruthConsumer slots.

#include <branes/racing/compose/stack.hpp>
#include <branes/racing/config/config.hpp>
#include <branes/racing/harness/harness.hpp>
#include <branes/racing/msg/codec.hpp>
#include <branes/racing/slots/context.hpp>
#include <branes/racing/slots/ground_truth.hpp>
#include <branes/racing/slots/null.hpp>
#include <branes/racing/slots/waypoint.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>

using namespace branes::racing;
using Catch::Matchers::WithinAbs;

static_assert(Estimator<GroundTruthEstimator<double>, double>);
static_assert(TruthConsumer<GroundTruthEstimator<double>, double>);
static_assert(Planner<WaypointPlanner<double>, double>);
static_assert(Planner<WaypointPlanner<float>, float>);  // scalar-generic
static_assert(ContextBuilt<WaypointPlanner<double>>);
static_assert(!ContextBuilt<GroundTruthEstimator<double>>);
static_assert(!ContextBuilt<NullController<double>>);
static_assert(!TruthConsumer<NullEstimator<double>, double>);

namespace {

const std::filesystem::path src = RACING_SOURCE_DIR;

template <typename T>
using GtStack = Stack<T,
                      GroundTruthEstimator<T>,
                      NullGateDetector<T>,
                      WaypointPlanner<T>,
                      NullController<T>,
                      NullSafetyFilter<T>,
                      NullDaaTracker<T>,
                      NullFcBridge<T>>;

TruthState<double> truth(std::int64_t t_ns, Vec3<double> p, Vec3<double> v = {}) {
    TruthState<double> x{};
    x.t = Timestamp{t_ns};
    x.position = p;
    x.velocity = v;
    x.attitude = Quat<double>{0.9238795325112867, 0, 0, 0.3826834323650898};  // 45° yaw, unit
    x.body_rates = {0.1, -0.2, 0.3};
    return x;
}

StateEstimate<double> at(Vec3<double> p, std::int64_t t_ns = 0) {
    StateEstimate<double> x{};
    x.t_valid = Timestamp{t_ns};
    x.position = p;
    return x;
}

CourseMap<double> a2rl() {
    const auto c = config::load_course(src / "courses/a2rl-2025.yaml");
    REQUIRE(c);
    return c->map;
}

/// The gate normal (x axis) of gate `g`, as the planner and scorer define it.
Vec3<double> normal_of(const Gate<double>& g) {
    const Quat<double>& q = g.orientation;
    return {1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 2 * (q.x * q.z - q.w * q.y)};
}

const Gate<double>& gate(const CourseMap<double>& m, std::int32_t id) {
    std::uint32_t i = 0;
    while (i + 1 < m.gate_count && m.gates[i].id != id) {
        ++i;
    }
    REQUIRE(m.gates[i].id == id);
    return m.gates[i];
}

double dist(const Vec3<double>& a, const Vec3<double>& b) {
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

config::BaselineConfig baseline_with(std::map<std::string, std::string> params) {
    config::BaselineConfig b;
    b.name = "t";
    b.parameters = std::move(params);
    return b;
}

}  // namespace

// ── GroundTruthEstimator ────────────────────────────────────────────

TEST_CASE("ground truth: before any truth, at rest and not ok", "[b0][estimator]") {
    const GroundTruthEstimator<double> e;
    const StateEstimate<double> x = e.state_at(Timestamp{5});
    CHECK(valid(x));
    CHECK(x == NullEstimator<double>{}.state_at(Timestamp{5}));
    CHECK_FALSE(e.health().ok);
}

TEST_CASE("ground truth: the latest truth is the estimate, exactly", "[b0][estimator]") {
    GroundTruthEstimator<double> e;
    e.process_truth(truth(1'000'000, {1, 2, 3}, {4, 5, 6}));
    const TruthState<double> x = truth(2'000'000, {7, 8, 9}, {-1, -2, -3});
    e.process_truth(x);
    const StateEstimate<double> s = e.state_at(Timestamp{2'500'000});
    CHECK(valid(s));
    CHECK(s.t_valid == x.t);  // the truth's time, not the query's
    CHECK(s.position == x.position);
    CHECK(s.velocity == x.velocity);
    CHECK(s.attitude == x.attitude);
    CHECK(s.body_rates == x.body_rates);
    CHECK(s.accel_bias == Vec3<double>{});
    CHECK(s.gyro_bias == Vec3<double>{});
    for (const double c : s.covariance) {
        CHECK(c == 0);
    }
}

TEST_CASE("ground truth: health is ok with truth; staleness runs on the input clock", "[b0][estimator]") {
    GroundTruthEstimator<double> e;
    e.process_truth(truth(1'000'000, {0, 0, 0}));
    CHECK(e.health() == EstimatorHealth<double>{0, 0, true});
    ImuSample<double> s{};
    s.t = Timestamp{4'000'000};
    e.process_imu(s);
    CHECK(e.health().staleness_ns == 3'000'000);
    e.process_frame(CameraFrame{}, GateObservations<double>{});  // ignored
    e.process_truth(truth(4'000'000, {0, 0, 0}));
    CHECK(e.health().staleness_ns == 0);
}

// ── WaypointPlanner ─────────────────────────────────────────────────

TEST_CASE("waypoint: the path climbs, then passes pre, center, post of each gate, lap after lap", "[b0][planner]") {
    const CourseMap<double> m = a2rl();
    WaypointPlanner<double> p;
    const Vec3<double> start{3.1, 3.9, 0.1};
    const Reference<double> r = p.plan(m, at(start));
    REQUIRE(valid(r));
    REQUIRE(r.count == 1);
    REQUIRE(p.built());
    const double d = p.params().gate_approach;
    CHECK(p.waypoint_count() == 2 + 3 * std::uint64_t{m.sequence_length} * m.laps);
    CHECK(p.waypoint(0) == start);
    CHECK(p.waypoint(1) == Vec3<double>{start[0], start[1], gate(m, m.sequence[0]).center[2]});
    for (std::uint32_t lap = 0; lap < m.laps; ++lap) {
        for (std::uint32_t i = 0; i < m.sequence_length; ++i) {
            const Gate<double>& g = gate(m, m.sequence[i]);
            const Vec3<double> n = normal_of(g);
            const std::uint64_t k = 2 + 3 * (std::uint64_t{lap} * m.sequence_length + i);
            INFO("lap " << lap << " gate " << g.id);
            CHECK(p.waypoint(k + 1) == g.center);
            for (std::size_t a = 0; a < 3; ++a) {
                // The pre- and post-points lie on the normal through the
                // center, d either side (to rounding of one multiply-add).
                CHECK_THAT(p.waypoint(k)[a], WithinAbs(g.center[a] - d * n[a], 1e-12));
                CHECK_THAT(p.waypoint(k + 2)[a], WithinAbs(g.center[a] + d * n[a], 1e-12));
            }
        }
    }
}

TEST_CASE("waypoint: along a gate approach the reference points along the gate normal", "[b0][planner]") {
    const CourseMap<double> m = a2rl();
    for (const std::int32_t id : {1, 6, 11}) {  // along +x, +y, −y
        WaypointPlanner<double> p;
        (void)p.plan(m, at(m.start.position));
        const std::uint32_t i = static_cast<std::uint32_t>(id - 1);  // sequence is 1 … 11
        const std::uint64_t k = 2 + 3 * std::uint64_t{i};
        // A vehicle on the pre → center segment, 0.5 m past the pre-point,
        // after a drive along the path so the planner is on this segment.
        for (std::uint64_t j = 0; j <= k; ++j) {
            (void)p.plan(m, at(p.waypoint(j)));
        }
        const Gate<double>& g = gate(m, id);
        const Vec3<double> n = normal_of(g);
        const Vec3<double> pre = p.waypoint(k);
        const double s = 0.5;
        const Reference<double> r = p.plan(m, at({pre[0] + s * n[0], pre[1] + s * n[1], pre[2] + s * n[2]}));
        INFO("gate " << id << " segment " << p.segment());
        REQUIRE(p.segment() == k);
        const ReferenceSample<double>& x = r.samples[0];
        const double v =
            std::sqrt(x.velocity[0] * x.velocity[0] + x.velocity[1] * x.velocity[1] + x.velocity[2] * x.velocity[2]);
        // Cruise speed, or corner speed after a sharp turn onto the approach (gate 11).
        CHECK((std::abs(v - p.params().cruise_speed) < 1e-12 || std::abs(v - p.params().corner_speed) < 1e-12));
        for (std::size_t a = 0; a < 3; ++a) {
            CHECK_THAT(x.velocity[a], WithinAbs(v * n[a], 1e-12));
            CHECK_THAT(x.position[a], WithinAbs(pre[a] + (s + p.params().lookahead) * n[a], 1e-12));
        }
        CHECK_THAT(x.yaw, WithinAbs(std::atan2(n[1], n[0]), 1e-12));
    }
}

TEST_CASE("waypoint: a vehicle on the carrot advances monotonically and holds at the end", "[b0][planner]") {
    const CourseMap<double> m = a2rl();
    WaypointPlanner<double> p;
    Vec3<double> pos = m.start.position;
    std::uint64_t last_segment = 0;
    double last_progress = 0;
    std::int64_t t = 0;
    Reference<double> r{};
    // A follower that moves to each carrot: the path is 2 laps of ~150 m, the
    // carrot 0.3 m ahead, so the end is reached within 2·200/0.3 steps.
    for (int step = 0; step < 2000; ++step) {
        r = p.plan(m, at(pos, t));
        REQUIRE(valid(r));
        REQUIRE(p.segment() >= last_segment);           // I2
        REQUIRE(p.segment() + 1 < p.waypoint_count());  // I1
        REQUIRE(r.samples[0].progress >= last_progress);
        last_segment = p.segment();
        last_progress = r.samples[0].progress;
        pos = r.samples[0].position;
        t += 4'000'000;
    }
    CHECK(p.segment() == p.waypoint_count() - 2);  // the final segment
    const Vec3<double> end = p.waypoint(p.waypoint_count() - 1);
    CHECK(dist(pos, end) < 1e-9);
    CHECK(r.samples[0].velocity == Vec3<double>{});  // hold
    CHECK(r.samples[0].t == Timestamp{t - 4'000'000});
}

TEST_CASE("waypoint: corner speed near a sharp corner, cruise speed elsewhere", "[b0][planner]") {
    CourseMap<double> m{};
    m.gate_count = 1;
    m.gates[0] = Gate<double>{1, {10, 0, 2}, {}, 1.5, 1.5};  // along +x
    m.sequence_length = 1;
    m.sequence[0] = 1;
    m.laps = 1;
    m.geofence = Geofence<double>{{-5, -5, 0}, {30, 5, 5}};
    m.start = StartPose<double>{{0, 0, 0.1}, 0};
    REQUIRE(valid(m));
    WaypointPlanner<double> p;
    const auto speed = [&](Vec3<double> pos) {
        const Vec3<double> v = p.plan(m, at(pos)).samples[0].velocity;
        return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    };
    const auto& q = p.params();
    // Climb from the pad: the climb point turns 90° onto the gate line.
    CHECK_THAT(speed({0, 0, 0.1}), WithinAbs(q.cruise_speed, 1e-12));    // 1.9 m below the corner
    CHECK_THAT(speed({0, 0, 0.8}), WithinAbs(q.corner_speed, 1e-12));    // 1.2 m below: within 1.5 m
    CHECK_THAT(speed({4.0, 0, 2.0}), WithinAbs(q.cruise_speed, 1e-12));  // 4 m along, the pre-point straight ahead
    CHECK_THAT(speed({10.5, 0, 2.0}), WithinAbs(0.0, 1e-12));            // final segment: hold
}

TEST_CASE("waypoint: from_context reads and checks the baseline parameters", "[b0][planner]") {
    const auto vehicle = config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(vehicle);
    const CourseMap<double> m = a2rl();
    const auto make = [&](std::map<std::string, std::string> params) {
        const auto b = baseline_with(std::move(params));
        return WaypointPlanner<double>::from_context(SlotContext{*vehicle, b, m});
    };
    const auto p = make({{"cruise_speed_m_s", "3.5"}, {"lookahead_m", "0.2"}, {"corner_angle_deg", "90"}});
    REQUIRE(p);
    CHECK(p->params().cruise_speed == 3.5);
    CHECK(p->params().lookahead == 0.2);
    CHECK_THAT(p->params().corner_cos, WithinAbs(0.0, 1e-15));
    CHECK(p->params().gate_approach == WaypointPlanner<double>::Params{}.gate_approach);  // default
    CHECK_FALSE(make({{"cruise_speed_m_s", "fast"}}));
    CHECK_FALSE(make({{"cruise_speed_m_s", "-1"}}));
    CHECK_FALSE(make({{"corner_speed_m_s", "5"}}));                           // > cruise 4
    CHECK_FALSE(make({{"lookahead_m", "1.0"}, {"switch_radius_m", "1.0"}}));  // lookahead must be < switch
}

// ── SlotContext, Stack::create, on_truth, Runner ────────────────────

TEST_CASE("stack: create builds context slots and reports their errors", "[b0][stack]") {
    const auto vehicle = config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(vehicle);
    const CourseMap<double> m = a2rl();
    const auto good = baseline_with({{"cruise_speed_m_s", "3.0"}});
    auto s = GtStack<double>::create(m, SlotContext{*vehicle, good, m});
    REQUIRE(s);
    CHECK(s->planner().params().cruise_speed == 3.0);
    const auto bad = baseline_with({{"cruise_speed_m_s", "x"}});
    const auto f = GtStack<double>::create(m, SlotContext{*vehicle, bad, m});
    REQUIRE_FALSE(f);
    CHECK(f.error().find("cruise_speed_m_s") != std::string::npos);
}

TEST_CASE("stack: on_truth reaches the estimator, and the Runner feeds the truth channel", "[b0][stack]") {
    const auto vehicle = config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(vehicle);
    const CourseMap<double> m = a2rl();
    const auto b = baseline_with({});
    auto s = harness::make_stack<GtStack<double>>(m, *vehicle, b);
    REQUIRE(s);
    harness::Runner runner(*s);
    const TruthState<double> x = truth(2'000'000, {5, 4, 2}, {1, 0, 0});
    REQUIRE(runner.feed(log::LogRecord{log::Channel::truth, x.t, codec::encode(x)}));
    const auto out = runner.feed(log::LogRecord{log::Channel::tick, Timestamp{2'000'000}, {}});
    REQUIRE(out);
    REQUIRE(out->size() == 2);
    const auto state = codec::decode_state((*out)[0].payload);
    REQUIRE(state);
    CHECK(state->position == x.position);
    CHECK(state->velocity == x.velocity);
    CHECK(state->t_valid == x.t);

    const auto broken = runner.feed(log::LogRecord{log::Channel::truth, Timestamp{3'000'000}, {}});
    REQUIRE_FALSE(broken);
    CHECK(broken.error().find("undecodable truth") != std::string::npos);
}
