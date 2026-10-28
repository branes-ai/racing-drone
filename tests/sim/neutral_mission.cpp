// The mission-neutral framework pieces, driven by a mission that is not
// racing (docs/plan/framework-split). This file includes only headers listed
// in cmake/framework_boundary.cmake: a world without a course, a stack without
// racing's slots, and a mission channel of its own.

#include <branes/racing/compose/tick.hpp>
#include <branes/racing/config/baseline.hpp>
#include <branes/racing/config/vehicle.hpp>
#include <branes/racing/glue/cascaded_pid.hpp>
#include <branes/racing/harness/runner.hpp>
#include <branes/racing/log/log.hpp>
#include <branes/racing/msg/region.hpp>
#include <branes/racing/slots/ground_truth_core.hpp>
#include <branes/racing/slots/neutral.hpp>
#include <branes/racing/slots/neutral_nulls.hpp>
#include <branes/racing/slots/slot_context.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <filesystem>
#include <string>

using namespace branes::racing;

namespace {

const std::filesystem::path src = RACING_SOURCE_DIR;

/// A scouting mission's world: a goal and a start pose, no gates.
struct Field {
    StartPose<double> start{{0.0, 0.0, 0.1}, 0.0};
    Vec3<double> goal{40.0, 0.0, 2.0};
};
static_assert(MissionWorld<Field>);

/// A depth-image channel of that mission (unused by the framework).
constexpr log::Channel kDepth = static_cast<log::Channel>(7);

/// A stack of the mission: the truth as the estimate, a fixed reference, and
/// the neutral null controller. It satisfies RunnableStack.
class FieldStack {
public:
    using scalar_type = double;

    void on_imu(const ImuSample<double>& s) {
        est_.process_imu(s);
    }
    void on_frame(const CameraFrame&) {}
    void on_truth(const TruthState<double>& x) {
        est_.process_truth(x);
    }
    TickOutput<double> on_tick(Timestamp now) {
        const StateEstimate<double> x = est_.state_at(now);
        Reference<double> r{};
        r.count = 1;
        r.samples[0].t = now;
        r.samples[0].position = Vec3<double>{40.0, 0.0, 2.0};
        return TickOutput<double>{x, r, ctl_.step(x, r, ConflictSet<double>{})};
    }

private:
    GroundTruthCore<double> est_;
    NullController<double> ctl_;
};
static_assert(RunnableStack<FieldStack>);

}  // namespace

TEST_CASE("neutral: the runner drives another mission's stack and passes its channel through", "[neutral]") {
    FieldStack stack;
    const std::array<log::Channel, 1> mission = {kDepth};
    harness::Runner runner(stack, mission);

    const auto tick = runner.feed(log::LogRecord{log::Channel::tick, Timestamp{1'000'000}, {}});
    REQUIRE(tick);
    CHECK(tick->size() == 3u);

    const auto depth = runner.feed(log::LogRecord{kDepth, Timestamp{2'000'000}, {}});
    REQUIRE(depth);
    CHECK(depth->empty());  // recorded, not consumed

    // Racing's oracle channel is not this mission's: it is not an input here.
    const auto racing = runner.feed(log::LogRecord{static_cast<log::Channel>(5), Timestamp{3'000'000}, {}});
    REQUIRE_FALSE(racing);
    CHECK(racing.error() == "not an input channel");
}

TEST_CASE("neutral: the cascaded PID builds from another mission's slot context", "[neutral]") {
    const auto vehicle = config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(vehicle);
    const config::BaselineConfig baseline{"field", {}, {}};
    const Field field{};
    const BasicSlotContext<Field> ctx{*vehicle, baseline, field};
    static_assert(ContextBuilt<glue::CascadedPid<double>, BasicSlotContext<Field>>);
    const auto pid = make_slot<glue::CascadedPid<double>>(ctx);
    REQUIRE(pid);
}
