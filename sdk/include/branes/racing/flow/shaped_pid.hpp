#pragma once
/// @file shaped_pid.hpp
/// @brief A body of several operators and a research graph, from SetpointShaper
/// and CascadedPid (slots/shaped_pid.hpp; anatomy docs/plan/flow-testing, T5).
///
/// Racing-only (docs/assessments/framework-boundary.md).
///  - ShapedPidBody fills the controller composite of a scoreboard graph: the
///    shaper shapes the reference, and CascadedPid steps on it. Baseline
///    b0-shaped-pid's controller.
///  - ShapedPidResearch is a research graph: B0's operators and the two of
///    the body, each an instance of its own, between racing's external ports,
///    in the regions and the order of the scoreboard graph (flow/graph.hpp).
///    Baseline b0-shaped-research.
/// Each runs the operators in the order ShapedPid, the slot controller wired
/// by hand, calls them, so a graph build's log is its slot build's.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/anatomy_glue.hpp>
#include <branes/racing/anatomy_mobility.hpp>
#include <branes/racing/flow/bindings.hpp>
#include <branes/racing/flow/graph.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/slots/ground_truth.hpp>
#include <branes/racing/slots/null.hpp>
#include <branes/racing/slots/shaped_pid.hpp>
#include <branes/racing/slots/waypoint.hpp>

#include <string_view>
#include <tuple>

namespace branes::racing::graph {

/// SetpointShaper: shape(reference) → reference, with no state.
template <class T>
struct ShaperBinding {
    using signature = fl::signature<fl::ports<fl::port<"reference", Reference<T>, fl::direction::in>,
                                              fl::port<"shaped", Reference<T>, fl::direction::out>>>;

    struct shape : fl::handler<"shape", fl::ins<"reference">, fl::outs<"shaped">> {
        static Reference<T> call(const auto& s, const Reference<T>& ref) {
            return s.shape(ref);
        }
    };

    using handlers = fl::handlers<shape>;
};

template <class T>
using SetpointShaperOp = fl::registered<"SetpointShaper", SetpointShaper<T>, ShaperBinding<T>>;
template <class T>
using CascadedPidOp = fl::registered<"CascadedPid", glue::CascadedPid<T>, fl::ControllerBinding<T>>;

/// The controller's body: the shaper, then CascadedPid on the shaped reference.
template <class T>
struct ShapedPidBody {
    static constexpr std::string_view name = "ShapedPid";
    using binding = fl::ControllerBinding<T>;

    template <class G>
    static void build(fl::body<binding, G>& b) {
        const auto shaper = fl::instance<SetpointShaperOp<T>>(b, "shaper");
        const auto pid = fl::instance<CascadedPidOp<T>>(b, "pid");
        fl::on<"step">(b, [=](fl::scope& s, auto estimate, auto reference, auto conflicts) {
            auto [shaped] = fl::fire<"shape">(s, shaper, reference);
            auto [command] = fl::fire<"step">(s, pid, estimate, shaped, conflicts);
            return std::tuple{command};
        });
    }
};

/// The research graph: B0's operators, with the shaper before CascadedPid.
template <class T>
struct ShapedPidResearch {
    static constexpr std::string_view name = "ShapedPidResearch";

    template <class G>
    static Ports<T> build(G& g, const CourseMap<T>& course) {
        const Ports<T> p = ports<T>(g, course);
        using Estimator = fl::registered<"GroundTruthEstimator", GroundTruthEstimator<T>, EstimatorBinding<T>>;
        using Detector = fl::registered<"NullGateDetector", NullGateDetector<T>, GateDetectorBinding<T>>;
        using Planner = fl::registered<"WaypointPlanner", WaypointPlanner<T>, PlannerBinding<T>>;
        using Safety = fl::registered<"NullSafetyFilter", NullSafetyFilter<T>, fl::SafetyFilterBinding<T>>;
        using Daa = fl::registered<"NullDaaTracker", NullDaaTracker<T>, fl::DaaTrackerBinding<T>>;
        using Fc = fl::registered<"NullFcBridge", NullFcBridge<T>, fl::FcBridgeBinding<T>>;
        const auto est = fl::instance<Estimator>(g, "estimator");
        const auto det = fl::instance<Detector>(g, "gate_detector");
        const auto planner = fl::instance<Planner>(g, "planner");
        const auto shaper = fl::instance<SetpointShaperOp<T>>(g, "shaper");
        const auto pid = fl::instance<CascadedPidOp<T>>(g, "pid");
        const auto safe = fl::instance<Safety>(g, "safety_filter");
        const auto daa = fl::instance<Daa>(g, "daa_tracker");
        const auto fc = fl::instance<Fc>(g, "fc_bridge");

        fl::scope truth = g.region(p.truth);
        fl::fire<"truth">(truth, est, p.truth);

        fl::scope imu = g.region(p.imu);
        fl::fire<"imu">(imu, est, p.imu);

        fl::scope frame = g.region(p.camera);
        auto [prior] = fl::fire<"estimate">(frame, est, fl::at(p.camera));
        auto [gates] = fl::fire<"detect">(frame, det, p.camera, p.course, prior);
        fl::fire<"frame">(frame, est, p.camera, gates);
        fl::fire<"update">(frame, daa, p.detections, prior);

        fl::scope tick = g.region(p.tick);
        auto [x] = fl::fire<"estimate">(tick, est, fl::at(p.tick));
        auto [reference] = fl::fire<"plan">(tick, planner, p.course, x);
        auto [conflicts] = fl::fire<"conflicts">(tick, daa, x);
        auto [shaped] = fl::fire<"shape">(tick, shaper, reference);
        auto [command] = fl::fire<"step">(tick, pid, x, shaped, conflicts);
        auto [safe_command] = fl::fire<"filter">(
            tick, safe, x, command, conflicts, fl::project<"geofence", &CourseMap<T>::geofence>(p.course));
        fl::fire<"send">(tick, fc, fl::at(p.tick), safe_command);
        g.produce(p.estimate, x);
        g.produce(p.reference, reference);
        g.produce(p.command, safe_command);
        return p;
    }
};

}  // namespace branes::racing::graph
