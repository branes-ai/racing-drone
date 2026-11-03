#pragma once
/// @file bindings.hpp
/// @brief The bindings of racing's three task composites, the estimator, the
/// gate detector and the planner: the CPU calling convention of their
/// operators (anatomy docs/adr/0004, step 3; docs/plan/flow-graph/).
///
/// Racing-only (docs/assessments/framework-boundary.md). One binding per task
/// slot concept of slots/slots.hpp, as anatomy's are for the mission-neutral
/// ones (mobility/flow/bindings.hpp). Each handler calls the method the concept
/// names, so every implementation of the slot is an operator of the composite,
/// unchanged. Every handler of a composite must fire (G6), so a method no graph
/// reads has none: the estimator's health().
///
/// Two handlers keep a branch of Stack (compose/stack.hpp), so the graph needs
/// none:
///  - the estimator's truth handler hands the true state to an estimator that
///    takes it (TruthConsumer), as Stack::on_truth() does, and is a no-op for
///    one that doesn't. So the graph reads the truth, and is SITL-only,
///    whichever estimator fills it;
///  - the gate detector's detect handler sees no gates in a corrupted frame,
///    without calling the detector, as Stack::on_frame() does.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/anatomy_mobility.hpp>
#include <branes/racing/flow/domains.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/perception.hpp>

namespace branes::racing::graph {

namespace fl = racing::flow;

/// Estimator: imu(imu), frame(camera, gates) and truth(truth) update its
/// state; estimate at a time reads it.
template <class T>
struct EstimatorBinding {
    using signature = fl::signature<fl::ports<fl::port<"imu", ImuSample<T>, fl::direction::in>,
                                              fl::port<"camera", CameraFrame, fl::direction::in>,
                                              fl::port<"gates", GateObservations<T>, fl::direction::in>,
                                              fl::port<"truth", TruthState<T>, fl::direction::in>,
                                              fl::port<"estimate", StateEstimate<T>, fl::direction::out>>>;

    struct imu : fl::handler<"imu", fl::ins<"imu">, fl::outs<>, fl::state_role::update> {
        static void call(auto& est, const ImuSample<T>& s) {
            est.process_imu(s);
        }
    };
    struct frame : fl::handler<"frame", fl::ins<"camera", "gates">, fl::outs<>, fl::state_role::update> {
        static void call(auto& est, const CameraFrame& f, const GateObservations<T>& gates) {
            est.process_frame(f, gates);
        }
    };
    struct truth : fl::handler<"truth", fl::ins<"truth">, fl::outs<>, fl::state_role::update> {
        template <class E>
        static void call(E& est, const TruthState<T>& x) {
            if constexpr (TruthConsumer<E, T>) {
                est.process_truth(x);
            } else {
                (void)est;
                (void)x;
            }
        }
    };
    struct estimate
        : fl::handler<"estimate", fl::ins<>, fl::outs<"estimate">, fl::state_role::read, fl::takes_time::yes> {
        static StateEstimate<T> call(const auto& est, Timestamp t) {
            return est.state_at(t);
        }
    };

    using handlers = fl::handlers<imu, frame, truth, estimate>;
};

/// Gate detector: detect(camera, course, prior) → gates. A detector may track
/// the gates it saw, so detect updates its state.
template <class T>
struct GateDetectorBinding {
    using signature = fl::signature<fl::ports<fl::port<"camera", CameraFrame, fl::direction::in>,
                                              fl::port<"course", CourseMap<T>, fl::direction::in>,
                                              fl::port<"prior", StateEstimate<T>, fl::direction::in>,
                                              fl::port<"gates", GateObservations<T>, fl::direction::out>>>;

    struct detect
        : fl::handler<"detect", fl::ins<"camera", "course", "prior">, fl::outs<"gates">, fl::state_role::update> {
        static GateObservations<T>
        call(auto& det, const CameraFrame& f, const CourseMap<T>& course, const StateEstimate<T>& prior) {
            return f.header.corrupted ? GateObservations<T>{} : det.detect(f, course, prior);
        }
    };

    using handlers = fl::handlers<detect>;
};

/// Planner: plan(course, estimate) → reference. A planner may lay out its path
/// on the first plan() (WaypointPlanner), so plan updates its state.
template <class T>
struct PlannerBinding {
    using signature = fl::signature<fl::ports<fl::port<"course", CourseMap<T>, fl::direction::in>,
                                              fl::port<"estimate", StateEstimate<T>, fl::direction::in>,
                                              fl::port<"reference", Reference<T>, fl::direction::out>>>;

    struct plan : fl::handler<"plan", fl::ins<"course", "estimate">, fl::outs<"reference">, fl::state_role::update> {
        static Reference<T> call(auto& p, const CourseMap<T>& course, const StateEstimate<T>& x) {
            return p.plan(course, x);
        }
    };

    using handlers = fl::handlers<plan>;
};

}  // namespace branes::racing::graph
