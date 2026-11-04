#pragma once
/// @file graph.hpp
/// @brief Racing's stack as a graph: its ports, its composites and its regions
/// (anatomy docs/adr/0004, step 3; docs/plan/flow-graph/), and graph::Stack,
/// the CPU stack that runs it.
///
/// Racing-only (docs/assessments/framework-boundary.md). The graph is the slot
/// stack (compose/stack.hpp) written down: one region per input, its firings
/// in Stack's order, so a baseline's graph build and its slot build write the
/// same records.
///  - truth: the estimator takes the true state, if it reads it (SITL only);
///  - imu: the estimator takes the sample;
///  - frame: the estimate at the frame's mid-exposure is read; the gate
///    detector finds the gates in the frame with that prior, and none in a
///    corrupted frame; the estimator takes the frame and the gates; and the
///    DAA tracker is updated with the prior and the intruder detections, of
///    which there are none yet;
///  - tick: estimate, plan, conflicts, step, filter and send, at the tick's
///    time, with the course's geofence for the safety filter.
///
/// Stack::on_truth() hands the truth to every slot that reads it; the graph
/// hands it to the estimator only, the one slot of racing's that reads it
/// (GroundTruthEstimator).
///
/// A scoreboard baseline's file names the body of each composite (`graph:`);
/// a research baseline's names its graph (flow/shaped_pid.hpp), which places
/// operators of its own between these ports. cmake/baselines.cmake configures
/// the header that names their types.
///
/// trace() writes every edge a stack's firings write to a trace file
/// (anatomy flow/trace.hpp), with racing's codecs: `--trace` on the SITL
/// binary of a graph build.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/anatomy_mobility.hpp>
#include <branes/racing/flow/bindings.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/perception.hpp>
#include <branes/racing/slots/context.hpp>

#include <cassert>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

namespace branes::racing::graph {

/// The graph's external ports.
template <class T>
struct Ports {
    fl::input_port<ImuSample<T>> imu;
    fl::input_port<CameraFrame> camera;
    fl::input_port<TruthState<T>> truth;
    fl::input_port<Timestamp> tick;
    fl::constant_port<CourseMap<T>> course;
    fl::constant_port<IntruderDetections<T>> detections;  ///< no producer yet: none
    fl::output_port<StateEstimate<T>> estimate;
    fl::output_port<Reference<T>> reference;
    fl::output_port<ThrustRates<T>> command;
};

template <class T>
[[nodiscard]] Ports<T> ports(fl::builder& g, const CourseMap<T>& course) {
    Ports<T> p;
    p.imu = fl::input<ImuSample<T>>(g, "imu");
    p.camera = fl::input<CameraFrame>(
        g,
        "camera",
        {.trigger = "frame", .time = [](const CameraFrame& f) { return f.header.t_mid; }, .sitl_only = false});
    p.truth = fl::input<TruthState<T>>(g, "truth", {.trigger = "truth", .time = nullptr, .sitl_only = true});
    p.tick = g.tick("tick");
    p.course = fl::constant<CourseMap<T>>(g, "course", course);
    p.detections = fl::constant<IntruderDetections<T>>(g, "detections");
    p.estimate = fl::output<StateEstimate<T>>(g, "estimate", p.tick);
    p.reference = fl::output<Reference<T>>(g, "reference", p.tick);
    p.command = fl::output<ThrustRates<T>>(g, "command", p.tick);
    return p;
}

/// The scoreboard graph: racing's seven composites, filled by Bodies.
template <class T, class Bodies, class G>
Ports<T> scoreboard(G& g, const CourseMap<T>& course) {
    const Ports<T> p = ports<T>(g, course);
    const auto est = fl::composite<EstimatorBinding<T>, typename Bodies::estimator>(g, "estimator");
    const auto det = fl::composite<GateDetectorBinding<T>, typename Bodies::gate_detector>(g, "gate_detector");
    const auto planner = fl::composite<PlannerBinding<T>, typename Bodies::planner>(g, "planner");
    const auto ctl = fl::composite<fl::ControllerBinding<T>, typename Bodies::controller>(g, "controller");
    const auto safe = fl::composite<fl::SafetyFilterBinding<T>, typename Bodies::safety_filter>(g, "safety_filter");
    const auto daa = fl::composite<fl::DaaTrackerBinding<T>, typename Bodies::daa_tracker>(g, "daa_tracker");
    const auto fc = fl::composite<fl::FcBridgeBinding<T>, typename Bodies::fc_bridge>(g, "fc_bridge");

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
    auto [command] = fl::fire<"step">(tick, ctl, x, reference, conflicts);
    auto [safe_command] = fl::fire<"filter">(
        tick, safe, x, command, conflicts, fl::project<"geofence", &CourseMap<T>::geofence>(p.course));
    fl::fire<"send">(tick, fc, fl::at(p.tick), safe_command);
    g.produce(p.estimate, x);
    g.produce(p.reference, reference);
    g.produce(p.command, safe_command);
    return p;
}

/// A baseline's stack: the CPU stack of its graph, which the harnesses drive
/// as they drive the slot stack (RunnableStack, with MobilityProfile). Each
/// hook hands the stack its input, which runs that input's region; the tick
/// returns the tick region's outputs. `Baseline` names what fills the graph
/// (cmake/racing_graph.hpp.in): a scoreboard baseline its bodies, a research
/// baseline its graph, `research`, whose build() returns the ports.
///
/// Model
/// -----
///   Stack<T, Baseline>
///     stack_     : flow::stack [owned] — the operators, the values, the firings
///     ports_     : Ports<T> — the handles of the graph's external ports
///     last_tick_ : std::optional<Timestamp> — time of the previous on_tick
///
/// Invariants
/// ----------
///   I1: stack_ runs the checked graph of Baseline's file   — constrains stack_
///   I2: ticks arrive in non-decreasing time order          — constrains last_tick_
template <class T, class Baseline>
class Stack {
public:
    using scalar_type = T;
    using profile = MobilityProfile;  ///< the slot stack's (compose/stack.hpp)

    /// Build the graph and its operators from `ctx` (make_slot), and check the
    /// baseline file against the graph (G8); the first failure is returned.
    /// Pre: valid(course). Post: I1, I2 (no tick yet).
    [[nodiscard]] static result<Stack, std::string> create(const CourseMap<T>& course, const SlotContext& ctx) {
        assert(valid(course));  // Pre
        fl::stack_builder<SlotContext> g{"racing", Baseline::kind, ctx};
        const Ports<T> p = [&] {
            if constexpr (Baseline::kind == fl::baseline_class::scoreboard) {
                return scoreboard<T, typename Baseline::bodies>(g, course);
            } else {
                return Baseline::research::build(g, course);
            }
        }();
        auto built = std::move(g).build_stack();
        if (!built) {
            return built.error().to_string();
        }
        if (const auto c = fl::check_baseline(ctx.baseline, Baseline::name, built->graph()); !c) {
            return c.error().to_string();
        }
        return Stack(std::move(built).value(), p);
    }

    /// Pre: valid(x).
    void on_truth(const TruthState<T>& x) {
        assert(valid(x));  // Pre
        stack_.input(ports_.truth, x);
    }

    /// Pre: valid(s).
    void on_imu(const ImuSample<T>& s) {
        assert(valid(s));  // Pre
        stack_.input(ports_.imu, s);
    }

    /// Pre: valid(f).
    void on_frame(const CameraFrame& f) {
        assert(valid(f));  // Pre
        stack_.input(ports_.camera, f);
    }

    /// Run the tick region at `now`. Pre: now ≥ the previous tick (I2).
    /// Post: I2 with last_tick_ == now.
    TickOutput<T> on_tick(Timestamp now) {
        assert(!last_tick_ || *last_tick_ <= now);  // Pre / I2
        stack_.input(ports_.tick, now);
        last_tick_ = now;
        return TickOutput<T>{
            stack_.output(ports_.estimate), stack_.output(ports_.reference), stack_.output(ports_.command)};
    }

    /// SHA-256 of the checked graph's canonical form, which the log records as
    /// its composition_hash.
    [[nodiscard]] const std::string& hash() const noexcept {
        return stack_.hash();
    }

    [[nodiscard]] const fl::stack& stack() const noexcept {
        return stack_;
    }

    /// Hand every firing to `o` from now on (anatomy flow::stack::observe).
    void observe(fl::observer o) {
        stack_.observe(std::move(o));
    }

private:
    Stack(fl::stack s, Ports<T> p) : stack_(std::move(s)), ports_(std::move(p)) {}

    fl::stack stack_;
    Ports<T> ports_;
    std::optional<Timestamp> last_tick_;
};

/// The codecs a trace writes racing's messages with: the log's (msg/codec).
/// The gates and the conflicts have none, so their edges are traced without
/// their values.
[[nodiscard]] inline fl::trace_codecs trace_codecs() {
    fl::trace_codecs c;
    c.add<StateEstimate<double>>([](const StateEstimate<double>& x) { return codec::encode(x); });
    c.add<Reference<double>>([](const Reference<double>& r) { return codec::encode(r); });
    c.add<ThrustRates<double>>([](const ThrustRates<double>& u) { return codec::encode(u); });
    return c;
}

/// Write every edge `stack` writes from now on to a trace at `path`, which is
/// created (truncated). The writer is the stack's observer: it must outlive
/// the stack's runs, and its close() reports the first write error.
template <class T, class Baseline>
[[nodiscard]] result<std::unique_ptr<fl::trace_writer>, std::string> trace(Stack<T, Baseline>& stack,
                                                                           const std::filesystem::path& path) {
    static_assert(std::is_same_v<T, double>, "racing's codecs are of double messages");
    auto opened = fl::trace_writer::open(path, stack.stack().graph(), trace_codecs());
    if (!opened) {
        return opened.error().path + ": " + opened.error().message;
    }
    auto w = std::make_unique<fl::trace_writer>(std::move(opened).value());
    stack.observe([p = w.get()](const fl::fired& f) { p->write(f); });
    return w;
}

}  // namespace branes::racing::graph
