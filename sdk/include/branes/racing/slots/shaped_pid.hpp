#pragma once
/// @file shaped_pid.hpp
/// @brief SetpointShaper, an operator that shapes the planner's reference, and
/// ShapedPid, the controller that runs it before CascadedPid, wired by hand
/// (anatomy docs/plan/flow-testing, T5).
///
/// Baseline b0-shaped-pid's controller is a body of these two operators
/// (flow/shaped_pid.hpp), and b0-shaped-research places them in a research
/// graph. Each baseline's slot build fills the controller with ShapedPid,
/// which calls the same two operators in the order the body fires them, so the
/// slot build is the graph builds' oracle: their logs must be equal.
///
/// SetpointShaper scales every sample's velocity and acceleration feed-forward
/// by the baseline parameter feedforward_gain ∈ [0, 1] (default 1, which
/// changes nothing), and leaves the position, the yaw, the progress and the
/// times as they are. It holds no state, so the reference it returns depends
/// only on the reference it is given.
///
/// Model
/// -----
///   SetpointShaper<T>
///     gain_ : T [immutable] — the feed-forward gain
///   ShapedPid<T>
///     shaper_ : SetpointShaper<T> [owned]
///     pid_    : glue::CascadedPid<T> [owned]
///
/// Invariants
/// ----------
///   SetpointShaper I1: 0 ≤ gain_ ≤ 1              — constrains gain_

#include <branes/racing/anatomy_glue.hpp>
#include <branes/racing/anatomy_mobility.hpp>
#include <branes/racing/slots/context.hpp>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

namespace branes::racing {

template <typename T>
class SetpointShaper {
public:
    /// Pre: 0 ≤ gain ≤ 1. Post: I1.
    explicit SetpointShaper(T gain = T(1)) : gain_(gain) {
        assert(gain_ >= T(0) && gain_ <= T(1));  // Pre / I1
    }

    /// From the baseline parameter feedforward_gain (default 1). Fails if it is
    /// outside [0, 1].
    [[nodiscard]] static result<SetpointShaper, std::string> from_context(const SlotContext& ctx) {
        const auto gain = parameter_in(ctx.baseline, "feedforward_gain", 1.0, 0.0, 1.0);
        if (!gain) {
            return "SetpointShaper: " + gain.error();
        }
        return SetpointShaper(T(*gain));
    }

    /// `ref` with each sample's velocity and acceleration scaled by the gain.
    /// Pre: valid(ref). Post: valid(result); result.count == ref.count, and
    /// only the velocity and acceleration of its samples differ from ref's.
    [[nodiscard]] Reference<T> shape(const Reference<T>& ref) const {
        assert(valid(ref));  // Pre
        Reference<T> out = ref;
        for (std::uint32_t i = 0; i < out.count; ++i) {
            ReferenceSample<T>& s = out.samples[i];
            for (std::size_t k = 0; k < 3; ++k) {
                s.velocity[k] = gain_ * s.velocity[k];
                s.acceleration[k] = gain_ * s.acceleration[k];
            }
        }
        return out;
    }

    [[nodiscard]] T gain() const noexcept {
        return gain_;
    }

private:
    T gain_;
};

/// The controller of b0-shaped-pid's slot build: SetpointShaper, then
/// CascadedPid on the shaped reference.
template <typename T>
class ShapedPid {
public:
    ShapedPid(SetpointShaper<T> shaper, glue::CascadedPid<T> pid) : shaper_(std::move(shaper)), pid_(std::move(pid)) {}

    /// Each operator from the context, as the graph builds it.
    [[nodiscard]] static result<ShapedPid, std::string> from_context(const SlotContext& ctx) {
        auto shaper = SetpointShaper<T>::from_context(ctx);
        if (!shaper) {
            return shaper.error();
        }
        auto pid = glue::CascadedPid<T>::from_context(ctx);
        if (!pid) {
            return pid.error();
        }
        return ShapedPid(std::move(shaper).value(), std::move(pid).value());
    }

    /// CascadedPid's step on the shaped reference. Pre: CascadedPid::step's.
    [[nodiscard]] ThrustRates<T> step(const StateEstimate<T>& x, const Reference<T>& ref, const ConflictSet<T>& c) {
        return pid_.step(x, shaper_.shape(ref), c);
    }

private:
    SetpointShaper<T> shaper_;
    glue::CascadedPid<T> pid_;
};

}  // namespace branes::racing
