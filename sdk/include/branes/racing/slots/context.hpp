#pragma once
/// @file context.hpp
/// @brief SlotContext: what a slot may be built from, and the optional slot hooks.
///
/// A slot type with a factory
///
///   static result<S, std::string> from_context(const SlotContext&)
///
/// is built from the context by Stack::create(); any other slot is
/// default-constructed. The choice is made per slot at compile time, so the
/// null slots stay as they are. The context holds load-time configs (double):
/// the vehicle file, the baseline file (its `parameters:` block), and the course.
///
/// A slot that declares `process_truth(const TruthState<T>&)` (TruthConsumer)
/// receives the simulated true state from Stack::on_truth(). Only SITL logs
/// carry truth (SYS-02), so in flight such a slot never receives any.

#include <branes/racing/config/config.hpp>
#include <branes/racing/msg/course.hpp>
#include <branes/racing/msg/truth.hpp>
#include <branes/racing/result.hpp>

#include <concepts>
#include <string>
#include <string_view>

namespace branes::racing {

/// The configs a slot may be built from. Non-owning: valid for the duration of
/// the from_context() call only.
struct SlotContext {
    const config::VehicleConfig& vehicle;
    const config::BaselineConfig& baseline;
    const CourseMap<double>& course;
};

/// A slot type built by `S::from_context(ctx)`.
template <class S>
concept ContextBuilt = requires(const SlotContext& ctx) {
    { S::from_context(ctx) } -> std::same_as<result<S, std::string>>;
};

/// A slot that reads the simulated true state (SITL only).
template <class S, class T>
concept TruthConsumer = requires(S s, const TruthState<T>& x) { s.process_truth(x); };

/// Build slot S from `ctx` if it is ContextBuilt, else default-construct it.
template <class S>
[[nodiscard]] result<S, std::string> make_slot(const SlotContext& ctx) {
    if constexpr (ContextBuilt<S>) {
        return S::from_context(ctx);
    } else {
        (void)ctx;
        return S{};
    }
}

/// A baseline parameter as a number: `fallback` if absent, an error if present
/// but not a number or outside [lo, hi].
[[nodiscard]] inline result<double, std::string>
parameter_in(const config::BaselineConfig& b, std::string_view key, double fallback, double lo, double hi) {
    double v = fallback;
    if (b.parameters.count(std::string(key)) != 0) {
        const auto n = config::numeric_parameter(b, key);
        if (!n) {
            return std::string("baseline parameter " + std::string(key) + " is not a number");
        }
        v = *n;
    }
    if (!(v >= lo && v <= hi)) {
        return std::string("baseline parameter " + std::string(key) + " must be in [" + std::to_string(lo) + ", " +
                           std::to_string(hi) + "]");
    }
    return v;
}

}  // namespace branes::racing
