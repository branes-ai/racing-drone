#pragma once
/// @file time.hpp
/// @brief Timestamp: the one time type of the data contracts.
///
/// All contract timestamps are nanoseconds on the companion's monotonic clock.
/// FC-side samples (IMU) are mapped to that clock by the FcBridge before they
/// enter the stack. Integer nanoseconds keep replays bit-exact: no rounding
/// differences between platforms when times are compared or subtracted.

#include <compare>
#include <cstdint>

namespace branes::racing {

struct Timestamp {
    std::int64_t ns{0};

    friend constexpr auto operator<=>(const Timestamp&, const Timestamp&) = default;
};

/// Period in nanoseconds of a rate in hertz, rounded to the nearest ns.
/// Pre: hz > 0.
[[nodiscard]] constexpr std::int64_t period_ns(double hz) noexcept {
    return static_cast<std::int64_t>(1e9 / hz + 0.5);
}

/// Seconds as a double (for logging and arithmetic in the scalar type).
[[nodiscard]] constexpr double to_seconds(Timestamp t) noexcept {
    return static_cast<double>(t.ns) * 1e-9;
}

}  // namespace branes::racing
