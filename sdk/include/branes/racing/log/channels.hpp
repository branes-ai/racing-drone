#pragma once
/// @file channels.hpp
/// @brief Racing's own log channels, beside the framework's (log.hpp).
///
/// Racing-only (docs/assessments/framework-boundary.md). The id is the one
/// racing logs have always used, so existing logs read unchanged.

#include <branes/racing/anatomy.hpp>

#include <array>

namespace branes::racing::log {

/// GateObservations projected from the truth (SITL), a sim channel.
inline constexpr Channel oracle_gates = static_cast<Channel>(5);

/// The channels racing adds to the framework's, for LogReader::open.
inline constexpr std::array<Channel, 1> kRacingChannels = {oracle_gates};

static_assert(!is_framework(static_cast<std::uint16_t>(oracle_gates)), "racing channels are not framework channels");
static_assert(!is_output(oracle_gates), "oracle_gates is an input (sim) channel");

}  // namespace branes::racing::log
