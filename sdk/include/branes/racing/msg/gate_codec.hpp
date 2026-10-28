#pragma once
/// @file gate_codec.hpp
/// @brief Log payload encoding of GateObservations, the racing oracle_gates channel.
///
/// Racing-only (docs/assessments/framework-boundary.md); the mission-neutral
/// codecs and their helpers are in codec.hpp.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/msg/perception.hpp>

#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace branes::racing::codec {

// ── GateObservations (sim channel) ──────────────────────────────────

/// Encodes the `count` used entries only.
[[nodiscard]] inline std::vector<std::byte> encode(const GateObservations<double>& g) {
    ByteWriter w;
    w.u32(g.count);
    for (std::uint32_t i = 0; i < g.count && i < kMaxVisibleGates; ++i) {
        const GateObservation<double>& o = g.gates[i];
        w.i32(o.gate_id);
        for (const auto& c : o.corners) {
            w.f64(c[0]);
            w.f64(c[1]);
        }
        for (const double v : o.covariance) {
            w.f64(v);
        }
        w.f64(o.quality);
    }
    return w.take();
}

[[nodiscard]] inline std::optional<GateObservations<double>> decode_gates(std::span<const std::byte> b) {
    ByteReader r(b);
    GateObservations<double> g{};
    g.count = r.u32();
    if (g.count > kMaxVisibleGates) {
        return std::nullopt;
    }
    for (std::uint32_t i = 0; i < g.count; ++i) {
        GateObservation<double>& o = g.gates[i];
        o.gate_id = r.i32();
        for (auto& c : o.corners) {
            c[0] = r.f64();
            c[1] = r.f64();
            if (!std::isfinite(c[0]) || !std::isfinite(c[1])) {
                return std::nullopt;
            }
        }
        for (double& v : o.covariance) {
            v = r.f64();
        }
        o.quality = r.f64();
        if (!detail::finite(o.covariance) || !std::isfinite(o.quality)) {
            return std::nullopt;
        }
    }
    if (!r.done() || !valid(g)) {
        return std::nullopt;
    }
    return g;
}

}  // namespace branes::racing::codec
