// Unit tests for racing's log payload codec (msg/gate_codec.hpp): the gate
// observations round-trip exactly, and malformed payloads are rejected. The
// framework's codecs are tested in anatomy.

#include <branes/racing/anatomy.hpp>
#include <branes/racing/msg/gate_codec.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

using namespace branes::racing;

TEST_CASE("codec: GateObservations round trip", "[codec]") {
    GateObservations<double> g{};
    g.count = 2;
    for (std::uint32_t i = 0; i < 2; ++i) {
        g.gates[i].gate_id = static_cast<std::int32_t>(7 + i);
        g.gates[i].quality = 1;
        g.gates[i].corners[2] = {410.5, 300.25 + i};
        g.gates[i].covariance[9] = 1.0;
    }
    const auto gb = codec::decode_gates(codec::encode(g));
    REQUIRE(gb);
    REQUIRE(*gb == g);
    REQUIRE(codec::encode(GateObservations<double>{}).size() == 4);  // only the count when empty
}

TEST_CASE("codec: invalid gate records are rejected", "[codec]") {
    GateObservations<double> g{};
    g.count = 1;
    g.gates[0].quality = 2;  // I2: outside [0, 1]
    REQUIRE_FALSE(codec::decode_gates(codec::encode(g)));
    g.gates[0].quality = 1;
    g.gates[0].corners[0][1] = std::numeric_limits<double>::infinity();
    REQUIRE_FALSE(codec::decode_gates(codec::encode(g)));
    ByteWriter w;
    w.u32(kMaxVisibleGates + 1);  // too many gates
    REQUIRE_FALSE(codec::decode_gates(w.take()));
}
