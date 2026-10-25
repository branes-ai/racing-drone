// Unit tests for the log payload codec (msg/codec.hpp) and the byte encoding
// beneath it (util/bytes.hpp): round trips are exact, and malformed payloads
// are rejected rather than half-decoded.

#include <branes/racing/msg/codec.hpp>
#include <branes/racing/util/bytes.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

using namespace branes::racing;

TEST_CASE("ByteWriter writes little-endian; ByteReader reads it back", "[codec]") {
    ByteWriter w;
    w.u16(0x0102);
    w.i64(-2);
    w.f64(-0.0);
    w.str("gate");
    const auto& b = w.data();
    REQUIRE(b.size() == 2 + 8 + 8 + 4 + 4);
    REQUIRE(b[0] == std::byte{0x02});  // little-endian
    REQUIRE(b[1] == std::byte{0x01});

    ByteReader r(b);
    REQUIRE(r.u16() == 0x0102);
    REQUIRE(r.i64() == -2);
    const double z = r.f64();
    REQUIRE(z == 0.0);
    REQUIRE(std::signbit(z));  // the bit pattern survives, sign included
    REQUIRE(r.str() == "gate");
    REQUIRE(r.done());
}

TEST_CASE("ByteReader: reading past the end fails instead of overrunning", "[codec]") {
    const std::vector<std::byte> b(3);
    ByteReader r(b);
    REQUIRE(r.u32() == 0u);
    REQUIRE(r.failed());
    REQUIRE_FALSE(r.done());
}

TEST_CASE("codec: ImuSample round trip", "[codec]") {
    ImuSample<double> s{};
    s.t = Timestamp{123456789};
    s.gyro = {0.1, -0.2, 0.3};
    s.accel = {1.0, 2.0, 9.80665};
    s.saturated = accel_z_saturated;
    s.temperature = 31.5;
    const auto bytes = codec::encode(s);
    const auto back = codec::decode_imu(bytes);
    REQUIRE(back);
    REQUIRE(*back == s);
    REQUIRE(codec::encode(*back) == bytes);  // equal values, equal bytes
}

TEST_CASE("codec: StateEstimate and ThrustRates round trip", "[codec]") {
    StateEstimate<double> x{};
    x.t_valid = Timestamp{-5};
    x.position = {1.0, 2.0, 3.0};
    x.attitude = Quat<double>{0.5, -0.5, 0.5, -0.5};
    x.covariance[0] = 0.25;
    const auto xb = codec::decode_state(codec::encode(x));
    REQUIRE(xb);
    REQUIRE(*xb == x);

    ThrustRates<double> u{};
    u.t = Timestamp{99};
    u.kind = CommandKind::motors;
    u.thrust = 4.5;
    u.motors = {0.1, 0.2, 0.3, 0.4};
    const auto ub = codec::decode_command(codec::encode(u));
    REQUIRE(ub);
    REQUIRE(*ub == u);
}

TEST_CASE("codec: CameraFrameRecord round trip", "[codec]") {
    CameraFrameRecord f{};
    f.header = FrameHeader{Timestamp{7}, 2000, 18, 3, 2, 3, PixelFormat::mono8, false};
    f.bytes = {std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}, std::byte{5}, std::byte{6}};
    const auto back = codec::decode_frame(codec::encode(f));
    REQUIRE(back);
    REQUIRE(*back == f);
}

TEST_CASE("codec: wrong size and invalid values are rejected", "[codec]") {
    ImuSample<double> s{};
    auto bytes = codec::encode(s);
    bytes.push_back(std::byte{0});
    REQUIRE_FALSE(codec::decode_imu(bytes));  // trailing byte
    bytes.resize(bytes.size() - 2);
    REQUIRE_FALSE(codec::decode_imu(bytes));  // truncated

    StateEstimate<double> x{};
    x.attitude = Quat<double>{2.0, 0.0, 0.0, 0.0};  // not unit: invalid
    REQUIRE_FALSE(codec::decode_state(codec::encode(x)));

    CameraFrameRecord f{};
    f.header = FrameHeader{Timestamp{0}, 0, 0, 2, 2, 2, PixelFormat::mono8, false};
    f.bytes.resize(3);  // needs 4
    REQUIRE_FALSE(codec::decode_frame(codec::encode(f)));
}

TEST_CASE("codec: NaN and Inf are rejected in every decoder", "[codec]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    ImuSample<double> s{};
    s.accel[1] = nan;
    REQUIRE_FALSE(codec::decode_imu(codec::encode(s)));
    s = ImuSample<double>{};
    s.temperature = inf;
    REQUIRE_FALSE(codec::decode_imu(codec::encode(s)));

    StateEstimate<double> x{};
    x.velocity[0] = inf;
    REQUIRE_FALSE(codec::decode_state(codec::encode(x)));
    x = StateEstimate<double>{};
    x.gyro_bias[2] = nan;
    REQUIRE_FALSE(codec::decode_state(codec::encode(x)));

    ThrustRates<double> u{};
    u.body_rates[0] = -inf;
    REQUIRE_FALSE(codec::decode_command(codec::encode(u)));
}
