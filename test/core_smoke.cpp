#include "core.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

using MagicAapSpatial::AapStreamFramer;
using MagicAapSpatial::Packet;
using MagicAapSpatial::PoseEstimator;

namespace
{
void AppendVarint(Packet& output, std::uint64_t value)
{
    while (value > 0x7f)
    {
        output.push_back(static_cast<std::uint8_t>((value & 0x7f) | 0x80));
        value >>= 7;
    }
    output.push_back(static_cast<std::uint8_t>(value));
}

void AppendBytesField(Packet& output, std::uint32_t number, const Packet& value)
{
    AppendVarint(output, (static_cast<std::uint64_t>(number) << 3) | 2);
    AppendVarint(output, value.size());
    output.insert(output.end(), value.begin(), value.end());
}

void AppendVarintField(Packet& output, std::uint32_t number, std::uint64_t value)
{
    AppendVarint(output, static_cast<std::uint64_t>(number) << 3);
    AppendVarint(output, value);
}

Packet MotionPacket(std::uint32_t service = 6, std::int16_t x = 7000, std::int16_t y = 0)
{
    Packet payload(58, 0);
    payload[0] = 1;
    payload[9] = 3;
    payload[20] = static_cast<std::uint8_t>(x & 0xff);
    payload[21] = static_cast<std::uint8_t>((static_cast<std::uint16_t>(x) >> 8) & 0xff);
    payload[22] = static_cast<std::uint8_t>(y & 0xff);
    payload[23] = static_cast<std::uint8_t>((static_cast<std::uint16_t>(y) >> 8) & 0xff);
    payload[28] = 123;
    payload[30] = 0xd3;
    payload[31] = 0xff;

    Packet command;
    AppendVarintField(command, 1, service);
    AppendBytesField(command, 3, payload);
    Packet body;
    AppendBytesField(body, 7, command);

    Packet packet{0x04, 0x00, 0x04, 0x00, 0x17, 0x00, 0x00, 0x00, 0x10, 0x00};
    packet.push_back(static_cast<std::uint8_t>(body.size() & 0xff));
    packet.push_back(static_cast<std::uint8_t>((body.size() >> 8) & 0xff));
    packet.insert(packet.end(), body.begin(), body.end());
    return packet;
}

}

int main()
{
    const auto first = MotionPacket();
    const auto second = MotionPacket(14, 0, 7000);
    Packet joined = first;
    joined.insert(joined.end(), second.begin(), second.end());

    AapStreamFramer framer;
    assert(framer.Push(std::span<const std::uint8_t>(joined).first(9)).empty());
    const auto framed = framer.Push(std::span<const std::uint8_t>(joined).subspan(9));
    assert(framed.size() == 1);
    assert(framed.front() == first);
    assert(framer.Complete().front() == second);

    const auto decoded = MagicAapSpatial::DecodeMotionPacket(first);
    assert(decoded.has_value());
    assert(decoded->service == 6);
    assert(decoded->horizontalAcceleration == 123);
    assert(decoded->verticalAcceleration == -45);
    assert(MagicAapSpatial::DecodeMotionPacket(MotionPacket(14)).has_value());
    assert(!MagicAapSpatial::DecodeMotionPacket(MotionPacket(2)).has_value());
    assert(!MagicAapSpatial::DecodeMotionPacket(MotionPacket(6, 0, 0)).has_value());

    auto malformed = first;
    malformed[12] = 0;
    assert(!MagicAapSpatial::DecodeMotionPacket(malformed).has_value());

    PoseEstimator estimator(4);
    assert(!estimator.ProcessPacket(first)->calibrated);
    assert(estimator.ProcessPacket(first)->calibrationCount == 2);
    assert(estimator.ProcessPacket(first)->calibrationCount == 3);
    assert(estimator.ProcessPacket(first)->calibrated);
    assert(estimator.IsCalibrated());
    const auto changedOrientation = MotionPacket(16, 0, 7000);
    const auto firstPose = estimator.ProcessPacket(changedOrientation);
    const auto secondPose = estimator.ProcessPacket(changedOrientation);
    assert(firstPose->yawDegrees.has_value());
    assert(secondPose->yawDegrees.has_value());
    assert(std::abs(*secondPose->yawDegrees) + std::abs(*secondPose->pitchDegrees) +
        std::abs(*secondPose->rollDegrees) > 1.0);
    estimator.Reset();
    assert(!estimator.IsCalibrated());
    assert(estimator.CalibrationCount() == 0);
    return 0;
}