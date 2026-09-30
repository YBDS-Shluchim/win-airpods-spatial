#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace MagicAapSpatial
{
using Packet = std::vector<std::uint8_t>;
using Quaternion = std::array<double, 4>;

class AapStreamFramer final
{
public:
    std::vector<Packet> Push(std::span<const std::uint8_t> chunk);
    std::vector<Packet> Complete();

private:
    Packet pending_;
};

struct MotionFrame
{
    Quaternion orientation;
    std::int16_t horizontalAcceleration;
    std::int16_t verticalAcceleration;
    std::uint32_t service;
};

std::optional<MotionFrame> DecodeMotionPacket(std::span<const std::uint8_t> packet);

struct PoseResult
{
    bool calibrated = false;
    std::size_t calibrationCount = 0;
    std::optional<double> yawDegrees;
    std::optional<double> pitchDegrees;
    std::optional<double> rollDegrees;
    std::int16_t horizontalAcceleration = 0;
    std::int16_t verticalAcceleration = 0;
};

class PoseEstimator final
{
public:
    explicit PoseEstimator(std::size_t calibrationSamples = 25);

    std::optional<PoseResult> ProcessPacket(std::span<const std::uint8_t> packet);
    bool IsCalibrated() const noexcept;
    std::size_t CalibrationCount() const noexcept;
    void Reset();

private:
    std::size_t calibrationSamples_;
    std::vector<Quaternion> calibration_;
    std::optional<Quaternion> neutral_;
    std::optional<Quaternion> smoothed_;
};
}