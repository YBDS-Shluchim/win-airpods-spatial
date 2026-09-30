#include "spatializer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace MagicAapSpatial
{
namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kMillisecondsPerSecond = 1000.0f;
constexpr float kMaximumInterauralDelaySeconds = 0.00067f;
}

Spatializer::Spatializer(float sampleRate)
    : sampleRate_(std::clamp(sampleRate, 8000.0f, 192000.0f))
{
}

void Spatializer::SetMode(Mode mode)
{
    requestedMode_.store(static_cast<int>(mode), std::memory_order_relaxed);
}

void Spatializer::SetPose(Pose pose)
{
    const float yaw = std::isfinite(pose.yawDegrees)
        ? std::clamp(pose.yawDegrees, -120.0f, 120.0f)
        : 0.0f;
    const float pitch = std::isfinite(pose.pitchDegrees)
        ? std::clamp(pose.pitchDegrees, -45.0f, 45.0f)
        : 0.0f;
    requestedYaw_.store(yaw, std::memory_order_relaxed);
    requestedPitch_.store(pitch, std::memory_order_relaxed);
}

void Spatializer::SetStageWidth(float degrees)
{
    requestedStageWidth_.store(
        std::isfinite(degrees) ? std::clamp(degrees, 20.0f, 120.0f) : 68.0f,
        std::memory_order_relaxed);
}

void Spatializer::SetRoomReflection(float amount)
{
    requestedRoomReflection_.store(
        std::isfinite(amount) ? std::clamp(amount, 0.0f, 0.35f) : 0.12f,
        std::memory_order_relaxed);
}

void Spatializer::Reset()
{
    resetRequested_.store(true, std::memory_order_relaxed);
}

void Spatializer::Process(
    const float* inputLeft,
    const float* inputRight,
    float* outputLeft,
    float* outputRight,
    std::size_t frameCount)
{
    if (inputLeft == nullptr || inputRight == nullptr ||
        outputLeft == nullptr || outputRight == nullptr || frameCount == 0)
    {
        return;
    }

    const auto requestedMode = static_cast<Mode>(requestedMode_.load(std::memory_order_relaxed));
    if (requestedMode != activeMode_ || resetRequested_.exchange(false, std::memory_order_relaxed))
    {
        delayLeft_.fill(0.0f);
        delayRight_.fill(0.0f);
        filterState_.fill(0.0f);
        writeIndex_ = 0;
        smoothedYaw_ = 0.0f;
        smoothedPitch_ = 0.0f;
        activeMode_ = requestedMode;
    }

    if (activeMode_ == Mode::Off)
    {
        if (outputLeft != inputLeft)
        {
            std::memmove(outputLeft, inputLeft, frameCount * sizeof(float));
        }
        if (outputRight != inputRight)
        {
            std::memmove(outputRight, inputRight, frameCount * sizeof(float));
        }
        return;
    }

    const float targetYaw = activeMode_ == Mode::Tracked
        ? requestedYaw_.load(std::memory_order_relaxed)
        : 0.0f;
    const float targetPitch = activeMode_ == Mode::Tracked
        ? requestedPitch_.load(std::memory_order_relaxed)
        : 0.0f;
    const float blockSeconds = static_cast<float>(frameCount) / sampleRate_;
    const float smoothing = 1.0f - std::exp(-blockSeconds / 0.055f);
    smoothedYaw_ += WrapDegrees(targetYaw - smoothedYaw_) * smoothing;
    smoothedPitch_ += (targetPitch - smoothedPitch_) * smoothing;

    const float halfStageWidth = requestedStageWidth_.load(std::memory_order_relaxed) * 0.5f;
    const float roomReflection = requestedRoomReflection_.load(std::memory_order_relaxed);
    const auto leftSource = CalculatePath(-halfStageWidth - smoothedYaw_);
    const auto rightSource = CalculatePath(halfStageWidth - smoothedYaw_);
    const float reflectionSide = std::sin(WrapDegrees(-smoothedYaw_) * kPi / 180.0f);
    const float elevationGain = std::clamp(
        1.0f + (smoothedPitch_ / 45.0f) * 0.035f,
        0.965f,
        1.035f);

    for (std::size_t frame = 0; frame < frameCount; frame++)
    {
        const float left = inputLeft[frame];
        const float right = inputRight[frame];
        delayLeft_[writeIndex_] = left;
        delayRight_[writeIndex_] = right;

        const float leftToLeft = Filter(
            ReadDelay(delayLeft_, leftSource.delayLeft),
            0,
            leftSource.alphaLeft);
        const float leftToRight = Filter(
            ReadDelay(delayLeft_, leftSource.delayRight),
            1,
            leftSource.alphaRight);
        const float rightToLeft = Filter(
            ReadDelay(delayRight_, rightSource.delayLeft),
            2,
            rightSource.alphaLeft);
        const float rightToRight = Filter(
            ReadDelay(delayRight_, rightSource.delayRight),
            3,
            rightSource.alphaRight);

        const float earlyLeft = CalculateEarlyReflection(reflectionSide, true);
        const float earlyRight = CalculateEarlyReflection(reflectionSide, false);
        outputLeft[frame] = SoftLimit(
            (leftToLeft * leftSource.gainLeft +
             rightToLeft * rightSource.gainLeft) *
                0.72f * elevationGain +
            earlyLeft * roomReflection);
        outputRight[frame] = SoftLimit(
            (leftToRight * leftSource.gainRight +
             rightToRight * rightSource.gainRight) *
                0.72f * elevationGain +
            earlyRight * roomReflection);

        writeIndex_ = (writeIndex_ + 1) % kDelayBufferSize;
    }
}

Spatializer::PathParameters Spatializer::CalculatePath(float azimuthDegrees) const
{
    const float azimuth = WrapDegrees(azimuthDegrees) * kPi / 180.0f;
    const float side = std::sin(azimuth);
    const float lateral = std::abs(side);
    const float rear = std::max(0.0f, -std::cos(azimuth));
    const float nearGain = 0.88f + 0.12f * lateral - 0.05f * rear;
    const float farGain = 0.76f - 0.18f * lateral - 0.06f * rear;
    const float farCutoff = std::clamp(
        7600.0f - 4800.0f * lateral - 1700.0f * rear,
        1500.0f,
        7600.0f);
    const float nearCutoff = std::clamp(
        16500.0f - 3500.0f * rear,
        9000.0f,
        18000.0f);
    const float itd = lateral * kMaximumInterauralDelaySeconds * sampleRate_;

    if (side >= 0.0f)
    {
        return {
            farGain,
            nearGain,
            itd,
            0.0f,
            LowPassAlpha(farCutoff),
            LowPassAlpha(nearCutoff)};
    }

    return {
        nearGain,
        farGain,
        0.0f,
        itd,
        LowPassAlpha(nearCutoff),
        LowPassAlpha(farCutoff)};
}

float Spatializer::ReadDelay(
    const std::array<float, kDelayBufferSize>& buffer,
    float delaySamples) const
{
    const float wrappedDelay = std::clamp(delaySamples, 0.0f, static_cast<float>(kDelayBufferSize - 2));
    float position = static_cast<float>(writeIndex_) - wrappedDelay;
    if (position < 0.0f) position += static_cast<float>(kDelayBufferSize);
    const auto first = static_cast<std::size_t>(position);
    const auto second = (first + 1) % kDelayBufferSize;
    const float fraction = position - static_cast<float>(first);
    return buffer[first] + (buffer[second] - buffer[first]) * fraction;
}

float Spatializer::Filter(float input, std::size_t stateIndex, float alpha)
{
    float& state = filterState_[stateIndex];
    state += alpha * (input - state);
    return state;
}

float Spatializer::CalculateEarlyReflection(float side, bool leftOutput) const
{
    const auto tap = [this](
                         const std::array<float, kDelayBufferSize>& buffer,
                         float milliseconds)
    {
        return ReadDelay(buffer, milliseconds * sampleRate_ / kMillisecondsPerSecond);
    };

    if (leftOutput)
    {
        const float directSide = side < 0.0f ? 1.0f : 0.72f;
        return directSide * (
            tap(delayLeft_, 3.7f) * 0.34f +
            tap(delayRight_, 4.9f) * 0.21f -
            tap(delayLeft_, 6.1f) * 0.16f +
            tap(delayRight_, 8.3f) * 0.15f +
            tap(delayLeft_, 10.7f) * 0.13f -
            tap(delayRight_, 13.9f) * 0.10f);
    }

    const float directSide = side > 0.0f ? 1.0f : 0.72f;
    return directSide * (
        tap(delayRight_, 4.1f) * 0.34f +
        tap(delayLeft_, 5.3f) * 0.21f -
        tap(delayRight_, 6.7f) * 0.16f +
        tap(delayLeft_, 8.9f) * 0.15f +
        tap(delayRight_, 11.3f) * 0.13f -
        tap(delayLeft_, 14.9f) * 0.10f);
}

float Spatializer::LowPassAlpha(float cutoffHz) const
{
    return 1.0f - std::exp(-2.0f * kPi * cutoffHz / sampleRate_);
}

float Spatializer::WrapDegrees(float value)
{
    while (value > 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}

float Spatializer::SoftLimit(float value)
{
    const float magnitude = std::abs(value);
    if (magnitude <= 0.85f) return value;
    const float limited = 0.85f + 0.15f * std::tanh((magnitude - 0.85f) / 0.15f);
    return std::copysign(limited, value);
}
}