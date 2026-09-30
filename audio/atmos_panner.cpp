#include "atmos_panner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace MagicAapSpatial
{
namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadiansPerDegree = kPi / 180.0f;

float SoftLimit(float value)
{
    const float magnitude = std::abs(value);
    if (magnitude <= 0.85f) return value;
    const float limited = 0.85f + 0.15f * std::tanh((magnitude - 0.85f) / 0.15f);
    return std::copysign(limited, value);
}
}

AtmosPanner::AtmosPanner(AtmosPannerSettings settings)
{
    SetSettings(settings);
}

void AtmosPanner::SetSettings(AtmosPannerSettings settings)
{
    settings.objectWidth = std::isfinite(settings.objectWidth)
        ? std::clamp(settings.objectWidth, 0.5f, 2.0f)
        : 1.0f;
    settings.diffuseBedLevel = std::isfinite(settings.diffuseBedLevel)
        ? std::clamp(settings.diffuseBedLevel, 0.0f, 0.7f)
        : 0.35f;
    settings_ = settings;
}

void AtmosPanner::Process(
    const CavernObjectBlock& block,
    float listenerYawDegrees,
    float listenerPitchDegrees,
    float* outputLeft,
    float* outputRight,
    std::size_t outputCapacity)
{
    if (outputLeft == nullptr || outputRight == nullptr ||
        outputCapacity < static_cast<std::size_t>(block.frameCount))
    {
        throw std::invalid_argument("Atmos panner output buffer is too small.");
    }
    std::fill_n(outputLeft, block.frameCount, 0.0f);
    std::fill_n(outputRight, block.frameCount, 0.0f);

    const float yaw = std::isfinite(listenerYawDegrees) ? listenerYawDegrees : 0.0f;
    const float pitch = std::isfinite(listenerPitchDegrees) ? listenerPitchDegrees : 0.0f;
    const float pitchRadians = pitch * kRadiansPerDegree;
    const float cosPitch = std::cos(pitchRadians);
    const float sinPitch = std::sin(pitchRadians);

    for (const auto& object : block.objects)
    {
        const auto lockMode = object.isDynamic ? settings_.dynamicObjects : settings_.bed;
        if (object.isLfe)
        {
            for (int frame = 0; frame < block.frameCount &&
                static_cast<std::size_t>(frame) < object.samples.size(); frame++)
            {
                const float sample = object.samples[static_cast<std::size_t>(frame)] * 0.20f;
                outputLeft[frame] += sample;
                outputRight[frame] += sample;
            }
            continue;
        }

        if (lockMode == ObjectLockMode::Diffuse)
        {
            for (int frame = 0; frame < block.frameCount &&
                static_cast<std::size_t>(frame) < object.samples.size(); frame++)
            {
                const float sample = object.samples[static_cast<std::size_t>(frame)] * settings_.diffuseBedLevel;
                outputLeft[frame] += sample;
                outputRight[frame] += sample;
            }
            continue;
        }

        const float x = object.x;
        const float y = object.y;
        const float z = object.z;
        const float relativeYaw = lockMode == ObjectLockMode::WorldLocked ? -yaw : 0.0f;
        const float yawRadians = relativeYaw * kRadiansPerDegree;
        const float rotatedX = (std::cos(yawRadians) * x + std::sin(yawRadians) * z) * settings_.objectWidth;
        const float rotatedZ = (-std::sin(yawRadians) * x + std::cos(yawRadians) * z);
        const float rotatedY = y * cosPitch - rotatedZ * sinPitch;
        const float depth = std::max(0.25f, std::sqrt(rotatedX * rotatedX + rotatedY * rotatedY + rotatedZ * rotatedZ));
        const float azimuth = std::atan2(rotatedX, std::max(0.001f, rotatedZ));
        const float side = std::sin(azimuth);
        const float elevation = std::clamp(rotatedY / depth, -1.0f, 1.0f);
        const float distanceGain = 1.0f / (1.0f + 0.12f * std::max(0.0f, depth - 1.0f));
        const float elevationGain = 1.0f - std::abs(elevation) * 0.06f;
        const float leftGain = std::sqrt(std::clamp(0.5f * (1.0f - side), 0.0f, 1.0f)) * distanceGain * elevationGain;
        const float rightGain = std::sqrt(std::clamp(0.5f * (1.0f + side), 0.0f, 1.0f)) * distanceGain * elevationGain;

        for (int frame = 0; frame < block.frameCount &&
            static_cast<std::size_t>(frame) < object.samples.size(); frame++)
        {
            const float sample = object.samples[static_cast<std::size_t>(frame)];
            outputLeft[frame] += sample * leftGain;
            outputRight[frame] += sample * rightGain;
        }
    }

    for (int frame = 0; frame < block.frameCount; frame++)
    {
        outputLeft[frame] = SoftLimit(outputLeft[frame]);
        outputRight[frame] = SoftLimit(outputRight[frame]);
    }
}
}