#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <memory>

namespace MagicAapSpatial
{
class AppleHrtf final
{
public:
    static constexpr std::size_t kTapCount = 64;

    struct Filter final
    {
        std::array<std::array<float, kTapCount>, 2> coefficients{};
        std::array<float, 2> auxiliary{};
        float modelingDelaySamples = 0.0f;
    };

    static std::shared_ptr<const AppleHrtf> Load(const std::filesystem::path& path);

    Filter Interpolate(float azimuthDegrees, float elevationDegrees) const;

private:
    static constexpr std::size_t kElevationCount = 50;
    static constexpr std::size_t kAzimuthCount = 25;

    std::array<Filter, kElevationCount * kAzimuthCount> filters_{};
};
}