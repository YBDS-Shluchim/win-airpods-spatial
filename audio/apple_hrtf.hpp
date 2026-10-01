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

    // Mean of the per-ear "auxiliary" scalar across the whole dataset. The exact
    // meaning of that scalar is not documented by Apple; it trends smoothly with
    // azimuth (unlike the noisy FIR tail), so it is treated as a per-position/per-ear
    // gain correction rather than an extra filter tap. This average lets callers
    // apply it as a relative correction without needing a hand-tuned baseline gain.
    float AverageAuxiliary() const noexcept
    {
        return averageAuxiliary_;
    }

private:
    static constexpr std::size_t kElevationCount = 50;
    static constexpr std::size_t kAzimuthCount = 25;

    std::array<Filter, kElevationCount * kAzimuthCount> filters_{};
    float averageAuxiliary_ = 1.0f;
};
}