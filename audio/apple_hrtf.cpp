#include "apple_hrtf.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <vector>

namespace MagicAapSpatial
{
namespace
{
constexpr std::array<float, 25> kCipicAzimuths{
    -80.0f, -65.0f, -55.0f, -45.0f, -40.0f,
    -35.0f, -30.0f, -25.0f, -20.0f, -15.0f,
    -10.0f, -5.0f, 0.0f, 5.0f, 10.0f,
    15.0f, 20.0f, 25.0f, 30.0f, 35.0f,
    40.0f, 45.0f, 55.0f, 65.0f, 80.0f};
constexpr float kFirstCipicElevation = -45.0f;
constexpr float kCipicElevationStep = 5.625f;
constexpr std::size_t kRecordSize = 532;
constexpr std::size_t kExpectedSize = 50 * (sizeof(std::uint16_t) + 25 * kRecordSize);
constexpr float kPi = 3.14159265358979323846f;

class BigEndianReader final
{
public:
    explicit BigEndianReader(const std::vector<std::uint8_t>& bytes)
        : bytes_(bytes)
    {
    }

    bool Read(std::uint16_t& value)
    {
        if (offset_ + 2 > bytes_.size()) return false;
        value = static_cast<std::uint16_t>(bytes_[offset_]) << 8 |
            static_cast<std::uint16_t>(bytes_[offset_ + 1]);
        offset_ += 2;
        return true;
    }

    bool Read(float& value)
    {
        if (offset_ + 4 > bytes_.size()) return false;
        const auto bits = static_cast<std::uint32_t>(bytes_[offset_]) << 24 |
            static_cast<std::uint32_t>(bytes_[offset_ + 1]) << 16 |
            static_cast<std::uint32_t>(bytes_[offset_ + 2]) << 8 |
            static_cast<std::uint32_t>(bytes_[offset_ + 3]);
        value = std::bit_cast<float>(bits);
        offset_ += 4;
        return std::isfinite(value);
    }

    std::size_t Offset() const noexcept
    {
        return offset_;
    }

private:
    const std::vector<std::uint8_t>& bytes_;
    std::size_t offset_ = 0;
};

struct AxisPosition final
{
    std::size_t lower;
    std::size_t upper;
    float fraction;
};

AxisPosition FindAzimuth(float value)
{
    value = std::clamp(value, kCipicAzimuths.front(), kCipicAzimuths.back());
    const auto upper = std::lower_bound(kCipicAzimuths.begin(), kCipicAzimuths.end(), value);
    if (upper == kCipicAzimuths.begin()) return {0, 0, 0.0f};
    if (upper == kCipicAzimuths.end())
    {
        return {kCipicAzimuths.size() - 1, kCipicAzimuths.size() - 1, 0.0f};
    }
    const auto upperIndex = static_cast<std::size_t>(upper - kCipicAzimuths.begin());
    const auto lowerIndex = upperIndex - 1;
    const float fraction = (value - kCipicAzimuths[lowerIndex]) /
        (kCipicAzimuths[upperIndex] - kCipicAzimuths[lowerIndex]);
    return {lowerIndex, upperIndex, fraction};
}

AxisPosition FindElevation(float value)
{
    while (value < kFirstCipicElevation) value += 360.0f;
    while (value >= kFirstCipicElevation + 360.0f) value -= 360.0f;
    const float gridPosition = std::clamp(
        (value - kFirstCipicElevation) / kCipicElevationStep,
        0.0f,
        49.0f);
    const auto lower = static_cast<std::size_t>(gridPosition);
    const auto upper = std::min<std::size_t>(lower + 1, 49);
    return {lower, upper, gridPosition - static_cast<float>(lower)};
}

float Lerp(float lower, float upper, float fraction)
{
    return lower + fraction * (upper - lower);
}
}

std::shared_ptr<const AppleHrtf> AppleHrtf::Load(const std::filesystem::path& path)
{
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input || input.tellg() != static_cast<std::streamoff>(kExpectedSize)) return {};

    std::vector<std::uint8_t> bytes(kExpectedSize);
    input.seekg(0, std::ios::beg);
    if (!input.read(reinterpret_cast<char*>(bytes.data()), bytes.size())) return {};

    auto hrtf = std::shared_ptr<AppleHrtf>(new AppleHrtf());
    BigEndianReader reader(bytes);
    for (std::size_t elevation = 0; elevation < kElevationCount; elevation++)
    {
        std::uint16_t azimuthCount = 0;
        if (!reader.Read(azimuthCount) || azimuthCount != kAzimuthCount) return {};
        for (std::size_t azimuth = 0; azimuth < kAzimuthCount; azimuth++)
        {
            auto& filter = hrtf->filters_[elevation * kAzimuthCount + azimuth];
            for (std::size_t ear = 0; ear < 2; ear++)
            {
                std::uint16_t tapCount = 0;
                if (!reader.Read(tapCount) || tapCount != kTapCount) return {};
                for (auto& coefficient : filter.coefficients[ear])
                {
                    if (!reader.Read(coefficient)) return {};
                }
                if (!reader.Read(filter.auxiliary[ear])) return {};
            }
            float duplicateDelay = 0.0f;
            if (!reader.Read(filter.modelingDelaySamples) || !reader.Read(duplicateDelay) ||
                filter.modelingDelaySamples != duplicateDelay || filter.modelingDelaySamples < 0.0f)
            {
                return {};
            }
        }
    }
    if (reader.Offset() != bytes.size()) return {};
    return hrtf;
}

AppleHrtf::Filter AppleHrtf::Interpolate(float azimuthDegrees, float elevationDegrees) const
{
    const float azimuth = azimuthDegrees * kPi / 180.0f;
    const float elevation = elevationDegrees * kPi / 180.0f;
    const float x = std::cos(elevation) * std::cos(azimuth);
    const float y = std::cos(elevation) * std::sin(azimuth);
    const float z = std::sin(elevation);
    const float lateral = -std::atan2(y, std::sqrt(x * x + z * z)) * 180.0f / kPi;
    const float polar = std::atan2(z, x) * 180.0f / kPi;
    const auto azimuthPosition = FindAzimuth(lateral);
    const auto elevationPosition = FindElevation(polar);
    const auto& lowerLeft = filters_[elevationPosition.lower * kAzimuthCount + azimuthPosition.lower];
    const auto& lowerRight = filters_[elevationPosition.lower * kAzimuthCount + azimuthPosition.upper];
    const auto& upperLeft = filters_[elevationPosition.upper * kAzimuthCount + azimuthPosition.lower];
    const auto& upperRight = filters_[elevationPosition.upper * kAzimuthCount + azimuthPosition.upper];

    Filter result;
    for (std::size_t ear = 0; ear < 2; ear++)
    {
        for (std::size_t tap = 0; tap < kTapCount; tap++)
        {
            const float lower = Lerp(
                lowerLeft.coefficients[ear][tap], lowerRight.coefficients[ear][tap],
                azimuthPosition.fraction);
            const float upper = Lerp(
                upperLeft.coefficients[ear][tap], upperRight.coefficients[ear][tap],
                azimuthPosition.fraction);
            result.coefficients[ear][tap] = Lerp(lower, upper, elevationPosition.fraction);
        }
        const float lower = Lerp(
            lowerLeft.auxiliary[ear], lowerRight.auxiliary[ear], azimuthPosition.fraction);
        const float upper = Lerp(
            upperLeft.auxiliary[ear], upperRight.auxiliary[ear], azimuthPosition.fraction);
        result.auxiliary[ear] = Lerp(lower, upper, elevationPosition.fraction);
    }
    const float lowerDelay = Lerp(
        lowerLeft.modelingDelaySamples, lowerRight.modelingDelaySamples,
        azimuthPosition.fraction);
    const float upperDelay = Lerp(
        upperLeft.modelingDelaySamples, upperRight.modelingDelaySamples,
        azimuthPosition.fraction);
    result.modelingDelaySamples = Lerp(lowerDelay, upperDelay, elevationPosition.fraction);
    return result;
}
}