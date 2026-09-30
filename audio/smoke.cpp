#include "spatializer.hpp"

#include <array>
#include <cassert>
#include <cmath>

using MagicAapSpatial::Mode;
using MagicAapSpatial::Pose;
using MagicAapSpatial::Spatializer;

int main()
{
    constexpr std::size_t frameCount = 512;
    std::array<float, frameCount> inputLeft{};
    std::array<float, frameCount> inputRight{};
    std::array<float, frameCount> outputLeft{};
    std::array<float, frameCount> outputRight{};

    for (std::size_t index = 0; index < frameCount; index++)
    {
        inputLeft[index] = 0.35f * std::sin(static_cast<float>(index) * 0.037f);
        inputRight[index] = 0.28f * std::sin(static_cast<float>(index) * 0.053f);
    }

    Spatializer spatializer;
    spatializer.Process(inputLeft.data(), inputRight.data(), outputLeft.data(), outputRight.data(), frameCount);
    assert(outputLeft == inputLeft);
    assert(outputRight == inputRight);

    spatializer.SetMode(Mode::Fixed);
    spatializer.Process(inputLeft.data(), inputRight.data(), outputLeft.data(), outputRight.data(), frameCount);
    bool changed = false;
    for (std::size_t index = 0; index < frameCount; index++)
    {
        assert(std::isfinite(outputLeft[index]));
        assert(std::isfinite(outputRight[index]));
        assert(std::abs(outputLeft[index]) <= 1.0f);
        assert(std::abs(outputRight[index]) <= 1.0f);
        changed = changed || std::abs(outputLeft[index] - inputLeft[index]) > 0.001f;
    }
    assert(changed);

    spatializer.SetMode(Mode::Tracked);
    spatializer.SetPose(Pose{70.0f, 20.0f, 0.0f});
    spatializer.Process(inputLeft.data(), inputRight.data(), outputLeft.data(), outputRight.data(), frameCount);
    for (std::size_t index = 0; index < frameCount; index++)
    {
        assert(std::isfinite(outputLeft[index]));
        assert(std::isfinite(outputRight[index]));
        assert(std::abs(outputLeft[index]) <= 1.0f);
        assert(std::abs(outputRight[index]) <= 1.0f);
    }

    spatializer.SetMode(Mode::Off);
    spatializer.Process(inputLeft.data(), inputRight.data(), outputLeft.data(), outputRight.data(), frameCount);
    assert(outputLeft == inputLeft);
    assert(outputRight == inputRight);
    return 0;
}