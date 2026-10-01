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
    assert(spatializer.IsHrtfReady());
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

    const auto fixedLeft = outputLeft;
    const auto fixedRight = outputRight;
    Spatializer centeredTracked;
    centeredTracked.SetMode(Mode::Tracked);
    centeredTracked.SetPose(Pose{});
    std::array<float, frameCount> centeredLeft{};
    std::array<float, frameCount> centeredRight{};
    centeredTracked.Process(
        inputLeft.data(), inputRight.data(), centeredLeft.data(), centeredRight.data(), frameCount);
    for (std::size_t index = 0; index < frameCount; index++)
    {
        assert(std::abs(centeredLeft[index] - fixedLeft[index]) < 0.00001f);
        assert(std::abs(centeredRight[index] - fixedRight[index]) < 0.00001f);
    }

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

    Spatializer turnedTracked;
    turnedTracked.SetMode(Mode::Tracked);
    turnedTracked.SetPose(Pose{70.0f, 20.0f, 0.0f});
    std::array<float, frameCount> turnedLeft{};
    std::array<float, frameCount> turnedRight{};
    turnedTracked.Process(
        inputLeft.data(), inputRight.data(), turnedLeft.data(), turnedRight.data(), frameCount);
    bool poseChanged = false;
    for (std::size_t index = 0; index < frameCount; index++)
    {
        poseChanged = poseChanged || std::abs(turnedLeft[index] - centeredLeft[index]) > 0.001f;
    }
    assert(poseChanged);

    const auto sideEnergy = [](float yawDegrees)
    {
        Spatializer renderer;
        renderer.SetMode(Mode::Tracked);
        renderer.SetStageWidth(68.0f);
        renderer.SetRoomReflection(0.0f);
        std::array<float, frameCount> monoInput{};
        std::array<float, frameCount> renderedLeft{};
        std::array<float, frameCount> renderedRight{};
        for (std::size_t index = 0; index < frameCount; index++)
        {
            monoInput[index] = 0.2f * std::sin(static_cast<float>(index) * 0.071f);
        }

        double leftEnergy = 0.0;
        double rightEnergy = 0.0;
        for (int block = 0; block < 12; block++)
        {
            renderer.SetPose(Pose{yawDegrees, 0.0f, 0.0f});
            renderer.Process(
                monoInput.data(), monoInput.data(),
                renderedLeft.data(), renderedRight.data(), frameCount);
            if (block < 4) continue;
            for (std::size_t index = 0; index < frameCount; index++)
            {
                leftEnergy += renderedLeft[index] * renderedLeft[index];
                rightEnergy += renderedRight[index] * renderedRight[index];
            }
        }
        return leftEnergy - rightEnergy;
    };

    const double leftTurnSideEnergy = sideEnergy(-60.0f);
    const double rightTurnSideEnergy = sideEnergy(60.0f);
    assert(leftTurnSideEnergy * rightTurnSideEnergy < 0.0);

    Spatializer dryRoom;
    Spatializer wetRoom;
    dryRoom.SetMode(Mode::Fixed);
    wetRoom.SetMode(Mode::Fixed);
    dryRoom.SetRoomReflection(0.0f);
    wetRoom.SetRoomReflection(0.08f);
    std::array<float, frameCount> dryLeft{};
    std::array<float, frameCount> dryRight{};
    std::array<float, frameCount> wetLeft{};
    std::array<float, frameCount> wetRight{};
    dryRoom.Process(inputLeft.data(), inputRight.data(), dryLeft.data(), dryRight.data(), frameCount);
    wetRoom.Process(inputLeft.data(), inputRight.data(), wetLeft.data(), wetRight.data(), frameCount);
    bool roomChanged = false;
    for (std::size_t index = 0; index < frameCount; index++)
    {
        roomChanged = roomChanged || std::abs(wetLeft[index] - dryLeft[index]) > 0.0001f;
    }
    assert(roomChanged);

    spatializer.SetMode(Mode::Off);
    spatializer.Process(inputLeft.data(), inputRight.data(), outputLeft.data(), outputRight.data(), frameCount);
    assert(outputLeft == inputLeft);
    assert(outputRight == inputRight);
    return 0;
}