#pragma once

#include <cstddef>
#include <memory>

namespace MagicAapSpatial
{
enum class Mode
{
    Off,
    Fixed,
    Tracked
};

struct Pose
{
    float yawDegrees = 0.0f;
    float pitchDegrees = 0.0f;
    float rollDegrees = 0.0f;
};

class Spatializer final
{
public:
    explicit Spatializer(float sampleRate = 48000.0f);
    ~Spatializer();

    Spatializer(const Spatializer&) = delete;
    Spatializer& operator=(const Spatializer&) = delete;

    void SetMode(Mode mode);
    void SetPose(Pose pose);
    void SetStageWidth(float degrees);
    void SetRoomReflection(float amount);
    void Reset();
    bool IsHrtfReady() const;

    void Process(
        const float* inputLeft,
        const float* inputRight,
        float* outputLeft,
        float* outputRight,
        std::size_t frameCount);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}