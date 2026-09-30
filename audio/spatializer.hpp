#pragma once

#include <array>
#include <atomic>
#include <cstddef>

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

    void SetMode(Mode mode);
    void SetPose(Pose pose);
    void SetStageWidth(float degrees);
    void SetRoomReflection(float amount);
    void Reset();

    void Process(
        const float* inputLeft,
        const float* inputRight,
        float* outputLeft,
        float* outputRight,
        std::size_t frameCount);

private:
    struct PathParameters
    {
        float gainLeft;
        float gainRight;
        float delayLeft;
        float delayRight;
        float alphaLeft;
        float alphaRight;
    };

    PathParameters CalculatePath(float azimuthDegrees) const;
    float ReadDelay(const std::array<float, 8192>& buffer, float delaySamples) const;
    float Filter(float input, std::size_t stateIndex, float alpha);
    float CalculateEarlyReflection(float side, bool leftOutput) const;
    float LowPassAlpha(float cutoffHz) const;
    static float WrapDegrees(float value);
    static float SoftLimit(float value);

    static constexpr std::size_t kDelayBufferSize = 8192;
    float sampleRate_;
    std::atomic<int> requestedMode_{static_cast<int>(Mode::Off)};
    std::atomic<bool> resetRequested_{false};
    std::atomic<float> requestedYaw_{0.0f};
    std::atomic<float> requestedPitch_{0.0f};
    std::atomic<float> requestedStageWidth_{68.0f};
    std::atomic<float> requestedRoomReflection_{0.12f};
    Mode activeMode_ = Mode::Off;
    float smoothedYaw_ = 0.0f;
    float smoothedPitch_ = 0.0f;
    std::array<float, kDelayBufferSize> delayLeft_{};
    std::array<float, kDelayBufferSize> delayRight_{};
    std::array<float, 4> filterState_{};
    std::size_t writeIndex_ = 0;
};
}