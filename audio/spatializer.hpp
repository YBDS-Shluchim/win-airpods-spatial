#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
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

class HrtfProfile final
{
public:
    static constexpr std::size_t kMaxFilterTaps = 1024;

    struct Filter final
    {
        std::array<float, kMaxFilterTaps> left{};
        std::array<float, kMaxFilterTaps> right{};
        std::size_t tapCount = 0;
    };

    static std::shared_ptr<const HrtfProfile> Load(
        const std::filesystem::path& path,
        float sampleRate = 48000.0f);

    const std::filesystem::path& Path() const noexcept;

    // Builds a combined left/right FIR filter (including the modeled propagation
    // delay) for a single source at the given azimuth/elevation, resampled to
    // sampleRate. Used by both the stereo Spatializer and per-object Atmos panning.
    Filter ComputeFilter(float sampleRate, float azimuthDegrees, float elevationDegrees) const;

private:
    struct Impl;
    explicit HrtfProfile(std::shared_ptr<Impl> impl);
    std::shared_ptr<Impl> impl_;

    friend class Spatializer;
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
    void SetHrtfBlend(float amount);
    void SetHrtfProfile(std::shared_ptr<const HrtfProfile> profile);
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