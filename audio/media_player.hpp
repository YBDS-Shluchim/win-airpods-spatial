#pragma once

#include "atmos_panner.hpp"
#include "spatializer.hpp"

#include <atomic>
#include <filesystem>
#include <memory>

namespace MagicAapSpatial
{
enum class PlaybackSpatialMode
{
    Off,
    Fixed,
    Tracked
};

struct PlaybackSettings
{
    PlaybackSpatialMode mode = PlaybackSpatialMode::Fixed;
    float stageWidthDegrees = 68.0f;
    float roomReflection = 0.08f;
    bool reverseHeadTracking = true;
    bool channelBedMode = false;
    bool cavernAtmosObjects = false;
    AtmosPannerSettings atmosPanner{};
};

class MediaPlayer final
{
public:
    void SetHeadPose(float yawDegrees, float pitchDegrees);
    void SetHrtfProfile(std::shared_ptr<const HrtfProfile> profile);

    int PlayFile(
        const std::filesystem::path& path,
        PlaybackSettings settings,
        const std::atomic_bool& stopRequested);

private:
    std::atomic<float> yawDegrees_{0.0f};
    std::atomic<float> pitchDegrees_{0.0f};
    std::atomic<std::shared_ptr<const HrtfProfile>> hrtfProfile_{};
};
}