#pragma once

#include "cavern_object_stream.hpp"
#include "spatializer.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace MagicAapSpatial
{
enum class ObjectLockMode
{
    WorldLocked,
    HeadLocked,
    Diffuse
};

struct AtmosPannerSettings
{
    ObjectLockMode dynamicObjects = ObjectLockMode::WorldLocked;
    ObjectLockMode bed = ObjectLockMode::Diffuse;
    float objectWidth = 1.0f;
    float diffuseBedLevel = 0.35f;
};

class AtmosPanner final
{
public:
    explicit AtmosPanner(AtmosPannerSettings settings = {});
    void SetSettings(AtmosPannerSettings settings);
    void SetSampleRate(float sampleRate);

    // Renders non-diffuse objects through the Apple HRTF convolver instead of plain
    // equal-power panning. Falls back to equal-power panning when no profile is set.
    void SetHrtfProfile(std::shared_ptr<const HrtfProfile> profile);

    void Process(
        const CavernObjectBlock& block,
        float listenerYawDegrees,
        float listenerPitchDegrees,
        float* outputLeft,
        float* outputRight,
        std::size_t outputCapacity);

private:
    struct ObjectVoice final
    {
        std::vector<float> history = std::vector<float>(HrtfProfile::kMaxFilterTaps, 0.0f);
        std::size_t historyIndex = 0;
        HrtfProfile::Filter currentFilter;
        HrtfProfile::Filter targetFilter;
        bool initialized = false;
    };

    AtmosPannerSettings settings_;
    float sampleRate_ = 48000.0f;
    std::shared_ptr<const HrtfProfile> hrtfProfile_;
    std::vector<ObjectVoice> voices_;
};
}