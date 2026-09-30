#pragma once

#include "cavern_object_stream.hpp"

#include <cstddef>
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

    void Process(
        const CavernObjectBlock& block,
        float listenerYawDegrees,
        float listenerPitchDegrees,
        float* outputLeft,
        float* outputRight,
        std::size_t outputCapacity);

private:
    AtmosPannerSettings settings_;
};
}