#include "atmos_panner.hpp"

#include <array>
#include <cassert>
#include <cmath>

using namespace MagicAapSpatial;

int main()
{
    CavernObjectBlock block;
    block.frameCount = 64;
    CavernObjectFrame object;
    object.isDynamic = true;
    object.z = 1.0f;
    object.samples.assign(block.frameCount, 0.5f);
    block.objects.push_back(object);

    std::array<float, 64> left{};
    std::array<float, 64> right{};
    AtmosPannerSettings settings;
    settings.dynamicObjects = ObjectLockMode::WorldLocked;
    settings.bed = ObjectLockMode::Diffuse;
    AtmosPanner panner(settings);

    panner.Process(block, 0.0f, 0.0f, left.data(), right.data(), left.size());
    const float centerLeft = left[0];
    const float centerRight = right[0];
    panner.Process(block, 60.0f, 0.0f, left.data(), right.data(), left.size());
    assert(std::abs(left[0] - centerLeft) > 0.05f || std::abs(right[0] - centerRight) > 0.05f);

    settings.dynamicObjects = ObjectLockMode::HeadLocked;
    panner.SetSettings(settings);
    panner.Process(block, 0.0f, 0.0f, left.data(), right.data(), left.size());
    const float headLockedLeft = left[0];
    const float headLockedRight = right[0];
    panner.Process(block, 60.0f, 0.0f, left.data(), right.data(), left.size());
    assert(std::abs(left[0] - headLockedLeft) < 0.001f);
    assert(std::abs(right[0] - headLockedRight) < 0.001f);

    settings.dynamicObjects = ObjectLockMode::WorldLocked;
    settings.bed = ObjectLockMode::Diffuse;
    panner.SetSettings(settings);
    block.objects[0].isDynamic = false;
    panner.Process(block, 0.0f, 0.0f, left.data(), right.data(), left.size());
    assert(std::abs(left[0] - right[0]) < 0.001f);
    return 0;
}