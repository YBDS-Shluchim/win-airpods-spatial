#pragma once

#include "core.hpp"

#include <atomic>
#include <functional>
#include <string>

namespace MagicAapSpatial
{
class HeadTracker final
{
public:
    using PoseCallback = std::function<void(const PoseResult&)>;
    using StatusCallback = std::function<void(const std::string&)>;

    void Run(
        const std::atomic_bool& stopRequested,
        const PoseCallback& onPose,
        const StatusCallback& onStatus);
};
}