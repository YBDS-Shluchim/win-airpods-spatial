#include "head_tracker.hpp"
#include "windows_device.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace MagicAapSpatial
{
namespace
{
struct TrackingProfile
{
    const char* name;
    Packet start;
    Packet stop;
};

Packet ParseHex(const char* text)
{
    Packet bytes;
    for (std::size_t index = 0; text[index] != '\0'; index += 2)
    {
        const auto pair = std::string(text + index, 2);
        bytes.push_back(static_cast<std::uint8_t>(std::stoul(pair, nullptr, 16)));
    }
    return bytes;
}

const std::array<Packet, 4> kInitPackets{
    ParseHex("00000000010003000000000000000000"),
    ParseHex("04000000010000"),
    ParseHex("00000400010003000000000000000000"),
    ParseHex("04000400010000")};

const Packet kOwnsConnection = ParseHex("0400040009000601000000");

const std::array<TrackingProfile, 3> kProfiles{{
    {
        "alternate",
        ParseHex("040004001700000010000f000873420b081010021a0501409c0000"),
        ParseHex("040004001700000010000f000875420b081010021a050100000000"),
    },
    {
        "devmotion6",
        ParseHex("040004001700000010001000089301420b081010021a0501204e0000"),
        ParseHex("040004001700000010001000089501420b081010021a050100000000"),
    },
    {
        "max2",
        ParseHex("040004001700000010001000089801420b080610021a0501204e0000"),
        ParseHex("040004001700000010001000089a01420b080610021a050100000000"),
    },
}};

bool DelayUnlessStopped(std::chrono::milliseconds duration, const std::atomic_bool& stopRequested)
{
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (stopRequested.load(std::memory_order_relaxed)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return !stopRequested.load(std::memory_order_relaxed);
}

DeviceCandidate ChooseDevice()
{
    auto candidates = EnumerateMagicAapInterfaces();
    std::erase_if(candidates, [](const auto& candidate)
    {
        return candidate.kind != "private" && candidate.kind != "service";
    });
    std::stable_sort(candidates.begin(), candidates.end(), [](const auto& left, const auto& right)
    {
        return left.kind == "private" && right.kind != "private";
    });
    for (const auto& candidate : candidates)
    {
        std::string error;
        if (CanOpenDevice(candidate.path, error)) return candidate;
    }
    throw std::runtime_error("No openable MagicAAP interface found.");
}
}

void HeadTracker::Run(
    const std::atomic_bool& stopRequested,
    const PoseCallback& onPose,
    const StatusCallback& onStatus)
{
    const auto candidate = ChooseDevice();
    MagicAapDevice device(candidate.path);
    AapStreamFramer framer;
    PoseEstimator estimator;
    std::array<std::uint8_t, 4096> buffer{};
    std::vector<Packet> replayPackets;
    const TrackingProfile* activeProfile = nullptr;

    onStatus("Connecting to MagicAAP...");
    try
    {
        for (std::size_t index = 0; index < kInitPackets.size(); index++)
        {
            if (stopRequested.load(std::memory_order_relaxed)) return;
            device.Write(kInitPackets[index]);
            if (index < kInitPackets.size() - 1 &&
                !DelayUnlessStopped(
                    index % 2 == 0 ? std::chrono::milliseconds(180) : std::chrono::milliseconds(220),
                    stopRequested))
            {
                return;
            }
        }
        if (!DelayUnlessStopped(std::chrono::milliseconds(1000), stopRequested)) return;
        device.Write(kOwnsConnection);
        if (!DelayUnlessStopped(std::chrono::milliseconds(350), stopRequested)) return;

        onStatus("Detecting head-tracking protocol...");
        for (const auto& profile : kProfiles)
        {
            if (stopRequested.load(std::memory_order_relaxed)) return;
            device.Write(profile.start);
            activeProfile = &profile;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1800);
            bool motionFound = false;
            while (!stopRequested.load(std::memory_order_relaxed) &&
                std::chrono::steady_clock::now() < deadline)
            {
                const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - std::chrono::steady_clock::now());
                const auto count = device.ReadSome(
                    buffer,
                    stopRequested,
                    std::max(std::chrono::milliseconds(1), std::min(std::chrono::milliseconds(100), remaining)));
                if (count == 0) continue;
                auto packets = framer.Push(std::span<const std::uint8_t>(buffer).first(count));
                for (auto& packet : packets)
                {
                    if (DecodeMotionPacket(packet)) motionFound = true;
                    replayPackets.push_back(std::move(packet));
                    if (motionFound) break;
                }
                if (motionFound) break;
            }
            if (motionFound) break;
            device.Write(profile.stop);
            activeProfile = nullptr;
        }

        if (activeProfile == nullptr)
        {
            throw std::runtime_error("No compatible head-tracking packets were detected.");
        }
        onStatus(std::string("Tracking active: ") + activeProfile->name + ". Calibrating; hold still...");

        const auto process = [&](const Packet& packet)
        {
            const auto pose = estimator.ProcessPacket(packet);
            if (pose) onPose(*pose);
        };
        for (const auto& packet : replayPackets) process(packet);

        while (!stopRequested.load(std::memory_order_relaxed))
        {
            const auto count = device.ReadSome(buffer, stopRequested, std::chrono::milliseconds(100));
            if (count == 0) continue;
            for (const auto& packet : framer.Push(std::span<const std::uint8_t>(buffer).first(count)))
            {
                process(packet);
            }
        }
    }
    catch (...)
    {
        if (activeProfile != nullptr)
        {
            try { device.Write(activeProfile->stop); } catch (...) { }
        }
        throw;
    }

    if (activeProfile != nullptr)
    {
        try { device.Write(activeProfile->stop); } catch (...) { }
    }
    onStatus("Head tracking stopped.");
}
}