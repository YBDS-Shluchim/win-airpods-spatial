#include "spatializer.hpp"

#include <mysofa.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#else
#include <unistd.h>
#endif

namespace MagicAapSpatial
{
namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kMetersPerSofaRadius = 1.4f;
constexpr float kSpatialGain = 0.84f;
constexpr std::size_t kMaximumHrtfTaps = 1024;
constexpr float kRoomDecaySeconds = 0.38f;
constexpr const char* kHrtfFilename = "MIT_KEMAR_normal_pinna.sofa";

float WrapDegrees(float value)
{
    while (value > 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}

std::filesystem::path ExecutableDirectory()
{
#ifdef _WIN32
    std::vector<wchar_t> path(32768);
    const auto length = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
    if (length == 0 || length >= path.size()) return {};
    return std::filesystem::path(std::wstring(path.data(), length)).parent_path();
#else
    std::array<char, 4096> path{};
    const auto length = readlink("/proc/self/exe", path.data(), path.size());
    if (length <= 0 || static_cast<std::size_t>(length) >= path.size()) return {};
    return std::filesystem::path(std::string(path.data(), static_cast<std::size_t>(length))).parent_path();
#endif
}

std::filesystem::path FindHrtfFile()
{
    std::vector<std::filesystem::path> candidates;
    if (const auto* overridePath = std::getenv("MAGIC_AAP_HRTF_PATH");
        overridePath != nullptr && overridePath[0] != '\0')
    {
        candidates.emplace_back(overridePath);
    }

    std::error_code error;
    const auto workingDirectory = std::filesystem::current_path(error);
    if (!error)
    {
        candidates.push_back(workingDirectory / "assets" / kHrtfFilename);
        candidates.push_back(workingDirectory / "audio" / "assets" / kHrtfFilename);
    }

    const auto executableDirectory = ExecutableDirectory();
    if (!executableDirectory.empty())
    {
        candidates.push_back(executableDirectory / "assets" / kHrtfFilename);
    }

    for (const auto& candidate : candidates)
    {
        error.clear();
        if (std::filesystem::is_regular_file(candidate, error) && !error)
        {
            return candidate;
        }
    }
    return {};
}

struct SharedHrtf final
{
    ~SharedHrtf()
    {
        if (easy != nullptr) mysofa_close(easy);
    }

    MYSOFA_EASY* easy = nullptr;
    int filterLength = 0;
    std::filesystem::path path;
    float sampleRate = 0.0f;
    std::mutex queryMutex;
};

std::shared_ptr<SharedHrtf> OpenSharedHrtf(
    const std::filesystem::path& path,
    float sampleRate)
{
    static std::mutex cacheMutex;
    static std::weak_ptr<SharedHrtf> cachedHrtf;
    static std::filesystem::path cachedPath;
    static float cachedSampleRate = 0.0f;

    std::error_code error;
    auto canonicalPath = std::filesystem::weakly_canonical(path, error);
    if (error) canonicalPath = path;

    std::lock_guard cacheLock(cacheMutex);
    if (auto cached = cachedHrtf.lock();
        cached != nullptr && cachedPath == canonicalPath && cachedSampleRate == sampleRate)
    {
        return cached;
    }

    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) return {};
    const auto end = file.tellg();
    if (end <= 0 || end > 128 * 1024 * 1024) return {};
    std::vector<char> data(static_cast<std::size_t>(end));
    file.seekg(0, std::ios::beg);
    if (!file.read(data.data(), static_cast<std::streamsize>(data.size()))) return {};

    int filterLength = 0;
    int errorCode = 0;
    auto* easy = mysofa_open_data_no_norm(
        data.data(), static_cast<long>(data.size()), sampleRate, &filterLength, &errorCode);
    if (easy == nullptr || filterLength <= 0 ||
        filterLength > static_cast<int>(kMaximumHrtfTaps - 64))
    {
        if (easy != nullptr) mysofa_close(easy);
        return {};
    }

    auto loaded = std::make_shared<SharedHrtf>();
    loaded->easy = easy;
    loaded->filterLength = filterLength;
    loaded->path = canonicalPath;
    loaded->sampleRate = sampleRate;
    cachedPath = canonicalPath;
    cachedSampleRate = sampleRate;
    cachedHrtf = loaded;
    return loaded;
}

class CombFilter final
{
public:
    CombFilter(float sampleRate, float delayMilliseconds)
        : buffer_(std::max<std::size_t>(1, static_cast<std::size_t>(std::round(
              sampleRate * delayMilliseconds / 1000.0f)))),
          feedback_(std::pow(0.001f, delayMilliseconds / 1000.0f / kRoomDecaySeconds))
    {
    }

    float Process(float input)
    {
        const float delayed = buffer_[index_];
        dampingState_ += 0.65f * (delayed - dampingState_);
        buffer_[index_] = input + dampingState_ * feedback_;
        index_ = (index_ + 1) % buffer_.size();
        return delayed;
    }

    void Reset()
    {
        std::fill(buffer_.begin(), buffer_.end(), 0.0f);
        index_ = 0;
        dampingState_ = 0.0f;
    }

private:
    std::vector<float> buffer_;
    std::size_t index_ = 0;
    float feedback_;
    float dampingState_ = 0.0f;
};

class AllPassFilter final
{
public:
    AllPassFilter(float sampleRate, float delayMilliseconds, float gain)
        : buffer_(std::max<std::size_t>(1, static_cast<std::size_t>(std::round(
              sampleRate * delayMilliseconds / 1000.0f)))),
          gain_(gain)
    {
    }

    float Process(float input)
    {
        const float delayed = buffer_[index_];
        const float output = delayed - gain_ * input;
        buffer_[index_] = input + gain_ * output;
        index_ = (index_ + 1) % buffer_.size();
        return output;
    }

    void Reset()
    {
        std::fill(buffer_.begin(), buffer_.end(), 0.0f);
        index_ = 0;
    }

private:
    std::vector<float> buffer_;
    std::size_t index_ = 0;
    float gain_;
};

class RoomModel final
{
public:
    explicit RoomModel(float sampleRate)
        : sampleRate_(sampleRate),
          history_{{std::vector<float>(static_cast<std::size_t>(sampleRate * 0.05f) + 2),
                    std::vector<float>(static_cast<std::size_t>(sampleRate * 0.05f) + 2)}}
    {
        constexpr std::array<std::array<float, 4>, 2> combDelays{{
            {{29.7f, 37.1f, 41.1f, 43.7f}},
            {{31.1f, 35.9f, 42.3f, 45.1f}}}};
        constexpr std::array<std::array<float, 2>, 2> allPassDelays{{
            {{5.1f, 1.7f}},
            {{5.7f, 2.3f}}}};

        for (std::size_t ear = 0; ear < 2; ear++)
        {
            for (const float delay : combDelays[ear])
            {
                combs_[ear].emplace_back(sampleRate_, delay);
            }
            for (const float delay : allPassDelays[ear])
            {
                allPasses_[ear].emplace_back(sampleRate_, delay, 0.5f);
            }
        }
    }

    std::array<float, 2> Process(float left, float right)
    {
        history_[0][historyIndex_] = left;
        history_[1][historyIndex_] = right;

        const float mono = (left + right) * 0.5f;
        std::array<float, 2> wet{};
        for (std::size_t ear = 0; ear < 2; ear++)
        {
            float late = 0.0f;
            for (auto& comb : combs_[ear])
            {
                late += comb.Process(mono);
            }
            late *= 0.25f;
            for (auto& allPass : allPasses_[ear])
            {
                late = allPass.Process(late);
            }

            if (ear == 0)
            {
                wet[ear] = Tap(0, 5.3f) * 0.22f + Tap(1, 7.1f) * 0.12f +
                    Tap(0, 11.7f) * 0.07f + Tap(1, 18.9f) * 0.04f + late * 0.45f;
            }
            else
            {
                wet[ear] = Tap(1, 5.9f) * 0.22f + Tap(0, 8.3f) * 0.12f +
                    Tap(1, 12.6f) * 0.07f + Tap(0, 20.1f) * 0.04f + late * 0.45f;
            }
        }

        historyIndex_ = (historyIndex_ + 1) % history_[0].size();
        return wet;
    }

    void Reset()
    {
        for (auto& channel : history_)
        {
            std::fill(channel.begin(), channel.end(), 0.0f);
        }
        for (auto& ear : combs_)
        {
            for (auto& comb : ear) comb.Reset();
        }
        for (auto& ear : allPasses_)
        {
            for (auto& allPass : ear) allPass.Reset();
        }
        historyIndex_ = 0;
    }

private:
    float Tap(std::size_t channel, float delayMilliseconds) const
    {
        const auto delay = std::min<std::size_t>(
            static_cast<std::size_t>(std::round(sampleRate_ * delayMilliseconds / 1000.0f)),
            history_[channel].size() - 1);
        const auto index = (historyIndex_ + history_[channel].size() - delay) % history_[channel].size();
        return history_[channel][index];
    }

    float sampleRate_;
    std::array<std::vector<float>, 2> history_;
    std::array<std::vector<CombFilter>, 2> combs_;
    std::array<std::vector<AllPassFilter>, 2> allPasses_;
    std::size_t historyIndex_ = 0;
};
}

struct HrtfProfile::Impl final
{
    std::shared_ptr<SharedHrtf> hrtf;
    std::filesystem::path path;
};

HrtfProfile::HrtfProfile(std::shared_ptr<Impl> impl)
    : impl_(std::move(impl))
{
}

std::shared_ptr<const HrtfProfile> HrtfProfile::Load(
    const std::filesystem::path& path,
    float sampleRate)
{
    auto hrtf = OpenSharedHrtf(path, sampleRate);
    if (hrtf == nullptr) return {};

    auto impl = std::make_shared<Impl>();
    impl->hrtf = std::move(hrtf);
    impl->path = path;
    return std::shared_ptr<const HrtfProfile>(new HrtfProfile(std::move(impl)));
}

const std::filesystem::path& HrtfProfile::Path() const noexcept
{
    return impl_->path;
}

namespace
{
struct HrtfSelection final
{
    std::shared_ptr<const HrtfProfile> profile;
    std::shared_ptr<SharedHrtf> hrtf;
};
}

struct Spatializer::Impl final
{
    explicit Impl(float sampleRate)
        : sampleRate(std::clamp(sampleRate, 8000.0f, 192000.0f)),
          room(this->sampleRate)
    {
        const auto path = FindHrtfFile();
        if (!path.empty()) hrtf = OpenSharedHrtf(path, this->sampleRate);
    }

    void UpdateFilters(float yaw, float pitch, float stageWidth, float smoothing)
    {
        targetFilters = {};
        targetTapCount = 0;
        const float halfWidth = stageWidth * 0.5f;
        const float elevation = std::clamp(-pitch, -45.0f, 45.0f) * kPi / 180.0f;
        const float radius = kMetersPerSofaRadius;
        const float radiusAtElevation = radius * std::cos(elevation);

        {
            std::lock_guard queryLock(hrtf->queryMutex);
            for (std::size_t source = 0; source < 2; source++)
            {
                const float azimuthDegrees = (source == 0 ? halfWidth : -halfWidth) + yaw;
                const float azimuth = azimuthDegrees * kPi / 180.0f;
                const float position[3]{
                    radiusAtElevation * std::cos(azimuth),
                    radiusAtElevation * std::sin(azimuth),
                    radius * std::sin(elevation)};
                std::array<float, kMaximumHrtfTaps> rawLeft{};
                std::array<float, kMaximumHrtfTaps> rawRight{};
                float delayLeft = 0.0f;
                float delayRight = 0.0f;
                mysofa_getfilter_float(
                    hrtf->easy,
                    position[0],
                    position[1],
                    position[2],
                    rawLeft.data(),
                    rawRight.data(),
                    &delayLeft,
                    &delayRight);

                const std::array<const float*, 2> raw{{rawLeft.data(), rawRight.data()}};
                const std::array<float, 2> delays{{delayLeft, delayRight}};
                for (std::size_t ear = 0; ear < 2; ear++)
                {
                    const std::size_t filter = source * 2 + ear;
                    const float delaySamples = std::max(0.0f, delays[ear] * sampleRate);
                    const auto integerDelay = static_cast<std::size_t>(delaySamples);
                    const float fraction = delaySamples - static_cast<float>(integerDelay);
                    for (int tap = 0; tap < hrtf->filterLength; tap++)
                    {
                        const auto destination = static_cast<std::size_t>(tap) + integerDelay;
                        if (destination + 1 >= kMaximumHrtfTaps) break;
                        targetFilters[filter][destination] += raw[ear][tap] * (1.0f - fraction);
                        targetFilters[filter][destination + 1] += raw[ear][tap] * fraction;
                    }
                    targetTapCount = std::max(targetTapCount,
                        std::min(kMaximumHrtfTaps,
                            static_cast<std::size_t>(hrtf->filterLength) + integerDelay + 1));
                }
            }
        }

        if (!filtersInitialized)
        {
            currentFilters = targetFilters;
            filtersInitialized = true;
        }
        else
        {
            for (std::size_t filter = 0; filter < currentFilters.size(); filter++)
            {
                for (std::size_t tap = 0; tap < kMaximumHrtfTaps; tap++)
                {
                    currentFilters[filter][tap] +=
                        smoothing * (targetFilters[filter][tap] - currentFilters[filter][tap]);
                }
            }
        }
        activeTapCount = targetTapCount;
    }

    void Process(
        const float* inputLeft,
        const float* inputRight,
        float* outputLeft,
        float* outputRight,
        std::size_t frameCount)
    {
        if (inputLeft == nullptr || inputRight == nullptr ||
            outputLeft == nullptr || outputRight == nullptr || frameCount == 0)
        {
            return;
        }

        if (auto selection = requestedHrtf.exchange(nullptr, std::memory_order_relaxed))
        {
            activeProfile = std::move(selection->profile);
            hrtf = std::move(selection->hrtf);
        }

        const auto requested = static_cast<Mode>(requestedMode.load(std::memory_order_relaxed));
        if (requested != activeMode || resetRequested.exchange(false, std::memory_order_relaxed))
        {
            for (auto& channel : inputHistory) channel.fill(0.0f);
            historyIndex = 0;
            room.Reset();
            smoothedYaw = 0.0f;
            smoothedPitch = 0.0f;
            activeMode = requested;
        }

        if (activeMode == Mode::Off || hrtf == nullptr)
        {
            if (outputLeft != inputLeft)
            {
                std::memmove(outputLeft, inputLeft, frameCount * sizeof(float));
            }
            if (outputRight != inputRight)
            {
                std::memmove(outputRight, inputRight, frameCount * sizeof(float));
            }
            return;
        }

        const float targetYaw = activeMode == Mode::Tracked
            ? requestedYaw.load(std::memory_order_relaxed)
            : 0.0f;
        const float targetPitch = activeMode == Mode::Tracked
            ? requestedPitch.load(std::memory_order_relaxed)
            : 0.0f;
        const float blockSeconds = static_cast<float>(frameCount) / sampleRate;
        const float smoothing = 1.0f - std::exp(-blockSeconds / 0.020f);
        smoothedYaw += WrapDegrees(targetYaw - smoothedYaw) * smoothing;
        smoothedPitch += (targetPitch - smoothedPitch) * smoothing;
        UpdateFilters(
            smoothedYaw,
            smoothedPitch,
            requestedStageWidth.load(std::memory_order_relaxed),
            smoothing);

        const float roomAmount = requestedRoomReflection.load(std::memory_order_relaxed);
        const float hrtfBlend = requestedHrtfBlend.load(std::memory_order_relaxed);
        for (std::size_t frame = 0; frame < frameCount; frame++)
        {
            inputHistory[0][historyIndex] = inputLeft[frame];
            inputHistory[1][historyIndex] = inputRight[frame];
            float spatialLeft = 0.0f;
            float spatialRight = 0.0f;
            std::size_t historyPosition = historyIndex;
            for (std::size_t tap = 0; tap < activeTapCount; tap++)
            {
                spatialLeft += inputHistory[0][historyPosition] * currentFilters[0][tap] +
                    inputHistory[1][historyPosition] * currentFilters[2][tap];
                spatialRight += inputHistory[0][historyPosition] * currentFilters[1][tap] +
                    inputHistory[1][historyPosition] * currentFilters[3][tap];
                historyPosition = historyPosition == 0 ? kMaximumHrtfTaps - 1 : historyPosition - 1;
            }

            const float directLeft = spatialLeft * kSpatialGain;
            const float directRight = spatialRight * kSpatialGain;
            const auto wet = room.Process(directLeft, directRight);
            const float spatialOutLeft = directLeft + roomAmount * wet[0];
            const float spatialOutRight = directRight + roomAmount * wet[1];
            outputLeft[frame] = inputLeft[frame] + hrtfBlend * (spatialOutLeft - inputLeft[frame]);
            outputRight[frame] = inputRight[frame] + hrtfBlend * (spatialOutRight - inputRight[frame]);
            historyIndex = (historyIndex + 1) % kMaximumHrtfTaps;
        }
    }

    float sampleRate;
    RoomModel room;
    std::shared_ptr<SharedHrtf> hrtf;
    std::shared_ptr<const HrtfProfile> activeProfile;
    std::atomic<std::shared_ptr<const HrtfSelection>> requestedHrtf{};
    std::atomic<int> requestedMode{static_cast<int>(Mode::Off)};
    std::atomic<bool> resetRequested{false};
    std::atomic<float> requestedYaw{0.0f};
    std::atomic<float> requestedPitch{0.0f};
    std::atomic<float> requestedStageWidth{68.0f};
    std::atomic<float> requestedRoomReflection{0.08f};
    std::atomic<float> requestedHrtfBlend{0.85f};
    Mode activeMode = Mode::Off;
    float smoothedYaw = 0.0f;
    float smoothedPitch = 0.0f;
    std::array<std::array<float, kMaximumHrtfTaps>, 2> inputHistory{};
    std::array<std::array<float, kMaximumHrtfTaps>, 4> currentFilters{};
    std::array<std::array<float, kMaximumHrtfTaps>, 4> targetFilters{};
    std::size_t historyIndex = 0;
    std::size_t activeTapCount = 0;
    std::size_t targetTapCount = 0;
    bool filtersInitialized = false;
};

Spatializer::Spatializer(float sampleRate)
    : impl_(std::make_unique<Impl>(sampleRate))
{
}

Spatializer::~Spatializer() = default;

void Spatializer::SetMode(Mode mode)
{
    impl_->requestedMode.store(static_cast<int>(mode), std::memory_order_relaxed);
}

void Spatializer::SetPose(Pose pose)
{
    const float yaw = std::isfinite(pose.yawDegrees)
        ? std::clamp(pose.yawDegrees, -120.0f, 120.0f)
        : 0.0f;
    const float pitch = std::isfinite(pose.pitchDegrees)
        ? std::clamp(pose.pitchDegrees, -45.0f, 45.0f)
        : 0.0f;
    impl_->requestedYaw.store(yaw, std::memory_order_relaxed);
    impl_->requestedPitch.store(pitch, std::memory_order_relaxed);
}

void Spatializer::SetStageWidth(float degrees)
{
    impl_->requestedStageWidth.store(
        std::isfinite(degrees) ? std::clamp(degrees, 20.0f, 120.0f) : 68.0f,
        std::memory_order_relaxed);
}

void Spatializer::SetRoomReflection(float amount)
{
    impl_->requestedRoomReflection.store(
        std::isfinite(amount) ? std::clamp(amount, 0.0f, 0.35f) : 0.08f,
        std::memory_order_relaxed);
}

void Spatializer::SetHrtfBlend(float amount)
{
    impl_->requestedHrtfBlend.store(
        std::isfinite(amount) ? std::clamp(amount, 0.0f, 1.0f) : 0.85f,
        std::memory_order_relaxed);
}

void Spatializer::SetHrtfProfile(std::shared_ptr<const HrtfProfile> profile)
{
    if (profile == nullptr || profile->impl_ == nullptr || profile->impl_->hrtf == nullptr) return;
    auto selection = std::make_shared<HrtfSelection>();
    selection->profile = std::move(profile);
    selection->hrtf = selection->profile->impl_->hrtf;
    impl_->requestedHrtf.store(std::move(selection), std::memory_order_relaxed);
}

void Spatializer::Reset()
{
    impl_->resetRequested.store(true, std::memory_order_relaxed);
}

bool Spatializer::IsHrtfReady() const
{
    return impl_->hrtf != nullptr && impl_->hrtf->easy != nullptr;
}

void Spatializer::Process(
    const float* inputLeft,
    const float* inputRight,
    float* outputLeft,
    float* outputRight,
    std::size_t frameCount)
{
    impl_->Process(inputLeft, inputRight, outputLeft, outputRight, frameCount);
}
}