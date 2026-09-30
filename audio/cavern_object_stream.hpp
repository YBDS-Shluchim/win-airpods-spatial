#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace MagicAapSpatial
{
struct CavernObjectFrame
{
    bool isLfe = false;
    bool isDynamic = false;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    std::vector<float> samples;
};

struct CavernObjectBlock
{
    int frameCount = 0;
    std::vector<CavernObjectFrame> objects;
};

class CavernObjectStream final
{
public:
    explicit CavernObjectStream(const std::filesystem::path& mediaPath);
    ~CavernObjectStream();
    CavernObjectStream(const CavernObjectStream&) = delete;
    CavernObjectStream& operator=(const CavernObjectStream&) = delete;

    int SampleRate() const noexcept { return sampleRate_; }
    std::int64_t TotalFrames() const noexcept { return totalFrames_; }
    int ObjectCount() const noexcept { return objectCount_; }
    int DynamicObjectCount() const noexcept { return dynamicObjectCount_; }
    bool ReadBlock(CavernObjectBlock& block, std::chrono::milliseconds wait);
    bool IsFinished() const noexcept { return finished_.load(std::memory_order_relaxed); }
    std::string Error() const;
    void Stop();

private:
    void ReadLoop();

    void* process_ = nullptr;
    void* pipe_ = nullptr;
    std::atomic_bool stopRequested_{false};
    std::atomic_bool finished_{false};
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::deque<CavernObjectBlock> blocks_;
    std::string error_;
    std::thread readerThread_;
    int sampleRate_ = 0;
    int objectCount_ = 0;
    int dynamicObjectCount_ = 0;
    int blockFrames_ = 0;
    std::int64_t totalFrames_ = 0;
};
}