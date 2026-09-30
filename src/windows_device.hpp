#pragma once

#include "core.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace MagicAapSpatial
{
struct DeviceCandidate
{
    std::string kind;
    std::string guid;
    std::wstring path;
};

std::vector<DeviceCandidate> EnumerateMagicAapInterfaces();
bool CanOpenDevice(const std::wstring& path, std::string& error);

class MagicAapDevice final
{
public:
    explicit MagicAapDevice(const std::wstring& path);
    ~MagicAapDevice();
    MagicAapDevice(const MagicAapDevice&) = delete;
    MagicAapDevice& operator=(const MagicAapDevice&) = delete;

    std::size_t ReadSome(
        std::span<std::uint8_t> buffer,
        const std::atomic_bool& stopRequested,
        std::chrono::milliseconds timeout);
    void Write(std::span<const std::uint8_t> packet);

private:
    void* handle_ = nullptr;
};
}