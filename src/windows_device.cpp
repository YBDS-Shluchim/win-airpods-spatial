#include "windows_device.hpp"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <objbase.h>
#include <setupapi.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace MagicAapSpatial
{
namespace
{
struct InterfaceClass
{
    const char* kind;
    GUID guid;
};

constexpr std::array<InterfaceClass, 3> kInterfaceClasses{{
    {"private", {0x9eec98bb, 0x3c54, 0x45d4, {0xa8, 0x43, 0x79, 0x00, 0xc4, 0x63, 0x5e, 0x08}}},
    {"service", {0x74ec2172, 0x0bad, 0x4d01, {0x8f, 0x77, 0x99, 0x7b, 0x2b, 0xe0, 0x72, 0x2a}}},
    {"client", {0x1ff31936, 0x572e, 0x4b36, {0xa2, 0xbf, 0xb2, 0x40, 0x9b, 0x1a, 0xa6, 0xf4}}},
}};

std::string FormatGuid(const GUID& guid)
{
    wchar_t text[40]{};
    if (StringFromGUID2(guid, text, static_cast<int>(std::size(text))) == 0)
    {
        return "{unknown}";
    }
    std::string result;
    for (const auto* current = text; *current != L'\0'; current++)
    {
        if (*current <= 0x7f) result.push_back(static_cast<char>(*current));
    }
    return result;
}

std::string ErrorText(DWORD error)
{
    return "Win32 error " + std::to_string(error);
}

HANDLE OpenDeviceHandle(const std::wstring& path, DWORD access)
{
    return CreateFileW(
        path.c_str(),
        access,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        nullptr,
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,
        nullptr);
}

std::size_t CompleteOverlappedRead(
    HANDLE handle,
    OVERLAPPED& operation,
    const std::atomic_bool& stopRequested,
    std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!stopRequested.load(std::memory_order_relaxed) &&
        std::chrono::steady_clock::now() < deadline)
    {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        const auto waitMilliseconds = static_cast<DWORD>(std::clamp<std::int64_t>(
            remaining.count(), 1, 100));
        const DWORD result = WaitForSingleObject(operation.hEvent, waitMilliseconds);
        if (result == WAIT_OBJECT_0)
        {
            DWORD bytesRead = 0;
            if (!GetOverlappedResult(handle, &operation, &bytesRead, FALSE))
            {
                const DWORD error = GetLastError();
                if (error == ERROR_OPERATION_ABORTED) return 0;
                throw std::runtime_error("MagicAAP read completion failed: " + ErrorText(error));
            }
            return bytesRead;
        }
        if (result == WAIT_FAILED)
        {
            throw std::runtime_error("MagicAAP read wait failed: " + ErrorText(GetLastError()));
        }
    }

    CancelIoEx(handle, &operation);
    WaitForSingleObject(operation.hEvent, INFINITE);
    DWORD ignored = 0;
    GetOverlappedResult(handle, &operation, &ignored, FALSE);
    return 0;
}

void CompleteOverlappedWrite(HANDLE handle, OVERLAPPED& operation, DWORD& bytesWritten)
{
    if (WaitForSingleObject(operation.hEvent, INFINITE) != WAIT_OBJECT_0)
    {
        throw std::runtime_error("MagicAAP write wait failed: " + ErrorText(GetLastError()));
    }
    if (!GetOverlappedResult(handle, &operation, &bytesWritten, FALSE))
    {
        throw std::runtime_error("MagicAAP write completion failed: " + ErrorText(GetLastError()));
    }
}
}

std::vector<DeviceCandidate> EnumerateMagicAapInterfaces()
{
    std::vector<DeviceCandidate> candidates;
    for (const auto& interfaceClass : kInterfaceClasses)
    {
        const HDEVINFO deviceInfoSet = SetupDiGetClassDevsW(
            &interfaceClass.guid,
            nullptr,
            nullptr,
            DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
        if (deviceInfoSet == INVALID_HANDLE_VALUE)
        {
            throw std::runtime_error("SetupDiGetClassDevsW failed: " + ErrorText(GetLastError()));
        }

        for (DWORD index = 0;; index++)
        {
            SP_DEVICE_INTERFACE_DATA interfaceData{};
            interfaceData.cbSize = sizeof(interfaceData);
            if (!SetupDiEnumDeviceInterfaces(
                    deviceInfoSet,
                    nullptr,
                    &interfaceClass.guid,
                    index,
                    &interfaceData))
            {
                const DWORD error = GetLastError();
                if (error != ERROR_NO_MORE_ITEMS)
                {
                    SetupDiDestroyDeviceInfoList(deviceInfoSet);
                    throw std::runtime_error("SetupDiEnumDeviceInterfaces failed: " + ErrorText(error));
                }
                break;
            }

            DWORD requiredSize = 0;
            SetupDiGetDeviceInterfaceDetailW(
                deviceInfoSet,
                &interfaceData,
                nullptr,
                0,
                &requiredSize,
                nullptr);
            if (GetLastError() != ERROR_INSUFFICIENT_BUFFER || requiredSize == 0)
            {
                const DWORD error = GetLastError();
                SetupDiDestroyDeviceInfoList(deviceInfoSet);
                throw std::runtime_error("SetupDiGetDeviceInterfaceDetailW(size) failed: " + ErrorText(error));
            }

            std::vector<std::byte> detailBuffer(requiredSize);
            auto* detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W*>(detailBuffer.data());
            detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
            if (!SetupDiGetDeviceInterfaceDetailW(
                    deviceInfoSet,
                    &interfaceData,
                    detail,
                    requiredSize,
                    &requiredSize,
                    nullptr))
            {
                const DWORD error = GetLastError();
                SetupDiDestroyDeviceInfoList(deviceInfoSet);
                throw std::runtime_error("SetupDiGetDeviceInterfaceDetailW(path) failed: " + ErrorText(error));
            }

            candidates.push_back({
                interfaceClass.kind,
                FormatGuid(interfaceClass.guid),
                detail->DevicePath});
        }
        SetupDiDestroyDeviceInfoList(deviceInfoSet);
    }
    return candidates;
}

bool CanOpenDevice(const std::wstring& path, std::string& error)
{
    const HANDLE handle = OpenDeviceHandle(path, 0);
    if (handle == INVALID_HANDLE_VALUE)
    {
        error = ErrorText(GetLastError());
        return false;
    }
    CloseHandle(handle);
    error.clear();
    return true;
}

MagicAapDevice::MagicAapDevice(const std::wstring& path)
{
    const HANDLE handle = OpenDeviceHandle(path, GENERIC_READ | GENERIC_WRITE);
    if (handle == INVALID_HANDLE_VALUE)
    {
        throw std::runtime_error("Could not open MagicAAP device: " + ErrorText(GetLastError()));
    }
    handle_ = handle;
}

MagicAapDevice::~MagicAapDevice()
{
    if (handle_ != nullptr) CloseHandle(static_cast<HANDLE>(handle_));
}

std::size_t MagicAapDevice::ReadSome(
    std::span<std::uint8_t> buffer,
    const std::atomic_bool& stopRequested,
    std::chrono::milliseconds timeout)
{
    auto* handle = static_cast<HANDLE>(handle_);
    OVERLAPPED operation{};
    operation.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    if (operation.hEvent == nullptr)
    {
        throw std::runtime_error("Could not create MagicAAP read event: " + ErrorText(GetLastError()));
    }

    DWORD bytesRead = 0;
    const BOOL completed = ReadFile(
        handle,
        buffer.data(),
        static_cast<DWORD>(buffer.size()),
        &bytesRead,
        &operation);
    if (!completed)
    {
        const DWORD error = GetLastError();
        if (error != ERROR_IO_PENDING)
        {
            CloseHandle(operation.hEvent);
            if (error == ERROR_OPERATION_ABORTED) return 0;
            throw std::runtime_error("MagicAAP read failed: " + ErrorText(error));
        }
        try
        {
            bytesRead = static_cast<DWORD>(CompleteOverlappedRead(handle, operation, stopRequested, timeout));
        }
        catch (...)
        {
            CloseHandle(operation.hEvent);
            throw;
        }
    }
    CloseHandle(operation.hEvent);
    return bytesRead;
}

void MagicAapDevice::Write(std::span<const std::uint8_t> packet)
{
    auto* handle = static_cast<HANDLE>(handle_);
    std::size_t offset = 0;
    while (offset < packet.size())
    {
        OVERLAPPED operation{};
        operation.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
        if (operation.hEvent == nullptr)
        {
            throw std::runtime_error("Could not create MagicAAP write event: " + ErrorText(GetLastError()));
        }

        DWORD bytesWritten = 0;
        const BOOL completed = WriteFile(
            handle,
            packet.data() + offset,
            static_cast<DWORD>(packet.size() - offset),
            &bytesWritten,
            &operation);
        if (!completed)
        {
            const DWORD error = GetLastError();
            if (error != ERROR_IO_PENDING)
            {
                CloseHandle(operation.hEvent);
                throw std::runtime_error("MagicAAP write failed: " + ErrorText(error));
            }
            try
            {
                CompleteOverlappedWrite(handle, operation, bytesWritten);
            }
            catch (...)
            {
                CloseHandle(operation.hEvent);
                throw;
            }
        }
        CloseHandle(operation.hEvent);
        if (bytesWritten == 0)
        {
            throw std::runtime_error("MagicAAP accepted a zero-length write.");
        }
        offset += bytesWritten;
    }
}
}
#else
#include <stdexcept>

namespace MagicAapSpatial
{
std::vector<DeviceCandidate> EnumerateMagicAapInterfaces()
{
    return {};
}

bool CanOpenDevice(const std::wstring&, std::string& error)
{
    error = "MagicAAP device access is only supported on Windows.";
    return false;
}

MagicAapDevice::MagicAapDevice(const std::wstring&)
{
    throw std::runtime_error("MagicAAP device access is only supported on Windows.");
}

MagicAapDevice::~MagicAapDevice() = default;

std::size_t MagicAapDevice::ReadSome(
    std::span<std::uint8_t>,
    const std::atomic_bool&,
    std::chrono::milliseconds)
{
    throw std::runtime_error("MagicAAP device access is only supported on Windows.");
}

void MagicAapDevice::Write(std::span<const std::uint8_t>)
{
    throw std::runtime_error("MagicAAP device access is only supported on Windows.");
}
}
#endif