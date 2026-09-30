#include "cavern_object_stream.hpp"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>
#include <string>

namespace MagicAapSpatial
{
namespace
{
std::wstring QuoteArgument(const std::wstring& argument)
{
    std::wstring quoted = L"\"";
    std::size_t backslashes = 0;
    for (const auto character : argument)
    {
        if (character == L'\\')
        {
            backslashes++;
            continue;
        }
        if (character == L'"')
        {
            quoted.append(backslashes * 2 + 1, L'\\');
            quoted.push_back(character);
            backslashes = 0;
            continue;
        }
        quoted.append(backslashes, L'\\');
        backslashes = 0;
        quoted.push_back(character);
    }
    quoted.append(backslashes * 2, L'\\');
    quoted.push_back(L'"');
    return quoted;
}

bool ReadExact(HANDLE pipe, void* destination, std::size_t size)
{
    auto* output = static_cast<std::uint8_t*>(destination);
    std::size_t offset = 0;
    while (offset < size)
    {
        DWORD bytesRead = 0;
        const auto remaining = static_cast<DWORD>(std::min<std::size_t>(size - offset, MAXDWORD));
        if (!ReadFile(pipe, output + offset, remaining, &bytesRead, nullptr))
        {
            const DWORD error = GetLastError();
            if (error == ERROR_BROKEN_PIPE || error == ERROR_HANDLE_EOF) return false;
            throw std::runtime_error("Atmos decoder pipe read failed (Win32 " + std::to_string(error) + ").");
        }
        if (bytesRead == 0) return false;
        offset += bytesRead;
    }
    return true;
}

template <typename Value>
Value ReadValue(HANDLE pipe)
{
    Value value{};
    if (!ReadExact(pipe, &value, sizeof(value)))
    {
        throw std::runtime_error("Atmos decoder ended in the middle of an object frame.");
    }
    return value;
}
}

CavernObjectStream::CavernObjectStream(const std::filesystem::path& mediaPath)
{
    SECURITY_ATTRIBUTES security{sizeof(security), nullptr, TRUE};
    HANDLE childOutput = nullptr;
    HANDLE parentOutput = nullptr;
    if (!CreatePipe(&parentOutput, &childOutput, &security, 64 * 1024))
    {
        throw std::runtime_error("Could not create the Atmos decoder output pipe.");
    }
    if (!SetHandleInformation(parentOutput, HANDLE_FLAG_INHERIT, 0))
    {
        CloseHandle(parentOutput);
        CloseHandle(childOutput);
        throw std::runtime_error("Could not configure the Atmos decoder output pipe.");
    }

    HANDLE childInput = CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, &security, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    HANDLE childError = CreateFileW(L"NUL", GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, &security, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (childInput == INVALID_HANDLE_VALUE || childError == INVALID_HANDLE_VALUE)
    {
        if (childInput != INVALID_HANDLE_VALUE) CloseHandle(childInput);
        if (childError != INVALID_HANDLE_VALUE) CloseHandle(childError);
        CloseHandle(parentOutput);
        CloseHandle(childOutput);
        throw std::runtime_error("Could not open null handles for the Atmos decoder.");
    }

    std::array<wchar_t, 32768> modulePath{};
    const DWORD moduleLength = GetModuleFileNameW(nullptr, modulePath.data(), static_cast<DWORD>(modulePath.size()));
    if (moduleLength == 0 || moduleLength >= modulePath.size())
    {
        CloseHandle(childInput);
        CloseHandle(childError);
        CloseHandle(parentOutput);
        CloseHandle(childOutput);
        throw std::runtime_error("Could not locate the player executable.");
    }
    const auto helperPath = std::filesystem::path(std::wstring(modulePath.data(), moduleLength)).parent_path() / L"AtmosDecoder.exe";
    std::wstring commandLine = QuoteArgument(helperPath.wstring()) + L" " + QuoteArgument(mediaPath.wstring());
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
    startup.wShowWindow = SW_HIDE;
    startup.hStdInput = childInput;
    startup.hStdOutput = childOutput;
    startup.hStdError = childError;
    PROCESS_INFORMATION processInfo{};
    const BOOL created = CreateProcessW(
        helperPath.c_str(),
        commandLine.data(),
        nullptr,
        nullptr,
        TRUE,
        CREATE_NO_WINDOW,
        nullptr,
        helperPath.parent_path().c_str(),
        &startup,
        &processInfo);
    CloseHandle(childInput);
    CloseHandle(childError);
    CloseHandle(childOutput);
    if (!created)
    {
        CloseHandle(parentOutput);
        throw std::runtime_error("Could not launch AtmosDecoder.exe (Win32 " + std::to_string(GetLastError()) + ").");
    }
    CloseHandle(processInfo.hThread);
    process_ = processInfo.hProcess;
    pipe_ = parentOutput;

    try
    {
        std::array<char, 8> magic{};
        if (!ReadExact(static_cast<HANDLE>(pipe_), magic.data(), magic.size()) ||
            magic != std::array<char, 8>{'C', 'A', 'V', 'O', 'B', 'J', '0', '1'})
        {
            throw std::runtime_error("AtmosDecoder did not send a valid CAVOBJ01 header.");
        }
        sampleRate_ = ReadValue<std::int32_t>(static_cast<HANDLE>(pipe_));
        objectCount_ = ReadValue<std::int32_t>(static_cast<HANDLE>(pipe_));
        dynamicObjectCount_ = ReadValue<std::int32_t>(static_cast<HANDLE>(pipe_));
        totalFrames_ = ReadValue<std::int64_t>(static_cast<HANDLE>(pipe_));
        blockFrames_ = ReadValue<std::int32_t>(static_cast<HANDLE>(pipe_));
        if (sampleRate_ <= 0 || objectCount_ <= 0 || objectCount_ > 256 ||
            dynamicObjectCount_ < 0 || dynamicObjectCount_ > objectCount_ ||
            totalFrames_ <= 0 || blockFrames_ <= 0 || blockFrames_ > 4096)
        {
            throw std::runtime_error("AtmosDecoder sent invalid stream dimensions.");
        }
        readerThread_ = std::thread(&CavernObjectStream::ReadLoop, this);
    }
    catch (...)
    {
        Stop();
        throw;
    }
}

CavernObjectStream::~CavernObjectStream()
{
    Stop();
}

bool CavernObjectStream::ReadBlock(CavernObjectBlock& block, std::chrono::milliseconds wait)
{
    std::unique_lock lock(mutex_);
    changed_.wait_for(lock, wait, [&]
    {
        return !blocks_.empty() || finished_.load(std::memory_order_relaxed) ||
            stopRequested_.load(std::memory_order_relaxed);
    });
    if (blocks_.empty()) return false;
    block = std::move(blocks_.front());
    blocks_.pop_front();
    lock.unlock();
    changed_.notify_all();
    return true;
}

std::string CavernObjectStream::Error() const
{
    std::lock_guard lock(mutex_);
    return error_;
}

void CavernObjectStream::Stop()
{
    stopRequested_.store(true, std::memory_order_relaxed);
    changed_.notify_all();
    if (process_ != nullptr) TerminateProcess(static_cast<HANDLE>(process_), 1);
    if (readerThread_.joinable()) readerThread_.join();
    if (pipe_ != nullptr)
    {
        CloseHandle(static_cast<HANDLE>(pipe_));
        pipe_ = nullptr;
    }
    if (process_ != nullptr)
    {
        WaitForSingleObject(static_cast<HANDLE>(process_), INFINITE);
        CloseHandle(static_cast<HANDLE>(process_));
        process_ = nullptr;
    }
}

void CavernObjectStream::ReadLoop()
{
    try
    {
        while (!stopRequested_.load(std::memory_order_relaxed))
        {
            std::int32_t frameCount = 0;
            if (!ReadExact(static_cast<HANDLE>(pipe_), &frameCount, sizeof(frameCount))) break;
            if (frameCount <= 0 || frameCount > blockFrames_)
            {
                throw std::runtime_error("AtmosDecoder sent an invalid object block length.");
            }

            CavernObjectBlock block;
            block.frameCount = frameCount;
            block.objects.resize(static_cast<std::size_t>(objectCount_));
            for (auto& object : block.objects)
            {
                const auto flags = ReadValue<std::int32_t>(static_cast<HANDLE>(pipe_));
                object.isLfe = (flags & 1) != 0;
                object.isDynamic = (flags & 2) != 0;
                object.x = ReadValue<float>(static_cast<HANDLE>(pipe_));
                object.y = ReadValue<float>(static_cast<HANDLE>(pipe_));
                object.z = ReadValue<float>(static_cast<HANDLE>(pipe_));
                object.samples.resize(static_cast<std::size_t>(frameCount));
                if (!ReadExact(
                        static_cast<HANDLE>(pipe_),
                        object.samples.data(),
                        object.samples.size() * sizeof(float)))
                {
                    throw std::runtime_error("AtmosDecoder ended in the middle of object PCM.");
                }
            }

            std::unique_lock lock(mutex_);
            changed_.wait(lock, [&]
            {
                return blocks_.size() < 8 || stopRequested_.load(std::memory_order_relaxed);
            });
            if (stopRequested_.load(std::memory_order_relaxed)) break;
            blocks_.push_back(std::move(block));
            lock.unlock();
            changed_.notify_all();
        }
    }
    catch (const std::exception& error)
    {
        {
            std::lock_guard lock(mutex_);
            error_ = error.what();
        }
        finished_.store(true, std::memory_order_relaxed);
        changed_.notify_all();
        return;
    }

    if (!stopRequested_.load(std::memory_order_relaxed) && process_ != nullptr)
    {
        WaitForSingleObject(static_cast<HANDLE>(process_), INFINITE);
        DWORD exitCode = 0;
        if (GetExitCodeProcess(static_cast<HANDLE>(process_), &exitCode) && exitCode != 0)
        {
            std::lock_guard lock(mutex_);
            error_ = "AtmosDecoder exited with code " + std::to_string(exitCode) + ".";
        }
    }
    finished_.store(true, std::memory_order_relaxed);
    changed_.notify_all();
}
}
#else
#include <stdexcept>

namespace MagicAapSpatial
{
CavernObjectStream::CavernObjectStream(const std::filesystem::path&)
{
    throw std::runtime_error("Cavern Atmos object rendering is available only on Windows.");
}

CavernObjectStream::~CavernObjectStream() = default;

bool CavernObjectStream::ReadBlock(CavernObjectBlock&, std::chrono::milliseconds)
{
    return false;
}

std::string CavernObjectStream::Error() const
{
    return "Cavern Atmos object rendering is available only on Windows.";
}

void CavernObjectStream::Stop() { }
}
#endif