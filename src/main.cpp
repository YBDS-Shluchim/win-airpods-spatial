#include "core.hpp"
#include "media_player.hpp"
#include "windows_device.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace
{
using namespace MagicAapSpatial;
using Clock = std::chrono::steady_clock;

std::atomic_bool g_stopRequested{false};

struct StartProfile
{
    const char* name;
    const char* description;
    Packet start;
    Packet stop;
};

Packet HexBytes(const std::string& hex);

const std::array<StartProfile, 3> kProfiles{{
    {
        "alternate",
        "alternate AAP head-tracking request",
        HexBytes("040004001700000010000f000873420b081010021a0501409c0000"),
        HexBytes("040004001700000010000f000875420b081010021a050100000000"),
    },
    {
        "devmotion6",
        "RTBuddy DEVMOTION6 service 16 at 50 Hz",
        HexBytes("040004001700000010001000089301420b081010021a0501204e0000"),
        HexBytes("040004001700000010001000089501420b081010021a050100000000"),
    },
    {
        "max2",
        "AirPods Max 2 firmware 8E258, DEVMOTION6 service 6",
        HexBytes("040004001700000010001000089801420b080610021a0501204e0000"),
        HexBytes("040004001700000010001000089a01420b080610021a050100000000"),
    },
}};

const std::array<Packet, 4> kRtBuddyInitPackets{{
    HexBytes("00000000010003000000000000000000"),
    HexBytes("04000000010000"),
    HexBytes("00000400010003000000000000000000"),
    HexBytes("04000400010000"),
}};

const Packet kOwnsConnection = HexBytes("0400040009000601000000");

Packet HexBytes(const std::string& hex)
{
    if (hex.size() % 2 != 0) throw std::invalid_argument("hex packet must have an even number of characters");
    Packet bytes;
    bytes.reserve(hex.size() / 2);
    for (std::size_t index = 0; index < hex.size(); index += 2)
    {
        bytes.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(index, 2), nullptr, 16)));
    }
    return bytes;
}

std::string ToHex(std::span<const std::uint8_t> bytes, std::size_t maximum = 24)
{
    std::ostringstream output;
    output << std::uppercase << std::hex << std::setfill('0');
    const auto count = std::min(bytes.size(), maximum);
    for (std::size_t index = 0; index < count; index++)
    {
        output << std::setw(2) << static_cast<unsigned>(bytes[index]);
    }
    if (bytes.size() > count) output << "...";
    return output.str();
}

void SignalHandler(int)
{
    g_stopRequested.store(true, std::memory_order_relaxed);
}

#ifdef _WIN32
BOOL WINAPI ConsoleControlHandler(DWORD event)
{
    if (event == CTRL_C_EVENT || event == CTRL_BREAK_EVENT || event == CTRL_CLOSE_EVENT)
    {
        g_stopRequested.store(true, std::memory_order_relaxed);
        return TRUE;
    }
    return FALSE;
}

std::string Narrow(const std::wstring& value)
{
    if (value.empty()) return {};
    const int count = WideCharToMultiByte(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), nullptr, 0, nullptr, nullptr);
    if (count <= 0) return "<unprintable device path>";
    std::string result(static_cast<std::size_t>(count), '\0');
    WideCharToMultiByte(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), result.data(), count, nullptr, nullptr);
    return result;
}
#else
std::string Narrow(const std::wstring& value)
{
    return std::string(value.begin(), value.end());
}
#endif

void PrintHelp()
{
    std::cout << R"(Magic AAP Spatial

Commands:
  magic-aap-spatial devices
  magic-aap-spatial probe [--interface auto|private|service] [--path DEVICE_PATH] [--seconds N]
  magic-aap-spatial track [--interface auto|private|service] [--path DEVICE_PATH] [--seconds N]
      [--start-format auto|alternate|devmotion6|max2] [--probe-ms N] [--no-init] [--no-takeover]
    magic-aap-spatial play <file> [--spatial off|fixed]
  magic-aap-spatial audio

Tracking defaults to auto profile probing and selects a profile only after valid motion packets arrive.
MagicAAP device access requires Windows. The Max 2 service-6 request is the final automatic fallback.
play decodes a file and sends only app-owned audio to the default shared-mode output.
It does not capture or reroute other applications' audio.
)";
}

struct Options
{
    std::string command;
    std::string interfacePreference = "auto";
    std::optional<std::wstring> path;
    std::string startFormat = "auto";
    std::optional<std::filesystem::path> mediaPath;
    PlaybackSpatialMode spatialMode = PlaybackSpatialMode::Fixed;
    double seconds = 0.0;
    int probeMilliseconds = 1800;
    bool initialize = true;
    bool takeOwnership = true;
};

double ParseSeconds(const std::string& value)
{
    std::size_t parsed = 0;
    const double seconds = std::stod(value, &parsed);
    if (parsed != value.size() || seconds < 0.0 || !std::isfinite(seconds))
    {
        throw std::invalid_argument("seconds must be a non-negative number");
    }
    return seconds;
}

int ParseMilliseconds(const std::string& value)
{
    std::size_t parsed = 0;
    const int milliseconds = std::stoi(value, &parsed);
    if (parsed != value.size() || milliseconds < 100 || milliseconds > 10000)
    {
        throw std::invalid_argument("probe milliseconds must be from 100 to 10000");
    }
    return milliseconds;
}

Options ParseOptions(int argc, char** argv)
{
    if (argc < 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")
    {
        return {};
    }

    Options options;
    options.command = argv[1];
    int firstOption = 2;
    if (options.command == "play")
    {
        if (argc < 3 || std::string(argv[2]).starts_with("--"))
        {
            throw std::invalid_argument("play requires an audio file path");
        }
        options.mediaPath = std::filesystem::path(argv[2]);
        firstOption = 3;
    }

    for (int index = firstOption; index < argc; index++)
    {
        const std::string argument = argv[index];
        auto value = [&]() -> std::string
        {
            if (index + 1 >= argc) throw std::invalid_argument("missing value after " + argument);
            return argv[++index];
        };

        if (argument == "--interface") options.interfacePreference = value();
        else if (argument == "--path")
        {
            const auto text = value();
            options.path = std::wstring(text.begin(), text.end());
        }
        else if (argument == "--start-format") options.startFormat = value();
        else if (argument == "--seconds") options.seconds = ParseSeconds(value());
        else if (argument == "--probe-ms") options.probeMilliseconds = ParseMilliseconds(value());
        else if (argument == "--spatial")
        {
            const auto spatial = value();
            if (spatial == "off") options.spatialMode = PlaybackSpatialMode::Off;
            else if (spatial == "fixed") options.spatialMode = PlaybackSpatialMode::Fixed;
            else throw std::invalid_argument("spatial mode must be off or fixed");
        }
        else if (argument == "--no-init") options.initialize = false;
        else if (argument == "--no-takeover") options.takeOwnership = false;
        else throw std::invalid_argument("unknown option: " + argument);
    }

    if (options.interfacePreference != "auto" &&
        options.interfacePreference != "private" &&
        options.interfacePreference != "service")
    {
        throw std::invalid_argument("interface must be auto, private, or service");
    }
    return options;
}

const StartProfile* FindProfile(const std::string& name)
{
    for (const auto& profile : kProfiles)
    {
        if (name == profile.name) return &profile;
    }
    return nullptr;
}

std::optional<DeviceCandidate> ChooseDevice(const Options& options)
{
    if (options.path)
    {
        return DeviceCandidate{"explicit", "", *options.path};
    }

    auto candidates = EnumerateMagicAapInterfaces();
    std::erase_if(candidates, [&](const DeviceCandidate& candidate)
    {
        if (options.interfacePreference == "auto")
        {
            return candidate.kind != "private" && candidate.kind != "service";
        }
        return candidate.kind != options.interfacePreference;
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
    return std::nullopt;
}

void ProcessPacket(
    std::span<const std::uint8_t> packet,
    bool tracking,
    PoseEstimator& estimator,
    std::size_t& packets,
    std::size_t& headPackets)
{
    packets++;
    if (!tracking)
    {
        std::cout << "len=" << packet.size() << " " << ToHex(packet) << '\n';
        return;
    }

    const auto pose = estimator.ProcessPacket(packet);
    if (!pose) return;
    if (!pose->calibrated)
    {
        headPackets++;
        if (headPackets % 5 == 0)
        {
            std::cout << "\rCalibrating " << pose->calibrationCount << "/25 motion packets" << std::flush;
        }
        return;
    }
    if (!pose->yawDegrees)
    {
        std::cout << "\rCalibration complete. Move your head to see pose." << std::flush;
        return;
    }

    headPackets++;
    std::cout << '\r' << std::fixed << std::setprecision(1)
              << "Yaw " << std::setw(7) << *pose->yawDegrees
              << "  Pitch " << std::setw(7) << *pose->pitchDegrees
              << "  Roll " << std::setw(7) << *pose->rollDegrees
              << "  packets " << headPackets << "   " << std::flush;
}

int RunReader(const Options& options, bool tracking)
{
    const auto* explicitProfile = options.startFormat == "auto"
        ? nullptr
        : FindProfile(options.startFormat);
    if (tracking && options.startFormat != "auto" && explicitProfile == nullptr)
    {
        throw std::invalid_argument("unknown start format; choose auto, alternate, devmotion6, or max2");
    }

    const auto candidate = ChooseDevice(options);
    if (!candidate)
    {
        throw std::runtime_error("No openable MagicAAP interface found. Run 'devices' to inspect interfaces.");
    }

    MagicAapDevice device(candidate->path);
    std::cout << "Opened " << candidate->kind << " MagicAAP interface.\n"
              << Narrow(candidate->path) << '\n';

    AapStreamFramer framer;
    PoseEstimator estimator;
    std::array<std::uint8_t, 4096> buffer{};
    std::vector<Packet> replayPackets;
    std::vector<Packet> pendingPackets;
    std::size_t chunks = 0;
    std::size_t packets = 0;
    std::size_t headPackets = 0;
    const auto startedAt = Clock::now();
    const auto runDeadline = options.seconds > 0.0
        ? startedAt + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(options.seconds))
        : Clock::time_point::max();
    const StartProfile* activeProfile = nullptr;

    try
    {
        if (tracking && options.initialize)
        {
            for (std::size_t index = 0; index < kRtBuddyInitPackets.size(); index++)
            {
                device.Write(kRtBuddyInitPackets[index]);
                if (index < kRtBuddyInitPackets.size() - 1)
                {
                    std::this_thread::sleep_for(index % 2 == 0
                        ? std::chrono::milliseconds(180)
                        : std::chrono::milliseconds(220));
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        if (tracking && options.takeOwnership) device.Write(kOwnsConnection);

        if (tracking)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(350));
            const std::vector<const StartProfile*> profiles = options.startFormat == "auto"
                ? std::vector<const StartProfile*>{&kProfiles[0], &kProfiles[1], &kProfiles[2]}
                : std::vector<const StartProfile*>{explicitProfile};

            for (const auto* profile : profiles)
            {
                if (g_stopRequested.load(std::memory_order_relaxed) || Clock::now() >= runDeadline) break;
                if (std::string(profile->name) == "max2")
                {
                    std::cerr << "Trying the Max 2 service-6 profile as a final fallback.\n";
                }
                device.Write(profile->start);
                activeProfile = profile;
                if (options.startFormat != "auto") break;

                const auto deadline = std::min(
                    runDeadline,
                    Clock::now() + std::chrono::milliseconds(options.probeMilliseconds));
                bool motionFound = false;
                while (!g_stopRequested.load(std::memory_order_relaxed) &&
                    Clock::now() < deadline && Clock::now() < runDeadline)
                {
                    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
                    const auto count = device.ReadSome(
                        buffer,
                        g_stopRequested,
                        std::max(std::chrono::milliseconds(1), std::min(std::chrono::milliseconds(100), remaining)));
                    if (count == 0) continue;
                    chunks++;
                    auto framed = framer.Push(std::span<const std::uint8_t>(buffer).first(count));
                    for (auto& packet : framed)
                    {
                        if (DecodeMotionPacket(packet)) motionFound = true;
                        replayPackets.push_back(std::move(packet));
                        if (motionFound) break;
                    }
                    if (motionFound) break;
                }
                if (motionFound) break;

                device.Write(profile->stop);
                activeProfile = nullptr;
            }

            if (activeProfile == nullptr &&
                !g_stopRequested.load(std::memory_order_relaxed) &&
                Clock::now() < runDeadline)
            {
                throw std::runtime_error("No valid motion packets detected from known profiles; AirPods firmware may use an unsupported protocol.");
            }
            if (activeProfile != nullptr)
            {
                std::cout << "Tracking profile: " << activeProfile->name << " (" << activeProfile->description << ")\n"
                          << "Hold still while the 25-sample neutral pose calibrates; press Ctrl+C to stop.\n";
            }
        }

        for (const auto& packet : replayPackets)
        {
            ProcessPacket(packet, tracking, estimator, packets, headPackets);
        }

        while (!g_stopRequested.load(std::memory_order_relaxed) && Clock::now() < runDeadline)
        {
            if (pendingPackets.empty())
            {
                const auto count = device.ReadSome(buffer, g_stopRequested, std::chrono::milliseconds(100));
                if (count == 0) continue;
                chunks++;
                pendingPackets = framer.Push(std::span<const std::uint8_t>(buffer).first(count));
            }
            if (!pendingPackets.empty())
            {
                ProcessPacket(pendingPackets.front(), tracking, estimator, packets, headPackets);
                pendingPackets.erase(pendingPackets.begin());
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
        try
        {
            device.Write(activeProfile->stop);
            std::cout << "\nSent tracking stop packet.\n";
        }
        catch (const std::exception& error)
        {
            std::cerr << "Could not send tracking stop packet: " << error.what() << '\n';
        }
    }

    std::cout << "\n" << (tracking ? "Tracking" : "Probe") << " summary: chunks=" << chunks
              << ", packets=" << packets << ", headPackets=" << headPackets << '\n';
    return tracking && headPackets == 0 ? 3 : 0;
}

int RunDevices()
{
    const auto candidates = EnumerateMagicAapInterfaces();
    if (candidates.empty())
    {
        std::cout << "No MagicAAP/AAP device interfaces were found.\n";
        return 2;
    }
    for (const auto& candidate : candidates)
    {
        std::string error;
        const bool openable = CanOpenDevice(candidate.path, error);
        std::cout << candidate.kind << ": " << candidate.guid << '\n'
                  << "  " << Narrow(candidate.path) << '\n'
                  << "  status: " << (openable ? "openable" : "not openable: " + error) << '\n';
    }
    return 0;
}
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, SignalHandler);
#ifdef _WIN32
    SetConsoleCtrlHandler(ConsoleControlHandler, TRUE);
#endif

    try
    {
        const auto options = ParseOptions(argc, argv);
        if (options.command.empty() || options.command == "help")
        {
            PrintHelp();
            return 0;
        }
        if (options.command == "devices") return RunDevices();
        if (options.command == "probe") return RunReader(options, false);
        if (options.command == "track") return RunReader(options, true);
        if (options.command == "play")
        {
            if (!options.mediaPath) throw std::invalid_argument("play requires an audio file path");
            MediaPlayer player;
            PlaybackSettings settings;
            settings.mode = options.spatialMode;
            return player.PlayFile(*options.mediaPath, settings, g_stopRequested);
        }
        if (options.command == "audio")
        {
            std::cout << "In-app file playback is available with 'play <file>'.\n"
                      << "Other applications' system audio is not captured or redirected.\n"
                      << "No virtual audio endpoint or custom audio driver is required.\n";
            return 0;
        }
        PrintHelp();
        return 64;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}