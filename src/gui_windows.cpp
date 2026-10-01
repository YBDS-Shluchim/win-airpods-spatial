#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>
#include <commctrl.h>

#include "media_player.hpp"
#include "head_tracker.hpp"

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <iterator>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace
{
using namespace MagicAapSpatial;

constexpr wchar_t kWindowClass[] = L"MagicAapSpatialPlayerWindow";
constexpr UINT kPlaybackFinished = WM_APP + 1;
constexpr UINT kTrackingStatusChanged = WM_APP + 2;
constexpr int kFilePathId = 1001;
constexpr int kBrowseId = 1002;
constexpr int kPlayId = 1003;
constexpr int kStopId = 1004;
constexpr int kProfileId = 1005;
constexpr int kWidthSliderId = 1006;
constexpr int kRoomSliderId = 1007;
constexpr int kWidthValueId = 1008;
constexpr int kRoomValueId = 1009;
constexpr int kStatusId = 1010;
constexpr int kDynamicLockId = 1011;
constexpr int kBedLockId = 1012;
constexpr int kReverseHeadTrackingId = 1013;
constexpr int kHrtfProfileId = 1014;
constexpr int kHrtfBrowseId = 1015;
constexpr int kHrtfBlendId = 1016;
constexpr int kHrtfBlendValueId = 1017;

struct PlayerWindow
{
    HWND window = nullptr;
    HWND filePath = nullptr;
    HWND playButton = nullptr;
    HWND stopButton = nullptr;
    HWND profile = nullptr;
    HWND hrtfProfile = nullptr;
    HWND hrtfBlend = nullptr;
    HWND widthSlider = nullptr;
    HWND roomSlider = nullptr;
    HWND widthValue = nullptr;
    HWND roomValue = nullptr;
    HWND hrtfBlendValue = nullptr;
    HWND dynamicLock = nullptr;
    HWND bedLock = nullptr;
    HWND reverseHeadTracking = nullptr;
    HWND status = nullptr;
    WPARAM activeTrackingProfile = 0;
    MediaPlayer player;
    std::filesystem::path selectedFile;
    std::vector<std::filesystem::path> hrtfPaths;
    std::vector<std::shared_ptr<const HrtfProfile>> loadedHrtfProfiles;
    std::atomic_bool stopRequested{false};
    std::atomic_bool trackingStopRequested{false};
    std::thread playbackThread;
    std::thread trackingThread;
    bool isPlaying = false;

    ~PlayerWindow()
    {
        StopPlayback();
    }

    void StopPlayback()
    {
        trackingStopRequested.store(true, std::memory_order_relaxed);
        stopRequested.store(true, std::memory_order_relaxed);
        if (playbackThread.joinable()) playbackThread.join();
        if (trackingThread.joinable()) trackingThread.join();
        isPlaying = false;
    }
};

void SetStatus(PlayerWindow& app, const std::wstring& text)
{
    SetWindowTextW(app.status, text.c_str());
}

void SetTrackingStatus(PlayerWindow& app, bool calibrated)
{
    const wchar_t* profile = nullptr;
    const wchar_t* rate = nullptr;
    switch (app.activeTrackingProfile)
    {
    case 4:
        profile = L"alternate";
        rate = L"25";
        break;
    case 5:
        profile = L"devmotion6";
        rate = L"50";
        break;
    case 6:
        profile = L"max2";
        rate = L"50";
        break;
    }

    if (profile == nullptr)
    {
        SetStatus(app, calibrated ? L"Head tracking active." : L"Calibrating head pose; hold still...");
        return;
    }

    const std::wstring detail = std::wstring(profile) + L" (" + rate + L" Hz requested)";
    SetStatus(app, calibrated
        ? std::wstring(L"Tracking active: ") + detail + L"."
        : std::wstring(L"Calibrating; ") + detail + L"...");
}

std::filesystem::path ExecutableDirectory()
{
    std::vector<wchar_t> path(32768);
    const auto length = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
    if (length == 0 || length >= path.size()) return {};
    return std::filesystem::path(std::wstring(path.data(), length)).parent_path();
}

void AddHrtfPath(PlayerWindow& app, const std::filesystem::path& path)
{
    const auto normalized = path.lexically_normal();
    const auto found = std::find(app.hrtfPaths.begin(), app.hrtfPaths.end(), normalized);
    if (found != app.hrtfPaths.end())
    {
        SendMessageW(app.hrtfProfile, CB_SETCURSEL,
            static_cast<WPARAM>(std::distance(app.hrtfPaths.begin(), found)), 0);
        return;
    }

    std::wstring profileName = normalized.stem().wstring();
    constexpr std::wstring_view sadieSuffix = L"_48K_24bit_256tap_FIR_SOFA";
    if (const auto suffix = profileName.find(sadieSuffix); suffix != std::wstring::npos)
    {
        profileName.resize(suffix);
    }
    const std::wstring label = normalized.parent_path().filename().wstring() +
        L" / " + profileName;
    const auto index = SendMessageW(
        app.hrtfProfile, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label.c_str()));
    if (index == CB_ERR || index == CB_ERRSPACE) return;
    app.hrtfPaths.push_back(normalized);
    app.loadedHrtfProfiles.emplace_back();
    SendMessageW(app.hrtfProfile, CB_SETCURSEL, static_cast<WPARAM>(index), 0);
}

bool SelectHrtfProfile(PlayerWindow& app, int index)
{
    if (index < 0 || static_cast<std::size_t>(index) >= app.hrtfPaths.size()) return false;
    auto& profile = app.loadedHrtfProfiles[static_cast<std::size_t>(index)];
    if (!profile)
    {
        profile = HrtfProfile::Load(app.hrtfPaths[static_cast<std::size_t>(index)]);
    }
    if (!profile)
    {
        MessageBoxW(app.window,
            L"This SOFA file is not a supported HRTF profile.",
            L"HRTF profile",
            MB_OK | MB_ICONWARNING);
        return false;
    }

    app.player.SetHrtfProfile(profile);
    SetStatus(app, std::wstring(L"HRTF switched: ") +
        app.hrtfPaths[static_cast<std::size_t>(index)].filename().wstring());
    return true;
}

void PopulateHrtfProfiles(PlayerWindow& app)
{
    const auto assetDirectory = ExecutableDirectory() / "assets";
    std::vector<std::filesystem::path> paths;
    std::error_code error;
    for (std::filesystem::recursive_directory_iterator iterator(
             assetDirectory, std::filesystem::directory_options::skip_permission_denied, error),
         end;
         iterator != end;
         iterator.increment(error))
    {
        if (error)
        {
            error.clear();
            continue;
        }
        if (iterator->is_regular_file(error) && iterator->path().extension() == L".sofa")
        {
            paths.push_back(iterator->path());
        }
    }

    std::sort(paths.begin(), paths.end());
    const auto defaultPath = assetDirectory / "MIT_KEMAR_normal_pinna.sofa";
    const auto defaultProfile = std::find(paths.begin(), paths.end(), defaultPath);
    if (defaultProfile != paths.end())
    {
        std::rotate(paths.begin(), defaultProfile, std::next(defaultProfile));
    }
    for (const auto& path : paths)
    {
        AddHrtfPath(app, path);
    }
    if (!app.hrtfPaths.empty())
    {
        SendMessageW(app.hrtfProfile, CB_SETCURSEL, 0, 0);
        SelectHrtfProfile(app, 0);
    }
}

void BrowseForHrtfProfile(PlayerWindow& app)
{
    wchar_t path[MAX_PATH * 4]{};
    OPENFILENAMEW dialog{};
    dialog.lStructSize = sizeof(dialog);
    dialog.hwndOwner = app.window;
    dialog.lpstrFilter = L"SOFA HRTF profiles\0*.sofa\0All files\0*.*\0";
    dialog.lpstrFile = path;
    dialog.nMaxFile = static_cast<DWORD>(std::size(path));
    dialog.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    dialog.lpstrTitle = L"Select an HRTF profile";
    if (!GetOpenFileNameW(&dialog)) return;

    AddHrtfPath(app, path);
    SelectHrtfProfile(app, static_cast<int>(SendMessageW(app.hrtfProfile, CB_GETCURSEL, 0, 0)));
}

HWND AddControl(
    HWND parent,
    const wchar_t* type,
    const wchar_t* text,
    DWORD style,
    int x,
    int y,
    int width,
    int height,
    int id = 0)
{
    return CreateWindowExW(
        0,
        type,
        text,
        WS_CHILD | WS_VISIBLE | style,
        x,
        y,
        width,
        height,
        parent,
        reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
        GetModuleHandleW(nullptr),
        nullptr);
}

void UpdateSliderLabels(PlayerWindow& app)
{
    const int width = static_cast<int>(SendMessageW(app.widthSlider, TBM_GETPOS, 0, 0));
    const int room = static_cast<int>(SendMessageW(app.roomSlider, TBM_GETPOS, 0, 0));
    const int hrtfBlend = static_cast<int>(SendMessageW(app.hrtfBlend, TBM_GETPOS, 0, 0));
    const auto widthText = std::to_wstring(width) + L" degrees";
    const auto roomText = std::to_wstring(room) + L"%";
    SetWindowTextW(app.widthValue, widthText.c_str());
    SetWindowTextW(app.roomValue, roomText.c_str());
    const auto hrtfBlendText = std::to_wstring(hrtfBlend) + L"%";
    SetWindowTextW(app.hrtfBlendValue, hrtfBlendText.c_str());
}

void CreateControls(PlayerWindow& app)
{
    AddControl(app.window, L"STATIC", L"MAGIC AAP SPATIAL", SS_LEFT, 24, 18, 600, 32);
    AddControl(app.window, L"STATIC", L"Local playback through the selected Windows output", SS_LEFT, 24, 51, 650, 24);
    AddControl(app.window, L"STATIC", L"Audio file", SS_LEFT, 24, 98, 90, 24);
    app.filePath = AddControl(app.window, L"EDIT", L"", ES_AUTOHSCROLL | WS_TABSTOP | WS_BORDER, 24, 124, 570, 32, kFilePathId);
    AddControl(app.window, L"BUTTON", L"Browse...", BS_PUSHBUTTON | WS_TABSTOP, 608, 124, 112, 32, kBrowseId);

    app.playButton = AddControl(app.window, L"BUTTON", L"Play", BS_PUSHBUTTON | WS_TABSTOP, 24, 177, 106, 38, kPlayId);
    app.stopButton = AddControl(app.window, L"BUTTON", L"Stop", BS_PUSHBUTTON | WS_TABSTOP, 140, 177, 106, 38, kStopId);
    EnableWindow(app.stopButton, FALSE);
    app.status = AddControl(app.window, L"STATIC", L"Choose a file to begin.", SS_LEFT | SS_CENTERIMAGE, 270, 177, 450, 38, kStatusId);

    AddControl(app.window, L"STATIC", L"Spatial profile", SS_LEFT, 24, 247, 150, 24);
    app.profile = AddControl(app.window, WC_COMBOBOXW, L"", CBS_DROPDOWNLIST | WS_TABSTOP | WS_VSCROLL, 24, 274, 360, 180, kProfileId);
    SendMessageW(app.profile, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Head-tracked stereo (computer anchored)"));
    SendMessageW(app.profile, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Fixed stereo soundstage"));
    SendMessageW(app.profile, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Unprocessed stereo"));
    SendMessageW(app.profile, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"E-AC-3 channel bed (approximation)"));
    SendMessageW(app.profile, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Atmos objects (Cavern decoder)"));
    SendMessageW(app.profile, CB_SETCURSEL, 0, 0);

    AddControl(app.window, L"STATIC", L"HRTF profile", SS_LEFT, 400, 247, 160, 24);
    app.hrtfProfile = AddControl(
        app.window,
        WC_COMBOBOXW,
        L"",
        CBS_DROPDOWNLIST | WS_TABSTOP | WS_VSCROLL,
        400,
        274,
        210,
        180,
        kHrtfProfileId);
    AddControl(
        app.window,
        L"BUTTON",
        L"Browse...",
        BS_PUSHBUTTON | WS_TABSTOP,
        618,
        274,
        102,
        32,
        kHrtfBrowseId);
    PopulateHrtfProfiles(app);
    SetStatus(app, L"Choose a file to begin.");

    AddControl(app.window, L"STATIC", L"HRTF blend", SS_LEFT, 400, 318, 160, 24);
    app.hrtfBlend = AddControl(
        app.window,
        TRACKBAR_CLASSW,
        L"",
        TBS_HORZ | TBS_AUTOTICKS | WS_TABSTOP,
        400,
        344,
        210,
        32,
        kHrtfBlendId);
    SendMessageW(app.hrtfBlend, TBM_SETRANGE, TRUE, MAKELPARAM(0, 100));
    SendMessageW(app.hrtfBlend, TBM_SETPOS, TRUE, 85);
    app.hrtfBlendValue = AddControl(app.window, L"STATIC", L"85%", SS_RIGHT, 618, 344, 102, 28, kHrtfBlendValueId);

    AddControl(app.window, L"STATIC", L"Speaker width", SS_LEFT, 24, 389, 150, 24);
    app.widthSlider = AddControl(app.window, TRACKBAR_CLASSW, L"", TBS_HORZ | TBS_AUTOTICKS | WS_TABSTOP, 24, 416, 570, 32, kWidthSliderId);
    SendMessageW(app.widthSlider, TBM_SETRANGE, TRUE, MAKELPARAM(20, 120));
    SendMessageW(app.widthSlider, TBM_SETPOS, TRUE, 68);
    app.widthValue = AddControl(app.window, L"STATIC", L"68 degrees", SS_RIGHT, 608, 416, 112, 28, kWidthValueId);

    AddControl(app.window, L"STATIC", L"Room / diffuse bed", SS_LEFT, 24, 464, 180, 24);
    app.roomSlider = AddControl(app.window, TRACKBAR_CLASSW, L"", TBS_HORZ | TBS_AUTOTICKS | WS_TABSTOP, 24, 491, 570, 32, kRoomSliderId);
    SendMessageW(app.roomSlider, TBM_SETRANGE, TRUE, MAKELPARAM(0, 35));
    SendMessageW(app.roomSlider, TBM_SETPOS, TRUE, 8);
    app.roomValue = AddControl(app.window, L"STATIC", L"8%", SS_RIGHT, 608, 491, 112, 28, kRoomValueId);

    AddControl(app.window, L"STATIC", L"JOC objects", SS_LEFT, 24, 544, 150, 24);
    app.dynamicLock = AddControl(app.window, WC_COMBOBOXW, L"", CBS_DROPDOWNLIST | WS_TABSTOP | WS_VSCROLL, 24, 570, 300, 150, kDynamicLockId);
    SendMessageW(app.dynamicLock, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"World-locked"));
    SendMessageW(app.dynamicLock, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Head-locked"));
    SendMessageW(app.dynamicLock, CB_SETCURSEL, 0, 0);

    AddControl(app.window, L"STATIC", L"Bed / ambience", SS_LEFT, 376, 544, 160, 24);
    app.bedLock = AddControl(app.window, WC_COMBOBOXW, L"", CBS_DROPDOWNLIST | WS_TABSTOP | WS_VSCROLL, 376, 570, 344, 150, kBedLockId);
    SendMessageW(app.bedLock, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Diffuse / head-relative"));
    SendMessageW(app.bedLock, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"World-locked"));
    SendMessageW(app.bedLock, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Head-locked"));
    SendMessageW(app.bedLock, CB_SETCURSEL, 0, 0);

    app.reverseHeadTracking = AddControl(
        app.window,
        L"BUTTON",
        L"Reverse head tracking",
        BS_AUTOCHECKBOX | WS_TABSTOP,
        24,
        608,
        240,
        24,
        kReverseHeadTrackingId);
    SendMessageW(app.reverseHeadTracking, BM_SETCHECK, BST_CHECKED, 0);

    AddControl(
        app.window,
        L"STATIC",
        L"E-AC-3 channel bed is an FFmpeg approximation. Cavern mode decodes JOC objects; Cavern's non-commercial license applies.",
        SS_LEFT,
        24,
        644,
        696,
        44);
}

void BrowseForFile(PlayerWindow& app)
{
    wchar_t path[MAX_PATH * 4]{};
    OPENFILENAMEW dialog{};
    dialog.lStructSize = sizeof(dialog);
    dialog.hwndOwner = app.window;
    dialog.lpstrFilter = L"Supported audio\0*.wav;*.flac;*.mp3;*.eac3;*.ec3;*.ac3;*.aac;*.m4a;*.m4b;*.ogg;*.opus;*.wma;*.mkv;*.mp4\0Common formats\0*.wav;*.flac;*.mp3\0E-AC-3 and containers\0*.eac3;*.ec3;*.ac3;*.m4a;*.m4b;*.mkv;*.mp4\0All files\0*.*\0";
    dialog.lpstrFile = path;
    dialog.nMaxFile = static_cast<DWORD>(std::size(path));
    dialog.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    dialog.lpstrTitle = L"Open audio file";
    if (!GetOpenFileNameW(&dialog)) return;

    app.selectedFile = path;
    SetWindowTextW(app.filePath, path);
    SetStatus(app, L"Ready. Set AirPods as the Windows default output.");
}

void StartPlayback(PlayerWindow& app)
{
    if (app.isPlaying) return;
    const int length = GetWindowTextLengthW(app.filePath);
    if (length <= 0)
    {
        SetStatus(app, L"Choose an audio file first.");
        return;
    }

    std::wstring path(static_cast<std::size_t>(length) + 1, L'\0');
    GetWindowTextW(app.filePath, path.data(), length + 1);
    path.resize(static_cast<std::size_t>(length));
    app.selectedFile = path;
    if (app.playbackThread.joinable()) app.playbackThread.join();

    PlaybackSettings settings;
    const auto profile = SendMessageW(app.profile, CB_GETCURSEL, 0, 0);
    settings.mode = profile == 0 || profile == 3 || profile == 4
        ? PlaybackSpatialMode::Tracked
        : profile == 2
            ? PlaybackSpatialMode::Off
            : PlaybackSpatialMode::Fixed;
    settings.channelBedMode = profile == 3;
    settings.cavernAtmosObjects = profile == 4;
    settings.stageWidthDegrees = static_cast<float>(SendMessageW(app.widthSlider, TBM_GETPOS, 0, 0));
    settings.roomReflection = static_cast<float>(SendMessageW(app.roomSlider, TBM_GETPOS, 0, 0)) / 100.0f;
    settings.hrtfBlend = static_cast<float>(SendMessageW(app.hrtfBlend, TBM_GETPOS, 0, 0)) / 100.0f;
    settings.reverseHeadTracking = SendMessageW(app.reverseHeadTracking, BM_GETCHECK, 0, 0) == BST_CHECKED;
    settings.atmosPanner.objectWidth = settings.stageWidthDegrees / 68.0f;
    settings.atmosPanner.diffuseBedLevel = settings.roomReflection;
    settings.atmosPanner.dynamicObjects = SendMessageW(app.dynamicLock, CB_GETCURSEL, 0, 0) == 1
        ? ObjectLockMode::HeadLocked
        : ObjectLockMode::WorldLocked;
    settings.atmosPanner.bed = SendMessageW(app.bedLock, CB_GETCURSEL, 0, 0) == 1
        ? ObjectLockMode::WorldLocked
        : SendMessageW(app.bedLock, CB_GETCURSEL, 0, 0) == 2
            ? ObjectLockMode::HeadLocked
            : ObjectLockMode::Diffuse;
    app.stopRequested.store(false, std::memory_order_relaxed);
    app.trackingStopRequested.store(false, std::memory_order_relaxed);
    app.activeTrackingProfile = 0;
    app.player.SetHeadPose(0.0f, 0.0f);
    app.player.SetHrtfBlend(settings.hrtfBlend);
    app.isPlaying = true;
    EnableWindow(app.playButton, FALSE);
    EnableWindow(app.stopButton, TRUE);
    EnableWindow(app.profile, FALSE);
    EnableWindow(app.widthSlider, FALSE);
    EnableWindow(app.roomSlider, FALSE);
    EnableWindow(app.dynamicLock, FALSE);
    EnableWindow(app.bedLock, FALSE);
    EnableWindow(app.reverseHeadTracking, FALSE);
    SetStatus(app, L"Opening audio stream...");

    const auto selectedFile = app.selectedFile;
    app.playbackThread = std::thread([&app, selectedFile, settings]
    {
        const int result = app.player.PlayFile(selectedFile, settings, app.stopRequested);
        app.trackingStopRequested.store(true, std::memory_order_relaxed);
        PostMessageW(app.window, kPlaybackFinished, static_cast<WPARAM>(result), 0);
    });

    if (settings.mode == PlaybackSpatialMode::Tracked)
    {
        app.trackingThread = std::thread([&app]
        {
            HeadTracker tracker;
            try
            {
                tracker.Run(
                    app.trackingStopRequested,
                    [&app](const PoseResult& pose)
                    {
                        if (pose.calibrated && pose.yawDegrees && pose.pitchDegrees)
                        {
                            app.player.SetHeadPose(
                                static_cast<float>(*pose.yawDegrees),
                                static_cast<float>(*pose.pitchDegrees));
                            PostMessageW(app.window, kTrackingStatusChanged, 2, 0);
                        }
                        else
                        {
                            PostMessageW(app.window, kTrackingStatusChanged, 1, 0);
                        }
                    },
                    [&app](const std::string& status)
                    {
                        if (status.starts_with("Detecting"))
                        {
                            PostMessageW(app.window, kTrackingStatusChanged, 0, 0);
                        }
                        else if (status.starts_with("Tracking active: alternate"))
                        {
                            PostMessageW(app.window, kTrackingStatusChanged, 4, 0);
                        }
                        else if (status.starts_with("Tracking active: devmotion6"))
                        {
                            PostMessageW(app.window, kTrackingStatusChanged, 5, 0);
                        }
                        else if (status.starts_with("Tracking active: max2"))
                        {
                            PostMessageW(app.window, kTrackingStatusChanged, 6, 0);
                        }
                    });
            }
            catch (...)
            {
                PostMessageW(app.window, kTrackingStatusChanged, 3, 0);
            }
        });
    }
}

void StopPlayback(PlayerWindow& app)
{
    if (app.playbackThread.joinable())
    {
        app.stopRequested.store(true, std::memory_order_relaxed);
        app.trackingStopRequested.store(true, std::memory_order_relaxed);
        app.playbackThread.join();
    }
    if (app.trackingThread.joinable()) app.trackingThread.join();
    app.isPlaying = false;
    EnableWindow(app.playButton, TRUE);
    EnableWindow(app.stopButton, FALSE);
    EnableWindow(app.profile, TRUE);
    EnableWindow(app.widthSlider, TRUE);
    EnableWindow(app.roomSlider, TRUE);
    EnableWindow(app.dynamicLock, TRUE);
    EnableWindow(app.bedLock, TRUE);
    EnableWindow(app.reverseHeadTracking, TRUE);
    SetStatus(app, L"Stopped.");
}

LRESULT CALLBACK WindowProcedure(HWND window, UINT message, WPARAM wParam, LPARAM lParam)
{
    auto* app = reinterpret_cast<PlayerWindow*>(GetWindowLongPtrW(window, GWLP_USERDATA));
    if (message == WM_NCCREATE)
    {
        const auto* create = reinterpret_cast<CREATESTRUCTW*>(lParam);
        app = static_cast<PlayerWindow*>(create->lpCreateParams);
        app->window = window;
        SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(app));
    }

    if (app == nullptr) return DefWindowProcW(window, message, wParam, lParam);

    switch (message)
    {
    case WM_CREATE:
        CreateControls(*app);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wParam))
        {
        case kBrowseId:
            BrowseForFile(*app);
            return 0;
        case kHrtfBrowseId:
            BrowseForHrtfProfile(*app);
            return 0;
        case kHrtfProfileId:
            if (HIWORD(wParam) == CBN_SELCHANGE)
            {
                SelectHrtfProfile(
                    *app,
                    static_cast<int>(SendMessageW(app->hrtfProfile, CB_GETCURSEL, 0, 0)));
            }
            return 0;
        case kPlayId:
            StartPlayback(*app);
            return 0;
        case kStopId:
            StopPlayback(*app);
            return 0;
        }
        break;
    case WM_HSCROLL:
        UpdateSliderLabels(*app);
        app->player.SetHrtfBlend(
            static_cast<float>(SendMessageW(app->hrtfBlend, TBM_GETPOS, 0, 0)) / 100.0f);
        return 0;
    case kPlaybackFinished:
        if (app->playbackThread.joinable()) app->playbackThread.join();
        app->trackingStopRequested.store(true, std::memory_order_relaxed);
        if (app->trackingThread.joinable()) app->trackingThread.join();
        app->isPlaying = false;
        EnableWindow(app->playButton, TRUE);
        EnableWindow(app->stopButton, FALSE);
        EnableWindow(app->profile, TRUE);
        EnableWindow(app->widthSlider, TRUE);
        EnableWindow(app->roomSlider, TRUE);
        EnableWindow(app->dynamicLock, TRUE);
        EnableWindow(app->bedLock, TRUE);
        EnableWindow(app->reverseHeadTracking, TRUE);
        SetStatus(*app, wParam == 0
            ? L"Finished."
            : wParam == 3
                ? L"Cavern decoder failed; check E-AC-3 JOC input and AtmosDecoder.exe."
                : wParam == 2
                ? L"This format needs FFmpeg. Use the Windows vcpkg build."
                : L"Playback failed. Check the file format and default output device.");
        return 0;
    case kTrackingStatusChanged:
        if (wParam == 0) SetStatus(*app, L"Detecting head-tracking protocol...");
        else if (wParam == 1 || wParam == 2) SetTrackingStatus(*app, wParam == 2);
        else if (wParam >= 4 && wParam <= 6)
        {
            app->activeTrackingProfile = wParam;
            SetTrackingStatus(*app, false);
        }
        else SetStatus(*app, L"Head tracking unavailable; audio will remain centered.");
        return 0;
    case WM_CLOSE:
        StopPlayback(*app);
        DestroyWindow(window);
        return 0;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(window, message, wParam, lParam);
}
}

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int showCommand)
{
    INITCOMMONCONTROLSEX commonControls{sizeof(commonControls), ICC_BAR_CLASSES};
    InitCommonControlsEx(&commonControls);

    WNDCLASSW windowClass{};
    windowClass.lpfnWndProc = WindowProcedure;
    windowClass.hInstance = instance;
    windowClass.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    windowClass.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    windowClass.lpszClassName = kWindowClass;
    if (!RegisterClassW(&windowClass)) return 1;

    PlayerWindow app;
    const HWND window = CreateWindowExW(
        0,
        kWindowClass,
        L"Magic AAP Spatial Player",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        780,
        760,
        nullptr,
        nullptr,
        instance,
        &app);
    if (window == nullptr) return 1;

    ShowWindow(window, showCommand);
    UpdateWindow(window);
    MSG message{};
    while (GetMessageW(&message, nullptr, 0, 0) > 0)
    {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
    return static_cast<int>(message.wParam);
}