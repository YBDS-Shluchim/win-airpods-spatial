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

#include <atomic>
#include <filesystem>
#include <iterator>
#include <string>
#include <thread>

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

struct PlayerWindow
{
    HWND window = nullptr;
    HWND filePath = nullptr;
    HWND playButton = nullptr;
    HWND stopButton = nullptr;
    HWND profile = nullptr;
    HWND widthSlider = nullptr;
    HWND roomSlider = nullptr;
    HWND widthValue = nullptr;
    HWND roomValue = nullptr;
    HWND dynamicLock = nullptr;
    HWND bedLock = nullptr;
    HWND status = nullptr;
    MediaPlayer player;
    std::filesystem::path selectedFile;
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
    const auto widthText = std::to_wstring(width) + L" degrees";
    const auto roomText = std::to_wstring(room) + L"%";
    SetWindowTextW(app.widthValue, widthText.c_str());
    SetWindowTextW(app.roomValue, roomText.c_str());
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

    AddControl(app.window, L"STATIC", L"Speaker width", SS_LEFT, 24, 329, 150, 24);
    app.widthSlider = AddControl(app.window, TRACKBAR_CLASSW, L"", TBS_HORZ | TBS_AUTOTICKS | WS_TABSTOP, 24, 356, 570, 32, kWidthSliderId);
    SendMessageW(app.widthSlider, TBM_SETRANGE, TRUE, MAKELPARAM(20, 120));
    SendMessageW(app.widthSlider, TBM_SETPOS, TRUE, 68);
    app.widthValue = AddControl(app.window, L"STATIC", L"68 degrees", SS_RIGHT, 608, 356, 112, 28, kWidthValueId);

    AddControl(app.window, L"STATIC", L"Room / diffuse bed", SS_LEFT, 24, 404, 180, 24);
    app.roomSlider = AddControl(app.window, TRACKBAR_CLASSW, L"", TBS_HORZ | TBS_AUTOTICKS | WS_TABSTOP, 24, 431, 570, 32, kRoomSliderId);
    SendMessageW(app.roomSlider, TBM_SETRANGE, TRUE, MAKELPARAM(0, 35));
    SendMessageW(app.roomSlider, TBM_SETPOS, TRUE, 12);
    app.roomValue = AddControl(app.window, L"STATIC", L"12%", SS_RIGHT, 608, 431, 112, 28, kRoomValueId);

    AddControl(app.window, L"STATIC", L"JOC objects", SS_LEFT, 24, 484, 150, 24);
    app.dynamicLock = AddControl(app.window, WC_COMBOBOXW, L"", CBS_DROPDOWNLIST | WS_TABSTOP | WS_VSCROLL, 24, 510, 300, 150, kDynamicLockId);
    SendMessageW(app.dynamicLock, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"World-locked"));
    SendMessageW(app.dynamicLock, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Head-locked"));
    SendMessageW(app.dynamicLock, CB_SETCURSEL, 0, 0);

    AddControl(app.window, L"STATIC", L"Bed / ambience", SS_LEFT, 376, 484, 160, 24);
    app.bedLock = AddControl(app.window, WC_COMBOBOXW, L"", CBS_DROPDOWNLIST | WS_TABSTOP | WS_VSCROLL, 376, 510, 344, 150, kBedLockId);
    SendMessageW(app.bedLock, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Diffuse / head-relative"));
    SendMessageW(app.bedLock, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"World-locked"));
    SendMessageW(app.bedLock, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Head-locked"));
    SendMessageW(app.bedLock, CB_SETCURSEL, 0, 0);

    AddControl(
        app.window,
        L"STATIC",
        L"E-AC-3 channel bed is an FFmpeg approximation. Cavern mode decodes JOC objects; Cavern's non-commercial license applies.",
        SS_LEFT,
        24,
        584,
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
    app.player.SetHeadPose(0.0f, 0.0f);
    app.isPlaying = true;
    EnableWindow(app.playButton, FALSE);
    EnableWindow(app.stopButton, TRUE);
    EnableWindow(app.profile, FALSE);
    EnableWindow(app.widthSlider, FALSE);
    EnableWindow(app.roomSlider, FALSE);
    EnableWindow(app.dynamicLock, FALSE);
    EnableWindow(app.bedLock, FALSE);
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
        else if (wParam == 1) SetStatus(*app, L"Calibrating head pose; hold still...");
        else if (wParam == 2) SetStatus(*app, L"Head tracking active.");
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
        700,
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