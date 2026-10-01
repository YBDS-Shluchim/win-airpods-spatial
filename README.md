# win-airpods-spatial

Native C++20 CLI for MagicAAP discovery, head-tracking control, pose processing, and a custom stereo spatial DSP core. It uses Windows SetupAPI and the existing MagicAAP driver; this project does not install or sign a Bluetooth or audio kernel driver.

## Commands

On Windows, launch `MagicAapSpatialPlayer.exe` for the GUI. The CLI remains available for diagnostics and simple playback:

```text
magic-aap-spatial devices
magic-aap-spatial probe [--interface auto|private|service] [--path DEVICE_PATH] [--seconds N]
magic-aap-spatial track [--interface auto|private|service] [--path DEVICE_PATH] [--seconds N]
  [--start-format auto|alternate|devmotion6|max2] [--probe-ms N] [--no-init] [--no-takeover]
magic-aap-spatial play <file> [--spatial off|fixed]
magic-aap-spatial audio
```

`devices` lists the same MagicAAP/AAP private, server, and client interfaces used by the original tool. `probe` prints packet data. `track` initializes RTBuddy services, requests connection ownership, tries known motion start packets, and keeps a profile only after valid motion packets arrive. `auto` tries alternate AAP, DEVMOTION6 service 16, then the Max 2 service-6 profile. This is protocol probing rather than complete capability negotiation; firmware variants may need additional profiles.

The pose estimator validates the received quaternion, calibrates neutral orientation with a 25-sample Markley mean, converts to relative yaw/pitch/roll, and smooths with quaternion interpolation. Keep the headset still during startup calibration.

## In-App Playback

`play <file>` decodes WAV, FLAC, and MP3 with miniaudio. With the Windows FFmpeg build, other formats—including E-AC-3 in `.eac3`, `.m4a`, `.m4b`, `.mp4`, and `.mkv` containers—are decoded to a channel-bed approximation and rendered through the built-in DSP. Audio goes to the current default playback endpoint using miniaudio's shared-mode backend (WASAPI on Windows). Set the AirPods as the Windows default output before playing. The command handles only its own media stream; other apps continue using Windows normally and are not captured or spatialized.

The GUI exposes head-tracked stereo, fixed stereo, bypass, FFmpeg channel-bed, and Cavern JOC object profiles, plus sliders for HRTF blend, stage width, and ambience. HRTF blend defaults to 100% binaural rendering and can be reduced during playback to mix back the original stereo signal. There is exactly one bundled HRTF profile (the Apple Generic HRTF asset below); it loads automatically and is not user-selectable, matching how macOS doesn't expose it as a user-facing choice either. In Cavern mode, dynamic objects default to world-locked and the bed defaults to diffuse/head-relative; each policy can be toggled independently between head-locked and world-locked, and non-diffuse objects are rendered through the same Apple HRTF convolver as stereo content. `AtmosDecoder.exe` streams Cavern-decoded object PCM and positions to the C++ panner, which applies live MagicAAP head pose. Cavern.Format by Bence Sgánetz is distributed under its custom non-commercial/share-alike license with attribution requirements; the Windows ZIP includes both Cavern license notices. Do not use the Cavern-enabled artifact commercially without permission from its author.

Stereo spatialization and per-object Atmos panning both use Apple's Generic HRTF dataset from `audio/assets/Apple_Generic_HRTF.bin`. The native reader interpolates its 25-by-50 CIPIC grid, applies the stored 64-tap filters, the per-ear scale factor stored alongside each filter, and the fractional modeling delay, and resamples its 44.1 kHz coefficients for the output device. Fixed and head-tracked profiles share the same binaural renderer; tracking changes the virtual speaker angles using the listener pose. A low-level small-room reflection and late-decay model is adjustable from the ambience slider. This is Apple's generic (non-personalized) system HRTF resource, consumed on macOS by the `HRTFPanner` Audio Unit inside `AudioDSP.component`; live memory inspection of `coreaudiod` while AirPods Spatial Audio (Fixed mode) was actively playing confirmed `AudioDSP.component`, `libAudioDSPCore.dylib`, and the `AudioDSPManager`/`AudioDSPGraph`/`AudioDSPAnalysis` frameworks are all loaded into that process, so this is the real rendering backbone rather than legacy/unused code. It does not include Apple's per-user ear-scan personalization. The bundled binary was copied from `/System/Library/Components/AudioDSP.component/Contents/Resources/Tunings/Generic/AU/HRTF.bin` and has SHA-256 `75171ff7602dfa8aa240af73cfcd89e67f286c58501bce196f8aa75bbeb7006c`.

## E-AC-3 / Atmos

The GUI's FFmpeg channel-bed profile remains a channel-based fallback. The separate Cavern JOC profile decodes E-AC-3 object metadata and emits per-object samples and positions; C++ applies the selected lock policies during live playback. This is the actual object-based mode; it is distinct from the FFmpeg downmix.

## Audio Constraint

The spatial DSP core is connected to this app's own playback, but not to system-wide Windows audio. Microsoft documents custom APOs as user-mode COM processing objects, but its supported custom-APO deployment uses a driver extension INF plus an APO INF. Windows validates driver-package signatures; that package-signing requirement is separate from whether the APO contains kernel-mode code. This app does not include that package or a custom kernel driver.

The `audio` command reports this system-wide boundary. See Microsoft's [APO architecture](https://learn.microsoft.com/en-us/windows-hardware/drivers/audio/audio-processing-object-architecture), [APO implementation](https://learn.microsoft.com/en-us/windows-hardware/drivers/audio/implementing-audio-processing-objects), [APO deployment](https://learn.microsoft.com/en-us/windows-hardware/drivers/dashboard/deploying-audio-processing-objects), and [driver signing](https://learn.microsoft.com/en-us/windows-hardware/drivers/install/driver-signing) guidance.

## Build and Test

Requires CMake 3.20+, a C++20 compiler, .NET 10 for the Cavern helper, and network access to restore dependencies. E-AC-3 channel-bed fallback uses the FFmpeg-enabled Windows vcpkg build; actual JOC uses Cavern.Format. Build and run native tests with:

```text
cmake -S audio -B build/audio
cmake --build build/audio
ctest --test-dir build/audio --output-on-failure
```