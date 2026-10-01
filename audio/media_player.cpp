#include "media_player.hpp"
#include "cavern_object_stream.hpp"
#include "ffmpeg_channel_decoder.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstring>
#include <iostream>
#include <optional>
#include <thread>
#include <vector>

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

namespace MagicAapSpatial
{
namespace
{
constexpr ma_uint32 kSampleRate = 48000;
constexpr ma_uint32 kCallbackChunkFrames = 2048;

bool UsesMiniaudioDecoder(const std::filesystem::path& path, bool forceChannelBed)
{
    if (forceChannelBed) return false;
    auto extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char value)
    {
        return static_cast<char>(std::tolower(value));
    });
    return extension == ".wav" || extension == ".flac" || extension == ".mp3";
}

class PlaybackSession final
{
public:
    PlaybackSession(
        PlaybackSettings settings,
        const std::atomic<float>& yawDegrees,
        const std::atomic<float>& pitchDegrees,
        const std::atomic<float>& hrtfBlend,
        const std::atomic<std::shared_ptr<const HrtfProfile>>& hrtfProfile)
        : interleaved_(kCallbackChunkFrames * 4),
          inputLeft_(kCallbackChunkFrames),
          inputRight_(kCallbackChunkFrames),
                    ambienceLeft_(kCallbackChunkFrames),
                    ambienceRight_(kCallbackChunkFrames),
          outputLeft_(kCallbackChunkFrames),
          outputRight_(kCallbackChunkFrames),
                    ambienceOutputLeft_(kCallbackChunkFrames),
                    ambienceOutputRight_(kCallbackChunkFrames),
                    objectOutputLeft_(kCallbackChunkFrames),
                    objectOutputRight_(kCallbackChunkFrames),
          settings_(settings),
          yawDegrees_(yawDegrees),
          pitchDegrees_(pitchDegrees),
          hrtfBlend_(hrtfBlend),
          hrtfProfile_(hrtfProfile)
    {
        const auto mode = settings.mode == PlaybackSpatialMode::Tracked
            ? Mode::Tracked
            : settings.mode == PlaybackSpatialMode::Fixed
                ? Mode::Fixed
                : Mode::Off;
        spatializer_.SetMode(mode);
        spatializer_.SetStageWidth(settings.stageWidthDegrees);
        spatializer_.SetRoomReflection(settings.roomReflection);
        if (!spatializer_.IsHrtfReady())
        {
            std::cerr << "Measured HRTF data was not found; spatial playback will use unprocessed stereo.\n";
        }
        ambienceSpatializer_.SetMode(Mode::Fixed);
        ambienceSpatializer_.SetStageWidth(120.0f);
        ambienceSpatializer_.SetRoomReflection(settings.roomReflection * 0.5f);
        atmosPanner_.SetSettings(settings.atmosPanner);
    }

    static void DataCallback(
        ma_device* device,
        void* output,
        const void*,
        ma_uint32 frameCount)
    {
        auto* session = static_cast<PlaybackSession*>(device->pUserData);
        session->Render(static_cast<float*>(output), frameCount);
    }

    ma_result Initialize(const std::filesystem::path& path)
    {
        ma_result result = MA_SUCCESS;
        if (settings_.cavernAtmosObjects)
        {
            try
            {
                cavernStream_ = std::make_unique<CavernObjectStream>(path);
            }
            catch (const std::exception& error)
            {
                std::cerr << "Could not start Cavern Atmos decoder: " << error.what() << '\n';
                return MA_ERROR;
            }
            if (cavernStream_->SampleRate() < 8000 || cavernStream_->SampleRate() > 192000)
            {
                return MA_FORMAT_NOT_SUPPORTED;
            }
            playbackSampleRate_ = static_cast<ma_uint32>(cavernStream_->SampleRate());
            if (!cavernStream_->ReadBlock(currentObjectBlock_, std::chrono::seconds(5)))
            {
                return MA_NO_DATA_AVAILABLE;
            }
            atmosPanner_.Process(
                currentObjectBlock_,
                ListenerYaw(),
                pitchDegrees_.load(std::memory_order_relaxed),
                objectOutputLeft_.data(),
                objectOutputRight_.data(),
                objectOutputLeft_.size());
            objectBlockPosition_ = 0;
        }
        else
        {
            auto decoderConfig = ma_decoder_config_init(
                ma_format_f32,
                settings_.channelBedMode ? 4 : 2,
                kSampleRate);
            const auto pathText = path.string();
            result = ma_decoder_init_file(pathText.c_str(), &decoderConfig, &decoder_);
            if (result != MA_SUCCESS) return result;
            decoderInitialized_ = true;
        }

        auto deviceConfig = ma_device_config_init(ma_device_type_playback);
        deviceConfig.playback.format = ma_format_f32;
        deviceConfig.playback.channels = 2;
        deviceConfig.playback.shareMode = ma_share_mode_shared;
        deviceConfig.sampleRate = playbackSampleRate_;
        deviceConfig.dataCallback = DataCallback;
        deviceConfig.pUserData = this;
        result = ma_device_init(nullptr, &deviceConfig, &device_);
        if (result != MA_SUCCESS) return result;
        deviceInitialized_ = true;
        return MA_SUCCESS;
    }

    ma_result Start()
    {
        return ma_device_start(&device_);
    }

    bool Wait(const std::atomic_bool& stopRequested)
    {
        while (!finished_.load(std::memory_order_relaxed) &&
            !stopRequested.load(std::memory_order_relaxed))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (cavernStream_)
        {
            const auto error = cavernStream_->Error();
            if (!error.empty())
            {
                std::cerr << "Cavern Atmos decode failed: " << error << '\n';
                return false;
            }
        }
        return true;
    }

    void Stop()
    {
        if (deviceInitialized_)
        {
            ma_device_stop(&device_);
            ma_device_uninit(&device_);
            deviceInitialized_ = false;
        }
        if (decoderInitialized_)
        {
            ma_decoder_uninit(&decoder_);
            decoderInitialized_ = false;
        }
        cavernStream_.reset();
    }

    ~PlaybackSession()
    {
        Stop();
    }

private:
    float ListenerYaw() const
    {
        const float yaw = yawDegrees_.load(std::memory_order_relaxed);
        return settings_.reverseHeadTracking ? -yaw : yaw;
    }

    void Render(float* output, ma_uint32 frameCount)
    {
        if (settings_.cavernAtmosObjects)
        {
            RenderCavernAtmos(output, frameCount);
            return;
        }

        ma_uint32 renderedFrames = 0;
        while (renderedFrames < frameCount)
        {
            const auto chunkFrames = std::min(
                kCallbackChunkFrames,
                frameCount - renderedFrames);
            ma_uint64 decodedFrames = 0;
            const auto result = ma_decoder_read_pcm_frames(
                &decoder_,
                interleaved_.data(),
                chunkFrames,
                &decodedFrames);
            if (result != MA_SUCCESS || decodedFrames == 0)
            {
                finished_.store(true, std::memory_order_relaxed);
                break;
            }

            const auto frames = static_cast<std::size_t>(decodedFrames);
            for (std::size_t index = 0; index < frames; index++)
            {
                const auto stride = settings_.channelBedMode ? 4u : 2u;
                inputLeft_[index] = interleaved_[index * stride];
                inputRight_[index] = interleaved_[index * stride + 1];
                if (settings_.channelBedMode)
                {
                    ambienceLeft_[index] = interleaved_[index * stride + 2];
                    ambienceRight_[index] = interleaved_[index * stride + 3];
                }
            }

            if (settings_.mode == PlaybackSpatialMode::Tracked)
            {
                spatializer_.SetPose(Pose{
                    ListenerYaw(),
                    pitchDegrees_.load(std::memory_order_relaxed),
                    0.0f});
            }

            auto selectedProfile = hrtfProfile_.load(std::memory_order_relaxed);
            if (selectedProfile != activeHrtfProfile_)
            {
                spatializer_.SetHrtfProfile(selectedProfile);
                ambienceSpatializer_.SetHrtfProfile(selectedProfile);
                activeHrtfProfile_ = std::move(selectedProfile);
            }
            const float hrtfBlend = hrtfBlend_.load(std::memory_order_relaxed);
            spatializer_.SetHrtfBlend(hrtfBlend);
            ambienceSpatializer_.SetHrtfBlend(hrtfBlend);

            spatializer_.Process(
                inputLeft_.data(),
                inputRight_.data(),
                outputLeft_.data(),
                outputRight_.data(),
                frames);

            if (settings_.channelBedMode)
            {
                ambienceSpatializer_.Process(
                    ambienceLeft_.data(),
                    ambienceRight_.data(),
                    ambienceOutputLeft_.data(),
                    ambienceOutputRight_.data(),
                    frames);
            }

            for (std::size_t index = 0; index < frames; index++)
            {
                const float ambientLeft = settings_.channelBedMode ? ambienceOutputLeft_[index] : 0.0f;
                const float ambientRight = settings_.channelBedMode ? ambienceOutputRight_[index] : 0.0f;
                output[(renderedFrames + index) * 2] = outputLeft_[index] + ambientLeft;
                output[(renderedFrames + index) * 2 + 1] = outputRight_[index] + ambientRight;
            }
            renderedFrames += static_cast<ma_uint32>(frames);
            if (decodedFrames < chunkFrames)
            {
                finished_.store(true, std::memory_order_relaxed);
                break;
            }
        }

        const auto remainingFrames = frameCount - renderedFrames;
        if (remainingFrames > 0)
        {
            std::memset(
                output + renderedFrames * 2,
                0,
                static_cast<std::size_t>(remainingFrames) * 2 * sizeof(float));
        }
    }

    void RenderCavernAtmos(float* output, ma_uint32 frameCount)
    {
        std::size_t renderedFrames = 0;
        while (renderedFrames < frameCount)
        {
            if (objectBlockPosition_ >= static_cast<std::size_t>(currentObjectBlock_.frameCount))
            {
                if (!cavernStream_->ReadBlock(currentObjectBlock_, std::chrono::milliseconds(5)))
                {
                    if (cavernStream_->IsFinished()) finished_.store(true, std::memory_order_relaxed);
                    break;
                }
                atmosPanner_.Process(
                    currentObjectBlock_,
                    ListenerYaw(),
                    pitchDegrees_.load(std::memory_order_relaxed),
                    objectOutputLeft_.data(),
                    objectOutputRight_.data(),
                    objectOutputLeft_.size());
                objectBlockPosition_ = 0;
            }

            const auto available = static_cast<std::size_t>(currentObjectBlock_.frameCount) - objectBlockPosition_;
            const auto count = std::min<std::size_t>(available, frameCount - renderedFrames);
            for (std::size_t index = 0; index < count; index++)
            {
                output[(renderedFrames + index) * 2] = objectOutputLeft_[objectBlockPosition_ + index];
                output[(renderedFrames + index) * 2 + 1] = objectOutputRight_[objectBlockPosition_ + index];
            }
            renderedFrames += count;
            objectBlockPosition_ += count;
        }
        if (renderedFrames < frameCount)
        {
            std::memset(
                output + renderedFrames * 2,
                0,
                static_cast<std::size_t>(frameCount - renderedFrames) * 2 * sizeof(float));
        }
    }

    ma_decoder decoder_{};
    ma_device device_{};
    bool decoderInitialized_ = false;
    bool deviceInitialized_ = false;
    std::atomic_bool finished_{false};
    std::vector<float> interleaved_;
    std::vector<float> inputLeft_;
    std::vector<float> inputRight_;
    std::vector<float> ambienceLeft_;
    std::vector<float> ambienceRight_;
    std::vector<float> outputLeft_;
    std::vector<float> outputRight_;
    std::vector<float> ambienceOutputLeft_;
    std::vector<float> ambienceOutputRight_;
    std::vector<float> objectOutputLeft_;
    std::vector<float> objectOutputRight_;
    std::unique_ptr<CavernObjectStream> cavernStream_;
    CavernObjectBlock currentObjectBlock_;
    std::size_t objectBlockPosition_ = 0;
    ma_uint32 playbackSampleRate_ = kSampleRate;
    AtmosPanner atmosPanner_;
    PlaybackSettings settings_;
    const std::atomic<float>& yawDegrees_;
    const std::atomic<float>& pitchDegrees_;
    const std::atomic<float>& hrtfBlend_;
    const std::atomic<std::shared_ptr<const HrtfProfile>>& hrtfProfile_;
    std::shared_ptr<const HrtfProfile> activeHrtfProfile_;
    Spatializer spatializer_;
    Spatializer ambienceSpatializer_;
};
}

int MediaPlayer::PlayFile(
    const std::filesystem::path& path,
    PlaybackSettings settings,
    const std::atomic_bool& stopRequested)
{
    yawDegrees_.store(0.0f, std::memory_order_relaxed);
    pitchDegrees_.store(0.0f, std::memory_order_relaxed);
    std::optional<std::filesystem::path> decodedPath;
    if (!settings.cavernAtmosObjects && !UsesMiniaudioDecoder(path, settings.channelBedMode))
    {
        settings.channelBedMode = true;
        try
        {
            decodedPath = DecodeChannelBedToTemporaryWav(path);
        }
        catch (const std::exception& error)
        {
            std::cerr << "Could not decode E-AC-3 channel bed: " << error.what() << '\n';
            return 2;
        }
    }

    const auto& playbackPath = decodedPath ? *decodedPath : path;
    const auto play = [&]() -> int
    {
        PlaybackSession session(settings, yawDegrees_, pitchDegrees_, hrtfBlend_, hrtfProfile_);
        auto audioResult = session.Initialize(playbackPath);
        if (audioResult != MA_SUCCESS)
        {
            std::cerr << "Could not initialize file playback: " << ma_result_description(audioResult) << '\n';
            return settings.cavernAtmosObjects ? 3 : 1;
        }
        audioResult = session.Start();
        if (audioResult != MA_SUCCESS)
        {
            std::cerr << "Could not start the default playback device: " << ma_result_description(audioResult) << '\n';
            return 1;
        }

        std::cout << "Playing " << path.string() << " through the default output in WASAPI shared mode.\n"
                << (settings.cavernAtmosObjects
                    ? "Cavern JOC objects active; object and bed lock modes are applied.\n"
                    : settings.channelBedMode
                    ? "Channel-bed approximation is on: center/front tracks, surround bed stays diffuse. Atmos objects are not preserved.\n"
                        : settings.mode == PlaybackSpatialMode::Fixed
                            ? "Fixed spatial processing is on.\n"
                            : "Spatial processing is off.\n")
                  << "Press Ctrl+C or Stop to end playback.\n";
        return session.Wait(stopRequested) ? 0 : 3;
    };

    const auto result = play();
    if (decodedPath)
    {
        std::error_code ignored;
        std::filesystem::remove(*decodedPath, ignored);
    }
    return result;
}

void MediaPlayer::SetHeadPose(float yawDegrees, float pitchDegrees)
{
    yawDegrees_.store(yawDegrees, std::memory_order_relaxed);
    pitchDegrees_.store(pitchDegrees, std::memory_order_relaxed);
}

void MediaPlayer::SetHrtfBlend(float amount)
{
    hrtfBlend_.store(std::isfinite(amount) ? std::clamp(amount, 0.0f, 1.0f) : 1.0f,
        std::memory_order_relaxed);
}

void MediaPlayer::SetHrtfProfile(std::shared_ptr<const HrtfProfile> profile)
{
    hrtfProfile_.store(std::move(profile), std::memory_order_relaxed);
}
}