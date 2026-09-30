#include "ffmpeg_channel_decoder.hpp"

#ifdef MAGIC_AAP_HAS_FFMPEG
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/channel_layout.h>
#include <libavutil/error.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
}

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace MagicAapSpatial
{
namespace
{
constexpr int kOutputRate = 48000;
constexpr int kOutputChannels = 4;
constexpr int kFloatBits = 32;
std::atomic_uint64_t g_tempSequence{0};

std::string AvError(int result)
{
    std::array<char, AV_ERROR_MAX_STRING_SIZE> buffer{};
    av_strerror(result, buffer.data(), buffer.size());
    return buffer.data();
}

void CheckAv(int result, const char* operation)
{
    if (result < 0)
    {
        throw std::runtime_error(std::string(operation) + ": " + AvError(result));
    }
}

struct FfmpegState
{
    AVFormatContext* format = nullptr;
    AVCodecContext* codec = nullptr;
    AVPacket* packet = nullptr;
    AVFrame* frame = nullptr;
    SwrContext* resampler = nullptr;

    ~FfmpegState()
    {
        swr_free(&resampler);
        av_frame_free(&frame);
        av_packet_free(&packet);
        avcodec_free_context(&codec);
        avformat_close_input(&format);
    }
};

struct TemporaryFileCleanup
{
    std::filesystem::path path;
    bool keep = false;

    ~TemporaryFileCleanup()
    {
        if (!keep && !path.empty())
        {
            std::error_code ignored;
            std::filesystem::remove(path, ignored);
        }
    }
};

void WriteU16(std::ostream& output, std::uint16_t value)
{
    output.put(static_cast<char>(value & 0xff));
    output.put(static_cast<char>((value >> 8) & 0xff));
}

void WriteU32(std::ostream& output, std::uint32_t value)
{
    WriteU16(output, static_cast<std::uint16_t>(value & 0xffff));
    WriteU16(output, static_cast<std::uint16_t>(value >> 16));
}

void WriteWaveHeader(std::ostream& output, std::uint32_t dataBytes)
{
    output.write("RIFF", 4);
    WriteU32(output, 36 + dataBytes);
    output.write("WAVEfmt ", 8);
    WriteU32(output, 16);
    WriteU16(output, 3);
    WriteU16(output, kOutputChannels);
    WriteU32(output, kOutputRate);
    WriteU32(output, kOutputRate * kOutputChannels * sizeof(float));
    WriteU16(output, kOutputChannels * sizeof(float));
    WriteU16(output, kFloatBits);
    output.write("data", 4);
    WriteU32(output, dataBytes);
}

std::array<double, kOutputChannels> CoefficientsFor(AVChannel channel)
{
    switch (channel)
    {
    case AV_CHAN_FRONT_LEFT: return {1.0, 0.0, 0.0, 0.0};
    case AV_CHAN_FRONT_RIGHT: return {0.0, 1.0, 0.0, 0.0};
    case AV_CHAN_FRONT_CENTER: return {0.70710678, 0.70710678, 0.0, 0.0};
    case AV_CHAN_LOW_FREQUENCY: return {0.0, 0.0, 0.31622777, 0.31622777};
    case AV_CHAN_BACK_LEFT:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_FRONT_LEFT_OF_CENTER: return {0.5, 0.0, 0.0, 0.0};
    case AV_CHAN_BACK_RIGHT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_FRONT_RIGHT_OF_CENTER: return {0.0, 0.5, 0.0, 0.0};
    default: return {0.0, 0.0, 0.25, 0.25};
    }
}

std::string Utf8Path(const std::filesystem::path& path)
{
    const auto encoded = path.u8string();
    return std::string(reinterpret_cast<const char*>(encoded.data()), encoded.size());
}

void DecodeFrames(
    FfmpegState& state,
    std::ostream& output,
    std::uint64_t& totalDataBytes)
{
    const int capacity = swr_get_out_samples(state.resampler, state.frame->nb_samples);
    if (capacity <= 0) return;
    std::vector<float> stereo(static_cast<std::size_t>(capacity) * kOutputChannels);
    std::uint8_t* outputPlanes[] = {
        reinterpret_cast<std::uint8_t*>(stereo.data())};
    const auto** inputPlanes = const_cast<const std::uint8_t**>(state.frame->extended_data);
    const int converted = swr_convert(
        state.resampler,
        outputPlanes,
        capacity,
        inputPlanes,
        state.frame->nb_samples);
    CheckAv(converted, "Resample decoded E-AC-3 audio");
    const auto bytes = static_cast<std::size_t>(converted) * kOutputChannels * sizeof(float);
    output.write(reinterpret_cast<const char*>(stereo.data()), static_cast<std::streamsize>(bytes));
    totalDataBytes += bytes;
}

void FlushResampler(FfmpegState& state, std::ostream& output, std::uint64_t& totalDataBytes)
{
    while (true)
    {
        const int capacity = swr_get_out_samples(state.resampler, 0);
        if (capacity <= 0) return;
        std::vector<float> stereo(static_cast<std::size_t>(capacity) * kOutputChannels);
        std::uint8_t* outputPlanes[] = {reinterpret_cast<std::uint8_t*>(stereo.data())};
        const int converted = swr_convert(state.resampler, outputPlanes, capacity, nullptr, 0);
        CheckAv(converted, "Flush channel-bed resampler");
        if (converted == 0) return;
        const auto bytes = static_cast<std::size_t>(converted) * kOutputChannels * sizeof(float);
        output.write(reinterpret_cast<const char*>(stereo.data()), static_cast<std::streamsize>(bytes));
        totalDataBytes += bytes;
    }
}

void DrainDecoder(
    FfmpegState& state,
    std::ostream& output,
    std::uint64_t& totalDataBytes)
{
    while (true)
    {
        const int result = avcodec_receive_frame(state.codec, state.frame);
        if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) return;
        CheckAv(result, "Decode E-AC-3 frame");
        DecodeFrames(state, output, totalDataBytes);
        av_frame_unref(state.frame);
    }
}
}

std::filesystem::path DecodeChannelBedToTemporaryWav(const std::filesystem::path& sourcePath)
{
    FfmpegState state;
    const auto source = Utf8Path(sourcePath);
    CheckAv(avformat_open_input(&state.format, source.c_str(), nullptr, nullptr), "Open media file with FFmpeg");
    CheckAv(avformat_find_stream_info(state.format, nullptr), "Read media stream information");

    const AVCodec* decoder = nullptr;
    const int audioStream = av_find_best_stream(
        state.format,
        AVMEDIA_TYPE_AUDIO,
        -1,
        -1,
        &decoder,
        0);
    CheckAv(audioStream, "Find audio stream");
    if (decoder == nullptr) throw std::runtime_error("FFmpeg found no audio decoder for this file.");

    state.codec = avcodec_alloc_context3(decoder);
    if (state.codec == nullptr) throw std::runtime_error("Could not allocate FFmpeg decoder context.");
    CheckAv(avcodec_parameters_to_context(state.codec, state.format->streams[audioStream]->codecpar), "Read audio codec parameters");
    CheckAv(avcodec_open2(state.codec, decoder, nullptr), "Open FFmpeg audio decoder");

    AVChannelLayout inputLayout{};
    if (state.codec->ch_layout.nb_channels > 0)
    {
        CheckAv(av_channel_layout_copy(&inputLayout, &state.codec->ch_layout), "Copy input channel layout");
    }
    else throw std::runtime_error("FFmpeg did not report the decoded channel count.");
    if (inputLayout.order == AV_CHANNEL_ORDER_UNSPEC)
    {
        const int channels = inputLayout.nb_channels;
        av_channel_layout_uninit(&inputLayout);
        av_channel_layout_default(&inputLayout, channels);
    }
    const int channelCount = inputLayout.nb_channels;
    if (channelCount < 2 || channelCount > 18)
    {
        av_channel_layout_uninit(&inputLayout);
        throw std::runtime_error("E-AC-3 channel-bed mode requires a 2- to 18-channel decoded stream.");
    }

    AVChannelLayout outputLayout{};
    av_channel_layout_default(&outputLayout, kOutputChannels);
    std::array<double, kOutputChannels * 18> matrix{};
    const double normalization = channelCount > 2 ? 0.75 : 1.0;
    for (int input = 0; input < channelCount; input++)
    {
        const auto coefficients = CoefficientsFor(av_channel_layout_channel_from_index(&inputLayout, input));
        for (int output = 0; output < kOutputChannels; output++)
        {
            matrix[output * channelCount + input] = coefficients[output] * normalization;
        }
    }
    CheckAv(swr_alloc_set_opts2(
        &state.resampler,
        &outputLayout,
        AV_SAMPLE_FMT_FLT,
        kOutputRate,
        &inputLayout,
        state.codec->sample_fmt,
        state.codec->sample_rate,
        0,
        nullptr), "Configure channel-bed resampler");
    av_channel_layout_uninit(&inputLayout);
    av_channel_layout_uninit(&outputLayout);
    CheckAv(swr_set_matrix(state.resampler, matrix.data(), channelCount), "Configure channel-bed downmix");
    CheckAv(swr_init(state.resampler), "Initialize channel-bed resampler");

    state.packet = av_packet_alloc();
    state.frame = av_frame_alloc();
    if (state.packet == nullptr || state.frame == nullptr)
    {
        throw std::runtime_error("Could not allocate FFmpeg decode buffers.");
    }

    const auto sequence = g_tempSequence.fetch_add(1, std::memory_order_relaxed);
    const auto temporaryPath = std::filesystem::temp_directory_path() /
        ("MagicAapSpatial-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
         "-" + std::to_string(sequence) + ".wav");
    TemporaryFileCleanup cleanup{temporaryPath};
    std::ofstream output(temporaryPath, std::ios::binary | std::ios::trunc);
    if (!output) throw std::runtime_error("Could not create temporary decoded audio file.");
    WriteWaveHeader(output, 0);

    std::uint64_t totalDataBytes = 0;
    int readResult = 0;
    while ((readResult = av_read_frame(state.format, state.packet)) >= 0)
    {
        if (state.packet->stream_index == audioStream)
        {
            CheckAv(avcodec_send_packet(state.codec, state.packet), "Submit E-AC-3 packet");
            DrainDecoder(state, output, totalDataBytes);
        }
        av_packet_unref(state.packet);
    }
    if (readResult != AVERROR_EOF) CheckAv(readResult, "Read encoded audio packet");
    CheckAv(avcodec_send_packet(state.codec, nullptr), "Flush E-AC-3 decoder");
    DrainDecoder(state, output, totalDataBytes);
    FlushResampler(state, output, totalDataBytes);

    if (totalDataBytes == 0 || totalDataBytes > UINT32_MAX - 36)
    {
        throw std::runtime_error("The decoded channel bed is empty or too large for temporary WAV output.");
    }
    output.seekp(0, std::ios::beg);
    WriteWaveHeader(output, static_cast<std::uint32_t>(totalDataBytes));
    output.close();
    cleanup.keep = true;
    return temporaryPath;
}
}
#else
#include <stdexcept>

namespace MagicAapSpatial
{
std::filesystem::path DecodeChannelBedToTemporaryWav(const std::filesystem::path&)
{
    throw std::runtime_error("E-AC-3 channel-bed decoding requires the FFmpeg libraries.");
}
}
#endif