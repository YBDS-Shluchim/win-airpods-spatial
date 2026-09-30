#include "core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace MagicAapSpatial
{
namespace
{
constexpr std::array<std::uint8_t, 4> kAapPrefix{0x04, 0x00, 0x04, 0x00};
constexpr double kQuaternionScale = 32768.0;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

struct Field
{
    std::uint32_t number;
    std::uint32_t wireType;
    std::uint64_t value;
    std::span<const std::uint8_t> bytes;
};

bool ReadVarint(
    std::span<const std::uint8_t> data,
    std::size_t& offset,
    std::uint64_t& value)
{
    value = 0;
    for (unsigned shift = 0; shift < 64 && offset < data.size(); shift += 7)
    {
        const auto byte = data[offset++];
        if (shift == 63 && (byte & 0xfeu) != 0)
        {
            return false;
        }
        value |= static_cast<std::uint64_t>(byte & 0x7fu) << shift;
        if ((byte & 0x80u) == 0)
        {
            return true;
        }
    }
    return false;
}

std::optional<std::vector<Field>> ReadFields(std::span<const std::uint8_t> data)
{
    std::vector<Field> fields;
    std::size_t offset = 0;
    while (offset < data.size())
    {
        std::uint64_t key = 0;
        if (!ReadVarint(data, offset, key)) return std::nullopt;
        const auto number = static_cast<std::uint32_t>(key >> 3);
        const auto wireType = static_cast<std::uint32_t>(key & 0x07u);
        if (number == 0) return std::nullopt;

        if (wireType == 0)
        {
            std::uint64_t value = 0;
            if (!ReadVarint(data, offset, value)) return std::nullopt;
            fields.push_back({number, wireType, value, {}});
        }
        else if (wireType == 1 || wireType == 5)
        {
            const std::size_t length = wireType == 1 ? 8 : 4;
            if (data.size() - offset < length) return std::nullopt;
            fields.push_back({number, wireType, 0, data.subspan(offset, length)});
            offset += length;
        }
        else if (wireType == 2)
        {
            std::uint64_t rawLength = 0;
            if (!ReadVarint(data, offset, rawLength) ||
                rawLength > data.size() - offset)
            {
                return std::nullopt;
            }
            const auto length = static_cast<std::size_t>(rawLength);
            fields.push_back({number, wireType, 0, data.subspan(offset, length)});
            offset += length;
        }
        else
        {
            return std::nullopt;
        }
    }
    return fields;
}

std::uint16_t ReadUInt16LE(std::span<const std::uint8_t> data, std::size_t offset)
{
    return static_cast<std::uint16_t>(data[offset]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[offset + 1]) << 8);
}

std::uint32_t ReadUInt32LE(std::span<const std::uint8_t> data, std::size_t offset)
{
    return static_cast<std::uint32_t>(data[offset]) |
        (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
        (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
        (static_cast<std::uint32_t>(data[offset + 3]) << 24);
}

std::int16_t ReadInt16LE(std::span<const std::uint8_t> data, std::size_t offset)
{
    return static_cast<std::int16_t>(ReadUInt16LE(data, offset));
}

Quaternion Normalize(Quaternion value)
{
    const double length = std::sqrt(
        value[0] * value[0] + value[1] * value[1] +
        value[2] * value[2] + value[3] * value[3]);
    if (!std::isfinite(length) || length <= std::numeric_limits<double>::epsilon())
    {
        return {0.0, 0.0, 0.0, 1.0};
    }
    for (auto& component : value) component /= length;
    return value;
}

Quaternion Conjugate(const Quaternion& value)
{
    return {-value[0], -value[1], -value[2], value[3]};
}

Quaternion Multiply(const Quaternion& left, const Quaternion& right)
{
    const auto [x1, y1, z1, w1] = left;
    const auto [x2, y2, z2, w2] = right;
    return {
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2};
}

double Dot(const Quaternion& left, const Quaternion& right)
{
    return left[0] * right[0] + left[1] * right[1] +
        left[2] * right[2] + left[3] * right[3];
}

Quaternion MarkleyAverage(const std::vector<Quaternion>& samples)
{
    std::array<std::array<double, 4>, 4> matrix{};
    for (const auto& sample : samples)
    {
        for (std::size_t row = 0; row < 4; row++)
        {
            for (std::size_t column = 0; column < 4; column++)
            {
                matrix[row][column] += sample[row] * sample[column];
            }
        }
    }

    auto eigenvector = samples.front();
    for (int iteration = 0; iteration < 24; iteration++)
    {
        Quaternion next{};
        for (std::size_t row = 0; row < 4; row++)
        {
            for (std::size_t column = 0; column < 4; column++)
            {
                next[row] += matrix[row][column] * eigenvector[column];
            }
        }
        eigenvector = Normalize(next);
    }
    return eigenvector;
}

Quaternion Slerp(Quaternion from, Quaternion to, double amount)
{
    double dot = Dot(from, to);
    if (dot < 0.0)
    {
        for (auto& component : to) component = -component;
        dot = -dot;
    }
    dot = std::clamp(dot, -1.0, 1.0);
    if (dot > 0.9995)
    {
        for (std::size_t index = 0; index < 4; index++)
        {
            from[index] += amount * (to[index] - from[index]);
        }
        return Normalize(from);
    }

    const double angle = std::acos(dot);
    const double denominator = std::sin(angle);
    const double leftWeight = std::sin((1.0 - amount) * angle) / denominator;
    const double rightWeight = std::sin(amount * angle) / denominator;
    for (std::size_t index = 0; index < 4; index++)
    {
        from[index] = leftWeight * from[index] + rightWeight * to[index];
    }
    return Normalize(from);
}

std::array<double, 3> ToYawPitchRoll(const Quaternion& q)
{
    const auto [x, y, z, w] = q;
    const double r02 = 2.0 * (x * z + y * w);
    const double r22 = 1.0 - 2.0 * (x * x + y * y);
    const double r12 = 2.0 * (y * z - x * w);
    const double r10 = 2.0 * (x * y + z * w);
    const double r11 = 1.0 - 2.0 * (x * x + z * z);
    return {
        -std::atan2(r02, r22) * kRadiansToDegrees,
        -std::atan2(r10, r11) * kRadiansToDegrees,
        std::asin(std::clamp(-r12, -1.0, 1.0)) * kRadiansToDegrees};
}

std::size_t LongestPrefixSuffix(std::span<const std::uint8_t> data)
{
    const auto maxLength = std::min(kAapPrefix.size() - 1, data.size());
    for (std::size_t length = maxLength; length > 0; length--)
    {
        if (std::equal(data.end() - static_cast<std::ptrdiff_t>(length), data.end(), kAapPrefix.begin()))
        {
            return length;
        }
    }
    return 0;
}
}

std::vector<Packet> AapStreamFramer::Push(std::span<const std::uint8_t> chunk)
{
    pending_.insert(pending_.end(), chunk.begin(), chunk.end());
    std::vector<Packet> packets;

    while (!pending_.empty())
    {
        const auto prefix = std::search(pending_.begin(), pending_.end(), kAapPrefix.begin(), kAapPrefix.end());
        if (prefix == pending_.end())
        {
            const auto suffixLength = LongestPrefixSuffix(pending_);
            const auto rawLength = pending_.size() - suffixLength;
            if (rawLength == 0) break;
            packets.emplace_back(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(rawLength));
            pending_.erase(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(rawLength));
            continue;
        }

        const auto prefixIndex = static_cast<std::size_t>(std::distance(pending_.begin(), prefix));
        if (prefixIndex > 0)
        {
            packets.emplace_back(pending_.begin(), prefix);
            pending_.erase(pending_.begin(), prefix);
            continue;
        }

        const auto nextPrefix = std::search(prefix + 1, pending_.end(), kAapPrefix.begin(), kAapPrefix.end());
        if (nextPrefix == pending_.end()) break;
        packets.emplace_back(pending_.begin(), nextPrefix);
        pending_.erase(pending_.begin(), nextPrefix);
    }
    return packets;
}

std::vector<Packet> AapStreamFramer::Complete()
{
    if (pending_.empty()) return {};
    std::vector<Packet> packets;
    packets.push_back(std::move(pending_));
    pending_.clear();
    return packets;
}

std::optional<MotionFrame> DecodeMotionPacket(std::span<const std::uint8_t> packet)
{
    if (packet.size() < 12 ||
        !std::equal(kAapPrefix.begin(), kAapPrefix.end(), packet.begin()) ||
        packet[4] != 0x17 || packet[5] != 0x00 ||
        ReadUInt32LE(packet, 6) != 0x00100000)
    {
        return std::nullopt;
    }

    const auto declaredLength = ReadUInt16LE(packet, 10);
    const auto bodyLength = std::min<std::size_t>(declaredLength, packet.size() - 12);
    const auto rootFields = ReadFields(packet.subspan(12, bodyLength));
    if (!rootFields) return std::nullopt;

    std::optional<std::uint32_t> service;
    std::span<const std::uint8_t> payload;
    for (const auto& rootField : *rootFields)
    {
        if (rootField.number != 7 || rootField.wireType != 2) continue;
        const auto commandFields = ReadFields(rootField.bytes);
        if (!commandFields) return std::nullopt;
        for (const auto& commandField : *commandFields)
        {
            if (commandField.number == 1 && commandField.wireType == 0 &&
                commandField.value <= std::numeric_limits<std::uint32_t>::max())
            {
                service = static_cast<std::uint32_t>(commandField.value);
            }
            else if (commandField.number == 3 && commandField.wireType == 2)
            {
                payload = commandField.bytes;
            }
        }
        break;
    }

    if (!service || (*service != 6 && *service != 14 && *service != 16) ||
        payload.size() < 58 || payload[0] != 1 || payload[9] != 3)
    {
        return std::nullopt;
    }

    const double x = ReadInt16LE(payload, 20) / kQuaternionScale;
    const double y = ReadInt16LE(payload, 22) / kQuaternionScale;
    const double z = ReadInt16LE(payload, 24) / kQuaternionScale;
    const double vectorLengthSquared = x * x + y * y + z * z;
    if (vectorLengthSquared < 0.01 || vectorLengthSquared > 1.01)
    {
        return std::nullopt;
    }

    const auto orientation = Normalize({
        x,
        y,
        z,
        std::sqrt(std::max(0.0, 1.0 - vectorLengthSquared))});
    return MotionFrame{
        orientation,
        ReadInt16LE(payload, 28),
        ReadInt16LE(payload, 30),
        *service};
}

PoseEstimator::PoseEstimator(std::size_t calibrationSamples)
    : calibrationSamples_(calibrationSamples)
{
    if (calibrationSamples_ == 0)
    {
        throw std::invalid_argument("calibration sample count must be positive");
    }
}

std::optional<PoseResult> PoseEstimator::ProcessPacket(std::span<const std::uint8_t> packet)
{
    const auto motion = DecodeMotionPacket(packet);
    if (!motion) return std::nullopt;

    if (!neutral_)
    {
        calibration_.push_back(motion->orientation);
        if (calibration_.size() < calibrationSamples_)
        {
            return PoseResult{
                false,
                calibration_.size(),
                std::nullopt,
                std::nullopt,
                std::nullopt,
                0,
                0};
        }
        neutral_ = MarkleyAverage(calibration_);
        smoothed_ = Quaternion{0.0, 0.0, 0.0, 1.0};
        calibration_.clear();
        return PoseResult{
            true,
            calibrationSamples_,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            0,
            0};
    }

    auto relative = Normalize(Multiply(Conjugate(*neutral_), motion->orientation));
    if (relative[3] < 0.0)
    {
        for (auto& component : relative) component = -component;
    }

    if (!smoothed_)
    {
        smoothed_ = relative;
    }
    else
    {
        const auto delta = Normalize(Multiply(Conjugate(*smoothed_), relative));
        const double angle = 2.0 * std::acos(std::clamp(std::abs(delta[3]), 0.0, 1.0)) * kRadiansToDegrees;
        const double amount = std::clamp(0.12 + angle * 0.018, 0.12, 0.42);
        smoothed_ = Slerp(*smoothed_, relative, amount);
    }

    const auto angles = ToYawPitchRoll(*smoothed_);
    return PoseResult{
        true,
        calibrationSamples_,
        angles[0],
        angles[1],
        angles[2],
        motion->horizontalAcceleration,
        motion->verticalAcceleration};
}

bool PoseEstimator::IsCalibrated() const noexcept
{
    return neutral_.has_value();
}

std::size_t PoseEstimator::CalibrationCount() const noexcept
{
    return calibration_.size();
}

void PoseEstimator::Reset()
{
    calibration_.clear();
    neutral_.reset();
    smoothed_.reset();
}
}