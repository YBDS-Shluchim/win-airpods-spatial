#pragma once

#include <filesystem>

namespace MagicAapSpatial
{
std::filesystem::path DecodeChannelBedToTemporaryWav(const std::filesystem::path& sourcePath);
}