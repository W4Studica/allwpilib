// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/vmx/ChannelMap.hpp"

#include <charconv>
#include <cstdlib>
#include <string_view>
#include <cstring>

using namespace wpilibvmx;

namespace {

bool ParseInt(std::string_view text, int* out) {
  if (text.empty()) {
    return false;
  }
  auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), *out);
  return ec == std::errc{} && ptr == text.data() + text.size();
}

}  // namespace

ChannelMap ChannelMap::Parse(const std::string& text) {
  ChannelMap result;
  std::string_view rest{text};
  while (!rest.empty()) {
    auto comma = rest.find(',');
    std::string_view entry = rest.substr(0, comma);
    rest = comma == std::string_view::npos ? std::string_view{}
                                           : rest.substr(comma + 1);

    auto colon = entry.find(':');
    if (colon == std::string_view::npos) {
      continue;
    }
    int wpilib = 0;
    int vmx = 0;
    if (ParseInt(entry.substr(0, colon), &wpilib) &&
        ParseInt(entry.substr(colon + 1), &vmx)) {
      result.m_map[wpilib] = vmx;
    }
  }
  return result;
}

ChannelMap ChannelMap::FromEnv(const char* envName) {
  const char* value = std::getenv(envName);
  return value ? Parse(value) : ChannelMap{};
}

std::optional<int> ChannelMap::Get(int wpilibChannel) const {
  auto it = m_map.find(wpilibChannel);
  if (it == m_map.end()) {
    return std::nullopt;
  }
  return it->second;
}

ChannelMaps ChannelMaps::FromEnv() {
  return {ChannelMap::FromEnv("HALSIMVMX_DIO_MAP"),
          ChannelMap::FromEnv("HALSIMVMX_ANALOG_MAP")};
}

Config Config::FromEnv() {
  Config config;
  config.maps = ChannelMaps::FromEnv();
  const char* imu = std::getenv("HALSIMVMX_IMU");
  config.imu = imu != nullptr && std::strcmp(imu, "1") == 0;
  return config;
}
