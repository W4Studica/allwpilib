// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/studica/ChannelMap.hpp"

#include "wpi/studica/detail/backend/HardwareBackend.hpp"

using namespace wpi::studica;

ChannelMap ChannelMap::Linear(int count, int firstVmx) {
  ChannelMap result;
  for (int channel = 0; channel < count; ++channel) {
    result.m_map.emplace(channel, firstVmx + channel);
  }
  return result;
}

std::optional<int> ChannelMap::Get(int wpilibChannel) const {
  auto it = m_map.find(wpilibChannel);
  return it == m_map.end() ? std::nullopt : std::optional<int>{it->second};
}

Config Config::ForVmx() {
  Config config;
  config.maps.dio = ChannelMap::Linear(22);
  config.maps.analog = ChannelMap::Linear(4, 22);
  config.maps.pwm = ChannelMap::Linear(22);
  config.imu = true;
  config.i2cPort = 1;
  return config;
}

Config Config::ForRuntime() {
  return IsHardwareRuntime() ? ForVmx() : Config{};
}
