// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace wpi::studica {

/**
 * Maps WPILib channel numbers to VMX channel indexes.
 *
 * No mapping is assumed. Entries come from an environment variable of the form
 * "<wpilib>:<vmx>,<wpilib>:<vmx>" (for example HALSIMSTUDICA_DIO_MAP="0:12,1:13").
 * Unmapped channels are left alone.
 */
class ChannelMap {
 public:
  ChannelMap() = default;

  /// Parses the given string. Malformed entries are skipped and listed by Rejected().
  static ChannelMap Parse(const std::string& text);

  /// Reads and parses the environment variable. Empty map if it is unset.
  static ChannelMap FromEnv(const char* envName);

  std::optional<int> Get(int wpilibChannel) const;
  size_t size() const { return m_map.size(); }

  /// Non-empty entries that could not be parsed, as written (for example "7;8" or " 7:4").
  const std::vector<std::string>& Rejected() const { return m_rejected; }

  /// VMX channels that more than one WPILib channel is mapped to. Only one of them can claim it.
  std::vector<int> DuplicateVmxChannels() const;

 private:
  std::map<int, int> m_map;
  std::vector<std::string> m_rejected;
};

/// One map per kind of channel. Each is read from its own environment variable.
struct ChannelMaps {
  ChannelMap dio;
  ChannelMap analog;
  ChannelMap pwm;

  /// HALSIMSTUDICA_DIO_MAP, HALSIMSTUDICA_ANALOG_MAP and HALSIMSTUDICA_PWM_MAP.
  static ChannelMaps FromEnv();
};

/// Everything halsim_studica is told by its environment.
struct Config {
  ChannelMaps maps;
  /// HALSIMSTUDICA_IMU=1. The sim IMU has no "initialized" signal, so it is opt-in.
  bool imu = false;

  static Config FromEnv();
};

}  // namespace wpi::studica
