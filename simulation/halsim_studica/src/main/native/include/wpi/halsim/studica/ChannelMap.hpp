// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <initializer_list>
#include <map>
#include <optional>

namespace wpi::studica {

// Fixed board wiring. Explicit entries are used by backend unit tests only.
class ChannelMap {
 public:
  ChannelMap() = default;
  ChannelMap(std::initializer_list<std::pair<const int, int>> entries)
      : m_map{entries} {}
  static ChannelMap Linear(int count, int firstVmx = 0);
  std::optional<int> Get(int wpilibChannel) const;
  size_t size() const { return m_map.size(); }

 private:
  std::map<int, int> m_map;
};

struct ChannelMaps {
  ChannelMap dio;
  ChannelMap analog;
  ChannelMap pwm;
};

// Internal bridge settings; ordinary simulation does not claim hardware.
struct Config {
  ChannelMaps maps;
  bool imu = false;
  int i2cPort = -1;

  // DIO/PWM use VMX channel indexes; AnalogInput 0..3 uses VMX 22..25.
  static Config ForVmx();
  static Config ForRuntime();
};

}  // namespace wpi::studica
