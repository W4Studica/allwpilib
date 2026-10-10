// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <optional>
#include <string>

namespace wpilibvmx {

/**
 * Maps WPILib channel numbers to VMX channel indexes.
 *
 * The VMX-pi pin map has not been filled in yet (see DESIGN.md section 4), so
 * no mapping is assumed. Entries come from an environment variable of the form
 * "<wpilib>:<vmx>,<wpilib>:<vmx>" (for example HALSIMVMX_DIO_MAP="0:12,1:13").
 * Unmapped channels are left alone.
 */
class ChannelMap {
 public:
  ChannelMap() = default;

  /// Parses the given string. Malformed entries are skipped.
  static ChannelMap Parse(const std::string& text);

  /// Reads and parses the environment variable. Empty map if it is unset.
  static ChannelMap FromEnv(const char* envName);

  std::optional<int> Get(int wpilibChannel) const;
  size_t size() const { return m_map.size(); }

 private:
  std::map<int, int> m_map;
};

}  // namespace wpilibvmx
