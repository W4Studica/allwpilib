// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <cstdint>

namespace wpi::studica {
/// Raw calendar fields from the VMX SDK, including its month/weekday numbering
/// and two-digit year. No timezone or Unix epoch conversion is assumed.
struct RtcReading {
  uint8_t hour = 0, minute = 0, second = 0;
  uint32_t milliseconds = 0;
  uint8_t weekday = 0, day = 0, month = 0, year = 0;
  int daylightSavingsAdjustment =
      0;  // SDK: 0 none, 1 add hour, 2 subtract hour.
};

/// Link against studica_hardware. Uses the shared VMXPi. Returns false on a
/// read error or a date rollover that cannot be sampled consistently. Success
/// does not imply the RTC was set to the correct time. Does not change the RTC
/// or the OS clock.
bool ReadRtc(RtcReading* reading);
}  // namespace wpi::studica
