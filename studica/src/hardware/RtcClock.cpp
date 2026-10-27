// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/RtcClock.hpp"

#include "wpi/studica/SharedVMX.hpp"

namespace wpi::studica {
bool ReadRtc(RtcReading* reading) {
  if (!reading) {
    return false;
  }
  const auto vmx = SharedVMX();
  if (!vmx->IsOpen()) {
    return false;
  }
  for (int attempt = 0; attempt < 2; ++attempt) {
    RtcReading result;
    VMXErrorCode error = 0;
    uint8_t weekday, day, month, year;
    VMXTime::DaylightSavingsAdjustment adjustment;
    if (!vmx->time.GetRTCDate(result.weekday, result.day, result.month,
                              result.year, &error) ||
        !vmx->time.GetRTCTime(result.hour, result.minute, result.second,
                              result.milliseconds, &error) ||
        !vmx->time.GetRTCDate(weekday, day, month, year, &error) ||
        !vmx->time.GetRTCDaylightSavingsAdjustment(adjustment, &error)) {
      return false;
    }
    if (weekday != result.weekday || day != result.day ||
        month != result.month || year != result.year) {
      continue;
    }
    result.daylightSavingsAdjustment = static_cast<int>(adjustment);
    *reading = result;
    return true;
  }
  return false;
}

}  // namespace wpi::studica
