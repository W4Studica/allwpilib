// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <chrono>
#include <cstdint>

#include "wpi/studica/detail/backend/HardwareBackend.hpp"

namespace wpi::studica {

/// WPILib coordinates and SI units: radians, radians/s and m/s^2.
struct ImuReading {
  double yaw, roll, pitch;
  double gyroX, gyroY, gyroZ;
  double accelX, accelY, accelZ;
};

struct QuaternionReading {
  double w = 1, x = 0, y = 0, z = 0;
};
QuaternionReading ImuQuaternion(const ImuReading& reading);
DutyCycleReading ConvertDutyCycle(uint32_t periodMicros, uint32_t highMicros);

ImuReading ConvertImu(const ImuSample& sample);
int PwmPeriodMicros(int periodCode);

struct EncoderReading {
  int32_t count;
  double rate;
  bool direction;
  bool directionChanged;
};

/// Provides reset, direction and distance-per-pulse semantics over raw driver
/// counts.
class EncoderState {
 public:
  void Initialize(int32_t raw);
  EncoderReading Update(int32_t raw, bool reset, bool reverse,
                        double distancePerPulse,
                        std::chrono::steady_clock::time_point now);

 private:
  int32_t m_offset = 0;
  int32_t m_lastRaw = 0;
  bool m_haveLast = false;
  std::chrono::steady_clock::time_point m_lastTime;
};

}  // namespace wpi::studica
