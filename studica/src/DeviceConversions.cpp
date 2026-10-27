// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/DeviceConversions.hpp"

#include <numbers>

namespace wpi::studica {

ImuReading ConvertImu(const ImuSample& s) {
  constexpr double kDegToRad = std::numbers::pi / 180.0;
  constexpr double kGravity = 9.80665;
  // navX yaw and Z rate are clockwise-positive; WPILib is counterclockwise-positive.
  // Yaw and Z acceleration were measured. Roll/pitch and Z rate still need hardware verification.
  return {-s.yawDeg * kDegToRad, s.rollDeg * kDegToRad,
          s.pitchDeg * kDegToRad, s.gyroXDps * kDegToRad,
          s.gyroYDps * kDegToRad, -s.gyroZDps * kDegToRad,
          s.accelXG * kGravity, s.accelYG * kGravity, s.accelZG * kGravity};
}

int PwmPeriodMicros(int code) {
  switch (code) {
    case 0: return 5000;
    case 1:
    case 2: return 10000;
    default: return 20000;
  }
}

void EncoderState::Initialize(int32_t raw) {
  m_offset = raw;
  m_haveLast = false;
}

EncoderReading EncoderState::Update(
    int32_t raw, bool reset, bool reverse, double distancePerPulse,
    std::chrono::steady_clock::time_point now) {
  if (reset) {
    m_offset = raw;
  }
  const int32_t delta = m_haveLast ? raw - m_lastRaw : 0;
  double rate = 0.0;
  if (m_haveLast) {
    const double seconds = std::chrono::duration<double>(now - m_lastTime).count();
    if (seconds > 0.0) {
      rate = delta * distancePerPulse / seconds * (reverse ? -1 : 1);
    }
  }
  const int32_t count = (raw - m_offset) * (reverse ? -1 : 1);
  m_lastRaw = raw;
  m_lastTime = now;
  m_haveLast = true;
  return {count, rate, (delta > 0) != reverse, delta != 0};
}

}  // namespace wpi::studica
