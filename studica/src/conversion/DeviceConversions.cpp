// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/detail/conversion/DeviceConversions.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <numbers>

namespace wpi::studica {

ImuReading ConvertImu(const ImuSample& s) {
  constexpr double kDegToRad = std::numbers::pi / 180.0;
  constexpr double kGravity = 9.80665;
  // navX yaw and Z rate are clockwise-positive; WPILib is
  // counterclockwise-positive. Yaw and Z acceleration were measured. Roll/pitch
  // and Z rate still need hardware verification.
  return {
      -s.yawDeg * kDegToRad,  s.rollDeg * kDegToRad,  s.pitchDeg * kDegToRad,
      s.gyroXDps * kDegToRad, s.gyroYDps * kDegToRad, -s.gyroZDps * kDegToRad,
      s.accelXG * kGravity,   s.accelYG * kGravity,   s.accelZG * kGravity};
}

QuaternionReading ImuQuaternion(const ImuReading& r) {
  const double cr = std::cos(r.roll / 2), sr = std::sin(r.roll / 2);
  const double cp = std::cos(r.pitch / 2), sp = std::sin(r.pitch / 2);
  const double cy = std::cos(r.yaw / 2), sy = std::sin(r.yaw / 2);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
          cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

DutyCycleReading ConvertDutyCycle(uint32_t periodMicros, uint32_t highMicros) {
  if (periodMicros == 0) {
    return {};
  }
  return {1e6 / periodMicros,
          std::clamp(static_cast<double>(highMicros) / periodMicros, 0.0, 1.0)};
}

int PwmPeriodMicros(int code) {
  switch (code) {
    case 0:
      return 5000;
    case 1:
    case 2:
      return 10000;
    default:
      return 20000;
  }
}

void EncoderState::Initialize(int32_t raw) {
  m_offset = raw;
  m_haveLast = false;
}

EncoderReading EncoderState::Update(int32_t raw, bool reset, bool reverse,
                                    double distancePerPulse,
                                    std::chrono::steady_clock::time_point now) {
  if (reset) {
    m_offset = raw;
  }
  // VMX counters wrap at 32 bits. Subtract in the unsigned domain and
  // preserve the bit pattern rather than overflowing a signed integer.
  const int32_t delta =
      m_haveLast ? std::bit_cast<int32_t>(static_cast<uint32_t>(raw) -
                                          static_cast<uint32_t>(m_lastRaw))
                 : 0;
  double rate = 0.0;
  if (m_haveLast) {
    const double seconds =
        std::chrono::duration<double>(now - m_lastTime).count();
    if (seconds > 0.0) {
      rate = delta * distancePerPulse / seconds * (reverse ? -1 : 1);
    }
  }
  uint32_t relative =
      static_cast<uint32_t>(raw) - static_cast<uint32_t>(m_offset);
  if (reverse)
    relative = uint32_t{0} - relative;
  const int32_t count = std::bit_cast<int32_t>(relative);
  m_lastRaw = raw;
  m_lastTime = now;
  m_haveLast = true;
  return {count, rate, (delta > 0) != reverse, delta != 0};
}

}  // namespace wpi::studica
