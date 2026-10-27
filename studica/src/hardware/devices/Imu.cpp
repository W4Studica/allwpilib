// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../StudicaBackend.hpp"

namespace wpi::studica {
bool StudicaBackend::InitImu() {
  if (!m_vmx || !m_vmx->IsOpen() || m_imu) {
    return false;
  }
  // Use the constructor that takes our VMXPi: Imu() would create another.
  m_imu = std::make_unique<wpi::studica::drivers::Imu>(m_vmx);
  return true;
}

void StudicaBackend::ReleaseImu() {
  m_imu.reset();
}

bool StudicaBackend::GetImu(ImuSample* sample) {
  if (!m_imu || !m_imu->IsConnected() || m_imu->IsCalibrating()) {
    return false;
  }
  sample->yawDeg = m_imu->GetYaw();
  sample->pitchDeg = m_imu->GetPitch();
  sample->rollDeg = m_imu->GetRoll();
  sample->gyroXDps = m_imu->GetRawGyroX();
  sample->gyroYDps = m_imu->GetRawGyroY();
  sample->gyroZDps = m_imu->GetRawGyroZ();
  sample->accelXG = m_imu->GetRawAccelX();
  sample->accelYG = m_imu->GetRawAccelY();
  sample->accelZG = m_imu->GetRawAccelZ();
  return true;
}
}  // namespace wpi::studica
