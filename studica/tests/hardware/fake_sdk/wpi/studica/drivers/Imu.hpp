// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once
#include <memory>

#include "VMXPi.h"
namespace wpi::studica::drivers {
class Imu {
 public:
  explicit Imu(std::shared_ptr<VMXPi> vmx) : m_vmx{std::move(vmx)} {}
  bool IsConnected() { return m_vmx->connected; }
  bool IsCalibrating() { return m_vmx->calibrating; }
  float GetYaw() { return 10; }
  float GetPitch() { return 20; }
  float GetRoll() { return 30; }
  float GetRawGyroX() { return 1; }
  float GetRawGyroY() { return 2; }
  float GetRawGyroZ() { return 3; }
  float GetRawAccelX() { return 0; }
  float GetRawAccelY() { return 0; }
  float GetRawAccelZ() { return 1; }

 private:
  std::shared_ptr<VMXPi> m_vmx;
};
}  // namespace wpi::studica::drivers
