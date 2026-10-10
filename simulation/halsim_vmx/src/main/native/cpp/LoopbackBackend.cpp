// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/vmx/LoopbackBackend.hpp"

using namespace wpilibvmx;

bool LoopbackBackend::DigitalChannelBusy(int vmxChannel) const {
  if (m_pins.contains(vmxChannel)) {
    return true;
  }
  for (const auto& [a, enc] : m_encoders) {
    if (a == vmxChannel || enc.channelB == vmxChannel) {
      return true;
    }
  }
  return false;
}

bool LoopbackBackend::InitDigital(int vmxChannel, bool isInput) {
  std::scoped_lock lock{m_mutex};
  if (DigitalChannelBusy(vmxChannel)) {
    return false;  // already claimed
  }
  auto it = m_pins.try_emplace(vmxChannel).first;
  it->second.isInput = isInput;
  return true;
}

void LoopbackBackend::ReleaseDigital(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  m_pins.erase(vmxChannel);
}

void LoopbackBackend::SetDigital(int vmxChannel, bool value) {
  std::scoped_lock lock{m_mutex};
  if (auto it = m_pins.find(vmxChannel); it != m_pins.end()) {
    it->second.value = value;
  }
}

bool LoopbackBackend::GetDigital(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  if (auto it = m_pins.find(vmxChannel); it != m_pins.end()) {
    return it->second.value;
  }
  return false;
}

bool LoopbackBackend::IsClaimed(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  return m_pins.contains(vmxChannel);
}

bool LoopbackBackend::IsInput(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  auto it = m_pins.find(vmxChannel);
  return it != m_pins.end() && it->second.isInput;
}

void LoopbackBackend::Drive(int vmxChannel, bool value) {
  SetDigital(vmxChannel, value);
}

bool LoopbackBackend::InitAnalog(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  return m_analog.try_emplace(vmxChannel, 0.0).second;
}

void LoopbackBackend::ReleaseAnalog(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  m_analog.erase(vmxChannel);
}

bool LoopbackBackend::GetAnalogVoltage(int vmxChannel, double* volts) {
  std::scoped_lock lock{m_mutex};
  auto it = m_analog.find(vmxChannel);
  if (it == m_analog.end()) {
    return false;
  }
  *volts = it->second;
  return true;
}

bool LoopbackBackend::IsAnalogClaimed(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  return m_analog.contains(vmxChannel);
}

void LoopbackBackend::DriveAnalog(int vmxChannel, double volts) {
  std::scoped_lock lock{m_mutex};
  if (auto it = m_analog.find(vmxChannel); it != m_analog.end()) {
    it->second = volts;
  }
}

bool LoopbackBackend::InitEncoder(int vmxChannelA, int vmxChannelB) {
  std::scoped_lock lock{m_mutex};
  if (vmxChannelA == vmxChannelB || DigitalChannelBusy(vmxChannelA) ||
      DigitalChannelBusy(vmxChannelB)) {
    return false;
  }
  m_encoders[vmxChannelA].channelB = vmxChannelB;
  return true;
}

void LoopbackBackend::ReleaseEncoder(int vmxChannelA) {
  std::scoped_lock lock{m_mutex};
  m_encoders.erase(vmxChannelA);
}

bool LoopbackBackend::GetEncoderCount(int vmxChannelA, int32_t* count) {
  std::scoped_lock lock{m_mutex};
  auto it = m_encoders.find(vmxChannelA);
  if (it == m_encoders.end()) {
    return false;
  }
  *count = it->second.count;
  return true;
}

bool LoopbackBackend::IsEncoderClaimed(int vmxChannelA) {
  std::scoped_lock lock{m_mutex};
  return m_encoders.contains(vmxChannelA);
}

void LoopbackBackend::DriveEncoder(int vmxChannelA, int32_t count) {
  std::scoped_lock lock{m_mutex};
  if (auto it = m_encoders.find(vmxChannelA); it != m_encoders.end()) {
    it->second.count = count;
  }
}

bool LoopbackBackend::InitImu() {
  std::scoped_lock lock{m_mutex};
  if (m_imuClaimed) {
    return false;
  }
  m_imuClaimed = true;
  return true;
}

void LoopbackBackend::ReleaseImu() {
  std::scoped_lock lock{m_mutex};
  m_imuClaimed = false;
}

bool LoopbackBackend::GetImu(ImuSample* sample) {
  std::scoped_lock lock{m_mutex};
  if (!m_imuClaimed || !m_imuConnected) {
    return false;
  }
  *sample = m_imu;
  return true;
}

bool LoopbackBackend::IsImuClaimed() {
  std::scoped_lock lock{m_mutex};
  return m_imuClaimed;
}

void LoopbackBackend::DriveImu(const ImuSample& sample, bool connected) {
  std::scoped_lock lock{m_mutex};
  m_imu = sample;
  m_imuConnected = connected;
}
