// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/detail/backend/LoopbackBackend.hpp"

using namespace wpi::studica;

bool LoopbackBackend::DigitalChannelBusy(int vmxChannel) const {
  if (m_pins.contains(vmxChannel)) {
    return true;
  }
  for (const auto& [a, enc] : m_encoders) {
    if (a == vmxChannel || enc.channelB == vmxChannel) {
      return true;
    }
  }
  return m_pwm.contains(vmxChannel) || m_dutyCycles.contains(vmxChannel) ||
         m_interrupts.contains(vmxChannel);
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

bool LoopbackBackend::InitPwm(int vmxChannel, int periodMicros) {
  std::scoped_lock lock{m_mutex};
  if (periodMicros <= 0 || DigitalChannelBusy(vmxChannel)) {
    return false;
  }
  m_pwm[vmxChannel] = Pwm{periodMicros, 0};
  return true;
}

void LoopbackBackend::ReleasePwm(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  m_pwm.erase(vmxChannel);
}

void LoopbackBackend::SetPwmPulse(int vmxChannel, int pulseMicros) {
  std::scoped_lock lock{m_mutex};
  if (auto it = m_pwm.find(vmxChannel); it != m_pwm.end()) {
    it->second.pulseMicros = pulseMicros;
  }
}

bool LoopbackBackend::GetBatteryVoltage(double* volts) {
  std::scoped_lock lock{m_mutex};
  if (!m_batteryValid) {
    return false;
  }
  *volts = m_battery;
  return true;
}

bool LoopbackBackend::IsPwmClaimed(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  return m_pwm.contains(vmxChannel) || m_dutyCycles.contains(vmxChannel) ||
         m_interrupts.contains(vmxChannel);
}

int LoopbackBackend::PwmPeriod(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  auto it = m_pwm.find(vmxChannel);
  return it == m_pwm.end() ? 0 : it->second.periodMicros;
}

int LoopbackBackend::PwmPulse(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  auto it = m_pwm.find(vmxChannel);
  return it == m_pwm.end() ? 0 : it->second.pulseMicros;
}

void LoopbackBackend::DriveBattery(double volts, bool valid) {
  std::scoped_lock lock{m_mutex};
  m_battery = volts;
  m_batteryValid = valid;
}

bool LoopbackBackend::InitI2c() {
  std::scoped_lock lock{m_mutex};
  if (m_i2cActive) {
    return false;
  }
  m_i2cActive = true;
  return true;
}
void LoopbackBackend::ReleaseI2c() {
  std::scoped_lock lock{m_mutex};
  m_i2cActive = false;
}
bool LoopbackBackend::TransactionI2c(int address, const uint8_t* tx, int txSize,
                                     uint8_t* rx, int rxSize) {
  std::scoped_lock lock{m_mutex};
  if (!m_i2cActive || !m_i2cHandler || address < 0 || address > 0x7f ||
      txSize < 0 || rxSize < 0 || txSize > UINT16_MAX || rxSize > UINT16_MAX ||
      (txSize && !tx) || (rxSize && !rx)) {
    return false;
  }
  return m_i2cHandler(address, tx, txSize, rx, rxSize);
}
void LoopbackBackend::SetI2cHandler(I2cHandler handler) {
  std::scoped_lock lock{m_mutex};
  m_i2cHandler = std::move(handler);
}
bool LoopbackBackend::InitDutyCycle(int channel) {
  std::scoped_lock lock{m_mutex};
  if (DigitalChannelBusy(channel)) {
    return false;
  }
  m_dutyCycles.emplace(channel, DutyCycleReading{});
  return true;
}
void LoopbackBackend::ReleaseDutyCycle(int channel) {
  std::scoped_lock lock{m_mutex};
  m_dutyCycles.erase(channel);
}
bool LoopbackBackend::GetDutyCycle(int channel, DutyCycleReading* reading) {
  std::scoped_lock lock{m_mutex};
  auto it = m_dutyCycles.find(channel);
  if (it == m_dutyCycles.end()) {
    return false;
  }
  *reading = it->second;
  return true;
}
void LoopbackBackend::DriveDutyCycle(int channel, double frequency,
                                     double output) {
  std::scoped_lock lock{m_mutex};
  if (auto it = m_dutyCycles.find(channel); it != m_dutyCycles.end()) {
    it->second = {frequency, output};
  }
}
bool LoopbackBackend::InitInterrupt(int channel) {
  std::scoped_lock lock{m_mutex};
  if (DigitalChannelBusy(channel)) {
    return false;
  }
  m_interrupts.emplace(channel, std::make_shared<detail::InterruptQueue>());
  return true;
}
void LoopbackBackend::ReleaseInterrupt(int channel) {
  std::scoped_lock lock{m_mutex};
  m_interrupts.erase(channel);
}
bool LoopbackBackend::ReadInterrupt(int channel, InterruptEvent* event) {
  std::scoped_lock lock{m_mutex};
  auto it = m_interrupts.find(channel);
  return it != m_interrupts.end() && it->second->Pop(event);
}
void LoopbackBackend::DriveInterrupt(int channel, bool rising,
                                     uint64_t timestamp) {
  std::scoped_lock lock{m_mutex};
  if (auto it = m_interrupts.find(channel); it != m_interrupts.end()) {
    it->second->Push({rising, timestamp, 0});
  }
}
