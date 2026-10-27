// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <memory>

#include "VMXPi.h"
#include "VMXResource.hpp"
#include "wpi/studica/detail/backend/HardwareBackend.hpp"
#include "wpi/studica/detail/io/InterruptQueue.hpp"
#include "wpi/studica/drivers/Imu.hpp"

namespace wpi::studica {
class StudicaBackend final : public HardwareBackend {
 public:
  StudicaBackend();
  ~StudicaBackend() override;
  bool ChannelBusy(int channel) const;
  bool ChannelCan(int vmxChannel, VMXChannelCapability capability) const;
  bool InitDigital(int vmxChannel, bool isInput) override;
  void ReleaseDigital(int vmxChannel) override;
  void SetDigital(int vmxChannel, bool value) override;
  bool GetDigital(int vmxChannel) override;
  bool TryGetDigital(int channel, bool* value) override;
  bool TrySetDigital(int channel, bool value) override;
  bool TrySetPwmPulse(int channel, int pulse) override;
  bool InitAnalog(int vmxChannel) override;
  void ReleaseAnalog(int vmxChannel) override;
  bool GetAnalogVoltage(int vmxChannel, double* volts) override;
  bool InitEncoder(int vmxChannelA, int vmxChannelB) override;
  void ReleaseEncoder(int vmxChannelA) override;
  bool GetEncoderCount(int vmxChannelA, int32_t* count) override;
  bool InitImu() override;
  void ReleaseImu() override;
  bool GetImu(ImuSample* sample) override;
  bool InitPwm(int vmxChannel, int periodMicros) override;
  void ReleasePwm(int vmxChannel) override;
  void SetPwmPulse(int vmxChannel, int pulseMicros) override;
  bool InitI2c() override;
  void ReleaseI2c() override;
  bool TransactionI2c(int address, const uint8_t* tx, int txSize, uint8_t* rx,
                      int rxSize) override;
  bool InitDutyCycle(int channel) override;
  void ReleaseDutyCycle(int channel) override;
  bool GetDutyCycle(int channel, DutyCycleReading* reading) override;
  bool InitInterrupt(int channel) override;
  void ReleaseInterrupt(int channel) override;
  bool ReadInterrupt(int channel, InterruptEvent* event) override;
  bool GetBatteryVoltage(double* volts) override;

 private:
  static constexpr int kTickMicros = 4;

  struct Pwm {
    detail::VMXResource resource;
    VMXChannelIndex index = 0;
    uint16_t maxDuty = 0;
  };

  struct Interrupt {
    detail::VMXResource resource;
    uintptr_t token = 0;
    std::shared_ptr<detail::InterruptQueue> queue;
  };
  std::map<int, Interrupt> m_interrupts;
  std::map<int, detail::VMXResource> m_dutyCycles;
  std::map<int, int> m_encoderB;
  detail::VMXResource m_i2cResource;
  bool m_i2cActive = false;
  VMXChannelIndex m_sda = INVALID_VMX_CHANNEL_INDEX;
  VMXChannelIndex m_scl = INVALID_VMX_CHANNEL_INDEX;
  std::shared_ptr<VMXPi> m_vmx;
  std::map<int, Pwm> m_pwm;
  std::map<int, detail::VMXResource> m_dio;
  std::map<int, detail::VMXResource> m_analog;
  std::map<int, detail::VMXResource> m_encoders;
  std::unique_ptr<wpi::studica::drivers::Imu> m_imu;
};
}  // namespace wpi::studica
