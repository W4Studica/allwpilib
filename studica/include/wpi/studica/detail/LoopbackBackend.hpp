// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <map>
#include <mutex>

#include "wpi/studica/detail/HardwareBackend.hpp"
#include "wpi/studica/detail/InterruptQueue.hpp"

namespace wpi::studica {

/// In-memory backend used on a PC (no VMX hardware) and in tests.
class LoopbackBackend : public HardwareBackend {
 public:
  struct Pin {
    bool isInput = false;
    bool value = false;
  };

  bool InitDigital(int vmxChannel, bool isInput) override;
  void ReleaseDigital(int vmxChannel) override;
  void SetDigital(int vmxChannel, bool value) override;
  bool GetDigital(int vmxChannel) override;
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
  bool GetBatteryVoltage(double* volts) override;

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

  using I2cHandler =
      std::function<bool(int, const uint8_t*, int, uint8_t*, int)>;
  void SetI2cHandler(I2cHandler handler);
  void DriveDutyCycle(int channel, double frequency, double output);
  void DriveInterrupt(int channel, bool rising, uint64_t timestamp);
  /// Test hooks.
  bool IsClaimed(int vmxChannel);
  bool IsInput(int vmxChannel);
  /// Drives a pin as if an external signal changed it (input pins).
  void Drive(int vmxChannel, bool value);
  bool IsAnalogClaimed(int vmxChannel);
  void DriveAnalog(int vmxChannel, double volts);
  bool IsEncoderClaimed(int vmxChannelA);
  void DriveEncoder(int vmxChannelA, int32_t count);
  bool IsImuClaimed();
  bool IsPwmClaimed(int vmxChannel);
  int PwmPeriod(int vmxChannel);
  int PwmPulse(int vmxChannel);
  /// Sets the battery reading; valid=false makes GetBatteryVoltage() fail.
  void DriveBattery(double volts, bool valid = true);
  /// Sets the IMU reading; connected=false makes GetImu() fail.
  void DriveImu(const ImuSample& sample, bool connected = true);

 private:
  std::mutex m_mutex;
  bool m_i2cActive = false;
  I2cHandler m_i2cHandler;
  std::map<int, DutyCycleReading> m_dutyCycles;
  std::map<int, std::shared_ptr<detail::InterruptQueue>> m_interrupts;
  std::map<int, Pin> m_pins;
  std::map<int, double> m_analog;

  struct Enc {
    int channelB = 0;
    int32_t count = 0;
  };
  std::map<int, Enc> m_encoders;

  struct Pwm {
    int periodMicros = 0;
    int pulseMicros = 0;
  };
  std::map<int, Pwm> m_pwm;
  double m_battery = 0;
  bool m_batteryValid = false;

  bool m_imuClaimed = false;
  bool m_imuConnected = true;
  ImuSample m_imu;

  // Mirrors the assumption that a VMX channel can serve only one function.
  // Caller holds m_mutex.
  bool DigitalChannelBusy(int vmxChannel) const;
};

}  // namespace wpi::studica
