// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <mutex>

#include "wpi/halsim/vmx/VmxBackend.hpp"

namespace wpilibvmx {

/// In-memory backend used on a PC (no VMX hardware) and in tests.
class LoopbackBackend : public VmxBackend {
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
  /// Sets the IMU reading; connected=false makes GetImu() fail.
  void DriveImu(const ImuSample& sample, bool connected = true);

 private:
  std::mutex m_mutex;
  std::map<int, Pin> m_pins;
  std::map<int, double> m_analog;

  struct Enc {
    int channelB = 0;
    int32_t count = 0;
  };
  std::map<int, Enc> m_encoders;

  bool m_imuClaimed = false;
  bool m_imuConnected = true;
  ImuSample m_imu;

  // Mirrors the assumption that a VMX channel can serve only one function.
  // Caller holds m_mutex.
  bool DigitalChannelBusy(int vmxChannel) const;
};

}  // namespace wpilibvmx
