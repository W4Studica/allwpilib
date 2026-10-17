// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Backend plugin for VMX-pi (WPILIB_WITH_STUDICA=ON), built on the device as
// libhalsim_vmx_studica.so and loaded by halsim_vmx through HALSIMVMX_BACKEND.
// Calls the Studica drivers directly. NOT YET COMPILED OR RUN: VMXPi.h and
// vmxpi_hal_cpp are only available on a VMX-pi.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>

#include "VMXPi.h"
#include "analog_input.hpp"
#include "dio.hpp"
#include "encoder.hpp"
#include "imu.hpp"
#include "wpi/halsim/vmx/BackendPlugin.hpp"
#include "wpi/halsim/vmx/SharedVMX.hpp"
#include "wpi/halsim/vmx/VmxBackend.hpp"

namespace wpilibvmx {

std::shared_ptr<VMXPi> SharedVMX() {
  // Thread-safe lazy initialization. Never reset: devices may outlive any one backend.
  static std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50);
  return vmx;
}

class StudicaBackend : public VmxBackend {
 public:
  StudicaBackend() : m_vmx{SharedVMX()} {}

  /// VMXChannelIndex is a uint8_t: an out-of-range number would wrap around to another channel.
  /// The Studica Analog/Encoder drivers report nothing when they cannot claim a channel, so check
  /// here that the channel exists and has the function we are about to ask for.
  bool ChannelCan(int vmxChannel, VMXChannelCapability capability) const {
    if (vmxChannel < 0 || vmxChannel >= INVALID_VMX_CHANNEL_INDEX) {
      return false;
    }
    return m_vmx->io.ChannelSupportsCapability(
        static_cast<VMXChannelIndex>(vmxChannel), capability);
  }

  bool InitDigital(int vmxChannel, bool isInput) override {
    if (!m_vmx || !m_vmx->IsOpen() || m_dio.contains(vmxChannel) ||
        vmxChannel < 0 || vmxChannel >= INVALID_VMX_CHANNEL_INDEX) {
      return false;
    }
    auto dio = std::make_unique<studica_driver::DIO>(
        static_cast<VMXChannelIndex>(vmxChannel),
        isInput ? studica_driver::PinMode::INPUT
                : studica_driver::PinMode::OUTPUT,
        m_vmx);
    if (!dio->IsInitialized()) {
      return false;
    }
    m_dio.emplace(vmxChannel, std::move(dio));
    return true;
  }

  void ReleaseDigital(int vmxChannel) override { m_dio.erase(vmxChannel); }

  void SetDigital(int vmxChannel, bool value) override {
    if (auto it = m_dio.find(vmxChannel); it != m_dio.end()) {
      it->second->Set(value);
    }
  }

  bool GetDigital(int vmxChannel) override {
    if (auto it = m_dio.find(vmxChannel); it != m_dio.end()) {
      return it->second->Get();
    }
    return false;
  }

  bool InitAnalog(int vmxChannel) override {
    if (!m_vmx || !m_vmx->IsOpen() || m_analog.contains(vmxChannel) ||
        !ChannelCan(vmxChannel, VMXChannelCapability::AccumulatorInput)) {
      return false;
    }
    m_analog.emplace(vmxChannel,
                     std::make_unique<studica_driver::AnalogInput>(
                         static_cast<VMXChannelIndex>(vmxChannel), m_vmx));
    return true;
  }

  void ReleaseAnalog(int vmxChannel) override { m_analog.erase(vmxChannel); }

  bool GetAnalogVoltage(int vmxChannel, double* volts) override {
    auto it = m_analog.find(vmxChannel);
    if (it == m_analog.end()) {
      return false;
    }
    float v = 0.0f;
    if (!it->second->GetAverageVoltage(v)) {
      return false;
    }
    *volts = v;
    return true;
  }

  bool InitEncoder(int vmxChannelA, int vmxChannelB) override {
    if (!m_vmx || !m_vmx->IsOpen() || m_encoders.contains(vmxChannelA) ||
        !ChannelCan(vmxChannelA, VMXChannelCapability::EncoderAInput) ||
        !ChannelCan(vmxChannelB, VMXChannelCapability::EncoderBInput)) {
      return false;
    }
    m_encoders.emplace(
        vmxChannelA,
        std::make_unique<studica_driver::Encoder>(
            static_cast<VMXChannelIndex>(vmxChannelA),
            static_cast<VMXChannelIndex>(vmxChannelB), m_vmx));
    return true;
  }

  void ReleaseEncoder(int vmxChannelA) override {
    m_encoders.erase(vmxChannelA);
  }

  bool GetEncoderCount(int vmxChannelA, int32_t* count) override {
    auto it = m_encoders.find(vmxChannelA);
    if (it == m_encoders.end()) {
      return false;
    }
    *count = it->second->GetCount();
    return true;
  }

  bool InitImu() override {
    if (!m_vmx || !m_vmx->IsOpen() || m_imu) {
      return false;
    }
    // Use the constructor that takes our VMXPi: Imu() would create another.
    m_imu = std::make_unique<studica_driver::Imu>(m_vmx);
    return true;
  }

  void ReleaseImu() override { m_imu.reset(); }

  bool GetImu(ImuSample* sample) override {
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

  /// PWM goes straight to the VMX API, not through studica_driver::PWM/Servo: those have no way to
  /// write a pulse width, and Servo prints a line on every change. A tick is 4 us (a 20 ms frame
  /// is 5000 ticks, the resolution studica_driver::PWM uses), so duty ticks = microseconds / 4.
  bool InitPwm(int vmxChannel, int periodMicros) override {
    if (!m_vmx || !m_vmx->IsOpen() || m_pwm.contains(vmxChannel) ||
        periodMicros < kTickMicros || periodMicros > kTickMicros * 65535 ||
        !ChannelCan(vmxChannel, VMXChannelCapability::PWMGeneratorOutput)) {
      return false;
    }
    PWMGeneratorConfig config(static_cast<uint32_t>(1000000 / periodMicros));
    const auto maxDuty = static_cast<uint16_t>(periodMicros / kTickMicros);
    config.SetMaxDutyCycleValue(maxDuty);
    Pwm pwm;
    pwm.index = static_cast<VMXChannelIndex>(vmxChannel);
    pwm.maxDuty = maxDuty;
    VMXErrorCode err = 0;
    if (!m_vmx->io.ActivateSinglechannelResource(
            VMXChannelInfo(pwm.index, VMXChannelCapability::PWMGeneratorOutput),
            &config, pwm.handle, &err)) {
      std::fprintf(stderr, "HALSim VMX: PWM on VMX channel %d: %s\n", vmxChannel,
                   GetVMXErrorString(err));
      return false;
    }
    m_pwm.emplace(vmxChannel, pwm);
    SetPwmPulse(vmxChannel, 0);
    return true;
  }

  void ReleasePwm(int vmxChannel) override {
    auto it = m_pwm.find(vmxChannel);
    if (it == m_pwm.end()) {
      return;
    }
    SetPwmPulse(vmxChannel, 0);
    VMXErrorCode err = 0;
    m_vmx->io.DeallocateResource(it->second.handle, &err);
    m_pwm.erase(it);
  }

  void SetPwmPulse(int vmxChannel, int pulseMicros) override {
    auto it = m_pwm.find(vmxChannel);
    if (it == m_pwm.end()) {
      return;
    }
    const long ticks = std::lround(static_cast<double>(pulseMicros) / kTickMicros);
    const auto duty = static_cast<uint16_t>(
        std::clamp<long>(ticks, 0, it->second.maxDuty));
    VMXErrorCode err = 0;
    if (!m_vmx->io.PWMGenerator_SetDutyCycle(it->second.handle, it->second.index, duty, &err)) {
      std::fprintf(stderr, "HALSim VMX: PWM duty on VMX channel %d: %s\n", vmxChannel,
                   GetVMXErrorString(err));
    }
  }

  bool GetBatteryVoltage(double* volts) override {
    if (!m_vmx || !m_vmx->IsOpen()) {
      return false;
    }
    float v = 0.0f;
    VMXErrorCode err = 0;
    if (!m_vmx->power.GetSystemVoltage(v, &err)) {
      return false;
    }
    *volts = v;
    return true;
  }

 private:
  static constexpr int kTickMicros = 4;

  struct Pwm {
    VMXResourceHandle handle{};
    VMXChannelIndex index = 0;
    uint16_t maxDuty = 0;
  };

  std::shared_ptr<VMXPi> m_vmx;
  std::map<int, Pwm> m_pwm;
  std::map<int, std::unique_ptr<studica_driver::DIO>> m_dio;
  std::map<int, std::unique_ptr<studica_driver::AnalogInput>> m_analog;
  std::map<int, std::unique_ptr<studica_driver::Encoder>> m_encoders;
  std::unique_ptr<studica_driver::Imu> m_imu;
};

}  // namespace wpilibvmx

extern "C" {

WPILIBVMX_PLUGIN_EXPORT wpilibvmx_BackendApi* wpilibvmx_CreateBackendV2(void) {
  try {
    return wpilibvmx::MakeBackendApi(
        std::make_unique<wpilibvmx::StudicaBackend>());
  } catch (...) {
    return nullptr;
  }
}

WPILIBVMX_PLUGIN_EXPORT void wpilibvmx_DestroyBackendV2(
    wpilibvmx_BackendApi* api) {
  wpilibvmx::DestroyBackendApi(api);
}

}  // extern "C"
