// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Backend plugin for VMX-pi (WPILIB_WITH_STUDICA=ON), built on the device as
// libhalsim_vmx_studica.so and loaded by halsim_vmx through HALSIMVMX_BACKEND.
// Calls the Studica drivers directly. NOT YET COMPILED OR RUN: VMXPi.h and
// vmxpi_hal_cpp are only available on a VMX-pi.

#include <map>
#include <memory>

#include "VMXPi.h"
#include "analog_input.hpp"
#include "dio.hpp"
#include "encoder.hpp"
#include "imu.hpp"
#include "wpi/halsim/vmx/BackendPlugin.hpp"
#include "wpi/halsim/vmx/VmxBackend.hpp"

namespace wpilibvmx {

class StudicaBackend : public VmxBackend {
 public:
  StudicaBackend() : m_vmx{std::make_shared<VMXPi>(true, 50)} {}

  bool InitDigital(int vmxChannel, bool isInput) override {
    if (!m_vmx || !m_vmx->IsOpen() || m_dio.contains(vmxChannel)) {
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
    if (!m_vmx || !m_vmx->IsOpen() || m_analog.contains(vmxChannel)) {
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
    if (!m_vmx || !m_vmx->IsOpen() || m_encoders.contains(vmxChannelA)) {
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

 private:
  std::shared_ptr<VMXPi> m_vmx;
  std::map<int, std::unique_ptr<studica_driver::DIO>> m_dio;
  std::map<int, std::unique_ptr<studica_driver::AnalogInput>> m_analog;
  std::map<int, std::unique_ptr<studica_driver::Encoder>> m_encoders;
  std::unique_ptr<studica_driver::Imu> m_imu;
};

}  // namespace wpilibvmx

extern "C" {

WPILIBVMX_PLUGIN_EXPORT wpilibvmx_BackendApi* wpilibvmx_CreateBackendV1(void) {
  try {
    return wpilibvmx::MakeBackendApi(
        std::make_unique<wpilibvmx::StudicaBackend>());
  } catch (...) {
    return nullptr;
  }
}

WPILIBVMX_PLUGIN_EXPORT void wpilibvmx_DestroyBackendV1(
    wpilibvmx_BackendApi* api) {
  wpilibvmx::DestroyBackendApi(api);
}

}  // extern "C"
