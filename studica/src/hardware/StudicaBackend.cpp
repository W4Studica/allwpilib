// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StudicaBackend.hpp"

#include "wpi/studica/SharedVMX.hpp"
#include "wpi/studica/detail/backend/BackendPlugin.hpp"

namespace wpi::studica {
StudicaBackend::StudicaBackend() : m_vmx{SharedVMX()} {}

StudicaBackend::~StudicaBackend() {
  while (!m_interrupts.empty()) {
    ReleaseInterrupt(m_interrupts.begin()->first);
  }
  while (!m_dutyCycles.empty()) {
    ReleaseDutyCycle(m_dutyCycles.begin()->first);
  }
  while (!m_pwm.empty()) {
    ReleasePwm(m_pwm.begin()->first);
  }
  ReleaseI2c();
}

bool StudicaBackend::ChannelBusy(int channel) const {
  if (m_dio.contains(channel) || m_analog.contains(channel) ||
      m_encoders.contains(channel) || m_pwm.contains(channel) ||
      m_dutyCycles.contains(channel) || m_interrupts.contains(channel)) {
    return true;
  }
  for (const auto& [a, b] : m_encoderB) {
    if (b == channel) {
      return true;
    }
  }
  return m_i2cActive && (channel == m_sda || channel == m_scl);
}

bool StudicaBackend::ChannelCan(int vmxChannel,
                                VMXChannelCapability capability) const {
  if (vmxChannel < 0 || vmxChannel >= INVALID_VMX_CHANNEL_INDEX) {
    return false;
  }
  return m_vmx->io.ChannelSupportsCapability(
      static_cast<VMXChannelIndex>(vmxChannel), capability);
}

bool StudicaBackend::GetBatteryVoltage(double* volts) {
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
}  // namespace wpi::studica

extern "C" {

WPILIBSTUDICA_PLUGIN_EXPORT wpilibstudica_BackendApi*
wpilibstudica_CreateBackendV4(void) {
  try {
    return wpi::studica::MakeBackendApi(
        std::make_unique<wpi::studica::StudicaBackend>());
  } catch (...) {
    return nullptr;
  }
}

WPILIBSTUDICA_PLUGIN_EXPORT void wpilibstudica_DestroyBackendV4(
    wpilibstudica_BackendApi* api) {
  wpi::studica::DestroyBackendApi(api);
}

}  // extern "C"
