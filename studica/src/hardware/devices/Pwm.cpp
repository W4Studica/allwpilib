// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "../StudicaBackend.hpp"

namespace wpi::studica {
bool StudicaBackend::InitPwm(int vmxChannel, int periodMicros) {
  if (!m_vmx || !m_vmx->IsOpen() || ChannelBusy(vmxChannel) ||
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
  VMXResourceHandle handle{};
  if (!m_vmx->io.ActivateSinglechannelResource(
          VMXChannelInfo(pwm.index, VMXChannelCapability::PWMGeneratorOutput),
          &config, handle, &err)) {
    std::fprintf(stderr, "HALSim Studica: PWM on VMX channel %d: %s\n",
                 vmxChannel, GetVMXErrorString(err));
    return false;
  }
  pwm.resource = detail::OwnVMXResource(m_vmx, handle);
  m_pwm.emplace(vmxChannel, std::move(pwm));
  if (!TrySetPwmPulse(vmxChannel, 0)) {
    m_pwm.erase(vmxChannel);
    return false;
  }
  return true;
}

void StudicaBackend::ReleasePwm(int vmxChannel) {
  auto it = m_pwm.find(vmxChannel);
  if (it == m_pwm.end()) {
    return;
  }
  SetPwmPulse(vmxChannel, 0);
  m_pwm.erase(it);
}

void StudicaBackend::SetPwmPulse(int vmxChannel, int pulseMicros) {
  TrySetPwmPulse(vmxChannel, pulseMicros);
}
bool StudicaBackend::TrySetPwmPulse(int vmxChannel, int pulseMicros) {
  auto it = m_pwm.find(vmxChannel);
  if (it == m_pwm.end()) {
    return false;
  }
  const long ticks =
      std::lround(static_cast<double>(pulseMicros) / kTickMicros);
  const auto duty =
      static_cast<uint16_t>(std::clamp<long>(ticks, 0, it->second.maxDuty));
  VMXErrorCode err = 0;
  if (!m_vmx->io.PWMGenerator_SetDutyCycle(it->second.resource.Get(),
                                           it->second.index, duty, &err)) {
    std::fprintf(stderr, "HALSim Studica: PWM duty on VMX channel %d: %s\n",
                 vmxChannel, GetVMXErrorString(err));
    return false;
  }
  return true;
}
}  // namespace wpi::studica
