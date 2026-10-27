// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../StudicaBackend.hpp"
#include "wpi/studica/detail/conversion/DeviceConversions.hpp"

namespace wpi::studica {
bool StudicaBackend::InitDutyCycle(int channel) {
  if (!m_vmx || !m_vmx->IsOpen() || ChannelBusy(channel) ||
      !ChannelCan(channel, VMXChannelCapability::PWMCaptureInput)) {
    return false;
  }
  PWMCaptureConfig config(1, PWMCaptureConfig::x2);
  VMXResourceHandle handle;
  VMXErrorCode err = 0;
  if (!m_vmx->io.ActivateSinglechannelResource(
          VMXChannelInfo(channel, VMXChannelCapability::PWMCaptureInput),
          &config, handle, &err)) {
    return false;
  }
  auto resource = detail::OwnVMXResource(m_vmx, handle);
  m_dutyCycles.emplace(channel, std::move(resource));
  return true;
}

void StudicaBackend::ReleaseDutyCycle(int channel) {
  auto it = m_dutyCycles.find(channel);
  if (it == m_dutyCycles.end()) {
    return;
  }
  m_dutyCycles.erase(it);
}

bool StudicaBackend::GetDutyCycle(int channel, DutyCycleReading* reading) {
  auto it = m_dutyCycles.find(channel);
  if (!reading || it == m_dutyCycles.end()) {
    return false;
  }
  uint32_t period = 0, high = 0;
  VMXErrorCode err = 0;
  if (!m_vmx->io.PWMCapture_GetCount(it->second.Get(), period, high, &err)) {
    return false;
  }
  *reading = ConvertDutyCycle(period, high);
  return true;
}
}  // namespace wpi::studica
