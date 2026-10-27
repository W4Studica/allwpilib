// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../StudicaBackend.hpp"
#include "wpi/studica/SharedVMX.hpp"
namespace wpi::studica {

bool StudicaBackend::InitAnalog(int channel) {
  if (!m_vmx->IsOpen() || ChannelBusy(channel) || m_analog.contains(channel) ||
      !ChannelCan(channel, VMXChannelCapability::AccumulatorInput))
    return false;
  AccumulatorConfig config;
  config.SetNumAverageBits(9);
  VMXResourceHandle handle{};
  VMXErrorCode error = 0;
  if (!m_vmx->io.ActivateSinglechannelResource(
          VMXChannelInfo(channel, VMXChannelCapability::AccumulatorInput),
          &config, handle, &error))
    return false;
  auto resource = detail::OwnVMXResource(m_vmx, handle);
  m_analog.emplace(channel, std::move(resource));
  return true;
}
void StudicaBackend::ReleaseAnalog(int channel) {
  m_analog.erase(channel);
}
bool StudicaBackend::GetAnalogVoltage(int channel, double* volts) {
  auto it = m_analog.find(channel);
  if (!volts || it == m_analog.end())
    return false;
  float value = 0;
  VMXErrorCode error = 0;
  if (!m_vmx->io.Accumulator_GetAverageVoltage(it->second.Get(), value, &error))
    return false;
  *volts = value;
  return true;
}
}  // namespace wpi::studica
