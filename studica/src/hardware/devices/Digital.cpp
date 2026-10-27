// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../StudicaBackend.hpp"
#include "wpi/studica/SharedVMX.hpp"
namespace wpi::studica {

bool StudicaBackend::InitDigital(int channel, bool input) {
  const auto capability = input ? VMXChannelCapability::DigitalInput
                                : VMXChannelCapability::DigitalOutput;
  if (!m_vmx->IsOpen() || ChannelBusy(channel) ||
      !ChannelCan(channel, capability))
    return false;
  DIOConfig config;
  if (input)
    config.SetInputMode(DIOConfig::InputMode::PULLUP);
  VMXResourceHandle handle{};
  VMXErrorCode error = 0;
  if (!m_vmx->io.ActivateSinglechannelResource(
          VMXChannelInfo(channel, capability), &config, handle, &error))
    return false;
  auto resource = detail::OwnVMXResource(m_vmx, handle);
  m_dio.emplace(channel, std::move(resource));
  return true;
}
void StudicaBackend::ReleaseDigital(int channel) {
  m_dio.erase(channel);
}
void StudicaBackend::SetDigital(int channel, bool value) {
  TrySetDigital(channel, value);
}
bool StudicaBackend::GetDigital(int channel) {
  bool value = false;
  TryGetDigital(channel, &value);
  return value;
}
bool StudicaBackend::TrySetDigital(int channel, bool value) {
  auto it = m_dio.find(channel);
  if (it == m_dio.end())
    return false;
  VMXErrorCode error = 0;
  return m_vmx->io.DIO_Set(it->second.Get(), value, &error);
}
bool StudicaBackend::TryGetDigital(int channel, bool* value) {
  auto it = m_dio.find(channel);
  if (!value || it == m_dio.end())
    return false;
  bool result = false;
  VMXErrorCode error = 0;
  if (!m_vmx->io.DIO_Get(it->second.Get(), result, &error))
    return false;
  *value = result;
  return true;
}
}  // namespace wpi::studica
