// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../StudicaBackend.hpp"
#include "wpi/studica/SharedVMX.hpp"
namespace wpi::studica {

bool StudicaBackend::InitEncoder(int a, int b) {
  if (!m_vmx->IsOpen() || a == b || ChannelBusy(a) || ChannelBusy(b) ||
      !ChannelCan(a, VMXChannelCapability::EncoderAInput) ||
      !ChannelCan(b, VMXChannelCapability::EncoderBInput))
    return false;
  EncoderConfig config(EncoderConfig::EncoderEdge::x4);
  VMXResourceHandle handle{};
  VMXErrorCode error = 0;
  if (!m_vmx->io.ActivateDualchannelResource(
          VMXChannelInfo(a, VMXChannelCapability::EncoderAInput),
          VMXChannelInfo(b, VMXChannelCapability::EncoderBInput), &config,
          handle, &error))
    return false;
  auto resource = detail::OwnVMXResource(m_vmx, handle);
  m_encoderB.emplace(a, b);
  try {
    m_encoders.emplace(a, std::move(resource));
  } catch (...) {
    m_encoderB.erase(a);
    throw;
  }
  return true;
}
void StudicaBackend::ReleaseEncoder(int channel) {
  m_encoders.erase(channel);
  m_encoderB.erase(channel);
}
bool StudicaBackend::GetEncoderCount(int channel, int32_t* count) {
  auto it = m_encoders.find(channel);
  if (!count || it == m_encoders.end())
    return false;
  int32_t value = 0;
  VMXErrorCode error = 0;
  if (!m_vmx->io.Encoder_GetCount(it->second.Get(), value, &error))
    return false;
  *count = value;
  return true;
}
}  // namespace wpi::studica
