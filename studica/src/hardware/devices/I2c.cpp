// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <vector>

#include "../StudicaBackend.hpp"

namespace wpi::studica {
bool StudicaBackend::InitI2c() {
  if (!m_vmx || !m_vmx->IsOpen() || m_i2cActive) {
    return false;
  }
  m_sda = m_vmx->io.GetSoleChannelIndex(VMXChannelCapability::I2C_SDA);
  m_scl = m_vmx->io.GetSoleChannelIndex(VMXChannelCapability::I2C_SCL);
  if (!ChannelCan(m_sda, VMXChannelCapability::I2C_SDA) ||
      !ChannelCan(m_scl, VMXChannelCapability::I2C_SCL) || ChannelBusy(m_sda) ||
      ChannelBusy(m_scl)) {
    return false;
  }
  I2CConfig config;
  VMXErrorCode err = 0;
  VMXResourceHandle handle{};
  m_i2cActive = m_vmx->io.ActivateDualchannelResource(
      VMXChannelInfo(m_sda, VMXChannelCapability::I2C_SDA),
      VMXChannelInfo(m_scl, VMXChannelCapability::I2C_SCL), &config, handle,
      &err);
  if (m_i2cActive)
    m_i2cResource = detail::OwnVMXResource(m_vmx, handle);
  return m_i2cActive;
}

void StudicaBackend::ReleaseI2c() {
  if (!m_i2cActive) {
    return;
  }
  m_i2cResource.Reset();
  m_i2cActive = false;
}

bool StudicaBackend::TransactionI2c(int address, const uint8_t* tx, int txSize,
                                    uint8_t* rx, int rxSize) {
  if (!m_i2cActive || address < 0 || address > 0x7f || txSize < 0 ||
      rxSize < 0 || txSize > UINT16_MAX || rxSize > UINT16_MAX ||
      (txSize && !tx) || (rxSize && !rx)) {
    return false;
  }
  // SDK takes a mutable transmit buffer. Never expose the caller's const
  // memory.
  std::vector<uint8_t> buffer;
  if (txSize) {
    buffer.assign(tx, tx + txSize);
  }
  VMXErrorCode err = 0;
  return m_vmx->io.I2C_Transaction(
      m_i2cResource.Get(), static_cast<uint8_t>(address), buffer.data(),
      static_cast<uint16_t>(txSize), rx, static_cast<uint16_t>(rxSize), &err);
}
}  // namespace wpi::studica
