// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>

#include "wpi/hal/simulation/I2CData.h"
#include "wpi/halsim/studica/HALSimStudica.hpp"

using namespace wpi::studica;

int32_t HALSimStudica::OnI2cTransaction(void* param, int32_t address,
                                        const uint8_t* tx, int32_t txSize,
                                        uint8_t* rx, int32_t rxSize) {
  auto& self = *static_cast<HALSimStudica*>(param);
  std::scoped_lock lock{self.m_mutex};
  if (!self.m_running) {
    return -1;
  }
  if (!self.m_i2cApplied) {
    self.m_i2cApplied = self.m_backend->InitI2c();
  }
  if (!self.m_i2cApplied) {
    return -1;
  }
  return self.m_backend->TransactionI2c(address, tx, txSize, rx, rxSize) ? 0
                                                                         : -1;
}

void HALSimStudica::PollI2c() {
  if (m_config.i2cPort < 0) {
    return;
  }
  const bool initialized = HALSIM_GetI2CInitialized(m_config.i2cPort);
  std::scoped_lock lock{m_mutex};
  if (!initialized && m_i2cApplied) {
    m_backend->ReleaseI2c();
    m_i2cApplied = false;
  }
}
