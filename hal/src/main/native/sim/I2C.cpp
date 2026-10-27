// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/hal/I2C.h"

#include "HALInitializer.hpp"
#include "PortsInternal.hpp"
#include "mockdata/I2CDataInternal.hpp"
#include "wpi/hal/Errors.h"

using namespace wpi::hal;

namespace wpi::hal::init {
void InitializeI2C() {}
}  // namespace wpi::hal::init

namespace {
int32_t Transfer(HAL_I2CPort port, int32_t address, const uint8_t* tx,
                 int32_t txSize, uint8_t* rx, int32_t rxSize,
                 int legacyCallbacks) {
  if (port < 0 || port >= I2C_PORTS || address < 0 || address > 0x7f ||
      txSize < 0 || rxSize < 0 || (txSize > 0 && !tx) || (rxSize > 0 && !rx)) {
    return -1;
  }
  auto& data = SimI2CData[port];
  std::scoped_lock lock{data.transactionMutex};
  if (data.transaction) {
    if (!data.initialized) {
      return -1;
    }
    return data.transaction(data.transactionParam, address, tx, txSize, rx,
                            rxSize);
  }
  // Preserve existing simulation callbacks, including zero-length transfers.
  if (legacyCallbacks & 1) {
    data.Write(address, tx, txSize);
  }
  if (legacyCallbacks & 2) {
    data.Read(address, rx, rxSize);
  }
  return 0;
}
}  // namespace

extern "C" {
void HAL_InitializeI2C(HAL_I2CPort port, int32_t* status) {
  wpi::hal::init::CheckInit();
  if (port < 0 || port >= I2C_PORTS) {
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return;
  }
  std::scoped_lock lock{SimI2CData[port].transactionMutex};
  SimI2CData[port].initialized = true;
}
int32_t HAL_TransactionI2C(HAL_I2CPort port, int32_t deviceAddress,
                           const uint8_t* dataToSend, int32_t sendSize,
                           uint8_t* dataReceived, int32_t receiveSize) {
  return Transfer(port, deviceAddress, dataToSend, sendSize, dataReceived,
                  receiveSize, 3);
}
int32_t HAL_WriteI2C(HAL_I2CPort port, int32_t deviceAddress,
                     const uint8_t* dataToSend, int32_t sendSize) {
  return Transfer(port, deviceAddress, dataToSend, sendSize, nullptr, 0, 1);
}
int32_t HAL_ReadI2C(HAL_I2CPort port, int32_t deviceAddress, uint8_t* buffer,
                    int32_t count) {
  return Transfer(port, deviceAddress, nullptr, 0, buffer, count, 2);
}
void HAL_CloseI2C(HAL_I2CPort port) {
  if (port < 0 || port >= I2C_PORTS) {
    return;
  }
  std::scoped_lock lock{SimI2CData[port].transactionMutex};
  SimI2CData[port].initialized = false;
}
}  // extern "C"
