// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <limits>

#include "../PortsInternal.hpp"
#include "I2CDataInternal.hpp"

using namespace wpi::hal;

namespace wpi::hal::init {
void InitializeI2CData() {
  static I2CData sid[I2C_PORTS];
  ::wpi::hal::SimI2CData = sid;
}
}  // namespace wpi::hal::init

I2CData* wpi::hal::SimI2CData;

void I2CData::ResetData() {
  {
    std::scoped_lock lock{transactionMutex};
    transaction = nullptr;
    transactionParam = nullptr;
    transactionUid = 0;
  }
  initialized.Reset(false);
  read.Reset();
  write.Reset();
}

void I2CData::Write(int32_t deviceAddress, const uint8_t* dataToSend,
                    int32_t sendSize) {
  write(dataToSend, sendSize);
}

void I2CData::Read(int32_t deviceAddress, uint8_t* buffer, int32_t count) {
  read(buffer, count);
}

extern "C" {
int32_t HALSIM_RegisterI2CTransactionCallback(
    int32_t index, HAL_I2CTransactionCallback callback, void* param) {
  if (index < 0 || index >= I2C_PORTS || !callback) {
    return -1;
  }
  auto& data = SimI2CData[index];
  std::scoped_lock lock{data.transactionMutex};
  if (data.transaction ||
      data.nextTransactionUid == std::numeric_limits<int32_t>::max()) {
    return -1;
  }
  data.transaction = callback;
  data.transactionParam = param;
  data.transactionUid = data.nextTransactionUid++;
  return data.transactionUid;
}
void HALSIM_CancelI2CTransactionCallback(int32_t index, int32_t uid) {
  if (index < 0 || index >= I2C_PORTS) {
    return;
  }
  auto& data = SimI2CData[index];
  std::scoped_lock lock{data.transactionMutex};
  if (uid > 0 && uid == data.transactionUid) {
    data.transaction = nullptr;
    data.transactionParam = nullptr;
    data.transactionUid = 0;
  }
}

void HALSIM_ResetI2CData(int32_t index) {
  SimI2CData[index].ResetData();
}

#define DEFINE_CAPI(TYPE, CAPINAME, LOWERNAME)                          \
  HAL_SIMDATAVALUE_DEFINE_CAPI(TYPE, HALSIM, I2C##CAPINAME, SimI2CData, \
                               LOWERNAME)

DEFINE_CAPI(HAL_Bool, Initialized, initialized)

#undef DEFINE_CAPI
#define DEFINE_CAPI(TYPE, CAPINAME, LOWERNAME)                                 \
  HAL_SIMCALLBACKREGISTRY_DEFINE_CAPI(TYPE, HALSIM, I2C##CAPINAME, SimI2CData, \
                                      LOWERNAME)

DEFINE_CAPI(HAL_BufferCallback, Read, read)
DEFINE_CAPI(HAL_ConstBufferCallback, Write, write)

}  // extern "C"
