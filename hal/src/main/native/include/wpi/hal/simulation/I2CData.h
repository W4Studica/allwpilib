// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "wpi/hal/Types.h"
#include "wpi/hal/simulation/NotifyListener.h"

#ifdef __cplusplus
extern "C" {
#endif

void HALSIM_ResetI2CData(int32_t index);

/// Returns 0 on success, -1 on a bus error. Buffers are valid only during the
/// call.
typedef int32_t (*HAL_I2CTransactionCallback)(void* param, int32_t address,
                                              const uint8_t* tx, int32_t txSize,
                                              uint8_t* rx, int32_t rxSize);
/// One provider per port. Returns a positive uid, or -1 if already
/// owned/invalid.
int32_t HALSIM_RegisterI2CTransactionCallback(
    int32_t index, HAL_I2CTransactionCallback callback, void* param);
/// Waits for an in-flight transaction before releasing its provider.
void HALSIM_CancelI2CTransactionCallback(int32_t index, int32_t uid);

int32_t HALSIM_RegisterI2CInitializedCallback(int32_t index,
                                              HAL_NotifyCallback callback,
                                              void* param,
                                              HAL_Bool initialNotify);
void HALSIM_CancelI2CInitializedCallback(int32_t index, int32_t uid);
HAL_Bool HALSIM_GetI2CInitialized(int32_t index);
void HALSIM_SetI2CInitialized(int32_t index, HAL_Bool initialized);

int32_t HALSIM_RegisterI2CReadCallback(int32_t index,
                                       HAL_BufferCallback callback,
                                       void* param);
void HALSIM_CancelI2CReadCallback(int32_t index, int32_t uid);

int32_t HALSIM_RegisterI2CWriteCallback(int32_t index,
                                        HAL_ConstBufferCallback callback,
                                        void* param);
void HALSIM_CancelI2CWriteCallback(int32_t index, int32_t uid);

#ifdef __cplusplus
}  // extern "C"
#endif
