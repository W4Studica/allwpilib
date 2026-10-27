// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>

#include "wpi/hal/Types.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Capture is opt-in and applies only to initialized digital inputs. Capturing
/// owns the physical input until disabled. Events keep the hardware microsecond
/// clock; it must not be compared to HAL monotonic or Unix timestamps.
void HALSIM_SetDIOEdgeCaptureEnabled(int32_t index, HAL_Bool enabled);
HAL_Bool HALSIM_GetDIOEdgeCaptureEnabled(int32_t index);
typedef void (*HAL_DigitalEdgeCallback)(const char* name, void* param,
                                        HAL_Bool rising,
                                        uint64_t timestampMicros,
                                        uint32_t dropped);
int32_t HALSIM_RegisterDIOEdgeCallback(int32_t index,
                                       HAL_DigitalEdgeCallback callback,
                                       void* param);
void HALSIM_CancelDIOEdgeCallback(int32_t index, int32_t uid);
/// Called by a hardware bridge; subscribers run on the bridge's polling thread.
void HALSIM_NotifyDIOEdge(int32_t index, HAL_Bool rising,
                          uint64_t timestampMicros, uint32_t dropped);

#ifdef __cplusplus
}
#endif
