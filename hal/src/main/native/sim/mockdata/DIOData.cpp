// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../PortsInternal.hpp"
#include "DIODataInternal.hpp"

using namespace wpi::hal;

namespace wpi::hal::init {
void InitializeDIOData() {
  static DIOData sdd[NUM_DIGITAL_CHANNELS];
  ::wpi::hal::SimDIOData = sdd;
}
}  // namespace wpi::hal::init

DIOData* wpi::hal::SimDIOData;
void DIOData::ResetData() {
  edgeCaptureEnabled = false;
  edge.Reset();
  initialized.Reset(false);
  simDevice = 0;
  value.Reset(true);
  pulseLength.Reset(0.0);
  isInput.Reset(true);
  filterIndex.Reset(-1);
}

extern "C" {
void HALSIM_SetDIOEdgeCaptureEnabled(int32_t index, HAL_Bool enabled) {
  if (index < 0 || index >= NUM_DIGITAL_CHANNELS) {
    return;
  }
  SimDIOData[index].edgeCaptureEnabled = enabled != 0;
}
HAL_Bool HALSIM_GetDIOEdgeCaptureEnabled(int32_t index) {
  if (index < 0 || index >= NUM_DIGITAL_CHANNELS) {
    return false;
  }
  return SimDIOData[index].edgeCaptureEnabled;
}
int32_t HALSIM_RegisterDIOEdgeCallback(int32_t index,
                                       HAL_DigitalEdgeCallback callback,
                                       void* param) {
  if (index < 0 || index >= NUM_DIGITAL_CHANNELS) {
    return -1;
  }
  return SimDIOData[index].edge.Register(callback, param);
}
void HALSIM_CancelDIOEdgeCallback(int32_t index, int32_t uid) {
  if (index < 0 || index >= NUM_DIGITAL_CHANNELS) {
    return;
  }
  SimDIOData[index].edge.Cancel(uid);
}
void HALSIM_NotifyDIOEdge(int32_t index, HAL_Bool rising,
                          uint64_t timestampMicros, uint32_t dropped) {
  if (index < 0 || index >= NUM_DIGITAL_CHANNELS) {
    return;
  }
  SimDIOData[index].edge(rising, timestampMicros, dropped);
}

void HALSIM_ResetDIOData(int32_t index) {
  SimDIOData[index].ResetData();
}

HAL_SimDeviceHandle HALSIM_GetDIOSimDevice(int32_t index) {
  return SimDIOData[index].simDevice;
}

#define DEFINE_CAPI(TYPE, CAPINAME, LOWERNAME)                          \
  HAL_SIMDATAVALUE_DEFINE_CAPI(TYPE, HALSIM, DIO##CAPINAME, SimDIOData, \
                               LOWERNAME)

DEFINE_CAPI(HAL_Bool, Initialized, initialized)
DEFINE_CAPI(HAL_Bool, Value, value)
DEFINE_CAPI(double, PulseLength, pulseLength)
DEFINE_CAPI(HAL_Bool, IsInput, isInput)
DEFINE_CAPI(int32_t, FilterIndex, filterIndex)

#define REGISTER(NAME) \
  SimDIOData[index].NAME.RegisterCallback(callback, param, initialNotify)

void HALSIM_RegisterDIOAllCallbacks(int32_t index, HAL_NotifyCallback callback,
                                    void* param, HAL_Bool initialNotify) {
  REGISTER(initialized);
  REGISTER(value);
  REGISTER(pulseLength);
  REGISTER(isInput);
  REGISTER(filterIndex);
}
}  // extern "C"
