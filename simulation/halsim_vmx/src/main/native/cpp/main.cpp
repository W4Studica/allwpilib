// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>
#include <memory>
#include <utility>

#include "wpi/hal/Extensions.h"
#include "wpi/halsim/vmx/ChannelMap.hpp"
#include "wpi/halsim/vmx/HALSimVMX.hpp"
#include "wpi/halsim/vmx/VmxBackend.hpp"

using namespace wpilibvmx;

static std::unique_ptr<HALSimVMX> gSim;

extern "C" {
#if defined(WIN32) || defined(_WIN32)
__declspec(dllexport)
#endif

int HALSIM_InitExtension(void) {
  std::puts("HALSim VMX Extension Initializing");

  HAL_OnShutdown(nullptr, [](void*) { gSim.reset(); });

  auto dioMap = ChannelMap::FromEnv("HALSIMVMX_DIO_MAP");
  if (dioMap.size() == 0) {
    std::puts(
        "HALSim VMX: HALSIMVMX_DIO_MAP is not set; no DIO channels mapped");
  }

  gSim = std::make_unique<HALSimVMX>(CreateBackend(), std::move(dioMap));
  gSim->Start();

  std::puts("HALSim VMX Extension Initialized");
  return 0;
}

}  // extern "C"
