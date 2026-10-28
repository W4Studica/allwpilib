// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>
#include <memory>
#include <utility>

#include "wpi/hal/Extensions.h"
#include "wpi/halsim/studica/ChannelMap.hpp"
#include "wpi/halsim/studica/HALSimStudica.hpp"
#include "wpi/studica/detail/backend/HardwareBackend.hpp"
#include "wpi/studica/detail/runtime/TerminationHandlers.hpp"

using namespace wpi::studica;

static std::unique_ptr<HALSimStudica> gSim;

extern "C" {
#if defined(WIN32) || defined(_WIN32)
__declspec(dllexport)
#endif

int HALSIM_InitExtension(void) {
  std::puts("HALSim Studica Extension Initializing");

  HAL_OnShutdown(nullptr, [](void*) { gSim.reset(); });

  auto config = Config::ForRuntime();

  if (config.i2cPort >= 0) {
    std::printf(
        "HALSim Studica: I2C port %d mapped to VMX bus (opened on first "
        "transaction)\n",
        config.i2cPort);
  }
  auto backend = CreateBackend();
  if (!backend) {
    return -1;
  }

  // The VMX HAL's SIGINT/SIGTERM handlers do not end the process on their own.
  // They may not exist yet, so the poll thread keeps calling this too.
  InstallTerminationHandlers();

  gSim = std::make_unique<HALSimStudica>(std::move(backend), std::move(config));
  if (!gSim->Start()) {
    gSim.reset();
    return -1;
  }

  std::puts("HALSim Studica Extension Initialized");
  return 0;
}

}  // extern "C"
