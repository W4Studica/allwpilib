// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <utility>

#include "wpi/hal/Extensions.h"
#include "wpi/halsim/vmx/ChannelMap.hpp"
#include "wpi/halsim/vmx/HALSimVMX.hpp"
#include "wpi/halsim/vmx/VmxBackend.hpp"

using namespace wpilibvmx;

static std::unique_ptr<HALSimVMX> gSim;

// A mistyped map must not look like an unset one: say what was ignored.
static void ReportMap(const char* envName, const char* kind,
                      const ChannelMap& map) {
  const char* value = std::getenv(envName);
  const bool set = value != nullptr && *value != '\0';
  if (!set) {
    std::printf("HALSim VMX: %s is not set; no %s channels mapped\n", envName,
                kind);
  } else if (map.size() == 0) {
    std::fprintf(stderr,
                 "HALSim VMX: %s is set to '%s' but has no valid entry "
                 "(format: <wpilib>:<vmx>,<wpilib>:<vmx>, no spaces); no %s "
                 "channels mapped\n",
                 envName, value, kind);
  }
  for (const auto& entry : map.Rejected()) {
    std::fprintf(stderr, "HALSim VMX: %s: ignoring '%s'\n", envName,
                 entry.c_str());
  }
  for (int vmx : map.DuplicateVmxChannels()) {
    std::fprintf(stderr,
                 "HALSim VMX: %s maps more than one channel to VMX channel %d; "
                 "only one of them can claim it\n",
                 envName, vmx);
  }
}

extern "C" {
#if defined(WIN32) || defined(_WIN32)
__declspec(dllexport)
#endif

int HALSIM_InitExtension(void) {
  std::puts("HALSim VMX Extension Initializing");

  HAL_OnShutdown(nullptr, [](void*) { gSim.reset(); });

  auto config = Config::FromEnv();
  ReportMap("HALSIMVMX_DIO_MAP", "DIO", config.maps.dio);
  ReportMap("HALSIMVMX_ANALOG_MAP", "analog", config.maps.analog);

  auto backend = CreateBackend();
  if (!backend) {
    return -1;
  }

  gSim = std::make_unique<HALSimVMX>(std::move(backend), std::move(config));
  gSim->Start();

  std::puts("HALSim VMX Extension Initialized");
  return 0;
}

}  // extern "C"
