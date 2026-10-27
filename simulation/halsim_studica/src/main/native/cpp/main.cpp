// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <utility>

#include "wpi/hal/Extensions.h"
#include "wpi/halsim/studica/ChannelMap.hpp"
#include "wpi/halsim/studica/HALSimStudica.hpp"
#include "wpi/studica/detail/backend/HardwareBackend.hpp"
#include "wpi/studica/detail/runtime/TerminationHandlers.hpp"

using namespace wpi::studica;

static std::unique_ptr<HALSimStudica> gSim;

// A mistyped map must not look like an unset one: say what was ignored.
static void ReportMap(const char* envName, const char* kind,
                      const ChannelMap& map) {
  const char* value = std::getenv(envName);
  const bool set = value != nullptr && *value != '\0';
  if (!set) {
    std::printf("HALSim Studica: %s is not set; no %s channels mapped\n",
                envName, kind);
  } else if (map.size() == 0) {
    std::fprintf(stderr,
                 "HALSim Studica: %s is set to '%s' but has no valid entry "
                 "(format: <wpilib>:<vmx>,<wpilib>:<vmx>, no spaces); no %s "
                 "channels mapped\n",
                 envName, value, kind);
  }
  for (const auto& entry : map.Rejected()) {
    std::fprintf(stderr, "HALSim Studica: %s: ignoring '%s'\n", envName,
                 entry.c_str());
  }
  for (int vmx : map.DuplicateVmxChannels()) {
    std::fprintf(
        stderr,
        "HALSim Studica: %s maps more than one channel to VMX channel %d; "
        "only one of them can claim it\n",
        envName, vmx);
  }
}

extern "C" {
#if defined(WIN32) || defined(_WIN32)
__declspec(dllexport)
#endif

int HALSIM_InitExtension(void) {
  std::puts("HALSim Studica Extension Initializing");

  HAL_OnShutdown(nullptr, [](void*) { gSim.reset(); });

  auto config = Config::FromEnv();
  ReportMap("HALSIMSTUDICA_DIO_MAP", "DIO", config.maps.dio);
  ReportMap("HALSIMSTUDICA_ANALOG_MAP", "analog", config.maps.analog);
  ReportMap("HALSIMSTUDICA_PWM_MAP", "PWM", config.maps.pwm);

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
