// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>
#include <string>

#include "wpi/studica/detail/backend/DlBackend.hpp"
#include "wpi/studica/detail/backend/HardwareBackend.hpp"
#include "wpi/studica/detail/backend/LoopbackBackend.hpp"

namespace wpi::studica {

bool IsHardwareRuntime() {
#ifdef WPI_STUDICA_HARDWARE
  return true;
#else
  return false;
#endif
}

std::unique_ptr<HardwareBackend> CreateHardwareBackend() {
  // The runtime libraries are deployed together. The VMX build sets $ORIGIN
  // as its search path, so no project-specific absolute path is needed.
  constexpr const char* library = "libstudica_hardware.so";
  std::string error;
  auto backend = LoadBackendPlugin(library, &error);
  if (!backend) {
    std::fprintf(stderr, "HALSim Studica: cannot load hardware backend: %s\n",
                 error.c_str());
    return nullptr;  // A hardware failure must never look like successful simulation.
  }
  std::puts("HALSim Studica: hardware backend loaded");
  return backend;
}

std::unique_ptr<HardwareBackend> CreateBackend() {
  if (IsHardwareRuntime()) {
    return CreateHardwareBackend();
  }
  return std::make_unique<LoopbackBackend>();
}

}  // namespace wpi::studica
