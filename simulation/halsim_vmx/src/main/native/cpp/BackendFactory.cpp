// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>
#include <cstdlib>
#include <string>

#include "wpi/halsim/vmx/DlBackend.hpp"
#include "wpi/halsim/vmx/LoopbackBackend.hpp"
#include "wpi/halsim/vmx/VmxBackend.hpp"

namespace wpilibvmx {

std::unique_ptr<VmxBackend> CreateBackend() {
  const char* path = std::getenv("HALSIMVMX_BACKEND");
  if (path == nullptr || *path == '\0') {
    std::puts(
        "HALSim VMX: HALSIMVMX_BACKEND is not set; using the loopback backend "
        "(no VMX hardware is driven)");
    return std::make_unique<LoopbackBackend>();
  }

  // A backend was asked for explicitly. Never fall back to loopback on failure:
  // the robot code would appear to work while no hardware is driven.
  std::string error;
  auto backend = LoadBackendPlugin(path, &error);
  if (!backend) {
    std::fprintf(stderr, "HALSim VMX: cannot load backend '%s': %s\n", path,
                 error.c_str());
    return nullptr;
  }
  std::printf("HALSim VMX: using backend %s\n", path);
  return backend;
}

}  // namespace wpilibvmx
