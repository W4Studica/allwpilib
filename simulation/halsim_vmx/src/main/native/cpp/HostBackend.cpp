// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Backend for builds without VMX hardware support (WPILIB_WITH_STUDICA=OFF).

#include "wpi/halsim/vmx/LoopbackBackend.hpp"

namespace wpilibvmx {

std::unique_ptr<VmxBackend> CreateBackend() {
  return std::make_unique<LoopbackBackend>();
}

}  // namespace wpilibvmx
