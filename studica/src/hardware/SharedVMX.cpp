// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/SharedVMX.hpp"

namespace wpi::studica {
std::shared_ptr<VMXPi> SharedVMX() {
  // Thread-safe lazy initialization. Never reset: devices may outlive any one
  // backend.
  static std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50);
  return vmx;
}

}  // namespace wpi::studica
