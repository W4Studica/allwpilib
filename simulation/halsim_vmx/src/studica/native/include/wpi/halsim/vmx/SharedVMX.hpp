// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>

#include "VMXPi.h"

namespace wpilibvmx {

/**
 * The one VMXPi of this process, created on first use.
 *
 * The Studica driver constructors default their last argument to
 * std::make_shared<VMXPi>(true, 50), i.e. a new VMXPi per device. studica_drivers must
 * not be modified, so pass this instead:
 *
 *   studica_driver::Titan titan(42, 20000, 0.0f, wpilibvmx::SharedVMX());
 *
 * The halsim_vmx Studica backend uses the same instance, so devices created by robot
 * code and devices driven by the simulation HAL share one VMXPi. Link against
 * halsim_vmx_studica (the same library the extension loads) to call this.
 */
std::shared_ptr<VMXPi> SharedVMX();

}  // namespace wpilibvmx
