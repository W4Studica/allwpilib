// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <type_traits>
#include <utility>

#include "wpi/studica/SharedVMX.hpp"

namespace wpi::studica {
/// Creates an original driver while injecting this process's shared VMX.
/// Include the corresponding wpi/studica/drivers header first. Pass all device
/// arguments before the VMX argument, including optional bounds for PWM/Servo.
/// Example: MakeDriver<drivers::DIO>(0, drivers::PinMode::INPUT).
template <class Driver, class... Args>
std::unique_ptr<Driver> MakeDriver(Args&&... args) {
  if constexpr (std::is_constructible_v<Driver, Args...,
                                        std::shared_ptr<VMXPi>>) {
    return std::make_unique<Driver>(std::forward<Args>(args)..., SharedVMX());
  } else {
    static_assert(
        std::is_constructible_v<Driver, std::shared_ptr<VMXPi>, Args...>,
        "Pass all device constructor arguments; VMX is injected automatically");
    return std::make_unique<Driver>(SharedVMX(), std::forward<Args>(args)...);
  }
}
}  // namespace wpi::studica
