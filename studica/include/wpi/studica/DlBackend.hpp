// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <string>

#include "wpi/studica/HardwareBackend.hpp"

namespace wpi::studica {

/**
 * Loads a backend plugin (see BackendApi.h) with dlopen and adapts it to HardwareBackend.
 * Returns nullptr and fills *error if the library cannot be loaded, lacks the entry
 * points, or was built for a different ABI version.
 */
std::unique_ptr<HardwareBackend> LoadBackendPlugin(const std::string& path,
                                              std::string* error);

}  // namespace wpi::studica
