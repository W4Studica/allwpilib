// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <string_view>

#include <catch2/catch_session.hpp>

#include "wpi/hal/HAL.h"

int main(int argc, char** argv) {
  bool listOnly = false;
  for (int i = 1; i < argc; ++i) {
    std::string_view arg{argv[i]};
    if (arg == "--list-tests" || arg == "--list-tags" ||
        arg == "--list-reporters" || arg == "--list-listeners") {
      listOnly = true;
    }
  }
  if (!listOnly) {
    HAL_Initialize();
  }
  return Catch::Session().run(argc, argv);
}
