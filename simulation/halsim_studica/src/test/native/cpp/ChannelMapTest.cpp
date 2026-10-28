// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/studica/ChannelMap.hpp"

#include <catch2/catch_test_macros.hpp>

using namespace wpi::studica;

TEST_CASE("VMX board wiring needs no runtime configuration", "[studica][wiring]") {
  const auto config = Config::ForVmx();
  REQUIRE(config.imu);
  REQUIRE(config.i2cPort == 1);
  for (int channel = 0; channel < 22; ++channel) {
    REQUIRE(config.maps.dio.Get(channel) == channel);
    REQUIRE(config.maps.pwm.Get(channel) == channel);
  }
  for (int channel = 0; channel < 4; ++channel) {
    REQUIRE(config.maps.analog.Get(channel) == 22 + channel);
  }
  REQUIRE_FALSE(config.maps.dio.Get(-1));
  REQUIRE_FALSE(config.maps.dio.Get(22));
  REQUIRE_FALSE(config.maps.pwm.Get(22));
  REQUIRE_FALSE(config.maps.analog.Get(4));
}

TEST_CASE("Desktop defaults leave simulated devices in control", "[studica][wiring]") {
  const auto config = Config::ForRuntime();
  REQUIRE_FALSE(config.imu);
  REQUIRE(config.i2cPort == -1);
  REQUIRE(config.maps.dio.size() == 0);
  REQUIRE(config.maps.analog.size() == 0);
  REQUIRE(config.maps.pwm.size() == 0);
}
