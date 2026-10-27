// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/detail/DlBackend.hpp"

#include <cstdint>
#include <cstdlib>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "wpi/studica/detail/HardwareBackend.hpp"

using namespace wpi::studica;

extern "C" int HALSIM_InitExtension(void);

#ifndef _WIN32

TEST_CASE("Plugin backend works through the C ABI",
          "[halsim_studica][plugin]") {
  std::string error;
  auto backend = LoadBackendPlugin(HALSIMSTUDICA_TEST_PLUGIN, &error);
  INFO(error);
  REQUIRE(backend != nullptr);

  // Digital: claimed once, value round trip, released and claimable again.
  REQUIRE(backend->InitDigital(5, false));
  REQUIRE_FALSE(backend->InitDigital(5, false));
  backend->SetDigital(5, true);
  REQUIRE(backend->GetDigital(5) == true);
  backend->SetDigital(5, false);
  REQUIRE(backend->GetDigital(5) == false);
  backend->ReleaseDigital(5);
  REQUIRE(backend->InitDigital(5, true));

  // Analog.
  REQUIRE(backend->InitAnalog(7));
  double volts = -1.0;
  REQUIRE(backend->GetAnalogVoltage(7, &volts));
  REQUIRE(volts == 0.0);
  REQUIRE_FALSE(backend->GetAnalogVoltage(8, &volts));  // not claimed

  // Encoder: a channel used by the digital pin above cannot be reused.
  REQUIRE_FALSE(backend->InitEncoder(5, 6));
  REQUIRE(backend->InitEncoder(10, 11));
  int32_t count = -1;
  REQUIRE(backend->GetEncoderCount(10, &count));
  REQUIRE(count == 0);

  // IMU.
  ImuSample sample;
  REQUIRE_FALSE(backend->GetImu(&sample));  // not claimed yet
  REQUIRE(backend->InitImu());
  REQUIRE(backend->GetImu(&sample));
  REQUIRE(sample.yawDeg == 0.0);

  // PWM: claimed once, a channel used by a digital pin cannot be reused,
  // released and claimable again.
  REQUIRE(backend->InitPwm(15, 20000));
  REQUIRE_FALSE(backend->InitPwm(15, 20000));
  REQUIRE_FALSE(backend->InitPwm(5, 20000));
  backend->SetPwmPulse(15, 1500);
  backend->ReleasePwm(15);
  REQUIRE(backend->InitPwm(15, 5000));

  // Battery: the loopback has no reading.
  double battery = -1.0;
  REQUIRE_FALSE(backend->GetBatteryVoltage(&battery));
}

TEST_CASE("Loading a missing library reports a dlopen error",
          "[halsim_studica][plugin]") {
  std::string error;
  REQUIRE(LoadBackendPlugin("/nonexistent/libnothing.so", &error) == nullptr);
  REQUIRE(error.find("dlopen failed") != std::string::npos);
}

TEST_CASE("A library that is not a backend plugin is rejected",
          "[halsim_studica][plugin]") {
  std::string error;
  REQUIRE(LoadBackendPlugin(HALSIMSTUDICA_TEST_NOT_A_PLUGIN, &error) ==
          nullptr);
  REQUIRE(error.find("missing") != std::string::npos);
}

TEST_CASE("A plugin with a different ABI version is rejected",
          "[halsim_studica][plugin]") {
  std::string error;
  REQUIRE(LoadBackendPlugin(HALSIMSTUDICA_TEST_BAD_ABI_PLUGIN, &error) ==
          nullptr);
  REQUIRE(error.find("ABI mismatch") != std::string::npos);
}

TEST_CASE("CreateBackend uses loopback only when no backend is named",
          "[halsim_studica][plugin]") {
  unsetenv("HALSIMSTUDICA_BACKEND");
  REQUIRE(CreateBackend() != nullptr);

  setenv("HALSIMSTUDICA_BACKEND", HALSIMSTUDICA_TEST_PLUGIN, 1);
  auto fromPlugin = CreateBackend();
  REQUIRE(fromPlugin != nullptr);

  // A named backend that cannot be loaded must NOT fall back to loopback.
  setenv("HALSIMSTUDICA_BACKEND", "/nonexistent/libnothing.so", 1);
  REQUIRE(CreateBackend() == nullptr);

  unsetenv("HALSIMSTUDICA_BACKEND");
}

TEST_CASE("The extension refuses to initialize when its backend cannot load",
          "[halsim_studica][plugin]") {
  setenv("HALSIMSTUDICA_BACKEND", "/nonexistent/libnothing.so", 1);
  REQUIRE(HALSIM_InitExtension() == -1);
  unsetenv("HALSIMSTUDICA_BACKEND");
}

#endif

#ifndef _WIN32
TEST_CASE("ABI v3 forwards capture and I2C lifecycle", "[studica][plugin]") {
  std::string error;
  auto backend = LoadBackendPlugin(HALSIMSTUDICA_TEST_PLUGIN, &error);
  REQUIRE(backend != nullptr);
  REQUIRE(backend->InitI2c());
  REQUIRE_FALSE(backend->TransactionI2c(0x50, nullptr, 0, nullptr, 0));
  backend->ReleaseI2c();
  REQUIRE(backend->InitI2c());
  backend->ReleaseI2c();
  REQUIRE(backend->InitDutyCycle(12));
  DutyCycleReading reading;
  REQUIRE(backend->GetDutyCycle(12, &reading));
  REQUIRE(reading.frequencyHz == 0);
  REQUIRE_FALSE(backend->InitDigital(12, true));
  backend->ReleaseDutyCycle(12);
  REQUIRE(backend->InitInterrupt(12));
  InterruptEvent event;
  REQUIRE_FALSE(backend->ReadInterrupt(12, &event));
  backend->ReleaseInterrupt(12);
  REQUIRE(backend->InitDigital(12, true));
}
#endif
