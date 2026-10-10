// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/vmx/DlBackend.hpp"

#include <cstdint>
#include <cstdlib>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "wpi/halsim/vmx/VmxBackend.hpp"

using namespace wpilibvmx;

extern "C" int HALSIM_InitExtension(void);

#ifndef _WIN32

TEST_CASE("Plugin backend works through the C ABI", "[halsim_vmx][plugin]") {
  std::string error;
  auto backend = LoadBackendPlugin(HALSIMVMX_TEST_PLUGIN, &error);
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
}

TEST_CASE("Loading a missing library reports a dlopen error",
          "[halsim_vmx][plugin]") {
  std::string error;
  REQUIRE(LoadBackendPlugin("/nonexistent/libnothing.so", &error) == nullptr);
  REQUIRE(error.find("dlopen failed") != std::string::npos);
}

TEST_CASE("A library that is not a backend plugin is rejected",
          "[halsim_vmx][plugin]") {
  std::string error;
  REQUIRE(LoadBackendPlugin(HALSIMVMX_TEST_NOT_A_PLUGIN, &error) == nullptr);
  REQUIRE(error.find("missing") != std::string::npos);
}

TEST_CASE("A plugin with a different ABI version is rejected",
          "[halsim_vmx][plugin]") {
  std::string error;
  REQUIRE(LoadBackendPlugin(HALSIMVMX_TEST_BAD_ABI_PLUGIN, &error) == nullptr);
  REQUIRE(error.find("ABI mismatch") != std::string::npos);
}

TEST_CASE("CreateBackend uses loopback only when no backend is named",
          "[halsim_vmx][plugin]") {
  unsetenv("HALSIMVMX_BACKEND");
  REQUIRE(CreateBackend() != nullptr);

  setenv("HALSIMVMX_BACKEND", HALSIMVMX_TEST_PLUGIN, 1);
  auto fromPlugin = CreateBackend();
  REQUIRE(fromPlugin != nullptr);

  // A named backend that cannot be loaded must NOT fall back to loopback.
  setenv("HALSIMVMX_BACKEND", "/nonexistent/libnothing.so", 1);
  REQUIRE(CreateBackend() == nullptr);

  unsetenv("HALSIMVMX_BACKEND");
}

TEST_CASE("The extension refuses to initialize when its backend cannot load",
          "[halsim_vmx][plugin]") {
  setenv("HALSIMVMX_BACKEND", "/nonexistent/libnothing.so", 1);
  REQUIRE(HALSIM_InitExtension() == -1);
  unsetenv("HALSIMVMX_BACKEND");
}

#endif
