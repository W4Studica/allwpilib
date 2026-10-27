// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/detail/backend/BackendPlugin.hpp"

#include <stdexcept>

#include <catch2/catch_test_macros.hpp>

#include "wpi/studica/detail/backend/LoopbackBackend.hpp"

using namespace wpi::studica;
TEST_CASE("Checked plugin IO preserves LOW and reports missing channels",
          "[studica][plugin]") {
  std::unique_ptr<wpilibstudica_BackendApi, decltype(&DestroyBackendApi)> api{
      MakeBackendApi(std::make_unique<LoopbackBackend>()), DestroyBackendApi};
  int value = 123;
  REQUIRE(api->tryGetDigital(api->ctx, 2, &value) == 0);
  REQUIRE(value == 123);
  REQUIRE(api->initDigital(api->ctx, 2, 1) == 1);
  REQUIRE(api->tryGetDigital(api->ctx, 2, &value) == 1);
  REQUIRE(value == 0);
  REQUIRE(api->trySetDigital(api->ctx, 2, 1) == 0);
  REQUIRE(api->trySetPwmPulse(api->ctx, 7, 1000) == 0);
}
TEST_CASE("Backend exceptions cannot become valid digital readings",
          "[studica][plugin]") {
  struct Failing : LoopbackBackend {
    bool TryGetDigital(int, bool*) override {
      throw std::runtime_error("read failure");
    }
  };
  std::unique_ptr<wpilibstudica_BackendApi, decltype(&DestroyBackendApi)> api{
      MakeBackendApi(std::make_unique<Failing>()), DestroyBackendApi};
  int value = 123;
  REQUIRE(api->tryGetDigital(api->ctx, 2, &value) == 0);
  REQUIRE(value == 123);
}

TEST_CASE("Plugin ABI rejects null output pointers and context",
          "[studica][plugin]") {
  std::unique_ptr<wpilibstudica_BackendApi, decltype(&DestroyBackendApi)> api{
      MakeBackendApi(std::make_unique<LoopbackBackend>()), DestroyBackendApi};
  REQUIRE(api->initAnalog(api->ctx, 22));
  REQUIRE(api->initEncoder(api->ctx, 0, 1));
  REQUIRE(api->initDutyCycle(api->ctx, 2));
  REQUIRE(api->initImu(api->ctx));
  CHECK(api->getAnalogVoltage(api->ctx, 22, nullptr) == 0);
  CHECK(api->getEncoderCount(api->ctx, 0, nullptr) == 0);
  CHECK(api->getImu(api->ctx, nullptr) == 0);
  double output = 99;
  CHECK(api->getDutyCycle(api->ctx, 2, nullptr, &output) == 0);
  CHECK(api->getDutyCycle(api->ctx, 2, &output, nullptr) == 0);
  CHECK(output == 99);
  CHECK(api->getBatteryVoltage(api->ctx, nullptr) == 0);
  CHECK(api->readInterrupt(api->ctx, 2, nullptr, nullptr, nullptr) == 0);
  CHECK(api->tryGetDigital(nullptr, 2, nullptr) == 0);
  CHECK(api->initDigital(nullptr, 2, 1) == 0);
}
