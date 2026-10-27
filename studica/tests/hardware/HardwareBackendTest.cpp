// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <atomic>
#include <limits>
#include <thread>

#include <catch2/catch_test_macros.hpp>

#include "StudicaBackend.hpp"
#include "wpi/studica/MakeDriver.hpp"
#include "wpi/studica/RtcClock.hpp"
#include "wpi/studica/SharedVMX.hpp"
#include "wpi/studica/detail/backend/BackendPlugin.hpp"

using namespace wpi::studica;
namespace {
struct Fixture {
  std::shared_ptr<VMXPi> vmx = SharedVMX();
  std::unique_ptr<StudicaBackend> backend;
  Fixture() {
    REQUIRE(vmx->io.resources.empty());
    vmx->io = {};
    vmx->power = {};
    vmx->time = {};
    vmx->open = vmx->connected = true;
    vmx->calibrating = false;
    backend = std::make_unique<StudicaBackend>();
  }
  ~Fixture() {
    backend.reset();
    CHECK(vmx->io.resources.empty());
    CHECK(vmx->io.allocated == vmx->io.released);
    CHECK(vmx->io.duplicateReleases == 0);
  }
};
}  // namespace

TEST_CASE("Real device implementations reject failed SDK activation",
          "[studica][hardware]") {
  Fixture f;
  f.vmx->io.failActivate = true;
  CHECK_FALSE(f.backend->InitAnalog(22));
  CHECK_FALSE(f.backend->InitEncoder(0, 1));
  CHECK_FALSE(f.backend->InitDigital(2, true));
  CHECK_FALSE(f.backend->InitPwm(3, 20000));
  CHECK_FALSE(f.backend->InitI2c());
  CHECK_FALSE(f.backend->InitDutyCycle(4));
  CHECK_FALSE(f.backend->InitInterrupt(5));
  CHECK(f.vmx->io.allocated == 0);
  f.vmx->io.failActivate = false;
  CHECK(f.backend->InitAnalog(22));
  CHECK(f.backend->InitEncoder(0, 1));
  CHECK(f.backend->InitDigital(2, true));
  CHECK(f.backend->InitPwm(3, 20000));
  CHECK(f.backend->InitI2c());
  CHECK(f.backend->InitDutyCycle(4));
  CHECK(f.backend->InitInterrupt(5));
}
TEST_CASE("Repeated real Analog creation releases even handle zero",
          "[studica][hardware]") {
  Fixture f;
  for (int i = 0; i < 1000; ++i) {
    REQUIRE(f.backend->InitAnalog(22));
    REQUIRE_FALSE(f.backend->InitAnalog(22));
    f.backend->ReleaseAnalog(22);
    f.backend->ReleaseAnalog(22);
    REQUIRE(f.vmx->io.resources.empty());
  }
  REQUIRE(f.vmx->io.allocated == 1000);
}
TEST_CASE("Real SDK reads preserve caller outputs on failure",
          "[studica][hardware]") {
  Fixture f;
  REQUIRE(f.backend->InitAnalog(22));
  REQUIRE(f.backend->InitEncoder(0, 1));
  REQUIRE(f.backend->InitDigital(2, true));
  REQUIRE(f.backend->InitDutyCycle(3));
  f.vmx->io.failRead = true;
  double voltage = 999;
  int32_t count = 999;
  bool digital = true;
  DutyCycleReading duty{999, 999};
  CHECK_FALSE(f.backend->GetAnalogVoltage(22, &voltage));
  CHECK_FALSE(f.backend->GetEncoderCount(0, &count));
  CHECK_FALSE(f.backend->TryGetDigital(2, &digital));
  CHECK_FALSE(f.backend->GetDutyCycle(3, &duty));
  CHECK(voltage == 999);
  CHECK(count == 999);
  CHECK(digital);
  CHECK(duty.frequencyHz == 999);
  f.vmx->io.failRead = false;
  f.vmx->io.encoderCount = -1;  // A legitimate negative count is not an error.
  CHECK(f.backend->GetEncoderCount(0, &count));
  CHECK(count == -1);
  CHECK(f.backend->TryGetDigital(2, &digital));
  CHECK_FALSE(digital);
}
TEST_CASE("Real hardware ownership rejects overlapping channels",
          "[studica][hardware]") {
  Fixture f;
  REQUIRE(f.backend->InitEncoder(0, 1));
  CHECK_FALSE(f.backend->InitDigital(0, true));
  CHECK_FALSE(f.backend->InitDutyCycle(1));
  f.backend->ReleaseEncoder(0);
  CHECK(f.backend->InitDigital(1, true));
  CHECK_FALSE(f.backend->InitInterrupt(1));
  CHECK(f.backend->InitI2c());
  CHECK_FALSE(f.backend->InitDigital(32, true));
  f.backend->ReleaseI2c();
  CHECK(f.backend->InitDigital(32, true));
}
TEST_CASE("Real PWM unwinds a failed initial output and can retry",
          "[studica][hardware]") {
  Fixture f;
  f.vmx->io.failWrite = true;
  CHECK_FALSE(f.backend->InitPwm(3, 20000));
  CHECK(f.vmx->io.resources.empty());
  f.vmx->io.failWrite = false;
  REQUIRE(f.backend->InitPwm(3, 20000));
  CHECK(f.backend->TrySetPwmPulse(3, 1500));
  CHECK(f.vmx->io.pwmDuty == 375);
  f.vmx->io.failWrite = true;
  CHECK_FALSE(f.backend->TrySetPwmPulse(3, 1600));
  CHECK(f.vmx->io.pwmDuty == 375);
  f.vmx->io.failWrite = false;
  f.backend->ReleasePwm(3);
  CHECK(f.vmx->io.pwmDuty == 0);
}
TEST_CASE("Real I2C never exposes the caller's const transmit buffer",
          "[studica][hardware]") {
  Fixture f;
  REQUIRE(f.backend->InitI2c());
  const uint8_t tx[1]{7};
  uint8_t rx[2]{};
  CHECK(f.backend->TransactionI2c(0x50, tx, 1, rx, 2));
  CHECK(tx[0] == 7);
  CHECK(rx[0] == 0xab);
  CHECK(rx[1] == 0xab);
  CHECK_FALSE(f.backend->TransactionI2c(128, tx, 1, rx, 2));
  CHECK_FALSE(f.backend->TransactionI2c(0x50, nullptr, 1, rx, 2));
  CHECK_FALSE(f.backend->TransactionI2c(0x50, tx, 1, nullptr, 1));
}
TEST_CASE("Late SDK interrupt callbacks cannot access a released backend",
          "[studica][hardware]") {
  Fixture f;
  REQUIRE(f.backend->InitInterrupt(5));
  f.vmx->io.FireLateInterrupt();
  InterruptEvent event;
  REQUIRE(f.backend->ReadInterrupt(5, &event));
  CHECK(event.rising);
  CHECK(event.timestampMicros == 123);
  f.backend->ReleaseInterrupt(5);
  f.vmx->io.FireLateInterrupt();
  CHECK_FALSE(f.backend->ReadInterrupt(5, &event));
  REQUIRE(f.backend->InitInterrupt(5));
  f.backend.reset();
  f.vmx->io.FireLateInterrupt();
}
TEST_CASE("Real RTC handles rollover and keeps failed output unchanged",
          "[studica][hardware]") {
  Fixture f;
  RtcReading reading;
  reading.day = 99;
  f.vmx->time.dates = {28, 29, 29, 29};
  REQUIRE(ReadRtc(&reading));
  REQUIRE(reading.day == 29);
  REQUIRE(f.vmx->time.dateReads == 4);
  f.vmx->time.fail = true;
  CHECK_FALSE(ReadRtc(&reading));
  CHECK(reading.day == 29);
  CHECK_FALSE(ReadRtc(nullptr));
}
TEST_CASE("Hardware rejects bad channels before SDK narrowing",
          "[studica][hardware]") {
  Fixture f;
  CHECK_FALSE(f.backend->InitEncoder(1, 1));
  CHECK_FALSE(f.backend->InitEncoder(-1, 0));
  CHECK_FALSE(f.backend->InitAnalog(256));
  CHECK_FALSE(f.backend->InitDigital(-1, true));
  CHECK_FALSE(f.backend->InitPwm(256, 20000));
  CHECK_FALSE(f.backend->InitDutyCycle(256));
  CHECK_FALSE(f.backend->InitInterrupt(256));
  CHECK(f.vmx->io.allocated == 0);
}
TEST_CASE("SDK exceptions cannot cross the real plugin ABI",
          "[studica][hardware]") {
  Fixture f;
  std::unique_ptr<wpilibstudica_BackendApi, decltype(&DestroyBackendApi)> api{
      MakeBackendApi(std::make_unique<StudicaBackend>()), DestroyBackendApi};
  f.vmx->io.throwActivate = true;
  CHECK(api->initAnalog(api->ctx, 22) == 0);
  CHECK(api->initEncoder(api->ctx, 0, 1) == 0);
  CHECK(api->initInterrupt(api->ctx, 5) == 0);
  f.vmx->io.throwActivate = false;
  CHECK(api->initAnalog(api->ctx, 22) == 1);
}

TEST_CASE("Real hardware rejects null SDK outputs without consuming data",
          "[studica][hardware]") {
  Fixture f;
  REQUIRE(f.backend->InitAnalog(22));
  REQUIRE(f.backend->InitEncoder(0, 1));
  REQUIRE(f.backend->InitDigital(2, true));
  REQUIRE(f.backend->InitDutyCycle(3));
  REQUIRE(f.backend->InitInterrupt(4));
  REQUIRE(f.backend->InitImu());
  f.vmx->io.FireLateInterrupt();
  CHECK_FALSE(f.backend->GetAnalogVoltage(22, nullptr));
  CHECK_FALSE(f.backend->GetEncoderCount(0, nullptr));
  CHECK_FALSE(f.backend->TryGetDigital(2, nullptr));
  CHECK_FALSE(f.backend->GetDutyCycle(3, nullptr));
  CHECK_FALSE(f.backend->GetImu(nullptr));
  CHECK_FALSE(f.backend->GetBatteryVoltage(nullptr));
  CHECK_FALSE(f.backend->ReadInterrupt(4, nullptr));
  InterruptEvent event;
  CHECK(f.backend->ReadInterrupt(4, &event));
}

TEST_CASE("Interrupt callbacks may overlap hardware teardown",
          "[studica][hardware]") {
  Fixture f;
  REQUIRE(f.backend->InitInterrupt(5));
  std::atomic<bool> entered{false};
  std::thread callbacks{[&] {
    entered = true;
    for (int i = 0; i < 10000; ++i)
      f.vmx->io.FireLateInterrupt();
  }};
  while (!entered)
    std::this_thread::yield();
  f.backend.reset();
  callbacks.join();
  CHECK(f.vmx->io.resources.empty());
}

TEST_CASE("Disconnected hardware and IMU calibration reject reads",
          "[studica][hardware]") {
  Fixture f;
  f.vmx->open = false;
  CHECK_FALSE(f.backend->InitAnalog(22));
  CHECK_FALSE(f.backend->InitEncoder(0, 1));
  CHECK_FALSE(f.backend->InitDigital(2, true));
  CHECK_FALSE(f.backend->InitPwm(3, 20000));
  CHECK_FALSE(f.backend->InitDutyCycle(4));
  CHECK_FALSE(f.backend->InitInterrupt(5));
  CHECK_FALSE(f.backend->InitI2c());
  CHECK_FALSE(f.backend->InitImu());
  f.vmx->open = true;
  REQUIRE(f.backend->InitImu());
  ImuSample sample;
  sample.yawDeg = 999;
  f.vmx->connected = false;
  CHECK_FALSE(f.backend->GetImu(&sample));
  CHECK(sample.yawDeg == 999);
  f.vmx->connected = true;
  f.vmx->calibrating = true;
  CHECK_FALSE(f.backend->GetImu(&sample));
  CHECK(sample.yawDeg == 999);
  f.vmx->calibrating = false;
  CHECK(f.backend->GetImu(&sample));
  CHECK(sample.yawDeg == 10);
}
TEST_CASE(
    "Driver factories inject the same VMX into either constructor convention",
    "[studica][hardware]") {
  struct Last {
    std::shared_ptr<VMXPi> vmx;
    Last(int, std::shared_ptr<VMXPi> owner) : vmx{std::move(owner)} {}
  };
  struct First {
    std::shared_ptr<VMXPi> vmx;
    First(std::shared_ptr<VMXPi> owner, int) : vmx{std::move(owner)} {}
  };
  Fixture f;
  CHECK(MakeDriver<Last>(7)->vmx == f.vmx);
  CHECK(MakeDriver<First>(7)->vmx == f.vmx);
}
