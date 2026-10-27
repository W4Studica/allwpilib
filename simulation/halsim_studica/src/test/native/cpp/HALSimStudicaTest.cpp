// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/studica/HALSimStudica.hpp"

#include <cmath>
#include <memory>
#include <utility>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "wpi/hal/AnalogInput.h"
#include "wpi/hal/DIO.h"
#include "wpi/hal/Encoder.h"
#include "wpi/hal/IMU.h"
#include "wpi/hal/PWM.h"
#include "wpi/hal/simulation/AnalogInData.h"
#include "wpi/hal/simulation/DIOData.h"
#include "wpi/hal/simulation/EncoderData.h"
#include "wpi/hal/simulation/PWMData.h"
#include "wpi/hal/simulation/RoboRioData.h"
#include "wpi/studica/detail/backend/LoopbackBackend.hpp"

using namespace wpi::studica;

namespace {

// WPILib DIO 2 <-> VMX channel 20 (arbitrary numbers for the test only).
constexpr int kDio = 2;
constexpr int kVmx = 20;
// WPILib analog 1 <-> VMX channel 30.
constexpr int kAnalog = 1;
constexpr int kVmxAnalog = 30;
// Encoder on WPILib DIO 4/5 <-> VMX 40/41.
constexpr int kEncA = 4;
constexpr int kEncB = 5;
constexpr int kVmxEncA = 40;
constexpr int kVmxEncB = 41;
// PWM 3 <-> VMX channel 15.
constexpr int kPwm = 3;
constexpr int kVmxPwm = 15;

struct Fixture {
  explicit Fixture(bool imuEnabled = false,
                   std::unique_ptr<LoopbackBackend> backend =
                       std::make_unique<LoopbackBackend>()) {
    HALSIM_ResetDIOData(kDio);
    HALSIM_ResetAnalogInData(kAnalog);
    HALSIM_ResetEncoderData(0);
    HALSIM_ResetDIOData(kEncA);
    HALSIM_ResetDIOData(kEncB);
    HALSIM_ResetPWMData(kPwm);
    HALSIM_ResetRoboRioData();
    hw = backend.get();
    Config config;
    config.maps.dio = ChannelMap::Parse("2:20,4:40,5:41");
    config.maps.analog = ChannelMap::Parse("1:30");
    config.maps.pwm = ChannelMap::Parse("3:15");
    config.imu = imuEnabled;
    sim =
        std::make_unique<HALSimStudica>(std::move(backend), std::move(config));
    sim->Start(false);
  }
  ~Fixture() {
    sim.reset();
    HALSIM_ResetDIOData(kDio);
    HALSIM_ResetAnalogInData(kAnalog);
    HALSIM_ResetEncoderData(0);
    HALSIM_ResetDIOData(kEncA);
    HALSIM_ResetDIOData(kEncB);
    HALSIM_ResetPWMData(kPwm);
    HALSIM_ResetRoboRioData();
  }
  LoopbackBackend* hw;
  std::unique_ptr<HALSimStudica> sim;
};

}  // namespace

TEST_CASE("Unmapped channels are ignored", "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeDIOPort(5, false, nullptr, &status);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsClaimed(5));
  HAL_FreeDIOPort(handle);
  HALSIM_ResetDIOData(5);
}

TEST_CASE("Output pin is claimed and follows the sim value",
          "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeDIOPort(kDio, false, nullptr, &status);
  REQUIRE(status == 0);

  f.sim->Poll();
  REQUIRE(f.hw->IsClaimed(kVmx));
  REQUIRE_FALSE(f.hw->IsInput(kVmx));
  REQUIRE(f.hw->GetDigital(kVmx) == true);  // sim HAL initializes outputs high

  HAL_SetDIO(handle, false, &status);
  REQUIRE(f.hw->GetDigital(kVmx) == false);
  HAL_SetDIO(handle, true, &status);
  REQUIRE(f.hw->GetDigital(kVmx) == true);

  HAL_FreeDIOPort(handle);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsClaimed(kVmx));
}

TEST_CASE("Input pin publishes hardware changes to the sim",
          "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeDIOPort(kDio, true, nullptr, &status);

  f.sim->Poll();
  REQUIRE(f.hw->IsClaimed(kVmx));
  REQUIRE(f.hw->IsInput(kVmx));

  f.hw->Drive(kVmx, false);
  f.sim->Poll();
  REQUIRE(HAL_GetDIO(handle, &status) == false);

  f.hw->Drive(kVmx, true);
  f.sim->Poll();
  REQUIRE(HAL_GetDIO(handle, &status) == true);

  HAL_FreeDIOPort(handle);
}

TEST_CASE("Input value written by the extension is not echoed to hardware",
          "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeDIOPort(kDio, true, nullptr, &status);
  f.sim->Poll();

  f.hw->Drive(kVmx, false);
  f.sim->Poll();  // publishes false into the sim; callback must not write back
  REQUIRE(f.hw->GetDigital(kVmx) == false);

  HAL_FreeDIOPort(handle);
}

TEST_CASE("Direction change re-claims the pin", "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeDIOPort(kDio, true, nullptr, &status);
  f.sim->Poll();
  REQUIRE(f.hw->IsInput(kVmx));

  HALSIM_SetDIOIsInput(kDio, false);
  f.sim->Poll();
  REQUIRE(f.hw->IsClaimed(kVmx));
  REQUIRE_FALSE(f.hw->IsInput(kVmx));

  HAL_FreeDIOPort(handle);
}

TEST_CASE("Stopping releases claimed pins", "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeDIOPort(kDio, false, nullptr, &status);
  f.sim->Poll();
  REQUIRE(f.hw->IsClaimed(kVmx));

  f.sim->Stop();
  REQUIRE_FALSE(f.hw->IsClaimed(kVmx));

  HAL_FreeDIOPort(handle);
}

TEST_CASE("Analog input is claimed and publishes hardware voltage",
          "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeAnalogInputPort(kAnalog, nullptr, &status);
  REQUIRE(status == 0);

  f.sim->Poll();
  REQUIRE(f.hw->IsAnalogClaimed(kVmxAnalog));

  f.hw->DriveAnalog(kVmxAnalog, 2.5);
  f.sim->Poll();
  REQUIRE(HAL_GetAnalogVoltage(handle, &status) == 2.5);

  f.hw->DriveAnalog(kVmxAnalog, 0.75);
  f.sim->Poll();
  REQUIRE(HAL_GetAnalogVoltage(handle, &status) == 0.75);

  HAL_FreeAnalogInputPort(handle);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsAnalogClaimed(kVmxAnalog));
}

TEST_CASE("Unmapped analog channels are ignored", "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeAnalogInputPort(3, nullptr, &status);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsAnalogClaimed(3));
  HAL_FreeAnalogInputPort(handle);
  HALSIM_ResetAnalogInData(3);
}

TEST_CASE("Stopping releases analog channels", "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeAnalogInputPort(kAnalog, nullptr, &status);
  f.sim->Poll();
  REQUIRE(f.hw->IsAnalogClaimed(kVmxAnalog));

  f.sim->Stop();
  REQUIRE_FALSE(f.hw->IsAnalogClaimed(kVmxAnalog));

  HAL_FreeAnalogInputPort(handle);
}

namespace {

// What WPILib's Encoder(a, b) does: two DigitalInputs plus the encoder itself.
struct EncoderHandles {
  HAL_DigitalHandle a;
  HAL_DigitalHandle b;
  HAL_EncoderHandle enc;
};

EncoderHandles MakeEncoder(bool reverse = false) {
  int32_t status = 0;
  EncoderHandles h;
  h.a = HAL_InitializeDIOPort(kEncA, true, nullptr, &status);
  h.b = HAL_InitializeDIOPort(kEncB, true, nullptr, &status);
  h.enc = HAL_InitializeEncoder(kEncA, kEncB, reverse, HAL_ENCODER_4X_ENCODING,
                                &status);
  return h;
}

void FreeEncoder(const EncoderHandles& h) {
  HAL_FreeEncoder(h.enc);
  HAL_FreeDIOPort(h.a);
  HAL_FreeDIOPort(h.b);
}

}  // namespace

TEST_CASE("Encoder claims its channels and the DIOs on them are skipped",
          "[halsim_studica]") {
  Fixture f;
  auto h = MakeEncoder();
  f.sim->Poll();
  REQUIRE(f.hw->IsEncoderClaimed(kVmxEncA));
  REQUIRE_FALSE(f.hw->IsClaimed(kVmxEncA));
  REQUIRE_FALSE(f.hw->IsClaimed(kVmxEncB));

  FreeEncoder(h);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsEncoderClaimed(kVmxEncA));
}

TEST_CASE("Encoder claim wins even if the DIOs were polled first",
          "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  // DigitalInputs exist before the encoder (Encoder constructor order).
  auto a = HAL_InitializeDIOPort(kEncA, true, nullptr, &status);
  auto b = HAL_InitializeDIOPort(kEncB, true, nullptr, &status);
  f.sim->Poll();
  REQUIRE(f.hw->IsClaimed(kVmxEncA));  // DIO claimed it for now

  auto enc = HAL_InitializeEncoder(kEncA, kEncB, false, HAL_ENCODER_4X_ENCODING,
                                   &status);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsClaimed(kVmxEncA));
  REQUIRE_FALSE(f.hw->IsClaimed(kVmxEncB));
  REQUIRE(f.hw->IsEncoderClaimed(kVmxEncA));

  HAL_FreeEncoder(enc);
  HAL_FreeDIOPort(a);
  HAL_FreeDIOPort(b);
}

TEST_CASE("Encoder count follows hardware counts since it was claimed",
          "[halsim_studica]") {
  Fixture f;
  auto h = MakeEncoder();
  int32_t status = 0;
  f.sim->Poll();
  REQUIRE(HAL_GetEncoder(h.enc, &status) == 0);

  f.hw->DriveEncoder(kVmxEncA, 120);
  f.sim->Poll();
  REQUIRE(HAL_GetEncoder(h.enc, &status) == 120);
  REQUIRE(HAL_GetEncoderDirection(h.enc, &status) == true);
  REQUIRE(HAL_GetEncoderRate(h.enc, &status) > 0.0);

  f.hw->DriveEncoder(kVmxEncA, 100);
  f.sim->Poll();
  REQUIRE(HAL_GetEncoder(h.enc, &status) == 100);
  REQUIRE(HAL_GetEncoderDirection(h.enc, &status) == false);
  REQUIRE(HAL_GetEncoderRate(h.enc, &status) < 0.0);

  f.sim->Poll();  // no movement
  REQUIRE(HAL_GetEncoderRate(h.enc, &status) == 0.0);

  FreeEncoder(h);
}

TEST_CASE("Encoder reset zeroes the reported count and clears the flag",
          "[halsim_studica]") {
  Fixture f;
  auto h = MakeEncoder();
  int32_t status = 0;
  f.sim->Poll();

  f.hw->DriveEncoder(kVmxEncA, 500);
  f.sim->Poll();
  REQUIRE(HAL_GetEncoder(h.enc, &status) == 500);

  HAL_ResetEncoder(h.enc, &status);
  f.sim->Poll();
  REQUIRE(HAL_GetEncoder(h.enc, &status) == 0);
  REQUIRE_FALSE(HALSIM_GetEncoderReset(0));

  f.hw->DriveEncoder(kVmxEncA, 530);
  f.sim->Poll();
  REQUIRE(HAL_GetEncoder(h.enc, &status) == 30);

  FreeEncoder(h);
}

TEST_CASE("Encoder honors reverse direction", "[halsim_studica]") {
  Fixture f;
  auto h = MakeEncoder(true);
  int32_t status = 0;
  f.sim->Poll();

  f.hw->DriveEncoder(kVmxEncA, 40);
  f.sim->Poll();
  REQUIRE(HAL_GetEncoder(h.enc, &status) == -40);

  FreeEncoder(h);
}

TEST_CASE("Stopping releases encoders", "[halsim_studica]") {
  Fixture f;
  auto h = MakeEncoder();
  f.sim->Poll();
  REQUIRE(f.hw->IsEncoderClaimed(kVmxEncA));

  f.sim->Stop();
  REQUIRE_FALSE(f.hw->IsEncoderClaimed(kVmxEncA));

  FreeEncoder(h);
}

TEST_CASE("IMU is not claimed unless enabled", "[halsim_studica]") {
  Fixture f;
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsImuClaimed());
}

TEST_CASE("IMU converts navX units and signs to the WPILib IMU HAL",
          "[halsim_studica]") {
  Fixture f{true};
  ImuSample sample;
  sample.yawDeg = 90;
  sample.pitchDeg = 10;
  sample.rollDeg = -20;
  sample.gyroXDps = 180;
  sample.gyroYDps = 0;
  sample.gyroZDps = 90;
  sample.accelXG = 0.5;
  sample.accelYG = 0;
  sample.accelZG = 1;
  f.sim->Poll();  // claims the IMU
  REQUIRE(f.hw->IsImuClaimed());
  f.hw->DriveImu(sample);
  f.sim->Poll();

  constexpr double kPi = 3.14159265358979323846;
  int64_t timestamp = 0;
  REQUIRE(HAL_GetIMUYawFlat(&timestamp) == Catch::Approx(-kPi / 2));

  int32_t status = 0;
  HAL_EulerAngles3d angles;
  HAL_GetIMUEulerAnglesFlat(&angles, &status);
  REQUIRE(angles.x == Catch::Approx(-20 * kPi / 180));
  REQUIRE(angles.y == Catch::Approx(10 * kPi / 180));
  REQUIRE(angles.z == Catch::Approx(-kPi / 2));

  HAL_GyroRate3d rates;
  HAL_GetIMUGyroRates(&rates, &status);
  REQUIRE(rates.x == Catch::Approx(kPi));
  REQUIRE(rates.z == Catch::Approx(-kPi / 2));

  HAL_Acceleration3d accel;
  HAL_GetIMUAcceleration(&accel, &status);
  REQUIRE(accel.x == Catch::Approx(0.5 * 9.80665));
  REQUIRE(accel.z == Catch::Approx(9.80665));
}

TEST_CASE("IMU values are not published while disconnected",
          "[halsim_studica]") {
  Fixture f{true};
  f.sim->Poll();
  ImuSample sample;
  sample.yawDeg = 45;
  f.hw->DriveImu(sample, false);
  f.sim->Poll();

  int64_t timestamp = 0;
  REQUIRE(HAL_GetIMUYawFlat(&timestamp) !=
          Catch::Approx(-45 * 3.14159265 / 180));
}

TEST_CASE("Stopping releases the IMU", "[halsim_studica]") {
  Fixture f{true};
  f.sim->Poll();
  REQUIRE(f.hw->IsImuClaimed());
  f.sim->Stop();
  REQUIRE_FALSE(f.hw->IsImuClaimed());
}

TEST_CASE("PWM output is claimed and follows the pulse width",
          "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializePWMPort(kPwm, nullptr, &status);
  REQUIRE(status == 0);
  HAL_SetPWMOutputPeriod(handle, 3, &status);  // 20 ms

  f.sim->Poll();
  REQUIRE(f.hw->IsPwmClaimed(kVmxPwm));
  REQUIRE(f.hw->PwmPeriod(kVmxPwm) == 20000);
  REQUIRE(f.hw->PwmPulse(kVmxPwm) == 0);  // initialised disabled

  HAL_SetPWMPulseTimeMicroseconds(handle, 1500, &status);
  f.sim->Poll();
  REQUIRE(f.hw->PwmPulse(kVmxPwm) == 1500);

  HAL_SetPWMPulseTimeMicroseconds(handle, 0, &status);  // disabled again
  f.sim->Poll();
  REQUIRE(f.hw->PwmPulse(kVmxPwm) == 0);

  HAL_FreePWMPort(handle);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsPwmClaimed(kVmxPwm));
}

TEST_CASE("A new PWM frame length restarts the generator", "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializePWMPort(kPwm, nullptr, &status);
  HAL_SetPWMOutputPeriod(handle, 3, &status);
  HAL_SetPWMPulseTimeMicroseconds(handle, 1500, &status);
  f.sim->Poll();
  REQUIRE(f.hw->PwmPeriod(kVmxPwm) == 20000);

  HAL_SetPWMOutputPeriod(handle, 0, &status);  // 5 ms
  f.sim->Poll();
  REQUIRE(f.hw->PwmPeriod(kVmxPwm) == 5000);
  REQUIRE(f.hw->PwmPulse(kVmxPwm) ==
          1500);  // the pulse is written again to the new generator

  HAL_FreePWMPort(handle);
  f.sim->Poll();
}

TEST_CASE("Unmapped PWM channels are ignored", "[halsim_studica]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializePWMPort(7, nullptr, &status);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsPwmClaimed(7));
  HAL_FreePWMPort(handle);
  HALSIM_ResetPWMData(7);
}

TEST_CASE("The battery voltage reaches the RoboRIO sim data",
          "[halsim_studica]") {
  Fixture f;
  f.sim->Poll();  // no reading yet: the sim value is left alone
  const double before = HALSIM_GetRoboRioVInVoltage();

  f.hw->DriveBattery(11.4);
  // Read once per ten polls.
  for (int i = 0; i < 10; ++i) {
    f.sim->Poll();
  }
  REQUIRE(HALSIM_GetRoboRioVInVoltage() == Catch::Approx(11.4));

  f.hw->DriveBattery(0.0, false);  // a lost reading keeps the last value
  for (int i = 0; i < 10; ++i) {
    f.sim->Poll();
  }
  REQUIRE(HALSIM_GetRoboRioVInVoltage() == Catch::Approx(11.4));
  (void)before;
}

TEST_CASE("Encoder reset survives a failed hardware read",
          "[halsim_studica][encoder]") {
  struct Backend : LoopbackBackend {
    bool fail = false;
    bool GetEncoderCount(int channel, int32_t* count) override {
      return !fail && LoopbackBackend::GetEncoderCount(channel, count);
    }
  };
  auto backend = std::make_unique<Backend>();
  auto* hw = backend.get();
  Fixture f{false, std::move(backend)};
  auto h = MakeEncoder();
  f.sim->Poll();
  hw->DriveEncoder(kVmxEncA, 120);
  f.sim->Poll();
  hw->fail = true;
  HALSIM_SetEncoderReset(0, true);
  f.sim->Poll();
  const bool retained = HALSIM_GetEncoderReset(0);
  hw->fail = false;
  hw->DriveEncoder(kVmxEncA, 130);
  f.sim->Poll();
  const auto count = HALSIM_GetEncoderCount(0);
  const bool acknowledged = !HALSIM_GetEncoderReset(0);
  FreeEncoder(h);
  REQUIRE(retained);
  REQUIRE(acknowledged);
  REQUIRE(count == 0);
}
TEST_CASE("Encoder waits for a valid initial hardware baseline",
          "[halsim_studica][encoder]") {
  struct Backend : LoopbackBackend {
    bool fail = true;
    bool InitEncoder(int a, int b) override {
      if (!LoopbackBackend::InitEncoder(a, b))
        return false;
      DriveEncoder(a, 700);
      return true;
    }
    bool GetEncoderCount(int channel, int32_t* count) override {
      return !fail && LoopbackBackend::GetEncoderCount(channel, count);
    }
  };
  auto backend = std::make_unique<Backend>();
  auto* hw = backend.get();
  Fixture f{false, std::move(backend)};
  auto h = MakeEncoder();
  f.sim->Poll();
  hw->fail = false;
  f.sim->Poll();
  const auto initial = HALSIM_GetEncoderCount(0);
  hw->DriveEncoder(kVmxEncA, 705);
  f.sim->Poll();
  const auto moved = HALSIM_GetEncoderCount(0);
  FreeEncoder(h);
  REQUIRE(initial == 0);
  REQUIRE(moved == 5);
}
