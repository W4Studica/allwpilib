// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/vmx/HALSimVMX.hpp"

#include <memory>
#include <utility>

#include <catch2/catch_test_macros.hpp>

#include "wpi/hal/AnalogInput.h"
#include "wpi/hal/DIO.h"
#include "wpi/hal/Encoder.h"
#include "wpi/hal/simulation/AnalogInData.h"
#include "wpi/hal/simulation/DIOData.h"
#include "wpi/hal/simulation/EncoderData.h"
#include "wpi/halsim/vmx/LoopbackBackend.hpp"

using namespace wpilibvmx;

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

struct Fixture {
  Fixture() {
    HALSIM_ResetDIOData(kDio);
    HALSIM_ResetAnalogInData(kAnalog);
    HALSIM_ResetEncoderData(0);
    HALSIM_ResetDIOData(kEncA);
    HALSIM_ResetDIOData(kEncB);
    auto backend = std::make_unique<LoopbackBackend>();
    hw = backend.get();
    ChannelMaps maps;
    maps.dio = ChannelMap::Parse("2:20,4:40,5:41");
    maps.analog = ChannelMap::Parse("1:30");
    sim = std::make_unique<HALSimVMX>(std::move(backend), std::move(maps));
    sim->Start(false);
  }
  ~Fixture() {
    sim.reset();
    HALSIM_ResetDIOData(kDio);
    HALSIM_ResetAnalogInData(kAnalog);
    HALSIM_ResetEncoderData(0);
    HALSIM_ResetDIOData(kEncA);
    HALSIM_ResetDIOData(kEncB);
  }
  LoopbackBackend* hw;
  std::unique_ptr<HALSimVMX> sim;
};

}  // namespace

TEST_CASE("Unmapped channels are ignored", "[halsim_vmx]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeDIOPort(5, false, nullptr, &status);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsClaimed(5));
  HAL_FreeDIOPort(handle);
  HALSIM_ResetDIOData(5);
}

TEST_CASE("Output pin is claimed and follows the sim value", "[halsim_vmx]") {
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

TEST_CASE("Input pin publishes hardware changes to the sim", "[halsim_vmx]") {
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
          "[halsim_vmx]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeDIOPort(kDio, true, nullptr, &status);
  f.sim->Poll();

  f.hw->Drive(kVmx, false);
  f.sim->Poll();  // publishes false into the sim; callback must not write back
  REQUIRE(f.hw->GetDigital(kVmx) == false);

  HAL_FreeDIOPort(handle);
}

TEST_CASE("Direction change re-claims the pin", "[halsim_vmx]") {
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

TEST_CASE("Stopping releases claimed pins", "[halsim_vmx]") {
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
          "[halsim_vmx]") {
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

TEST_CASE("Unmapped analog channels are ignored", "[halsim_vmx]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeAnalogInputPort(3, nullptr, &status);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsAnalogClaimed(3));
  HAL_FreeAnalogInputPort(handle);
  HALSIM_ResetAnalogInData(3);
}

TEST_CASE("Stopping releases analog channels", "[halsim_vmx]") {
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
  h.enc = HAL_InitializeEncoder(kEncA, kEncB, reverse,
                                HAL_ENCODER_4X_ENCODING, &status);
  return h;
}

void FreeEncoder(const EncoderHandles& h) {
  HAL_FreeEncoder(h.enc);
  HAL_FreeDIOPort(h.a);
  HAL_FreeDIOPort(h.b);
}

}  // namespace

TEST_CASE("Encoder claims its channels and the DIOs on them are skipped",
          "[halsim_vmx]") {
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
          "[halsim_vmx]") {
  Fixture f;
  int32_t status = 0;
  // DigitalInputs exist before the encoder (Encoder constructor order).
  auto a = HAL_InitializeDIOPort(kEncA, true, nullptr, &status);
  auto b = HAL_InitializeDIOPort(kEncB, true, nullptr, &status);
  f.sim->Poll();
  REQUIRE(f.hw->IsClaimed(kVmxEncA));  // DIO claimed it for now

  auto enc = HAL_InitializeEncoder(kEncA, kEncB, false,
                                   HAL_ENCODER_4X_ENCODING, &status);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsClaimed(kVmxEncA));
  REQUIRE_FALSE(f.hw->IsClaimed(kVmxEncB));
  REQUIRE(f.hw->IsEncoderClaimed(kVmxEncA));

  HAL_FreeEncoder(enc);
  HAL_FreeDIOPort(a);
  HAL_FreeDIOPort(b);
}

TEST_CASE("Encoder count follows hardware counts since it was claimed",
          "[halsim_vmx]") {
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
          "[halsim_vmx]") {
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

TEST_CASE("Encoder honors reverse direction", "[halsim_vmx]") {
  Fixture f;
  auto h = MakeEncoder(true);
  int32_t status = 0;
  f.sim->Poll();

  f.hw->DriveEncoder(kVmxEncA, 40);
  f.sim->Poll();
  REQUIRE(HAL_GetEncoder(h.enc, &status) == -40);

  FreeEncoder(h);
}

TEST_CASE("Stopping releases encoders", "[halsim_vmx]") {
  Fixture f;
  auto h = MakeEncoder();
  f.sim->Poll();
  REQUIRE(f.hw->IsEncoderClaimed(kVmxEncA));

  f.sim->Stop();
  REQUIRE_FALSE(f.hw->IsEncoderClaimed(kVmxEncA));

  FreeEncoder(h);
}
