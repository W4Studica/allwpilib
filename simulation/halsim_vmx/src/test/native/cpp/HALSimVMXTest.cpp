// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/vmx/HALSimVMX.hpp"

#include <memory>
#include <utility>

#include <catch2/catch_test_macros.hpp>

#include "wpi/hal/DIO.h"
#include "wpi/hal/simulation/DIOData.h"
#include "wpi/halsim/vmx/LoopbackBackend.hpp"

using namespace wpilibvmx;

namespace {

// WPILib DIO 2 <-> VMX channel 20 (arbitrary numbers for the test only).
constexpr int kDio = 2;
constexpr int kVmx = 20;

struct Fixture {
  Fixture() {
    HALSIM_ResetDIOData(kDio);
    auto backend = std::make_unique<LoopbackBackend>();
    hw = backend.get();
    sim = std::make_unique<HALSimVMX>(std::move(backend),
                                      ChannelMap::Parse("2:20"));
    sim->Start(false);
  }
  ~Fixture() {
    sim.reset();
    HALSIM_ResetDIOData(kDio);
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
