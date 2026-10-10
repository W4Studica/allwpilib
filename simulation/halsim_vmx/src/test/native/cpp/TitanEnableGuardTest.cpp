// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/vmx/TitanEnableGuard.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "wpi/hal/simulation/DriverStationData.h"

using wpilibvmx::TitanEnableGuard;

namespace {

struct FakeTitan {
  void Enable(bool enable) {
    std::scoped_lock lock{mutex};
    calls.push_back(enable);
  }
  std::vector<bool> Calls() {
    std::scoped_lock lock{mutex};
    return calls;
  }
  std::mutex mutex;
  std::vector<bool> calls;
};

template <class Pred>
bool WaitFor(Pred pred) {
  auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < end) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

struct DsRestore {
  ~DsRestore() {
    HALSIM_SetDriverStationEnabled(false);
    HALSIM_SetDriverStationEStop(false);
    HALSIM_SetDriverStationDsAttached(false);
  }
};

}  // namespace

TEST_CASE("The first poll always tells the Titan its state", "[halsim_vmx][titan]") {
  FakeTitan titan;
  TitanEnableGuard<FakeTitan> guard{titan, [] { return false; }};
  guard.Poll();
  REQUIRE(titan.Calls() == std::vector<bool>{false});
}

TEST_CASE("Enable is sent only when the state changes", "[halsim_vmx][titan]") {
  FakeTitan titan;
  std::atomic<bool> enabled{false};
  TitanEnableGuard<FakeTitan> guard{titan, [&] { return enabled.load(); }};

  guard.Poll();
  guard.Poll();
  enabled = true;
  guard.Poll();
  guard.Poll();
  enabled = false;
  guard.Poll();
  guard.Poll();

  REQUIRE(titan.Calls() == std::vector<bool>{false, true, false});
}

TEST_CASE("Stopping disables the Titan even while the robot is enabled",
          "[halsim_vmx][titan]") {
  FakeTitan titan;
  {
    TitanEnableGuard<FakeTitan> guard{titan, [] { return true; }};
    guard.Poll();
    REQUIRE(titan.Calls().back() == true);
    guard.Stop();
    REQUIRE(titan.Calls().back() == false);
  }
  // The destructor stops again; the last word is still "disabled".
  REQUIRE(titan.Calls().back() == false);
}

TEST_CASE("Destroying a guard that never started disables the Titan",
          "[halsim_vmx][titan]") {
  FakeTitan titan;
  { TitanEnableGuard<FakeTitan> guard{titan, [] { return true; }}; }
  REQUIRE(titan.Calls() == std::vector<bool>{false});
}

TEST_CASE("The polling thread follows the robot state", "[halsim_vmx][titan]") {
  FakeTitan titan;
  std::atomic<bool> enabled{false};
  TitanEnableGuard<FakeTitan> guard{titan, [&] { return enabled.load(); },
                                    std::chrono::milliseconds(2)};
  guard.Start();

  REQUIRE(WaitFor([&] { return !titan.Calls().empty(); }));
  enabled = true;
  REQUIRE(WaitFor([&] { return titan.Calls().back() == true; }));
  enabled = false;
  REQUIRE(WaitFor([&] { return titan.Calls().back() == false; }));

  guard.Stop();
  auto calls = titan.Calls();
  REQUIRE(calls.front() == false);
  REQUIRE(calls.back() == false);
}

TEST_CASE("The default source reads the HAL control word", "[halsim_vmx][titan]") {
  DsRestore restore;
  using Guard = TitanEnableGuard<FakeTitan>;

  HALSIM_SetDriverStationDsAttached(false);
  HALSIM_SetDriverStationEnabled(true);
  // Enabled but no DS attached: the sim HAL reports a zero control word, so disabled.
  REQUIRE_FALSE(Guard::HalEnabled());

  HALSIM_SetDriverStationDsAttached(true);
  REQUIRE(Guard::HalEnabled());

  HALSIM_SetDriverStationEStop(true);
  REQUIRE_FALSE(Guard::HalEnabled());  // E-stop wins over enabled

  HALSIM_SetDriverStationEStop(false);
  HALSIM_SetDriverStationEnabled(false);
  REQUIRE_FALSE(Guard::HalEnabled());
}

TEST_CASE("A guard with the default source follows the simulated DS",
          "[halsim_vmx][titan]") {
  DsRestore restore;
  FakeTitan titan;
  TitanEnableGuard<FakeTitan> guard{titan};

  HALSIM_SetDriverStationDsAttached(true);
  HALSIM_SetDriverStationEnabled(false);
  guard.Poll();
  HALSIM_SetDriverStationEnabled(true);
  guard.Poll();
  HALSIM_SetDriverStationEnabled(false);
  guard.Poll();

  REQUIRE(titan.Calls() == std::vector<bool>{false, true, false});
}
