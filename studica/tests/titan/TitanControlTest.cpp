// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/detail/titan/TitanControl.hpp"

#include <chrono>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "wpi/hal/simulation/DriverStationData.h"
#include "wpi/studica/Titan.hpp"

using namespace std::chrono_literals;
using wpi::studica::detail::ControlLoopWatchdog;
using wpi::studica::detail::TitanControl;

// The user-facing header compiles without the VMX SDK or vendor driver headers.
static_assert(std::is_constructible_v<wpi::studica::Titan, uint8_t>);
static_assert(!std::is_copy_constructible_v<wpi::studica::Titan>);

namespace {
struct Transport {
  bool enabled = false;
  bool healthy = true;
  std::vector<bool> transitions;
  void Enable(bool state) {
    enabled = state;
    transitions.push_back(state);
  }
  void KeepAlive() {}
  bool LastSendOk() const { return healthy; }
};
using Clock = ControlLoopWatchdog::Clock;
}  // namespace

TEST_CASE("Unified Titan requires enabled state and both fresh heartbeats",
          "[studica][titan]") {
  Transport transport;
  Clock::time_point now{};
  bool enabled = false;
  TitanControl control{
      transport,           100ms, 150ms, [&] { return enabled; },
      [&] { return now; }, false};
  REQUIRE_FALSE(control.CanCommand());
  enabled = true;
  control.FeedControlLoop();
  REQUIRE_FALSE(control.CanCommand());
  control.FeedDriverStation();
  REQUIRE(control.CanCommand());
  REQUIRE(transport.enabled);
  enabled = false;
  REQUIRE_FALSE(control.CanCommand());
  REQUIRE_FALSE(transport.enabled);
}

TEST_CASE("Motor commands cannot feed the unified Titan watchdog",
          "[studica][titan]") {
  Transport transport;
  Clock::time_point now{};
  TitanControl control{transport,           100ms, 150ms, [] { return true; },
                       [&] { return now; }, false};
  control.FeedControlLoop();
  control.FeedDriverStation();
  REQUIRE(control.CanCommand());
  now += 99ms;
  for (int i = 0; i < 20; ++i) {
    REQUIRE(control.CanCommand());
  }
  now += 1ms;
  REQUIRE_FALSE(control.CanCommand());
  REQUIRE_FALSE(transport.enabled);
  // A control heartbeat alone cannot conceal an expired DS heartbeat.
  control.FeedControlLoop();
  REQUIRE(control.CanCommand());
  now += 50ms;
  control.FeedControlLoop();
  REQUIRE_FALSE(control.CanCommand());
  REQUIRE_FALSE(transport.enabled);
}

TEST_CASE("Unified Titan suppresses commands when enable delivery fails",
          "[studica][titan]") {
  Transport transport;
  Clock::time_point now{};
  TitanControl control{transport,           100ms, 150ms, [] { return true; },
                       [&] { return now; }, false};
  control.FeedControlLoop();
  control.FeedDriverStation();
  transport.healthy = false;
  REQUIRE_FALSE(control.CanCommand());
  transport.healthy = true;
  REQUIRE(control.CanCommand());
}

TEST_CASE("Stopping a unified Titan cannot be undone by feeding it",
          "[studica][titan]") {
  Transport transport;
  Clock::time_point now{};
  TitanControl control{transport,           100ms, 150ms, [] { return true; },
                       [&] { return now; }, false};
  control.FeedControlLoop();
  control.FeedDriverStation();
  REQUIRE(control.CanCommand());
  control.Stop();
  control.FeedControlLoop();
  control.FeedDriverStation();
  REQUIRE_FALSE(control.CanCommand());
  REQUIRE_FALSE(transport.enabled);
}

TEST_CASE("Unified Titan destruction disables the device", "[studica][titan]") {
  Transport transport;
  Clock::time_point now{};
  {
    TitanControl control{transport,           100ms, 150ms, [] { return true; },
                         [&] { return now; }, false};
    control.FeedControlLoop();
    control.FeedDriverStation();
    REQUIRE(control.CanCommand());
  }
  REQUIRE_FALSE(transport.enabled);
  REQUIRE_FALSE(transport.transitions.back());
}

TEST_CASE("A failing Titan enabled source is treated as disabled",
          "[studica][titan]") {
  Transport transport;
  Clock::time_point now{};
  TitanControl control{transport,
                       100ms,
                       150ms,
                       []() -> bool { throw std::runtime_error("DS failed"); },
                       [&] { return now; },
                       false};
  control.FeedControlLoop();
  control.FeedDriverStation();
  REQUIRE_FALSE(control.CanCommand());
  REQUIRE_FALSE(transport.enabled);
}

TEST_CASE("Unified Titan default source observes HAL disable and E-stop",
          "[studica][titan]") {
  const bool attached = HALSIM_GetDriverStationDsAttached();
  const bool enabled = HALSIM_GetDriverStationEnabled();
  const bool stopped = HALSIM_GetDriverStationEStop();
  struct Restore {
    bool attached, enabled, stopped;
    ~Restore() {
      HALSIM_SetDriverStationDsAttached(attached);
      HALSIM_SetDriverStationEnabled(enabled);
      HALSIM_SetDriverStationEStop(stopped);
    }
  } restore{attached, enabled, stopped};
  HALSIM_SetDriverStationDsAttached(true);
  HALSIM_SetDriverStationEnabled(true);
  HALSIM_SetDriverStationEStop(false);
  REQUIRE(wpi::studica::Titan::HalEnabled());
  HALSIM_SetDriverStationEStop(true);
  REQUIRE_FALSE(wpi::studica::Titan::HalEnabled());
  HALSIM_SetDriverStationEStop(false);
  HALSIM_SetDriverStationDsAttached(false);
  REQUIRE_FALSE(wpi::studica::Titan::HalEnabled());
}
