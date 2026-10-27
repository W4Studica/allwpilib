// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/detail/titan/TitanEnableGuard.hpp"

#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "wpi/hal/simulation/DriverStationData.h"
#include "wpi/studica/detail/titan/ControlLoopWatchdog.hpp"

using wpi::studica::detail::TitanEnableGuard;

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

// A Titan whose switch also repeats its state, like the internal
// TitanEnableSwitch.
struct KeepAliveTitan : FakeTitan {
  void KeepAlive() {
    std::scoped_lock lock{mutex};
    ++keepAlives;
  }
  int KeepAlives() {
    std::scoped_lock lock{mutex};
    return keepAlives;
  }
  int keepAlives = 0;
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

TEST_CASE("The first poll always tells the Titan its state",
          "[halsim_studica][titan]") {
  FakeTitan titan;
  TitanEnableGuard<FakeTitan> guard{titan, [] { return false; }};
  guard.Poll();
  REQUIRE(titan.Calls() == std::vector<bool>{false});
}

TEST_CASE("Enable is sent only when the state changes",
          "[halsim_studica][titan]") {
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
          "[halsim_studica][titan]") {
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
          "[halsim_studica][titan]") {
  FakeTitan titan;
  {
    TitanEnableGuard<FakeTitan> guard{titan, [] { return true; }};
  }
  REQUIRE(titan.Calls() == std::vector<bool>{false});
}

TEST_CASE("The polling thread follows the robot state",
          "[halsim_studica][titan]") {
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

TEST_CASE("The default source reads the HAL control word",
          "[halsim_studica][titan]") {
  DsRestore restore;
  using Guard = TitanEnableGuard<FakeTitan>;

  HALSIM_SetDriverStationDsAttached(false);
  HALSIM_SetDriverStationEnabled(true);
  // Enabled but no DS attached: the sim HAL reports a zero control word, so
  // disabled.
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
          "[halsim_studica][titan]") {
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

TEST_CASE(
    "A Titan with KeepAlive() hears its state again on every poll, but "
    "Enable() only on a change",
    "[halsim_studica][titan]") {
  KeepAliveTitan titan;
  TitanEnableGuard<KeepAliveTitan> guard{titan, [] { return true; }};
  guard.Poll();  // the first pass: Enable(true)
  REQUIRE(titan.Calls() == std::vector<bool>{true});
  REQUIRE(titan.KeepAlives() == 0);
  guard.Poll();
  guard.Poll();
  guard.Poll();
  REQUIRE(titan.Calls() == std::vector<bool>{true});  // no new Enable()
  REQUIRE(titan.KeepAlives() == 3);
}

TEST_CASE("A Titan without KeepAlive() still works",
          "[halsim_studica][titan]") {
  FakeTitan titan;
  TitanEnableGuard<FakeTitan> guard{titan, [] { return true; }};
  guard.Poll();
  guard.Poll();
  REQUIRE(titan.Calls() == std::vector<bool>{true});
}

TEST_CASE(
    "Stalled control or DS input disables Titan even with cached enabled state",
    "[halsim_studica][titan][watchdog]") {
  using namespace std::chrono_literals;
  using Watchdog = wpi::studica::detail::ControlLoopWatchdog;
  using Guard = TitanEnableGuard<KeepAliveTitan>;
  DsRestore restore;
  HALSIM_SetDriverStationDsAttached(true);
  HALSIM_SetDriverStationEnabled(true);
  Watchdog::Clock::time_point now{};
  Watchdog watchdog{100ms, 150ms, [&] { return now; }};
  KeepAliveTitan titan;
  Guard guard{titan, [&] { return Guard::HalEnabled() && watchdog.IsFresh(); }};

  guard.Poll();
  REQUIRE_FALSE(titan.Calls().back());
  watchdog.FeedControlLoop();
  guard.Poll();
  REQUIRE_FALSE(titan.Calls().back());
  watchdog.FeedDriverStation();
  guard.Poll();
  REQUIRE(titan.Calls().back());
  guard.Poll();
  REQUIRE(titan.KeepAlives() == 2);  // one disabled, one enabled keepalive

  // Main loop stops, but DS input and the guard keep running.
  now += 100ms;
  watchdog.FeedDriverStation();
  guard.Poll();
  REQUIRE(Guard::HalEnabled());
  REQUIRE_FALSE(titan.Calls().back());

  watchdog.FeedControlLoop();
  guard.Poll();
  REQUIRE(titan.Calls().back());
  // DS reception stops while the main loop and guard keep running.
  now += 150ms;
  watchdog.FeedControlLoop();
  guard.Poll();
  REQUIRE(Guard::HalEnabled());
  REQUIRE_FALSE(titan.Calls().back());
  watchdog.FeedDriverStation();
  guard.Poll();
  REQUIRE(titan.Calls().back());
  HALSIM_SetDriverStationEStop(true);
  guard.Poll();
  REQUIRE_FALSE(titan.Calls().back());
}

TEST_CASE("Watchdog rejects nonpositive deadlines",
          "[halsim_studica][titan][watchdog]") {
  using namespace std::chrono_literals;
  REQUIRE_THROWS_AS((wpi::studica::detail::ControlLoopWatchdog{0ms, 100ms}),
                    std::invalid_argument);
  REQUIRE_THROWS_AS((wpi::studica::detail::ControlLoopWatchdog{100ms, -1ms}),
                    std::invalid_argument);
}

TEST_CASE("Stopping waits for an in-flight enable decision",
          "[studica][titan]") {
  struct Switch {
    bool enabled = false;
    void Enable(bool state) { enabled = state; }
  } transport;
  std::promise<void> entered, release;
  auto proceed = release.get_future().share();
  wpi::studica::detail::TitanEnableGuard guard{transport, [&] {
                                                 entered.set_value();
                                                 proceed.wait();
                                                 return true;
                                               }};
  auto poll = std::async(std::launch::async, [&] { guard.Poll(); });
  entered.get_future().wait();
  auto stop = std::async(std::launch::async, [&] { guard.Stop(); });
  const bool waited = stop.wait_for(std::chrono::milliseconds(50)) ==
                      std::future_status::timeout;
  release.set_value();
  poll.get();
  stop.get();
  REQUIRE(waited);
  REQUIRE_FALSE(transport.enabled);
}
