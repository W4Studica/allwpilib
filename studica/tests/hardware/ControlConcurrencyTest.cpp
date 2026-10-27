// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <atomic>
#include <chrono>
#include <thread>

#include <catch2/catch_test_macros.hpp>

#include "wpi/studica/detail/titan/TitanControl.hpp"

TEST_CASE("Titan feeding and state reads may overlap permanent stop",
          "[studica][concurrency]") {
  using namespace std::chrono_literals;
  struct Transport {
    std::atomic<bool> enabled{false};
    void Enable(bool value) { enabled = value; }
    void KeepAlive() {}
    bool LastSendOk() const { return true; }
  };
  for (int run = 0; run < 50; ++run) {
    Transport transport;
    wpi::studica::detail::TitanControl control{transport, 100ms, 150ms,
                                               [] { return true; }};
    std::atomic<bool> done{false};
    std::atomic<int> feeds{0}, reads{0};
    std::thread feeder{[&] {
      while (!done) {
        control.FeedControlLoop();
        control.FeedDriverStation();
        ++feeds;
      }
    }};
    std::thread observer{[&] {
      while (!done) {
        control.CanCommand();
        control.IsEnabled();
        ++reads;
      }
    }};
    while (feeds < 10 || reads < 10)
      std::this_thread::yield();
    control.Stop();
    done = true;
    feeder.join();
    observer.join();
    REQUIRE_FALSE(control.IsEnabled());
    REQUIRE_FALSE(transport.enabled);
  }
}
