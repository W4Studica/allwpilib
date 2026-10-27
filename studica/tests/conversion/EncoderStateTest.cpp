// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <chrono>
#include <limits>

#include <catch2/catch_test_macros.hpp>

#include "wpi/studica/detail/conversion/DeviceConversions.hpp"
using namespace wpi::studica;
using namespace std::chrono_literals;

TEST_CASE("Encoder arithmetic wraps at hardware counter boundaries",
          "[studica][encoder]") {
  constexpr auto max = std::numeric_limits<int32_t>::max();
  constexpr auto min = std::numeric_limits<int32_t>::min();
  const std::chrono::steady_clock::time_point t{};
  EncoderState state;
  state.Initialize(max);
  state.Update(max, false, false, 1, t);
  auto forward = state.Update(min, false, false, 1, t + 1s);
  REQUIRE(forward.count == 1);
  REQUIRE(forward.rate == 1);
  REQUIRE(forward.direction);
  auto backward = state.Update(max, false, false, 1, t + 2s);
  REQUIRE(backward.count == 0);
  REQUIRE(backward.rate == -1);
  REQUIRE_FALSE(backward.direction);
}
TEST_CASE("Reversing an INT_MIN encoder count has defined wrapping semantics",
          "[studica][encoder]") {
  EncoderState state;
  state.Initialize(0);
  auto reading =
      state.Update(std::numeric_limits<int32_t>::min(), false, true, 1, {});
  REQUIRE(reading.count == std::numeric_limits<int32_t>::min());
}
