// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/detail/runtime/UniqueResource.hpp"

#include <stdexcept>
#include <vector>

#include <catch2/catch_test_macros.hpp>
using wpi::studica::detail::UniqueResource;

TEST_CASE("Owned resources release once across moves and reset",
          "[studica][resource]") {
  std::vector<int> released;
  auto release = [&](int handle) { released.push_back(handle); };
  {
    UniqueResource<int> first{0, release};
    UniqueResource<int> second{7, release};
    second = std::move(first);
    REQUIRE(released == std::vector<int>{7});
    UniqueResource<int> third{std::move(second)};
    REQUIRE(third.Get() == 0);
    third.Reset();
    third.Reset();
  }
  REQUIRE(released == std::vector<int>{7, 0});
}
TEST_CASE("Owned resources release during failed construction",
          "[studica][resource]") {
  int released = 0;
  try {
    UniqueResource<int> resource{42, [&](int handle) { released = handle; }};
    throw std::runtime_error("later device setup failed");
  } catch (const std::runtime_error&) {
  }
  REQUIRE(released == 42);
}
