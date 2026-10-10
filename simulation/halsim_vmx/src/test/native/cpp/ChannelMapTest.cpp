// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/vmx/ChannelMap.hpp"

#include <catch2/catch_test_macros.hpp>

using wpilibvmx::ChannelMap;

TEST_CASE("ChannelMap parses entries", "[halsim_vmx]") {
  auto map = ChannelMap::Parse("0:12,1:13, 7:4");
  REQUIRE(map.size() == 2);  // " 7:4" has a leading space and is skipped
  REQUIRE(map.Get(0) == 12);
  REQUIRE(map.Get(1) == 13);
  REQUIRE_FALSE(map.Get(7).has_value());
}

TEST_CASE("ChannelMap skips malformed entries", "[halsim_vmx]") {
  auto map = ChannelMap::Parse("a:1,2:b,3,4:5:6,,8:9");
  REQUIRE(map.size() == 1);
  REQUIRE(map.Get(8) == 9);
}

TEST_CASE("ChannelMap empty input is empty", "[halsim_vmx]") {
  REQUIRE(ChannelMap::Parse("").size() == 0);
  REQUIRE_FALSE(ChannelMap::Parse("").Get(0).has_value());
}
