// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/ds_relay/DsRelayFrames.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string_view>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_session.hpp>
#include <catch2/catch_test_macros.hpp>

#include "wpi/hal/DriverStationTypes.h"
#include "wpi/hal/HAL.h"
#include "wpi/hal/simulation/DriverStationData.h"
#include "wpi/hal/simulation/RoboRioData.h"

extern "C" int HALSIM_InitExtension(void);

using namespace wpi::halsim::ds_relay;

namespace {
/// Builds the golden STATE vector from the fixed byte offsets.
std::array<uint8_t, kStateFrameSize> MakeGoldenStateVector() {
  std::array<uint8_t, kStateFrameSize> frame{};
  frame[kStateMagicOffset] = kStateMagic;
  frame[kStateVersionOffset] = kFrameVersion;
  frame[kStateFlagsOffset] = 0;               // not attached, disabled
  frame[kStateRobotModeOffset] = 0;           // unknown
  frame[kStateAllianceStationOffset] = 0;     // unknown
  frame[kStateMatchTimeOffset] = 0xFF;        // -1 = unknown
  frame[kStateMatchTimeOffset + 1] = 0xFF;
  // Joystick 0: only axis 0 available, raw axis 0 = 127, buttons bit 1.
  const size_t base = kStateJoysticksOffset;
  frame[base + kJoyAvailableAxesOffset] = 0x01;
  frame[base + kJoyRawAxesOffset] = 0x7F;
  frame[base + kJoyButtonsOffset] = 0x02;
  frame[base + kJoyAvailableButtonsOffset] = 0x02;
  return frame;
}
}  // namespace

static_assert(kStateFrameSize == 248);
static_assert(kStatusFrameSize == 16);

TEST_CASE("frame sizes are fixed", "[ds_relay]") {
  STATIC_REQUIRE(kStateFrameSize == 248);
  STATIC_REQUIRE(kStatusFrameSize == 16);
  STATIC_REQUIRE(kJoystickBlockSize == 40);
  STATIC_REQUIRE(kStateJoysticksOffset + kJoystickCount * kJoystickBlockSize ==
                 kStateFrameSize);
}

TEST_CASE("state frame golden vector decodes", "[ds_relay]") {
  auto golden = MakeGoldenStateVector();
  REQUIRE(golden.size() == 248);

  DsState state;
  REQUIRE(DecodeStateFrame(golden.data(), golden.size(), state));

  REQUIRE_FALSE(state.dsAttached);
  REQUIRE_FALSE(state.enabled);
  REQUIRE_FALSE(state.eStop);
  REQUIRE_FALSE(state.fmsAttached);
  REQUIRE_FALSE(state.redAlliance);
  REQUIRE(state.robotMode == 0);
  REQUIRE(state.allianceStation == 0);
  REQUIRE(state.matchTimeTenths == -1);

  REQUIRE(state.joysticks[0].availableAxes == 0x0001);
  REQUIRE(state.joysticks[0].rawAxes[0] == 127);
  for (int k = 1; k < kAxisCount; ++k) {
    REQUIRE(state.joysticks[0].rawAxes[k] == 0);
  }
  REQUIRE(state.joysticks[0].buttons == 0x2);
  REQUIRE(state.joysticks[0].availableButtons == 0x2);
  REQUIRE(state.joysticks[0].availablePovs == 0);
  for (int k = 0; k < kPovCount; ++k) {
    REQUIRE(state.joysticks[0].povs[k] == 0);
  }

  // Blocks 1..5 are all zero bytes.
  for (int i = 1; i < kJoystickCount; ++i) {
    REQUIRE(state.joysticks[i].availableAxes == 0);
    REQUIRE(state.joysticks[i].buttons == 0);
    REQUIRE(state.joysticks[i].availableButtons == 0);
    REQUIRE(state.joysticks[i].availablePovs == 0);
  }
}

TEST_CASE("apply state frame updates the simulation HAL", "[ds_relay]") {
  HALSIM_ResetDriverStationData();

  DsState state;
  state.dsAttached = true;
  state.enabled = true;
  state.eStop = false;
  state.fmsAttached = true;
  state.redAlliance = false;
  state.robotMode = 2;         // teleoperated
  state.allianceStation = 4;   // BLUE_1
  state.matchTimeTenths = 123; // 12.3 s

  state.joysticks[2].availableAxes = 0x0005;
  state.joysticks[2].rawAxes[0] = 127;
  state.joysticks[2].rawAxes[1] = -127;
  state.joysticks[2].rawAxes[2] = 64;
  state.joysticks[2].buttons = 0x5;
  state.joysticks[2].availableButtons = 0xFF;
  state.joysticks[2].availablePovs = 0x3;
  state.joysticks[2].povs[0] = 0x01;  // UP
  state.joysticks[2].povs[1] = 0x0A;  // RIGHT | LEFT

  ApplyStateFrame(state);

  REQUIRE(HALSIM_GetDriverStationDsAttached());
  REQUIRE(HALSIM_GetDriverStationEnabled());
  REQUIRE_FALSE(HALSIM_GetDriverStationEStop());
  REQUIRE(HALSIM_GetDriverStationFmsAttached());
  REQUIRE(HALSIM_GetDriverStationAllianceStationId() ==
          HAL_ALLIANCE_STATION_BLUE_1);
  REQUIRE(HALSIM_GetDriverStationRobotMode() == HAL_ROBOT_MODE_TELEOPERATED);
  REQUIRE(HALSIM_GetDriverStationMatchTime() == Catch::Approx(12.3));

  HAL_JoystickAxes axes{};
  HALSIM_GetJoystickAxes(2, &axes);
  REQUIRE(axes.available == 0x0005);
  REQUIRE(axes.raw[0] == 127);
  REQUIRE(axes.axes[0] == Catch::Approx(1.0f));
  REQUIRE(axes.raw[1] == -127);
  REQUIRE(axes.axes[1] == Catch::Approx(-1.0f));
  REQUIRE(axes.raw[2] == 64);
  REQUIRE(axes.axes[2] == Catch::Approx(64.0f / 127.0f));

  HAL_JoystickButtons buttons{};
  HALSIM_GetJoystickButtons(2, &buttons);
  REQUIRE(buttons.buttons == 0x5);
  REQUIRE(buttons.available == 0xFF);

  HAL_JoystickPOVs povs{};
  HALSIM_GetJoystickPOVs(2, &povs);
  REQUIRE(povs.available == 0x3);
  REQUIRE(static_cast<uint8_t>(povs.povs[0]) == 0x01);
  REQUIRE(static_cast<uint8_t>(povs.povs[1]) == 0x0A);

  // An untouched joystick stays reset.
  HAL_JoystickAxes zeroAxes{};
  HALSIM_GetJoystickAxes(0, &zeroAxes);
  REQUIRE(zeroAxes.available == 0);
}

TEST_CASE("status frame round-trips battery and count", "[ds_relay]") {
  HALSIM_SetRoboRioVInVoltage(12.34);

  std::array<uint8_t, kStatusFrameSize> buffer{};
  REQUIRE(EncodeStatusFrame(true,
                            static_cast<float>(HALSIM_GetRoboRioVInVoltage()),
                            7, buffer.data()) == kStatusFrameSize);

  REQUIRE(buffer[kStatusMagicOffset] == kStatusMagic);
  REQUIRE(buffer[kStatusVersionOffset] == kFrameVersion);
  REQUIRE(buffer[3] == 0);
  REQUIRE(buffer[12] == 0);
  REQUIRE(buffer[13] == 0);
  REQUIRE(buffer[14] == 0);
  REQUIRE(buffer[15] == 0);

  StatusFrame status;
  REQUIRE(DecodeStatusFrame(buffer.data(), buffer.size(), status));
  REQUIRE(status.robotProgramAlive);
  REQUIRE(status.batteryVolts == Catch::Approx(12.34f));
  REQUIRE(status.statusCount == 7);
}

TEST_CASE("invalid frames are ignored", "[ds_relay]") {
  DsState state;
  StatusFrame status;

  std::array<uint8_t, kStateFrameSize> frame{};
  frame[kStateMagicOffset] = kStateMagic;
  frame[kStateVersionOffset] = kFrameVersion;

  REQUIRE_FALSE(DecodeStateFrame(frame.data(), kStateFrameSize - 1, state));
  REQUIRE_FALSE(DecodeStateFrame(frame.data(), kStateFrameSize + 1, state));
  REQUIRE_FALSE(DecodeStateFrame(nullptr, kStateFrameSize, state));

  frame[kStateVersionOffset] = 9;
  REQUIRE_FALSE(DecodeStateFrame(frame.data(), kStateFrameSize, state));

  frame[kStateVersionOffset] = kFrameVersion;
  frame[kStateMagicOffset] = 0x00;
  REQUIRE_FALSE(DecodeStateFrame(frame.data(), kStateFrameSize, state));

  std::array<uint8_t, kStatusFrameSize> statusFrame{};
  statusFrame[kStatusMagicOffset] = kStatusMagic;
  statusFrame[kStatusVersionOffset] = kFrameVersion;
  REQUIRE_FALSE(DecodeStatusFrame(statusFrame.data(), 8, status));
  statusFrame[kStatusMagicOffset] = 0x00;
  REQUIRE_FALSE(DecodeStatusFrame(statusFrame.data(), kStatusFrameSize, status));
}

TEST_CASE("socket round trip relays state and returns status", "[ds_relay]") {
  constexpr int kTestPort = 24631;
  ::setenv(kPortEnvVar, "24631", 1);
  REQUIRE(GetRelayPort() == kTestPort);

  HALSIM_SetRoboRioVInVoltage(11.5);
  REQUIRE(HALSIM_InitExtension() == 0);

  // State frame: attached, enabled, teleoperated, RED_1.
  std::array<uint8_t, kStateFrameSize> frame{};
  frame[kStateMagicOffset] = kStateMagic;
  frame[kStateVersionOffset] = kFrameVersion;
  frame[kStateFlagsOffset] = kStateFlagDsAttached | kStateFlagEnabled;
  frame[kStateRobotModeOffset] = 2;
  frame[kStateAllianceStationOffset] = 1;

  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  REQUIRE(fd >= 0);
  timeval timeout{};
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  sockaddr_in relay{};
  relay.sin_family = AF_INET;
  relay.sin_port = htons(static_cast<uint16_t>(kTestPort));
  relay.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

  std::array<uint8_t, kStatusFrameSize> statusBuffer{};
  ssize_t received = -1;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    REQUIRE(::sendto(fd, frame.data(), frame.size(), 0,
                     reinterpret_cast<sockaddr*>(&relay),
                     sizeof(relay)) == static_cast<ssize_t>(frame.size()));
    received = ::recvfrom(fd, statusBuffer.data(), statusBuffer.size(), 0,
                          nullptr, nullptr);
    if (received == static_cast<ssize_t>(kStatusFrameSize)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ::close(fd);

  REQUIRE(received == static_cast<ssize_t>(kStatusFrameSize));
  REQUIRE(statusBuffer[kStatusMagicOffset] == kStatusMagic);
  REQUIRE(statusBuffer[kStatusVersionOffset] == kFrameVersion);

  StatusFrame status;
  REQUIRE(DecodeStatusFrame(statusBuffer.data(), statusBuffer.size(), status));
  REQUIRE(status.robotProgramAlive);
  REQUIRE(status.batteryVolts == Catch::Approx(11.5f));

  // The relay applied the received state to the simulation HAL.
  REQUIRE(HALSIM_GetDriverStationEnabled());
  REQUIRE(HALSIM_GetDriverStationDsAttached());
  REQUIRE(HALSIM_GetDriverStationRobotMode() == HAL_ROBOT_MODE_TELEOPERATED);
  REQUIRE(HALSIM_GetDriverStationAllianceStationId() ==
          HAL_ALLIANCE_STATION_RED_1);
}

int main(int argc, char** argv) {
  bool listOnly = false;
  for (int i = 1; i < argc; ++i) {
    std::string_view arg{argv[i]};
    if (arg == "--list-tests" || arg == "--list-tags" ||
        arg == "--list-reporters" || arg == "--list-listeners") {
      listOnly = true;
    }
  }
  if (!listOnly) {
    HAL_Initialize();
  }
  return Catch::Session().run(argc, argv);
}
