// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/ds_relay/DsRelayFrames.hpp"

#include <cstdlib>
#include <cstring>

#include "wpi/hal/DriverStationTypes.h"
#include "wpi/hal/Types.h"
#include "wpi/hal/simulation/DriverStationData.h"

namespace wpi::halsim::ds_relay {

// The joystick block layout must exactly cover the on-wire block.
static_assert(kJoyPovsOffset + kPovCount == kJoystickBlockSize);
static_assert(kStateJoysticksOffset + kJoystickCount * kJoystickBlockSize ==
              kStateFrameSize);

namespace {

uint16_t ReadU16(const uint8_t* p) noexcept {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}

uint32_t ReadU32(const uint8_t* p) noexcept {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

uint64_t ReadU64(const uint8_t* p) noexcept {
  uint64_t v = 0;
  for (int i = 7; i >= 0; --i) {
    v = (v << 8) | static_cast<uint64_t>(p[i]);
  }
  return v;
}

void WriteU32(uint8_t* p, uint32_t v) noexcept {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

}  // namespace

int GetRelayPort() {
  const char* env = std::getenv(kPortEnvVar);
  if (env != nullptr && *env != '\0') {
    char* end = nullptr;
    long value = std::strtol(env, &end, 10);
    if (end != env && *end == '\0' && value > 0 && value <= 65535) {
      return static_cast<int>(value);
    }
  }
  return static_cast<int>(kDefaultPort);
}

bool DecodeStateFrame(const uint8_t* data, size_t len, DsState& out) noexcept {
  if (data == nullptr || len != kStateFrameSize) {
    return false;
  }
  if (data[kStateMagicOffset] != kStateMagic ||
      data[kStateVersionOffset] != kFrameVersion) {
    return false;
  }

  const uint8_t flags = data[kStateFlagsOffset];
  out.dsAttached = (flags & kStateFlagDsAttached) != 0;
  out.enabled = (flags & kStateFlagEnabled) != 0;
  out.eStop = (flags & kStateFlagEStop) != 0;
  out.fmsAttached = (flags & kStateFlagFmsAttached) != 0;
  out.redAlliance = (flags & kStateFlagRedAlliance) != 0;
  out.robotMode = data[kStateRobotModeOffset];
  out.allianceStation = data[kStateAllianceStationOffset];
  out.matchTimeTenths =
      static_cast<int16_t>(ReadU16(data + kStateMatchTimeOffset));

  for (int i = 0; i < kJoystickCount; ++i) {
    const uint8_t* base = data + kStateJoysticksOffset + i * kJoystickBlockSize;
    JoystickState& joy = out.joysticks[i];
    joy.availableAxes = ReadU16(base + kJoyAvailableAxesOffset);
    for (int k = 0; k < kAxisCount; ++k) {
      joy.rawAxes[k] = static_cast<int8_t>(base[kJoyRawAxesOffset + k]);
    }
    joy.buttons = ReadU64(base + kJoyButtonsOffset);
    joy.availableButtons = ReadU64(base + kJoyAvailableButtonsOffset);
    joy.availablePovs = ReadU16(base + kJoyAvailablePovsOffset);
    for (int k = 0; k < kPovCount; ++k) {
      joy.povs[k] = base[kJoyPovsOffset + k];
    }
  }
  return true;
}

bool DecodeStatusFrame(const uint8_t* data, size_t len,
                       StatusFrame& out) noexcept {
  if (data == nullptr || len != kStatusFrameSize) {
    return false;
  }
  if (data[kStatusMagicOffset] != kStatusMagic ||
      data[kStatusVersionOffset] != kFrameVersion) {
    return false;
  }

  out.robotProgramAlive =
      (data[kStatusFlagsOffset] & kStatusFlagRobotProgramAlive) != 0;
  const uint32_t bits = ReadU32(data + kStatusBatteryOffset);
  std::memcpy(&out.batteryVolts, &bits, sizeof(out.batteryVolts));
  out.statusCount = ReadU32(data + kStatusCountOffset);
  return true;
}

void ApplyStateFrame(const DsState& state) noexcept {
  HALSIM_SetDriverStationDsAttached(state.dsAttached);
  HALSIM_SetDriverStationEnabled(state.enabled);
  HALSIM_SetDriverStationEStop(state.eStop);
  HALSIM_SetDriverStationFmsAttached(state.fmsAttached);
  HALSIM_SetDriverStationAllianceStationId(
      static_cast<HAL_AllianceStationID>(state.allianceStation));
  HALSIM_SetDriverStationRobotMode(
      static_cast<HAL_RobotMode>(state.robotMode));
  if (state.matchTimeTenths >= 0) {
    HALSIM_SetDriverStationMatchTime(state.matchTimeTenths / 10.0);
  }

  for (int i = 0; i < kJoystickCount; ++i) {
    const JoystickState& joy = state.joysticks[i];

    HAL_JoystickAxes axes{};
    axes.available = joy.availableAxes;
    for (int k = 0; k < kAxisCount; ++k) {
      axes.raw[k] = joy.rawAxes[k];
      axes.axes[k] = joy.rawAxes[k] / 127.0f;
    }
    HALSIM_SetJoystickAxes(i, &axes);

    HAL_JoystickButtons buttons{};
    buttons.buttons = joy.buttons;
    buttons.available = joy.availableButtons;
    HALSIM_SetJoystickButtons(i, &buttons);

    HAL_JoystickPOVs povs{};
    povs.available = static_cast<uint8_t>(joy.availablePovs);
    for (int k = 0; k < kPovCount; ++k) {
      povs.povs[k] = static_cast<HAL_JoystickPOV>(joy.povs[k]);
    }
    HALSIM_SetJoystickPOVs(i, &povs);
  }

  HALSIM_NotifyDriverStationNewData();
}

void ApplyWatchdogDisconnected() noexcept {
  HALSIM_SetDriverStationEnabled(false);
  HALSIM_SetDriverStationDsAttached(false);
  HALSIM_SetDriverStationRobotMode(HAL_ROBOT_MODE_UNKNOWN);
  HALSIM_NotifyDriverStationNewData();
}

size_t EncodeStatusFrame(bool robotProgramAlive, float batteryVolts,
                         uint32_t statusCount, uint8_t* out) noexcept {
  if (out == nullptr) {
    return 0;
  }
  std::memset(out, 0, kStatusFrameSize);
  out[kStatusMagicOffset] = kStatusMagic;
  out[kStatusVersionOffset] = kFrameVersion;
  out[kStatusFlagsOffset] =
      robotProgramAlive ? kStatusFlagRobotProgramAlive : 0;
  uint32_t bits = 0;
  std::memcpy(&bits, &batteryVolts, sizeof(bits));
  WriteU32(out + kStatusBatteryOffset, bits);
  WriteU32(out + kStatusCountOffset, statusCount);
  return kStatusFrameSize;
}

}  // namespace wpi::halsim::ds_relay
