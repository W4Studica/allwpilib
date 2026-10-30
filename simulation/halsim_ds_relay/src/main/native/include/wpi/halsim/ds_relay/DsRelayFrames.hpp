// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstddef>
#include <cstdint>

/**
 * @defgroup halsim_ds_relay DS Relay Extension
 * @ingroup halsim_extensions
 *
 * Private loopback protocol spoken between Robot-Manager's `mrccomm` daemon
 * and this extension.
 *
 * mrccomm owns the real Driver Station sockets (UDP 1110/1150) and forwards
 * Driver Station state to the robot program process over loopback UDP. This
 * extension receives those STATE frames, applies them to the simulation HAL,
 * and answers with a STATUS frame carrying battery voltage and a liveness
 * flag.
 *
 * Both frames are little-endian and fixed size with no padding.
 * @{
 */

namespace wpi::halsim::ds_relay {

/// STATE frame magic byte (mrccomm -> relay).
inline constexpr uint8_t kStateMagic = 0x57;
/// STATUS frame magic byte (relay -> mrccomm).
inline constexpr uint8_t kStatusMagic = 0x53;
/// Protocol version carried by both frames.
inline constexpr uint8_t kFrameVersion = 1;

/// Exact on-wire size of a STATE frame.
inline constexpr size_t kStateFrameSize = 248;
/// Exact on-wire size of a STATUS frame.
inline constexpr size_t kStatusFrameSize = 16;

/// Number of joystick blocks in a STATE frame.
inline constexpr int kJoystickCount = 6;
/// Number of axes in each joystick block.
inline constexpr int kAxisCount = 12;
/// Number of POVs in each joystick block.
inline constexpr int kPovCount = 8;

/// Default UDP port the relay binds on loopback.
inline constexpr uint16_t kDefaultPort = 1742;
/// Environment variable overriding the relay port.
inline constexpr const char* kPortEnvVar = "W4STUDICA_DS_RELAY_PORT";

// --- STATE frame offsets -----------------------------------------------------
inline constexpr size_t kStateMagicOffset = 0;
inline constexpr size_t kStateVersionOffset = 1;
inline constexpr size_t kStateFlagsOffset = 2;
inline constexpr size_t kStateRobotModeOffset = 3;
inline constexpr size_t kStateAllianceStationOffset = 4;
inline constexpr size_t kStateMatchTimeOffset = 6;
inline constexpr size_t kStateJoysticksOffset = 8;

// --- STATE flags bits --------------------------------------------------------
inline constexpr uint8_t kStateFlagDsAttached = 1u << 0;
inline constexpr uint8_t kStateFlagEnabled = 1u << 1;
inline constexpr uint8_t kStateFlagEStop = 1u << 2;
inline constexpr uint8_t kStateFlagFmsAttached = 1u << 3;
inline constexpr uint8_t kStateFlagRedAlliance = 1u << 4;

// --- joystick block size and offsets (relative to the block start) -----------
inline constexpr size_t kJoystickBlockSize = 40;
inline constexpr size_t kJoyAvailableAxesOffset = 0;
inline constexpr size_t kJoyRawAxesOffset = 2;
inline constexpr size_t kJoyButtonsOffset = 14;
inline constexpr size_t kJoyAvailableButtonsOffset = 22;
inline constexpr size_t kJoyAvailablePovsOffset = 30;
inline constexpr size_t kJoyPovsOffset = 32;

// --- STATUS frame offsets ----------------------------------------------------
inline constexpr size_t kStatusMagicOffset = 0;
inline constexpr size_t kStatusVersionOffset = 1;
inline constexpr size_t kStatusFlagsOffset = 2;
inline constexpr size_t kStatusBatteryOffset = 4;
inline constexpr size_t kStatusCountOffset = 8;

// --- STATUS flags bits -------------------------------------------------------
inline constexpr uint8_t kStatusFlagRobotProgramAlive = 1u << 0;

/// Decoded joystick block.
struct JoystickState {
  uint16_t availableAxes = 0;
  int8_t rawAxes[kAxisCount] = {};
  uint64_t buttons = 0;
  uint64_t availableButtons = 0;
  uint16_t availablePovs = 0;
  uint8_t povs[kPovCount] = {};
};

/// Decoded STATE frame.
struct DsState {
  bool dsAttached = false;
  bool enabled = false;
  bool eStop = false;
  bool fmsAttached = false;
  bool redAlliance = false;
  uint8_t robotMode = 0;         ///< 0 unknown, 1 auto, 2 teleop, 3 utility
  uint8_t allianceStation = 0;   ///< 0 unknown, 1..3 RED, 4..6 BLUE
  int16_t matchTimeTenths = -1;  ///< -1 unknown
  JoystickState joysticks[kJoystickCount];
};

/// Decoded STATUS frame.
struct StatusFrame {
  bool robotProgramAlive = true;
  float batteryVolts = 0.0f;
  uint32_t statusCount = 0;
};

/**
 * Returns the UDP port the relay should bind, from the environment variable
 * W4STUDICA_DS_RELAY_PORT, falling back to kDefaultPort.
 */
int GetRelayPort();

/**
 * Decodes a STATE frame. Returns false (and leaves `out` unspecified if the
 * data is invalid) when the length, magic, or version do not match.
 */
bool DecodeStateFrame(const uint8_t* data, size_t len, DsState& out) noexcept;

/**
 * Decodes a STATUS frame. Returns false when the length, magic, or version do
 * not match.
 */
bool DecodeStatusFrame(const uint8_t* data, size_t len,
                       StatusFrame& out) noexcept;

/**
 * Applies a decoded STATE frame to the simulation HAL and notifies the Driver
 * Station data listeners.
 */
void ApplyStateFrame(const DsState& state) noexcept;

/**
 * Applies the defensive watchdog state: disabled and not attached. Called once
 * when valid STATE frames stop arriving.
 */
void ApplyWatchdogDisconnected() noexcept;

/**
 * Encodes a STATUS frame into `out` (which must be at least
 * kStatusFrameSize bytes). Returns the number of bytes written.
 */
size_t EncodeStatusFrame(bool robotProgramAlive, float batteryVolts,
                         uint32_t statusCount, uint8_t* out) noexcept;

}  // namespace wpi::halsim::ds_relay

/** @} */
