// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "wpi/hal/DriverStation.h"

namespace wpi::studica {
/// One Titan device: owns the original driver, shared VMX connection, enable
/// keepalive and freshness checks. Disabled until both heartbeats are fed.
class Titan {
 public:
  struct Config {
    uint16_t motorFrequencyHz = 15600;
    float distancePerTick = 1.0f;
    std::chrono::milliseconds controlTimeout{100};
    std::chrono::milliseconds driverStationTimeout{150};
  };
  using EnabledSource = std::function<bool()>;

  /// The default source is compiled into the robot program, so the hardware
  /// library never links a second release/debug HAL into the process.
  static bool HalEnabled() {
    HAL_ControlWord word;
    return HAL_GetUncachedControlWord(&word) == 0 &&
           HAL_ControlWord_IsEnabled(word) && !HAL_ControlWord_IsEStopped(word);
  }

  explicit Titan(uint8_t canId, EnabledSource enabled = &Titan::HalEnabled);
  Titan(uint8_t canId, Config config,
        EnabledSource enabled = &Titan::HalEnabled);
  Titan(uint8_t canId, uint16_t motorFrequencyHz, float distancePerTick,
        EnabledSource enabled = &Titan::HalEnabled);
  ~Titan();
  Titan(const Titan&) = delete;
  Titan& operator=(const Titan&) = delete;

  /// Feed only after a successfully completed control cycle.
  void FeedControlLoop();
  /// Feed only after a new valid Driver Station packet/state update.
  void FeedDriverStation();
  bool IsEnabled() const;
  /// Whether the most recent enable/disable CAN frame was enqueued
  /// successfully.
  bool LastSendOk() const;
  /// Permanently stops this object's enable worker and disables its device.
  void Stop();

  void SetupEncoder(uint8_t encoder);
  /// Configured CAN ID; does not issue a firmware query.
  uint8_t GetID();
  uint16_t GetFrequency();
  std::string GetFirmwareVersion();
  std::string GetHardwareVersion();
  float GetControllerTemp();
  bool GetLimitSwitch(uint8_t motor, uint8_t direction);
  bool GetLimitSwitchesFresh(bool fwd[4], bool rev[4], bool& is_fresh,
                             uint64_t* out_timestamp_us = nullptr);
  float GetRPM(uint8_t motor);
  bool TryGetRPM(uint8_t motor, float* out_rpm);
  bool GetRPMFresh(uint8_t motor, float& rpm, bool& is_fresh,
                   uint64_t* out_timestamp_us = nullptr);
  std::string GetSerialNumber();
  double GetEncoderDistance(uint8_t motor);
  int32_t GetEncoderCount(uint8_t motor);
  bool GetEncoderDistanceFresh(uint8_t motor, double& distance, bool& is_fresh,
                               uint64_t* out_timestamp_us = nullptr);
  void ConfigureEncoder(uint8_t motor, double cfg);
  void ResetEncoder(uint8_t motor);
  double GetCypherAngle(uint8_t port);
  bool GetCypherAnglesFresh(double angles[4], bool& is_fresh,
                            uint64_t* out_timestamp_us = nullptr);
  /// Signed output in [-1, 1]. Motion commands require enabled and fresh state.
  void SetSpeed(uint8_t motor, double speedCfg);
  /// Applies the same signed output to all motors, respecting their inversion.
  void SetSpeedAll(double duty);
  void InvertMotorDirection(uint8_t motor);
  void InvertMotorRPM(uint8_t motor);
  void InvertEncoderDirection(uint8_t motor);
  void InvertMotor(uint8_t motor);
  void SetTargetVelocity(uint8_t motor, float velocityRpm);
  float GetTargetRPM(uint8_t motor);
  bool TryGetTargetRPM(uint8_t motor, float* out_rpm);
  bool TryGetTargetRPMFromAll(float targetRpm[4]);
  void SetTargetDistance(uint8_t motor, int32_t distanceCounts);
  void SetTargetAngle(uint8_t motor, double angleDeg);
  void SetPositionHold(uint8_t motor, bool hold);
  void SetEncoderResolution(uint8_t channel, uint16_t cpr);
  void SetCurrentLimit(uint8_t channel, float limitAmps);
  void SetCurrentLimitMode(uint8_t channel, uint8_t mode);
  void SetMotorStopMode(uint8_t mode);
  void SetPIDType(uint8_t type);
  void SetMotorPIDType(uint8_t motor, uint8_t type);
  void AutotuneAll();
  void AutotuneMotor(uint8_t motor);
  void SetSensitivity(uint8_t motor, uint8_t sensitivity);
  void SetCANSensorOsDelay(uint16_t periodMs);
  void DisableMotor(uint8_t motor);
  void SetAngleLimits(uint8_t motor, double minDeg, double maxDeg);
  void GetAngleLimits(uint8_t motor, double& minDeg, double& maxDeg) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};
}  // namespace wpi::studica
