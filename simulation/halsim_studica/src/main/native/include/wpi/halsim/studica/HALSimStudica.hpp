// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <vector>

#include "wpi/hal/simulation/NotifyListener.h"
#include "wpi/halsim/studica/ChannelMap.hpp"
#include "wpi/studica/detail/backend/HardwareBackend.hpp"
#include "wpi/studica/detail/conversion/DeviceConversions.hpp"

namespace wpi::studica {

/**
 * Connects the simulation HAL to VMX hardware. Currently digital I/O, analog
 * inputs, quadrature encoders, PWM outputs, I2C, duty-cycle capture, digital
 * edges, the IMU and battery voltage.
 *
 * Pin state (initialized, direction) is reconciled by a polling thread because
 * the sim HAL sets "initialized" before it sets the direction, so the mode
 * cannot be read reliably from the initialized callback. Output values are
 * forwarded immediately from the value callback; input values are polled.
 */
class HALSimStudica {
 public:
  HALSimStudica(std::unique_ptr<HardwareBackend> backend, Config config);
  ~HALSimStudica();
  HALSimStudica(const HALSimStudica&) = delete;
  HALSimStudica& operator=(const HALSimStudica&) = delete;

  /// Registers sim callbacks and, if spawnPollThread, starts the poll thread.
  /// Tests pass false and call Poll() themselves for deterministic behavior.
  bool Start(bool spawnPollThread = true);
  void Stop();

  /// One reconcile/poll pass. Public so tests can drive it without the thread.
  void Poll();

 private:
  struct DioPin {
    HALSimStudica* owner = nullptr;
    int channel = 0;
    int vmxChannel = -1;
    int valueCbKey = 0;
    // State currently applied to the backend.
    bool applied = false;
    bool appliedInput = false;
    std::optional<bool> outputValue;
    bool interruptApplied = false;
    bool claimFailedLogged =
        false;  // log a failed claim once, not on every poll
  };

  struct AnalogPin {
    int channel = 0;
    int vmxChannel = -1;
    bool applied = false;
    bool claimFailedLogged = false;
  };

  struct EncoderPin {
    int index = 0;
    // Channels currently applied to the backend (VMX numbering).
    int vmxA = -1;
    int vmxB = -1;
    bool applied = false;
    EncoderState state;
    bool claimFailedLogged = false;
  };

  struct PwmPin {
    int channel = 0;
    int vmxChannel = -1;
    bool applied = false;
    int appliedPeriodMicros = 0;
    int appliedPulseMicros = -1;
    bool claimFailedLogged = false;
  };

  struct DutyCyclePin {
    int channel = 0;
    int vmxChannel = -1;
    bool applied = false;
    bool claimFailedLogged = false;
  };

  static int32_t OnI2cTransaction(void* param, int32_t address,
                                  const uint8_t* tx, int32_t txSize,
                                  uint8_t* rx, int32_t rxSize);
  void PollI2c();
  void PollDutyCycle(DutyCyclePin& pin);
  bool PollInterrupt(DioPin& pin, bool ownedByEncoder);
  static void OnDioValue(const char* name, void* param, const HAL_Value* value);

  void PollDio(DioPin& pin, bool ownedByEncoder);
  void PollAnalog(AnalogPin& pin);
  void PollEncoder(EncoderPin& pin);
  void PollImu();
  void PollPwm(PwmPin& pin);
  void PollBattery();

  /// WPILib DIO channels used as encoder A/B by encoders that are mapped.
  std::set<int> EncoderOwnedDioChannels() const;

  std::unique_ptr<HardwareBackend> m_backend;
  Config m_config;
  ChannelMaps& m_maps;
  bool m_imuApplied = false;
  bool m_imuClaimFailedLogged = false;
  std::vector<DioPin> m_dio;
  std::vector<AnalogPin> m_analog;
  std::vector<EncoderPin> m_encoders;
  std::vector<PwmPin> m_pwm;
  std::vector<DutyCyclePin> m_dutyCycles;
  int m_i2cCallback = 0;
  bool m_i2cApplied = false;
  int m_batteryPolls = 0;
  bool m_batteryFailedLogged = false;
  std::mutex m_mutex;
  std::mutex m_i2cMutex;  // Acquire before m_mutex when both are needed.
  std::thread m_thread;
  std::atomic<bool> m_running{false};
};

}  // namespace wpi::studica
