// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <set>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "wpi/hal/simulation/NotifyListener.h"
#include "wpi/halsim/vmx/ChannelMap.hpp"
#include "wpi/halsim/vmx/VmxBackend.hpp"

namespace wpilibvmx {

/**
 * Connects the simulation HAL to VMX hardware. Currently digital I/O, analog inputs, quadrature
 * encoders and the IMU.
 *
 * Pin state (initialized, direction) is reconciled by a polling thread because
 * the sim HAL sets "initialized" before it sets the direction, so the mode
 * cannot be read reliably from the initialized callback. Output values are
 * forwarded immediately from the value callback; input values are polled.
 */
class HALSimVMX {
 public:
  HALSimVMX(std::unique_ptr<VmxBackend> backend, Config config);
  ~HALSimVMX();
  HALSimVMX(const HALSimVMX&) = delete;
  HALSimVMX& operator=(const HALSimVMX&) = delete;

  /// Registers sim callbacks and, if spawnPollThread, starts the poll thread.
  /// Tests pass false and call Poll() themselves for deterministic behavior.
  void Start(bool spawnPollThread = true);
  void Stop();

  /// One reconcile/poll pass. Public so tests can drive it without the thread.
  void Poll();

 private:
  struct DioPin {
    HALSimVMX* owner = nullptr;
    int channel = 0;
    int vmxChannel = -1;
    int valueCbKey = 0;
    // State currently applied to the backend.
    bool applied = false;
    bool appliedInput = false;
    bool claimFailedLogged = false;  // log a failed claim once, not on every poll
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
    // The Studica Encoder has no reset, so reset is an offset on the raw count.
    int32_t offset = 0;
    int32_t lastRaw = 0;
    bool haveLast = false;
    bool claimFailedLogged = false;
    std::chrono::steady_clock::time_point lastTime;
  };

  static void OnDioValue(const char* name, void* param,
                         const HAL_Value* value);
  void PollDio(DioPin& pin, bool ownedByEncoder);
  void PollAnalog(AnalogPin& pin);
  void PollEncoder(EncoderPin& pin);
  void PollImu();

  /// WPILib DIO channels used as encoder A/B by encoders that are mapped.
  std::set<int> EncoderOwnedDioChannels() const;

  std::unique_ptr<VmxBackend> m_backend;
  Config m_config;
  ChannelMaps& m_maps;
  bool m_imuApplied = false;
  bool m_imuClaimFailedLogged = false;
  std::vector<DioPin> m_dio;
  std::vector<AnalogPin> m_analog;
  std::vector<EncoderPin> m_encoders;
  std::mutex m_mutex;
  std::thread m_thread;
  std::atomic<bool> m_running{false};
};

}  // namespace wpilibvmx
