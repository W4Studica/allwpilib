// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "wpi/hal/simulation/NotifyListener.h"
#include "wpi/halsim/vmx/ChannelMap.hpp"
#include "wpi/halsim/vmx/VmxBackend.hpp"

namespace wpilibvmx {

/**
 * Connects the simulation HAL to VMX hardware. Currently only digital I/O.
 *
 * Pin state (initialized, direction) is reconciled by a polling thread because
 * the sim HAL sets "initialized" before it sets the direction, so the mode
 * cannot be read reliably from the initialized callback. Output values are
 * forwarded immediately from the value callback; input values are polled.
 */
class HALSimVMX {
 public:
  HALSimVMX(std::unique_ptr<VmxBackend> backend, ChannelMap dioMap);
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
  };

  static void OnDioValue(const char* name, void* param,
                         const HAL_Value* value);
  void PollDio(DioPin& pin);

  std::unique_ptr<VmxBackend> m_backend;
  ChannelMap m_dioMap;
  std::vector<DioPin> m_dio;
  std::mutex m_mutex;
  std::thread m_thread;
  std::atomic<bool> m_running{false};
};

}  // namespace wpilibvmx
