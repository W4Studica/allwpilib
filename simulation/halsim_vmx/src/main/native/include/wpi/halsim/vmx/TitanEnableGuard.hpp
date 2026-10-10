// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

#include "wpi/hal/DriverStation.h"

namespace wpilibvmx {

/**
 * Ties a Titan's enable state to the robot's enable state.
 *
 * A Titan powers up disabled and ignores motor commands until Enable(true); it stops by
 * itself if it hears nothing for 200 ms (see studica_drivers/titan.hpp). This guard
 * calls Enable(true) when the robot becomes enabled and Enable(false) when it becomes
 * disabled or E-stopped, and always Enable(false) when it is stopped or destroyed, so a
 * crashed or finished program leaves the motors off.
 *
 * It does NOT send motor commands. Robot code still has to send SetSpeed or
 * SetTargetVelocity at least every ~150 ms while enabled (TITAN_CAN_KEEPALIVE_MS).
 *
 * The robot state is read from the HAL control word, so it follows whatever drives the
 * (simulated) Driver Station, for example the team's own MockDS. In the simulation HAL the
 * control word is only non-zero while the DS is marked attached, so such a MockDS must set
 * DsAttached as well as Enabled (DriverStationSim.setDsAttached(true)).
 *
 * TitanT only needs `void Enable(bool)`, so this is a template and can be tested without
 * hardware. The Titan must outlive the guard: declare the guard after the Titan.
 */
template <class TitanT>
class TitanEnableGuard {
 public:
  using EnabledSource = std::function<bool()>;

  /// Enabled and not E-stopped, from the latest HAL control word. A failed read counts
  /// as disabled.
  static bool HalEnabled() {
    HAL_ControlWord word;
    if (HAL_GetUncachedControlWord(&word) != 0) {
      return false;
    }
    return HAL_ControlWord_IsEnabled(word) && !HAL_ControlWord_IsEStopped(word);
  }

  explicit TitanEnableGuard(
      TitanT& titan, EnabledSource source = &TitanEnableGuard::HalEnabled,
      std::chrono::milliseconds period = std::chrono::milliseconds(20))
      : m_titan{titan}, m_source{std::move(source)}, m_period{period} {}

  TitanEnableGuard(const TitanEnableGuard&) = delete;
  TitanEnableGuard& operator=(const TitanEnableGuard&) = delete;

  ~TitanEnableGuard() { Stop(); }

  /// Starts the polling thread. Does nothing if already started.
  void Start() {
    if (m_running.exchange(true)) {
      return;
    }
    m_thread = std::thread{[this] {
      while (m_running) {
        Poll();
        std::this_thread::sleep_for(m_period);
      }
    }};
  }

  /// Stops the thread and disables the Titan.
  void Stop() {
    if (m_running.exchange(false) && m_thread.joinable()) {
      m_thread.join();
    }
    // Always sent, even if the last state was already disabled: it costs one CAN frame and
    // the last word to the Titan should never depend on what we think it was told before.
    std::scoped_lock lock{m_mutex};
    m_titan.Enable(false);
    m_haveState = true;
    m_state = false;
  }

  /// One pass: sends Enable() only when the state changes (the first pass always sends).
  /// Public so tests can drive it without the thread.
  void Poll() {
    bool enabled = m_source();
    std::scoped_lock lock{m_mutex};
    if (!m_haveState || enabled != m_state) {
      m_titan.Enable(enabled);
      m_state = enabled;
      m_haveState = true;
    }
  }

 private:
  TitanT& m_titan;
  EnabledSource m_source;
  std::chrono::milliseconds m_period;
  std::mutex m_mutex;
  bool m_haveState = false;
  bool m_state = false;
  std::atomic<bool> m_running{false};
  std::thread m_thread;
};

}  // namespace wpilibvmx
