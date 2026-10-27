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

namespace wpi::studica::detail {

/// Internal enable polling and shutdown guard. The transport must outlive it.
template <class TitanT>
class TitanEnableGuard {
 public:
  using EnabledSource = std::function<bool()>;

  /// Enabled and not E-stopped, from the latest HAL control word. A failed read
  /// counts as disabled.
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
    // Always sent, even if the last state was already disabled: it costs one
    // CAN frame and the last word to the Titan should never depend on what we
    // think it was told before.
    std::scoped_lock lock{m_mutex};
    m_titan.Enable(false);
    m_haveState = true;
    m_state = false;
  }

  /// One pass: sends Enable() only when the state changes (the first pass
  /// always sends). Public so tests can drive it without the thread.
  void Poll() {
    bool enabled = m_source();
    std::scoped_lock lock{m_mutex};
    if (!m_haveState || enabled != m_state) {
      m_titan.Enable(enabled);
      m_state = enabled;
      m_haveState = true;
    } else if constexpr (requires { m_titan.KeepAlive(); }) {
      // Repeat the state on every pass, from this process. The Titan stops by
      // itself when it hears nothing for 200 ms, but only if nothing repeats
      // the enable for it: studica_driver::Titan::Enable() hands the repetition
      // to the VMX board, which keeps sending after the program has died
      // (MEASURED: the motor kept turning after kill -9).
      m_titan.KeepAlive();
    }
  }

  bool IsEnabled() const {
    std::scoped_lock lock{m_mutex};
    return m_haveState && m_state;
  }

 private:
  TitanT& m_titan;
  EnabledSource m_source;
  std::chrono::milliseconds m_period;
  mutable std::mutex m_mutex;
  bool m_haveState = false;
  bool m_state = false;
  std::atomic<bool> m_running{false};
  std::thread m_thread;
};

}  // namespace wpi::studica::detail
