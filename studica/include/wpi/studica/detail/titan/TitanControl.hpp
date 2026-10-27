// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <utility>

#include "wpi/studica/detail/titan/ControlLoopWatchdog.hpp"
#include "wpi/studica/detail/titan/TitanEnableGuard.hpp"

namespace wpi::studica::detail {
/// Owns the enable policy; the SDK transport is injected so the real policy can
/// be tested without creating a VMXPi or sending CAN frames.
template <typename Switch>
class TitanControl {
 public:
  using EnabledSource = std::function<bool()>;
  TitanControl(Switch& transport, std::chrono::milliseconds controlTimeout,
               std::chrono::milliseconds dsTimeout, EnabledSource enabled,
               ControlLoopWatchdog::Now now = &ControlLoopWatchdog::Clock::now,
               bool startThread = true)
      : m_transport{transport},
        m_watchdog{controlTimeout, dsTimeout, std::move(now)},
        m_enabled{std::move(enabled)},
        m_guard{transport, [this] { return Requested(); }} {
    if (!m_enabled) {
      throw std::invalid_argument("Titan requires an enabled source");
    }
    if (startThread) {
      m_guard.Start();
    }
  }

  void FeedControlLoop() { m_watchdog.FeedControlLoop(); }
  void FeedDriverStation() { m_watchdog.FeedDriverStation(); }
  bool IsEnabled() const {
    return Requested() && m_guard.IsEnabled() && m_transport.LastSendOk();
  }
  bool CanCommand() {
    if (IsEnabled()) {
      return true;
    }
    // Immediately propagate a stale/disabled state instead of waiting for the
    // next background pass. This never feeds either watchdog heartbeat.
    if (!m_stopped) {
      m_guard.Poll();
    }
    return IsEnabled();
  }
  void Stop() {
    m_stopped = true;
    m_guard.Stop();
  }
  void Poll() { m_guard.Poll(); }

 private:
  bool Requested() const {
    try {
      return !m_stopped && m_enabled() && m_watchdog.IsFresh();
    } catch (...) {
      return false;
    }
  }
  Switch& m_transport;
  ControlLoopWatchdog m_watchdog;
  EnabledSource m_enabled;
  std::atomic<bool> m_stopped{false};
  // Destroyed first, while its source and transport still exist.
  TitanEnableGuard<Switch> m_guard;
};
}  // namespace wpi::studica::detail
