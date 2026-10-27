// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace wpi::studica::detail {

/**
 * Two independent freshness checks for TitanEnableGuard's EnabledSource.
 * FeedControlLoop() belongs in the successfully completed robot control cycle;
 * FeedDriverStation() belongs after receiving a new valid DS packet. Never feed
 * either one from the guard thread or just because a cached HAL value is
 * enabled. This helper does not receive DS packets or change the simulation
 * DriverStation.
 */
class ControlLoopWatchdog {
 public:
  using Clock = std::chrono::steady_clock;
  using Now = std::function<Clock::time_point()>;

  ControlLoopWatchdog(std::chrono::milliseconds controlTimeout,
                      std::chrono::milliseconds dsTimeout,
                      Now now = &Clock::now)
      : m_controlTimeout{controlTimeout},
        m_dsTimeout{dsTimeout},
        m_now{std::move(now)} {
    if (controlTimeout.count() <= 0 || dsTimeout.count() <= 0) {
      throw std::invalid_argument("watchdog timeouts must be positive");
    }
  }

  void FeedControlLoop() {
    std::scoped_lock lock{m_mutex};
    m_control = m_now();
  }

  void FeedDriverStation() {
    std::scoped_lock lock{m_mutex};
    m_ds = m_now();
  }

  /// Disabled until both sources have arrived, and at either timeout boundary.
  bool IsFresh() const {
    std::scoped_lock lock{m_mutex};
    const auto now = m_now();
    return m_control && m_ds && now >= *m_control && now >= *m_ds &&
           now - *m_control < m_controlTimeout && now - *m_ds < m_dsTimeout;
  }

 private:
  std::chrono::milliseconds m_controlTimeout;
  std::chrono::milliseconds m_dsTimeout;
  Now m_now;
  mutable std::mutex m_mutex;
  std::optional<Clock::time_point> m_control;
  std::optional<Clock::time_point> m_ds;
};

}  // namespace wpi::studica::detail
