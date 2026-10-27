// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <array>
#include <cstddef>
#include <limits>
#include <mutex>

#include "wpi/studica/detail/HardwareBackend.hpp"

namespace wpi::studica::detail {
/// Bounded queue: SDK callbacks do not allocate or invoke application code.
class InterruptQueue {
 public:
  void Push(InterruptEvent event) {
    std::scoped_lock lock{m_mutex};
    if (m_size == m_events.size()) {
      m_head = (m_head + 1) % m_events.size();
      --m_size;
      if (m_dropped != std::numeric_limits<uint32_t>::max()) {
        ++m_dropped;
      }
    }
    m_events[(m_head + m_size++) % m_events.size()] = event;
  }
  bool Pop(InterruptEvent* event) {
    std::scoped_lock lock{m_mutex};
    if (!m_size) {
      return false;
    }
    *event = m_events[m_head];
    event->dropped = m_dropped;
    m_dropped = 0;
    m_head = (m_head + 1) % m_events.size();
    --m_size;
    return true;
  }

 private:
  std::mutex m_mutex;
  std::array<InterruptEvent, 256> m_events{};
  std::size_t m_head = 0, m_size = 0;
  uint32_t m_dropped = 0;
};
}  // namespace wpi::studica::detail
