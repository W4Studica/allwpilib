// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <memory>

#include "VMXPi.h"
#include "wpi/studica/SharedVMX.hpp"
#include "wpi/studica/drivers/Titan.hpp"

namespace wpi::studica::detail {

/// Sends enable/disable frames from the process, cancelling board-side
/// repeaters.
class TitanEnableSwitch {
 public:
  TitanEnableSwitch(uint8_t canId, std::shared_ptr<VMXPi> vmx = SharedVMX())
      : m_canId{canId}, m_vmx{std::move(vmx)} {}

  void Enable(bool enable) {
    // Frames a previous run left repeating on the board would keep the Titan
    // alive (or disabled) whatever we send now.
    StopRepeating(kEnabledFlag);
    StopRepeating(kDisabledFlag);
    m_enabled = enable;
    Send(enable ? kEnabledFlag : kDisabledFlag);
    if (!enable) {
      // A disable that is lost is the dangerous one: send it a few times.
      for (int i = 0; i < 2; ++i) {
        m_vmx->time.DelayMilliseconds(10);
        Send(kDisabledFlag);
      }
    }
  }

  /// False while the most recent frame could not be sent. A failing link is
  /// also reported on stderr, once when it starts failing and once when it
  /// recovers, so a lost disable frame is not silent.
  bool LastSendOk() const { return m_lastSendOk; }

  /// Sends the current state again (once). Call at least every 150 ms while the
  /// motors may run.
  void KeepAlive() { Send(m_enabled ? kEnabledFlag : kDisabledFlag); }

 private:
  static constexpr uint32_t kDisabledFlag = DISABLED_FLAG;
  static constexpr uint32_t kEnabledFlag = ENABLED_FLAG;

  VMXCANMessage Message(uint32_t addressBase) const {
    VMXCANMessage msg;
    msg.dataSize = 8;
    const uint8_t zeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    msg.setData(zeros, 8);
    msg.messageID = addressBase + m_canId;
    return msg;
  }

  void Send(uint32_t addressBase) {
    VMXCANMessage msg = Message(addressBase);
    VMXErrorCode err = 0;
    const bool ok =
        m_vmx->can.SendMessage(msg, VMXCAN_SEND_PERIOD_NO_REPEAT, &err);
    if (!ok && m_lastSendOk) {
      std::fprintf(stderr, "Titan %d: CAN send failed: %s\n", m_canId,
                   GetVMXErrorString(err));
    } else if (ok && !m_lastSendOk) {
      std::fprintf(stderr, "Titan %d: CAN send works again\n", m_canId);
    }
    m_lastSendOk = ok;
  }

  /// Stops the periodic retransmission of one of the Titan's enable frames.
  /// Nothing is sent.
  void StopRepeating(uint32_t addressBase) {
    VMXCANMessage msg = Message(addressBase);
    VMXErrorCode err;
    m_vmx->can.SendMessage(msg, VMXCAN_SEND_PERIOD_STOP_REPEATING, &err);
  }

  uint8_t m_canId;
  std::shared_ptr<VMXPi> m_vmx;
  bool m_enabled = false;
  std::atomic<bool> m_lastSendOk{true};
};

}  // namespace wpi::studica::detail
