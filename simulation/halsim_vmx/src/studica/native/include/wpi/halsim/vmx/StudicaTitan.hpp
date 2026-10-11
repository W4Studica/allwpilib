// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdint>
#include <memory>

#include "VMXPi.h"
#include "titan.hpp"
#include "wpi/halsim/vmx/SharedVMX.hpp"

namespace wpilibvmx {

/**
 * The enable switch of a Titan for TitanEnableGuard, without the periodic CAN frames of studica_driver::Titan::Enable().
 *
 * Titan::Enable(true) sends ENABLED_FLAG as a periodic frame every 100 ms, Titan::Enable(false) sends DISABLED_FLAG every
 * 10 ms, and the VMX board keeps repeating a periodic frame until it is told to stop. MEASURED on a VMX-pi, two problems:
 *  - after Enable(false) the Titan hears DISABLED_FLAG ten times as often as ENABLED_FLAG and can never be enabled again
 *    (the LED flickered, the motor never ran);
 *  - the board keeps sending ENABLED_FLAG after the program died (kill -9): the Titan never sees the 200 ms of silence it
 *    stops on and the motor kept turning.
 *
 * This class sends single frames. Enable() stops any periodic frame an earlier run left on the board, then sends the new
 * state once; KeepAlive() sends it again, which TitanEnableGuard does on every poll (20 ms). When the program stops, so do
 * the frames, and the Titan stops by itself after 200 ms. studica_drivers is not modified: these are plain VMX CAN calls
 * on the shared VMXPi.
 *
 *   studica_driver::Titan titan(42, 20000, 0.0f, wpilibvmx::SharedVMX());
 *   wpilibvmx::StudicaTitan switcher{titan, 42};
 *   wpilibvmx::TitanEnableGuard<wpilibvmx::StudicaTitan> guard{switcher};
 */
class StudicaTitan {
 public:
  /// The Titan reference is kept for symmetry with the other Studica classes; the frames go straight to the VMXPi.
  StudicaTitan(studica_driver::Titan& /*titan*/, uint8_t canId, std::shared_ptr<VMXPi> vmx = SharedVMX())
      : m_canId{canId}, m_vmx{std::move(vmx)} {}

  void Enable(bool enable) {
    // Frames a previous run left repeating on the board would keep the Titan alive (or disabled) whatever we send now.
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

  /// Sends the current state again (once). Call at least every 150 ms while the motors may run.
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
    VMXErrorCode err;
    m_vmx->can.SendMessage(msg, VMXCAN_SEND_PERIOD_NO_REPEAT, &err);
  }

  /// Stops the periodic retransmission of one of the Titan's enable frames. Nothing is sent.
  void StopRepeating(uint32_t addressBase) {
    VMXCANMessage msg = Message(addressBase);
    VMXErrorCode err;
    m_vmx->can.SendMessage(msg, VMXCAN_SEND_PERIOD_STOP_REPEATING, &err);
  }

  uint8_t m_canId;
  std::shared_ptr<VMXPi> m_vmx;
  bool m_enabled = false;
};

}  // namespace wpilibvmx
