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
 * Makes studica_driver::Titan::Enable() usable for switching on and off, for TitanEnableGuard.
 *
 * Titan::Enable(true) sends ENABLED_FLAG as a periodic CAN frame every 100 ms, Titan::Enable(false) sends
 * DISABLED_FLAG every 10 ms, and the VMX keeps repeating a periodic frame until it is told to stop. So after
 * Enable(false) the Titan keeps hearing DISABLED_FLAG ten times as often as ENABLED_FLAG and stays disabled
 * (MEASURED: the Titan LED flickered and the motor never ran when a program started disabled and then enabled).
 * Titan::Enable() only works for "enable once, disable once at the end", which is what studica_drivers/examples do.
 *
 * This stops the opposite frame's repetition before calling Titan::Enable(), so the last request wins.
 * studica_drivers is not modified; the stop is a plain VMX CAN call on the shared VMXPi.
 *
 *   studica_driver::Titan titan(42, 20000, 0.0f, wpilibvmx::SharedVMX());
 *   wpilibvmx::StudicaTitan switcher{titan, 42};
 *   wpilibvmx::TitanEnableGuard<wpilibvmx::StudicaTitan> guard{switcher};
 */
class StudicaTitan {
 public:
  StudicaTitan(studica_driver::Titan& titan, uint8_t canId, std::shared_ptr<VMXPi> vmx = SharedVMX())
      : m_titan{titan}, m_canId{canId}, m_vmx{std::move(vmx)} {}

  void Enable(bool enable) {
    StopRepeating(enable ? kDisabledFlag : kEnabledFlag);
    m_titan.Enable(enable);
  }

 private:
  static constexpr uint32_t kDisabledFlag = DISABLED_FLAG;
  static constexpr uint32_t kEnabledFlag = ENABLED_FLAG;

  /// Stops the periodic retransmission of one of the Titan's enable frames. Nothing is sent.
  void StopRepeating(uint32_t addressBase) {
    VMXCANMessage msg;
    msg.dataSize = 8;
    const uint8_t zeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    msg.setData(zeros, 8);
    msg.messageID = addressBase + m_canId;
    VMXErrorCode err;
    m_vmx->can.SendMessage(msg, VMXCAN_SEND_PERIOD_STOP_REPEATING, &err);
  }

  studica_driver::Titan& m_titan;
  uint8_t m_canId;
  std::shared_ptr<VMXPi> m_vmx;
};

}  // namespace wpilibvmx
