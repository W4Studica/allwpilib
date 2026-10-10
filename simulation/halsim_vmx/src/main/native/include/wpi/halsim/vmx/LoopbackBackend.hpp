// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <mutex>

#include "wpi/halsim/vmx/VmxBackend.hpp"

namespace wpilibvmx {

/// In-memory backend used on a PC (no VMX hardware) and in tests.
class LoopbackBackend : public VmxBackend {
 public:
  struct Pin {
    bool isInput = false;
    bool value = false;
  };

  bool InitDigital(int vmxChannel, bool isInput) override;
  void ReleaseDigital(int vmxChannel) override;
  void SetDigital(int vmxChannel, bool value) override;
  bool GetDigital(int vmxChannel) override;

  /// Test hooks.
  bool IsClaimed(int vmxChannel);
  bool IsInput(int vmxChannel);
  /// Drives a pin as if an external signal changed it (input pins).
  void Drive(int vmxChannel, bool value);

 private:
  std::mutex m_mutex;
  std::map<int, Pin> m_pins;
};

}  // namespace wpilibvmx
