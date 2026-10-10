// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/vmx/LoopbackBackend.hpp"

using namespace wpilibvmx;

bool LoopbackBackend::InitDigital(int vmxChannel, bool isInput) {
  std::scoped_lock lock{m_mutex};
  auto [it, inserted] = m_pins.try_emplace(vmxChannel);
  if (!inserted) {
    return false;  // already claimed
  }
  it->second.isInput = isInput;
  return true;
}

void LoopbackBackend::ReleaseDigital(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  m_pins.erase(vmxChannel);
}

void LoopbackBackend::SetDigital(int vmxChannel, bool value) {
  std::scoped_lock lock{m_mutex};
  if (auto it = m_pins.find(vmxChannel); it != m_pins.end()) {
    it->second.value = value;
  }
}

bool LoopbackBackend::GetDigital(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  if (auto it = m_pins.find(vmxChannel); it != m_pins.end()) {
    return it->second.value;
  }
  return false;
}

bool LoopbackBackend::IsClaimed(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  return m_pins.contains(vmxChannel);
}

bool LoopbackBackend::IsInput(int vmxChannel) {
  std::scoped_lock lock{m_mutex};
  auto it = m_pins.find(vmxChannel);
  return it != m_pins.end() && it->second.isInput;
}

void LoopbackBackend::Drive(int vmxChannel, bool value) {
  SetDigital(vmxChannel, value);
}
