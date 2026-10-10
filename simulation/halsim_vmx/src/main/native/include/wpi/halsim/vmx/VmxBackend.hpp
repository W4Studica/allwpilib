// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>

namespace wpilibvmx {

/**
 * Hardware access used by halsim_vmx. The real implementation wraps the
 * Studica drivers (studica_driver::*) and only exists on a VMX-pi; the host
 * implementation is a loopback so the extension can be built and tested on a
 * PC.
 *
 * All channel numbers here are VMX channel indexes, not WPILib channel numbers.
 */
class VmxBackend {
 public:
  virtual ~VmxBackend() = default;

  /// Claims a digital channel. Returns false if the channel could not be set up.
  virtual bool InitDigital(int vmxChannel, bool isInput) = 0;
  virtual void ReleaseDigital(int vmxChannel) = 0;
  virtual void SetDigital(int vmxChannel, bool value) = 0;
  virtual bool GetDigital(int vmxChannel) = 0;
};

/// Creates the backend selected at build time.
std::unique_ptr<VmxBackend> CreateBackend();

}  // namespace wpilibvmx
