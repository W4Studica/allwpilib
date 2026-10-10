// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Test-only backend plugin: the loopback backend behind the C ABI. Built into two
// libraries, one well-behaved and one (HALSIMVMX_TEST_BAD_ABI) that reports a wrong
// ABI version. Like the real Studica plugin it does not link against halsim_vmx.

#include <memory>

#include "wpi/halsim/vmx/BackendPlugin.hpp"
#include "wpi/halsim/vmx/LoopbackBackend.hpp"

extern "C" {

WPILIBVMX_PLUGIN_EXPORT wpilibvmx_BackendApi* wpilibvmx_CreateBackendV1(void) {
  auto* api = wpilibvmx::MakeBackendApi(
      std::make_unique<wpilibvmx::LoopbackBackend>());
#ifdef HALSIMVMX_TEST_BAD_ABI
  api->abiVersion = 999;
#endif
  return api;
}

WPILIBVMX_PLUGIN_EXPORT void wpilibvmx_DestroyBackendV1(
    wpilibvmx_BackendApi* api) {
  wpilibvmx::DestroyBackendApi(api);
}

}  // extern "C"
