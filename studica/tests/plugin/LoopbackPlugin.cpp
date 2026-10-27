// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Test-only backend plugin: the loopback backend behind the C ABI. Built into
// two libraries, one well-behaved and one (HALSIMSTUDICA_TEST_BAD_ABI) that
// reports a wrong ABI version. Like the real Studica plugin it does not link
// against halsim_studica.

#include <memory>

#include "wpi/studica/detail/backend/BackendPlugin.hpp"
#include "wpi/studica/detail/backend/LoopbackBackend.hpp"

extern "C" {

WPILIBSTUDICA_PLUGIN_EXPORT wpilibstudica_BackendApi*
wpilibstudica_CreateBackendV3(void) {
  auto* api = wpi::studica::MakeBackendApi(
      std::make_unique<wpi::studica::LoopbackBackend>());
#ifdef HALSIMSTUDICA_TEST_BAD_ABI
  api->abiVersion = 999;
#endif
  return api;
}

WPILIBSTUDICA_PLUGIN_EXPORT void wpilibstudica_DestroyBackendV3(
    wpilibstudica_BackendApi* api) {
  wpi::studica::DestroyBackendApi(api);
}

}  // extern "C"
