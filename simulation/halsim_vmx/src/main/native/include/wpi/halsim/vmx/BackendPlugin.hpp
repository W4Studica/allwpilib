// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <utility>

#include "wpi/halsim/vmx/BackendApi.h"
#include "wpi/halsim/vmx/VmxBackend.hpp"

/// Visibility for the two exported plugin functions.
#if defined(_WIN32)
#define WPILIBVMX_PLUGIN_EXPORT __declspec(dllexport)
#else
#define WPILIBVMX_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

namespace wpilibvmx {

/**
 * Header-only helpers for a plugin: wraps any VmxBackend in the C function table.
 * Exceptions never cross the boundary; a failing call reports failure.
 */
inline wpilibvmx_BackendApi* MakeBackendApi(std::unique_ptr<VmxBackend> backend) {
  auto* api = new wpilibvmx_BackendApi{};
  api->abiVersion = WPILIBVMX_BACKEND_ABI_VERSION;
  api->structSize = sizeof(wpilibvmx_BackendApi);
  api->ctx = backend.release();

  // Captureless lambdas convert to the C function pointers.
  api->initDigital = [](void* c, int ch, int in) -> int {
    try { return static_cast<VmxBackend*>(c)->InitDigital(ch, in != 0); } catch (...) { return 0; }
  };
  api->releaseDigital = [](void* c, int ch) {
    try { static_cast<VmxBackend*>(c)->ReleaseDigital(ch); } catch (...) {}
  };
  api->setDigital = [](void* c, int ch, int v) {
    try { static_cast<VmxBackend*>(c)->SetDigital(ch, v != 0); } catch (...) {}
  };
  api->getDigital = [](void* c, int ch) -> int {
    try { return static_cast<VmxBackend*>(c)->GetDigital(ch) ? 1 : 0; } catch (...) { return 0; }
  };
  api->initAnalog = [](void* c, int ch) -> int {
    try { return static_cast<VmxBackend*>(c)->InitAnalog(ch); } catch (...) { return 0; }
  };
  api->releaseAnalog = [](void* c, int ch) {
    try { static_cast<VmxBackend*>(c)->ReleaseAnalog(ch); } catch (...) {}
  };
  api->getAnalogVoltage = [](void* c, int ch, double* volts) -> int {
    try { return static_cast<VmxBackend*>(c)->GetAnalogVoltage(ch, volts); } catch (...) { return 0; }
  };
  api->initEncoder = [](void* c, int a, int b) -> int {
    try { return static_cast<VmxBackend*>(c)->InitEncoder(a, b); } catch (...) { return 0; }
  };
  api->releaseEncoder = [](void* c, int a) {
    try { static_cast<VmxBackend*>(c)->ReleaseEncoder(a); } catch (...) {}
  };
  api->getEncoderCount = [](void* c, int a, int32_t* count) -> int {
    try { return static_cast<VmxBackend*>(c)->GetEncoderCount(a, count); } catch (...) { return 0; }
  };
  api->initImu = [](void* c) -> int {
    try { return static_cast<VmxBackend*>(c)->InitImu(); } catch (...) { return 0; }
  };
  api->releaseImu = [](void* c) {
    try { static_cast<VmxBackend*>(c)->ReleaseImu(); } catch (...) {}
  };
  api->getImu = [](void* c, wpilibvmx_ImuSample* out) -> int {
    try {
      ImuSample s;
      if (!static_cast<VmxBackend*>(c)->GetImu(&s)) {
        return 0;
      }
      out->yawDeg = s.yawDeg;
      out->pitchDeg = s.pitchDeg;
      out->rollDeg = s.rollDeg;
      out->gyroXDps = s.gyroXDps;
      out->gyroYDps = s.gyroYDps;
      out->gyroZDps = s.gyroZDps;
      out->accelXG = s.accelXG;
      out->accelYG = s.accelYG;
      out->accelZG = s.accelZG;
      return 1;
    } catch (...) {
      return 0;
    }
  };
  return api;
}

inline void DestroyBackendApi(wpilibvmx_BackendApi* api) {
  if (api == nullptr) {
    return;
  }
  delete static_cast<VmxBackend*>(api->ctx);
  delete api;
}

}  // namespace wpilibvmx
