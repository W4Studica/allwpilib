// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdio>
#include <exception>
#include <memory>
#include <utility>

#include "wpi/studica/detail/backend/BackendApi.h"
#include "wpi/studica/detail/backend/HardwareBackend.hpp"

/// Visibility for the two exported plugin functions.
#if defined(_WIN32)
#define WPILIBSTUDICA_PLUGIN_EXPORT __declspec(dllexport)
#else
#define WPILIBSTUDICA_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

namespace wpi::studica {
inline void ReportBackendException(const char* operation) noexcept {
  try {
    throw;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "Studica backend: %s failed: %s\n", operation,
                 e.what());
  } catch (...) {
    std::fprintf(stderr, "Studica backend: %s failed: unknown exception\n",
                 operation);
  }
}

/**
 * Header-only helpers for a plugin: wraps any HardwareBackend in the C function
 * table. Exceptions never cross the boundary; a failing call reports failure.
 */
inline wpilibstudica_BackendApi* MakeBackendApi(
    std::unique_ptr<HardwareBackend> backend) {
  if (!backend)
    return nullptr;
  auto* api = new wpilibstudica_BackendApi{};
  api->abiVersion = WPILIBSTUDICA_BACKEND_ABI_VERSION;
  api->structSize = sizeof(wpilibstudica_BackendApi);
  api->ctx = backend.release();

  // Captureless lambdas convert to the C function pointers.
  api->initDigital = [](void* c, int ch, int in) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitDigital(ch, in != 0);
    } catch (...) {
      ReportBackendException("initDigital");
      return 0;
    }
  };
  api->releaseDigital = [](void* c, int ch) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseDigital(ch);
    } catch (...) {
      ReportBackendException("releaseDigital");
    }
  };
  api->setDigital = [](void* c, int ch, int v) {
    try {
      static_cast<HardwareBackend*>(c)->SetDigital(ch, v != 0);
    } catch (...) {
      ReportBackendException("setDigital");
    }
  };
  api->getDigital = [](void* c, int ch) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->GetDigital(ch) ? 1 : 0;
    } catch (...) {
      ReportBackendException("getDigital");
      return 0;
    }
  };
  api->initAnalog = [](void* c, int ch) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitAnalog(ch);
    } catch (...) {
      ReportBackendException("initAnalog");
      return 0;
    }
  };
  api->releaseAnalog = [](void* c, int ch) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseAnalog(ch);
    } catch (...) {
      ReportBackendException("releaseAnalog");
    }
  };
  api->getAnalogVoltage = [](void* c, int ch, double* volts) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->GetAnalogVoltage(ch, volts);
    } catch (...) {
      ReportBackendException("getAnalogVoltage");
      return 0;
    }
  };
  api->initEncoder = [](void* c, int a, int b) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitEncoder(a, b);
    } catch (...) {
      ReportBackendException("initEncoder");
      return 0;
    }
  };
  api->releaseEncoder = [](void* c, int a) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseEncoder(a);
    } catch (...) {
      ReportBackendException("releaseEncoder");
    }
  };
  api->getEncoderCount = [](void* c, int a, int32_t* count) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->GetEncoderCount(a, count);
    } catch (...) {
      ReportBackendException("getEncoderCount");
      return 0;
    }
  };
  api->initImu = [](void* c) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitImu();
    } catch (...) {
      ReportBackendException("initImu");
      return 0;
    }
  };
  api->releaseImu = [](void* c) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseImu();
    } catch (...) {
      ReportBackendException("releaseImu");
    }
  };
  api->getImu = [](void* c, wpilibstudica_ImuSample* out) -> int {
    try {
      ImuSample s;
      if (!static_cast<HardwareBackend*>(c)->GetImu(&s)) {
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
      ReportBackendException("getImu");
      return 0;
    }
  };
  api->initPwm = [](void* c, int ch, int period) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitPwm(ch, period);
    } catch (...) {
      ReportBackendException("initPwm");
      return 0;
    }
  };
  api->releasePwm = [](void* c, int ch) {
    try {
      static_cast<HardwareBackend*>(c)->ReleasePwm(ch);
    } catch (...) {
      ReportBackendException("releasePwm");
    }
  };
  api->setPwmPulse = [](void* c, int ch, int pulse) {
    try {
      static_cast<HardwareBackend*>(c)->SetPwmPulse(ch, pulse);
    } catch (...) {
      ReportBackendException("setPwmPulse");
    }
  };
  api->initI2c = [](void* c) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitI2c();
    } catch (...) {
      ReportBackendException("initI2c");
      return 0;
    }
  };
  api->releaseI2c = [](void* c) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseI2c();
    } catch (...) {
      ReportBackendException("releaseI2c");
    }
  };
  api->transactionI2c = [](void* c, int address, const uint8_t* tx, int txSize,
                           uint8_t* rx, int rxSize) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->TransactionI2c(
          address, tx, txSize, rx, rxSize);
    } catch (...) {
      ReportBackendException("transactionI2c");
      return 0;
    }
  };
  api->initDutyCycle = [](void* c, int ch) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitDutyCycle(ch);
    } catch (...) {
      ReportBackendException("initDutyCycle");
      return 0;
    }
  };
  api->releaseDutyCycle = [](void* c, int ch) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseDutyCycle(ch);
    } catch (...) {
      ReportBackendException("releaseDutyCycle");
    }
  };
  api->getDutyCycle = [](void* c, int ch, double* frequency,
                         double* output) -> int {
    try {
      DutyCycleReading reading;
      if (!static_cast<HardwareBackend*>(c)->GetDutyCycle(ch, &reading)) {
        return 0;
      }
      *frequency = reading.frequencyHz;
      *output = reading.output;
      return 1;
    } catch (...) {
      ReportBackendException("getDutyCycle");
      return 0;
    }
  };
  api->initInterrupt = [](void* c, int ch) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitInterrupt(ch);
    } catch (...) {
      ReportBackendException("initInterrupt");
      return 0;
    }
  };
  api->releaseInterrupt = [](void* c, int ch) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseInterrupt(ch);
    } catch (...) {
      ReportBackendException("releaseInterrupt");
    }
  };
  api->readInterrupt = [](void* c, int ch, int* rising, uint64_t* timestamp,
                          uint32_t* dropped) -> int {
    try {
      InterruptEvent event;
      if (!static_cast<HardwareBackend*>(c)->ReadInterrupt(ch, &event)) {
        return 0;
      }
      *rising = event.rising;
      *timestamp = event.timestampMicros;
      *dropped = event.dropped;
      return 1;
    } catch (...) {
      ReportBackendException("readInterrupt");
      return 0;
    }
  };
  api->getBatteryVoltage = [](void* c, double* volts) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->GetBatteryVoltage(volts);
    } catch (...) {
      ReportBackendException("getBatteryVoltage");
      return 0;
    }
  };
  api->tryGetDigital = [](void* c, int ch, int* out) -> int {
    try {
      bool value = false;
      if (!out || !static_cast<HardwareBackend*>(c)->TryGetDigital(ch, &value))
        return 0;
      *out = value;
      return 1;
    } catch (...) {
      ReportBackendException("tryGetDigital");
      return 0;
    }
  };
  api->trySetDigital = [](void* c, int ch, int value) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->TrySetDigital(ch, value != 0);
    } catch (...) {
      ReportBackendException("trySetDigital");
      return 0;
    }
  };
  api->trySetPwmPulse = [](void* c, int ch, int pulse) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->TrySetPwmPulse(ch, pulse);
    } catch (...) {
      ReportBackendException("trySetPwmPulse");
      return 0;
    }
  };
  return api;
}

inline void DestroyBackendApi(wpilibstudica_BackendApi* api) {
  if (api == nullptr) {
    return;
  }
  delete static_cast<HardwareBackend*>(api->ctx);
  delete api;
}

}  // namespace wpi::studica
