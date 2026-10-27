// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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

/**
 * Header-only helpers for a plugin: wraps any HardwareBackend in the C function
 * table. Exceptions never cross the boundary; a failing call reports failure.
 */
inline wpilibstudica_BackendApi* MakeBackendApi(
    std::unique_ptr<HardwareBackend> backend) {
  auto* api = new wpilibstudica_BackendApi{};
  api->abiVersion = WPILIBSTUDICA_BACKEND_ABI_VERSION;
  api->structSize = sizeof(wpilibstudica_BackendApi);
  api->ctx = backend.release();

  // Captureless lambdas convert to the C function pointers.
  api->initDigital = [](void* c, int ch, int in) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitDigital(ch, in != 0);
    } catch (...) {
      return 0;
    }
  };
  api->releaseDigital = [](void* c, int ch) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseDigital(ch);
    } catch (...) {
    }
  };
  api->setDigital = [](void* c, int ch, int v) {
    try {
      static_cast<HardwareBackend*>(c)->SetDigital(ch, v != 0);
    } catch (...) {
    }
  };
  api->getDigital = [](void* c, int ch) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->GetDigital(ch) ? 1 : 0;
    } catch (...) {
      return 0;
    }
  };
  api->initAnalog = [](void* c, int ch) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitAnalog(ch);
    } catch (...) {
      return 0;
    }
  };
  api->releaseAnalog = [](void* c, int ch) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseAnalog(ch);
    } catch (...) {
    }
  };
  api->getAnalogVoltage = [](void* c, int ch, double* volts) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->GetAnalogVoltage(ch, volts);
    } catch (...) {
      return 0;
    }
  };
  api->initEncoder = [](void* c, int a, int b) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitEncoder(a, b);
    } catch (...) {
      return 0;
    }
  };
  api->releaseEncoder = [](void* c, int a) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseEncoder(a);
    } catch (...) {
    }
  };
  api->getEncoderCount = [](void* c, int a, int32_t* count) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->GetEncoderCount(a, count);
    } catch (...) {
      return 0;
    }
  };
  api->initImu = [](void* c) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitImu();
    } catch (...) {
      return 0;
    }
  };
  api->releaseImu = [](void* c) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseImu();
    } catch (...) {
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
      return 0;
    }
  };
  api->initPwm = [](void* c, int ch, int period) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitPwm(ch, period);
    } catch (...) {
      return 0;
    }
  };
  api->releasePwm = [](void* c, int ch) {
    try {
      static_cast<HardwareBackend*>(c)->ReleasePwm(ch);
    } catch (...) {
    }
  };
  api->setPwmPulse = [](void* c, int ch, int pulse) {
    try {
      static_cast<HardwareBackend*>(c)->SetPwmPulse(ch, pulse);
    } catch (...) {
    }
  };
  api->initI2c = [](void* c) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitI2c();
    } catch (...) {
      return 0;
    }
  };
  api->releaseI2c = [](void* c) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseI2c();
    } catch (...) {
    }
  };
  api->transactionI2c = [](void* c, int address, const uint8_t* tx, int txSize,
                           uint8_t* rx, int rxSize) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->TransactionI2c(
          address, tx, txSize, rx, rxSize);
    } catch (...) {
      return 0;
    }
  };
  api->initDutyCycle = [](void* c, int ch) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitDutyCycle(ch);
    } catch (...) {
      return 0;
    }
  };
  api->releaseDutyCycle = [](void* c, int ch) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseDutyCycle(ch);
    } catch (...) {
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
      return 0;
    }
  };
  api->initInterrupt = [](void* c, int ch) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->InitInterrupt(ch);
    } catch (...) {
      return 0;
    }
  };
  api->releaseInterrupt = [](void* c, int ch) {
    try {
      static_cast<HardwareBackend*>(c)->ReleaseInterrupt(ch);
    } catch (...) {
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
      return 0;
    }
  };
  api->getBatteryVoltage = [](void* c, double* volts) -> int {
    try {
      return static_cast<HardwareBackend*>(c)->GetBatteryVoltage(volts);
    } catch (...) {
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
