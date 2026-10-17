// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * C ABI between halsim_vmx and a hardware backend plugin (a shared library).
 *
 * The Studica backend needs VMXPi.h, which only exists on a VMX-pi image, so it is built
 * on the device as a separate library and loaded with dlopen. A plain C function table is
 * used so the two libraries may be built by different compilers and C++ standard
 * libraries (for example the WPILib trixie toolchain and the Ubuntu one on the VMX).
 *
 * A plugin exports:
 *   wpilibvmx_BackendApi* wpilibvmx_CreateBackendV2(void);
 *   void wpilibvmx_DestroyBackendV2(wpilibvmx_BackendApi* api);
 *
 * Booleans are int (0 = false). Methods that can fail return int (0 = failure).
 * Channel numbers are VMX channel indexes.
 */

#include <stdint.h>

#define WPILIBVMX_BACKEND_ABI_VERSION 2

#ifdef __cplusplus
extern "C" {
#endif

typedef struct wpilibvmx_ImuSample {
  double yawDeg;
  double pitchDeg;
  double rollDeg;
  double gyroXDps;
  double gyroYDps;
  double gyroZDps;
  double accelXG;
  double accelYG;
  double accelZG;
} wpilibvmx_ImuSample;

typedef struct wpilibvmx_BackendApi {
  /// Must be WPILIBVMX_BACKEND_ABI_VERSION. Checked before anything else is used.
  uint32_t abiVersion;
  /// sizeof(wpilibvmx_BackendApi) as seen by the plugin. Checked against the host's.
  uint32_t structSize;
  /// Opaque, passed as the first argument of every function.
  void* ctx;

  int (*initDigital)(void* ctx, int vmxChannel, int isInput);
  void (*releaseDigital)(void* ctx, int vmxChannel);
  void (*setDigital)(void* ctx, int vmxChannel, int value);
  int (*getDigital)(void* ctx, int vmxChannel);

  int (*initAnalog)(void* ctx, int vmxChannel);
  void (*releaseAnalog)(void* ctx, int vmxChannel);
  int (*getAnalogVoltage)(void* ctx, int vmxChannel, double* volts);

  int (*initEncoder)(void* ctx, int vmxChannelA, int vmxChannelB);
  void (*releaseEncoder)(void* ctx, int vmxChannelA);
  int (*getEncoderCount)(void* ctx, int vmxChannelA, int32_t* count);

  int (*initImu)(void* ctx);
  void (*releaseImu)(void* ctx);
  int (*getImu)(void* ctx, wpilibvmx_ImuSample* sample);

  /// PWM output on a VMX channel. periodMicros is the frame (5000, 10000 or 20000 for WPILib).
  /// setPwmPulse(0) leaves the line low (output disabled); it is also what releasePwm leaves behind.
  int (*initPwm)(void* ctx, int vmxChannel, int periodMicros);
  void (*releasePwm)(void* ctx, int vmxChannel);
  void (*setPwmPulse)(void* ctx, int vmxChannel, int pulseMicros);

  /// Battery (system) voltage. Returns 0 if no valid reading is available.
  int (*getBatteryVoltage)(void* ctx, double* volts);
} wpilibvmx_BackendApi;

typedef wpilibvmx_BackendApi* (*wpilibvmx_CreateBackendV2_fn)(void);
typedef void (*wpilibvmx_DestroyBackendV2_fn)(wpilibvmx_BackendApi* api);

#define WPILIBVMX_CREATE_SYMBOL "wpilibvmx_CreateBackendV2"
#define WPILIBVMX_DESTROY_SYMBOL "wpilibvmx_DestroyBackendV2"

#ifdef __cplusplus
}  // extern "C"
#endif
