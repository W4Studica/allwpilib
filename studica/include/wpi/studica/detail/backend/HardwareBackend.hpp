// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdint>
#include <memory>

namespace wpi::studica {

/**
 * Hardware access used by halsim_studica. The real implementation wraps the
 * Studica drivers (wpi::studica::drivers::*) and only exists on a VMX-pi; the
 * host implementation is a loopback so the extension can be built and tested on
 * a PC.
 *
 * All channel numbers here are VMX channel indexes, not WPILib channel numbers.
 */
/// One IMU reading in the sensor's native units (navX: degrees, deg/s, g).
struct ImuSample {
  double yawDeg = 0;
  double pitchDeg = 0;
  double rollDeg = 0;
  double gyroXDps = 0;
  double gyroYDps = 0;
  double gyroZDps = 0;
  double accelXG = 0;
  double accelYG = 0;
  double accelZG = 0;
};

struct DutyCycleReading {
  double frequencyHz = 0;
  double output = 0;
};

struct InterruptEvent {
  bool rising = false;
  uint64_t timestampMicros = 0;  // VMX hardware clock, not the OS wall clock.
  uint32_t dropped =
      0;  // Events dropped before this event due to queue overflow.
};

class HardwareBackend {
 public:
  virtual ~HardwareBackend() = default;

  /// Claims a digital channel. Returns false if the channel could not be set
  /// up.
  virtual bool InitDigital(int vmxChannel, bool isInput) = 0;
  virtual void ReleaseDigital(int vmxChannel) = 0;
  virtual void SetDigital(int vmxChannel, bool value) = 0;
  virtual bool GetDigital(int vmxChannel) = 0;
  // Checked operations distinguish failure from a legitimate LOW input.
  // Existing backends retain their behavior through these compatibility
  // defaults.
  virtual bool TryGetDigital(int channel, bool* value) {
    if (!value)
      return false;
    *value = GetDigital(channel);
    return true;
  }
  virtual bool TrySetDigital(int channel, bool value) {
    SetDigital(channel, value);
    return true;
  }
  virtual bool TrySetPwmPulse(int channel, int pulse) {
    SetPwmPulse(channel, pulse);
    return true;
  }

  /// Claims an analog input channel. Returns false if it could not be set up.
  virtual bool InitAnalog(int vmxChannel) = 0;
  virtual void ReleaseAnalog(int vmxChannel) = 0;
  /// Reads the averaged voltage. Returns false if no valid reading is
  /// available.
  virtual bool GetAnalogVoltage(int vmxChannel, double* volts) = 0;

  /// Claims a quadrature encoder on two channels. Identified by channel A.
  virtual bool InitEncoder(int vmxChannelA, int vmxChannelB) = 0;
  virtual void ReleaseEncoder(int vmxChannelA) = 0;
  /// Reads the raw tick count since the encoder was claimed.
  virtual bool GetEncoderCount(int vmxChannelA, int32_t* count) = 0;

  /// Claims the IMU (navX). There is only one.
  virtual bool InitImu() = 0;
  virtual void ReleaseImu() = 0;
  /// Reads the IMU. Returns false while it is disconnected or calibrating.
  virtual bool GetImu(ImuSample* sample) = 0;

  /// Starts a PWM output with the given frame length (microseconds). Identified
  /// by VMX channel.
  virtual bool InitPwm(int vmxChannel, int periodMicros) = 0;
  /// Stops the output (line low) and frees the channel.
  virtual void ReleasePwm(int vmxChannel) = 0;
  /// Sets the high time of each frame. 0 disables the output (line low).
  virtual void SetPwmPulse(int vmxChannel, int pulseMicros) = 0;

  /// VMX has one I2C bus. Channel selection is discovered from SDK
  /// capabilities.
  virtual bool InitI2c() { return false; }
  virtual void ReleaseI2c() {}
  virtual bool TransactionI2c(int address, const uint8_t* tx, int txSize,
                              uint8_t* rx, int rxSize) {
    return false;
  }

  virtual bool InitDutyCycle(int channel) { return false; }
  virtual void ReleaseDutyCycle(int channel) {}
  virtual bool GetDutyCycle(int channel, DutyCycleReading* reading) {
    return false;
  }

  /// Captures both edges. ReadInterrupt drains timestamped events without
  /// invoking user code on SDK threads.
  virtual bool InitInterrupt(int channel) { return false; }
  virtual void ReleaseInterrupt(int channel) {}
  virtual bool ReadInterrupt(int channel, InterruptEvent* event) {
    return false;
  }

  /// Reads the battery (system) voltage. Returns false if no valid reading is
  /// available.
  virtual bool GetBatteryVoltage(double* volts) = 0;
};

/**
 * Creates the backend named by HALSIMSTUDICA_BACKEND (path of a backend plugin,
 * see BackendApi.h). If it is unset, the in-memory loopback backend is used.
 * Returns nullptr if a backend was named but could not be loaded.
 */
std::unique_ptr<HardwareBackend> CreateBackend();

}  // namespace wpi::studica
