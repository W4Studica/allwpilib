// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/vmx/HALSimVMX.hpp"

#include <chrono>
#include <cstdio>
#include <utility>

#include "wpi/hal/Ports.h"
#include "wpi/hal/simulation/AnalogInData.h"
#include "wpi/hal/simulation/DIOData.h"
#include "wpi/hal/simulation/EncoderData.h"
#include "wpi/hal/simulation/IMUData.h"

using namespace wpilibvmx;

namespace {
constexpr auto kPollPeriod = std::chrono::milliseconds(10);
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kGravity = 9.80665;  // m/s^2 per g
}  // namespace

HALSimVMX::HALSimVMX(std::unique_ptr<VmxBackend> backend, Config config)
    : m_backend{std::move(backend)},
      m_config{std::move(config)},
      m_maps{m_config.maps} {
  // Pins are fixed for the lifetime of the object: callbacks keep pointers
  // into this vector, so it must never reallocate after Start().
  int numChannels = HAL_GetNumDigitalChannels();
  m_dio.resize(numChannels);
  for (int ch = 0; ch < numChannels; ++ch) {
    m_dio[ch].owner = this;
    m_dio[ch].channel = ch;
    if (auto vmx = m_maps.dio.Get(ch)) {
      m_dio[ch].vmxChannel = *vmx;
    }
  }

  int numEncoders = HAL_GetNumEncoders();
  m_encoders.resize(numEncoders);
  for (int i = 0; i < numEncoders; ++i) {
    m_encoders[i].index = i;
  }

  int numAnalog = HAL_GetNumAnalogInputs();
  m_analog.resize(numAnalog);
  for (int ch = 0; ch < numAnalog; ++ch) {
    m_analog[ch].channel = ch;
    if (auto vmx = m_maps.analog.Get(ch)) {
      m_analog[ch].vmxChannel = *vmx;
    }
  }
}

HALSimVMX::~HALSimVMX() {
  Stop();
}

void HALSimVMX::Start(bool spawnPollThread) {
  if (m_running.exchange(true)) {
    return;
  }
  for (auto& pin : m_dio) {
    if (pin.vmxChannel >= 0) {
      pin.valueCbKey = HALSIM_RegisterDIOValueCallback(
          pin.channel, &HALSimVMX::OnDioValue, &pin, false);
    }
  }
  if (!spawnPollThread) {
    return;
  }
  m_thread = std::thread{[this] {
    while (m_running) {
      Poll();
      std::this_thread::sleep_for(kPollPeriod);
    }
  }};
}

void HALSimVMX::Stop() {
  if (!m_running.exchange(false)) {
    return;
  }
  if (m_thread.joinable()) {
    m_thread.join();
  }
  // Cancel callbacks before touching the backend so none can run during teardown.
  for (auto& pin : m_dio) {
    if (pin.valueCbKey != 0) {
      HALSIM_CancelDIOValueCallback(pin.channel, pin.valueCbKey);
      pin.valueCbKey = 0;
    }
  }
  std::scoped_lock lock{m_mutex};
  for (auto& pin : m_dio) {
    if (pin.applied) {
      m_backend->ReleaseDigital(pin.vmxChannel);
      pin.applied = false;
    }
  }
  for (auto& pin : m_analog) {
    if (pin.applied) {
      m_backend->ReleaseAnalog(pin.vmxChannel);
      pin.applied = false;
    }
  }
  for (auto& pin : m_encoders) {
    if (pin.applied) {
      m_backend->ReleaseEncoder(pin.vmxA);
      pin.applied = false;
    }
  }
  if (m_imuApplied) {
    m_backend->ReleaseImu();
    m_imuApplied = false;
  }
}

std::set<int> HALSimVMX::EncoderOwnedDioChannels() const {
  std::set<int> owned;
  for (const auto& enc : m_encoders) {
    if (!HALSIM_GetEncoderInitialized(enc.index)) {
      continue;
    }
    int a = HALSIM_GetEncoderDigitalChannelA(enc.index);
    int b = HALSIM_GetEncoderDigitalChannelB(enc.index);
    if (m_maps.dio.Get(a) && m_maps.dio.Get(b)) {
      owned.insert(a);
      owned.insert(b);
    }
  }
  return owned;
}

void HALSimVMX::Poll() {
  // WPILib's Encoder also creates DigitalInputs on its A/B channels. On VMX a
  // channel serves one function, so the encoder wins and those DIOs are skipped.
  const std::set<int> encoderOwned = EncoderOwnedDioChannels();
  for (auto& pin : m_dio) {
    if (pin.vmxChannel >= 0) {
      PollDio(pin, encoderOwned.contains(pin.channel));
    }
  }
  for (auto& pin : m_encoders) {
    PollEncoder(pin);
  }
  if (m_config.imu) {
    PollImu();
  }
  for (auto& pin : m_analog) {
    if (pin.vmxChannel >= 0) {
      PollAnalog(pin);
    }
  }
}

void HALSimVMX::OnDioValue(const char*, void* param, const HAL_Value* value) {
  auto* pin = static_cast<DioPin*>(param);
  std::scoped_lock lock{pin->owner->m_mutex};
  // Input pins get their value from hardware; our own HALSIM_SetDIOValue call
  // for an input comes back through here and must not be written out.
  if (pin->applied && !pin->appliedInput) {
    pin->owner->m_backend->SetDigital(pin->vmxChannel, value->data.v_boolean);
  }
}

void HALSimVMX::PollDio(DioPin& pin, bool ownedByEncoder) {
  // Never call HALSIM_* while holding m_mutex: the sim HAL may hold its own
  // lock while running OnDioValue, which takes m_mutex.
  const bool initialized =
      HALSIM_GetDIOInitialized(pin.channel) && !ownedByEncoder;
  const bool isInput = HALSIM_GetDIOIsInput(pin.channel);
  const bool simValue = HALSIM_GetDIOValue(pin.channel);

  bool publish = false;
  bool hardwareValue = false;
  {
    std::scoped_lock lock{m_mutex};

    if (pin.applied && (!initialized || pin.appliedInput != isInput)) {
      m_backend->ReleaseDigital(pin.vmxChannel);
      pin.applied = false;
    }
    if (!pin.applied && initialized) {
      if (m_backend->InitDigital(pin.vmxChannel, isInput)) {
        pin.applied = true;
        pin.appliedInput = isInput;
        pin.claimFailedLogged = false;
        std::printf("HALSim VMX: DIO %d -> VMX channel %d (%s)\n", pin.channel,
                    pin.vmxChannel, isInput ? "input" : "output");
        if (!isInput) {
          m_backend->SetDigital(pin.vmxChannel, simValue);
        }
      } else if (!pin.claimFailedLogged) {
        pin.claimFailedLogged = true;
        std::fprintf(stderr,
                     "HALSim VMX: cannot claim VMX channel %d for DIO %d as %s\n",
                     pin.vmxChannel, pin.channel, isInput ? "input" : "output");
      }
    }
    if (!initialized) {
      pin.claimFailedLogged = false;
    }
    if (pin.applied && pin.appliedInput) {
      hardwareValue = m_backend->GetDigital(pin.vmxChannel);
      publish = true;
    }
  }

  if (publish && hardwareValue != simValue) {
    HALSIM_SetDIOValue(pin.channel, hardwareValue);
  }
}

void HALSimVMX::PollAnalog(AnalogPin& pin) {
  // See PollDio: no HALSIM_* calls while holding m_mutex.
  const bool initialized = HALSIM_GetAnalogInInitialized(pin.channel);
  const double simVolts = HALSIM_GetAnalogInVoltage(pin.channel);

  bool publish = false;
  double hardwareVolts = 0.0;
  {
    std::scoped_lock lock{m_mutex};

    if (pin.applied && !initialized) {
      m_backend->ReleaseAnalog(pin.vmxChannel);
      pin.applied = false;
    }
    if (!pin.applied && initialized) {
      pin.applied = m_backend->InitAnalog(pin.vmxChannel);
      if (pin.applied) {
        pin.claimFailedLogged = false;
        std::printf("HALSim VMX: AnalogIn %d -> VMX channel %d\n", pin.channel,
                    pin.vmxChannel);
      } else if (!pin.claimFailedLogged) {
        pin.claimFailedLogged = true;
        std::fprintf(stderr,
                     "HALSim VMX: cannot claim VMX channel %d for AnalogIn %d\n",
                     pin.vmxChannel, pin.channel);
      }
    }
    if (!initialized) {
      pin.claimFailedLogged = false;
    }
    if (pin.applied) {
      publish = m_backend->GetAnalogVoltage(pin.vmxChannel, &hardwareVolts);
    }
  }

  if (publish && hardwareVolts != simVolts) {
    HALSIM_SetAnalogInVoltage(pin.channel, hardwareVolts);
  }
}

void HALSimVMX::PollEncoder(EncoderPin& pin) {
  // See PollDio: no HALSIM_* calls while holding m_mutex.
  const int i = pin.index;
  const bool initialized = HALSIM_GetEncoderInitialized(i);
  const auto vmxA = m_maps.dio.Get(HALSIM_GetEncoderDigitalChannelA(i));
  const auto vmxB = m_maps.dio.Get(HALSIM_GetEncoderDigitalChannelB(i));
  const bool mapped = vmxA && vmxB;
  const bool reverse = HALSIM_GetEncoderReverseDirection(i);
  const bool resetRequested = HALSIM_GetEncoderReset(i);
  const double distancePerPulse = HALSIM_GetEncoderDistancePerPulse(i);
  if (!initialized) {
    pin.claimFailedLogged = false;
  }

  bool publish = false;
  int32_t count = 0;
  double rate = 0.0;
  bool direction = false;
  bool updateDirection = false;
  {
    std::scoped_lock lock{m_mutex};

    if (pin.applied &&
        (!initialized || !mapped || *vmxA != pin.vmxA || *vmxB != pin.vmxB)) {
      m_backend->ReleaseEncoder(pin.vmxA);
      pin.applied = false;
    }
    if (!pin.applied && initialized && mapped) {
      if (m_backend->InitEncoder(*vmxA, *vmxB)) {
        pin.applied = true;
        pin.vmxA = *vmxA;
        pin.vmxB = *vmxB;
        pin.haveLast = false;
        pin.claimFailedLogged = false;
        int32_t raw = 0;
        pin.offset = m_backend->GetEncoderCount(pin.vmxA, &raw) ? raw : 0;
        std::printf("HALSim VMX: Encoder %d -> VMX channels %d/%d\n", pin.index,
                    pin.vmxA, pin.vmxB);
      } else if (!pin.claimFailedLogged) {
        pin.claimFailedLogged = true;
        std::fprintf(stderr,
                     "HALSim VMX: cannot claim VMX channels %d/%d for Encoder %d\n",
                     *vmxA, *vmxB, pin.index);
      }
    }
    if (pin.applied) {
      int32_t raw = 0;
      if (m_backend->GetEncoderCount(pin.vmxA, &raw)) {
        if (resetRequested) {
          pin.offset = raw;
        }
        const auto now = std::chrono::steady_clock::now();
        const int32_t delta = pin.haveLast ? raw - pin.lastRaw : 0;
        if (pin.haveLast) {
          const double seconds =
              std::chrono::duration<double>(now - pin.lastTime).count();
          if (seconds > 0.0) {
            rate = delta * distancePerPulse / seconds * (reverse ? -1 : 1);
          }
        }
        if (delta != 0) {
          direction = (delta > 0) != reverse;
          updateDirection = true;
        }
        pin.lastRaw = raw;
        pin.lastTime = now;
        pin.haveLast = true;
        count = (raw - pin.offset) * (reverse ? -1 : 1);
        publish = true;
      }
    }
  }

  if (publish) {
    if (count != HALSIM_GetEncoderCount(i)) {
      HALSIM_SetEncoderCount(i, count);
    }
    HALSIM_SetEncoderRate(i, rate);
    if (updateDirection) {
      HALSIM_SetEncoderDirection(i, direction);
    }
  }
  if (resetRequested) {
    HALSIM_SetEncoderReset(i, false);
  }
}

void HALSimVMX::PollImu() {
  ImuSample s;
  bool valid = false;
  {
    std::scoped_lock lock{m_mutex};
    if (!m_imuApplied) {
      m_imuApplied = m_backend->InitImu();
      if (m_imuApplied) {
        m_imuClaimFailedLogged = false;
        std::puts("HALSim VMX: IMU claimed");
      } else if (!m_imuClaimFailedLogged) {
        m_imuClaimFailedLogged = true;
        std::fputs("HALSim VMX: cannot claim the IMU\n", stderr);
      }
    }
    if (m_imuApplied) {
      valid = m_backend->GetImu(&s);
    }
  }
  if (!valid) {
    return;
  }

  // Unit and sign conversion (navX -> WPILib IMU HAL). ASSUMPTIONS, unverified
  // on hardware: navX yaw and Z rate are positive clockwise while WPILib is
  // positive counterclockwise, so they are negated; roll, pitch, X/Y rates and
  // acceleration are passed through unchanged.
  HALSIM_SetIMUYaw(-s.yawDeg * kDegToRad);
  HALSIM_SetIMUAngleX(s.rollDeg * kDegToRad);
  HALSIM_SetIMUAngleY(s.pitchDeg * kDegToRad);
  HALSIM_SetIMUAngleZ(-s.yawDeg * kDegToRad);
  HALSIM_SetIMUGyroRateX(s.gyroXDps * kDegToRad);
  HALSIM_SetIMUGyroRateY(s.gyroYDps * kDegToRad);
  HALSIM_SetIMUGyroRateZ(-s.gyroZDps * kDegToRad);
  HALSIM_SetIMUAccelX(s.accelXG * kGravity);
  HALSIM_SetIMUAccelY(s.accelYG * kGravity);
  HALSIM_SetIMUAccelZ(s.accelZG * kGravity);
}
