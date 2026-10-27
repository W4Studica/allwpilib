// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/studica/HALSimStudica.hpp"

#include <chrono>
#include <cstdio>
#include <utility>

#include "wpi/hal/HAL.h"
#include "wpi/hal/Ports.h"
#include "wpi/hal/simulation/AnalogInData.h"
#include "wpi/hal/simulation/DIOData.h"
#include "wpi/hal/simulation/DigitalEdgeData.h"
#include "wpi/hal/simulation/DutyCycleData.h"
#include "wpi/hal/simulation/EncoderData.h"
#include "wpi/hal/simulation/I2CData.h"
#include "wpi/hal/simulation/IMUData.h"
#include "wpi/hal/simulation/PWMData.h"
#include "wpi/hal/simulation/RoboRioData.h"
#include "wpi/studica/detail/runtime/TerminationHandlers.hpp"

using namespace wpi::studica;

namespace {
constexpr auto kPollPeriod = std::chrono::milliseconds(10);
// The system voltage is read every 100 ms.
constexpr int kBatteryPollEvery = 10;
}  // namespace

HALSimStudica::HALSimStudica(std::unique_ptr<HardwareBackend> backend,
                             Config config)
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

  int numPwm = HAL_GetNumPWMChannels();
  m_pwm.resize(numPwm);
  for (int ch = 0; ch < numPwm; ++ch) {
    m_pwm[ch].channel = ch;
    if (auto vmx = m_maps.pwm.Get(ch)) {
      m_pwm[ch].vmxChannel = *vmx;
    }
  }

  m_dutyCycles.resize(HAL_GetNumDutyCycles());
  for (int ch = 0; ch < static_cast<int>(m_dutyCycles.size()); ++ch) {
    m_dutyCycles[ch].channel = ch;
    if (auto vmx = m_maps.dio.Get(ch)) {
      m_dutyCycles[ch].vmxChannel = *vmx;
    }
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

HALSimStudica::~HALSimStudica() {
  Stop();
}

bool HALSimStudica::Start(bool spawnPollThread) {
  if (m_running.exchange(true)) {
    return true;
  }
  if (m_config.i2cPort >= 0) {
    m_i2cCallback = HALSIM_RegisterI2CTransactionCallback(
        m_config.i2cPort, &HALSimStudica::OnI2cTransaction, this);
    if (m_i2cCallback <= 0) {
      std::fprintf(
          stderr,
          "HALSim Studica: I2C port %d already has a transaction provider\n",
          m_config.i2cPort);
      m_running = false;
      return false;
    }
  }
  for (auto& pin : m_dio) {
    if (pin.vmxChannel >= 0) {
      pin.valueCbKey = HALSIM_RegisterDIOValueCallback(
          pin.channel, &HALSimStudica::OnDioValue, &pin, false);
    }
  }
  if (!spawnPollThread) {
    return true;
  }
  m_thread = std::thread{[this] {
    while (m_running) {
      Poll();
      // The VMX HAL installs its SIGINT/SIGTERM handlers when it starts,
      // possibly after this extension did.
      InstallTerminationHandlers();
      std::this_thread::sleep_for(kPollPeriod);
    }
  }};
  return true;
}

void HALSimStudica::Stop() {
  if (!m_running.exchange(false)) {
    return;
  }
  if (m_thread.joinable()) {
    m_thread.join();
  }
  // Cancel waits for a transaction; do it before taking m_mutex.
  if (m_i2cCallback > 0) {
    HALSIM_CancelI2CTransactionCallback(m_config.i2cPort, m_i2cCallback);
    m_i2cCallback = 0;
  }
  // Cancel callbacks before touching the backend so none can run during
  // teardown.
  for (auto& pin : m_dio) {
    if (pin.valueCbKey != 0) {
      HALSIM_CancelDIOValueCallback(pin.channel, pin.valueCbKey);
      pin.valueCbKey = 0;
    }
  }
  std::unique_lock busLock{m_i2cMutex};
  std::scoped_lock lock{m_mutex};
  if (m_i2cApplied) {
    m_backend->ReleaseI2c();
    m_i2cApplied = false;
  }
  for (auto& pin : m_dutyCycles) {
    if (pin.applied) {
      m_backend->ReleaseDutyCycle(pin.vmxChannel);
      pin.applied = false;
    }
  }
  for (auto& pin : m_dio) {
    if (pin.interruptApplied) {
      m_backend->ReleaseInterrupt(pin.vmxChannel);
      pin.interruptApplied = false;
    }
    if (pin.applied) {
      m_backend->ReleaseDigital(pin.vmxChannel);
      pin.applied = false;
      pin.outputValue.reset();
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
  for (auto& pin : m_pwm) {
    if (pin.applied) {
      m_backend->ReleasePwm(pin.vmxChannel);
      pin.applied = false;
    }
  }
  if (m_imuApplied) {
    m_backend->ReleaseImu();
    m_imuApplied = false;
  }
}

std::set<int> HALSimStudica::EncoderOwnedDioChannels() const {
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
  for (const auto& pin : m_dutyCycles) {
    if (pin.vmxChannel >= 0 && HALSIM_GetDutyCycleInitialized(pin.channel)) {
      owned.insert(pin.channel);
    }
  }
  return owned;
}

void HALSimStudica::Poll() {
  // WPILib's Encoder also creates DigitalInputs on its A/B channels. On VMX a
  // channel serves one function, so the encoder wins and those DIOs are
  // skipped.
  const std::set<int> encoderOwned = EncoderOwnedDioChannels();
  for (auto& pin : m_dio) {
    if (pin.vmxChannel >= 0) {
      const bool reserved = encoderOwned.contains(pin.channel);
      if (!PollInterrupt(pin, reserved)) {
        PollDio(pin, reserved);
      }
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
  for (auto& pin : m_pwm) {
    if (pin.vmxChannel >= 0) {
      PollPwm(pin);
    }
  }
  for (auto& pin : m_dutyCycles) {
    if (pin.vmxChannel >= 0) {
      PollDutyCycle(pin);
    }
  }
  PollI2c();
  PollBattery();
}

void HALSimStudica::OnDioValue(const char*, void* param,
                               const HAL_Value* value) {
  auto* pin = static_cast<DioPin*>(param);
  std::scoped_lock lock{pin->owner->m_mutex};
  // Input pins get their value from hardware; our own HALSIM_SetDIOValue call
  // for an input comes back through here and must not be written out.
  if (pin->applied && !pin->appliedInput) {
    bool output = value->data.v_boolean;
    if (pin->owner->m_backend->TrySetDigital(pin->vmxChannel, output)) {
      pin->outputValue = output;
    } else {
      pin->outputValue.reset();
    }
  }
}

void HALSimStudica::PollDio(DioPin& pin, bool ownedByEncoder) {
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
      pin.outputValue.reset();
    }
    if (!pin.applied && initialized) {
      if (m_backend->InitDigital(pin.vmxChannel, isInput)) {
        pin.applied = true;
        pin.appliedInput = isInput;
        pin.outputValue.reset();
        pin.claimFailedLogged = false;
        std::printf("HALSim Studica: DIO %d -> VMX channel %d (%s)\n",
                    pin.channel, pin.vmxChannel, isInput ? "input" : "output");
      } else if (!pin.claimFailedLogged) {
        pin.claimFailedLogged = true;
        std::fprintf(
            stderr,
            "HALSim Studica: cannot claim VMX channel %d for DIO %d as %s\n",
            pin.vmxChannel, pin.channel, isInput ? "input" : "output");
      }
    }
    if (!initialized) {
      pin.claimFailedLogged = false;
    }
    if (pin.applied && !pin.appliedInput && pin.outputValue != simValue) {
      if (m_backend->TrySetDigital(pin.vmxChannel, simValue))
        pin.outputValue = simValue;
    }
    if (pin.applied && pin.appliedInput) {
      publish = m_backend->TryGetDigital(pin.vmxChannel, &hardwareValue);
    }
  }

  if (publish && hardwareValue != simValue) {
    HALSIM_SetDIOValue(pin.channel, hardwareValue);
  }
}

void HALSimStudica::PollAnalog(AnalogPin& pin) {
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
        std::printf("HALSim Studica: AnalogIn %d -> VMX channel %d\n",
                    pin.channel, pin.vmxChannel);
      } else if (!pin.claimFailedLogged) {
        pin.claimFailedLogged = true;
        std::fprintf(
            stderr,
            "HALSim Studica: cannot claim VMX channel %d for AnalogIn %d\n",
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

void HALSimStudica::PollEncoder(EncoderPin& pin) {
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
        pin.claimFailedLogged = false;
        pin.haveBaseline = false;
        std::printf("HALSim Studica: Encoder %d -> VMX channels %d/%d\n",
                    pin.index, pin.vmxA, pin.vmxB);
      } else if (!pin.claimFailedLogged) {
        pin.claimFailedLogged = true;
        std::fprintf(
            stderr,
            "HALSim Studica: cannot claim VMX channels %d/%d for Encoder %d\n",
            *vmxA, *vmxB, pin.index);
      }
    }
    if (pin.applied) {
      int32_t raw = 0;
      if (m_backend->GetEncoderCount(pin.vmxA, &raw)) {
        if (!pin.haveBaseline) {
          pin.state.Initialize(raw);
          pin.haveBaseline = true;
        }
        const auto reading =
            pin.state.Update(raw, resetRequested, reverse, distancePerPulse,
                             std::chrono::steady_clock::now());
        count = reading.count;
        rate = reading.rate;
        direction = reading.direction;
        updateDirection = reading.directionChanged;
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
  if (resetRequested && publish) {
    HALSIM_SetEncoderReset(i, false);
  }
}

void HALSimStudica::PollImu() {
  ImuSample s;
  bool valid = false;
  {
    std::scoped_lock lock{m_mutex};
    if (!m_imuApplied) {
      m_imuApplied = m_backend->InitImu();
      if (m_imuApplied) {
        m_imuClaimFailedLogged = false;
        std::puts("HALSim Studica: IMU claimed");
      } else if (!m_imuClaimFailedLogged) {
        m_imuClaimFailedLogged = true;
        std::fputs("HALSim Studica: cannot claim the IMU\n", stderr);
      }
    }
    if (m_imuApplied) {
      valid = m_backend->GetImu(&s);
    }
  }
  if (!valid) {
    return;
  }

  const auto reading = ConvertImu(s);
  HALSIM_SetIMUYaw(reading.yaw);
  HALSIM_SetIMUAngleX(reading.roll);
  HALSIM_SetIMUAngleY(reading.pitch);
  HALSIM_SetIMUAngleZ(reading.yaw);
  HALSIM_SetIMUGyroRateX(reading.gyroX);
  HALSIM_SetIMUGyroRateY(reading.gyroY);
  HALSIM_SetIMUGyroRateZ(reading.gyroZ);
  HALSIM_SetIMUAccelX(reading.accelX);
  HALSIM_SetIMUAccelY(reading.accelY);
  HALSIM_SetIMUAccelZ(reading.accelZ);
  const auto q = ImuQuaternion(reading);
  const HAL_Quaternion quat{HAL_GetMonotonicTime(), q.w, q.x, q.y, q.z};
  HALSIM_SetIMUQuaternion(&quat);
}

void HALSimStudica::PollPwm(PwmPin& pin) {
  // See PollDio: no HALSIM_* calls while holding m_mutex.
  const bool initialized = HALSIM_GetPWMInitialized(pin.channel);
  const int periodMicros =
      PwmPeriodMicros(HALSIM_GetPWMOutputPeriod(pin.channel));
  const int pulseMicros = HALSIM_GetPWMPulseMicrosecond(pin.channel);
  if (!initialized) {
    pin.claimFailedLogged = false;
  }

  std::scoped_lock lock{m_mutex};
  // A new frame length needs a new generator.
  if (pin.applied &&
      (!initialized || pin.appliedPeriodMicros != periodMicros)) {
    m_backend->ReleasePwm(pin.vmxChannel);
    pin.applied = false;
  }
  if (!pin.applied && initialized) {
    if (m_backend->InitPwm(pin.vmxChannel, periodMicros)) {
      pin.applied = true;
      pin.appliedPeriodMicros = periodMicros;
      pin.appliedPulseMicros = -1;  // forces the pulse below to be written
      pin.claimFailedLogged = false;
      std::printf("HALSim Studica: PWM %d -> VMX channel %d (%d us frame)\n",
                  pin.channel, pin.vmxChannel, periodMicros);
    } else if (!pin.claimFailedLogged) {
      pin.claimFailedLogged = true;
      std::fprintf(stderr,
                   "HALSim Studica: cannot claim VMX channel %d for PWM %d\n",
                   pin.vmxChannel, pin.channel);
    }
  }
  if (pin.applied && pin.appliedPulseMicros != pulseMicros) {
    if (m_backend->TrySetPwmPulse(pin.vmxChannel, pulseMicros)) {
      pin.appliedPulseMicros = pulseMicros;
    }
  }
}

void HALSimStudica::PollBattery() {
  if (m_batteryPolls++ % kBatteryPollEvery != 0) {
    return;
  }
  double volts = 0.0;
  bool valid = false;
  {
    std::scoped_lock lock{m_mutex};
    valid = m_backend->GetBatteryVoltage(&volts);
  }
  if (valid) {
    m_batteryFailedLogged = false;
    HALSIM_SetRoboRioVInVoltage(volts);
  }
}
