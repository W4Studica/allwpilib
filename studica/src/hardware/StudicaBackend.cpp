// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Backend plugin for VMX-pi (WPILIB_WITH_STUDICA=ON), built on the device as
// libstudica_hardware.so and loaded by halsim_studica through
// HALSIMSTUDICA_BACKEND. Calls the unchanged Studica drivers directly using the
// VMX SDK.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "VMXPi.h"
#include "wpi/studica/RtcClock.hpp"
#include "wpi/studica/SharedVMX.hpp"
#include "wpi/studica/detail/backend/BackendPlugin.hpp"
#include "wpi/studica/detail/conversion/DeviceConversions.hpp"
#include "wpi/studica/detail/backend/HardwareBackend.hpp"
#include "wpi/studica/detail/io/InterruptQueue.hpp"
#include "wpi/studica/drivers/AnalogInput.hpp"
#include "wpi/studica/drivers/DIO.hpp"
#include "wpi/studica/drivers/Encoder.hpp"
#include "wpi/studica/drivers/Imu.hpp"

namespace wpi::studica {
namespace {
using detail::InterruptQueue;
// The SDK retains an opaque token, never a pointer to backend-owned memory.
// Late callbacks cannot access a destroyed backend, even while a pin is being
// freed.
std::mutex gInterruptMutex;
std::map<uintptr_t, std::shared_ptr<InterruptQueue>> gInterruptQueues;
uintptr_t gNextInterruptToken = 1;
void OnInterrupt(uint32_t, InterruptEdgeType edge, void* param,
                 uint64_t timestamp) {
  std::shared_ptr<InterruptQueue> queue;
  {
    std::scoped_lock lock{gInterruptMutex};
    auto it = gInterruptQueues.find(reinterpret_cast<uintptr_t>(param));
    if (it == gInterruptQueues.end()) {
      return;
    }
    queue = it->second;
  }
  queue->Push({edge == InterruptEdgeType::RISING_EDGE_INTERRUPT, timestamp, 0});
}
}  // namespace

std::shared_ptr<VMXPi> SharedVMX() {
  // Thread-safe lazy initialization. Never reset: devices may outlive any one
  // backend.
  static std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50);
  return vmx;
}

bool ReadRtc(RtcReading* reading) {
  if (!reading) {
    return false;
  }
  const auto vmx = SharedVMX();
  if (!vmx->IsOpen()) {
    return false;
  }
  for (int attempt = 0; attempt < 2; ++attempt) {
    RtcReading result;
    VMXErrorCode error = 0;
    uint8_t weekday, day, month, year;
    VMXTime::DaylightSavingsAdjustment adjustment;
    if (!vmx->time.GetRTCDate(result.weekday, result.day, result.month,
                              result.year, &error) ||
        !vmx->time.GetRTCTime(result.hour, result.minute, result.second,
                              result.milliseconds, &error) ||
        !vmx->time.GetRTCDate(weekday, day, month, year, &error) ||
        !vmx->time.GetRTCDaylightSavingsAdjustment(adjustment, &error)) {
      return false;
    }
    if (weekday != result.weekday || day != result.day ||
        month != result.month || year != result.year) {
      continue;
    }
    result.daylightSavingsAdjustment = static_cast<int>(adjustment);
    *reading = result;
    return true;
  }
  return false;
}

class StudicaBackend : public HardwareBackend {
 public:
  StudicaBackend() : m_vmx{SharedVMX()} {}
  ~StudicaBackend() override {
    while (!m_interrupts.empty()) {
      ReleaseInterrupt(m_interrupts.begin()->first);
    }
    while (!m_dutyCycles.empty()) {
      ReleaseDutyCycle(m_dutyCycles.begin()->first);
    }
    while (!m_pwm.empty()) {
      ReleasePwm(m_pwm.begin()->first);
    }
    ReleaseI2c();
  }

  bool ChannelBusy(int channel) const {
    if (m_dio.contains(channel) || m_encoders.contains(channel) ||
        m_pwm.contains(channel) || m_dutyCycles.contains(channel) ||
        m_interrupts.contains(channel)) {
      return true;
    }
    for (const auto& [a, b] : m_encoderB) {
      if (b == channel) {
        return true;
      }
    }
    return m_i2cActive && (channel == m_sda || channel == m_scl);
  }

  /// VMXChannelIndex is a uint8_t: an out-of-range number would wrap around to
  /// another channel. The Studica Analog/Encoder drivers report nothing when
  /// they cannot claim a channel, so check here that the channel exists and has
  /// the function we are about to ask for.
  bool ChannelCan(int vmxChannel, VMXChannelCapability capability) const {
    if (vmxChannel < 0 || vmxChannel >= INVALID_VMX_CHANNEL_INDEX) {
      return false;
    }
    return m_vmx->io.ChannelSupportsCapability(
        static_cast<VMXChannelIndex>(vmxChannel), capability);
  }

  bool InitDigital(int vmxChannel, bool isInput) override {
    if (!m_vmx || !m_vmx->IsOpen() || ChannelBusy(vmxChannel) ||
        vmxChannel < 0 || vmxChannel >= INVALID_VMX_CHANNEL_INDEX) {
      return false;
    }
    auto dio = std::make_unique<wpi::studica::drivers::DIO>(
        static_cast<VMXChannelIndex>(vmxChannel),
        isInput ? wpi::studica::drivers::PinMode::INPUT
                : wpi::studica::drivers::PinMode::OUTPUT,
        m_vmx);
    if (!dio->IsInitialized()) {
      return false;
    }
    m_dio.emplace(vmxChannel, std::move(dio));
    return true;
  }

  void ReleaseDigital(int vmxChannel) override { m_dio.erase(vmxChannel); }

  void SetDigital(int vmxChannel, bool value) override {
    if (auto it = m_dio.find(vmxChannel); it != m_dio.end()) {
      it->second->Set(value);
    }
  }

  bool GetDigital(int vmxChannel) override {
    if (auto it = m_dio.find(vmxChannel); it != m_dio.end()) {
      return it->second->Get();
    }
    return false;
  }

  bool InitAnalog(int vmxChannel) override {
    if (!m_vmx || !m_vmx->IsOpen() || m_analog.contains(vmxChannel) ||
        !ChannelCan(vmxChannel, VMXChannelCapability::AccumulatorInput)) {
      return false;
    }
    m_analog.emplace(vmxChannel,
                     std::make_unique<wpi::studica::drivers::AnalogInput>(
                         static_cast<VMXChannelIndex>(vmxChannel), m_vmx));
    return true;
  }

  void ReleaseAnalog(int vmxChannel) override { m_analog.erase(vmxChannel); }

  bool GetAnalogVoltage(int vmxChannel, double* volts) override {
    auto it = m_analog.find(vmxChannel);
    if (it == m_analog.end()) {
      return false;
    }
    float v = 0.0f;
    if (!it->second->GetAverageVoltage(v)) {
      return false;
    }
    *volts = v;
    return true;
  }

  bool InitEncoder(int vmxChannelA, int vmxChannelB) override {
    if (!m_vmx || !m_vmx->IsOpen() || ChannelBusy(vmxChannelA) ||
        ChannelBusy(vmxChannelB) ||
        !ChannelCan(vmxChannelA, VMXChannelCapability::EncoderAInput) ||
        !ChannelCan(vmxChannelB, VMXChannelCapability::EncoderBInput)) {
      return false;
    }
    m_encoders.emplace(vmxChannelA,
                       std::make_unique<wpi::studica::drivers::Encoder>(
                           static_cast<VMXChannelIndex>(vmxChannelA),
                           static_cast<VMXChannelIndex>(vmxChannelB), m_vmx));
    m_encoderB.emplace(vmxChannelA, vmxChannelB);
    return true;
  }

  void ReleaseEncoder(int vmxChannelA) override {
    m_encoders.erase(vmxChannelA);
    m_encoderB.erase(vmxChannelA);
  }

  bool GetEncoderCount(int vmxChannelA, int32_t* count) override {
    auto it = m_encoders.find(vmxChannelA);
    if (it == m_encoders.end()) {
      return false;
    }
    *count = it->second->GetCount();
    return true;
  }

  bool InitImu() override {
    if (!m_vmx || !m_vmx->IsOpen() || m_imu) {
      return false;
    }
    // Use the constructor that takes our VMXPi: Imu() would create another.
    m_imu = std::make_unique<wpi::studica::drivers::Imu>(m_vmx);
    return true;
  }

  void ReleaseImu() override { m_imu.reset(); }

  bool GetImu(ImuSample* sample) override {
    if (!m_imu || !m_imu->IsConnected() || m_imu->IsCalibrating()) {
      return false;
    }
    sample->yawDeg = m_imu->GetYaw();
    sample->pitchDeg = m_imu->GetPitch();
    sample->rollDeg = m_imu->GetRoll();
    sample->gyroXDps = m_imu->GetRawGyroX();
    sample->gyroYDps = m_imu->GetRawGyroY();
    sample->gyroZDps = m_imu->GetRawGyroZ();
    sample->accelXG = m_imu->GetRawAccelX();
    sample->accelYG = m_imu->GetRawAccelY();
    sample->accelZG = m_imu->GetRawAccelZ();
    return true;
  }

  /// PWM goes straight to the VMX API, not through
  /// wpi::studica::drivers::PWM/Servo: those have no way to write a pulse
  /// width, and Servo prints a line on every change. A tick is 4 us (a 20 ms
  /// frame is 5000 ticks, the resolution wpi::studica::drivers::PWM uses), so
  /// duty ticks = microseconds / 4.
  bool InitPwm(int vmxChannel, int periodMicros) override {
    if (!m_vmx || !m_vmx->IsOpen() || ChannelBusy(vmxChannel) ||
        periodMicros < kTickMicros || periodMicros > kTickMicros * 65535 ||
        !ChannelCan(vmxChannel, VMXChannelCapability::PWMGeneratorOutput)) {
      return false;
    }
    PWMGeneratorConfig config(static_cast<uint32_t>(1000000 / periodMicros));
    const auto maxDuty = static_cast<uint16_t>(periodMicros / kTickMicros);
    config.SetMaxDutyCycleValue(maxDuty);
    Pwm pwm;
    pwm.index = static_cast<VMXChannelIndex>(vmxChannel);
    pwm.maxDuty = maxDuty;
    VMXErrorCode err = 0;
    if (!m_vmx->io.ActivateSinglechannelResource(
            VMXChannelInfo(pwm.index, VMXChannelCapability::PWMGeneratorOutput),
            &config, pwm.handle, &err)) {
      std::fprintf(stderr, "HALSim Studica: PWM on VMX channel %d: %s\n",
                   vmxChannel, GetVMXErrorString(err));
      return false;
    }
    m_pwm.emplace(vmxChannel, pwm);
    SetPwmPulse(vmxChannel, 0);
    return true;
  }

  void ReleasePwm(int vmxChannel) override {
    auto it = m_pwm.find(vmxChannel);
    if (it == m_pwm.end()) {
      return;
    }
    SetPwmPulse(vmxChannel, 0);
    VMXErrorCode err = 0;
    m_vmx->io.DeallocateResource(it->second.handle, &err);
    m_pwm.erase(it);
  }

  void SetPwmPulse(int vmxChannel, int pulseMicros) override {
    auto it = m_pwm.find(vmxChannel);
    if (it == m_pwm.end()) {
      return;
    }
    const long ticks =
        std::lround(static_cast<double>(pulseMicros) / kTickMicros);
    const auto duty =
        static_cast<uint16_t>(std::clamp<long>(ticks, 0, it->second.maxDuty));
    VMXErrorCode err = 0;
    if (!m_vmx->io.PWMGenerator_SetDutyCycle(it->second.handle,
                                             it->second.index, duty, &err)) {
      std::fprintf(stderr, "HALSim Studica: PWM duty on VMX channel %d: %s\n",
                   vmxChannel, GetVMXErrorString(err));
    }
  }

  bool InitI2c() override {
    if (!m_vmx || !m_vmx->IsOpen() || m_i2cActive) {
      return false;
    }
    m_sda = m_vmx->io.GetSoleChannelIndex(VMXChannelCapability::I2C_SDA);
    m_scl = m_vmx->io.GetSoleChannelIndex(VMXChannelCapability::I2C_SCL);
    if (!ChannelCan(m_sda, VMXChannelCapability::I2C_SDA) ||
        !ChannelCan(m_scl, VMXChannelCapability::I2C_SCL) ||
        ChannelBusy(m_sda) || ChannelBusy(m_scl)) {
      return false;
    }
    I2CConfig config;
    VMXErrorCode err = 0;
    m_i2cActive = m_vmx->io.ActivateDualchannelResource(
        VMXChannelInfo(m_sda, VMXChannelCapability::I2C_SDA),
        VMXChannelInfo(m_scl, VMXChannelCapability::I2C_SCL), &config,
        m_i2cHandle, &err);
    return m_i2cActive;
  }
  void ReleaseI2c() override {
    if (!m_i2cActive) {
      return;
    }
    VMXErrorCode err = 0;
    m_vmx->io.DeallocateResource(m_i2cHandle, &err);
    m_i2cActive = false;
  }
  bool TransactionI2c(int address, const uint8_t* tx, int txSize, uint8_t* rx,
                      int rxSize) override {
    if (!m_i2cActive || address < 0 || address > 0x7f || txSize < 0 ||
        rxSize < 0 || txSize > UINT16_MAX || rxSize > UINT16_MAX ||
        (txSize && !tx) || (rxSize && !rx)) {
      return false;
    }
    // SDK takes a mutable transmit buffer. Never expose the caller's const
    // memory.
    std::vector<uint8_t> buffer;
    if (txSize) {
      buffer.assign(tx, tx + txSize);
    }
    VMXErrorCode err = 0;
    return m_vmx->io.I2C_Transaction(
        m_i2cHandle, static_cast<uint8_t>(address), buffer.data(),
        static_cast<uint16_t>(txSize), rx, static_cast<uint16_t>(rxSize), &err);
  }
  bool InitDutyCycle(int channel) override {
    if (!m_vmx || !m_vmx->IsOpen() || ChannelBusy(channel) ||
        !ChannelCan(channel, VMXChannelCapability::PWMCaptureInput)) {
      return false;
    }
    PWMCaptureConfig config(1, PWMCaptureConfig::x2);
    VMXResourceHandle handle;
    VMXErrorCode err = 0;
    if (!m_vmx->io.ActivateSinglechannelResource(
            VMXChannelInfo(channel, VMXChannelCapability::PWMCaptureInput),
            &config, handle, &err)) {
      return false;
    }
    m_dutyCycles.emplace(channel, handle);
    return true;
  }
  void ReleaseDutyCycle(int channel) override {
    auto it = m_dutyCycles.find(channel);
    if (it == m_dutyCycles.end()) {
      return;
    }
    VMXErrorCode err = 0;
    m_vmx->io.DeallocateResource(it->second, &err);
    m_dutyCycles.erase(it);
  }
  bool GetDutyCycle(int channel, DutyCycleReading* reading) override {
    auto it = m_dutyCycles.find(channel);
    if (it == m_dutyCycles.end()) {
      return false;
    }
    uint32_t period = 0, high = 0;
    VMXErrorCode err = 0;
    if (!m_vmx->io.PWMCapture_GetCount(it->second, period, high, &err)) {
      return false;
    }
    *reading = ConvertDutyCycle(period, high);
    return true;
  }
  bool InitInterrupt(int channel) override {
    if (!m_vmx || !m_vmx->IsOpen() || ChannelBusy(channel) ||
        !ChannelCan(channel, VMXChannelCapability::InterruptInput)) {
      return false;
    }
    Interrupt interrupt;
    interrupt.queue = std::make_shared<InterruptQueue>();
    {
      std::scoped_lock lock{gInterruptMutex};
      if (gNextInterruptToken == 0) {
        return false;
      }
      interrupt.token = gNextInterruptToken++;
      gInterruptQueues.emplace(interrupt.token, interrupt.queue);
    }
    InterruptConfig config(InterruptConfig::BOTH, OnInterrupt,
                           reinterpret_cast<void*>(interrupt.token));
    VMXErrorCode err = 0;
    if (!m_vmx->io.ActivateSinglechannelResource(
            VMXChannelInfo(channel, VMXChannelCapability::InterruptInput),
            &config, interrupt.handle, &err)) {
      std::scoped_lock lock{gInterruptMutex};
      gInterruptQueues.erase(interrupt.token);
      return false;
    }
    m_interrupts.emplace(channel, std::move(interrupt));
    return true;
  }
  void ReleaseInterrupt(int channel) override {
    auto it = m_interrupts.find(channel);
    if (it == m_interrupts.end()) {
      return;
    }
    {
      std::scoped_lock lock{gInterruptMutex};
      gInterruptQueues.erase(it->second.token);
    }
    VMXErrorCode err = 0;
    m_vmx->io.Interrupt_SetEnabled(it->second.handle, false, &err);
    m_vmx->io.DeallocateResource(it->second.handle, &err);
    m_interrupts.erase(it);
  }
  bool ReadInterrupt(int channel, InterruptEvent* event) override {
    auto it = m_interrupts.find(channel);
    if (it == m_interrupts.end()) {
      return false;
    }
    return it->second.queue->Pop(event);
  }

  bool GetBatteryVoltage(double* volts) override {
    if (!m_vmx || !m_vmx->IsOpen()) {
      return false;
    }
    float v = 0.0f;
    VMXErrorCode err = 0;
    if (!m_vmx->power.GetSystemVoltage(v, &err)) {
      return false;
    }
    *volts = v;
    return true;
  }

 private:
  static constexpr int kTickMicros = 4;

  struct Pwm {
    VMXResourceHandle handle{};
    VMXChannelIndex index = 0;
    uint16_t maxDuty = 0;
  };

  struct Interrupt {
    VMXResourceHandle handle{};
    uintptr_t token = 0;
    std::shared_ptr<InterruptQueue> queue;
  };
  std::map<int, Interrupt> m_interrupts;
  std::map<int, VMXResourceHandle> m_dutyCycles;
  std::map<int, int> m_encoderB;
  VMXResourceHandle m_i2cHandle{};
  bool m_i2cActive = false;
  VMXChannelIndex m_sda = INVALID_VMX_CHANNEL_INDEX;
  VMXChannelIndex m_scl = INVALID_VMX_CHANNEL_INDEX;
  std::shared_ptr<VMXPi> m_vmx;
  std::map<int, Pwm> m_pwm;
  std::map<int, std::unique_ptr<wpi::studica::drivers::DIO>> m_dio;
  std::map<int, std::unique_ptr<wpi::studica::drivers::AnalogInput>> m_analog;
  std::map<int, std::unique_ptr<wpi::studica::drivers::Encoder>> m_encoders;
  std::unique_ptr<wpi::studica::drivers::Imu> m_imu;
};

}  // namespace wpi::studica

extern "C" {

WPILIBSTUDICA_PLUGIN_EXPORT wpilibstudica_BackendApi*
wpilibstudica_CreateBackendV3(void) {
  try {
    return wpi::studica::MakeBackendApi(
        std::make_unique<wpi::studica::StudicaBackend>());
  } catch (...) {
    return nullptr;
  }
}

WPILIBSTUDICA_PLUGIN_EXPORT void wpilibstudica_DestroyBackendV3(
    wpilibstudica_BackendApi* api) {
  wpi::studica::DestroyBackendApi(api);
}

}  // extern "C"
