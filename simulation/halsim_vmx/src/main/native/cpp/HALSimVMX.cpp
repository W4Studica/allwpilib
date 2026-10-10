// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/vmx/HALSimVMX.hpp"

#include <chrono>
#include <utility>

#include "wpi/hal/Ports.h"
#include "wpi/hal/simulation/DIOData.h"

using namespace wpilibvmx;

namespace {
constexpr auto kPollPeriod = std::chrono::milliseconds(10);
}  // namespace

HALSimVMX::HALSimVMX(std::unique_ptr<VmxBackend> backend, ChannelMap dioMap)
    : m_backend{std::move(backend)}, m_dioMap{std::move(dioMap)} {
  // Pins are fixed for the lifetime of the object: callbacks keep pointers
  // into this vector, so it must never reallocate after Start().
  int numChannels = HAL_GetNumDigitalChannels();
  m_dio.resize(numChannels);
  for (int ch = 0; ch < numChannels; ++ch) {
    m_dio[ch].owner = this;
    m_dio[ch].channel = ch;
    if (auto vmx = m_dioMap.Get(ch)) {
      m_dio[ch].vmxChannel = *vmx;
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
}

void HALSimVMX::Poll() {
  for (auto& pin : m_dio) {
    if (pin.vmxChannel >= 0) {
      PollDio(pin);
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

void HALSimVMX::PollDio(DioPin& pin) {
  // Never call HALSIM_* while holding m_mutex: the sim HAL may hold its own
  // lock while running OnDioValue, which takes m_mutex.
  const bool initialized = HALSIM_GetDIOInitialized(pin.channel);
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
        if (!isInput) {
          m_backend->SetDigital(pin.vmxChannel, simValue);
        }
      }
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
