// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>

#include "wpi/hal/simulation/DutyCycleData.h"
#include "wpi/halsim/studica/HALSimStudica.hpp"

using namespace wpi::studica;

void HALSimStudica::PollDutyCycle(DutyCyclePin& pin) {
  const bool initialized = HALSIM_GetDutyCycleInitialized(pin.channel);
  DutyCycleReading reading;
  bool valid = false;
  {
    std::scoped_lock lock{m_mutex};
    if (pin.applied && !initialized) {
      m_backend->ReleaseDutyCycle(pin.vmxChannel);
      pin.applied = false;
    }
    if (!initialized) {
      pin.claimFailedLogged = false;
    }
    if (initialized && !pin.applied) {
      pin.applied = m_backend->InitDutyCycle(pin.vmxChannel);
      if (!pin.applied && !pin.claimFailedLogged) {
        std::fprintf(
            stderr,
            "HALSim Studica: cannot capture duty cycle on VMX channel %d\n",
            pin.vmxChannel);
        pin.claimFailedLogged = true;
      }
    }
    if (pin.applied) {
      valid = m_backend->GetDutyCycle(pin.vmxChannel, &reading);
    }
  }
  // Publish a disconnected signal on failure instead of retaining a stale valid
  // frequency.
  HALSIM_SetDutyCycleFrequency(pin.channel, valid ? reading.frequencyHz : 0);
  HALSIM_SetDutyCycleOutput(pin.channel, valid ? reading.output : 0);
}
