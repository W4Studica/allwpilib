// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>

#include "wpi/hal/simulation/DIOData.h"
#include "wpi/hal/simulation/DigitalEdgeData.h"
#include "wpi/halsim/studica/HALSimStudica.hpp"

using namespace wpi::studica;

bool HALSimStudica::PollInterrupt(DioPin& pin, bool reserved) {
  const bool requested = !reserved && HALSIM_GetDIOInitialized(pin.channel) &&
                         HALSIM_GetDIOIsInput(pin.channel) &&
                         HALSIM_GetDIOEdgeCaptureEnabled(pin.channel);
  std::vector<InterruptEvent> events;
  {
    std::scoped_lock lock{m_mutex};
    if (pin.interruptApplied && !requested) {
      m_backend->ReleaseInterrupt(pin.vmxChannel);
      pin.interruptApplied = false;
    }
    if (!requested) {
      return false;
    }
    if (pin.applied) {
      m_backend->ReleaseDigital(pin.vmxChannel);
      pin.applied = false;
    }
    if (!pin.interruptApplied) {
      pin.interruptApplied = m_backend->InitInterrupt(pin.vmxChannel);
      if (!pin.interruptApplied && !pin.claimFailedLogged) {
        std::fprintf(stderr,
                     "HALSim Studica: cannot capture edges on VMX channel %d\n",
                     pin.vmxChannel);
        pin.claimFailedLogged = true;
      }
    }
    if (pin.interruptApplied) {
      pin.claimFailedLogged = false;
      InterruptEvent event;
      // Bound work per pass even if hardware continuously produces edges.
      for (int i = 0;
           i < 256 && m_backend->ReadInterrupt(pin.vmxChannel, &event); ++i) {
        events.push_back(event);
      }
    }
  }
  for (const auto& event : events) {
    HALSIM_SetDIOValue(pin.channel, event.rising);
    HALSIM_NotifyDIOEdge(pin.channel, event.rising, event.timestampMicros,
                         event.dropped);
  }
  return true;
}
