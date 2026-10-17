// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/vmx/TerminationHandlers.hpp"

#if !defined(_WIN32)

#include <signal.h>

namespace {

constexpr int kSignals[] = {SIGINT, SIGTERM};
struct sigaction gPrevious[2];

void OnTerminate(int sig, siginfo_t* info, void* context) {
  for (int i = 0; i < 2; ++i) {
    if (kSignals[i] != sig) {
      continue;
    }
    const struct sigaction& previous = gPrevious[i];
    if (previous.sa_flags & SA_SIGINFO) {
      if (previous.sa_sigaction != nullptr) {
        previous.sa_sigaction(sig, info, context);
      }
    } else if (previous.sa_handler != SIG_DFL && previous.sa_handler != SIG_IGN) {
      previous.sa_handler(sig);
    }
  }
  // The hardware is released. End like a program without a handler: restore the default action and let the
  // signal, still pending while this handler runs, take effect when it returns.
  signal(sig, SIG_DFL);
  raise(sig);
}

bool HasHandler(const struct sigaction& action) {
  if (action.sa_flags & SA_SIGINFO) {
    return action.sa_sigaction != nullptr;
  }
  return action.sa_handler != SIG_DFL && action.sa_handler != SIG_IGN;
}

}  // namespace

void wpilibvmx::InstallTerminationHandlers() {
  for (int i = 0; i < 2; ++i) {
    struct sigaction current;
    if (sigaction(kSignals[i], nullptr, &current) != 0 || !HasHandler(current)) {
      continue;
    }
    gPrevious[i] = current;
    struct sigaction action;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_SIGINFO;
    action.sa_sigaction = OnTerminate;
    sigaction(kSignals[i], &action, nullptr);
  }
}

#else

void wpilibvmx::InstallTerminationHandlers() {}

#endif
