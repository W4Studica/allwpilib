// Loopback test of two VMX channels with the Studica DIO class directly: no HAL, no halsim_vmx.
// It separates a wiring / channel-number problem from a halsim_vmx problem.
//
//   dio_probe <output-vmx-channel> <input-vmx-channel>
//
// Wire the two channels together first. The Studica DIO opens inputs with a PULL-UP, so an input that is
// not connected to anything reads 1 whatever the output does.
// Uses one VMXPi. Run as root, and only one process may use the VMX at a time.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>

#include "VMXPi.h"
#include "dio.hpp"

static void Sleep(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <output-vmx-channel> <input-vmx-channel>\n", argv[0]);
    return 2;
  }
  const int outCh = std::atoi(argv[1]);
  const int inCh = std::atoi(argv[2]);

  auto vmx = std::make_shared<VMXPi>(true, 50);
  if (!vmx->IsOpen()) {
    std::fprintf(stderr, "VMX is not open (run as root; only one process may use the VMX)\n");
    return 1;
  }

  studica_driver::DIO out(static_cast<VMXChannelIndex>(outCh), studica_driver::PinMode::OUTPUT, vmx);
  studica_driver::DIO in(static_cast<VMXChannelIndex>(inCh), studica_driver::PinMode::INPUT, vmx);
  if (!out.IsInitialized() || !in.IsInitialized()) {
    std::fprintf(stderr, "could not claim the channels (output ok: %d, input ok: %d)\n", out.IsInitialized(),
                 in.IsInitialized());
    return 1;
  }

  Sleep(200);
  std::printf("input before driving anything: %d (1 is expected for an input with a pull-up)\n", in.Get() ? 1 : 0);

  int mismatches = 0;
  for (int i = 0; i < 10; ++i) {
    const int value = i % 2;
    out.Set(value != 0);
    Sleep(150);
    const int read = in.Get() ? 1 : 0;
    std::printf("wrote %d, read %d%s\n", value, read, read == value ? "" : "   <-- MISMATCH");
    if (read != value) {
      ++mismatches;
    }
  }
  std::printf(mismatches == 0 ? "loopback OK\n" : "loopback FAILED (%d mismatches)\n", mismatches);
  return mismatches == 0 ? 0 : 1;
}
