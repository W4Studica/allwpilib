// Reads a WPILib AnalogInput through the simulation HAL. With halsim_vmx loaded (HALSIM_EXTENSIONS) and a mapping
// from the WPILib channel to a VMX AnalogIn channel (HALSIMVMX_ANALOG_MAP), the voltage on the VMX pin shows up.
//
//   hal_analog_test <channel> [seconds]
//
// Prints the voltage every 250 ms. Touch the pin with a known voltage (3.3 V rail, GND, a potentiometer wiper).

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "wpi/hal/AnalogInput.h"
#include "wpi/hal/HAL.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <channel> [seconds]\n", argv[0]);
    return 2;
  }
  const int channel = std::atoi(argv[1]);
  const int seconds = argc > 2 ? std::atoi(argv[2]) : 10;

  if (!HAL_Initialize()) {  // loads the extensions named in HALSIM_EXTENSIONS
    std::fprintf(stderr, "HAL_Initialize failed\n");
    return 1;
  }
  int32_t status = 0;
  HAL_AnalogInputHandle in = HAL_InitializeAnalogInputPort(channel, nullptr, &status);
  if (status != 0) {
    std::fprintf(stderr, "cannot initialize analog input %d (status %d)\n", channel, status);
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));  // the extension claims the pin from a polling thread
  for (int i = 0; i < seconds * 4; ++i) {
    const double volts = HAL_GetAnalogVoltage(in, &status);
    std::printf("%6.3f V\n", volts);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }

  HAL_FreeAnalogInputPort(in);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  HAL_Shutdown();
  return 0;
}
