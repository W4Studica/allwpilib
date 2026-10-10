// Drives WPILib DIO through the simulation HAL. With halsim_vmx loaded (HALSIM_EXTENSIONS) and a mapping from the
// WPILib channels to VMX channels (HALSIMVMX_DIO_MAP), the pins on the VMX move.
//
//   hal_dio_test <output-channel> [input-channel]
//
// With an input channel, wire the two VMX pins together (output -> input) and the test checks the loopback.
// Without one, watch the output pin with a multimeter or LED. Channel numbers are WPILib DIO numbers.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "wpi/hal/DIO.h"
#include "wpi/hal/HAL.h"

static void Sleep(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <output-channel> [input-channel]\n", argv[0]);
    return 2;
  }
  const int outChannel = std::atoi(argv[1]);
  const int inChannel = argc > 2 ? std::atoi(argv[2]) : -1;

  if (!HAL_Initialize()) {  // loads the extensions named in HALSIM_EXTENSIONS
    std::fprintf(stderr, "HAL_Initialize failed\n");
    return 1;
  }

  int32_t status = 0;
  HAL_DigitalHandle out = HAL_InitializeDIOPort(outChannel, false, nullptr, &status);
  if (status != 0) {
    std::fprintf(stderr, "cannot initialize output DIO %d (status %d)\n", outChannel, status);
    return 1;
  }
  HAL_DigitalHandle in = HAL_INVALID_HANDLE;
  if (inChannel >= 0) {
    in = HAL_InitializeDIOPort(inChannel, true, nullptr, &status);
    if (status != 0) {
      std::fprintf(stderr, "cannot initialize input DIO %d (status %d)\n", inChannel, status);
      return 1;
    }
  }

  Sleep(300);  // halsim_vmx claims the pins from a polling thread (10 ms)

  int mismatches = 0;
  for (int i = 0; i < 10; ++i) {
    const int value = i % 2;
    HAL_SetDIO(out, value, &status);
    Sleep(150);
    if (in != HAL_INVALID_HANDLE) {
      const int read = HAL_GetDIO(in, &status);
      std::printf("wrote %d, read %d%s\n", value, read, read == value ? "" : "   <-- MISMATCH");
      if (read != value) {
        ++mismatches;
      }
    } else {
      std::printf("wrote %d\n", value);
    }
  }

  HAL_FreeDIOPort(out);
  if (in != HAL_INVALID_HANDLE) {
    HAL_FreeDIOPort(in);
  }
  Sleep(100);
  HAL_Shutdown();  // runs the extension's shutdown handler, which releases the VMX pins

  if (inChannel >= 0) {
    std::printf(mismatches == 0 ? "loopback OK\n" : "loopback FAILED (%d mismatches)\n", mismatches);
    return mismatches == 0 ? 0 : 1;
  }
  return 0;
}
