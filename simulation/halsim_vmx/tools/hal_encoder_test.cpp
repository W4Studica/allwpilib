// Reads a WPILib quadrature encoder through the simulation HAL. With halsim_vmx loaded (HALSIM_EXTENSIONS) and the
// A/B channels mapped (HALSIMVMX_DIO_MAP; the pair must be a FlexDIO encoder pair, A even and B odd), turning the
// encoder moves the count.
//
//   hal_encoder_test <a-channel> <b-channel> [seconds]
//
// Prints the count every 250 ms. Half way through, the count is reset once (the count must go back to ~0).

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "wpi/hal/Encoder.h"
#include "wpi/hal/HAL.h"

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <a-channel> <b-channel> [seconds]\n", argv[0]);
    return 2;
  }
  const int a = std::atoi(argv[1]);
  const int b = std::atoi(argv[2]);
  const int seconds = argc > 3 ? std::atoi(argv[3]) : 20;

  if (!HAL_Initialize()) {
    std::fprintf(stderr, "HAL_Initialize failed\n");
    return 1;
  }
  int32_t status = 0;
  HAL_EncoderHandle enc = HAL_InitializeEncoder(a, b, false, HAL_ENCODER_4X_ENCODING, &status);
  if (status != 0) {
    std::fprintf(stderr, "cannot initialize encoder %d/%d (status %d)\n", a, b, status);
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const int steps = seconds * 4;
  for (int i = 0; i < steps; ++i) {
    if (i == steps / 2) {
      HAL_ResetEncoder(enc, &status);
      std::printf("-- reset --\n");
    }
    std::printf("count %d\n", HAL_GetEncoder(enc, &status));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }

  HAL_FreeEncoder(enc);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  HAL_Shutdown();
  return 0;
}
