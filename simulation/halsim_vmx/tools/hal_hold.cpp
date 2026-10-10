// Initializes the simulation HAL (loading the extensions in HALSIM_EXTENSIONS) and then just waits, like a
// robot program does. Use it to try robot_manager's stop sequence with a real HAL process.
//
//   hal_hold [seconds]     (default: wait until a signal ends the process)
//
// It prints "hal_hold ready" once the HAL is up, and "hal_hold exiting" on a normal exit.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "wpi/hal/HAL.h"

int main(int argc, char** argv) {
  const int seconds = argc > 1 ? std::atoi(argv[1]) : -1;
  if (!HAL_Initialize()) {
    std::fprintf(stderr, "HAL_Initialize failed\n");
    return 1;
  }
  std::printf("hal_hold ready\n");
  std::fflush(stdout);

  if (seconds >= 0) {
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
  } else {
    for (;;) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  HAL_Shutdown();
  std::printf("hal_hold exiting\n");
  return 0;
}
