// Exercises TitanEnableGuard on a real Titan, driven by the simulated Driver Station, and (optionally) reads a
// WPILib Encoder through halsim_vmx at the same time.
//
//   hal_titan_test <titan-can-id> <motor 0-3> <speed 0..1> [encoder-a encoder-b]
//
// LIFT THE WHEELS OFF THE GROUND. The motor is commanded at <speed> the whole time (a command every 50 ms); only the
// enable state changes, so the motor must turn only in the "enabled" phase:
//
//   1. disabled   2 s   motor must NOT turn
//   2. enabled    3 s   motor turns
//   3. disabled   2 s   motor stops right away
//   4. enabled    2 s   motor turns again
//   5. e-stopped  2 s   motor stops right away
//
// The Titan is created from wpilibvmx::SharedVMX() (the same VMXPi halsim_vmx uses). Every 250 ms it prints the
// Titan's own encoder count and RPM; with an encoder pair it also prints the WPILib Encoder count.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "titan.hpp"
#include "wpi/halsim/vmx/SharedVMX.hpp"
#include "wpi/halsim/vmx/TitanEnableGuard.hpp"
#include "wpi/hal/Encoder.h"
#include "wpi/hal/HAL.h"
#include "wpi/hal/simulation/DriverStationData.h"

using namespace std::chrono_literals;

static void SetDs(bool enabled, bool estop) {
  HALSIM_SetDriverStationDsAttached(true);
  HALSIM_SetDriverStationEnabled(enabled);
  HALSIM_SetDriverStationEStop(estop);
  HALSIM_NotifyDriverStationNewData();
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::fprintf(stderr, "usage: %s <titan-can-id> <motor 0-3> <speed 0..1> [encoder-a encoder-b]\n", argv[0]);
    return 2;
  }
  const uint8_t canId = static_cast<uint8_t>(std::atoi(argv[1]));
  const uint8_t motor = static_cast<uint8_t>(std::atoi(argv[2]));
  const double speed = std::atof(argv[3]);
  if (motor > 3 || speed < -1.0 || speed > 1.0) {
    std::fprintf(stderr, "motor must be 0-3 and speed -1..1\n");
    return 2;
  }
  const bool haveEncoder = argc > 5;

  if (!HAL_Initialize()) {
    std::fprintf(stderr, "HAL_Initialize failed\n");
    return 1;
  }
  SetDs(false, false);

  int32_t status = 0;
  HAL_EncoderHandle enc = HAL_INVALID_HANDLE;
  if (haveEncoder) {
    enc = HAL_InitializeEncoder(std::atoi(argv[4]), std::atoi(argv[5]), false, HAL_ENCODER_4X_ENCODING, &status);
    if (status != 0) {
      std::fprintf(stderr, "cannot initialize encoder (status %d)\n", status);
      return 1;
    }
  }

  studica_driver::Titan titan(canId, 15600, 0.0006830601f, wpilibvmx::SharedVMX());
  std::this_thread::sleep_for(1s);  // required after configuring the Titan
  std::printf("Titan serial %s, firmware %s\n", titan.GetSerialNumber().c_str(), titan.GetFirmwareVersion().c_str());
  titan.ResetEncoder(motor);

  wpilibvmx::TitanEnableGuard<studica_driver::Titan> guard{titan};  // declared after the Titan
  guard.Start();

  auto phase = [&](const char* name, bool enabled, bool estop, int ms) {
    std::printf("--- %s ---\n", name);
    SetDs(enabled, estop);
    for (int t = 0; t < ms; t += 50) {
      titan.SetSpeed(motor, speed);  // always commanded: only the enable state may decide whether it turns
      if (t % 250 == 0) {
        if (haveEncoder) {
          std::printf("titan count %d  rpm %.1f   wpilib encoder %d\n", titan.GetEncoderCount(motor),
                      titan.GetRPM(motor), HAL_GetEncoder(enc, &status));
        } else {
          std::printf("titan count %d  rpm %.1f\n", titan.GetEncoderCount(motor), titan.GetRPM(motor));
        }
      }
      std::this_thread::sleep_for(50ms);
    }
  };

  phase("1. disabled: motor must NOT turn", false, false, 2000);
  phase("2. enabled: motor turns", true, false, 3000);
  phase("3. disabled: motor must stop", false, false, 2000);
  phase("4. enabled: motor turns", true, false, 2000);
  phase("5. e-stopped: motor must stop", true, true, 2000);

  titan.SetSpeed(motor, 0);
  guard.Stop();  // sends Enable(false)
  if (haveEncoder) {
    HAL_FreeEncoder(enc);
  }
  std::this_thread::sleep_for(100ms);
  HAL_Shutdown();
  return 0;
}
