# VMX Hardware Verification Checklist

This checklist was started before anything had been verified on physical hardware. Hardware results were recorded on one development VMX-pi on 2026-10-10 to 11 and are summarized in the **Results table** at the end; unchecked `[ ]` items are still open. Later work (`ControlLoopWatchdog`, RioLog, installer hardening, release builds) has PC tests only. Run the PC tests (see `CLAUDE.md`) for current counts instead of trusting a number written here.
This document lists the hardware checks in order. Each step describes what to do, the expected result, and what to do if it fails.

- Check `[ ]` items as they are completed and fill in the **Results table** at the end.
- "Untested command" means the author has not run it. If it does not work as written, fix the command and update this document.
- **For tests involving motors (§6 and §7), raise the robot so its wheels cannot touch the floor, start at low speed, and keep a way to cut power immediately.**

Dependencies: §0 → §1 → (§2, §3, and §4 can run in parallel) → §5 → §6 → §7 → §8. Later steps are not meaningful if an earlier prerequisite fails.

---

## 0. Record the environment

Write the command output in the Results table.

```bash
cat /etc/os-release | head -4
uname -r
cat /proc/device-tree/model; echo        # Raspberry Pi model
ldd --version | head -1                  # glibc
g++ --version | head -1                  # record "not installed" if absent
java -version 2>&1 | head -1             # record "not installed" if absent
python3 --version
```

- [x] Recorded OS, kernel, Pi model, glibc, g++, Java, and Python. (Environment above: Ubuntu 26.04.1, kernel 7.0.0-1020-raspi, Pi 4 Rev 1.5, glibc 2.43, g++ 15.2, Java 25, Python 3.14.)

**Values already confirmed (2026-10-10, one development VMX, from SSH login output):** Ubuntu 26.04.1 LTS, kernel `7.0.0-1020-raspi` (aarch64), glibc 2.43, OpenJDK 25.0.4.1 (`openjdk-25-jre-headless`), and successful installation of `vmxpi-hal_1.0~20240704`.
Also confirmed on the same robot: **Raspberry Pi 4 Model B Rev 1.5**, Python 3.14.4, **g++ not installed**, **`/usr/local/include/vmxpi` missing (no headers)**, and all dependencies of `libvmxpi_hal_cpp.so` (`libstdc++`, `libgcc_s`, `libc`, `libm`, `ld-linux`) resolve (**no `not found` from `ldd`**).
Devices present: `/dev/mem` (root:kmem 0640), `/dev/gpiomem` (root 0600), `/dev/i2c-1` (dialout), and `/dev/spidev0.0` / `0.1` (dialout).
Kernel configuration: `CONFIG_DEVMEM=y`, `CONFIG_STRICT_DEVMEM=y`, and `IO_STRICT_DEVMEM` disabled. Peripheral register access may be allowed, but **this must be verified by running it as root**.
The kernel is much newer than 6.6, so a pin-number offset (§3) may be present.
**Because g++ is missing, run `sudo apt install -y g++ make` before §4 (plugin build). `VMXPi.h` must still be obtained.**

Reference (confirmed on this PC): WPILib 2027 provides only the `aarch64-trixie-linux-gnu` arm64 toolchain (Debian 13, GCC 14.3, glibc 2.41).
Studica documentation says the official VMX image uses Ubuntu 22.04 (glibc 2.35).

## 1. Does the base Studica stack work on this OS?

If this fails, nothing else matters. Whether `vmxpi_hal_cpp` / `pigpio` work on this OS determines the OS choice (22.04 vs 26.04 vs Debian 13).

1. Check whether `vmxpi_hal_cpp` is installed: `ls /usr/local/include/vmxpi/VMXPi.h /usr/local/lib/vmxpi/`
   (Studica's ROS2 README says it is preinstalled on the official image. For other OSes, follow Studica's installation documentation.)

   > **Inspection only (not execution) of `vmxpi-hal_1.0~20240704_arm64.deb` (an unofficial repackaging found in the project root, extracted from the official Ubuntu 22.04 ROS2 Humble image):**
   > - It contains only libraries (such as `/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so`), **not the `VMXPi.h` header**. This deb alone cannot compile `studica_drivers` or the plugin. Get the header from `/usr/local/include/vmxpi/` on the official image.
   > - `libvmxpi_hal_cpp.so` requires at most `GLIBC_2.34` and `GLIBCXX_3.4.29`; its only external libraries are `libstdc++`, `libgcc_s`, and `libc`. It may load on OS versions newer than 22.04 due to backward compatibility (not tested by running it).
   > - pigpio appears to be included in the library rather than provided as a separate `.so` (`PIGPIOClient` symbol). Its behavior is unknown.
   >
   > **A second deb includes the headers: `vmx-hal_1.0~20240704_arm64.deb`** (also an unofficial repackaging, 13 MB, description: "libraries, headers, examples and tools").
   > - It contains 23 headers including `/usr/local/include/vmxpi/VMXPi.h`, `libvmxpi_hal_cpp.so` / `.a`, the `studica_drivers` library, test tools under `/usr/local/bin`, and `/usr/local/startup/initenv.sh` (only creates YDLIDAR udev rules; it is not run automatically during installation). Its installation script only runs `ldconfig`.
   > - **It installs the same files as `vmxpi-hal` under `/usr/local/lib/vmxpi/*`, so they conflict.** First run `sudo apt remove -y vmxpi-hal`, then `sudo apt install -y ./vmx-hal_1.0~20240704_arm64.deb`.
   > - Compile-only test on this PC using these headers (not linked or run; x86 object files): all 17 `studica_drivers` `.cpp` files compiled successfully, and our `StudicaBackend.cpp` compiled with no `-Wall -Wextra` warnings.
2. Build and install `studica_drivers` (Studica ROS2 README procedure):
   ```bash
   cd allwpilib/studica_drivers && make && sudo make install
   ```
3. Run an example **as root** (`pigpio` requires root):
   ```bash
   cd allwpilib/studica_drivers/examples/imu_example && make && sudo ./imu_example
   ```
   If a Titan is connected, also try `titan_example` with the wheels raised.

- [x] `VMXPi.h` and `libvmxpi_hal_cpp` are present (`vmx-hal` deb; confirmed on the robot).
- [ ] `studica_drivers` builds and installs (the deb already installed an older version; installing a fresh copy with `install.sh --upgrade-drivers` is unverified).
- [x] **The HAL communicates with hardware on this OS:** established communication with VMX board (model 0x32, hardware rev 60, firmware 3.0.436) and detected navX (with one instance). Reading values with an example has not been tested.

**Expected:** Values are readable without errors.
**If it fails:** Save the complete log. A `pigpio` error means the VMX HAL does not work on this OS. In that case, revert to OS 22.04 and solve the glibc issue in §5 another way (compatible sysroot or on-device build).

## 2. Do multiple `VMXPi` instances in one process cause problems?

Studica class constructors default to `std::make_shared<VMXPi>(true, 50)`, so creating a device without passing an instance creates another `VMXPi`. Since `studica_drivers` cannot be modified, our guidance passes `wpilibvmx::SharedVMX()`. Verify that this is actually required.

Create a small program (command untested):

```cpp
// two_vmx.cpp
#include <iostream>
#include <memory>
#include "VMXPi.h"
int main() {
  auto a = std::make_shared<VMXPi>(true, 50);
  std::cout << "first IsOpen: " << a->IsOpen() << std::endl;
  auto b = std::make_shared<VMXPi>(true, 50);
  std::cout << "second IsOpen: " << b->IsOpen() << std::endl;
  return 0;
}
```
```bash
g++ -std=c++17 two_vmx.cpp -o two_vmx -I/usr/local/include/vmxpi -L/usr/local/lib/vmxpi -lvmxpi_hal_cpp -lrt -lpthread -latomic
sudo ./two_vmx
```

- [x] **Result (2026-10-10, robot Ubuntu 26.04 / kernel 7.0): creating more than one `VMXPi` in a process breaks.**
  - **Two instances:** both report `IsOpen: 1`, but SPI writes fail (`Timeout waiting for comm ready HIGH during SPI transmit. Likely CRC ERROR`, `Write CRC Mismatches: 135`, `Write Failures: 45`). Then retries of `Aborted read from bank 0, address 4, length 108` continue indefinitely and the process **segfaults (exit code 139)**.
  - **One instance** (`vmx_n 1`): works. Logs `Established communication with VMX board model 0x32, hardware rev 60, firmware version 3.0.436` and `Acquired navX-Sensor configuration`; 19 writes and 9,172 reads, zero CRC mismatches, zero failures, normal exit.
  - **Conclusion: `SharedVMX()` is required.** Creating Studica classes without passing it (which creates a new `VMXPi` by default) breaks hardware communication.

**Interpretation:**
- Both report `1` and there is no crash → the default-argument trap is not a problem. The `SharedVMX()` guidance could be downgraded to a recommendation.
- The second reports `0` or crashes → there is a real problem. Require `SharedVMX()` and state this prominently in the docs.
- (Reference) Whether separate **processes** can open the VMX simultaneously does not need testing for our architecture (WPILib uses one process; `robot_manager` does not open the VMX).

## 3. Pin numbers and offsets (values for the `robot_manager` profile)

User recollection: an offset must be subtracted from pin numbers depending on OS/kernel version. **The versions and values are unknown (unverified).** Measure them.

1. For each channel in use, connect an **output pin to an input pin** with a jumper wire, or attach an LED/multimeter to the output.
2. Toggle the output with `studica_driver::DIO` and record which physical pin responds (compare with the VMX-pi pin map).
3. Check input, analog, and encoder pins the same way.

- [ ] Created a `VMXChannelIndex` ↔ physical-pin map → add it to the table in §4 of `DESIGN.md`. **Pending:** channel numbers were established from `vmx_channels` measurements in §4 of `DESIGN.md`, but the correspondence to the pin labels printed on the board is not yet known.
- [ ] Checked whether channel numbers vary by kernel/OS version. If they do, record the first affected version ____ and offset ____. **On this image (kernel 7.0), DIO loopback works with no offset.** The OS image is fixed, so no further check is planned.
  - **Result (2026-10-10, kernel 7.0.0-1020-raspi):** Opening the HAL prints `RPI GPIO Interrupt Enable: PI_BAD_ISR_INIT.` three times. Root cause confirmed: **`/sys/class/gpio` is absent and `# CONFIG_GPIO_SYSFS is not set`.** pigpio GPIO interrupts use sysfs GPIO, so **they cannot be initialized on this kernel.** The remembered "offset to subtract from the pin number" likely refers to kernels around 6.6 where sysfs was enabled. Here, the issue is missing interrupt support, not an offset. **Whether pin reads/writes need an offset is still unknown** (check with the pin-toggle test below).
  - Check with: `ls /sys/class/gpio; for c in /sys/class/gpio/gpiochip*; do echo $c $(cat $c/base) $(cat $c/ngpio) $(cat $c/label); done; grep GPIO_SYSFS /boot/config-$(uname -r)`
  - Impact: features using interrupts (such as `studica_driver::DIO::EnableInterrupt`). DIO/Analog/Encoder/IMU in our `halsim_vmx` do not use interrupts; they poll.
  - **Fix (in `gpio_isr_shim/` in the Robot-Manager repository):** `LD_PRELOAD` replaces `gpioSetISRFunc*` with GPIO character-device-based implementations. On the robot, `PI_BAD_ISR_INIT` disappeared. HAL requested GPIO 13/6/12 (all falling edges), and edges arrived on GPIO 12 at 50 Hz (about every 20 ms). With the shim enabled, the program completed normally (exit code 0), with zero SPI write CRC mismatches and failures.
- [ ] If an offset exists, recorded it in the `robot_manager` `config.json` profile (`kernel_regex`) using values such as `HALSIMVMX_DIO_MAP`. **Not applicable** for the reason above.

**Expected:** The mapping remains stable. **If there is no offset**, the profile feature is unnecessary (and `robot_manager` could be replaced by a single systemd unit).

## 4. Build the Studica plugin (`libhalsim_vmx_studica.so`)

On this PC, only **syntax checking** with stub headers has been done. Actual compile and link are still needed.

Requirements: `g++` (C++20, GCC 11 or later), `VMXPi.h`, and `libvmxpi_hal_cpp`.

Method A — manual build (command untested; simplest):
```bash
cd allwpilib
g++ -std=c++20 -shared -fPIC -O2 \
  -I/usr/local/include/vmxpi -Istudica_drivers \
  -Isimulation/halsim_vmx/src/main/native/include \
  -Isimulation/halsim_vmx/src/studica/native/include \
  simulation/halsim_vmx/src/studica/native/cpp/StudicaBackend.cpp \
  studica_drivers/analog_input.cpp studica_drivers/dio.cpp studica_drivers/encoder.cpp studica_drivers/imu.cpp \
  -L/usr/local/lib/vmxpi -lvmxpi_hal_cpp -lrt -lpthread -latomic \
  -o libhalsim_vmx_studica.so
```
Method B — CMake: build the `halsim_vmx_studica` target with `-DWPILIB_WITH_STUDICA=ON`. This pulls in the full allwpilib configuration and requires GCC 13 or later for C++23.

- [x] Compile succeeded — **confirmed on robot (Ubuntu 26.04, g++ 15.2.0) using `install.sh` on 2026-10-10**.
- [x] Link succeeded — `/opt/halsim_vmx/libhalsim_vmx_studica.so` (97 KB), no `not found` from `ldd`; `libvmxpi_hal_cpp.so` resolves from `/usr/local/lib/vmxpi`.
- [x] `nm -D` shows `wpilibvmx_CreateBackendV1`, `wpilibvmx_DestroyBackendV1`, and `wpilibvmx::SharedVMX()`.
- (Reference) `MakeBackendApi`'s lambda is also exported as a weak symbol. Harmless, but it could be hidden with `-fvisibility=hidden`.
- **Still not done:** load this `.so` and operate hardware with it (§6).

## 5. Can WPILib libraries load on this OS? (glibc)

This is the largest risk. `linuxarm64` libraries built with the trixie toolchain target glibc 2.41.

1. On the PC, download the `linuxarm64` JNI library and copy it to the VMX. For example, add `vmxRelease wpi.java.deps.wpilibJniRelease(wpi.platforms.linuxarm64)` to a test project, resolve dependencies with `./gradlew`, and extract `*.so` from the cached ZIP.
2. Check on the VMX:
   ```bash
   ldd /path/libwpiHal.so | grep "not found"        # should be empty
   objdump -T /path/libwpiHal.so | grep -o 'GLIBC_[0-9.]*' | sort -Vu | tail -1   # highest required glibc
   ldd --version | head -1                           # compare with glibc in §0
   ```
   (Check `libstdc++` the same way: `objdump -T ... | grep -o 'GLIBCXX_[0-9.]*' | sort -Vu | tail -1` versus `strings /usr/lib/aarch64-linux-gnu/libstdc++.so.6 | grep GLIBCXX | sort -Vu | tail -1`.)

- [x] **PC inspection (2026-10-10; not run on robot at the time):** `libwpiHal.so` from Maven `org.wpilib.hal:hal-cpp:2027.0.0-alpha-7:linuxarm64` (11.6 MB):
  - **It is a simulation HAL:** 460 `HALSIM_*` symbols and `HAL_LoadExtensions` / `HAL_LoadOneExtension` / `HAL_OnShutdown`. (`allwpilib/hal/build.gradle`: only platform `systemcore` compiles `src/main/native/systemcore`; every other platform compiles `src/main/native/sim`.)
  - Highest required versions: **`GLIBC_2.34`**, **`GLIBCXX_3.4.31`**. These are well below the toolchain's glibc 2.41; robot glibc 2.43 is sufficient. (Ubuntu 22.04 has a sufficient glibc but GCC 12's libstdc++ lacks `GLIBCXX_3.4.31`.)
  - Dependencies: `libwpiutil`, `libntcore`, `libdatalog`, `libwpinet` (other ZIPs), and system `libatomic.so.1`, `libstdc++`, `libm`, `libgcc_s`, `libc`.
  - Other libraries (`wpiutil`, `ntcore`, `wpimath`, etc.) and whether `libatomic1` is installed have not been checked.
- [x] No missing dependencies according to `ldd` on robot after copying the libraries there (2026-10-11: a real program ran with the same libraries).
- [x] Highest required GLIBC 2.34 ≤ this OS's glibc 2.43.
- [x] Highest required GLIBCXX 3.4.31 ≤ this OS's libstdc++ (GCC 15; recommended to reconfirm using `strings ... | grep GLIBCXX` on the robot's `libstdc++.so.6`).

**If this fails:** Published `linuxarm64` libraries cannot be used on this OS. Options: (a) upgrade VMX to a newer OS (Debian 13 / Ubuntu 26.04), then repeat §1; (b) build WPILib directly with a compatible sysroot; or (c) build directly on the VMX.

## 6. Does the extension load and operate hardware?

### 6a. Build tools on the robot (`tools/build_on_robot.sh`)

This does not build WPILib. It downloads the published `linuxarm64` libraries (about 42 MB: hal, wpiutil, ntcore, datalog, wpinet) and links against them. Run from the robot's allwpilib checkout:

```bash
./simulation/halsim_vmx/tools/build_on_robot.sh
```

Outputs under `~/halsim_vmx-build`: `libhalsim_vmx.so` (extension), `hal_dio_test` (test that drives WPILib DIO through the sim HAL), and `vmx_channels` (prints the VMX channel map). Libraries are placed under `~/wpilib-libs`. At the end, `ldd` checks for missing dependencies (the robot-side check from §5).
Use `--dry-run` to print commands without running them.
**Robot results (2026-10-10):** `libhalsim_vmx.so`, `hal_dio_test`, and `vmx_channels` all compiled without errors (g++ 15.2.0). The final `ldd` check found no `not found` dependencies (published `linuxarm64` WPILib libraries resolve on this robot), and `vmx_channels` printed the channel map (see §4 of `DESIGN.md`).

### 6b. Ask the robot for channel numbers; do not guess

```bash
sudo ~/halsim_vmx-build/vmx_channels
```

Prints the channel index ranges and capabilities (DigitalIn/Out, PWM, Encoder, Interrupt, SPI/I2C/UART, etc.) for FlexDIO / AnalogIn / HiCurrDIO / CommDIO. Use these values in `HALSIMVMX_DIO_MAP` / `HALSIMVMX_ANALOG_MAP` (and fill in the channel table in §3).

### 6c. Drive DIO through the sim HAL (`hal_dio_test`)

Set `HALSIMVMX_DIO_MAP="0:<N>,1:<M>"` (WPILib DIO 0 → VMX channel N, DIO 1 → VMX channel M). To test loopback, connect output N to input M **with a jumper wire** (`loopback OK`); without a jumper, inspect the output pin using a multimeter/LED. `build_on_robot.sh` prints the run command at the end.
If the log says `HALSim VMX: DIO 0 -> VMX channel N (output)`, the claim succeeded. `cannot claim VMX channel ...` means failure (the channel may not support that use or may already be in use).

**Second run (after connecting the jumper) — loopback passed:** All 10 lines of `wrote 0, read 0` / `wrote 1, read 1` matched and printed `loopback OK`. **The complete path works on the actual robot: WPILib sim HAL DIO → `halsim_vmx` extension → Studica plugin → VMX hardware → jumper wire → input → HAL.** (VMX output 10, input 11, `HALSIMVMX_DIO_MAP="0:10,1:11"`.)
**Shutdown hang — fixed (2026-10-10):** `hal_dio_test` did not exit after `pigpio library closed.` (`timeout` killed it after 60 seconds with exit code 124; the prompt never returned and Ctrl-C had no effect).
`gdb` (`thread apply all bt`) showed the cause: three pigpio threads (`pthAlertThread`, `pthFifoThread`, `pthSocketThread`) remained alive after shutdown. The FIFO thread waited in `fgets` while holding the stdio lock, so the main thread hung forever in `exit()` → `_IO_flush_all`. `gpioTerminate` had not actually been called.
**Root cause was a `gpio_isr_shim` bug:** the shim used `dlsym(RTLD_NEXT, "gpioTerminate")` to find the original function, but the Studica plugin loads HAL with `dlopen(RTLD_LOCAL)`, keeping HAL outside the global lookup scope and returning `NULL` (no issue in `vmx_n`, which links HAL directly). The stdin theory was wrong (redirecting stdin from `/dev/null` still hung).
**Confirmed on robot after the fix:** returned to the prompt immediately with `exit code: 0`. The pigpio shutdown statistics banner (`pigpio version=69 internals=300 ... cbTicks ...`) confirms that the real `gpioTerminate` ran. `loopback OK` still passes.

**First-run results (2026-10-10, output=VMX 10, input=VMX 11):** Confirmed extension load (`HAL Extensions: Successfully loaded extension`), plugin use, both channel claims, and output reaching the board (SPI writes increased from 19 to 38, zero CRC errors). However, loopback input always read `1` (`wrote 0, read 1`). Studica `DIO` opens inputs with pull-up, so a disconnected input reads `1`. First suspect wiring (or a mismatch between printed board labels and HAL channel numbers). **Loopback input was not yet verified.**
To distinguish wiring from extension issues, first run `dio_probe`, which uses Studica `DIO` without HAL/the extension:

```bash
sudo ~/halsim_vmx-build/dio_probe 10 11
```

If it prints `loopback OK`, wiring and channel numbers are correct, so investigate the extension. If it fails, investigate wiring/channel numbers (`input before driving anything: 1` means the input is not connected).

### 6d. Test `robot_manager` shutdown order with a real HAL process

A small test program initialized the sim HAL (loading the extension) and then waited. It was run as `robotCommand`, and the service was stopped to check whether shutdown completed promptly (the test program has since been deleted; the same test can be done with any robot program using `sudo systemctl stop robot_manager`).
Expected: after `stopping robot program ...`, see `robot program stopped by SIGINT after 0.x s` (no `sending SIGKILL`).

**First measurements (2026-10-10) found two problems:**

1. **`robot_manager` exited before cleaning up its child.** The systemd log showed `State 'final-sigterm' timed out. Killing.` / `Killing process ... (test program) with signal SIGKILL`: after `robot_manager` exited, systemd killed the remaining test program with `SIGKILL` 15 seconds later.
   Cause: the signal handler called `stop()` in a daemon thread. When the listener exited, `run()` returned first; the interpreter then shut down and interrupted group cleanup (grace period followed by `SIGKILL`). Unit tests missed it because they called `stop()` directly.
   **Fix:** the main thread joins the shutdown thread. First added a regression test (`test_the_service_finishes_stopping_the_program_before_it_exits`) and confirmed it failed, then fixed the issue (on PC). Robot verification was still pending at the time.
2. **The HAL appeared to clean up on `SIGTERM` without exiting the process.** A test program was still running 13.7 seconds after `kill -TERM`. Its log ended with `VMX HAL: SIGTERM signal received.` → `closing pigpio library to release VMX resources.` → `pigpio library closed and VMX resources released.` → `At VMXPi::Terminate` → `Stopped RemoteServer` → `spiClose: pigpio uninitialised` / `Error closing SPI AUX Channel 2.`
   (`Ctrl-C` / SIGINT showed `Exiting VMX-pi HAL application ... [normal exit]`.) It was only a **hypothesis, not yet confirmed**, that the handler cleaned up VMX resources and returned. If so, `robot_manager` would wait for the full `stop_grace_seconds` on each SIGTERM and then use `SIGKILL` (after VMX resources were already released).
   **Confirmed (2026-10-10):** `gdb` inspection 3 seconds after `SIGTERM` showed that the main thread remained in `nanosleep` (the wait loop) inside the test program's `main()`, while the three pigpio threads had disappeared. The HAL's `SIGTERM` handler cleans up VMX/pigpio and **returns without exiting the process**.
   **`SIGINT` exits cleanly in 0.21 seconds** (`Exiting VMX-pi HAL application due to receipt of signal 2 [normal exit]`; `Signal 2 received by PIGPIOClient::signal_func()` and its stack output are normal HAL output).
   **Fix:** `robot_manager` stops processes in this order: `SIGINT` → `SIGTERM` → `SIGKILL` (configured by `stop_signals`). Signal order, fallbacks, and config parsing passed PC tests (39 tests). The robot check for `stopped by SIGINT after 0.x s` is recorded below.
   **Confirmed on robot (2026-10-10, `d31287e`):** `stopping robot program (pid ...): SIGINT then SIGTERM, then SIGKILL; 5s in total` → `robot program stopped by SIGINT after 0.2s`. No `sending SIGKILL`, systemd `Killing process`, or `final-sigterm`. Both issue 1 (exit before child cleanup) and issue 2 (SIGTERM does not exit) are resolved.
   Reference: the old Studica GradleRIO fork stops programs with `frcKillRobot.sh -t` (`-t -r` for restart) and writes commands to `/home/lvuser/robotCommand`. The signal sent by that script has not been verified from source (it is inside the VMX image).

### 6d-2. `SIGILL` (exit code 132) at program shutdown — LSE instructions in `libMrcLib` (2026-10-11)

The C++ program (`vmx-cpp-test`, deployed with `./gradlew deploy`) consistently exited after testing with `Illegal instruction (core dumped)`, code 132. `gdb` traced it to `swp w1, w1, [x0]` in `uv_library_shutdown()` (`libMrcLib.so`).
**Cause:** `swp` / `ldadd` / `cas` are ARMv8.1 LSE atomic instructions unavailable on Pi 4 (Cortex-A72, ARMv8.0). `libMrcLib.so` (WPILib `mrclib`, source unavailable) is distributed as `linuxarm64`, but built with **SystemCore Buildroot GCC 14.3**, which emitted LSE instructions in 1,219 locations (`DsClient`, `MulticastResolverClient`, `UvDatapath`, and other Driver Station networking code). Other WPILib libraries, our executable, `libvmxpi_hal_cpp`, and our plugin use only CPU-detecting libgcc helper functions (`__aarch64_*`), so are safe (verified by disassembling about 90 libraries on PC; OpenCV was not checked). `libwpiHal` does not link `mrclib` (upstream commit `7d214e53a`).
**Fix:** `Robot-Manager/lse_emu/` provides an `LD_AUDIT` library that emulates LSE instructions using `ldxr/stxr` in a `SIGILL` handler. `LD_PRELOAD` does not work (the loader initializes libraries from the back, so preloading happens after the MrcLib constructor; reproduced in qemu). If `/proc/cpuinfo` lacks `atomics`, `robot_manager` sets `LD_AUDIT` automatically.

- [x] On Pi 4, `make test`: all 224 instruction forms, thread atomicity, constructor, and external `SIGILL` tests passed.
- [x] Service-run program exits with code **0** (`emulated 1 LSE instruction`).
- [x] Cost: **about 26.8 µs per instruction** (about 1,200× a normal atomic addition). Only one instruction is used at shutdown on the current path, so impact is negligible. Driver Station networking may be slower; count instructions first with `VMX_LSE_EMU_DEBUG=1`.
- [ ] Behavior/performance when actually using MrcLib's Driver Station networking code is unverified.

### 6d-3. Deploy with `./gradlew deploy` without building on the robot (2026-10-11)

The C++ project (`vmx-cpp-test`) **cross-compiles** `halsim_vmx` and the Studica backend plugin on the PC (two `NativeLibrarySpec` components targeting `linuxarm64`); deploy uploads both to the library directory (`~/wpilib/third-party/lib`). The only files fetched from the robot are `VMXPi.h` and `libvmxpi_hal_cpp.so` from its OS image (`fetchVmxSdkvmx`).

- [x] Robot logs: `HALSim VMX: using backend /home/ubuntu/wpilib/third-party/lib/libhalsim_vmx_studica.so`, `Successfully loaded extension`, `IMU claimed`, `Encoder 0 -> VMX channels 0/1`, `AnalogIn 0 -> VMX channel 22`. This program does not use `/opt/halsim_vmx` or `~/halsim_vmx-build` (removed from rpath).
- [x] GradleRIO automatically deploys a library linked in the project using `lib library: ...`. An extension not linked by that mechanism (`libhalsim_vmx`) must be linked by the executable to be uploaded.
- Found: putting a Groovy GString such as `"${...}"` into `environment.put(key, ...)` caused `robotCommand` generation to fail with `ClassCastException` → fixed in GradleRIO for both Java/C++ and added tests.
- `tools/build_on_robot.sh` is now a fallback if PC cross-compilation fails. Java projects still have no equivalent (there is nowhere to build the extension).
- [ ] Still outstanding: publish to Maven (this approach directly points the project at the `allwpilib` source directory) and test a Java project.

### 6d-4. A WPILib program did not stop on SIGINT/SIGTERM (2026-10-17)

MEASURED on a VMX-pi with a TimedRobot program deployed from the VS Code extension: after `kill -INT`, the VMX HAL printed `Exiting VMX-pi HAL application due to receipt of signal 2 [normal exit]` (pigpio was cleaned up by `gpioTerminate`), but the process stayed alive indefinitely (`SigCgt` had SIGINT and SIGTERM). `robot_manager` waited its whole 5 s grace period and sent SIGKILL, so every deploy spent 5 s in `systemctl stop`. A WPILib desktop program has no signal handler of its own and normally dies of the default action; the HAL's handler replaced it and returns.

Fix: `InstallTerminationHandlers()` (extension start, after the backend opened the VMX HAL) chains to the existing handler and then restores the default action and re-raises, so the process ends right after the hardware clean-up. PC tests (4 cases) cover SIGINT, SIGTERM, no previous handler and an ignored signal.

- [ ] On the robot, with the new `libhalsim_vmx.so`: `time sudo systemctl stop robot_manager` is well under 5 s and the journal says `robot program stopped by SIGINT` (not `killed`).
- [ ] The next start does not fail with `initMboxBlock` (pigpio was released).
- [ ] Titan: stopping the service while a motor runs stops it within 200 ms (the guard's destructor does not run on this path).

Also seen: one realtime thread (priority -50) used about 70 % of a core in the idle TimedRobot program, and `robot_manager` consumed 2 h 25 min CPU over 2 h 18 min. Not explained yet (MrcLib Driver Station polling with the emulated LSE atomics is a suspect: `top` showed 21 % system time).

### 6e. Existing verification items

Check with a minimal Java robot project (or the result of `./gradlew deploy` from PC). Manual run example (command untested):

```bash
sudo HALSIM_EXTENSIONS="/path/libhalsim_vmx.so" \
     HALSIMVMX_BACKEND="/path/libhalsim_vmx_studica.so" \
     HALSIMVMX_DIO_MAP="..." HALSIMVMX_ANALOG_MAP="..." HALSIMVMX_IMU=1 \
     /usr/bin/java -Djava.library.path=/path -cp ... robot.MainClass
```

Check the logs:
- [x] `HALSim VMX: using backend /path/libhalsim_vmx_studica.so`
- [x] `HALSim VMX Extension Initialized`
- [ ] If deliberately given an invalid backend (`HALSIMVMX_BACKEND=/nonexistent/path`), robot program reports extension initialization failure and does **not** silently fall back to loopback.

Check each device (using the channel map in §3). Read and write each using **WPILib standard classes**.

| Device | Procedure | Expected | Result |
|---|---|---|---|
| DigitalOutput | Toggle `set(true/false)`; check with a multimeter/LED | Pin toggles | |
| DigitalInput | Apply HIGH/LOW using a jumper | `get()` follows the input | |
| AnalogInput | Apply voltage from a potentiometer/known source | `getVoltage()` matches the actual voltage | **Confirmed (2026-10-10, C++ test program, `HALSIMVMX_ANALOG_MAP="0:22"`):** applied 5 V to pin 22 → `4.996 V` (same across all 40 reads). Claim log `AnalogIn 0 -> VMX channel 22`; 52,099 reads, zero CRC mismatches/failures. GND/3.3 V range not measured (assumed 0–5 V; scaling verified at only one 5 V point). |
| Encoder | Turn by hand | `get()` increments/decrements; zero after `reset()`; sign reverses after `setReverseDirection(true)` | **Partially confirmed (2026-10-11, deployed and ran C++ `vmx-cpp-test` with `./gradlew deploy`, `HALSIMVMX_DIO_MAP="0:0,1:1"`, FlexDIO 0/1):** while the motor ran, `wpi::Encoder::Get()` increased by about 120 every 0.25 s (0 → 2492), and held steady after the motor stopped. **Reset and `SetReverseDirection` also confirmed (2026-10-11, encoder wired to VMX FlexDIO 0/1, motor 0.2):** `Reset()` returned 1388 → 17 before counts rose again. After `SetReverseDirection(true)`, the count decreased for the same rotation (`-1626`, `-1752`, ...) and the direction bit became 0. Changing direction mid-run also flipped the current count's sign (+1505 → -1626). Disabling reverse returned it to +4162. |
| IMU (navX) | See §7 | | |

- [x] DIO output/input, AnalogInput, and Encoder work (confirmed by deploying and running a C++ program on 2026-10-11).
- [x] Creating Encoder A/B with `Encoder(a,b)` does **not** cause a channel conflict with `DigitalInput` on the same channels (confirmed without conflict using `Encoder(0,1)` and `HALSIMVMX_DIO_MAP="0:0,1:1"`; the encoder claims those channels).

## 7. IMU signs and Titan safety

### IMU (navX) — `HALSIMVMX_IMU=1`

Assumption (unverified): navX yaw and Z angular rate are **clockwise-positive**, so their signs are inverted to match WPILib (counterclockwise-positive).
Roll/pitch and X/Y angular rates/acceleration are passed through unchanged. Verify this assumption.

1. Slowly rotate the board **counterclockwise when viewed from above** → WPILib `OnboardIMU` (or HAL) yaw should **increase**.
2. Tilt the front (X) and then the side of the board → do the roll/pitch axes and signs match expectations?
3. At rest, is Z acceleration approximately **+9.8 m/s²** (gravity sign)?

- [x] Yaw sign ✔ (confirmed 2026-10-11 using `wpi::OnboardIMU::GetYaw()` from deployed C++ `vmx-cpp-test`): after rotating about 90° **counterclockwise** viewed from above, yaw **increased** from 26.8 → 132 (maximum) → 112.3 (at rest). The current sign inversion is correct. Z angular rate is inverted based on the same assumption as yaw, but its value has not been directly captured.
- [ ] Roll/pitch axes and signs ____ (unverified: only observed roll 0.4 and pitch 2.1 at rest; front/side tilt tests not yet done).
- [x] Z acceleration sign and magnitude ✔ (at rest, `+9.7–9.8` m/s², about ±0.05 noise).
- [ ] Confirm values are not updated while calibrating/disconnected (`IsCalibrating` / `IsConnected`).

### Titan safety — `TitanEnableGuard`

**Measured (2026-10-11) — `Titan::Enable()` alone cannot reliably do on/off/on.** Starting disabled and then enabling did not turn on the motor; the Titan LED alternated red-green-red (the documented enabled color is purple).
Cause: `Titan::Enable(true)` sends ENABLED_FLAG every **100 ms**, while `Enable(false)` sends DISABLED_FLAG every **10 ms**. VMX keeps repeating a periodic frame until told to stop (`VMXCAN_SEND_PERIOD_STOP_REPEATING`). Once disabled, DISABLED_FLAG is sent 10× more often than ENABLED_FLAG, so Titan remains disabled. The Studica example did not expose this because it enables once at startup and disables once at the end.
**Fix:** `StudicaTitan` (`src/studica/native/include/wpi/halsim/vmx/StudicaTitan.hpp`) stops periodic transmission of the opposite frame before calling `Titan::Enable()` (without modifying `studica_drivers`). Use `TitanEnableGuard<StudicaTitan>`. On the robot, the motor ran only while enabled in the sequence disabled (2 s) → enabled (3 s) → disabled → enabled → e-stop.
**Titan encoder (encoder connected to Titan motor 0 terminals, 2026-10-11):** While enabled, `GetEncoderCount` increased by about 125 every 0.25 s (51 → 2590), and `GetRPM` was about 655 (speed 0.2). In disabled/e-stop, count stopped and RPM was 0. When connected to VMX FlexDIO, `wpi::Encoder` measured a matching rate (about 120 every 0.25 s). `ResetEncoder` took about 0.2 s to take effect (the previous count appeared once just after startup).
**Measurement 2 — motor kept running after the program died (2026-10-11).** `Titan::Enable(true)` lets the VMX **board** repeat ENABLED_FLAG every 100 ms. The board continues transmitting even if the process is killed with `kill -9`. Titan's failsafe says it stops after "no messages for 200 ms," but repeated frames kept arriving, so the motor kept its last speed. The same happened when stopped with `systemctl stop` (what deploy does before uploading files).
**Fix:** `StudicaTitan` no longer uses `Titan::Enable()`; it sends enable/disable frames **once, without periodic repeats** (also clearing repeats left on the board by an earlier process at startup). `TitanEnableGuard` resends the current state using `KeepAlive()` every 20 ms. If the process dies, frames stop and Titan stops itself. All 37 `halsim_vmx` unit tests pass; on physical hardware, both `kill -9` and `systemctl stop` stop the motor immediately.
**Caution:** the object passed to the guard must implement `KeepAlive()`. If wrapping `StudicaTitan` in another class, forward `KeepAlive()` too (otherwise enable is sent only once and Titan disables itself after 200 ms; this happened in the test program).
**Side effect discovered — `kill -9` leaks pigpio resources.** A forcibly killed process cannot call `gpioTerminate`, leaking GPU memory (mailbox). After several occurrences, the next startup fails with `initMboxBlock: init mbox zaps failed` / `Error initializing pigpio library` and the program soon segfaults. **Rebooting clears this.** `robot_manager` sends SIGINT first, so the normal shutdown path avoids this; it occurs only if the process fails to exit within `stop_grace_seconds` and is sent SIGKILL.
`GetSerialNumber()` returns garbage if read before a Titan response arrives (wait 1 s after creating Titan; the read does not check for failure).

The user's MockDS must call both **`setDsAttached(true)` and `setEnabled(...)`** (the sim HAL populates the control word only when DS is attached).
Titan ignores commands until `Enable(true)` and stops itself if it receives no CAN messages for 200 ms (`titan.hpp`).

```cpp
#include "wpi/halsim/vmx/ControlLoopWatchdog.hpp"
#include "wpi/halsim/vmx/StudicaTitan.hpp"
#include "wpi/halsim/vmx/TitanEnableGuard.hpp"

using namespace std::chrono_literals;
studica_driver::Titan titan(id, freq, distPerTick, wpilibvmx::SharedVMX());
wpilibvmx::StudicaTitan switcher{titan, id};
// Example deadlines: tune and measure for the robot's control/DS periods.
wpilibvmx::ControlLoopWatchdog watchdog{100ms, 150ms};
using Guard = wpilibvmx::TitanEnableGuard<wpilibvmx::StudicaTitan>;
Guard guard{switcher, [&] { return Guard::HalEnabled() && watchdog.IsFresh(); }};
guard.Start();
// After each successfully completed control cycle: watchdog.FeedControlLoop();
// After each new valid packet in the team's MockDS: watchdog.FeedDriverStation();
```

(User code must keep sending `SetSpeed` within 150 ms while enabled.)

The default guard observes cached HAL state only; it cannot detect a stalled main
loop or stale DS reception by itself. The optional `ControlLoopWatchdog` keeps
these two deadlines separate and starts disabled until both sources have arrived.
Feed it from the actual sources, never from the guard or a timer reading cached
enabled state. It does not implement MockDS. Keep the watchdog and Titan switcher
alive until after the guard is stopped/destroyed.

PC regression tests verify startup, each deadline independently, recovery with
fresh heartbeats and E-stop precedence while cached enabled remains true.
They do not measure a physical stop time. On the fixed robot image:

- [ ] Stop control-cycle heartbeats while DS packets and the guard continue;
      record disable transmission and actual motor stop latency.
- [ ] Stop valid DS packets while the control loop and guard continue; record
      stop latency, then verify reconnect behavior.
- [ ] Verify that guard scheduling delays, CAN errors and main-loop logging load
      do not violate the chosen stop budget. A deadline is evaluated at the next
      guard poll; Linux scheduling and CAN delivery add latency.

- [x] Sending `SetSpeed` while disabled does **not** turn the motor (confirmed on robot, 2026-10-11).
- [x] Enabling turns the motor on.
- [x] Disabling again stops it **immediately**.
- [x] E-stop stops it.
- [x] Killing the robot program with `kill -9` stops the motor (2026-10-11; visually confirmed immediately after kill, exact time not measured). **It did not stop in the initial implementation:** see "Measurement 2" above.
- [x] `systemctl stop robot_manager` (SIGINT shutdown) stops the motor immediately (2026-10-11).
- [ ] `kill -TERM`: HAL's SIGTERM handler cleans up VMX/pigpio but does not exit (per `gdb` measurement above). Titan behavior in this state has not been tested. `robot_manager` sends SIGINT first and avoids this path.

## 8. End-to-end `robot_manager` and deploy

### `robot_manager` (repository: `W4Studica/Robot-Manager`; follow its README installation procedure)

- [x] After installation, `systemctl is-active robot_manager` reports `active` (confirmed on robot; no `robotCommand` existed yet, so the robot program itself was not run).
- [x] `./install.sh --sudoers` is applied and `sudo -n systemctl enable robot_manager` succeeds without a password (confirmed on robot).
- [x] If `/home/<user>/robotCommand` does not exist, the log prints a waiting message once (confirmed on robot).
- [x] Adding a simple `robotCommand` runs it. Check with `journalctl -u robot_manager -f` (confirmed on robot: `started robot program (pid ...)`).
- [ ] Program restarts after being killed (restart interval increases from 1 to 10 seconds after repeated crashes).
- [ ] `sudo systemctl stop robot_manager` cleans up the robot process tree too (check `pgrep -af robotCommand` / children).
- [x] Robot program runs as **root** (confirmed on robot: `user=root`).
- [x] The `env` from `config.json` reaches the robot process (confirmed on robot: `LD_PRELOAD=/opt/robot_manager/lib/libvmx_gpio_isr_shim.so`).
- [ ] Platform profile: if `kernel_regex` matches `uname -r`, profile environment variables are applied. The profile's `env` uses the same code path, but no profile has been populated yet → **will not test:** the OS image is fixed, so per-kernel profiles are unnecessary. Keep the feature.

### GradleRIO `./gradlew deploy` (from PC)

Use the example `build.gradle` in `GradleRIO/VMX.md`. The W4Studica `VmxPi` target defaults to the login `ubuntu` / `password` (override with `VMX_USER` / `VMX_PASSWORD` in the example); the address comes from `team` (`172.22.11.2` wired, `10.TE.AM.2` Wi-Fi AP) or `VMX_HOST`.

- [x] SSH and `sudo systemctl` / `sudo ldconfig` work **without a password** (sudoers; deploy runs both sudo commands without a password using `install.sh --sudoers`).
- [x] `./gradlew deploy` completes successfully.
- [x] `classpath/`, `third-party/lib/`, `robotCommand`, and `robotCommand.args` are uploaded to VMX.
- [x] Service restarts in stop → upload files → start order, and the new program runs.
- [x] `HALSIM_EXTENSIONS` / `HALSIMVMX_*` environment variables are included in `robotCommand`.
- [x] (C++) C++ robot program also deploys using `WPILibNativeArtifact` — **not yet used in a real build at the time** (2026-10-11, `GradleRIO/examples/vmx-cpp`).

### Network and deploy address (2026-10-11, physical hardware)

- [x] `setupHotspot` starts a Wi-Fi AP; PC receives DHCP (`10.12.34.X`) and can SSH to `10.TE.AM.2`.
- [x] Changed to 5 GHz (channel 36) and changed SSID prefix (`WorldSkills-1234`). Applying it from within an SSH session still works: robot applies it itself via `systemd-run` after 3 seconds.
- [x] If two NetworkManager profiles share a name, it activates the old one → activate by UUID and remove the old profile.
- [x] AP does not start on non-standard 5 GHz channel 34 → allow only standard channels; default to 36.
- [x] `./gradlew deploy` alone succeeds (automatic SDK fetch, team-number address `10.12.34.2`).
- **If PC Wi-Fi power saving (`Power save: on`) is enabled**, the first TCP response (SSH banner) is delayed by 1–10 seconds or times out, causing deploy discovery to fail. Fixed with `iw dev <if> set power_save off`. Our code increases the connection timeout to 30 seconds and retries SDK fetch in 3 rounds.
- [ ] Unverified: wired (`172.22.11.2`) path, phone-tethered Internet (robot could not access Internet), moving a new robot with `install.sh --network`, and whether the CPU-performance service persists after reboot.

## Results table

Development VMX-pi, 2026-10-10 to 11 (one robot; not a statement about other images).

| Item | Result | Notes |
|---|---|---|
| §0 OS / kernel / Pi model | Ubuntu 26.04.1, `7.0.0-1020-raspi`, Pi 4 Model B Rev 1.5 | No `/sys/class/gpio` (`CONFIG_GPIO_SYSFS` off) |
| §0 glibc / g++ / Java | glibc 2.43, g++ 15.2, Java 25 (Python 3.14) | |
| §1 Studica stack works | ✔ | `vmx-hal` installed; DIO/Analog/Encoder/IMU/Titan run |
| §2 Two `VMXPi` instances | First ✔ / second ✘ | SPI CRC errors, segfault at exit; use `SharedVMX()` |
| §3 Pin offset | No offset observed | DIO loopback, AnalogIn, Encoder work with plain `VMXChannelIndex`; printed-pin correspondence unchecked |
| §4 Plugin build | ✔ | Studica backend builds and loads |
| §5 glibc compatibility | ✔ | Published `linuxarm64` `libwpiHal` needs ≤ glibc 2.34 |
| §6 DIO/Analog/Encoder | ✔ | AnalogIn 5 V → 4.996 V; Encoder on VMX FlexDIO and Titan, reset, reverse direction |
| §7 IMU signs | Yaw ✔, Z accel ✔; roll/pitch and Z rate ✘ open | |
| §7 Titan safety | ✔ for guard + program death; watchdog latency open | Motor stops with the program (`kill -9`, `systemctl stop`) |
| §8 `robot_manager` | ✔ SIGINT stop, service, automatic `LD_AUDIT`; restart interval / child cleanup open | `kill -TERM` path open |
| §8 deploy | ✔ | PC cross build + `./gradlew deploy` (C++) |

## Documents to update based on results

- Any new failure → the matching statement in `CLAUDE.md`, `DESIGN.md`, `GradleRIO/VMX.md` and `robot_manager/README.md` (earlier results have already been propagated; check these four first when a row above changes).
- §2 → §3 "Still unknown" in `DESIGN.md` and `SharedVMX.hpp` guidance (recommendation vs requirement).
- §3 → channel map in §4 of `DESIGN.md`, and profile in `robot_manager/config.example.json` / README.
- §4 → `StudicaBackend.cpp` and the build commands in this document (remove unverified labels).
- §5 → Known risks in `GradleRIO/VMX.md`.
- §7 IMU → `PollImu` in `HALSimVMX.cpp` and the IMU row in §1 of `DESIGN.md`.
- Remove "unverified" / "not compiled" labels from every item that has since been confirmed.
