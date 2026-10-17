# VMX hardware verification record

Results measured on **one development VMX-pi** (2026-10-10 to 2026-10-17). They say nothing about other images or boards.
Unchecked `[ ]` items are open. PC tests (see `CLAUDE.md`) cover everything else; counts are not repeated here.

**Motor tests: raise the robot so the wheels cannot touch the floor, start slowly, keep a way to cut power.**

## Environment

| | |
|---|---|
| Board | Raspberry Pi 4 Model B Rev 1.5; VMX board model 0x32, hardware rev 60, firmware 3.0.436; navX present |
| OS | Ubuntu 26.04.1, kernel `7.0.0-1020-raspi`, glibc 2.43, g++ 15.2, Java 25, Python 3.14 |
| Kernel | no `/sys/class/gpio` (`CONFIG_GPIO_SYSFS` off); `/dev/mem`, `/dev/gpiomem`, `/dev/i2c-1`, `/dev/spidev0.*` present |
| SSH | Dropbear (`SSH-2.0-dropbear_2025.89`); `sudo` without password only for `systemctl stop/start/enable robot_manager`, `ldconfig`, `sync` |
| VMX HAL | `vmxpi-hal` installed (`/usr/local/lib/vmxpi`); `libvmxpi_hal_cpp.so` needs at most `GLIBC_2.34` / `GLIBCXX_3.4.29` |
| WPILib | published `linuxarm64` `libwpiHal.so` is the simulation HAL, needs at most `GLIBC_2.34` / `GLIBCXX_3.4.31` |

## Verified

| Area | Result |
|---|---|
| Studica stack on this OS | HAL talks to the VMX board and detects the navX |
| Plugin | `libhalsim_vmx_studica.so` builds with g++ 15.2 and loads; exports `wpilibvmx_CreateBackendV2/DestroyBackendV2` (the hardware-checked build exported V1) |
| DIO | WPILib sim HAL DIO to the extension to the plugin to the board and back through a jumper: loopback passes (VMX 10 to 11) |
| AnalogIn | 5 V on pin 22 reads 4.996 V (one point; the 0 to 5 V scale is assumed) |
| Encoder | on FlexDIO 0/1 and on a Titan: counts follow rotation (about 120 per 0.25 s at speed 0.2); `Reset()` and `SetReverseDirection(true)` work; the `DigitalInput`s WPILib creates for the same channels do not conflict |
| IMU (navX) | yaw sign (counterclockwise increases) and Z acceleration (+9.7 to +9.8 m/s2 at rest) |
| Titan | `StudicaTitan` + `TitanEnableGuard`: ignores `SetSpeed` while disabled; enable, disable and e-stop act at once; `kill -9` and `systemctl stop` stop the motor |
| Deploy | PC cross build plus `./gradlew deploy` of a C++ program works; the extension loads and claims IMU, Encoder, AnalogIn |
| robot_manager | service runs the program as root; waits for a missing `robotCommand`; `config.json` env reaches the program; `systemctl stop` stops the program (SIGINT first) |
| GPIO shim | `PI_BAD_ISR_INIT` is gone; edges arrive on GPIO 12 at 50 Hz; clean exit code 0 |
| LSE emulation | program exits with code 0 under `LD_AUDIT`; cost about 26.8 microseconds per emulated instruction |
| Network | AP (2.4 and 5 GHz), SSID change, DHCP, SSH to `10.TE.AM.2`, change applied by the robot itself 3 s later, deploy by team number |
| Stop after SIGINT/SIGTERM | with `InstallTerminationHandlers()` the stop takes 1.26 s (was 5.2 s) |
| Wi-Fi power saving | turning it off on the robot lowered the ping from avg 97 ms / max 388 ms to avg 5.4 ms / max 6.2 ms |

## What the measurements changed

- **Two `VMXPi` in one process break the SPI link** (hundreds of write CRC errors, then a segfault); one works. Studica class constructors default to a new `VMXPi`, so always pass `wpilibvmx::SharedVMX()`.
- **`studica_driver::Titan::Enable()` cannot switch on/off/on.** The VMX board repeats the frame (enable every 100 ms, disable every 10 ms), so after one disable the Titan can never be enabled again, and after the program dies (`kill -9`, `systemctl stop`) the board keeps sending, so the motor keeps running. `StudicaTitan` sends single frames and `TitanEnableGuard` repeats them every 20 ms through `KeepAlive()`; when the program dies the frames stop and the Titan stops by itself after 200 ms. The object given to the guard must forward `KeepAlive()`.
- **`GetSerialNumber()` of a Titan returns garbage before the first reply** (wait 1 s after creating it).
- **The VMX HAL's SIGINT/SIGTERM handlers clean up pigpio and return; they do not end the process.** A WPILib desktop program has no handler of its own and normally dies of the default action, so on a VMX it never stopped and `robot_manager` waited its whole 5 s grace period and sent SIGKILL (every deploy lost 5 s). `InstallTerminationHandlers()` chains to the HAL handler, then restores the default action and re-raises; a 1 s `SIGALRM` watchdog covers a HAL clean-up that hangs inside `exit()`. The handlers are re-installed on every poll because the HAL installs its own after the extension starts. The HAL-only test program measured earlier did exit 0.2 s after SIGINT; a TimedRobot program did not.
- **`robot_manager` stops SIGINT first.** On SIGTERM the HAL also only cleans up. A SIGKILL before the clean-up leaks pigpio GPU memory: the next start fails with `initMboxBlock: init mbox zaps failed` until a reboot.
- **`libMrcLib.so` (WPILib, binary only) uses ARMv8.1 LSE atomics and dies with SIGILL (exit 132) on a Pi 4.** `Robot-Manager/lse_emu` emulates them, loaded with `LD_AUDIT` (`LD_PRELOAD` initialises too late). Other libraries use CPU-detecting libgcc helpers and are safe (about 90 libraries disassembled on the PC; OpenCV not checked).
- **No `/sys/class/gpio` on this kernel**: pigpio's interrupts fail with `PI_BAD_ISR_INIT`. The extension itself polls and does not use them; the shim replaces `gpioSetISRFunc*` with the GPIO character device. DIO, AnalogIn and Encoder need no channel offset.
- **A Studica `DIO` input is pulled up**: a disconnected input reads 1.
- **A robot access point with Wi-Fi power saving on answers in bursts**; `vmx_net.py` now writes `powersave=2` into the AP and client profiles. A PC with power saving on delays the first SSH reply by 1 to 10 s: `nmcli connection modify <name> 802-11-wireless.powersave 2`.

## Open

- [ ] IMU roll/pitch axes and signs; Z angular-rate sign; no updates while calibrating or disconnected.
- [ ] `ControlLoopWatchdog` stop latency on a Titan: (a) stop the control-cycle heartbeats while DS packets and the guard continue, (b) stop DS packets while the loop continues, then check reconnect, (c) check that scheduling delay, CAN errors and logging load stay inside the stop budget. A deadline is evaluated at the next guard poll (20 ms) and CAN delivery adds latency.
- [ ] Titan stops within 200 ms when the service is stopped while a motor runs (the guard's destructor does not run on the signal path).
- [ ] The next start after a SIGALRM stop does not fail with `initMboxBlock`.
- [ ] Why the HAL's handler hangs inside `exit()` (the robot has no `gdb`).
- [ ] `kill -TERM`: Titan behaviour in that state.
- [ ] An idle TimedRobot program: one realtime thread (priority -50) used about 70 % of a core (21 % system time); MrcLib Driver Station polling with emulated LSE atomics is a suspect. Count with `VMX_LSE_EMU_DEBUG=1`.
- [ ] Analog and Encoder channels are still claimed with the new capability check (`ChannelSupportsCapability`): AnalogIn 22, Encoder 0/1.
- [ ] `robot_manager`: restart delay grows from 1 s to 10 s after repeated crashes; stopping the service leaves no child processes (`pgrep -af robotCommand`).
- [ ] `HALSIMVMX_BACKEND=/nonexistent`: the program reports an extension initialisation failure and does not fall back to loopback.
- [ ] PWM (`HALSIMVMX_PWM_MAP`): a servo moves with `wpi::Servo`/`PWMSparkMax`-style pulses (1000 to 2000 us), a pulse of 0 leaves the line low, and `HALSim VMX: PWM n -> VMX channel m` is printed. Check on a scope: the frame is 20 ms / 10 ms / 5 ms as asked, the pulse width is within 4 us. Two PWM channels with different frame lengths (shared VMX timer?).
- [ ] Battery voltage: `RobotController::GetBatteryVoltage()` follows the supply (compare with a meter). Without a reading the sim keeps its default 12 V.
- [ ] Printed board pins versus `VMXChannelIndex`.
- [ ] Wired path (`172.22.11.2`); `install.sh` network switch-over, its backup and `--undo-network`; the CPU-performance service after a reboot; `install.sh --upgrade-drivers`.
- [ ] AP power saving stays off after `install.sh` (`sudo iw dev wlan0 get power_save`).

## Channel map

Measured with `tools/vmx_channels` (VMX board model 0x32). See `DESIGN.md` section 4 for the table.

## Tools

`tools/build_on_robot.sh` builds the extension and the test tools on the robot (fallback if the PC cross build fails); `vmx_channels` prints the channel map; `hal_dio_test` drives WPILib DIO through the simulation HAL; `dio_probe` loops two channels back with the Studica `DIO` class, without the HAL, to tell wiring problems from extension problems. Run them as root (pigpio needs it).

## Checking a stop

On the robot, with a program running:

```bash
sleep 8; time sudo systemctl stop robot_manager
journalctl -u robot_manager -n 12 --no-pager | cut -c1-160
```

Expect about 1 s and `robot program stopped by SIGINT`; `killed after` or `still running after 5s; sending SIGKILL` means the program did not exit.
