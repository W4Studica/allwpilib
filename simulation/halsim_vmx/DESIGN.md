# halsim_vmx Design and Mapping

> Status: **Draft**. Written after reviewing `studica_drivers` (Studica-Robotics/ROS2 `f97edec`) and allwpilib `2027.0.0-alpha-7` headers.
> Nothing has yet been validated on physical VMX-pi hardware. Items marked "needs verification" require a hardware test.

## 0. Core decision: call Studica APIs directly

Robot code calls existing Studica classes such as `studica_driver::Titan`, `Servo`, `Imu`, and `DIO` **directly**. These classes access hardware through `VMXPi` (`vmxpi_hal_cpp`) and therefore bypass the simulation HAL.

The design has two paths:

```text
Path A (primary)   Robot code --studica_driver::*--> vmxpi_hal_cpp --> VMX hardware
Path B (optional)  Robot code --WPILib classes--> sim HAL --halsim_vmx--> studica_driver::* --> VMX
Shared             robot_manager: start/restart programs and select platform profiles
                   (MockDS is user-provided and out of scope)
```

- **Path A does not need halsim_vmx.** It needs (1) link the `studica_drivers` library, (2) pass along the enabled state (§2), and (3) share one `VMXPi` per process (§3).
- **Path B (`halsim_vmx`) connects as many WPILib standard classes as practical.** (Decision made.) The goal is to let code, examples, and third-party libraries using classes such as `DigitalInput`, `Encoder`, `AnalogInput`, and `PWM` run on VMX unchanged. It follows the `halsim_xrp` approach but calls Studica classes in the same process instead of using UDP.
- **Both paths must share the same single `VMXPi` in the same process** (§3). `halsim_vmx` owns it and passes that instance to user-created Studica objects.

## 0.5 Backend plugin (`dlopen`)

`libhalsim_vmx.so` does not depend on the VMX header (`VMXPi.h`, available only on the VMX image). A separate backend plugin (`.so`) handles hardware access and is loaded with `dlopen` when its path is provided in `HALSIMVMX_BACKEND`. This allows the extension itself to be cross-compiled for `linuxarm64`, while the Studica backend (`libhalsim_vmx_studica.so`) is built once on the VMX (`-DWPILIB_WITH_STUDICA=ON`; C++20 required).

- The interface is a **C function table** (`BackendApi.h`, ABI version 1), so the two libraries may use different compilers/libstdc++ versions (WPILib's trixie toolchain versus the VMX Ubuntu toolchain). The plugin exports `wpilibvmx_CreateBackendV1` / `wpilibvmx_DestroyBackendV1`. Any `VmxBackend` can be wrapped and exported using `MakeBackendApi` in `BackendPlugin.hpp`.
- If `HALSIMVMX_BACKEND` is **unset**, the extension uses loopback, an in-memory fake backend for PC simulation. It does not move hardware and logs that it is active.
- If `HALSIMVMX_BACKEND` is **set but loading fails**, extension initialization fails (`-1`). It must not silently fall back to loopback: the most dangerous outcome is robot code appearing to run normally while the hardware remains inactive.
- Plugins with a different ABI version or structure size are rejected. Exceptions do not cross the ABI boundary.
- Validation: a test plugin wrapping loopback through the C ABI was loaded on PC. Normal operation, missing files, non-plugin libraries, ABI mismatch, and environment-variable behavior were tested. **`StudicaBackend.cpp` itself has not yet been compiled or run.**

## 1. Mapping table (Path B: WPILib class → sim HAL → Studica class)

The "sim HAL data" column lists HALSIM field names from `hal/.../simulation/*Data.h`.

| WPILib class | Sim HAL data | Direction | Studica class / method | Notes |
|---|---|---|---|---|
| `DigitalOutput` | `DIO` `Initialized`, `IsInput=false`, `Value` | sim→HW | `DIO(ch, PinMode::OUTPUT)`, `Set(bool)` | Channel map required (§4) |
| `DigitalInput` | `DIO` `Initialized`, `IsInput=true`, `Value` | HW→sim | `DIO(ch, PinMode::INPUT)`, `Get()` | Can map interrupts to `EnableInterrupt()` |
| `AnalogInput` | `AnalogIn` `Initialized`, `Voltage` | HW→sim | `AnalogInput(port)`, `GetAverageVoltage(float&)` | Returns `bool` and writes an out parameter |
| `Encoder` | `Encoder` `Initialized`, `DigitalChannelA/B`, `Count`, `Reset`, `ReverseDirection` | HW→sim | `Encoder(port_a, port_b)`, `GetCount()` | Wrapper handles `Reset` / `DistancePerPulse` (`Studica Encoder` has no reset) |
| `PWM` / `Servo` | `PWM` `Initialized`, `PulseMicrosecond`, `OutputPeriod` | sim→HW | (deferred) | **Deferred.** Studica `PWM` has no public output setter (`Servo::SetAngle/SetSpeed` are the only outputs), and conversion from integer range to duty ticks does not match microsecond pulse widths. Since `studica_drivers` cannot be changed, users should use Studica `Servo` directly. `SetAngle` calls `printf` every time. |
| `DutyCycleEncoder` | `DutyCycle` `Initialized`, `Frequency`, `Output` | HW→sim | (deferred) | **Deferred.** Studica's VMX capture for 12-bit (4095) absolute encoders returns degrees only and exposes no raw duty/frequency. Inferring them would require assumptions about the sensor protocol and risks silently returning incorrect values. |
| IMU (`OnboardIMU` family) | `IMU` `Yaw`, `AngleX/Y/Z`, `GyroRateX/Y/Z`, `AccelX/Y/Z` (setters only, one global instance, no "initialized" signal) | HW→sim | `Imu` (navX): `GetYaw/Pitch/Roll`, `GetRawGyroX/Y/Z`, `GetRawAccelX/Y/Z`, `IsConnected`, `IsCalibrating` | **Implemented.** Enable explicitly with `HALSIMVMX_IMU=1` because no signal indicates the IMU is present. Conversions (**hardware behavior assumed, not yet verified**): degrees→radians, deg/s→rad/s, g→m/s². navX yaw and Z angular rate are clockwise-positive, while WPILib is counterclockwise-positive, so their signs are inverted. Roll/pitch and X/Y angular rates/acceleration are passed through. Values are not updated while disconnected or calibrating. Quaternion is unsupported because the sim HAL has no setter. |
| `I2C` | `I2C` `Initialized`, read/write **buffer callbacks** | Both ways | (deferred) | **Deferred.** The sim HAL does not pass `deviceAddress` to the callback (`hal/src/main/native/sim/mockdata/I2CData.cpp` receives the address but calls only `write(buf, size)` / `read(buf, count)`). The destination device is unknown, and `transaction` cannot be identified as a repeated-start pair because the read callback runs separately after the write callback. Fixing this requires modifying allwpilib itself, which conflicts with the merge policy. An environment variable forcing one address cannot support a multi-device bus. Use Studica I2C devices (such as `Cobra`) through the Studica API directly. |
| `DriverStation` | `DriverStation` `Enabled`, `RobotMode`, `OpMode`, `EStop`, `DsAttached`, `Joystick*` | Populated by user MockDS | (no hardware class) | We only read it. See §2. |

### Studica-only devices without a WPILib standard equivalent

These devices cannot be represented by the standard sim HAL and are available only through **Path A (direct Studica API use)**. Exposing them through Path B would require XRP-style vendordep classes based on `SimDevice`; that is lower priority.

| Studica class | Purpose | Notes |
|---|---|---|
| `Titan` | CAN motor controller (4 channels, encoder, RPM/position PID, limit switch, temperature) | WPILib's standard PWM/CAN motor HAL path is not a fit. **Enable/keepalive rules are important (§2).** |
| `Ultrasonic` | Ultrasonic `Ping()`, `GetDistanceMM/IN()` | Different shape from WPILib `Ultrasonic` |
| `Sharp` | Infrared distance sensor (`AnalogInput` based) | |
| `Cobra` | Line sensor (I2C ADC) | |
| `Colore`, `Parsec`, `*_usb` | Color / ToF sensors | CAN / USB |
| `LightTower` | Signal tower / buzzer | |

## 2. Enable / MockDS / safety (most important integration point)

`Titan` has device-specific rules (see comments in `titan.hpp`):

- It powers up **disabled**. Call `Enable(true)` before `SetSpeed` / `SetTargetVelocity` has an effect.
- If no CAN message arrives for **200 ms**, the device stops and disables itself. While controlling it, send commands every ≤150 ms (`TITAN_CAN_KEEPALIVE_MS`).

→ **The robot enabled state must be connected to Titan enable.** Only the robot program opens the VMX (WPILib uses one process), and `robot_manager` does not open it (§3), so the **robot program** is responsible for safety.

| Option | Description | Notes |
|---|---|---|
| **S1 (starting point)** | User code calls `titan.Enable(...)` based on `DriverStation::IsEnabled()`. Provide a helper. | Does not modify Studica classes. Users may forget to call it. |
| **S4 (implemented)** | **`TitanEnableGuard<TitanT>`** (header-only template outside `studica_drivers`) reads enable/E-stop from the HAL control word (`HAL_GetUncachedControlWord`) and calls `titan.Enable()` **only when state changes**. It always calls `Enable(false)` when stopped or destroyed. It does not send motor commands, so user code must continue calling `SetSpeed` within 150 ms while enabled. Tested without hardware because only `Enable(bool)` is required. | **Caution: the sim HAL fills the control word only when `DsAttached` is true.** The user's MockDS must call both `setEnabled(true)` and `setDsAttached(true)`. Create Titan before the guard. Do not pass `studica_driver::Titan` directly to the guard; wrap it in `wpilibvmx::StudicaTitan`. Periodic `Titan::Enable()` frames (100 ms for enable, 10 ms for disable) do not cancel each other, so disable can prevent a later enable (measured). |
| Hardware failsafe | Titan **stops itself after 200 ms** even if the process dies (comment in `titan.hpp`, "CAN Watchdog" in the ROS2 README). | Last line of defense against software bugs. |

**The user writes MockDS in their robot code (out of scope for this project).** When MockDS populates the sim `DriverStation` data (`Enabled`, `RobotMode`, `EStop`), our code (Titan helper and safety logic) only reads those values. `robot_manager` is not involved.

## 3. GitHub research and open questions

### Confirmed (based on Studica-Robotics/ROS2 `f97edec` code and README)

- **One process opens the VMX (`pigpio`).** The README / `setup_permissions.sh` says root is required because pigpio accesses `/dev/mem` and locks a **PID file**. `studica_control` logs "pigpio may be unavailable or in use" on failure. WPILib robot code already uses one process, so this restriction is fine.
  - **Only the robot program process opens the VMX.** `robot_manager` manages process lifetime and does not open the VMX. Avoid opening it from other processes such as diagnostic tools.
- **Run as root (`sudo`).** The deploy script and `robot_manager` service must run the robot program as root.
- **Creating and sharing one `VMXPi` per process is a validated pattern.** In `manual_composition.cpp`, `studica_control` creates `VMXPi(true, 50)` once and passes it to all components through `shared_ptr`.
- **Studica class constructors have a trap in their default arguments.** Most classes (`Titan(...)`, `Servo(...)`, `Encoder(...)`, `DIO(...)`, etc.) default to `vmx = std::make_shared<VMXPi>(true, 50)`, and `Imu()` creates a new one outright. If users instantiate Studica classes without arguments, each device creates another `VMXPi`.
  - **Do not modify `studica_drivers` (project rule).** Therefore, its defaults cannot be changed. Instead, the Studica plugin library exposes one process-wide instance as `wpilibvmx::SharedVMX()` (`wpi/halsim/vmx/SharedVMX.hpp`; implemented, and the extension's Studica backend uses the same instance). **Tell users to pass this instance when creating Studica objects**, for example: `studica_driver::Titan titan(42, 20000, 0.0f, wpilibvmx::SharedVMX());`. If they omit it, the extra-`VMXPi` trap remains (first verify whether that causes problems on hardware). The user program must link `halsim_vmx_studica`; C++ only, no JNI wrapper for Java.
  - Alternative: document "always pass the VMX instance provided by `halsim_vmx`" (risk of user error).
- **Titan watchdog:** The device enters a safe state after about 150–200 ms without commands. The ROS2 driver keeps the stopped state by resending all four motor speeds (including zero) at **50 Hz**. `halsim_vmx` must do the same.
- `examples/!watchdog_example/` contains only a `Makefile`, no source (the `!` in its name appears to mark it as excluded or unfinished). No useful reference is available there.
- The MIT-licensed **Studica-Robotics/GradleRIO** fork exists ("GradleRIO adapted to deploy for the VMX-Pi"). It appears old, and its README is sparse: it does not explain the WPILib version, host/path, or startup procedure and links to external `pdocs.kauailabs.com` docs. Read its `src/` (especially `WPIExtension.groovy`) separately as a reference when adding a VMX deploy target to our `GradleRIO`.

### Still unknown (requires hardware verification)

1. ~~Does creating multiple `VMXPi` instances in one process actually fail?~~ **Confirmed (2026-10-10, Ubuntu 26.04 / kernel 7.0): it fails.** Two instances produce repeated SPI write CRC errors and a shutdown segfault; one works. **`SharedVMX()` is required; do not instantiate Studica classes without passing it.** (`HARDWARE_CHECKLIST.md` §2)
2. **Channel-number mapping:** Complete the WPILib DIO/Analog/PWM ↔ `VMXChannelIndex` map in §4 after checking the VMX-pi pin map. The VMX HAL header (`VMXPi.h`, `/usr/local/include/vmxpi`) is needed; it is not on GitHub and is installed in the VMX OS image (README: `learn.studica.com/docs/ws/vmx/os-images`).
3. **Threading:** DIO interrupt callbacks run on a VMX background thread. Sim HAL updates must be thread-safe.
4. **Java support:** `studica_drivers` is C++ only. Java users need a JNI wrapper → recommend limiting the initial goal to **C++ only**.
5. **Python (RobotPy):** Could be a semiwrap binding target like `allwpilib/drivers`, but lower priority.

## 4. Channel map

### VMX channels (**measured** using the robot's `tools/vmx_channels`, 2026-10-10; VMX board model 0x32, hardware revision 60, firmware 3.0.436)

Based on `VMXChannelIndex`. **Not yet verified** whether these correspond to the pin numbers printed on the board (requires jumper-wire testing).

| Type | Channels | Capabilities |
|---|---|---|
| FlexDIO | 0–11 (12) | DigitalIn/Out, PWM generation (PWMGen/PWMGen2), PWM capture, interrupts. **Encoder A/B pairs: (0,1) (2,3) (4,5) (6,7) (8,9)** (even=EncA, odd=EncB). 10 and 11 have no encoder function. |
| HiCurrDIO | 12–21 (10) | **DigitalOut and PWMGen only** (no input). Direction is set by jumpers, so these are outputs on this robot. |
| AnalogIn | 22–25 (4) | Analog input (accumulator, analog trigger, interrupt). |
| CommDIO | 26–33 (8) | 26/27: DigitalIn/Out + PWMGen + interrupt + I2C SDA/SCL. 28 UART_TX (output), 29 UART_RX (input), 30 SPI_CLK, 31 SPI_MOSI (+LEDArray), 32 SPI_MISO (input), 33 SPI_CS. |

### WPILib channel → VMX channel (`HALSIMVMX_*_MAP`; configure at runtime, do not hard-code)

Teams choose mappings to match their wiring. The table above defines the available channels.

| WPILib type | Available VMX channels | Notes |
|---|---|---|
| DigitalInput | FlexDIO 0–11, CommDIO 26/27/29/32 | HiCurrDIO cannot be used as input. |
| DigitalOutput | FlexDIO 0–11, HiCurrDIO 12–21, CommDIO 26/27/28/30/31/33 | |
| AnalogInput | AnalogIn 22–25 (WPILib 0–3 → 22–25 is a natural mapping) | |
| Encoder(a, b) | FlexDIO pairs (0,1) (2,3) (4,5) (6,7) (8,9), **A=even, B=odd** | Other combinations may not be supported by hardware (unverified). The `DigitalInput` objects that WPILib `Encoder` creates for those same channels are claimed by the encoder. |
| Do not use | CommDIO 26–33 when assigned to I2C/UART/SPI | navX/Titan may be connected over SPI/CAN (unverified). |

> **It is not yet known whether reading/writing pins requires a channel-number offset.** On this kernel (7.0), missing sysfs GPIO affects interrupts only (fixed with `gpio_isr_shim`), and the channel number is the HAL's `VMXChannelIndex`, so this seems unrelated to the offset. If needed, `robot_manager` can pass `HALSIMVMX_*_MAP` through a platform profile. See `robot_manager/README.md`.

## 5. Patterns from the XRP implementation (for Path B)

Reference: `simulation/halsim_xrp/`

- Create the client in the extension entry point `HALSIM_InitExtension()` and clean it up with `HAL_OnShutdown` (`main.cpp`).
- Register only the needed HAL data: `AnalogIn`, `DIO`, `DriverStation`, `Encoder`, and `HAL` ("Minimized set of HAL providers" in `HALSimXRPClient.cpp`).
- Receive the robot code's `sim_periodic_after` signal and send all output values once per cycle (`HALSimXRP::OnSimValueChanged`).
- Map device names/channels using a hard-coded table (`XRP.cpp`: motorL→0, servo1→4, encoder channel pair 4/5→0, etc.). Use the same approach for our §4 map.
- **Difference:** XRP sends values to a remote board via UDP, while VMX calls `studica_driver::*` **directly in the same process**. No network layer (`uv::Udp`, tagged protocol, sequence numbers) is needed. Do not copy the UDP/packet code from `HALSimXRP.cpp` / `XRP.cpp`; implement only the two directions: "sim value changed callback → Studica call" and "Studica read → sim value update" (sim callbacks use `HALSIM_Register*Callback`).

## 6. Proposed implementation order

1. **Verify hardware** (item 1 in §3 "Still unknown") and obtain VMX HAL headers → complete §4 channel map.
2. (Done) `halsim_vmx` skeleton + `SharedVMX()` + `TitanEnableGuard`.
3. Connect standard classes from easiest to hardest: DIO → AnalogIn → Encoder → PWM/Servo → DutyCycle → IMU → I2C.
4. (Removed) Do not modify `studica_drivers`.
5. Minimal `robot_manager`: run/restart robot programs as root + platform profiles.
6. Add a VMX deploy target to our GradleRIO (refer to Studica-Robotics/GradleRIO).
