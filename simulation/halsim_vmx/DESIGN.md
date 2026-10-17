# halsim_vmx design

`halsim_vmx` is a HAL simulation extension that connects WPILib's standard classes to a Studica VMX-pi. Measured results and open items are in `HARDWARE_CHECKLIST.md`.

## 1. Two paths

```text
Path A (devices without a WPILib equivalent, such as Titan)
   robot code --studica_driver::*--> vmxpi_hal_cpp --> VMX hardware
Path B (standard WPILib classes)
   robot code --WPILib classes--> sim HAL --halsim_vmx--> studica_driver::* --> VMX
```

- Path B lets code, examples and third-party libraries that use `DigitalInput`, `Encoder`, `AnalogInput` and the IMU run on a VMX unchanged. It follows `halsim_xrp`, but calls the Studica classes in the same process instead of sending UDP.
- Both paths share **one `VMXPi` per process**. Two break the SPI link (measured), so Studica objects always get `wpilibvmx::SharedVMX()`; their default arguments would create another `VMXPi`, and `studica_drivers` is never modified.
- Only the robot program opens the VMX (pigpio, root, one process). `robot_manager` does not.
- Robot programs are C++ only.

Environment variables read by the extension: `HALSIMVMX_BACKEND`, `HALSIMVMX_DIO_MAP`, `HALSIMVMX_ANALOG_MAP`, `HALSIMVMX_PWM_MAP`, `HALSIMVMX_IMU`.

## 2. Backend plugin

`libhalsim_vmx.so` does not need `VMXPi.h` (only the VMX image has it). Hardware access is in a plugin loaded with `dlopen` from `HALSIMVMX_BACKEND` (`libhalsim_vmx_studica.so`), so the extension cross-compiles for `linuxarm64`.

- The interface is a C function table (`BackendApi.h`, ABI version 2), so the two libraries may use different compilers. The plugin exports `wpilibvmx_CreateBackendV2` / `wpilibvmx_DestroyBackendV2`; `MakeBackendApi` in `BackendPlugin.hpp` wraps any `VmxBackend`. A different ABI version or structure size is rejected; exceptions do not cross the boundary.
- `HALSIMVMX_BACKEND` unset: the in-memory loopback backend (PC simulation), logged. Set but failing to load: extension initialisation fails (`-1`). It never falls back to loopback, because robot code that looks normal while no hardware moves is the dangerous outcome.
- The release and debug HALs have different SONAMEs, so a release build produces `libhalsim_vmx.so` and `libhalsim_vmxd.so`, sharing one backend and driver library (`tools/w4studica/`).
- The extension makes SIGINT and SIGTERM end the process after the VMX HAL's clean-up (`TerminationHandlers`); see `HARDWARE_CHECKLIST.md`.

## 3. Mapping (WPILib class to sim HAL to Studica class)

| WPILib class | Sim HAL data | Studica class | Notes |
|---|---|---|---|
| `DigitalOutput` | `DIO` `Initialized`, `IsInput=false`, `Value` | `DIO(ch, OUTPUT)`, `Set` | channel map required |
| `DigitalInput` | `DIO` `IsInput=true`, `Value` | `DIO(ch, INPUT)`, `Get` | polled; no interrupts |
| `AnalogInput` | `AnalogIn` `Voltage` | `AnalogInput(port)`, `GetAverageVoltage` | |
| `Encoder` | `Encoder` `Count`, `Reset`, `ReverseDirection` | `Encoder(a, b)`, `GetCount` | the wrapper implements `Reset`, direction and `DistancePerPulse` (the Studica class has no reset); the encoder takes the A/B channels over from the `DigitalInput`s WPILib creates |
| `OnboardIMU` | `IMU` yaw, angles, rates, acceleration | `Imu` (navX) | `HALSIMVMX_IMU=1` (the sim HAL has no "initialized" signal). degrees to radians, deg/s to rad/s, g to m/s2. navX yaw and Z rate are clockwise-positive, WPILib counterclockwise-positive, so those two are negated; roll, pitch and the other rates pass through. Not updated while disconnected or calibrating. **`GetQuaternion()` and `GetRotation3d()` do not work**: the sim `HAL_GetIMUQuaternion` returns zeros and fixing it means editing allwpilib (decided against). Build the rotation from the angles instead: `wpi::math::Rotation3d{imu.GetAngleX(), imu.GetAngleY(), imu.GetYaw()}` (roll, pitch, yaw; the roll/pitch signs are not verified on hardware). For the sensor's own quaternion, connection and calibration state, use the Studica `Imu` directly (do not run it together with `HALSIMVMX_IMU=1`). The sim IMU ignores the mount orientation (`FLAT`, `LANDSCAPE` and `PORTRAIT` return the same angles); use `FLAT`. |
| `PWM`, `Servo`, motor controllers | `PWM` pulse width and output period | VMX PWM generator, called directly | `HALSIMVMX_PWM_MAP`. Not `studica_driver::PWM/Servo`: they cannot write a pulse width and `Servo` prints a line on every change. A tick is 4 us. A pulse of 0 (disabled) leaves the line low, so a servo goes limp. Channels that share a VMX timer must use one frame length (not measured). |
| `DutyCycleEncoder` | `DutyCycle` | deferred | the Studica capture returns degrees only, with no raw duty or frequency; would need VMX input capture called directly |
| `I2C` | read/write buffer callbacks | deferred | the sim HAL does not pass `deviceAddress` to the callback, so the target device is unknown; fixing that means editing allwpilib (decided against: no upstream source changes for this). Use the Studica I2C devices (such as `Cobra`) directly. |
| `DriverStation` | `Enabled`, `EStop`, `DsAttached`, ... | none | filled by the team's own MockDS; the extension only reads it |

The battery (system) voltage is read from `VMXPower` about every 100 ms into the sim RoboRIO input voltage, so `RobotController::GetBatteryVoltage()` is real.

Studica-only devices (`Titan`, `Ultrasonic`, `Sharp`, `Cobra`, `Colore`, `Parsec`, `LightTower`) are used through the Studica API (path A).

## 4. Channel map

Measured with `tools/vmx_channels` (`VMXChannelIndex`, VMX board model 0x32). Whether these indices match the numbers printed on the board has not been checked.

| Type | Channels | Capabilities |
|---|---|---|
| FlexDIO | 0 to 11 | DigitalIn/Out, PWM generation and capture, interrupts. **Encoder pairs (0,1) (2,3) (4,5) (6,7) (8,9): A even, B odd.** 10 and 11 have no encoder function. |
| HiCurrDIO | 12 to 21 | DigitalOut and PWMGen only (jumper-selected direction; outputs on this robot) |
| AnalogIn | 22 to 25 | analog input (accumulator) |
| CommDIO | 26 to 33 | 26/27 DigitalIn/Out + PWMGen + I2C SDA/SCL; 28 UART TX; 29 UART RX; 30 SPI CLK; 31 SPI MOSI; 32 SPI MISO; 33 SPI CS |

| WPILib type | VMX channels | Notes |
|---|---|---|
| DigitalInput | FlexDIO 0 to 11, CommDIO 26/27/29/32 | HiCurrDIO cannot be an input |
| DigitalOutput | FlexDIO 0 to 11, HiCurrDIO 12 to 21, CommDIO 26/27/28/30/31/33 | |
| AnalogInput | AnalogIn 22 to 25 | WPILib 0 to 3 maps naturally to 22 to 25 |
| Encoder(a, b) | the FlexDIO pairs above | other pairs are not supported by the hardware |
| PWM | FlexDIO 0 to 11, HiCurrDIO 12 to 21, CommDIO 26/27 | channels with PWM generation; checked with `ChannelSupportsCapability` |
| avoid | CommDIO 26 to 33 while used for I2C/UART/SPI | |

Maps are given at run time (`HALSIMVMX_DIO_MAP="0:0,1:1"`, `HALSIMVMX_ANALOG_MAP="0:22"`, `HALSIMVMX_PWM_MAP="0:12"`; entries `<wpilib>:<vmx>` separated by commas, no spaces). Malformed entries and two WPILib channels on one VMX channel are reported; Analog and Encoder channels are checked with `ChannelSupportsCapability` before they are claimed. No channel-number offset was needed on the development robot, so `robot_manager` platform profiles stay empty.

## 5. Enable, MockDS and safety

A Titan powers up **disabled** and ignores commands until it is enabled; it stops itself if it hears no CAN message for **200 ms**, so robot code must send `SetSpeed` at least every 150 ms. The robot's enabled state must therefore reach the Titan, and the robot program, not `robot_manager`, is responsible for it.

- **`TitanEnableGuard<T>`** (header only, outside `studica_drivers`) reads enabled and E-stop from the HAL control word (`HAL_GetUncachedControlWord`), sends `Enable()` when the state changes, repeats the current state with `KeepAlive()` every 20 ms, and always sends `Enable(false)` when stopped or destroyed. It sends no motor commands. The sim HAL fills the control word only while `DsAttached` is true, so the team's MockDS must set both `setDsAttached(true)` and `setEnabled(...)`.
- Do not hand a `studica_driver::Titan` to the guard: `Titan::Enable()` hands the frame repetition to the VMX board, which keeps sending after the program dies. Wrap the Titan in **`wpilibvmx::StudicaTitan`**, which sends single frames (it also stops repeats left on the board by an earlier run) and reports CAN send failures. Create the Titan before the guard.
- **`ControlLoopWatchdog`** (optional) gives the guard's enabled source two independent deadlines: the main control loop (`FeedControlLoop()` after each completed cycle) and fresh Driver Station data (`FeedDriverStation()` after each valid packet). It starts disabled until both have been fed. Feed it from the real events, never from a timer or the guard. The stop latency on a robot is not measured yet.
- The Titan's own 200 ms stop is the last line of defence.

MockDS is user code and out of scope here.

## 6. Status

| Area | Status |
|---|---|
| DIO, AnalogIn, Encoder, IMU yaw and Z acceleration, Titan + guard | implemented and verified on a VMX-pi |
| IMU roll/pitch and Z rate, `ControlLoopWatchdog` latency | implemented, not verified on hardware |
| PWM, battery voltage | implemented, compiled for arm64, **not verified on hardware** |
| I2C, DutyCycle, DutyCycleEncoder, IMU quaternion, DIO interrupts | not done (section 3). The 2027 alpha-7 HAL has no interrupt API; the sim `HAL_GetIMUQuaternion` returns zeros |
| Release builds, VS Code extension | beta; see the repositories' README files |
