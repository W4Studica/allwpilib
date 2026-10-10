# halsim_vmx 설계 / 매핑표

> 상태: **초안**. `studica_drivers`(Studica-Robotics/ROS2 `f97edec`)와 allwpilib `2027.0.0-alpha-7` 헤더를 읽고 작성함.
> 실제 VMX-pi 하드웨어에서는 아직 아무것도 검증하지 못했음. "확인 필요" 표시는 하드웨어에서 확인할 항목.

## 0. 핵심 결정: Studica 함수를 그대로 쓴다

사용자 로봇 코드는 `studica_driver::Titan`, `Servo`, `Imu`, `DIO` 등 **Studica 기존 클래스를 직접 호출**한다.
이 클래스들은 내부에서 `VMXPi`(vmxpi_hal_cpp)를 통해 하드웨어에 직접 접근하므로 **sim HAL을 거치지 않는다.**

따라서 구조는 두 경로로 나뉜다.

```
경로 A (주 경로)  로봇 코드 --studica_driver::*--> vmxpi_hal_cpp --> VMX 하드웨어
경로 B (선택)     로봇 코드 --WPILib 표준 클래스--> sim HAL --halsim_vmx--> studica_driver::* --> VMX
공통               robot_manager: 프로그램 실행/재시작, 플랫폼 프로필 (MockDS는 사용자가 작성, 범위 밖)
```

- **경로 A에는 halsim_vmx가 필요 없다.** 필요한 것은 (1) 라이브러리 링크(`studica_drivers`), (2) enable 상태 전달(§2), (3) **프로세스당 VMXPi 하나를 공유**(§3).
- **경로 B(halsim_vmx)는 WPILib 표준 클래스로 연결할 수 있는 건 최대한 연결한다.** (결정됨) `DigitalInput`, `Encoder`, `AnalogInput`, `PWM` 등을 쓰는 코드/예제/서드파티 라이브러리가 VMX에서 그대로 돌게 하는 것이 목표.
  XRP 방식(`halsim_xrp`)을 본뜨되 UDP 대신 같은 프로세스 안에서 Studica 클래스를 직접 호출한다.
- **두 경로는 같은 프로세스, 같은 `VMXPi` 하나를 공유해야 한다**(§3). 그래서 `halsim_vmx`가 VMXPi를 소유하고, 사용자가 직접 만드는 Studica 객체에도 그 인스턴스를 넘겨준다.

## 0.5 백엔드 플러그인 (dlopen)

`libhalsim_vmx.so`는 VMX 헤더(`VMXPi.h`, 이 헤더는 VMX 이미지에만 있음)에 의존하지 않는다. 하드웨어 접근은 **백엔드 플러그인**(별도 `.so`)이 하고,
`HALSIMVMX_BACKEND`에 그 경로를 주면 `dlopen`으로 불러온다. 이렇게 해서 확장 본체는 교차 컴파일(`linuxarm64`)할 수 있고,
Studica 백엔드(`libhalsim_vmx_studica.so`)는 VMX 위에서 한 번만 빌드한다(`-DWPILIB_WITH_STUDICA=ON`, C++20 필요).

- 인터페이스는 **C 함수표**(`BackendApi.h`, ABI 버전 1)라서 두 라이브러리를 서로 다른 컴파일러/libstdc++로 빌드해도 된다(WPILib trixie 툴체인 대 VMX의 Ubuntu GCC).
  플러그인은 `wpilibvmx_CreateBackendV1` / `wpilibvmx_DestroyBackendV1`을 내보낸다. 어떤 `VmxBackend`든 `BackendPlugin.hpp`의 `MakeBackendApi`로 감싸서 내보낼 수 있다.
- `HALSIMVMX_BACKEND`가 **없으면** loopback(메모리 안의 가짜 백엔드)을 쓴다(PC 시뮬레이션용, 하드웨어는 안 움직임, 로그에 표시).
- `HALSIMVMX_BACKEND`를 **줬는데 로드에 실패하면 확장 초기화를 실패**시킨다(`-1`). loopback으로 조용히 넘어가지 않는다. 로봇 코드가 정상 동작하는 것처럼 보이는데 하드웨어는 안 움직이는 게 가장 위험하기 때문이다.
- 플러그인이 ABI 버전이나 구조체 크기가 다르면 거부한다. 예외는 경계를 넘지 않는다.
- 검증: PC에서 테스트용 플러그인(loopback을 C ABI로 감싼 것)으로 로드, 정상 동작, 없는 파일, 플러그인이 아닌 라이브러리, ABI 불일치, 환경변수 동작을 테스트했다.
  **`StudicaBackend.cpp`는 여전히 컴파일/실행해 보지 못했다.**

## 1. 매핑표 (경로 B: WPILib 표준 클래스 → sim HAL → Studica 클래스)

"sim HAL" 열은 `hal/.../simulation/*Data.h`의 HALSIM 필드명.

| WPILib 클래스 | sim HAL 데이터 | 방향 | Studica 클래스 / 메서드 | 비고 |
|---|---|---|---|---|
| `DigitalOutput` | `DIO` `Initialized`, `IsInput=false`, `Value` | sim→HW | `DIO(ch, PinMode::OUTPUT)`, `Set(bool)` | 채널 번호 표 필요(§4) |
| `DigitalInput` | `DIO` `Initialized`, `IsInput=true`, `Value` | HW→sim | `DIO(ch, PinMode::INPUT)`, `Get()` | 인터럽트는 `EnableInterrupt()`로 대응 가능 |
| `AnalogInput` | `AnalogIn` `Initialized`, `Voltage` | HW→sim | `AnalogInput(port)`, `GetAverageVoltage(float&)` | 반환이 bool+out 인자 |
| `Encoder` | `Encoder` `Initialized`, `DigitalChannelA/B`, `Count`, `Reset`, `ReverseDirection` | HW→sim | `Encoder(port_a, port_b)`, `GetCount()` | `Reset`/`DistancePerPulse`는 래퍼에서 처리(Studica Encoder에 reset 없음) |
| `PWM`/`Servo` | `PWM` `Initialized`, `PulseMicrosecond`, `OutputPeriod` | sim→HW | (보류) | **보류.** Studica `PWM`에는 값을 쓰는 공개 함수가 없고(`Servo::SetAngle/SetSpeed`만 출력), 정수 범위→duty tick 변환이라 마이크로초 펄스폭과 맞지 않음. `studica_drivers`를 수정할 수 없으므로 사용자는 Studica `Servo`를 직접 사용. `SetAngle`은 호출마다 `printf`함 |
| `DutyCycleEncoder` | `DutyCycle` `Initialized`, `Frequency`, `Output` | HW→sim | (보류) | **보류.** Studica 쪽은 12비트(4095) 절대 엔코더용 VMX 캡처로 **도(degree) 값만** 반환하고 원시 duty/주파수를 노출하지 않음. 되돌려 계산하려면 센서 프로토콜을 가정해야 해서 조용히 틀린 값이 나올 위험 |
| IMU (`OnboardIMU` 계열) | `IMU` `Yaw`, `AngleX/Y/Z`, `GyroRateX/Y/Z`, `AccelX/Y/Z` (setter만 있고 전역 1개, "초기화됨" 신호 없음) | HW→sim | `Imu`(navX): `GetYaw/Pitch/Roll`, `GetRawGyroX/Y/Z`, `GetRawAccelX/Y/Z`, `IsConnected`, `IsCalibrating` | **구현됨.** 신호가 없어서 `HALSIMVMX_IMU=1`로 명시적으로 켬. 변환(**하드웨어 미검증 가정**): 도→라디안, deg/s→rad/s, g→m/s². navX yaw와 Z축 각속도는 시계 방향이 양수이고 WPILib은 반시계 방향이 양수라서 **부호를 뒤집음**. roll/pitch/X·Y 각속도/가속도는 그대로 전달. 연결 안 됨/보정 중이면 값을 갱신하지 않음. 쿼터니언은 sim HAL에 setter가 없어 미지원 |
| `I2C` | `I2C` `Initialized`, Read/Write **버퍼 콜백** | 양방향 | (보류) | **보류.** sim HAL이 `deviceAddress`를 콜백에 넘기지 않음(`hal/src/main/native/sim/mockdata/I2CData.cpp`: 주소는 받지만 `write(buf, size)`/`read(buf, count)`만 호출). 그래서 어떤 장치로 가는 요청인지 알 수 없고, `transaction`도 쓰기 콜백 뒤에 읽기 콜백이 따로 호출돼 repeated-start 쌍을 알 수 없음. 주소를 넘기려면 allwpilib 본체 수정이 필요해 merge 방침에 어긋남. 우회(주소를 환경변수로 하나만 고정)는 다중 장치 버스에서 쓸 수 없음. Studica I2C 장치(`Cobra` 등)는 Studica API로 직접 사용 |
| `DriverStation` | `DriverStation` `Enabled`, `RobotMode`, `OpMode`, `EStop`, `DsAttached`, `Joystick*` | 사용자의 MockDS가 채움 | (하드웨어 클래스 없음) | 우리는 읽기만 함. §2 |

### Studica에만 있고 WPILib 표준 대응이 없는 장치

표준 sim HAL로 표현할 수 없으므로 **경로 A(Studica API 직접 사용)** 로만 쓴다. (경로 B로 노출하려면 XRP처럼 `SimDevice` 기반 vendordep 클래스가 필요 — 후순위)

| Studica 클래스 | 용도 | 비고 |
|---|---|---|
| `Titan` | CAN 모터 컨트롤러(4채널, 엔코더, RPM/위치 PID, 리밋스위치, 온도) | WPILib에는 PWM/CAN 모터 표준 HAL 경로가 안 맞음. **enable/keepalive 규칙이 중요(§2)** |
| `Ultrasonic` | 초음파 `Ping()`, `GetDistanceMM/IN()` | WPILib `Ultrasonic`과 모양이 다름 |
| `Sharp` | 적외선 거리 (`AnalogInput` 기반) | |
| `Cobra` | 라인 센서(I2C ADC) | |
| `Colore`, `Parsec`, `*_usb` | 색상/ToF 센서 | CAN/USB |
| `LightTower` | 신호등/부저 | |

## 2. Enable / MockDS / 안전 (가장 중요한 연결점)

`Titan`은 장치 규칙이 있다 (`titan.hpp` 주석):
- 전원 켜면 **disabled**. `Enable(true)`를 호출해야 `SetSpeed`/`SetTargetVelocity`가 먹는다.
- CAN 메시지가 **200 ms** 없으면 장치가 스스로 정지+disabled. 제어 중에는 ≤150 ms(`TITAN_CAN_KEEPALIVE_MS`)마다 명령을 보내야 함.

→ **로봇 enable 상태와 Titan enable를 연결해야 한다.** VMX를 여는 것은 로봇 프로그램(WPILib은 단일 프로세스)뿐이고 `robot_manager`는 VMX를 열지 않으므로(§3), 안전은 **로봇 프로그램 안**에서 처리한다.

| 방안 | 내용 | 비고 |
|---|---|---|
| **S1 (시작점)** | 사용자 코드가 `DriverStation::IsEnabled()`로 `titan.Enable(...)`을 호출. 헬퍼 제공 | Studica 클래스 수정 없음. 사용자가 빼먹을 수 있음 |
| **S4 (구현됨)** | **`TitanEnableGuard<TitanT>`**(헤더 전용 템플릿, `studica_drivers` 밖): HAL 컨트롤 워드(`HAL_GetUncachedControlWord`)에서 enable/E-stop을 읽어 **상태가 바뀔 때만** `titan.Enable()` 호출. 중지/소멸자에서는 항상 `Enable(false)`. 모터 명령은 보내지 않으므로 사용자 코드가 enabled 동안 150 ms 이내로 `SetSpeed`를 계속 보내야 함. `Enable(bool)`만 있으면 되는 템플릿이라 하드웨어 없이 테스트함 | **주의: sim HAL은 `DsAttached`가 true일 때만 컨트롤 워드를 채운다.** 사용자의 MockDS는 `setEnabled(true)`와 함께 `setDsAttached(true)`도 해야 함. Titan은 가드보다 먼저 만들어야 함 |
| 하드웨어 안전망 | 프로세스가 죽어도 Titan이 **200 ms 후 스스로 정지**(`titan.hpp` 주석, ROS2 README의 "CAN Watchdog") | 소프트웨어 버그에 대비한 마지막 방어선 |

**MockDS는 사용자가 로봇 코드로 작성한다(이 프로젝트 범위 밖).** MockDS가 sim의 `DriverStation` 데이터(`Enabled`, `RobotMode`, `EStop`)를 채우면, 우리 쪽(Titan 헬퍼, 안전 처리)은 그 값을 **읽기만** 한다. `robot_manager`도 관여하지 않는다.

## 3. GitHub 조사 결과와 열린 질문

### 확인된 것 (Studica-Robotics/ROS2 `f97edec` 코드와 README 근거)

- **VMX(`pigpio`)는 한 프로세스가 연다.** README/`setup_permissions.sh`에 "pigpio가 `/dev/mem`에 접근하고 **PID 파일을 잠그기 때문에 root 필요**"라고 되어 있고, `studica_control`도 실패 시 "pigpio may be unavailable or in use"라고 로그를 남긴다. WPILib 로봇 프로그램은 어차피 단일 프로세스이므로 이 제약은 문제가 되지 않는다.
  - **VMX를 여는 것은 로봇 프로그램 프로세스 하나.** `robot_manager`는 VMX를 열지 않고 프로세스 수명 관리만 한다. (진단 도구 등 다른 프로세스가 VMX를 열지 않도록만 주의)
- **실행은 root(sudo)로 해야 한다.** deploy 스크립트와 `robot_manager` 서비스가 root로 로봇 프로그램을 띄워야 함.
- **VMXPi는 프로세스 안에서 하나를 만들어 공유하는 것이 검증된 패턴이다.** `studica_control`은 `manual_composition.cpp`에서 `VMXPi(true, 50)`를 **한 번만** 만들어 모든 컴포넌트에 `shared_ptr`로 넘긴다.
- **Studica 클래스 생성자의 기본 인자는 함정이다.** `Titan(...)`, `Servo(...)`, `Encoder(...)`, `DIO(...)` 등 대부분이 `vmx = std::make_shared<VMXPi>(true, 50)`가 기본값이고, `Imu()`는 아예 새로 만든다. 사용자가 인자 없이 Studica 클래스를 쓰면 **VMXPi가 장치 수만큼 생긴다.**
  - **`studica_drivers`는 수정하지 않는다(규칙).** 그래서 기본 인자를 바꿀 수 없다. 대신 Studica 플러그인 라이브러리가 `wpilibvmx::SharedVMX()`(`wpi/halsim/vmx/SharedVMX.hpp`)로 프로세스 공유 인스턴스를 노출한다(구현됨, 확장의 Studica 백엔드도 같은 인스턴스를 씀). **사용자는 Studica 객체를 만들 때 이 인스턴스를 인자로 넘기는 것을 규칙으로 안내**한다: `studica_driver::Titan titan(42, 20000, 0.0f, wpilibvmx::SharedVMX());`. 인자를 빼먹으면 `VMXPi`가 하나 더 생기는 함정이 남는다(하드웨어에서 실제로 문제인지부터 확인). 사용자 프로그램이 `halsim_vmx_studica`를 직접 링크해야 하며, C++ 전용이다(Java용 JNI 래퍼는 없음).
  - 대안: 문서로 "항상 `halsim_vmx`가 주는 VMX를 넘겨라"라고 안내(사용자 실수 위험).
- **Titan 워치독**: 장치가 약 150~200 ms 명령이 없으면 안전 상태로 들어가고, ROS2 드라이버는 4개 모터 속도(0 포함)를 **50 Hz로 계속 재전송**해서 "정지"를 유지한다. halsim_vmx도 같은 방식이어야 한다.
- `examples/!watchdog_example/`에는 **`Makefile`만 있고 소스가 없다**(이름의 `!`는 제외/미완 표시로 보임). 참고 자료 없음.
- **Studica-Robotics/GradleRIO**(MIT, "GradleRIO adapted to deploy for the VMX-Pi")가 존재한다. 오래된 포크로 보이며 README는 빈약하다(WPILib 버전, 호스트/경로, 시작 방식 설명 없음, 외부 `pdocs.kauailabs.com` 문서로 연결). **우리 `GradleRIO`에 VMX deploy 타깃을 만들 때 참고할 만한 자료**이므로 `src/`(특히 `WPIExtension.groovy`)를 따로 읽어볼 것.

### 아직 모르는 것 (하드웨어 확인)

1. ~~한 프로세스 안에서 `VMXPi` 인스턴스를 여러 개 만들면 실제로 실패하는가?~~ **확인됨(2026-10-10, Ubuntu 26.04 / 커널 7.0): 실패한다.** 2개를 만들면 SPI 쓰기 CRC 오류가 쏟아지고 종료 때 Segfault, 1개는 정상이다. **`SharedVMX()`는 필수이고, Studica 클래스를 인자 없이 만드는 것은 금지다.** (`HARDWARE_CHECKLIST.md` §2)
2. **채널 번호 체계**: WPILib DIO/Analog/PWM 번호 ↔ `VMXChannelIndex` 대응표(§4)는 VMX-pi 핀맵 확인 후 작성. VMX HAL 헤더(`VMXPi.h`, `/usr/local/include/vmxpi`)가 필요한데 GitHub에는 없고 VMX OS 이미지에 설치되어 있음(README: `learn.studica.com/docs/ws/vmx/os-images`).
4. **스레딩**: `DIO` 인터럽트 콜백은 VMX 백그라운드 스레드에서 실행됨. sim HAL 갱신은 스레드 안전하게 해야 함.
5. **Java 지원**: `studica_drivers`는 C++ 전용. Java 사용자는 JNI 래퍼가 필요 → **1차 목표는 C++만**으로 한정 권장.
6. **Python(RobotPy)**: `allwpilib/drivers`처럼 semiwrap 바인딩 대상이 될 수 있으나 후순위.

## 4. 채널 번호 표

### VMX 쪽 채널 지도 (**측정됨**: 로봇의 `tools/vmx_channels`, 2026-10-10, VMX 보드 모델 0x32, 하드웨어 rev 60, 펌웨어 3.0.436)

`VMXChannelIndex` 기준. 보드에 인쇄된 핀 번호와 같은지는 **아직 확인 안 함**(점퍼선으로 확인 필요).

| 종류 | 채널 | 기능 |
|---|---|---|
| FlexDIO | 0 - 11 (12개) | DigitalIn/Out, PWM 생성(PWMGen/PWMGen2), PWM 캡처, 인터럽트. **엔코더 A/B 쌍: (0,1) (2,3) (4,5) (6,7) (8,9)** (짝수=EncA, 홀수=EncB). 10, 11은 엔코더 기능 없음 |
| HiCurrDIO | 12 - 21 (10개) | **DigitalOut, PWMGen만**(입력 불가). 방향은 점퍼 설정이라 이 로봇에서는 출력 |
| AnalogIn | 22 - 25 (4개) | 아날로그 입력(누산기, 아날로그 트리거, 인터럽트) |
| CommDIO | 26 - 33 (8개) | 26/27: DigitalIn/Out + PWMGen + 인터럽트 + I2C SDA/SCL. 28 UART_TX(출력), 29 UART_RX(입력), 30 SPI_CLK, 31 SPI_MOSI(+LEDArray), 32 SPI_MISO(입력), 33 SPI_CS |

### WPILib 번호 -> VMX 채널 (`HALSIMVMX_*_MAP`로 정한다. 코드에 박지 않는다)

매핑은 팀이 배선에 맞춰 정한다. 가능한 채널은 위 표가 정한다.

| WPILib 종류 | 쓸 수 있는 VMX 채널 | 비고 |
|---|---|---|
| DigitalInput | FlexDIO 0-11, CommDIO 26/27/29/32 | HiCurrDIO는 입력 불가 |
| DigitalOutput | FlexDIO 0-11, HiCurrDIO 12-21, CommDIO 26/27/28/30/31/33 | |
| AnalogInput | AnalogIn 22-25 (WPILib 0-3 -> 22-25가 자연스러움) | |
| Encoder(a, b) | FlexDIO 쌍 (0,1) (2,3) (4,5) (6,7) (8,9), **A=짝수 B=홀수** | 다른 조합은 하드웨어가 지원하지 않을 수 있음(미확인). WPILib `Encoder`가 같은 채널에 만드는 `DigitalInput`은 엔코더가 가져간다 |
| 사용하지 말 것 | CommDIO 26-33 중 I2C/UART/SPI로 쓰는 채널 | navX/Titan 등이 SPI/CAN으로 연결돼 있을 수 있음(미확인) |

> **핀 읽기/쓰기에 번호 오프셋이 필요한지는 아직 모른다.** 이 커널(7.0)에서는 sysfs GPIO가 없어 인터럽트만 영향이 있고(`gpio_isr_shim`로 해결), 채널 번호는 HAL의 `VMXChannelIndex`라서 오프셋 문제와는 별개로 보인다. 필요하면 `robot_manager`가 플랫폼 프로필로 `HALSIMVMX_*_MAP`을 정해서 넘긴다. `robot_manager/README.md` 참고.

## 5. XRP 구현에서 가져올 패턴 (경로 B를 만들 때)

참고: `simulation/halsim_xrp/`

- 확장 진입점 `HALSIM_InitExtension()` 에서 클라이언트 객체 생성, `HAL_OnShutdown`으로 정리 (`main.cpp`).
- 필요한 HAL 데이터만 골라 등록: `AnalogIn`, `DIO`, `DriverStation`, `Encoder`, `HAL` (`HALSimXRPClient.cpp`의 "Minimized set of HAL providers").
- 로봇 코드의 `sim_periodic_after` 신호를 받아 한 주기마다 출력값을 일괄 전송 (`HALSimXRP::OnSimValueChanged`).
- 장치 이름/채널을 하드코딩된 표로 매핑 (`XRP.cpp`: motorL→0, servo1→4, 엔코더 채널쌍 4/5→0 ...). 우리도 같은 방식으로 §4 표를 코드에 둠.
- **차이점**: XRP는 값을 UDP 패킷으로 원격 보드에 보내지만, VMX는 같은 프로세스 안에서 `studica_driver::*`를 **직접 호출**한다. 즉 네트워크 계층(`uv::Udp`, 태그 프로토콜, 시퀀스 번호)이 필요 없다. `HALSimXRP.cpp`/`XRP.cpp`의 UDP·패킷 코드는 가져오지 않고, "sim 값 변경 콜백 → Studica 호출 / Studica 읽기 → sim 값 갱신" 두 방향만 구현하면 됨. (sim 콜백은 `HALSIM_Register*Callback` 사용)

## 6. 구현 순서 제안

1. **하드웨어 확인**(§3 "아직 모르는 것" 1번)과 VMX HAL 헤더 확보 → §4 채널표 작성.
2. (완료) `halsim_vmx` 골격 + `SharedVMX()` + `TitanEnableGuard`.
3. 표준 클래스 연결을 쉬운 것부터: DIO → AnalogIn → Encoder → PWM/Servo → DutyCycle → IMU → I2C.
4. (삭제됨) `studica_drivers`는 수정하지 않는다.
5. `robot_manager` 최소 버전: root로 로봇 프로그램 실행/재시작 + 플랫폼 프로필.
6. 우리 GradleRIO에 VMX deploy 타깃(Studica-Robotics/GradleRIO 참고).
