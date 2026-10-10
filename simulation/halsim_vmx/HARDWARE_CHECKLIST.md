# VMX 실기기 검증 체크리스트

PC에서는 `halsim_vmx` 테스트 35개, `GradleRIO` 테스트, `robot_manager` 테스트 13개가 통과했다. **VMX 실기기에서는 아무것도 확인되지 않았다.**
이 문서는 실기기에서 확인할 것을 순서대로 적은 것이다. 각 단계는 *무엇을 하는지 / 기대 결과 / 실패하면*으로 되어 있다.

- `[ ]`에 체크하고, 마지막의 **결과 기록표**를 채운다.
- "(명령 미검증)"은 이 문서를 쓴 사람이 실행해 보지 못한 명령이다. 그대로 되지 않으면 고쳐서 이 문서에 반영한다.
- **모터가 있는 테스트(§6, §7)는 바퀴가 바닥에 닿지 않게 로봇을 띄우고, 낮은 속도로 시작하고, 전원을 바로 끌 수 있는 상태에서 한다.**

의존 관계: §0 → §1 → (§2, §3, §4 병행 가능) → §5 → §6 → §7 → §8. 앞 단계가 실패하면 그 뒤는 의미가 없다.

---

## 0. 환경 기록

실행 결과를 결과 기록표에 적는다.

```bash
cat /etc/os-release | head -4
uname -r
cat /proc/device-tree/model; echo        # Raspberry Pi 모델
ldd --version | head -1                  # glibc
g++ --version | head -1                  # 없으면 "없음"
java -version 2>&1 | head -1             # 없으면 "없음"
python3 --version
```

- [ ] OS, 커널, Pi 모델, glibc, g++, java, python을 기록했다.

**이미 확인된 값(2026-10-10, 개발용 VMX 한 대, SSH 로그인 출력에서):** Ubuntu 26.04.1 LTS, 커널 `7.0.0-1020-raspi`(aarch64),
glibc 2.43, OpenJDK 25.0.4.1(`openjdk-25-jre-headless`), `vmxpi-hal_1.0~20240704` 설치 성공.
같은 로봇에서 추가로 확인됨: **Raspberry Pi 4 Model B Rev 1.5**, Python 3.14.4, **g++ 없음**, **`/usr/local/include/vmxpi` 없음(헤더 없음)**,
`libvmxpi_hal_cpp.so`의 의존 라이브러리(`libstdc++`, `libgcc_s`, `libc`, `libm`, `ld-linux`)가 **모두 해석됨**(`ldd`에 `not found` 없음).
디바이스: `/dev/mem`(root:kmem 0640), `/dev/gpiomem`(root 0600), `/dev/i2c-1`(dialout), `/dev/spidev0.0`/`0.1`(dialout) 존재.
커널 설정: `CONFIG_DEVMEM=y`, `CONFIG_STRICT_DEVMEM=y`, `IO_STRICT_DEVMEM` 꺼짐 — 주변장치 레지스터 접근은 막히지 않을 가능성이 있으나 **실행해서 확인해야 한다**(root로 실행).
커널이 6.6보다 훨씬 새로워서, 핀 번호 오프셋(§3)이 실제로 생길 가능성이 있다.
**g++가 없으므로 §4(플러그인 빌드) 전에 `sudo apt install -y g++ make`가 필요하다. `VMXPi.h`는 여전히 구해야 한다.**

참고(이 PC에서 확인한 사실): WPILib 2027의 arm64 툴체인은 `aarch64-trixie-linux-gnu`(Debian 13, GCC 14.3, glibc 2.41)뿐이다.
Studica 문서상 VMX 공식 이미지는 Ubuntu 22.04(glibc 2.35)이다.

## 1. Studica 기본 스택이 이 OS에서 동작하는가

이게 안 되면 나머지는 의미가 없다. 이 OS에서 `vmxpi_hal_cpp`/`pigpio`가 도는지가 OS 선택(22.04 vs 26.04 vs Debian 13)을 가른다.

1. `vmxpi_hal_cpp`가 설치돼 있는지 확인한다: `ls /usr/local/include/vmxpi/VMXPi.h /usr/local/lib/vmxpi/`
   (공식 이미지에는 미리 설치돼 있다고 Studica ROS2 README에 적혀 있다. 다른 OS면 설치 방법은 Studica 문서를 확인한다.)

   > **`vmxpi-hal_1.0~20240704_arm64.deb`(프로젝트 루트에 있던 비공식 재패키징, 공식 Ubuntu 22.04 ROS2 Humble 이미지에서 추출)를 읽기만 해서 확인한 것:**
   > - 라이브러리(`/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so` 등)만 들어 있고 **`VMXPi.h` 헤더는 없다.** 이 deb만으로는 `studica_drivers`/플러그인을 컴파일할 수 없다. 헤더는 공식 이미지의 `/usr/local/include/vmxpi/`에서 가져와야 한다.
   > - `libvmxpi_hal_cpp.so`가 필요로 하는 것: 최대 `GLIBC_2.34`, `GLIBCXX_3.4.29`, 외부 라이브러리는 `libstdc++`/`libgcc_s`/`libc`뿐. 상위 호환이라 22.04보다 새로운 OS에서도 로드될 가능성이 높다(실행해 본 것은 아님).
   > - pigpio는 별도 `.so`가 아니라 라이브러리 안에 들어 있는 것으로 보인다(`PIGPIOClient` 심볼). 동작 방식은 모른다.
   >
   > **헤더가 들어 있는 두 번째 deb: `vmx-hal_1.0~20240704_arm64.deb`**(역시 비공식 재패키징, 13MB, 설명: "libraries, headers, examples and tools").
   > - `/usr/local/include/vmxpi/VMXPi.h` 등 헤더 23개, `libvmxpi_hal_cpp.so`/`.a`, `studica_drivers` 라이브러리, 테스트 도구(`/usr/local/bin`), `/usr/local/startup/initenv.sh`(YDLIDAR udev 규칙을 만드는 스크립트일 뿐이며 설치 때 자동 실행되지 않음)가 들어 있다. 설치 스크립트는 `ldconfig`만 한다.
   > - **`vmxpi-hal`과 같은 파일(`/usr/local/lib/vmxpi/*`)을 설치하므로 충돌한다.** 먼저 `sudo apt remove -y vmxpi-hal` 한 뒤 `sudo apt install -y ./vmx-hal_1.0~20240704_arm64.deb` 한다.
   > - 이 PC에서 이 헤더로 컴파일해 본 결과(링크/실행 아님, x86 오브젝트): `studica_drivers`의 `.cpp` 17개 전부 컴파일 OK, 우리 `StudicaBackend.cpp`는 `-Wall -Wextra` 경고 없이 컴파일 OK.
2. `studica_drivers`를 빌드/설치한다(Studica ROS2 README 방식):
   ```bash
   cd allwpilib/studica_drivers && make && sudo make install
   ```
3. 예제 하나를 **root로** 실행한다(`pigpio`가 root를 요구한다):
   ```bash
   cd allwpilib/studica_drivers/examples/imu_example && make && sudo ./imu_example
   ```
   Titan이 연결돼 있으면 `titan_example`도 해 본다(바퀴를 띄운 상태에서).

- [x] `VMXPi.h`와 `libvmxpi_hal_cpp`가 있다 (`vmx-hal` deb, 로봇에서 확인)
- [ ] `studica_drivers` 빌드와 설치가 된다 (deb에 옛 버전이 이미 설치돼 있음. `install.sh --upgrade-drivers`로 새 복사본 설치는 미확인)
- [x] **HAL이 이 OS에서 하드웨어와 통신한다**: VMX 보드(모델 0x32, 하드웨어 rev 60, 펌웨어 3.0.436)와 통신 확립, navX 인식 (인스턴스 1개일 때). 예제로 값을 읽어 본 것은 아님

**기대**: 값이 읽히고 오류가 없다.
**실패하면**: 로그 전체를 기록한다. `pigpio` 관련 오류면 이 OS에서 VMX HAL이 안 도는 것이다. 이 경우 OS를 22.04로 돌려야 하고, 그러면 §5의 glibc 문제를 다른 방법(호환 sysroot, 온디바이스 빌드)으로 풀어야 한다.

## 2. 한 프로세스에서 `VMXPi`를 여러 개 만들면 문제가 되는가

Studica 클래스 생성자의 기본 인자가 `std::make_shared<VMXPi>(true, 50)`라서, 인자 없이 쓰면 장치마다 `VMXPi`가 생긴다.
`studica_drivers`를 수정할 수 없으므로 우리는 `wpilibvmx::SharedVMX()`를 넘기게 안내하고 있다. 이게 **실제로 필요한지** 확인한다.

작은 프로그램을 만든다(명령 미검증):

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

- [x] **결과(2026-10-10, 로봇 Ubuntu 26.04 / 커널 7.0): 한 프로세스에서 `VMXPi`를 둘 이상 만들면 깨진다.**
  - 인스턴스 **2개**: 둘 다 `IsOpen: 1`이지만 SPI 쓰기가 실패한다(`Timeout waiting for comm ready HIGH during SPI transmit. Likely CRC ERROR`, `Write CRC Mismatches: 135`, `Write Failures: 45`), 이후 `Aborted read from bank 0, address 4, length 108` 재시도가 끝없이 이어지고 **Segfault(종료 코드 139)**.
  - 인스턴스 **1개**(`vmx_n 1`): 정상. `Established communication with VMX board model 0x32, hardware rev 60, firmware version 3.0.436`, `Acquired navX-Sensor configuration`, 쓰기 19회/읽기 9172회에 CRC 불일치 0, 실패 0, 정상 종료.
  - **결론: `SharedVMX()`는 필수다.** Studica 클래스를 인자 없이 만들면(기본 인자가 새 `VMXPi`를 만든다) 하드웨어 통신이 망가진다.

**해석**:
- 둘 다 `1`이고 크래시 없음 → 기본 인자 함정은 문제가 아니다. `SharedVMX()` 안내는 권고로 낮춰도 된다.
- 두 번째가 `0`이거나 크래시 → 실제 문제다. `SharedVMX()`를 필수로 하고 문서에 강하게 적는다.
- (참고) 다른 **프로세스**가 동시에 VMX를 열 때의 동작은 우리 구조에서는 확인이 필요 없다(WPILib은 단일 프로세스, `robot_manager`는 VMX를 열지 않는다).

## 3. 핀 번호와 오프셋 (`robot_manager` 프로필에 채울 값)

사용자 기억: OS/커널 버전에 따라 핀 번호에서 뺄 오프셋이 있다. **버전과 값은 모른다(미검증).** 측정한다.

1. 사용할 채널마다 점퍼선으로 **출력 핀 → 입력 핀**을 연결하거나, 출력 핀에 LED/멀티미터를 연결한다.
2. `studica_driver::DIO`로 출력 핀을 토글하고, 어느 물리 핀이 반응하는지 기록한다(VMX-pi 핀맵 문서와 대조).
3. 같은 방법으로 입력, 아날로그, 엔코더용 핀을 확인한다.

- [ ] `VMXChannelIndex` ↔ 물리 핀 대응표를 만들었다 → `DESIGN.md` §4 표에 채운다
- [ ] 커널/OS 버전에 따라 번호가 어긋나는지 확인했다. 어긋난다면: 어느 버전부터 ____, 얼마 ____
  - **결과(2026-10-10, 커널 7.0.0-1020-raspi):** HAL을 열 때 `RPI GPIO Interrupt Enable:  PI_BAD_ISR_INIT.`가 3번 나온다. 원인 확인: **`/sys/class/gpio`가 없고 `# CONFIG_GPIO_SYSFS is not set`** 이다. pigpio의 GPIO 인터럽트는 sysfs GPIO를 쓰므로 **이 커널에서는 구조적으로 초기화할 수 없다.** 사용자가 기억하던 "핀 번호에서 뺄 오프셋"은 sysfs가 아직 켜져 있던 커널(6.6 부근)의 이야기로 보이며, 이 커널에서는 오프셋이 아니라 인터럽트 기능 부재가 문제다. **핀 읽기/쓰기에 오프셋이 필요한지는 아직 모른다**(아래 핀 토글 테스트로 확인).
  - 확인: `ls /sys/class/gpio; for c in /sys/class/gpio/gpiochip*; do echo $c $(cat $c/base) $(cat $c/ngpio) $(cat $c/label); done; grep GPIO_SYSFS /boot/config-$(uname -r)`
  - 영향: 인터럽트를 쓰는 기능(`studica_driver::DIO::EnableInterrupt` 등). 우리 `halsim_vmx`의 DIO/Analog/Encoder/IMU는 인터럽트를 쓰지 않는다(폴링).
  - **해결(Robot-Manager 레포의 `gpio_isr_shim/`):** `LD_PRELOAD`로 `gpioSetISRFunc*`를 GPIO 문자 장치 기반으로 대체했다. 로봇에서 `PI_BAD_ISR_INIT`이 사라졌고, HAL이 GPIO 13/6/12(모두 하강 에지)를 요청하며 GPIO 12에서 50 Hz(약 20 ms 간격)로 에지가 도착하는 것을 확인했다.
- [ ] 오프셋이 있으면 `robot_manager`의 `config.json` 프로필(`kernel_regex`)에 `HALSIMVMX_DIO_MAP` 등으로 적었다

**기대**: 대응표가 안정적이다. **오프셋이 없으면** 프로필 기능은 필요 없는 것이다(그러면 `robot_manager`는 systemd 유닛 하나로 대체할 수 있다).

## 4. Studica 플러그인 빌드 (`libhalsim_vmx_studica.so`)

이 PC에서는 스텁 헤더로 **문법 검사만** 했다. 실제 컴파일과 링크는 처음이다.

필요: `g++`(C++20, GCC 11 이상), `VMXPi.h`, `libvmxpi_hal_cpp`.

방법 A — 수동 빌드(명령 미검증, 가장 단순):
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
방법 B — CMake: `-DWPILIB_WITH_STUDICA=ON`로 `halsim_vmx_studica` 타깃을 빌드한다. allwpilib 전체 설정이 따라오므로 C++23이 되는 GCC(13 이상)가 필요하다.

- [x] 컴파일 성공 — **2026-10-10 로봇(Ubuntu 26.04, g++ 15.2.0)에서 `install.sh`로 확인됨**
- [x] 링크 성공 — `/opt/halsim_vmx/libhalsim_vmx_studica.so`(97KB), `ldd`에 `not found` 없음, `libvmxpi_hal_cpp.so`는 `/usr/local/lib/vmxpi`에서 해석됨
- [x] `nm -D`에 `wpilibvmx_CreateBackendV1`, `wpilibvmx_DestroyBackendV1`, `wpilibvmx::SharedVMX()`가 보인다
- (참고) `MakeBackendApi`의 람다가 약한 심볼로 같이 내보내진다. 해롭지 않지만 `-fvisibility=hidden`으로 숨길 수 있다.
- **아직 안 한 것: 이 `.so`를 실제로 로드해서 하드웨어를 움직이는 것(§6).**

## 5. WPILib 라이브러리가 이 OS에서 로드되는가 (glibc)

가장 큰 위험이다. trixie 툴체인으로 만든 `linuxarm64` 라이브러리는 glibc 2.41 기준이다.

1. PC에서 `linuxarm64` JNI 라이브러리를 받아 VMX로 복사한다. 예: 테스트 프로젝트에서 `vmxRelease wpi.java.deps.wpilibJniRelease(wpi.platforms.linuxarm64)`를 두고 `./gradlew` 의존성 해석 후 캐시된 zip에서 `*.so`를 꺼낸다.
2. VMX에서 확인한다:
   ```bash
   ldd /경로/libwpiHal.so | grep "not found"        # 비어 있어야 한다
   objdump -T /경로/libwpiHal.so | grep -o 'GLIBC_[0-9.]*' | sort -Vu | tail -1   # 필요한 최대 glibc 버전
   ldd --version | head -1                           # §0의 glibc와 비교
   ```
   (`libstdc++`도 같은 방식으로: `objdump -T ... | grep -o 'GLIBCXX_[0-9.]*' | sort -Vu | tail -1` 대 `strings /usr/lib/aarch64-linux-gnu/libstdc++.so.6 | grep GLIBCXX | sort -Vu | tail -1`)

- [ ] `not found` 없음
- [ ] 필요한 최대 GLIBC 버전 ____ ≤ 이 OS의 glibc ____
- [ ] 필요한 최대 GLIBCXX 버전 ____ ≤ 이 OS의 libstdc++ ____

**실패하면**: 이 OS에서는 공개된 `linuxarm64` 라이브러리를 쓸 수 없다. 선택지는 (a) VMX를 더 새로운 OS(Debian 13/Ubuntu 26.04)로 올린다 — 단 §1을 다시 통과해야 한다, (b) 호환 sysroot로 WPILib을 직접 빌드한다, (c) VMX에서 직접 빌드한다.

## 6. 확장이 로드되고 하드웨어를 움직이는가

최소 Java 로봇 프로젝트(또는 PC에서 `./gradlew deploy`한 결과)로 확인한다. 수동 실행 예(명령 미검증):

```bash
sudo HALSIM_EXTENSIONS="/경로/libhalsim_vmx.so" \
     HALSIMVMX_BACKEND="/경로/libhalsim_vmx_studica.so" \
     HALSIMVMX_DIO_MAP="..." HALSIMVMX_ANALOG_MAP="..." HALSIMVMX_IMU=1 \
     /usr/bin/java -Djava.library.path=/경로 -cp ... 로봇.메인클래스
```

로그에서 확인한다.
- [ ] `HALSim VMX: using backend /경로/libhalsim_vmx_studica.so`
- [ ] `HALSim VMX Extension Initialized`
- [ ] 백엔드를 **일부러 틀리게** 지정하면(`HALSIMVMX_BACKEND=/없는/경로`) 로봇 프로그램이 확장 초기화 오류를 내고, loopback으로 조용히 넘어가지 **않는다**

장치별로 확인한다(§3의 채널표 사용). 각각 **WPILib 표준 클래스**로 읽고 쓴다.

| 장치 | 방법 | 기대 | 결과 |
|---|---|---|---|
| DigitalOutput | `set(true/false)`를 번갈아, 멀티미터/LED로 확인 | 핀이 토글된다 | |
| DigitalInput | 점퍼로 HIGH/LOW를 줌 | `get()`이 따라간다 | |
| AnalogInput | 가변저항/알려진 전압을 줌 | `getVoltage()`가 실제 전압과 일치한다 | |
| Encoder | 손으로 돌림 | `get()`이 증가/감소, `reset()` 후 0, `setReverseDirection(true)`이면 부호가 반대 | |
| IMU(navX) | §7 참고 | | |

- [ ] DIO 출력/입력, AnalogInput, Encoder가 동작한다
- [ ] Encoder A/B를 `Encoder(a,b)`로 만들었을 때 같은 채널의 `DigitalInput`과 충돌 오류가 **없다**(엔코더가 채널을 가져간다는 가정을 확인)

## 7. IMU 부호와 Titan 안전

### IMU(navX) — `HALSIMVMX_IMU=1`

가정(미검증): navX의 yaw와 Z축 각속도는 **시계 방향이 양수**라서 부호를 뒤집어 WPILib(반시계 양수)으로 맞췄다.
roll/pitch/X·Y축 각속도/가속도는 **부호를 그대로** 넘겼다. 이 가정을 확인한다.

1. 보드를 **위에서 보아 반시계 방향으로** 천천히 돌린다 → WPILib `OnboardIMU`(또는 HAL)의 yaw가 **증가**해야 한다.
2. 보드를 앞쪽(X)으로 기울이고 다시 옆으로 기울인다 → roll/pitch 부호와 축이 기대와 맞는가.
3. 정지 상태에서 가속도 Z가 약 **+9.8 m/s²**인가(중력 방향 부호).

- [ ] yaw 부호  ✔/✘ ____ (✘이면 `HALSimVMX.cpp`의 `PollImu` 변환을 고친다)
- [ ] roll/pitch 축과 부호 ____
- [ ] 가속도 Z 부호와 크기 ____
- [ ] 보정 중/미연결일 때 값이 갱신되지 않는다(`IsCalibrating`/`IsConnected`)

### Titan 안전 — `TitanEnableGuard`

사용자의 MockDS가 **`setDsAttached(true)`와 `setEnabled(...)`를 둘 다** 해야 한다(sim HAL은 DS가 attached일 때만 컨트롤 워드를 채운다).
Titan은 `Enable(true)` 전에는 명령을 무시하고, 200 ms 동안 CAN 메시지가 없으면 스스로 멈춘다(`titan.hpp`).

`studica_driver::Titan titan(id, freq, distPerTick, wpilibvmx::SharedVMX());` 다음 `TitanEnableGuard<studica_driver::Titan> guard{titan};` `guard.Start();`로 만든다.
(사용자 코드는 enabled 동안 150 ms 이내로 `SetSpeed`를 계속 보내야 한다.)

- [ ] disabled에서 `SetSpeed`를 보내도 **모터가 안 돈다**
- [ ] enabled로 바꾸면 돈다
- [ ] 다시 disabled로 바꾸면 **즉시** 멈춘다
- [ ] E-stop이면 멈춘다
- [ ] 로봇 프로그램을 `kill -9`로 죽이면 모터가 약 200 ms 안에 멈춘다(하드웨어 안전망)
- [ ] 로봇 프로그램을 `kill -TERM`으로 종료하면 `Enable(false)`가 먼저 가고 멈춘다

## 8. `robot_manager`와 deploy end-to-end

### `robot_manager` (레포: `W4Studica/Robot-Manager`, README의 설치 절차)

- [x] 설치 후 `systemctl is-active robot_manager`가 `active` (로봇에서 확인됨. 단, `robotCommand`는 아직 없어서 프로그램을 돌려 본 건 아님)
- [x] `./install.sh --sudoers`가 적용되고 `sudo -n systemctl enable robot_manager`가 비밀번호 없이 된다 (로봇에서 확인됨)
- [ ] `/home/<user>/robotCommand`가 없으면 로그에 한 번만 대기 메시지가 나온다
- [ ] 간단한 `robotCommand`(`sleep 600` 한 줄)를 두면 실행된다. `journalctl -u robot_manager -f`
- [ ] 프로세스를 죽이면 다시 실행된다(크래시 반복 시 간격이 1→10초로 늘어난다)
- [ ] `sudo systemctl stop robot_manager`가 로봇 프로세스 트리까지 정리한다(`pgrep -af robotCommand`/자식 확인)
- [ ] 로봇 프로그램이 **root로** 실행된다(`ps -o user= -p <pid>`)
- [ ] 플랫폼 프로필: `kernel_regex`가 `uname -r`과 일치하면 환경변수가 로봇 프로세스에 들어간다(`cat /proc/<pid>/environ | tr '\0' '\n' | grep HALSIMVMX`)

### GradleRIO `./gradlew deploy` (PC에서)

`GradleRIO/VMX.md`의 예시 `build.gradle`을 쓴다. `username`/`password`/주소는 직접 넣는다(기본값 없음).

- [ ] SSH 접속과 `sudo systemctl`/`sudo ldconfig`가 **비밀번호 없이** 된다(sudoers)
- [ ] `./gradlew deploy`가 끝까지 성공한다
- [ ] VMX에 `classpath/`, `third-party/lib/`, `robotCommand`, `robotCommand.args`가 올라간다
- [ ] 서비스가 stop → 파일 업로드 → start 순서로 재시작되고 새 프로그램이 돈다
- [ ] `HALSIM_EXTENSIONS`/`HALSIMVMX_*` 환경변수가 `robotCommand`에 들어 있다
- [ ] (C++) `WPILibNativeArtifact`로 C++ 로봇 프로그램도 올라간다 — **지금까지 실제 빌드로 쓴 적 없음**

## 결과 기록표

| 항목 | 결과 | 비고 |
|---|---|---|
| §0 OS / 커널 / Pi 모델 | | |
| §0 glibc / g++ / java | | |
| §1 Studica 스택 동작 | ✔ / ✘ | |
| §2 `VMXPi` 두 개 | 첫 ____ / 둘째 ____ | |
| §3 핀 오프셋 | 있음(버전 ____, 값 ____) / 없음 | |
| §4 플러그인 빌드 | ✔ / ✘ | |
| §5 glibc 호환 | ✔ / ✘ | |
| §6 DIO/Analog/Encoder | | |
| §7 IMU 부호 | | |
| §7 Titan 안전 | | |
| §8 `robot_manager` | | |
| §8 deploy | | |

## 결과에 따라 고칠 문서

- §1 실패 → `CLAUDE.md`의 OS 결정, `GradleRIO/VMX.md`의 glibc 위험 항목
- §2 → `DESIGN.md` §3 "아직 모르는 것"과 `SharedVMX.hpp` 설명(권고 vs 필수)
- §3 → `DESIGN.md` §4 채널표, `robot_manager/config.example.json`/README의 프로필
- §4 → `StudicaBackend.cpp`, 그리고 이 문서의 빌드 명령(미검증 표시 제거)
- §5 → `GradleRIO/VMX.md` Known risks
- §7 IMU → `HALSimVMX.cpp`의 `PollImu`와 `DESIGN.md` §1 IMU 행
- 모든 "미검증"/"미컴파일" 표기를 확인한 항목에서 지운다
