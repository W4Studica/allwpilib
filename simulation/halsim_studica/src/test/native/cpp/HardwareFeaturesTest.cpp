// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "wpi/hal/DutyCycle.h"
#include "wpi/hal/I2C.h"
#include "wpi/hal/IMU.h"
#include "wpi/hal/simulation/DIOData.h"
#include "wpi/hal/simulation/DigitalEdgeData.h"
#include "wpi/hal/simulation/DutyCycleData.h"
#include "wpi/hal/simulation/I2CData.h"
#include "wpi/hal/simulation/PWMData.h"
#include "wpi/halsim/studica/HALSimStudica.hpp"
#include "wpi/studica/detail/backend/LoopbackBackend.hpp"

using namespace wpi::studica;
namespace {
struct Fixture {
  LoopbackBackend* hw;
  std::unique_ptr<HALSimStudica> sim;
  Fixture(bool imu = false, std::unique_ptr<LoopbackBackend> backend =
                                std::make_unique<LoopbackBackend>()) {
    HALSIM_ResetI2CData(1);
    HALSIM_ResetPWMData(3);
    HALSIM_ResetDIOData(2);
    HALSIM_ResetDutyCycleData(2);
    hw = backend.get();
    Config config;
    config.imu = imu;
    config.i2cPort = 1;
    config.maps.dio = ChannelMap::Parse("2:20");
    config.maps.pwm = ChannelMap::Parse("3:15");
    sim = std::make_unique<HALSimStudica>(std::move(backend), config);
    REQUIRE(sim->Start(false));
  }
  ~Fixture() {
    sim.reset();
    HALSIM_ResetDIOData(2);
    HALSIM_ResetDutyCycleData(2);
    HALSIM_ResetI2CData(1);
    HALSIM_ResetPWMData(3);
  }
};
}  // namespace

TEST_CASE("I2C forwards one combined transaction with its address and error",
          "[studica][i2c]") {
  Fixture f;
  int calls = 0;
  f.hw->SetI2cHandler(
      [&](int address, const uint8_t* tx, int txSize, uint8_t* rx, int rxSize) {
        ++calls;
        CHECK(address == 0x50);
        CHECK(txSize == 1);
        CHECK(tx[0] == 7);
        CHECK(rxSize == 2);
        rx[0] = 0xab;
        rx[1] = 0xcd;
        return true;
      });
  int32_t status = 0;
  HAL_InitializeI2C(HAL_I2C_PORT_1, &status);
  REQUIRE(status == 0);
  uint8_t tx = 7, rx[2]{};
  // No polling pass is needed before the first transaction.
  REQUIRE(HAL_TransactionI2C(HAL_I2C_PORT_1, 0x50, &tx, 1, rx, 2) == 0);
  REQUIRE(calls == 1);
  REQUIRE(rx[0] == 0xab);
  REQUIRE(rx[1] == 0xcd);
  f.hw->SetI2cHandler(
      [](int, const uint8_t*, int, uint8_t*, int) { return false; });
  REQUIRE(HAL_WriteI2C(HAL_I2C_PORT_1, 0x50, &tx, 1) == -1);
  REQUIRE(HAL_ReadI2C(HAL_I2C_PORT_1, 0x50, rx, 2) == -1);
  REQUIRE(HAL_ReadI2C(HAL_I2C_PORT_1, 0x180, rx, 2) == -1);
  REQUIRE(HAL_ReadI2C(HAL_I2C_PORT_1, 0x50, nullptr, 1) == -1);
  HAL_CloseI2C(HAL_I2C_PORT_1);
  REQUIRE(HAL_ReadI2C(HAL_I2C_PORT_1, 0x50, rx, 2) == -1);
  f.sim->Poll();
  REQUIRE(f.hw->InitI2c());  // The closed bus was released.
  f.hw->ReleaseI2c();
}

TEST_CASE("I2C write-only and read-only preserve their buffers",
          "[studica][i2c]") {
  Fixture f;
  int writes = 0, reads = 0;
  f.hw->SetI2cHandler(
      [&](int address, const uint8_t* tx, int txSize, uint8_t* rx, int rxSize) {
        CHECK(address == 0x27);
        if (txSize) {
          ++writes;
          CHECK(txSize == 1);
          CHECK(tx[0] == 3);
          CHECK(rxSize == 0);
        } else {
          ++reads;
          CHECK(rxSize == 1);
          rx[0] = 42;
        }
        return true;
      });
  int32_t status = 0;
  HAL_InitializeI2C(HAL_I2C_PORT_1, &status);
  uint8_t tx = 3, rx = 0;
  REQUIRE(HAL_WriteI2C(HAL_I2C_PORT_1, 0x27, &tx, 1) == 0);
  REQUIRE(HAL_ReadI2C(HAL_I2C_PORT_1, 0x27, &rx, 1) == 0);
  REQUIRE(writes == 1);
  REQUIRE(reads == 1);
  REQUIRE(rx == 42);
  HAL_CloseI2C(HAL_I2C_PORT_1);
}

TEST_CASE("I2C provider ownership and cancellation wait for a transaction",
          "[studica][i2c]") {
  using namespace std::chrono_literals;
  HALSIM_ResetI2CData(0);
  struct Context {
    std::promise<void> entered;
    std::shared_future<void> release;
  } context;
  std::promise<void> release;
  context.release = release.get_future().share();
  const auto callback = [](void* param, int32_t, const uint8_t*, int32_t,
                           uint8_t*, int32_t) -> int32_t {
    auto& c = *static_cast<Context*>(param);
    c.entered.set_value();
    c.release.wait();
    return -1;
  };
  int uid = HALSIM_RegisterI2CTransactionCallback(0, callback, &context);
  REQUIRE(uid > 0);
  REQUIRE(HALSIM_RegisterI2CTransactionCallback(0, callback, &context) == -1);
  int32_t status = 0;
  HAL_InitializeI2C(HAL_I2C_PORT_0, &status);
  auto transfer = std::async(std::launch::async, [] {
    return HAL_TransactionI2C(HAL_I2C_PORT_0, 0x50, nullptr, 0, nullptr, 0);
  });
  context.entered.get_future().wait();
  auto cancel = std::async(std::launch::async, [uid] {
    HALSIM_CancelI2CTransactionCallback(0, uid);
  });
  const auto waiting = cancel.wait_for(20ms);
  release.set_value();
  REQUIRE(transfer.get() == -1);
  cancel.get();
  REQUIRE(waiting == std::future_status::timeout);
  // Stale cancellation must not remove a replacement provider.
  int next = HALSIM_RegisterI2CTransactionCallback(0, callback, &context);
  REQUIRE(next > uid);
  HALSIM_CancelI2CTransactionCallback(0, uid);
  REQUIRE(HALSIM_RegisterI2CTransactionCallback(0, callback, &context) == -1);
  HALSIM_CancelI2CTransactionCallback(0, next);
  HAL_CloseI2C(HAL_I2C_PORT_0);
}

TEST_CASE("Duty-cycle data reaches HAL and releases its pin",
          "[studica][dutycycle]") {
  Fixture f;
  int32_t status = 0;
  auto handle = HAL_InitializeDutyCycle(2, "test", &status);
  REQUIRE(status == 0);
  HALSIM_SetDIOInitialized(
      2, true);  // An existing DigitalInput must yield to capture.
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->IsClaimed(20));
  REQUIRE_FALSE(f.hw->InitDigital(20, true));
  f.hw->DriveDutyCycle(20, 1000, 0.25);
  f.sim->Poll();
  REQUIRE(HAL_GetDutyCycleFrequency(handle, &status) == 1000);
  REQUIRE(HAL_GetDutyCycleOutput(handle, &status) == Catch::Approx(0.25));
  REQUIRE(HAL_GetDutyCycleHighTime(handle, &status) == 250000);
  f.hw->DriveDutyCycle(20, 0, 0);
  f.sim->Poll();
  REQUIRE(HAL_GetDutyCycleFrequency(handle, &status) == 0);
  HAL_FreeDutyCycle(handle);
  f.sim->Poll();
  f.sim->Poll();  // DIO reacquires the released pin on the next pass.
  REQUIRE(f.hw->IsClaimed(20));
}

TEST_CASE("Digital edge events retain timestamps and unsubscribe cleanly",
          "[studica][interrupt]") {
  Fixture f;
  std::vector<InterruptEvent> events;
  auto callback = [](const char*, void* param, HAL_Bool rising, uint64_t time,
                     uint32_t dropped) {
    static_cast<std::vector<InterruptEvent>*>(param)->push_back(
        {rising != 0, time, dropped});
  };
  int uid = HALSIM_RegisterDIOEdgeCallback(2, callback, &events);
  HALSIM_SetDIOInitialized(2, true);
  HALSIM_SetDIOIsInput(2, true);
  HALSIM_SetDIOEdgeCaptureEnabled(2, true);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->InitDigital(20, true));
  f.hw->DriveInterrupt(20, true, 100);
  f.hw->DriveInterrupt(20, false, 101);
  f.sim->Poll();
  REQUIRE(events.size() == 2);
  REQUIRE(events[0].rising);
  REQUIRE(events[0].timestampMicros == 100);
  REQUIRE_FALSE(events[1].rising);
  REQUIRE(events[1].timestampMicros == 101);
  REQUIRE_FALSE(HALSIM_GetDIOValue(2));
  HALSIM_CancelDIOEdgeCallback(2, uid);
  f.hw->DriveInterrupt(20, true, 102);
  f.sim->Poll();
  REQUIRE(events.size() == 2);
  HALSIM_SetDIOEdgeCaptureEnabled(2, false);
  f.sim->Poll();
  REQUIRE(f.hw->IsClaimed(20));
}

TEST_CASE("Interrupt queue reports overflow without unbounded growth",
          "[studica][interrupt]") {
  detail::InterruptQueue queue;
  for (uint64_t i = 0; i < 300; ++i) {
    queue.Push({true, i, 0});
  }
  InterruptEvent event;
  REQUIRE(queue.Pop(&event));
  REQUIRE(event.timestampMicros == 44);
  REQUIRE(event.dropped == 44);
  for (uint64_t i = 45; i < 300; ++i) {
    REQUIRE(queue.Pop(&event));
    REQUIRE(event.timestampMicros == i);
    REQUIRE(event.dropped == 0);
  }
  REQUIRE_FALSE(queue.Pop(&event));
}

TEST_CASE("Corrected IMU angles publish a normalized quaternion",
          "[studica][imu]") {
  Fixture f{true};
  ImuSample sample;
  sample.yawDeg = 90;
  f.hw->DriveImu(sample);
  f.sim->Poll();
  HAL_Quaternion q{};
  int32_t status = 0;
  HAL_GetIMUQuaternion(&q, &status);
  REQUIRE(status == 0);
  int64_t timestamp = -1;
  HAL_GetIMUYawFlat(&timestamp);
  REQUIRE(timestamp == q.timestamp);
  REQUIRE(q.w == Catch::Approx(std::sqrt(0.5)));
  REQUIRE(q.x == Catch::Approx(0));
  REQUIRE(q.y == Catch::Approx(0));
  REQUIRE(q.z == Catch::Approx(-std::sqrt(0.5)));
  REQUIRE(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z == Catch::Approx(1));
}

TEST_CASE("PWM capture conversion handles a disconnected signal",
          "[studica][dutycycle]") {
  auto reading = ConvertDutyCycle(20000, 1500);
  REQUIRE(reading.frequencyHz == Catch::Approx(50));
  REQUIRE(reading.output == Catch::Approx(0.075));
  REQUIRE(ConvertDutyCycle(0, 100).frequencyHz == 0);
  REQUIRE(ConvertDutyCycle(100, 200).output == 1);
}

TEST_CASE("Legacy I2C observers remain active only without a hardware provider",
          "[studica][i2c]") {
  HALSIM_ResetI2CData(0);
  int writes = 0, reads = 0;
  auto write = [](const char*, void* p, const uint8_t*, unsigned int) {
    ++*static_cast<int*>(p);
  };
  auto read = [](const char*, void* p, uint8_t*, unsigned int) {
    ++*static_cast<int*>(p);
  };
  int w = HALSIM_RegisterI2CWriteCallback(0, write, &writes);
  int r = HALSIM_RegisterI2CReadCallback(0, read, &reads);
  int32_t status = 0;
  HAL_InitializeI2C(HAL_I2C_PORT_0, &status);
  REQUIRE(HAL_TransactionI2C(HAL_I2C_PORT_0, 0x50, nullptr, 0, nullptr, 0) ==
          0);
  REQUIRE(writes == 1);
  REQUIRE(reads == 1);
  auto fail = [](void*, int32_t, const uint8_t*, int32_t, uint8_t*,
                 int32_t) -> int32_t { return -1; };
  int provider = HALSIM_RegisterI2CTransactionCallback(0, fail, nullptr);
  REQUIRE(provider > 0);
  REQUIRE(HAL_TransactionI2C(HAL_I2C_PORT_0, 0x50, nullptr, 0, nullptr, 0) ==
          -1);
  REQUIRE(writes == 1);
  REQUIRE(reads == 1);
  HALSIM_CancelI2CTransactionCallback(0, provider);
  REQUIRE(HAL_WriteI2C(HAL_I2C_PORT_0, 0x50, nullptr, 0) == 0);
  REQUIRE(writes == 2);
  REQUIRE(reads == 1);
  HALSIM_CancelI2CWriteCallback(0, w);
  HALSIM_CancelI2CReadCallback(0, r);
  HAL_CloseI2C(HAL_I2C_PORT_0);
}

TEST_CASE("Stopping the bridge releases capture and I2C resources",
          "[studica][lifecycle]") {
  Fixture f;
  HALSIM_SetDutyCycleInitialized(2, true);
  int32_t status = 0;
  HAL_InitializeI2C(HAL_I2C_PORT_1, &status);
  // No emulated device: the attempted transfer fails, but claims the bus.
  REQUIRE(HAL_TransactionI2C(HAL_I2C_PORT_1, 0x50, nullptr, 0, nullptr, 0) ==
          -1);
  f.sim->Poll();
  REQUIRE_FALSE(f.hw->InitI2c());
  REQUIRE_FALSE(f.hw->InitDutyCycle(20));
  f.sim->Stop();
  REQUIRE(f.hw->InitI2c());
  REQUIRE(f.hw->InitDutyCycle(20));
  f.hw->ReleaseI2c();
  f.hw->ReleaseDutyCycle(20);
  HAL_CloseI2C(HAL_I2C_PORT_1);
}

TEST_CASE("Edge capture never takes over a digital output",
          "[studica][interrupt]") {
  Fixture f;
  HALSIM_SetDIOInitialized(2, true);
  HALSIM_SetDIOIsInput(2, false);
  HALSIM_SetDIOEdgeCaptureEnabled(2, true);
  f.sim->Poll();
  REQUIRE(f.hw->IsClaimed(20));
  REQUIRE_FALSE(f.hw->IsInput(20));
  REQUIRE_FALSE(f.hw->InitInterrupt(20));
}

#ifndef _WIN32
TEST_CASE("Default PC simulation leaves legacy I2C callbacks in control",
          "[studica][i2c]") {
  struct UnsetEnv {
    const char* name;
    std::optional<std::string> old;
    explicit UnsetEnv(const char* n) : name(n) {
      if (const char* value = std::getenv(name)) {
        old = value;
      }
      unsetenv(name);
    }
    ~UnsetEnv() {
      if (old) {
        setenv(name, old->c_str(), 1);
      } else {
        unsetenv(name);
      }
    }
  } backendEnv{"HALSIMSTUDICA_BACKEND"}, portEnv{"HALSIMSTUDICA_I2C_PORT"};
  HALSIM_ResetI2CData(1);
  HALSimStudica sim{std::make_unique<LoopbackBackend>(), Config::FromEnv()};
  REQUIRE(sim.Start(false));
  int calls = 0;
  auto callback = [](const char*, void* param, const uint8_t*, unsigned int) {
    ++*static_cast<int*>(param);
  };
  int uid = HALSIM_RegisterI2CWriteCallback(1, callback, &calls);
  int32_t status = 0;
  HAL_InitializeI2C(HAL_I2C_PORT_1, &status);
  REQUIRE(HAL_WriteI2C(HAL_I2C_PORT_1, 0x50, nullptr, 0) == 0);
  REQUIRE(calls == 1);
  HALSIM_CancelI2CWriteCallback(1, uid);
  HAL_CloseI2C(HAL_I2C_PORT_1);
}
#endif

TEST_CASE("An occupied I2C provider prevents a second bridge from starting",
          "[studica][i2c]") {
  Fixture f;
  Config config;
  config.i2cPort = 1;
  HALSimStudica second{std::make_unique<LoopbackBackend>(), config};
  REQUIRE_FALSE(second.Start(false));
  second.Stop();
  f.hw->SetI2cHandler(
      [](int, const uint8_t*, int, uint8_t*, int) { return true; });
  int32_t status = 0;
  HAL_InitializeI2C(HAL_I2C_PORT_1, &status);
  REQUIRE(HAL_TransactionI2C(HAL_I2C_PORT_1, 0x50, nullptr, 0, nullptr, 0) ==
          0);
  HAL_CloseI2C(HAL_I2C_PORT_1);
}

TEST_CASE("Blocked I2C does not retain the SIM state lock", "[studica][i2c]") {
  using namespace std::chrono_literals;
  Fixture f;
  HALSIM_SetDIOInitialized(2, true);
  HALSIM_SetDIOIsInput(2, false);
  f.sim->Poll();
  std::promise<void> entered, release;
  auto proceed = release.get_future().share();
  f.hw->SetI2cHandler([&](int, const uint8_t*, int, uint8_t*, int) {
    entered.set_value();
    proceed.wait();
    return true;
  });
  int32_t status = 0;
  HAL_InitializeI2C(HAL_I2C_PORT_1, &status);
  auto transaction = std::async(std::launch::async, [&] {
    uint8_t byte = 0;
    return HAL_WriteI2C(HAL_I2C_PORT_1, 0x50, &byte, 1);
  });
  entered.get_future().wait();
  auto output =
      std::async(std::launch::async, [&] { HALSIM_SetDIOValue(2, true); });
  const bool outputReady = output.wait_for(500ms) == std::future_status::ready;
  auto poll = std::async(std::launch::async, [&] { f.sim->Poll(); });
  const bool pollReady = poll.wait_for(500ms) == std::future_status::ready;
  release.set_value();
  output.get();
  poll.get();
  REQUIRE(transaction.get() == 0);
  REQUIRE(outputReady);
  REQUIRE(pollReady);
  REQUIRE(f.hw->GetDigital(20));
  HAL_CloseI2C(HAL_I2C_PORT_1);
}

TEST_CASE("Failed digital reads preserve the last HAL input",
          "[studica][digital]") {
  struct Backend : LoopbackBackend {
    bool fail = true;
    bool TryGetDigital(int channel, bool* value) override {
      return !fail && LoopbackBackend::TryGetDigital(channel, value);
    }
  };
  auto backend = std::make_unique<Backend>();
  auto* hw = backend.get();
  Fixture f{false, std::move(backend)};
  HALSIM_SetDIOInitialized(2, true);
  HALSIM_SetDIOIsInput(2, true);
  HALSIM_SetDIOValue(2, true);
  f.sim->Poll();
  REQUIRE(HALSIM_GetDIOValue(2));
  hw->fail = false;
  f.sim->Poll();
  REQUIRE_FALSE(HALSIM_GetDIOValue(2));
}
TEST_CASE("Failed output writes retry without another user change",
          "[studica][outputs]") {
  struct Backend : LoopbackBackend {
    bool fail = true;
    bool TrySetDigital(int channel, bool value) override {
      return !fail && LoopbackBackend::TrySetDigital(channel, value);
    }
    bool TrySetPwmPulse(int channel, int value) override {
      return !fail && LoopbackBackend::TrySetPwmPulse(channel, value);
    }
  };
  auto backend = std::make_unique<Backend>();
  auto* hw = backend.get();
  Fixture f{false, std::move(backend)};
  HALSIM_SetDIOInitialized(2, true);
  HALSIM_SetDIOIsInput(2, false);
  HALSIM_SetDIOValue(2, true);
  HALSIM_SetPWMInitialized(3, true);
  HALSIM_SetPWMPulseMicrosecond(3, 1500);
  f.sim->Poll();
  REQUIRE_FALSE(hw->GetDigital(20));
  REQUIRE(hw->PwmPulse(15) == 0);
  hw->fail = false;
  f.sim->Poll();
  REQUIRE(hw->GetDigital(20));
  REQUIRE(hw->PwmPulse(15) == 1500);
  hw->fail = true;
  HALSIM_SetDIOValue(2, false);
  REQUIRE(hw->GetDigital(20));
  hw->fail = false;
  f.sim->Poll();
  REQUIRE_FALSE(hw->GetDigital(20));
}
