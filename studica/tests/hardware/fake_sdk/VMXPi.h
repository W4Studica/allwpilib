// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// Test double for the SDK boundary. Production device .cpp files are compiled
// unchanged against this header; this is not a model of electrical behavior.
#include <array>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <vector>

using VMXErrorCode = int;
using VMXChannelIndex = uint8_t;
using VMXResourceHandle = uint32_t;
constexpr VMXChannelIndex INVALID_VMX_CHANNEL_INDEX = 255;
inline const char* GetVMXErrorString(VMXErrorCode) {
  return "injected SDK failure";
}
enum class VMXChannelCapability {
  DigitalInput,
  DigitalOutput,
  AccumulatorInput,
  EncoderAInput,
  EncoderBInput,
  PWMGeneratorOutput,
  I2C_SDA,
  I2C_SCL,
  PWMCaptureInput,
  InterruptInput
};
struct VMXChannelInfo {
  VMXChannelIndex channel;
  VMXChannelCapability capability;
  VMXChannelInfo(VMXChannelIndex ch, VMXChannelCapability cap)
      : channel(ch), capability(cap) {}
};
struct ConfigBase {
  virtual ~ConfigBase() = default;
};
struct DIOConfig : ConfigBase {
  enum class InputMode { PULLUP };
  void SetInputMode(InputMode) {}
};
struct AccumulatorConfig : ConfigBase {
  void SetNumAverageBits(int) {}
};
struct EncoderConfig : ConfigBase {
  enum class EncoderEdge { x4 };
  explicit EncoderConfig(EncoderEdge) {}
};
struct PWMGeneratorConfig : ConfigBase {
  explicit PWMGeneratorConfig(uint32_t) {}
  void SetMaxDutyCycleValue(uint16_t) {}
};
struct I2CConfig : ConfigBase {};
struct PWMCaptureConfig : ConfigBase {
  enum Edge { x2 };
  PWMCaptureConfig(int, Edge) {}
};
enum class InterruptEdgeType { RISING_EDGE_INTERRUPT, FALLING_EDGE_INTERRUPT };
using InterruptCallback = void (*)(uint32_t, InterruptEdgeType, void*,
                                   uint64_t);
struct InterruptConfig : ConfigBase {
  enum Edge { BOTH };
  InterruptCallback callback;
  void* param;
  InterruptConfig(Edge, InterruptCallback cb, void* p)
      : callback(cb), param(p) {}
};
struct FakeIO {
  struct Resource {
    std::vector<int> channels;
    InterruptCallback callback = nullptr;
    void* param = nullptr;
  };
  std::map<VMXResourceHandle, Resource> resources;
  VMXResourceHandle next = 0;  // A zero handle is deliberately valid.
  int allocated = 0, released = 0, duplicateReleases = 0, writes = 0;
  bool failActivate = false, throwActivate = false, failRead = false,
       failWrite = false;
  int unsupported = -1;
  int32_t encoderCount = 0;
  float voltage = 2.5f;
  bool digital = false;
  uint16_t pwmDuty = 0;
  uint32_t period = 1000, high = 250;
  Resource lastInterrupt;
  std::vector<int> releaseOrder;
  bool ChannelSupportsCapability(VMXChannelIndex channel,
                                 VMXChannelCapability) {
    return channel < 64 && channel != unsupported;
  }
  VMXChannelIndex GetSoleChannelIndex(VMXChannelCapability cap) {
    return cap == VMXChannelCapability::I2C_SDA ? 32 : 33;
  }
  bool Activate(std::vector<int> channels, const ConfigBase* config,
                VMXResourceHandle& out) {
    if (throwActivate)
      throw std::runtime_error("injected activation exception");
    if (failActivate)
      return false;
    for (const auto& [handle, resource] : resources) {
      for (int used : resource.channels)
        for (int ch : channels)
          if (used == ch)
            return false;
    }
    Resource resource{std::move(channels)};
    if (auto* interrupt = dynamic_cast<const InterruptConfig*>(config)) {
      resource.callback = interrupt->callback;
      resource.param = interrupt->param;
      lastInterrupt = resource;
    }
    out = next++;
    resources.emplace(out, std::move(resource));
    ++allocated;
    return true;
  }
  bool ActivateSinglechannelResource(VMXChannelInfo info,
                                     const ConfigBase* config,
                                     VMXResourceHandle& out, VMXErrorCode*) {
    return Activate({info.channel}, config, out);
  }
  bool ActivateDualchannelResource(VMXChannelInfo a, VMXChannelInfo b,
                                   const ConfigBase* config,
                                   VMXResourceHandle& out, VMXErrorCode*) {
    return Activate({a.channel, b.channel}, config, out);
  }
  bool DeallocateResource(VMXResourceHandle handle, VMXErrorCode*) {
    if (!resources.contains(handle)) {
      ++duplicateReleases;
      return false;
    }
    releaseOrder.insert(releaseOrder.end(),
                        resources.at(handle).channels.begin(),
                        resources.at(handle).channels.end());
    resources.erase(handle);
    ++released;
    return true;
  }
  bool DIO_Set(VMXResourceHandle handle, bool value, VMXErrorCode*) {
    ++writes;
    if (failWrite || !resources.contains(handle))
      return false;
    digital = value;
    return true;
  }
  bool DIO_Get(VMXResourceHandle handle, bool& value, VMXErrorCode*) {
    if (failRead || !resources.contains(handle))
      return false;
    value = digital;
    return true;
  }
  bool Accumulator_GetAverageVoltage(VMXResourceHandle handle, float& value,
                                     VMXErrorCode*) {
    if (failRead || !resources.contains(handle))
      return false;
    value = voltage;
    return true;
  }
  bool Encoder_GetCount(VMXResourceHandle handle, int32_t& value,
                        VMXErrorCode*) {
    if (failRead || !resources.contains(handle))
      return false;
    value = encoderCount;
    return true;
  }
  bool PWMGenerator_SetDutyCycle(VMXResourceHandle handle, VMXChannelIndex,
                                 uint16_t duty, VMXErrorCode*) {
    ++writes;
    if (failWrite || !resources.contains(handle))
      return false;
    pwmDuty = duty;
    return true;
  }
  bool PWMCapture_GetCount(VMXResourceHandle handle, uint32_t& p, uint32_t& h,
                           VMXErrorCode*) {
    if (failRead || !resources.contains(handle))
      return false;
    p = period;
    h = high;
    return true;
  }
  bool I2C_Transaction(VMXResourceHandle handle, uint8_t, uint8_t* tx,
                       uint16_t txSize, uint8_t* rx, uint16_t rxSize,
                       VMXErrorCode*) {
    if (failRead || !resources.contains(handle))
      return false;
    if (txSize)
      tx[0] = 0xff;  // Exercise the SDK's mutable-buffer contract.
    for (int i = 0; i < rxSize; ++i)
      rx[i] = 0xab;
    return true;
  }
  bool Interrupt_SetEnabled(VMXResourceHandle handle, bool, VMXErrorCode*) {
    return resources.contains(handle);
  }
  void FireLateInterrupt() const {
    if (lastInterrupt.callback)
      lastInterrupt.callback(0, InterruptEdgeType::RISING_EDGE_INTERRUPT,
                             lastInterrupt.param, 123);
  }
};
struct FakePower {
  bool fail = false;
  bool GetSystemVoltage(float& value, VMXErrorCode*) {
    if (fail)
      return false;
    value = 12;
    return true;
  }
};
struct VMXTime {
  enum class DaylightSavingsAdjustment { None = 0 };
  bool fail = false;
  std::vector<uint8_t> dates;
  unsigned dateReads = 0;
  bool GetRTCDate(uint8_t& weekday, uint8_t& day, uint8_t& month, uint8_t& year,
                  VMXErrorCode*) {
    if (fail)
      return false;
    weekday = 1;
    month = 10;
    year = 26;
    day = dates.empty()
              ? 28
              : dates[dateReads < dates.size() ? dateReads : dates.size() - 1];
    ++dateReads;
    return true;
  }
  bool GetRTCTime(uint8_t& hour, uint8_t& minute, uint8_t& second, uint32_t& ms,
                  VMXErrorCode*) {
    if (fail)
      return false;
    hour = 12;
    minute = 34;
    second = 56;
    ms = 789;
    return true;
  }
  bool GetRTCDaylightSavingsAdjustment(DaylightSavingsAdjustment& value,
                                       VMXErrorCode*) {
    if (fail)
      return false;
    value = DaylightSavingsAdjustment::None;
    return true;
  }
};
struct VMXPi {
  bool open = true, connected = true, calibrating = false;
  FakeIO io;
  FakePower power;
  VMXTime time;
  VMXPi(bool, uint8_t) {}
  bool IsOpen() const { return open; }
};
