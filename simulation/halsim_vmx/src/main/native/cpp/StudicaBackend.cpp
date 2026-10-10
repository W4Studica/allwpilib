// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Backend for VMX-pi (WPILIB_WITH_STUDICA=ON). Calls the Studica drivers
// directly. NOT YET COMPILED OR RUN: vmxpi_hal_cpp is only available on a VMX-pi.

#include <map>
#include <memory>

#include "VMXPi.h"
#include "dio.hpp"
#include "wpi/halsim/vmx/VmxBackend.hpp"

namespace wpilibvmx {
namespace {

class StudicaBackend : public VmxBackend {
 public:
  StudicaBackend() : m_vmx{std::make_shared<VMXPi>(true, 50)} {}

  bool InitDigital(int vmxChannel, bool isInput) override {
    if (!m_vmx || !m_vmx->IsOpen() || m_dio.contains(vmxChannel)) {
      return false;
    }
    auto dio = std::make_unique<studica_driver::DIO>(
        static_cast<VMXChannelIndex>(vmxChannel),
        isInput ? studica_driver::PinMode::INPUT
                : studica_driver::PinMode::OUTPUT,
        m_vmx);
    if (!dio->IsInitialized()) {
      return false;
    }
    m_dio.emplace(vmxChannel, std::move(dio));
    return true;
  }

  void ReleaseDigital(int vmxChannel) override { m_dio.erase(vmxChannel); }

  void SetDigital(int vmxChannel, bool value) override {
    if (auto it = m_dio.find(vmxChannel); it != m_dio.end()) {
      it->second->Set(value);
    }
  }

  bool GetDigital(int vmxChannel) override {
    if (auto it = m_dio.find(vmxChannel); it != m_dio.end()) {
      return it->second->Get();
    }
    return false;
  }

 private:
  std::shared_ptr<VMXPi> m_vmx;
  std::map<int, std::unique_ptr<studica_driver::DIO>> m_dio;
};

}  // namespace

std::unique_ptr<VmxBackend> CreateBackend() {
  return std::make_unique<StudicaBackend>();
}

}  // namespace wpilibvmx
