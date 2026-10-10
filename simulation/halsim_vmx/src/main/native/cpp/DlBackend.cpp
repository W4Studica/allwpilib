// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/halsim/vmx/DlBackend.hpp"

#include <string>
#include <utility>

#include "wpi/halsim/vmx/BackendApi.h"

#if defined(_WIN32)

namespace wpilibvmx {
std::unique_ptr<VmxBackend> LoadBackendPlugin(const std::string&,
                                              std::string* error) {
  *error = "backend plugins are not supported on Windows";
  return nullptr;
}
}  // namespace wpilibvmx

#else

#include <dlfcn.h>

namespace wpilibvmx {
namespace {

class DlBackend : public VmxBackend {
 public:
  DlBackend(void* handle, wpilibvmx_BackendApi* api,
            wpilibvmx_DestroyBackendV1_fn destroy)
      : m_handle{handle}, m_api{api}, m_destroy{destroy} {}

  ~DlBackend() override {
    m_destroy(m_api);
    dlclose(m_handle);
  }

  bool InitDigital(int ch, bool isInput) override {
    return m_api->initDigital(m_api->ctx, ch, isInput ? 1 : 0) != 0;
  }
  void ReleaseDigital(int ch) override { m_api->releaseDigital(m_api->ctx, ch); }
  void SetDigital(int ch, bool value) override {
    m_api->setDigital(m_api->ctx, ch, value ? 1 : 0);
  }
  bool GetDigital(int ch) override { return m_api->getDigital(m_api->ctx, ch) != 0; }

  bool InitAnalog(int ch) override { return m_api->initAnalog(m_api->ctx, ch) != 0; }
  void ReleaseAnalog(int ch) override { m_api->releaseAnalog(m_api->ctx, ch); }
  bool GetAnalogVoltage(int ch, double* volts) override {
    return m_api->getAnalogVoltage(m_api->ctx, ch, volts) != 0;
  }

  bool InitEncoder(int a, int b) override {
    return m_api->initEncoder(m_api->ctx, a, b) != 0;
  }
  void ReleaseEncoder(int a) override { m_api->releaseEncoder(m_api->ctx, a); }
  bool GetEncoderCount(int a, int32_t* count) override {
    return m_api->getEncoderCount(m_api->ctx, a, count) != 0;
  }

  bool InitImu() override { return m_api->initImu(m_api->ctx) != 0; }
  void ReleaseImu() override { m_api->releaseImu(m_api->ctx); }
  bool GetImu(ImuSample* sample) override {
    wpilibvmx_ImuSample s;
    if (m_api->getImu(m_api->ctx, &s) == 0) {
      return false;
    }
    sample->yawDeg = s.yawDeg;
    sample->pitchDeg = s.pitchDeg;
    sample->rollDeg = s.rollDeg;
    sample->gyroXDps = s.gyroXDps;
    sample->gyroYDps = s.gyroYDps;
    sample->gyroZDps = s.gyroZDps;
    sample->accelXG = s.accelXG;
    sample->accelYG = s.accelYG;
    sample->accelZG = s.accelZG;
    return true;
  }

 private:
  void* m_handle;
  wpilibvmx_BackendApi* m_api;
  wpilibvmx_DestroyBackendV1_fn m_destroy;
};

}  // namespace

std::unique_ptr<VmxBackend> LoadBackendPlugin(const std::string& path,
                                              std::string* error) {
  void* handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (handle == nullptr) {
    *error = std::string{"dlopen failed: "} + dlerror();
    return nullptr;
  }

  auto create = reinterpret_cast<wpilibvmx_CreateBackendV1_fn>(
      dlsym(handle, WPILIBVMX_CREATE_SYMBOL));
  auto destroy = reinterpret_cast<wpilibvmx_DestroyBackendV1_fn>(
      dlsym(handle, WPILIBVMX_DESTROY_SYMBOL));
  if (create == nullptr || destroy == nullptr) {
    *error = std::string{"not a halsim_vmx backend plugin: missing "} +
             WPILIBVMX_CREATE_SYMBOL + " / " + WPILIBVMX_DESTROY_SYMBOL;
    dlclose(handle);
    return nullptr;
  }

  wpilibvmx_BackendApi* api = create();
  if (api == nullptr) {
    *error = "backend plugin could not be created";
    dlclose(handle);
    return nullptr;
  }
  if (api->abiVersion != WPILIBVMX_BACKEND_ABI_VERSION ||
      api->structSize != sizeof(wpilibvmx_BackendApi)) {
    *error = "backend plugin ABI mismatch (plugin version " +
             std::to_string(api->abiVersion) + ", size " +
             std::to_string(api->structSize) + "; expected version " +
             std::to_string(WPILIBVMX_BACKEND_ABI_VERSION) + ", size " +
             std::to_string(sizeof(wpilibvmx_BackendApi)) + ")";
    // Not safe to call anything on a table we do not understand, but destroy is a
    // plain function and takes the table as it was created.
    destroy(api);
    dlclose(handle);
    return nullptr;
  }

  return std::make_unique<DlBackend>(handle, api, destroy);
}

}  // namespace wpilibvmx

#endif
