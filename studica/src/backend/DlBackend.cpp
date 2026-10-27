// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/detail/backend/DlBackend.hpp"

#include <string>
#include <utility>

#include "wpi/studica/detail/backend/BackendApi.h"

#if defined(_WIN32)

namespace wpi::studica {
std::unique_ptr<HardwareBackend> LoadBackendPlugin(const std::string&,
                                                   std::string* error) {
  *error = "backend plugins are not supported on Windows";
  return nullptr;
}
}  // namespace wpi::studica

#else

#include <dlfcn.h>

namespace wpi::studica {
namespace {

class DlBackend : public HardwareBackend {
 public:
  DlBackend(void* handle, wpilibstudica_BackendApi* api,
            wpilibstudica_DestroyBackendV3_fn destroy)
      : m_handle{handle}, m_api{api}, m_destroy{destroy} {}

  ~DlBackend() override {
    m_destroy(m_api);
    dlclose(m_handle);
  }

  bool InitDigital(int ch, bool isInput) override {
    return m_api->initDigital(m_api->ctx, ch, isInput ? 1 : 0) != 0;
  }
  void ReleaseDigital(int ch) override {
    m_api->releaseDigital(m_api->ctx, ch);
  }
  void SetDigital(int ch, bool value) override {
    m_api->setDigital(m_api->ctx, ch, value ? 1 : 0);
  }
  bool GetDigital(int ch) override {
    return m_api->getDigital(m_api->ctx, ch) != 0;
  }

  bool InitAnalog(int ch) override {
    return m_api->initAnalog(m_api->ctx, ch) != 0;
  }
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
    wpilibstudica_ImuSample s;
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

  bool InitPwm(int ch, int periodMicros) override {
    return m_api->initPwm(m_api->ctx, ch, periodMicros) != 0;
  }
  void ReleasePwm(int ch) override { m_api->releasePwm(m_api->ctx, ch); }
  void SetPwmPulse(int ch, int pulseMicros) override {
    m_api->setPwmPulse(m_api->ctx, ch, pulseMicros);
  }
  bool InitI2c() override { return m_api->initI2c(m_api->ctx) != 0; }
  void ReleaseI2c() override { m_api->releaseI2c(m_api->ctx); }
  bool TransactionI2c(int address, const uint8_t* tx, int txSize, uint8_t* rx,
                      int rxSize) override {
    return m_api->transactionI2c(m_api->ctx, address, tx, txSize, rx, rxSize) !=
           0;
  }
  bool InitDutyCycle(int ch) override {
    return m_api->initDutyCycle(m_api->ctx, ch) != 0;
  }
  void ReleaseDutyCycle(int ch) override {
    m_api->releaseDutyCycle(m_api->ctx, ch);
  }
  bool GetDutyCycle(int ch, DutyCycleReading* reading) override {
    return m_api->getDutyCycle(m_api->ctx, ch, &reading->frequencyHz,
                               &reading->output) != 0;
  }
  bool InitInterrupt(int ch) override {
    return m_api->initInterrupt(m_api->ctx, ch) != 0;
  }
  void ReleaseInterrupt(int ch) override {
    m_api->releaseInterrupt(m_api->ctx, ch);
  }
  bool ReadInterrupt(int ch, InterruptEvent* event) override {
    int rising = 0;
    if (!m_api->readInterrupt(m_api->ctx, ch, &rising, &event->timestampMicros,
                              &event->dropped)) {
      return false;
    }
    event->rising = rising != 0;
    return true;
  }
  bool GetBatteryVoltage(double* volts) override {
    return m_api->getBatteryVoltage(m_api->ctx, volts) != 0;
  }

 private:
  void* m_handle;
  wpilibstudica_BackendApi* m_api;
  wpilibstudica_DestroyBackendV3_fn m_destroy;
};

}  // namespace

std::unique_ptr<HardwareBackend> LoadBackendPlugin(const std::string& path,
                                                   std::string* error) {
  int flags = RTLD_NOW | RTLD_LOCAL;
#if defined(RTLD_NODELETE)
  // The SDK can retain callback function pointers during hardware shutdown.
  flags |= RTLD_NODELETE;
#endif
  void* handle = dlopen(path.c_str(), flags);
  if (handle == nullptr) {
    *error = std::string{"dlopen failed: "} + dlerror();
    return nullptr;
  }

  auto create = reinterpret_cast<wpilibstudica_CreateBackendV3_fn>(
      dlsym(handle, WPILIBSTUDICA_CREATE_SYMBOL));
  auto destroy = reinterpret_cast<wpilibstudica_DestroyBackendV3_fn>(
      dlsym(handle, WPILIBSTUDICA_DESTROY_SYMBOL));
  if (create == nullptr || destroy == nullptr) {
    *error = std::string{"not a halsim_studica backend plugin: missing "} +
             WPILIBSTUDICA_CREATE_SYMBOL + " / " + WPILIBSTUDICA_DESTROY_SYMBOL;
    dlclose(handle);
    return nullptr;
  }

  wpilibstudica_BackendApi* api = create();
  if (api == nullptr) {
    *error = "backend plugin could not be created";
    dlclose(handle);
    return nullptr;
  }
  if (api->abiVersion != WPILIBSTUDICA_BACKEND_ABI_VERSION ||
      api->structSize != sizeof(wpilibstudica_BackendApi)) {
    *error = "backend plugin ABI mismatch (plugin version " +
             std::to_string(api->abiVersion) + ", size " +
             std::to_string(api->structSize) + "; expected version " +
             std::to_string(WPILIBSTUDICA_BACKEND_ABI_VERSION) + ", size " +
             std::to_string(sizeof(wpilibstudica_BackendApi)) + ")";
    // Not safe to call anything on a table we do not understand, but destroy is
    // a plain function and takes the table as it was created.
    destroy(api);
    dlclose(handle);
    return nullptr;
  }

  return std::make_unique<DlBackend>(handle, api, destroy);
}

}  // namespace wpi::studica

#endif
