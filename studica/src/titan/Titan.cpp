// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/studica/Titan.hpp"

#include <cmath>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "TitanEnableSwitch.hpp"
#include "wpi/studica/SharedVMX.hpp"
#include "wpi/studica/detail/titan/TitanControl.hpp"
#include "wpi/studica/drivers/Titan.hpp"

namespace wpi::studica {
namespace {
std::shared_ptr<VMXPi> OpenSharedVMX() {
  auto vmx = SharedVMX();
  if (!vmx->IsOpen()) {
    throw std::runtime_error("Cannot open the shared VMXPi");
  }
  return vmx;
}
void CheckMotor(uint8_t motor) {
  if (motor > 3) {
    throw std::out_of_range("Titan motor must be 0..3");
  }
}
void CheckFinite(double value) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument("Titan value must be finite");
  }
}
}  // namespace

struct Titan::Impl {
  uint8_t id;
  std::shared_ptr<VMXPi> vmx;
  std::mutex driverMutex;
  drivers::Titan driver;
  detail::TitanEnableSwitch enable;
  detail::TitanControl<detail::TitanEnableSwitch> control;

  Impl(uint8_t canId, const Config& config, EnabledSource enabled)
      : id{canId},
        vmx{OpenSharedVMX()},
        driver{canId, config.motorFrequencyHz, config.distancePerTick, vmx},
        enable{canId, vmx},
        control{enable, config.controlTimeout, config.driverStationTimeout,
                std::move(enabled)} {}
};

Titan::Titan(uint8_t canId, EnabledSource enabled)
    : Titan{canId, Config{}, std::move(enabled)} {}
Titan::Titan(uint8_t canId, uint16_t frequency, float distancePerTick,
             EnabledSource enabled)
    : Titan{canId, Config{frequency, distancePerTick}, std::move(enabled)} {}
Titan::Titan(uint8_t canId, Config config, EnabledSource enabled) {
  // Reject invalid input before opening VMX or starting a background worker.
  if (canId == 0 || canId > 63) {
    throw std::out_of_range("Titan CAN ID must be 1..63");
  }
  if (config.motorFrequencyHz == 0 || config.motorFrequencyHz > 20000) {
    throw std::out_of_range("Titan motor frequency must be 1..20000 Hz");
  }
  if (!std::isfinite(config.distancePerTick) || config.distancePerTick < 0 ||
      config.controlTimeout.count() <= 0 ||
      config.driverStationTimeout.count() <= 0 || !enabled) {
    throw std::invalid_argument("Invalid Titan configuration");
  }
  m_impl = std::make_unique<Impl>(canId, config, std::move(enabled));
}
Titan::~Titan() = default;
void Titan::FeedControlLoop() {
  m_impl->control.FeedControlLoop();
}
void Titan::FeedDriverStation() {
  m_impl->control.FeedDriverStation();
}
bool Titan::IsEnabled() const {
  return m_impl->control.IsEnabled();
}
bool Titan::LastSendOk() const {
  return m_impl->enable.LastSendOk();
}
void Titan::Stop() {
  m_impl->control.Stop();
}

void Titan::SetupEncoder(uint8_t encoder) {
  CheckMotor(encoder);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.SetupEncoder(encoder);
}

uint8_t Titan::GetID() {
  return m_impl->id;
}

uint16_t Titan::GetFrequency() {
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetFrequency();
}

std::string Titan::GetFirmwareVersion() {
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetFirmwareVersion();
}

std::string Titan::GetHardwareVersion() {
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetHardwareVersion();
}

float Titan::GetControllerTemp() {
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetControllerTemp();
}

bool Titan::GetLimitSwitch(uint8_t motor, uint8_t direction) {
  CheckMotor(motor);
  if (direction > 1) {
    throw std::out_of_range("Titan limit direction must be 0 or 1");
  }
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetLimitSwitch(motor, direction);
}

bool Titan::GetLimitSwitchesFresh(bool fwd[4], bool rev[4], bool& is_fresh,
                                  uint64_t* out_timestamp_us) {
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetLimitSwitchesFresh(fwd, rev, is_fresh,
                                              out_timestamp_us);
}

float Titan::GetRPM(uint8_t motor) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetRPM(motor);
}

bool Titan::TryGetRPM(uint8_t motor, float* out_rpm) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.TryGetRPM(motor, out_rpm);
}

bool Titan::GetRPMFresh(uint8_t motor, float& rpm, bool& is_fresh,
                        uint64_t* out_timestamp_us) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetRPMFresh(motor, rpm, is_fresh, out_timestamp_us);
}

std::string Titan::GetSerialNumber() {
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetSerialNumber();
}

double Titan::GetEncoderDistance(uint8_t motor) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetEncoderDistance(motor);
}

int32_t Titan::GetEncoderCount(uint8_t motor) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetEncoderCount(motor);
}

bool Titan::GetEncoderDistanceFresh(uint8_t motor, double& distance,
                                    bool& is_fresh,
                                    uint64_t* out_timestamp_us) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetEncoderDistanceFresh(motor, distance, is_fresh,
                                                out_timestamp_us);
}

void Titan::ConfigureEncoder(uint8_t motor, double cfg) {
  CheckMotor(motor);
  CheckFinite(cfg);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.ConfigureEncoder(motor, cfg);
}

void Titan::ResetEncoder(uint8_t motor) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.ResetEncoder(motor);
}

double Titan::GetCypherAngle(uint8_t port) {
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetCypherAngle(port);
}

bool Titan::GetCypherAnglesFresh(double angles[4], bool& is_fresh,
                                 uint64_t* out_timestamp_us) {
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetCypherAnglesFresh(angles, is_fresh,
                                             out_timestamp_us);
}

void Titan::SetSpeed(uint8_t motor, double speedCfg) {
  CheckMotor(motor);
  CheckFinite(speedCfg);
  std::scoped_lock lock{m_impl->driverMutex};
  if (!m_impl->control.CanCommand()) {
    return;
  }
  m_impl->driver.SetSpeed(motor, speedCfg);
}

void Titan::SetSpeedAll(double duty) {
  CheckFinite(duty);
  std::scoped_lock lock{m_impl->driverMutex};
  if (!m_impl->control.CanCommand()) {
    return;
  }
  for (uint8_t motor = 0; motor < 4; ++motor) {
    m_impl->driver.SetSpeed(motor, duty);
  }
}

void Titan::InvertMotorDirection(uint8_t motor) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.InvertMotorDirection(motor);
}

void Titan::InvertMotorRPM(uint8_t motor) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.InvertMotorRPM(motor);
}

void Titan::InvertEncoderDirection(uint8_t motor) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.InvertEncoderDirection(motor);
}

void Titan::InvertMotor(uint8_t motor) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.InvertMotor(motor);
}

void Titan::SetTargetVelocity(uint8_t motor, float velocityRpm) {
  CheckMotor(motor);
  CheckFinite(velocityRpm);
  std::scoped_lock lock{m_impl->driverMutex};
  if (!m_impl->control.CanCommand()) {
    return;
  }
  m_impl->driver.SetTargetVelocity(motor, velocityRpm);
}

float Titan::GetTargetRPM(uint8_t motor) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.GetTargetRPM(motor);
}

bool Titan::TryGetTargetRPM(uint8_t motor, float* out_rpm) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.TryGetTargetRPM(motor, out_rpm);
}

bool Titan::TryGetTargetRPMFromAll(float targetRpm[4]) {
  std::scoped_lock lock{m_impl->driverMutex};
  return m_impl->driver.TryGetTargetRPMFromAll(targetRpm);
}

void Titan::SetTargetDistance(uint8_t motor, int32_t distanceCounts) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  if (!m_impl->control.CanCommand()) {
    return;
  }
  m_impl->driver.SetTargetDistance(motor, distanceCounts);
}

void Titan::SetTargetAngle(uint8_t motor, double angleDeg) {
  CheckMotor(motor);
  CheckFinite(angleDeg);
  std::scoped_lock lock{m_impl->driverMutex};
  if (!m_impl->control.CanCommand()) {
    return;
  }
  m_impl->driver.SetTargetAngle(motor, angleDeg);
}

void Titan::SetPositionHold(uint8_t motor, bool hold) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  if (hold && !m_impl->control.CanCommand()) {
    return;
  }
  m_impl->driver.SetPositionHold(motor, hold);
}

void Titan::SetEncoderResolution(uint8_t channel, uint16_t cpr) {
  CheckMotor(channel);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.SetEncoderResolution(channel, cpr);
}

void Titan::SetCurrentLimit(uint8_t channel, float limitAmps) {
  CheckMotor(channel);
  CheckFinite(limitAmps);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.SetCurrentLimit(channel, limitAmps);
}

void Titan::SetCurrentLimitMode(uint8_t channel, uint8_t mode) {
  CheckMotor(channel);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.SetCurrentLimitMode(channel, mode);
}

void Titan::SetMotorStopMode(uint8_t mode) {
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.SetMotorStopMode(mode);
}

void Titan::SetPIDType(uint8_t type) {
  std::scoped_lock lock{m_impl->driverMutex};
  if (type > 2)
    throw std::out_of_range("Titan PID type must be 0..2");
  m_impl->driver.SetPIDType(type);
}

void Titan::SetMotorPIDType(uint8_t motor, uint8_t type) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  if (type > 2)
    throw std::out_of_range("Titan PID type must be 0..2");
  m_impl->driver.SetMotorPIDType(motor, type);
}

void Titan::AutotuneAll() {
  std::scoped_lock lock{m_impl->driverMutex};
  if (!m_impl->control.CanCommand()) {
    return;
  }
  m_impl->driver.AutotuneAll();
}

void Titan::AutotuneMotor(uint8_t motor) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  if (!m_impl->control.CanCommand()) {
    return;
  }
  m_impl->driver.AutotuneMotor(motor);
}

void Titan::SetSensitivity(uint8_t motor, uint8_t sensitivity) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.SetSensitivity(motor, sensitivity);
}

void Titan::SetCANSensorOsDelay(uint16_t periodMs) {
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.SetCANSensorOsDelay(periodMs);
}

void Titan::DisableMotor(uint8_t motor) {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.DisableMotor(motor);
}

void Titan::SetAngleLimits(uint8_t motor, double minDeg, double maxDeg) {
  CheckMotor(motor);
  CheckFinite(minDeg);
  CheckFinite(maxDeg);
  std::scoped_lock lock{m_impl->driverMutex};
  if (minDeg > maxDeg)
    throw std::invalid_argument("Titan minimum angle exceeds maximum");
  m_impl->driver.SetAngleLimits(motor, minDeg, maxDeg);
}

void Titan::GetAngleLimits(uint8_t motor, double& minDeg,
                           double& maxDeg) const {
  CheckMotor(motor);
  std::scoped_lock lock{m_impl->driverMutex};
  m_impl->driver.GetAngleLimits(motor, minDeg, maxDeg);
}

std::optional<Titan::Reading<float>> Titan::ReadRPM(uint8_t motor) {
  Reading<float> result;
  if (!GetRPMFresh(motor, result.value, result.fresh, &result.timestampMicros))
    return std::nullopt;
  return result;
}
std::optional<Titan::Reading<double>> Titan::ReadEncoderDistance(
    uint8_t motor) {
  Reading<double> result;
  if (!GetEncoderDistanceFresh(motor, result.value, result.fresh,
                               &result.timestampMicros))
    return std::nullopt;
  return result;
}
std::optional<Titan::Reading<std::array<double, 4>>> Titan::ReadCypherAngles() {
  Reading<std::array<double, 4>> result;
  if (!GetCypherAnglesFresh(result.value.data(), result.fresh,
                            &result.timestampMicros))
    return std::nullopt;
  return result;
}
std::optional<Titan::Reading<Titan::LimitSwitches>> Titan::ReadLimitSwitches() {
  Reading<LimitSwitches> result;
  if (!GetLimitSwitchesFresh(result.value.forward.data(),
                             result.value.reverse.data(), result.fresh,
                             &result.timestampMicros))
    return std::nullopt;
  return result;
}

}  // namespace wpi::studica
