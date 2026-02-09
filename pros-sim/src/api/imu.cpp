#include "pros/imu.h"

#include "../sim/sim_device_registry.h"
#include "pros/imu.hpp"

extern "C" {
namespace pros {

int32_t imu_reset(uint8_t port) {
  pros_sim::SimDeviceRegistry::instance().reset_imu(port);
  return 1;
}

double imu_get_heading(uint8_t port) {
  auto state = pros_sim::SimDeviceRegistry::instance().get_imu_state(port);
  return state.heading;
}

double imu_get_rotation(uint8_t port) {
  auto state = pros_sim::SimDeviceRegistry::instance().get_imu_state(port);
  return state.rotation;
}

int32_t imu_tare_heading(uint8_t port) {
  (void)port;
  return 1;
}

int32_t imu_tare_rotation(uint8_t port) { return imu_reset(port); }

int32_t imu_set_heading(uint8_t port, double target) {
  (void)port;
  (void)target;
  return 1;
}

int32_t imu_set_rotation(uint8_t port, double target) {
  (void)port;
  (void)target;
  return 1;
}

}  // namespace pros
}  // extern "C"

namespace pros {
inline namespace v5 {

Imu::Imu(const std::uint8_t port) : Device(port, DeviceType::imu) {}

std::int32_t Imu::reset() { return imu_reset(_port); }

double Imu::get_heading() const { return imu_get_heading(_port); }

double Imu::get_rotation() const { return imu_get_rotation(_port); }

std::int32_t Imu::tare_heading() { return imu_tare_heading(_port); }

std::int32_t Imu::tare_rotation() { return imu_tare_rotation(_port); }

std::int32_t Imu::set_heading(double target) {
  return imu_set_heading(_port, target);
}

std::int32_t Imu::set_rotation(double target) {
  return imu_set_rotation(_port, target);
}

}  // namespace v5
}  // namespace pros
