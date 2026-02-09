#include "pros/rotation.h"

#include "../sim/sim_device_registry.h"
#include "pros/rotation.hpp"

extern "C" {
namespace pros {

int32_t rotation_reset(uint8_t port) {
  pros_sim::SimDeviceRegistry::instance().reset_rotation(port);
  return 1;
}

int32_t rotation_get_position(uint8_t port) {
  auto state = pros_sim::SimDeviceRegistry::instance().get_rotation_state(port);
  return state.position_cdeg;
}

int32_t rotation_get_velocity(uint8_t port) {
  auto state = pros_sim::SimDeviceRegistry::instance().get_rotation_state(port);
  return state.velocity_dps;
}

int32_t rotation_set_position(uint8_t port, uint32_t position) {
  (void)port;
  (void)position;
  return 1;
}

int32_t rotation_reset_position(uint8_t port) { return rotation_reset(port); }

}  // namespace pros
}  // extern "C"

namespace pros {
inline namespace v5 {

Rotation::Rotation(const std::uint8_t port)
    : Device(port, DeviceType::rotation) {}

std::int32_t Rotation::reset() { return rotation_reset(_port); }

std::int32_t Rotation::get_position() const {
  return rotation_get_position(_port);
}

std::int32_t Rotation::get_velocity() const {
  return rotation_get_velocity(_port);
}

std::int32_t Rotation::set_position(std::uint32_t position) {
  return rotation_set_position(_port, position);
}

std::int32_t Rotation::reset_position() {
  return rotation_reset_position(_port);
}

}  // namespace v5
}  // namespace pros
