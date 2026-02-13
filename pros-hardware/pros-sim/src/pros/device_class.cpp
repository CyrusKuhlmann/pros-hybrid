/**
 * PROS Device C++ class stub implementations for simulation
 */
#include <vector>

#include "pros/device.h"
#include "pros/device.hpp"

namespace pros {
inline namespace v5 {

Device::Device(const std::uint8_t port)
    : _port(port), _deviceType(DeviceType::undefined) {}

std::uint8_t Device::get_port() const { return _port; }

bool Device::is_installed() { return true; }

DeviceType Device::get_plugged_type() const { return _deviceType; }

DeviceType Device::get_plugged_type(std::uint8_t port) {
  (void)port;
  return DeviceType::undefined;
}

std::vector<Device> Device::get_all_devices(DeviceType device_type) {
  (void)device_type;
  return {};
}

}  // namespace v5
}  // namespace pros
