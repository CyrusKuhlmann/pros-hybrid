#include "pros/device.hpp"

namespace pros {
inline namespace v5 {

Device::Device(const std::uint8_t port)
    : _port(port), _deviceType(DeviceType::none) {}

std::uint8_t Device::get_port() const { return _port; }

bool Device::is_installed() {
  // In simulator, all devices are "installed"
  return true;
}

DeviceType Device::get_plugged_type() const { return _deviceType; }

}  // namespace v5
}  // namespace pros
