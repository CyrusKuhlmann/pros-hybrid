/**
 * \file pros/device.hpp
 *
 * Base class for smart devices
 */

#ifndef _PROS_DEVICE_HPP_
#define _PROS_DEVICE_HPP_

#include <cstdint>

namespace pros {
inline namespace v5 {

enum class DeviceType {
  none = 0,
  motor = 2,
  rotation = 4,
  imu = 6,
  distance = 7,
  radio = 8,
  vision = 11,
  adi = 12,
  optical = 16,
  gps = 20,
  aivision = 29,
  serial = 129,
  undefined = 255
};

class Device {
 public:
  explicit Device(const std::uint8_t port);
  std::uint8_t get_port() const;
  virtual bool is_installed();
  DeviceType get_plugged_type() const;

 protected:
  Device(const std::uint8_t port, const DeviceType deviceType)
      : _port(port), _deviceType(deviceType) {}

  const std::uint8_t _port;
  const DeviceType _deviceType = DeviceType::none;
};

}  // namespace v5
}  // namespace pros

#endif  // _PROS_DEVICE_HPP_
