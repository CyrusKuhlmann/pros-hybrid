/**
 * \file pros/imu.hpp
 *
 * Contains C++ IMU class
 */

#ifndef _PROS_IMU_HPP_
#define _PROS_IMU_HPP_

#include <cstdint>

#include "pros/device.hpp"
#include "pros/imu.h"

namespace pros {
inline namespace v5 {

class Imu : public Device {
 public:
  explicit Imu(const std::uint8_t port);

  std::int32_t reset();
  double get_heading() const;
  double get_rotation() const;
  std::int32_t tare_heading();
  std::int32_t tare_rotation();
  std::int32_t set_heading(double target);
  std::int32_t set_rotation(double target);
};

}  // namespace v5
}  // namespace pros

#endif  // _PROS_IMU_HPP_
