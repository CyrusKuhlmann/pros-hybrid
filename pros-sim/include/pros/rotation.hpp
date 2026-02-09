/**
 * \file pros/rotation.hpp
 *
 * Contains C++ rotation sensor class
 */

#ifndef _PROS_ROTATION_HPP_
#define _PROS_ROTATION_HPP_

#include <cstdint>

#include "pros/device.hpp"
#include "pros/rotation.h"

namespace pros {
inline namespace v5 {

class Rotation : public Device {
 public:
  explicit Rotation(const std::uint8_t port);

  std::int32_t reset();
  std::int32_t get_position() const;
  std::int32_t get_velocity() const;
  std::int32_t set_position(std::uint32_t position);
  std::int32_t reset_position();
};

}  // namespace v5
}  // namespace pros

#endif  // _PROS_ROTATION_HPP_
