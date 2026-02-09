/**
 * \file pros/distance.hpp
 *
 * Contains C++ distance sensor class
 */

#ifndef _PROS_DISTANCE_HPP_
#define _PROS_DISTANCE_HPP_

#include <cstdint>

#include "pros/device.hpp"
#include "pros/distance.h"

namespace pros {
inline namespace v5 {

class Distance : public Device {
 public:
  explicit Distance(const std::uint8_t port);

  std::int32_t get() const;
  std::int32_t get_confidence() const;
};

}  // namespace v5
}  // namespace pros

#endif  // _PROS_DISTANCE_HPP_
