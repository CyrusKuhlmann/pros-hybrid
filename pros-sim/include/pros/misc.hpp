/**
 * \file pros/misc.hpp
 *
 * Contains miscellaneous C++ classes including Controller
 */

#ifndef _PROS_MISC_HPP_
#define _PROS_MISC_HPP_

#include <cstdint>

#include "pros/misc.h"

namespace pros {

class Controller {
 public:
  explicit Controller(controller_id_e_t id);

  std::int32_t get_analog(controller_analog_e_t channel);
  std::int32_t get_digital(controller_digital_e_t button);

 private:
  controller_id_e_t _id;
};

}  // namespace pros

#endif  // _PROS_MISC_HPP_
