#include "../sim/sim_device_registry.h"
#include "pros/misc.h"
#include "pros/misc.hpp"

extern "C" {
namespace pros {

int32_t controller_get_analog(controller_id_e_t id,
                              controller_analog_e_t channel) {
  bool is_partner = (id == E_CONTROLLER_PARTNER);
  auto state =
      pros_sim::SimDeviceRegistry::instance().get_controller_state(is_partner);

  switch (channel) {
    case E_CONTROLLER_ANALOG_LEFT_X:
      return state.left_x;
    case E_CONTROLLER_ANALOG_LEFT_Y:
      return state.left_y;
    case E_CONTROLLER_ANALOG_RIGHT_X:
      return state.right_x;
    case E_CONTROLLER_ANALOG_RIGHT_Y:
      return state.right_y;
    default:
      return 0;
  }
}

int32_t controller_get_digital(controller_id_e_t id,
                               controller_digital_e_t button) {
  bool is_partner = (id == E_CONTROLLER_PARTNER);
  auto state =
      pros_sim::SimDeviceRegistry::instance().get_controller_state(is_partner);

  int button_bit = static_cast<int>(button) - 6;  // Buttons start at 6
  return (state.buttons & (1 << button_bit)) ? 1 : 0;
}

}  // namespace pros
}  // extern "C"

namespace pros {

Controller::Controller(controller_id_e_t id) : _id(id) {}

std::int32_t Controller::get_analog(controller_analog_e_t channel) {
  return controller_get_analog(_id, channel);
}

std::int32_t Controller::get_digital(controller_digital_e_t button) {
  return controller_get_digital(_id, button);
}

}  // namespace pros
