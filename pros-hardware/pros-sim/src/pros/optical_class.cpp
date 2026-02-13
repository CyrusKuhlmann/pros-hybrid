/**
 * PROS Optical C++ class stub implementations for simulation
 */
#include "pros/optical.hpp"

namespace pros {
  inline namespace v5 {

    Optical::Optical(const std::uint8_t port) : Device(port, DeviceType::optical) {}
    std::vector<Optical> Optical::get_all_devices() { return {}; }
    double Optical::get_hue() { return 0.0; }
    double Optical::get_saturation() { return 0.0; }
    double Optical::get_brightness() { return 0.0; }
    std::int32_t Optical::get_proximity() { return 0; }
    std::int32_t Optical::set_led_pwm(std::uint8_t value) {
      (void)value;
      return 1;
    }
    std::int32_t Optical::get_led_pwm() { return 0; }
    pros::c::optical_rgb_s_t Optical::get_rgb() {
      pros::c::optical_rgb_s_t c = {};
      return c;
    }
    pros::c::optical_raw_s_t Optical::get_raw() {
      pros::c::optical_raw_s_t r = {};
      return r;
    }
    pros::c::optical_direction_e_t Optical::get_gesture() {
      return pros::c::NO_GESTURE;
    }
    pros::c::optical_gesture_s_t Optical::get_gesture_raw() {
      pros::c::optical_gesture_s_t g = {};
      return g;
    }
    std::int32_t Optical::enable_gesture() { return 1; }
    std::int32_t Optical::disable_gesture() { return 1; }
    double Optical::get_integration_time() { return 0.0; }
    std::int32_t Optical::set_integration_time(double time) {
      (void)time;
      return 1;
    }

    std::ostream& operator<<(std::ostream& os, const Optical& opt) {
      os << "Optical [port: " << (int)opt.get_port() << "]";
      return os;
    }

    namespace literals {
      const Optical operator""_opt(const unsigned long long int o) {
        return Optical((std::uint8_t)o);
      }
    }  // namespace literals

  }  // namespace v5
}  // namespace pros
