/**
 * PROS Optical C API stub implementations for simulation
 */
#include "pros/optical.h"

extern "C" {
  namespace pros {
    namespace c {

      double optical_get_hue(uint8_t port) {
        (void)port;
        return 0.0;
      }
      double optical_get_saturation(uint8_t port) {
        (void)port;
        return 0.0;
      }
      double optical_get_brightness(uint8_t port) {
        (void)port;
        return 0.0;
      }
      int32_t optical_get_proximity(uint8_t port) {
        (void)port;
        return 0;
      }
      int32_t optical_set_led_pwm(uint8_t port, uint8_t value) {
        (void)port;
        (void)value;
        return 1;
      }
      int32_t optical_get_led_pwm(uint8_t port) {
        (void)port;
        return 0;
      }
      optical_rgb_s_t optical_get_rgb(uint8_t port) {
        (void)port;
        optical_rgb_s_t c = {};
        return c;
      }
      optical_raw_s_t optical_get_raw(uint8_t port) {
        (void)port;
        optical_raw_s_t r = {};
        return r;
      }
      optical_direction_e_t optical_get_gesture(uint8_t port) {
        (void)port;
        return NO_GESTURE;
      }
      optical_gesture_s_t optical_get_gesture_raw(uint8_t port) {
        (void)port;
        optical_gesture_s_t g = {};
        return g;
      }
      int32_t optical_enable_gesture(uint8_t port) {
        (void)port;
        return 1;
      }
      int32_t optical_disable_gesture(uint8_t port) {
        (void)port;
        return 1;
      }
      double optical_get_integration_time(uint8_t port) {
        (void)port;
        return 0.0;
      }
      int32_t optical_set_integration_time(uint8_t port, double time) {
        (void)port;
        (void)time;
        return 1;
      }

    }  // namespace c
  }  // namespace pros
}  // extern "C"
