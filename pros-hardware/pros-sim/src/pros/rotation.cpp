/**
 * PROS Rotation C API stub implementations for simulation
 */
#include "pros/rotation.h"

extern "C" {
  namespace pros {
    namespace c {

      int32_t rotation_reset(uint8_t port) {
        (void)port;
        return 1;
      }
      int32_t rotation_set_data_rate(uint8_t port, uint32_t rate) {
        (void)port;
        (void)rate;
        return 1;
      }
      int32_t rotation_set_position(uint8_t port, int32_t position) {
        (void)port;
        (void)position;
        return 1;
      }
      int32_t rotation_reset_position(uint8_t port) {
        (void)port;
        return 1;
      }
      int32_t rotation_get_position(uint8_t port) {
        (void)port;
        return 0;
      }
      int32_t rotation_get_velocity(uint8_t port) {
        (void)port;
        return 0;
      }
      int32_t rotation_get_angle(uint8_t port) {
        (void)port;
        return 0;
      }
      int32_t rotation_set_reversed(uint8_t port, bool value) {
        (void)port;
        (void)value;
        return 1;
      }
      int32_t rotation_reverse(uint8_t port) {
        (void)port;
        return 1;
      }
      int32_t rotation_init_reverse(uint8_t port, bool reverse_flag) {
        (void)port;
        (void)reverse_flag;
        return 1;
      }
      int32_t rotation_get_reversed(uint8_t port) {
        (void)port;
        return 0;
      }

    }  // namespace c
  }  // namespace pros
}  // extern "C"
