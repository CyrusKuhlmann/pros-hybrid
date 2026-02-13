/**
 * PROS Distance C API stub implementations for simulation
 */
#include "pros/distance.h"

extern "C" {
  namespace pros {
    namespace c {

      int32_t distance_get(uint8_t port) {
        (void)port;
        return 0;
      }
      int32_t distance_get_confidence(uint8_t port) {
        (void)port;
        return 0;
      }
      int32_t distance_get_object_size(uint8_t port) {
        (void)port;
        return 0;
      }
      double distance_get_object_velocity(uint8_t port) {
        (void)port;
        return 0.0;
      }

    }  // namespace c
  }  // namespace pros
}  // extern "C"
