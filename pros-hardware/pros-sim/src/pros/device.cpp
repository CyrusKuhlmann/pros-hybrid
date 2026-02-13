/**
 * PROS Device C API stub implementations for simulation
 */
#include "pros/device.h"

extern "C" {
  namespace pros {
    namespace c {

      v5_device_e_t get_plugged_type(uint8_t port) {
        (void)port;
        return E_DEVICE_NONE;
      }

    }  // namespace c
  }  // namespace pros
}  // extern "C"
