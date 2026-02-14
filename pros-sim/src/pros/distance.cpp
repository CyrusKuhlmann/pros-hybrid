/**
 * PROS Distance C API implementations for simulation.
 * Reads state from the Python VEX simulator.
 */
#include "pros/distance.h"

#include "sim/sim_client.h"

extern "C" {
  namespace pros {
    namespace c {

      int32_t distance_get(uint8_t port) {
        return sim::SimClient::instance().get_distance_state(port).distance_mm;
      }
      int32_t distance_get_confidence(uint8_t port) {
        // Report high confidence if we have a reading
        auto d = sim::SimClient::instance().get_distance_state(port).distance_mm;
        return d > 0 ? 63 : 0;
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
