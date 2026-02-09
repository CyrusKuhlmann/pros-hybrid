#include "pros/distance.h"

#include "../sim/sim_device_registry.h"
#include "pros/distance.hpp"

extern "C" {
namespace pros {

int32_t distance_get(uint8_t port) {
  auto state = pros_sim::SimDeviceRegistry::instance().get_distance_state(port);
  return state.distance_mm;
}

int32_t distance_get_confidence(uint8_t port) {
  (void)port;
  return 100;  // Always 100% confident in simulation
}

}  // namespace pros
}  // extern "C"

namespace pros {
inline namespace v5 {

Distance::Distance(const std::uint8_t port)
    : Device(port, DeviceType::distance) {}

std::int32_t Distance::get() const { return distance_get(_port); }

std::int32_t Distance::get_confidence() const {
  return distance_get_confidence(_port);
}

}  // namespace v5
}  // namespace pros
