/**
 * PROS Distance C++ class implementations for simulation.
 * Reads state from the Python VEX simulator.
 */
#include "pros/distance.hpp"
#include "sim/sim_client.h"

namespace pros {
  inline namespace v5 {

    Distance::Distance(const std::uint8_t port)
      : Device(port, DeviceType::distance) {
    }
    std::vector<Distance> Distance::get_all_devices() { return {}; }

    std::int32_t Distance::get() {
      return sim::SimClient::instance().get_distance_state(_port).distance_mm;
    }
    std::int32_t Distance::get_distance() {
      return get();
    }
    std::int32_t Distance::get_confidence() {
      auto d = sim::SimClient::instance().get_distance_state(_port).distance_mm;
      return d > 0 ? 63 : 0;
    }
    std::int32_t Distance::get_object_size() { return 0; }
    double Distance::get_object_velocity() { return 0.0; }

    std::ostream& operator<<(std::ostream& os, const Distance& dist) {
      os << "Distance [port: " << (int)dist.get_port() << "]";
      return os;
    }

    namespace literals {
      const Distance operator""_dist(const unsigned long long int d) {
        return Distance((std::uint8_t)d);
      }
    }  // namespace literals

  }  // namespace v5
}  // namespace pros
