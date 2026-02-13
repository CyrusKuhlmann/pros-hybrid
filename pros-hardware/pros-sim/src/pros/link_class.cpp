/**
 * PROS Link C++ class stub implementations for simulation
 */
#include "pros/link.hpp"

namespace pros {

Link::Link(const std::uint8_t port, const std::string link_id,
           link_type_e_t type, bool ov)
    : Device(port, DeviceType::radio) {
  (void)link_id;
  (void)type;
  (void)ov;
}

bool Link::connected() { return false; }
std::uint32_t Link::raw_receivable_size() { return 0; }
std::uint32_t Link::raw_transmittable_size() { return 0; }
std::uint32_t Link::transmit_raw(void* data, std::uint16_t data_size) {
  (void)data;
  (void)data_size;
  return 0;
}
std::uint32_t Link::receive_raw(void* dest, std::uint16_t data_size) {
  (void)dest;
  (void)data_size;
  return 0;
}
std::uint32_t Link::transmit(void* data, std::uint16_t data_size) {
  (void)data;
  (void)data_size;
  return 0;
}
std::uint32_t Link::receive(void* dest, std::uint16_t data_size) {
  (void)dest;
  (void)data_size;
  return 0;
}
std::uint32_t Link::clear_receive_buf() { return 1; }

}  // namespace pros
