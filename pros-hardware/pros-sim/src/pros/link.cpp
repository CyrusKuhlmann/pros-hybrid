/**
 * PROS Link C API stub implementations for simulation
 */
#include "pros/link.h"

extern "C" {
  namespace pros {
    namespace c {

      uint32_t link_init(uint8_t port, const char* link_id, link_type_e_t type) {
        (void)port;
        (void)link_id;
        (void)type;
        return 1;
      }
      uint32_t link_init_override(uint8_t port, const char* link_id,
        link_type_e_t type) {
        (void)port;
        (void)link_id;
        (void)type;
        return 1;
      }
      bool link_connected(uint8_t port) {
        (void)port;
        return false;
      }
      uint32_t link_raw_receivable_size(uint8_t port) {
        (void)port;
        return 0;
      }
      uint32_t link_raw_transmittable_size(uint8_t port) {
        (void)port;
        return 0;
      }
      uint32_t link_transmit_raw(uint8_t port, void* data, uint16_t data_size) {
        (void)port;
        (void)data;
        (void)data_size;
        return 0;
      }
      uint32_t link_receive_raw(uint8_t port, void* dest, uint16_t data_size) {
        (void)port;
        (void)dest;
        (void)data_size;
        return 0;
      }
      uint32_t link_transmit(uint8_t port, void* data, uint16_t data_size) {
        (void)port;
        (void)data;
        (void)data_size;
        return 0;
      }
      uint32_t link_receive(uint8_t port, void* dest, uint16_t data_size) {
        (void)port;
        (void)dest;
        (void)data_size;
        return 0;
      }
      uint32_t link_clear_receive_buf(uint8_t port) {
        (void)port;
        return 1;
      }

    }  // namespace c
  }  // namespace pros
}  // extern "C"
