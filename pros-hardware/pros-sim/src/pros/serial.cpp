/**
 * PROS Serial C API stub implementations for simulation
 */
#include "pros/serial.h"

extern "C" {
namespace pros {
namespace c {

int32_t serial_enable(uint8_t port) {
  (void)port;
  return 1;
}
int32_t serial_set_baudrate(uint8_t port, int32_t baudrate) {
  (void)port;
  (void)baudrate;
  return 1;
}
int32_t serial_flush(uint8_t port) {
  (void)port;
  return 1;
}
int32_t serial_get_read_avail(uint8_t port) {
  (void)port;
  return 0;
}
int32_t serial_get_write_free(uint8_t port) {
  (void)port;
  return 128;
}
int32_t serial_peek_byte(uint8_t port) {
  (void)port;
  return -1;
}
int32_t serial_read_byte(uint8_t port) {
  (void)port;
  return -1;
}
int32_t serial_read(uint8_t port, uint8_t* buffer, int32_t length) {
  (void)port;
  (void)buffer;
  (void)length;
  return 0;
}
int32_t serial_write_byte(uint8_t port, uint8_t buffer) {
  (void)port;
  (void)buffer;
  return 1;
}
int32_t serial_write(uint8_t port, uint8_t* buffer, int32_t length) {
  (void)port;
  (void)buffer;
  (void)length;
  return length;
}

}  // namespace c
}  // namespace pros
}  // extern "C"
