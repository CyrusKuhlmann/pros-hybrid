/**
 * PROS ADI C API stub implementations for simulation
 */
#include "pros/adi.h"

extern "C" {
namespace pros {
namespace c {

adi_port_config_e_t adi_port_get_config(uint8_t port) {
  (void)port;
  return E_ADI_TYPE_UNDEFINED;
}
int32_t adi_port_get_value(uint8_t port) {
  (void)port;
  return 0;
}
int32_t adi_port_set_config(uint8_t port, adi_port_config_e_t type) {
  (void)port;
  (void)type;
  return 1;
}
int32_t adi_port_set_value(uint8_t port, int32_t value) {
  (void)port;
  (void)value;
  return 1;
}
int32_t adi_analog_calibrate(uint8_t port) {
  (void)port;
  return 0;
}
int32_t adi_analog_read(uint8_t port) {
  (void)port;
  return 0;
}
int32_t adi_analog_read_calibrated(uint8_t port) {
  (void)port;
  return 0;
}
int32_t adi_analog_read_calibrated_HR(uint8_t port) {
  (void)port;
  return 0;
}
int32_t adi_digital_read(uint8_t port) {
  (void)port;
  return 0;
}
int32_t adi_digital_get_new_press(uint8_t port) {
  (void)port;
  return 0;
}
int32_t adi_digital_write(uint8_t port, bool value) {
  (void)port;
  (void)value;
  return 1;
}
int32_t adi_pin_mode(uint8_t port, uint8_t mode) {
  (void)port;
  (void)mode;
  return 1;
}
int32_t adi_motor_set(uint8_t port, int8_t speed) {
  (void)port;
  (void)speed;
  return 1;
}
int32_t adi_motor_get(uint8_t port) {
  (void)port;
  return 0;
}
int32_t adi_motor_stop(uint8_t port) {
  (void)port;
  return 1;
}
int32_t adi_encoder_get(adi_encoder_t enc) {
  (void)enc;
  return 0;
}
adi_encoder_t adi_encoder_init(uint8_t port_top, uint8_t port_bottom,
                               bool reverse) {
  (void)port_top;
  (void)port_bottom;
  (void)reverse;
  return 0;
}
int32_t adi_encoder_reset(adi_encoder_t enc) {
  (void)enc;
  return 1;
}
int32_t adi_encoder_shutdown(adi_encoder_t enc) {
  (void)enc;
  return 1;
}
int32_t adi_ultrasonic_get(adi_ultrasonic_t ult) {
  (void)ult;
  return 0;
}
adi_ultrasonic_t adi_ultrasonic_init(uint8_t port_ping, uint8_t port_echo) {
  (void)port_ping;
  (void)port_echo;
  return 0;
}
int32_t adi_ultrasonic_shutdown(adi_ultrasonic_t ult) {
  (void)ult;
  return 1;
}
double adi_gyro_get(adi_gyro_t gyro) {
  (void)gyro;
  return 0.0;
}
adi_gyro_t adi_gyro_init(uint8_t port, double multiplier) {
  (void)port;
  (void)multiplier;
  return 0;
}
int32_t adi_gyro_reset(adi_gyro_t gyro) {
  (void)gyro;
  return 1;
}
int32_t adi_gyro_shutdown(adi_gyro_t gyro) {
  (void)gyro;
  return 1;
}
adi_potentiometer_t adi_potentiometer_init(uint8_t port) {
  (void)port;
  return 0;
}
adi_potentiometer_t adi_potentiometer_type_init(
    uint8_t port, adi_potentiometer_type_e_t potentiometer_type) {
  (void)port;
  (void)potentiometer_type;
  return 0;
}
double adi_potentiometer_get_angle(adi_potentiometer_t potentiometer) {
  (void)potentiometer;
  return 0.0;
}
adi_led_t adi_led_init(uint8_t port) {
  (void)port;
  return 0;
}
int32_t adi_led_clear_all(adi_led_t led, uint32_t* buffer,
                          uint32_t buffer_length) {
  (void)led;
  (void)buffer;
  (void)buffer_length;
  return 1;
}
int32_t adi_led_set(adi_led_t led, uint32_t* buffer, uint32_t buffer_length) {
  (void)led;
  (void)buffer;
  (void)buffer_length;
  return 1;
}
int32_t adi_led_set_all(adi_led_t led, uint32_t* buffer, uint32_t buffer_length,
                        uint32_t color) {
  (void)led;
  (void)buffer;
  (void)buffer_length;
  (void)color;
  return 1;
}
int32_t adi_led_set_pixel(adi_led_t led, uint32_t* buffer,
                          uint32_t buffer_length, uint32_t color,
                          uint32_t pixel_position) {
  (void)led;
  (void)buffer;
  (void)buffer_length;
  (void)color;
  (void)pixel_position;
  return 1;
}
int32_t adi_led_clear_pixel(adi_led_t led, uint32_t* buffer,
                            uint32_t buffer_length, uint32_t pixel_position) {
  (void)led;
  (void)buffer;
  (void)buffer_length;
  (void)pixel_position;
  return 1;
}

}  // namespace c
}  // namespace pros
}  // extern "C"
