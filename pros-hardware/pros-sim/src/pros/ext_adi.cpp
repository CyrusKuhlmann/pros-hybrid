/**
 * PROS Extended ADI C API stub implementations for simulation
 */
#include "pros/ext_adi.h"

extern "C" {
  namespace pros {
    namespace c {

      adi_port_config_e_t ext_adi_port_get_config(uint8_t smart_port,
        uint8_t adi_port) {
        (void)smart_port;
        (void)adi_port;
        return E_ADI_TYPE_UNDEFINED;
      }
      int32_t ext_adi_port_get_value(uint8_t smart_port, uint8_t adi_port) {
        (void)smart_port;
        (void)adi_port;
        return 0;
      }
      int32_t ext_adi_port_set_config(uint8_t smart_port, uint8_t adi_port,
        adi_port_config_e_t type) {
        (void)smart_port;
        (void)adi_port;
        (void)type;
        return 1;
      }
      int32_t ext_adi_port_set_value(uint8_t smart_port, uint8_t adi_port,
        int32_t value) {
        (void)smart_port;
        (void)adi_port;
        (void)value;
        return 1;
      }
      int32_t ext_adi_analog_calibrate(uint8_t smart_port, uint8_t adi_port) {
        (void)smart_port;
        (void)adi_port;
        return 0;
      }
      int32_t ext_adi_analog_read(uint8_t smart_port, uint8_t adi_port) {
        (void)smart_port;
        (void)adi_port;
        return 0;
      }
      int32_t ext_adi_analog_read_calibrated(uint8_t smart_port, uint8_t adi_port) {
        (void)smart_port;
        (void)adi_port;
        return 0;
      }
      int32_t ext_adi_analog_read_calibrated_HR(uint8_t smart_port,
        uint8_t adi_port) {
        (void)smart_port;
        (void)adi_port;
        return 0;
      }
      int32_t ext_adi_digital_read(uint8_t smart_port, uint8_t adi_port) {
        (void)smart_port;
        (void)adi_port;
        return 0;
      }
      int32_t ext_adi_digital_get_new_press(uint8_t smart_port, uint8_t adi_port) {
        (void)smart_port;
        (void)adi_port;
        return 0;
      }
      int32_t ext_adi_digital_write(uint8_t smart_port, uint8_t adi_port,
        bool value) {
        (void)smart_port;
        (void)adi_port;
        (void)value;
        return 1;
      }
      int32_t ext_adi_pin_mode(uint8_t smart_port, uint8_t adi_port, uint8_t mode) {
        (void)smart_port;
        (void)adi_port;
        (void)mode;
        return 1;
      }
      int32_t ext_adi_motor_set(uint8_t smart_port, uint8_t adi_port, int8_t speed) {
        (void)smart_port;
        (void)adi_port;
        (void)speed;
        return 1;
      }
      int32_t ext_adi_motor_get(uint8_t smart_port, uint8_t adi_port) {
        (void)smart_port;
        (void)adi_port;
        return 0;
      }
      int32_t ext_adi_motor_stop(uint8_t smart_port, uint8_t adi_port) {
        (void)smart_port;
        (void)adi_port;
        return 1;
      }
      int32_t ext_adi_encoder_get(ext_adi_encoder_t enc) {
        (void)enc;
        return 0;
      }
      ext_adi_encoder_t ext_adi_encoder_init(uint8_t smart_port, uint8_t adi_port_top,
        uint8_t adi_port_bottom, bool reverse) {
        (void)smart_port;
        (void)adi_port_top;
        (void)adi_port_bottom;
        (void)reverse;
        return 0;
      }
      int32_t ext_adi_encoder_reset(ext_adi_encoder_t enc) {
        (void)enc;
        return 1;
      }
      int32_t ext_adi_encoder_shutdown(ext_adi_encoder_t enc) {
        (void)enc;
        return 1;
      }
      int32_t ext_adi_ultrasonic_get(ext_adi_ultrasonic_t ult) {
        (void)ult;
        return 0;
      }
      ext_adi_ultrasonic_t ext_adi_ultrasonic_init(uint8_t smart_port,
        uint8_t adi_port_ping,
        uint8_t adi_port_echo) {
        (void)smart_port;
        (void)adi_port_ping;
        (void)adi_port_echo;
        return 0;
      }
      int32_t ext_adi_ultrasonic_shutdown(ext_adi_ultrasonic_t ult) {
        (void)ult;
        return 1;
      }
      double ext_adi_gyro_get(ext_adi_gyro_t gyro) {
        (void)gyro;
        return 0.0;
      }
      ext_adi_gyro_t ext_adi_gyro_init(uint8_t smart_port, uint8_t adi_port,
        double multiplier) {
        (void)smart_port;
        (void)adi_port;
        (void)multiplier;
        return 0;
      }
      int32_t ext_adi_gyro_reset(ext_adi_gyro_t gyro) {
        (void)gyro;
        return 1;
      }
      int32_t ext_adi_gyro_shutdown(ext_adi_gyro_t gyro) {
        (void)gyro;
        return 1;
      }
      ext_adi_potentiometer_t ext_adi_potentiometer_init(
        uint8_t smart_port, uint8_t adi_port,
        adi_potentiometer_type_e_t potentiometer_type) {
        (void)smart_port;
        (void)adi_port;
        (void)potentiometer_type;
        return 0;
      }
      double ext_adi_potentiometer_get_angle(ext_adi_potentiometer_t potentiometer) {
        (void)potentiometer;
        return 0.0;
      }
      ext_adi_led_t ext_adi_led_init(uint8_t smart_port, uint8_t adi_port) {
        (void)smart_port;
        (void)adi_port;
        return 0;
      }
      int32_t ext_adi_led_clear_all(ext_adi_led_t led, uint32_t* buffer,
        uint32_t buffer_length) {
        (void)led;
        (void)buffer;
        (void)buffer_length;
        return 1;
      }
      int32_t ext_adi_led_set(ext_adi_led_t led, uint32_t* buffer,
        uint32_t buffer_length) {
        (void)led;
        (void)buffer;
        (void)buffer_length;
        return 1;
      }
      int32_t ext_adi_led_set_all(ext_adi_led_t led, uint32_t* buffer,
        uint32_t buffer_length, uint32_t color) {
        (void)led;
        (void)buffer;
        (void)buffer_length;
        (void)color;
        return 1;
      }
      int32_t ext_adi_led_set_pixel(ext_adi_led_t led, uint32_t* buffer,
        uint32_t buffer_length, uint32_t color,
        uint32_t pixel_position) {
        (void)led;
        (void)buffer;
        (void)buffer_length;
        (void)color;
        (void)pixel_position;
        return 1;
      }
      int32_t ext_adi_led_clear_pixel(ext_adi_led_t led, uint32_t* buffer,
        uint32_t buffer_length,
        uint32_t pixel_position) {
        (void)led;
        (void)buffer;
        (void)buffer_length;
        (void)pixel_position;
        return 1;
      }

    }  // namespace c
  }  // namespace pros
}  // extern "C"
