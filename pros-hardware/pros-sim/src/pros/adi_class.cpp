/**
 * ADI C++ class stub implementations for desktop simulation.
 * Implements: Port, AnalogIn, AnalogOut, DigitalOut, DigitalIn,
 *             Motor, Encoder, Ultrasonic, Gyro, Potentiometer, Led, Pneumatics
 */

#include <cstdint>
#include <iostream>

#include "pros/adi.hpp"

namespace pros {
namespace adi {

// ========================= Port =========================

Port::Port(std::uint8_t adi_port, adi_port_config_e_t type)
    : _smart_port(22), _adi_port(adi_port) {
  (void)type;
}

Port::Port(ext_adi_port_pair_t port_pair, adi_port_config_e_t type)
    : _smart_port(port_pair.first), _adi_port(port_pair.second) {
  (void)type;
}

std::int32_t Port::get_config() const { return 0; }

std::int32_t Port::get_value() const { return 0; }

std::int32_t Port::set_config(adi_port_config_e_t type) const {
  (void)type;
  return 1;
}

std::int32_t Port::set_value(std::int32_t value) const {
  (void)value;
  return 1;
}

ext_adi_port_tuple_t Port::get_port() const {
  return ext_adi_port_tuple_t{_smart_port, _adi_port, 0};
}

// ========================= AnalogIn =========================

AnalogIn::AnalogIn(std::uint8_t adi_port) : Port(adi_port, E_ADI_ANALOG_IN) {}

AnalogIn::AnalogIn(ext_adi_port_pair_t port_pair)
    : Port(port_pair, E_ADI_ANALOG_IN) {}

std::int32_t AnalogIn::calibrate() const { return 0; }

std::int32_t AnalogIn::get_value_calibrated() const { return 0; }

std::int32_t AnalogIn::get_value_calibrated_HR() const { return 0; }

std::ostream& operator<<(std::ostream& os, pros::adi::AnalogIn& analog_in) {
  auto port = analog_in.get_port();
  os << "AnalogIn [smart_port: " << (int)std::get<0>(port)
     << ", adi_port: " << (int)std::get<1>(port)
     << ", value calibrated: " << analog_in.get_value_calibrated()
     << ", value calibrated HR: " << analog_in.get_value_calibrated_HR()
     << ", value: " << analog_in.get_value() << "]";
  return os;
}

// ========================= AnalogOut =========================

AnalogOut::AnalogOut(std::uint8_t adi_port)
    : Port(adi_port, E_ADI_ANALOG_OUT) {}

AnalogOut::AnalogOut(ext_adi_port_pair_t port_pair)
    : Port(port_pair, E_ADI_ANALOG_OUT) {}

std::ostream& operator<<(std::ostream& os, pros::adi::AnalogOut& analog_out) {
  auto port = analog_out.get_port();
  os << "AnalogOut [smart_port: " << (int)std::get<0>(port)
     << ", adi_port: " << (int)std::get<1>(port)
     << ", value: " << analog_out.get_value() << "]";
  return os;
}

// ========================= DigitalOut =========================

DigitalOut::DigitalOut(std::uint8_t adi_port, bool init_state)
    : Port(adi_port, E_ADI_DIGITAL_OUT) {
  (void)init_state;
}

DigitalOut::DigitalOut(ext_adi_port_pair_t port_pair, bool init_state)
    : Port(port_pair, E_ADI_DIGITAL_OUT) {
  (void)init_state;
}

std::ostream& operator<<(std::ostream& os, pros::adi::DigitalOut& digital_out) {
  auto port = digital_out.get_port();
  os << "DigitalOut [smart_port: " << (int)std::get<0>(port)
     << ", adi_port: " << (int)std::get<1>(port)
     << ", value: " << digital_out.get_value() << "]";
  return os;
}

// ========================= DigitalIn =========================

DigitalIn::DigitalIn(std::uint8_t adi_port)
    : Port(adi_port, E_ADI_DIGITAL_IN) {}

DigitalIn::DigitalIn(ext_adi_port_pair_t port_pair)
    : Port(port_pair, E_ADI_DIGITAL_IN) {}

std::int32_t DigitalIn::get_new_press() const { return 0; }

std::ostream& operator<<(std::ostream& os, pros::adi::DigitalIn& digital_in) {
  auto port = digital_in.get_port();
  os << "DigitalIn [smart_port: " << (int)std::get<0>(port)
     << ", adi_port: " << (int)std::get<1>(port)
     << ", value: " << digital_in.get_value() << "]";
  return os;
}

// ========================= Motor (ADI) =========================

Motor::Motor(std::uint8_t adi_port) : Port(adi_port, E_ADI_LEGACY_PWM) {}

Motor::Motor(ext_adi_port_pair_t port_pair)
    : Port(port_pair, E_ADI_LEGACY_PWM) {}

std::int32_t Motor::stop() const { return 1; }

// ========================= Encoder =========================

Encoder::Encoder(std::uint8_t adi_port_top, std::uint8_t adi_port_bottom,
                 bool reversed)
    : Port(adi_port_top, E_ADI_LEGACY_ENCODER),
      _port_pair{22, adi_port_bottom} {
  (void)reversed;
}

Encoder::Encoder(ext_adi_port_tuple_t port_tuple, bool reversed)
    : Port(
          ext_adi_port_pair_t{std::get<0>(port_tuple), std::get<1>(port_tuple)},
          E_ADI_LEGACY_ENCODER),
      _port_pair{std::get<0>(port_tuple), std::get<2>(port_tuple)} {
  (void)reversed;
}

std::int32_t Encoder::reset() const { return 1; }

std::int32_t Encoder::get_value() const { return 0; }

ext_adi_port_tuple_t Encoder::get_port() const {
  return ext_adi_port_tuple_t{_smart_port, _adi_port, _port_pair.second};
}

std::ostream& operator<<(std::ostream& os, pros::adi::Encoder& encoder) {
  auto port = encoder.get_port();
  os << "Encoder [smart_port: " << (int)std::get<0>(port)
     << ", adi_port: " << (int)std::get<1>(port)
     << ", value: " << encoder.get_value() << "]";
  return os;
}

// ========================= Ultrasonic =========================

Ultrasonic::Ultrasonic(std::uint8_t adi_port_ping, std::uint8_t adi_port_echo)
    : Port(adi_port_ping, E_ADI_LEGACY_ULTRASONIC) {
  (void)adi_port_echo;
}

Ultrasonic::Ultrasonic(ext_adi_port_tuple_t port_tuple)
    : Port(
          ext_adi_port_pair_t{std::get<0>(port_tuple), std::get<1>(port_tuple)},
          E_ADI_LEGACY_ULTRASONIC) {}

std::int32_t Ultrasonic::get_value() const { return 0; }

// ========================= Gyro =========================

Gyro::Gyro(std::uint8_t adi_port, double multiplier)
    : Port(adi_port, E_ADI_LEGACY_GYRO) {
  (void)multiplier;
}

Gyro::Gyro(ext_adi_port_pair_t port_pair, double multiplier)
    : Port(port_pair, E_ADI_LEGACY_GYRO) {
  (void)multiplier;
}

double Gyro::get_value() const { return 0.0; }

std::int32_t Gyro::reset() const { return 1; }

// ========================= Potentiometer =========================

Potentiometer::Potentiometer(std::uint8_t adi_port,
                             adi_potentiometer_type_e_t potentiometer_type)
    : AnalogIn(adi_port) {
  (void)potentiometer_type;
}

Potentiometer::Potentiometer(ext_adi_port_pair_t port_pair,
                             adi_potentiometer_type_e_t potentiometer_type)
    : AnalogIn(port_pair) {
  (void)potentiometer_type;
}

double Potentiometer::get_angle() const { return 0.0; }

std::ostream& operator<<(std::ostream& os,
                         pros::adi::Potentiometer& potentiometer) {
  os << "Potentiometer [value: " << potentiometer.get_value()
     << ", value calibrated: " << potentiometer.get_value_calibrated()
     << ", angle: " << potentiometer.get_angle() << "]";
  return os;
}

// ========================= Led =========================

Led::Led(std::uint8_t adi_port, std::uint32_t length)
    : Port(adi_port, E_ADI_DIGITAL_OUT), _buffer(length, 0) {}

Led::Led(ext_adi_port_pair_t port_pair, std::uint32_t length)
    : Port(port_pair, E_ADI_DIGITAL_OUT), _buffer(length, 0) {}

std::uint32_t& Led::operator[](size_t i) { return _buffer[i]; }

std::int32_t Led::clear_all() {
  for (auto& pixel : _buffer) pixel = 0;
  return 1;
}

std::int32_t Led::clear() { return clear_all(); }

std::int32_t Led::update() const { return 1; }

std::int32_t Led::set_all(uint32_t color) {
  for (auto& pixel : _buffer) pixel = color;
  return 1;
}

std::int32_t Led::set_pixel(uint32_t color, uint32_t pixel_position) {
  if (pixel_position < _buffer.size()) {
    _buffer[pixel_position] = color;
  }
  return 1;
}

std::int32_t Led::clear_pixel(uint32_t pixel_position) {
  if (pixel_position < _buffer.size()) {
    _buffer[pixel_position] = 0;
  }
  return 1;
}

std::int32_t Led::length() { return static_cast<std::int32_t>(_buffer.size()); }

// ========================= Pneumatics =========================

Pneumatics::Pneumatics(std::uint8_t adi_port, bool start_extended,
                       bool extended_is_low_val)
    : DigitalOut(adi_port, start_extended != extended_is_low_val),
      state(start_extended),
      extended_is_low(extended_is_low_val) {}

Pneumatics::Pneumatics(ext_adi_port_pair_t port_pair, bool start_extended,
                       bool extended_is_low_val)
    : DigitalOut(port_pair, start_extended != extended_is_low_val),
      state(start_extended),
      extended_is_low(extended_is_low_val) {}

std::int32_t Pneumatics::extend() {
  if (state) return 0;
  state = true;
  return 1;
}

std::int32_t Pneumatics::retract() {
  if (!state) return 0;
  state = false;
  return 1;
}

std::int32_t Pneumatics::toggle() {
  state = !state;
  return 1;
}

bool Pneumatics::is_extended() const { return state; }

}  // namespace adi
}  // namespace pros
