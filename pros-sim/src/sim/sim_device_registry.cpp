#include "sim_device_registry.h"

#include <iostream>
#include <sstream>
#include <thread>

#include "sim_transport.h"

namespace pros_sim {

SimDeviceRegistry& SimDeviceRegistry::instance() {
  static SimDeviceRegistry instance;
  return instance;
}

SimDeviceRegistry::SimDeviceRegistry()
    : transport_(std::make_unique<TcpTransport>()) {}

SimDeviceRegistry::~SimDeviceRegistry() { disconnect(); }

void SimDeviceRegistry::connect(const std::string& host, uint16_t port) {
  if (transport_->connect(host, port)) {
    // Start background thread to receive state updates
    std::thread([this]() {
      std::string json;
      while (transport_->is_connected()) {
        if (transport_->receive_json(json)) {
          process_state_update(json);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }).detach();
  }
}

void SimDeviceRegistry::disconnect() { transport_->disconnect(); }

bool SimDeviceRegistry::is_connected() const {
  return transport_->is_connected();
}

void SimDeviceRegistry::set_motor_voltage(uint8_t port, int32_t voltage) {
  std::ostringstream oss;
  oss << "{\"type\":\"motor_move\",\"port\":" << static_cast<int>(port)
      << ",\"voltage\":" << voltage << "}";
  transport_->send_json(oss.str());
}

MotorState SimDeviceRegistry::get_motor_state(uint8_t port) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto it = motors_.find(port);
  return (it != motors_.end()) ? it->second : MotorState{};
}

void SimDeviceRegistry::reset_imu(uint8_t port) {
  std::ostringstream oss;
  oss << "{\"type\":\"imu_reset\",\"port\":" << static_cast<int>(port) << "}";
  transport_->send_json(oss.str());
}

ImuState SimDeviceRegistry::get_imu_state(uint8_t port) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto it = imus_.find(port);
  return (it != imus_.end()) ? it->second : ImuState{};
}

void SimDeviceRegistry::reset_rotation(uint8_t port) {
  std::ostringstream oss;
  oss << "{\"type\":\"rotation_reset\",\"port\":" << static_cast<int>(port)
      << "}";
  transport_->send_json(oss.str());
}

RotationState SimDeviceRegistry::get_rotation_state(uint8_t port) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto it = rotations_.find(port);
  return (it != rotations_.end()) ? it->second : RotationState{};
}

DistanceState SimDeviceRegistry::get_distance_state(uint8_t port) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto it = distances_.find(port);
  return (it != distances_.end()) ? it->second : DistanceState{};
}

ControllerState SimDeviceRegistry::get_controller_state(bool is_partner) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return is_partner ? partner_controller_ : master_controller_;
}

// Simple JSON parser for state updates
void SimDeviceRegistry::process_state_update(const std::string& json) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Parse motors
  size_t motors_pos = json.find("\"motors\":");
  if (motors_pos != std::string::npos) {
    size_t start = json.find('{', motors_pos);
    size_t end = json.find('}', start);
    std::string motors_section = json.substr(start, end - start + 1);

    // Parse each motor entry
    size_t pos = 0;
    while ((pos = motors_section.find('\"', pos)) != std::string::npos) {
      pos++;
      size_t port_end = motors_section.find('\"', pos);
      if (port_end == std::string::npos) break;

      uint8_t port = static_cast<uint8_t>(
          std::stoi(motors_section.substr(pos, port_end - pos)));

      // Extract motor state values
      MotorState state;
      size_t val_start = motors_section.find('{', port_end);
      size_t val_end = motors_section.find('}', val_start);
      std::string values =
          motors_section.substr(val_start, val_end - val_start + 1);

      if (auto p = values.find("\"position_deg\":"); p != std::string::npos) {
        state.position_deg = std::stod(values.substr(p + 15));
      }
      if (auto p = values.find("\"velocity_rpm\":"); p != std::string::npos) {
        state.velocity_rpm = std::stod(values.substr(p + 15));
      }
      if (auto p = values.find("\"voltage\":"); p != std::string::npos) {
        state.voltage = std::stoi(values.substr(p + 10));
      }

      motors_[port] = state;
      pos = val_end;
    }
  }

  // Parse IMUs
  size_t imus_pos = json.find("\"imus\":");
  if (imus_pos != std::string::npos) {
    size_t start = json.find('{', imus_pos);
    size_t end = json.find('}', start);
    std::string imus_section = json.substr(start, end - start + 1);

    size_t pos = 0;
    while ((pos = imus_section.find('\"', pos)) != std::string::npos) {
      pos++;
      size_t port_end = imus_section.find('\"', pos);
      if (port_end == std::string::npos) break;

      uint8_t port = static_cast<uint8_t>(
          std::stoi(imus_section.substr(pos, port_end - pos)));

      ImuState state;
      size_t val_start = imus_section.find('{', port_end);
      size_t val_end = imus_section.find('}', val_start);
      std::string values =
          imus_section.substr(val_start, val_end - val_start + 1);

      if (auto p = values.find("\"heading\":"); p != std::string::npos) {
        state.heading = std::stod(values.substr(p + 10));
      }
      if (auto p = values.find("\"rotation\":"); p != std::string::npos) {
        state.rotation = std::stod(values.substr(p + 11));
      }

      imus_[port] = state;
      pos = val_end;
    }
  }

  // Parse controller master
  size_t ctrl_pos = json.find("\"master\":");
  if (ctrl_pos != std::string::npos) {
    size_t start = json.find('{', ctrl_pos);
    size_t end = json.find('}', start);
    std::string ctrl_section = json.substr(start, end - start + 1);

    if (auto p = ctrl_section.find("\"left_x\":"); p != std::string::npos) {
      master_controller_.left_x = std::stoi(ctrl_section.substr(p + 9));
    }
    if (auto p = ctrl_section.find("\"left_y\":"); p != std::string::npos) {
      master_controller_.left_y = std::stoi(ctrl_section.substr(p + 9));
    }
    if (auto p = ctrl_section.find("\"right_x\":"); p != std::string::npos) {
      master_controller_.right_x = std::stoi(ctrl_section.substr(p + 10));
    }
    if (auto p = ctrl_section.find("\"right_y\":"); p != std::string::npos) {
      master_controller_.right_y = std::stoi(ctrl_section.substr(p + 10));
    }
  }
}

}  // namespace pros_sim
