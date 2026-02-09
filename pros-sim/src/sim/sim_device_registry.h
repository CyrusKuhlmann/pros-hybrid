#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace pros_sim {

struct MotorState {
  double position_deg = 0.0;
  double velocity_rpm = 0.0;
  int32_t voltage = 0;
};

struct ImuState {
  double heading = 0.0;
  double rotation = 0.0;
};

struct RotationState {
  int32_t position_cdeg = 0;
  int32_t velocity_dps = 0;
};

struct DistanceState {
  int32_t distance_mm = 2000;
};

struct ControllerState {
  int32_t left_x = 0;
  int32_t left_y = 0;
  int32_t right_x = 0;
  int32_t right_y = 0;
  int32_t buttons = 0;
};

class SimTransport;

class SimDeviceRegistry {
 public:
  static SimDeviceRegistry& instance();

  void connect(const std::string& host, uint16_t port);
  void disconnect();
  bool is_connected() const;

  // Motor API
  void set_motor_voltage(uint8_t port, int32_t voltage);
  MotorState get_motor_state(uint8_t port) const;

  // IMU API
  void reset_imu(uint8_t port);
  ImuState get_imu_state(uint8_t port) const;

  // Rotation API
  void reset_rotation(uint8_t port);
  RotationState get_rotation_state(uint8_t port) const;

  // Distance API
  DistanceState get_distance_state(uint8_t port) const;

  // Controller API
  ControllerState get_controller_state(bool is_partner) const;

  // State update from simulator
  void process_state_update(const std::string& json_state);

 private:
  SimDeviceRegistry();
  ~SimDeviceRegistry();

  std::unique_ptr<SimTransport> transport_;
  mutable std::mutex state_mutex_;

  std::unordered_map<uint8_t, MotorState> motors_;
  std::unordered_map<uint8_t, ImuState> imus_;
  std::unordered_map<uint8_t, RotationState> rotations_;
  std::unordered_map<uint8_t, DistanceState> distances_;

  ControllerState master_controller_;
  ControllerState partner_controller_;
};

}  // namespace pros_sim
