/**
 * SimClient - TCP client that connects to the Python VEX simulator.
 *
 * Singleton that manages the connection, sends commands as newline-delimited
 * JSON, and caches the periodic state updates from the simulator for reading
 * by the PROS stub implementations.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sim {

  // ─── Cached device state structures ──────────────────────────────────────────

  struct MotorState {
    double position_deg = 0;
    double velocity_rpm = 0;
    double torque_nm = 0;
    double temp_c = 25.0;
    int current_ma = 0;
    int voltage = 0;
    double target_position_deg = 0;
    int target_velocity_rpm = 0;
  };

  struct ImuState {
    double heading = 0;
    double rotation = 0;
    double pitch = 0;
    double roll = 0;
    double yaw = 0;
  };

  struct RotationState {
    int position_cdeg = 0;
    double velocity_dps = 0;
  };

  struct DistanceState {
    int distance_mm = 0;
  };

  struct ControllerState {
    int left_x = 0;
    int left_y = 0;
    int right_x = 0;
    int right_y = 0;
    int buttons = 0;
  };

  // ─── Minimal JSON value (used internally for parsing state updates) ──────────

  class JsonValue {
  public:
    enum Type { Null, Bool, Number, String, Array, Object };

    Type type() const { return type_; }

    double number() const { return num_; }
    int integer() const { return static_cast<int>(num_); }
    bool boolean() const { return bool_; }
    const std::string& string() const { return str_; }

    bool has(const std::string& key) const;
    const JsonValue& operator[](const std::string& key) const;
    const JsonValue& operator[](std::size_t index) const;
    const std::map<std::string, JsonValue>& items() const { return obj_; }

    static JsonValue parse(const std::string& text);

  private:
    Type type_ = Null;
    double num_ = 0;
    bool bool_ = false;
    std::string str_;
    std::vector<JsonValue> arr_;
    std::map<std::string, JsonValue> obj_;

    static JsonValue parse_value(const char*& p);
    static JsonValue parse_object(const char*& p);
    static JsonValue parse_array(const char*& p);
    static JsonValue parse_string_val(const char*& p);
    static JsonValue parse_number(const char*& p);
    static std::string parse_raw_string(const char*& p);
    static void skip_ws(const char*& p);
  };

  // ─── SimClient singleton ─────────────────────────────────────────────────────

  class SimClient {
  public:
    static SimClient& instance();

    /// Connect to the simulator TCP server.  Returns true on success.
    bool connect(const std::string& host = "127.0.0.1", int port = 9090);

    /// Disconnect from the simulator.
    void disconnect();

    /// Check if connected.
    bool is_connected() const { return connected_.load(); }

    // ── Commands (client → simulator) ──────────────────────────────────────

    void send_motor_move(int port, int voltage);
    void send_motor_move_velocity(int port, int velocity);
    void send_motor_move_absolute(int port, double position, int velocity);
    void send_motor_move_relative(int port, double position, int velocity);
    void send_motor_brake(int port);
    void send_motor_set_brake_mode(int port, const std::string& mode);
    void send_motor_tare_position(int port);
    void send_imu_reset(int port);
    void send_rotation_reset(int port);
    void send_rotation_set_position(int port, int position_cdeg);

    // ── State getters (from cached state updates) ──────────────────────────

    MotorState get_motor_state(int port);
    ImuState get_imu_state(int port);
    RotationState get_rotation_state(int port);
    DistanceState get_distance_state(int port);
    ControllerState get_controller_state(const std::string& id);
    std::uint32_t get_timestamp_ms();

  private:
    SimClient() = default;
    ~SimClient();
    SimClient(const SimClient&) = delete;
    SimClient& operator=(const SimClient&) = delete;

    void receive_loop();
    void process_line(const std::string& line);
    void send_raw(const std::string& msg);

#ifdef _WIN32
    std::uintptr_t sock_ = ~static_cast<std::uintptr_t>(0);
#else
    int sock_ = -1;
#endif

    std::atomic<bool> connected_{ false };
    std::atomic<bool> running_{ false };
    std::thread recv_thread_;

    mutable std::mutex state_mutex_;
    std::map<int, MotorState> motors_;
    std::map<int, ImuState> imus_;
    std::map<int, RotationState> rotations_;
    std::map<int, DistanceState> distances_;
    std::map<std::string, ControllerState> controllers_;
    std::uint32_t timestamp_ms_ = 0;
  };

}  // namespace sim
