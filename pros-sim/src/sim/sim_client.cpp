/**
 * SimClient implementation - TCP client for the VEX Python simulator.
 *
 * Handles:
 *   - Socket connection (cross-platform: Winsock on Windows, POSIX elsewhere)
 *   - Background receive thread parsing newline-delimited JSON state updates
 *   - Sending JSON command messages
 *   - Minimal recursive-descent JSON parser
 */
#include "sim/sim_client.h"

#include <cstring>
#include <iostream>
#include <sstream>

 // ── Platform socket abstraction ──────────────────────────────────────────────

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
// Link against Ws2_32.lib (also added in CMakeLists.txt)
#pragma comment(lib, "Ws2_32.lib")

using socket_t = SOCKET;
static constexpr socket_t INVALID_SOCK = INVALID_SOCKET;

static bool platform_init() {
  WSADATA wsa;
  return WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
}
static void platform_cleanup() { WSACleanup(); }
static void close_socket(socket_t s) { closesocket(s); }
static int socket_errno() { return WSAGetLastError(); }

#else  // POSIX
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

using socket_t = int;
static constexpr socket_t INVALID_SOCK = -1;

static bool platform_init() { return true; }
static void platform_cleanup() {}
static void close_socket(socket_t s) { ::close(s); }
static int socket_errno() { return errno; }
#endif

// ─── JSON helpers ────────────────────────────────────────────────────────────
// Simple JSON string builder for commands.

static std::string json_cmd(const std::string& type, int port) {
  return "{\"type\":\"" + type + "\",\"port\":" + std::to_string(port) + "}\n";
}

static std::string json_cmd_int(const std::string& type, int port,
  const std::string& key, int value) {
  return "{\"type\":\"" + type + "\",\"port\":" + std::to_string(port) +
    ",\"" + key + "\":" + std::to_string(value) + "}\n";
}

static std::string json_cmd_str(const std::string& type, int port,
  const std::string& key,
  const std::string& value) {
  return "{\"type\":\"" + type + "\",\"port\":" + std::to_string(port) +
    ",\"" + key + "\":\"" + value + "\"}\n";
}

static std::string json_cmd_pos_vel(const std::string& type, int port,
  double position, int velocity) {
  std::ostringstream oss;
  oss << "{\"type\":\"" << type << "\",\"port\":" << port
    << ",\"position\":" << position
    << ",\"velocity\":" << velocity << "}\n";
  return oss.str();
}

// ─── JsonValue implementation ────────────────────────────────────────────────

namespace sim {

  static const JsonValue NULL_JSON;

  void JsonValue::skip_ws(const char*& p) {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
  }

  std::string JsonValue::parse_raw_string(const char*& p) {
    if (*p != '"') return "";
    ++p;  // skip opening "
    std::string result;
    while (*p && *p != '"') {
      if (*p == '\\') {
        ++p;
        switch (*p) {
        case '"': result += '"'; break;
        case '\\': result += '\\'; break;
        case '/': result += '/'; break;
        case 'n': result += '\n'; break;
        case 'r': result += '\r'; break;
        case 't': result += '\t'; break;
        default: result += *p; break;
        }
      }
      else {
        result += *p;
      }
      ++p;
    }
    if (*p == '"') ++p;  // skip closing "
    return result;
  }

  JsonValue JsonValue::parse_string_val(const char*& p) {
    JsonValue v;
    v.type_ = String;
    v.str_ = parse_raw_string(p);
    return v;
  }

  JsonValue JsonValue::parse_number(const char*& p) {
    JsonValue v;
    v.type_ = Number;
    char* end = nullptr;
    v.num_ = std::strtod(p, &end);
    p = end;
    return v;
  }

  JsonValue JsonValue::parse_array(const char*& p) {
    JsonValue v;
    v.type_ = Array;
    ++p;  // skip [
    skip_ws(p);
    if (*p == ']') { ++p; return v; }
    while (*p) {
      v.arr_.push_back(parse_value(p));
      skip_ws(p);
      if (*p == ',') { ++p; skip_ws(p); }
      else if (*p == ']') { ++p; break; }
    }
    return v;
  }

  JsonValue JsonValue::parse_object(const char*& p) {
    JsonValue v;
    v.type_ = Object;
    ++p;  // skip {
    skip_ws(p);
    if (*p == '}') { ++p; return v; }
    while (*p) {
      skip_ws(p);
      std::string key = parse_raw_string(p);
      skip_ws(p);
      if (*p == ':') ++p;
      skip_ws(p);
      v.obj_[key] = parse_value(p);
      skip_ws(p);
      if (*p == ',') { ++p; }
      else if (*p == '}') { ++p; break; }
    }
    return v;
  }

  JsonValue JsonValue::parse_value(const char*& p) {
    skip_ws(p);
    if (*p == '{') return parse_object(p);
    if (*p == '[') return parse_array(p);
    if (*p == '"') return parse_string_val(p);
    if (*p == 't') {  // true
      p += 4;
      JsonValue v; v.type_ = Bool; v.bool_ = true; return v;
    }
    if (*p == 'f') {  // false
      p += 5;
      JsonValue v; v.type_ = Bool; v.bool_ = false; return v;
    }
    if (*p == 'n') {  // null
      p += 4;
      return JsonValue{};
    }
    // Must be a number
    return parse_number(p);
  }

  JsonValue JsonValue::parse(const std::string& text) {
    const char* p = text.c_str();
    return parse_value(p);
  }

  bool JsonValue::has(const std::string& key) const {
    return obj_.count(key) > 0;
  }

  const JsonValue& JsonValue::operator[](const std::string& key) const {
    auto it = obj_.find(key);
    return it != obj_.end() ? it->second : NULL_JSON;
  }

  const JsonValue& JsonValue::operator[](std::size_t index) const {
    return index < arr_.size() ? arr_[index] : NULL_JSON;
  }

  // ─── SimClient implementation ────────────────────────────────────────────────

  SimClient& SimClient::instance() {
    static SimClient inst;
    return inst;
  }

  SimClient::~SimClient() { disconnect(); }

  bool SimClient::connect(const std::string& host, int port) {
    if (connected_.load()) return true;

    if (!platform_init()) {
      std::cerr << "[SimClient] Platform socket init failed\n";
      return false;
    }

    socket_t s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCK) {
      std::cerr << "[SimClient] socket() failed: " << socket_errno() << "\n";
      return false;
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<unsigned short>(port));
    inet_pton(AF_INET, host.c_str(), &addr.sin_addr);

    if (::connect(s, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) != 0) {
      std::cerr << "[SimClient] connect() to " << host << ":" << port
        << " failed: " << socket_errno() << "\n";
      close_socket(s);
      return false;
    }

    sock_ = static_cast<std::uintptr_t>(s);
    connected_.store(true);
    running_.store(true);

    // Start background receive thread
    recv_thread_ = std::thread(&SimClient::receive_loop, this);

    std::cout << "[SimClient] Connected to simulator at " << host << ":" << port
      << "\n";
    return true;
  }

  void SimClient::disconnect() {
    if (!connected_.load()) return;

    running_.store(false);
    connected_.store(false);

    // Send disconnect message (best-effort)
    send_raw("{\"type\":\"disconnect\",\"port\":0}\n");

    close_socket(static_cast<socket_t>(sock_));
    sock_ = static_cast<std::uintptr_t>(INVALID_SOCK);

    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }

    platform_cleanup();
    std::cout << "[SimClient] Disconnected from simulator\n";
  }

  void SimClient::send_raw(const std::string& msg) {
    if (!connected_.load()) return;
    socket_t s = static_cast<socket_t>(sock_);
    ::send(s, msg.c_str(), static_cast<int>(msg.size()), 0);
  }

  void SimClient::receive_loop() {
    socket_t s = static_cast<socket_t>(sock_);
    std::string buffer;
    char chunk[4096];

    while (running_.load()) {
      int n = ::recv(s, chunk, sizeof(chunk) - 1, 0);
      if (n <= 0) {
        if (running_.load()) {
          std::cerr << "[SimClient] Connection lost\n";
          connected_.store(false);
        }
        break;
      }
      chunk[n] = '\0';
      buffer.append(chunk, n);

      // Process complete lines
      std::size_t pos;
      while ((pos = buffer.find('\n')) != std::string::npos) {
        std::string line = buffer.substr(0, pos);
        buffer.erase(0, pos + 1);
        if (!line.empty()) {
          process_line(line);
        }
      }
    }
  }

  void SimClient::process_line(const std::string& line) {
    try {
      JsonValue root = JsonValue::parse(line);
      if (root.type() != JsonValue::Object) return;

      const std::string& msg_type = root["type"].string();

      if (msg_type == "handshake") {
        std::cout << "[SimClient] Handshake received (version: "
          << root["version"].string() << ")\n";
        return;
      }

      if (msg_type != "state_update") return;

      std::lock_guard<std::mutex> lock(state_mutex_);

      timestamp_ms_ = static_cast<std::uint32_t>(root["timestamp_ms"].number());

      // Parse motors
      if (root.has("motors")) {
        for (auto& [key, val] : root["motors"].items()) {
          int port = std::stoi(key);
          MotorState& ms = motors_[port];
          ms.position_deg = val["position_deg"].number();
          ms.velocity_rpm = val["velocity_rpm"].number();
          ms.torque_nm = val["torque_nm"].number();
          ms.temp_c = val["temp_c"].number();
          ms.current_ma = val["current_ma"].integer();
          ms.voltage = val["voltage"].integer();
          if (val.has("target_position_deg"))
            ms.target_position_deg = val["target_position_deg"].number();
          if (val.has("target_velocity_rpm"))
            ms.target_velocity_rpm = val["target_velocity_rpm"].integer();
        }
      }

      // Parse IMUs
      if (root.has("imus")) {
        for (auto& [key, val] : root["imus"].items()) {
          int port = std::stoi(key);
          ImuState& is = imus_[port];
          is.heading = val["heading"].number();
          is.rotation = val["rotation"].number();
          is.pitch = val["pitch"].number();
          is.roll = val["roll"].number();
          is.yaw = val["yaw"].number();
        }
      }

      // Parse rotation sensors
      if (root.has("rotation")) {
        for (auto& [key, val] : root["rotation"].items()) {
          int port = std::stoi(key);
          RotationState& rs = rotations_[port];
          rs.position_cdeg = val["position_cdeg"].integer();
          rs.velocity_dps = val["velocity_dps"].number();
        }
      }

      // Parse distance sensors
      if (root.has("distance")) {
        for (auto& [key, val] : root["distance"].items()) {
          int port = std::stoi(key);
          DistanceState& ds = distances_[port];
          ds.distance_mm = val["distance_mm"].integer();
        }
      }

      // Parse controllers
      if (root.has("controller")) {
        for (auto& [key, val] : root["controller"].items()) {
          ControllerState& cs = controllers_[key];
          cs.left_x = val["left_x"].integer();
          cs.left_y = val["left_y"].integer();
          cs.right_x = val["right_x"].integer();
          cs.right_y = val["right_y"].integer();
          cs.buttons = val["buttons"].integer();
        }
      }

    }
    catch (const std::exception& e) {
      // Silently ignore parse errors to avoid spamming
      (void)e;
    }
  }

  // ── Command senders ──────────────────────────────────────────────────────────

  void SimClient::send_motor_move(int port, int voltage) {
    send_raw(json_cmd_int("motor_move", port, "voltage", voltage));
  }

  void SimClient::send_motor_move_velocity(int port, int velocity) {
    send_raw(json_cmd_int("motor_move_velocity", port, "velocity", velocity));
  }

  void SimClient::send_motor_move_absolute(int port, double position,
    int velocity) {
    send_raw(json_cmd_pos_vel("motor_move_absolute", port, position, velocity));
  }

  void SimClient::send_motor_move_relative(int port, double position,
    int velocity) {
    send_raw(json_cmd_pos_vel("motor_move_relative", port, position, velocity));
  }

  void SimClient::send_motor_brake(int port) {
    send_raw(json_cmd("motor_brake", port));
  }

  void SimClient::send_motor_set_brake_mode(int port, const std::string& mode) {
    send_raw(json_cmd_str("motor_set_brake_mode", port, "mode", mode));
  }

  void SimClient::send_motor_tare_position(int port) {
    send_raw(json_cmd("motor_tare_position", port));
  }

  void SimClient::send_imu_reset(int port) {
    send_raw(json_cmd("imu_reset", port));
  }

  void SimClient::send_rotation_reset(int port) {
    send_raw(json_cmd("rotation_reset", port));
  }

  void SimClient::send_rotation_set_position(int port, int position_cdeg) {
    send_raw(
      json_cmd_int("rotation_set_position", port, "position", position_cdeg));
  }

  // ── State getters ────────────────────────────────────────────────────────────

  MotorState SimClient::get_motor_state(int port) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = motors_.find(port);
    return it != motors_.end() ? it->second : MotorState{};
  }

  ImuState SimClient::get_imu_state(int port) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = imus_.find(port);
    return it != imus_.end() ? it->second : ImuState{};
  }

  RotationState SimClient::get_rotation_state(int port) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = rotations_.find(port);
    return it != rotations_.end() ? it->second : RotationState{};
  }

  DistanceState SimClient::get_distance_state(int port) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = distances_.find(port);
    return it != distances_.end() ? it->second : DistanceState{};
  }

  ControllerState SimClient::get_controller_state(const std::string& id) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = controllers_.find(id);
    return it != controllers_.end() ? it->second : ControllerState{};
  }

  std::uint32_t SimClient::get_timestamp_ms() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return timestamp_ms_;
  }

}  // namespace sim
