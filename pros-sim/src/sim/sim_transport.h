#pragma once

#include <cstdint>
#include <string>

namespace pros_sim {

class SimTransport {
 public:
  virtual ~SimTransport() = default;

  virtual bool connect(const std::string& host, uint16_t port) = 0;
  virtual void disconnect() = 0;
  virtual bool is_connected() const = 0;

  virtual bool send_json(const std::string& json) = 0;
  virtual bool receive_json(std::string& json) = 0;
};

// TCP implementation for Windows
class TcpTransport : public SimTransport {
 public:
  TcpTransport();
  ~TcpTransport() override;

  bool connect(const std::string& host, uint16_t port) override;
  void disconnect() override;
  bool is_connected() const override;

  bool send_json(const std::string& json) override;
  bool receive_json(std::string& json) override;

 private:
  void*
      socket_;  // SOCKET on Windows (using void* to avoid windows.h in header)
  bool connected_;
};

}  // namespace pros_sim
