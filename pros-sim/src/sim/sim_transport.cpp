#include "sim_transport.h"

#include <cstring>
#include <iostream>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket close
using SOCKET = int;
#endif

namespace pros_sim {

TcpTransport::TcpTransport() : socket_(nullptr), connected_(false) {
#ifdef _WIN32
  WSADATA wsa_data;
  WSAStartup(MAKEWORD(2, 2), &wsa_data);
#endif
}

TcpTransport::~TcpTransport() {
  disconnect();
#ifdef _WIN32
  WSACleanup();
#endif
}

bool TcpTransport::connect(const std::string& host, uint16_t port) {
  SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock == INVALID_SOCKET) {
    std::cerr << "Failed to create socket\n";
    return false;
  }

  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr);

  if (::connect(sock, reinterpret_cast<sockaddr*>(&server_addr),
                sizeof(server_addr)) == SOCKET_ERROR) {
    std::cerr << "Failed to connect to simulator at " << host << ":" << port
              << "\n";
    closesocket(sock);
    return false;
  }

  socket_ = reinterpret_cast<void*>(static_cast<uintptr_t>(sock));
  connected_ = true;
  std::cout << "Connected to simulator at " << host << ":" << port << "\n";
  return true;
}

void TcpTransport::disconnect() {
  if (socket_) {
    closesocket(static_cast<SOCKET>(reinterpret_cast<uintptr_t>(socket_)));
    socket_ = nullptr;
  }
  connected_ = false;
}

bool TcpTransport::is_connected() const { return connected_; }

bool TcpTransport::send_json(const std::string& json) {
  if (!connected_) return false;

  std::string message = json + "\n";
  int result = send(static_cast<SOCKET>(reinterpret_cast<uintptr_t>(socket_)),
                    message.c_str(), static_cast<int>(message.size()), 0);
  return result != SOCKET_ERROR;
}

bool TcpTransport::receive_json(std::string& json) {
  if (!connected_) return false;

  char buffer[8192];
  int result = recv(static_cast<SOCKET>(reinterpret_cast<uintptr_t>(socket_)),
                    buffer, sizeof(buffer) - 1, 0);

  if (result <= 0) {
    connected_ = false;
    return false;
  }

  buffer[result] = '\0';
  json = buffer;
  return true;
}

}  // namespace pros_sim
