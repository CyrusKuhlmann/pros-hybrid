/**
 * PROS simulation demo — uses only raw motor voltage commands.
 * All higher-level control (PID, etc.) belongs in this user code,
 * not in the simulator.
 */

#include <chrono>
#include <iostream>
#include <thread>

#include "api.h"
#include "sim/sim_client.h"

 // Forward-declare user entry points (defined in main.cpp)
extern "C" {
  void autonomous(void);
  void initialize(void);
  void disabled(void);
  void competition_initialize(void);
  void opcontrol(void);
}

int main() {
  std::cout << "=== PROS Simulation ===" << std::endl;

  // Connect to the Python simulator
  if (!sim::SimClient::instance().connect("127.0.0.1", 9090)) {
    std::cerr << "Failed to connect to simulator. "
      "Make sure the Python simulator is running:\n"
      "  cd ../vex-simulator && python -m vex_simulator\n";
    return 1;
  }

  // Give the handshake a moment to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  try {
    initialize();
    autonomous();
  }
  catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }
  catch (...) {
    std::cerr << "Unknown exception!" << std::endl;
  }

  sim::SimClient::instance().disconnect();
  std::cout << "=== Finished ===" << std::endl;
  return 0;
}
