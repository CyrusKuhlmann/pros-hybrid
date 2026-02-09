#include <iostream>

#include "../pros-sim/src/sim/sim_device_registry.h"
#include "main.h"

// Entry point that connects to simulator and runs robot code
int main(int argc, char* argv[]) {
  std::string sim_host = "127.0.0.1";
  uint16_t sim_port = 9090;

  // Parse command line args
  if (argc > 1) {
    sim_host = argv[1];
  }
  if (argc > 2) {
    sim_port = static_cast<uint16_t>(std::atoi(argv[2]));
  }

  std::cout << "Connecting to simulator at " << sim_host << ":" << sim_port
            << "...\n";

  // Connect to simulator
  pros_sim::SimDeviceRegistry::instance().connect(sim_host, sim_port);

  if (!pros_sim::SimDeviceRegistry::instance().is_connected()) {
    std::cerr << "Failed to connect to simulator. Is it running?\n";
    return 1;
  }

  std::cout << "Connected! Starting robot code...\n";

  // Run robot initialization
  initialize();
  competition_initialize();

  // Run opcontrol (or autonomous based on command)
  opcontrol();

  // Cleanup
  pros_sim::SimDeviceRegistry::instance().disconnect();
  return 0;
}
