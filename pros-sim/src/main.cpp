/**
 * Hello World program demonstrating the PROS simulation library.
 * Connects to the Python VEX simulator via TCP, then runs user code.
 */
#include "main.h"

#include <iostream>
#include <thread>
#include <chrono>

#include "sim/sim_client.h"

 // Implement the PROS entry-point functions as required by main.h
void initialize() {
  std::cout << "[initialize] Starting..." << std::endl;
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Hello from PROS Sim!");
  std::cout << "[initialize] LCD initialized." << std::endl;
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  std::cout << "[opcontrol] Starting..." << std::endl;
  pros::Motor left_motor(1);
  pros::Motor right_motor(-2);

  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true) {
    int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int right_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    left_motor.move(left_y);
    right_motor.move(right_y);

    // Print telemetry periodically
    static int tick = 0;
    if (tick % 50 == 0) {
      auto m1 = sim::SimClient::instance().get_motor_state(1);
      auto m2 = sim::SimClient::instance().get_motor_state(2);
      std::cout << "[opcontrol] L_Y=" << left_y << " R_Y=" << right_y
        << " | M1 pos==" << m1.position_deg << " vel==" << m1.velocity_rpm
        << " | M2 pos==" << m2.position_deg << " vel==" << m2.velocity_rpm
        << std::endl;
    }
    tick++;

    pros::delay(20);
  }
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
    opcontrol();
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
