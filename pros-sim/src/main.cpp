/**
 * PROS simulation demo — uses only raw motor voltage commands.
 * All higher-level control (PID, etc.) belongs in this user code,
 * not in the simulator.
 */
#include "main.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "api.h"
#include "sim/sim_client.h"

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Hello from PROS Sim!");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {

}

void opcontrol() {
  std::cout << "[opcontrol] Starting..." << std::endl;

  // Left drive motors (ports 1-3), right drive motors (ports 4-6)
  pros::Motor left1(1);
  pros::Motor left2(2);
  pros::Motor left3(3);
  pros::Motor right1(4);
  pros::Motor right2(5);
  pros::Motor right3(6);

  // Set brake mode to coast (the default)
  left1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  left2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  left3.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  right1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  right2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  right3.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  pros::Controller master(pros::E_CONTROLLER_MASTER);

  int tick = 0;
  while (true) {
    // Tank drive: raw joystick → raw voltage
    int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int right_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    left1.move(left_y);
    left2.move(left_y);
    left3.move(left_y);
    right1.move(right_y);
    right2.move(right_y);
    right3.move(right_y);

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
