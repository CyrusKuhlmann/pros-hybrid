/**
 * Hello World program demonstrating the PROS simulation library.
 * This compiles on desktop (Windows/Mac/Linux) using the stubbed PROS API.
 */
#include "main.h"

#include <iostream>

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

  int count = 0;
  while (count < 5) {
    int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int right_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    left_motor.move(left_y);
    right_motor.move(right_y);

    std::cout << "[opcontrol] tick " << count << " | left_y=" << left_y
      << " right_y=" << right_y << std::endl;

    pros::delay(20);
    count++;
  }
  std::cout << "[opcontrol] Done." << std::endl;
}

int main() {
  std::cout << "=== PROS Simulation - Hello World ===" << std::endl;
  try {
    initialize();
    opcontrol();
  }
  catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }
  catch (...) {
    std::cerr << "Unknown exception!" << std::endl;
    return 1;
  }
  std::cout << "=== Finished ===" << std::endl;
  return 0;
}
