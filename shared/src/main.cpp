#include "main.h"

// Simple robot behavior: drive straight for 5 seconds, then display IMU heading

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Robot Initialized");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  // Create devices
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  pros::MotorGroup left_motors({1, -2, 3});
  pros::MotorGroup right_motors({-4, 5, -6});
  pros::Imu imu(7);

  pros::lcd::set_text(0, "Starting in 1s...");
  pros::delay(1000);

  // Drive straight for 5 seconds
  pros::lcd::set_text(0, "Driving straight...");
  left_motors.move(100);
  right_motors.move(100);
  pros::delay(5000);

  // Stop
  left_motors.move(0);
  right_motors.move(0);
  pros::lcd::set_text(0, "Stopped");

  // Main loop: display IMU heading
  while (true) {
    double heading = imu.get_heading();
    pros::lcd::print(1, "IMU Heading: %.2f", heading);

    // Also respond to controller
    int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    left_motors.move(dir - turn);
    right_motors.move(dir + turn);

    pros::delay(20);
  }
}
