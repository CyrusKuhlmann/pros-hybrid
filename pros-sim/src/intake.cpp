#include "intake.h"

// Constructor
Intake::Intake(pros::Motor& bottom_ref, pros::Motor& middle_ref, pros::Motor& top_ref, int default_speed)
    : bottomRoller(bottom_ref), middleRoller(middle_ref), topRoller(top_ref), speed(default_speed), is_running(false) {
  // Top roller is reversed
  topRoller.set_reversed(true);
}

// High goal score: bottom CCW, middle CCW, top CW
// Note: CCW is negative velocity, CW is positive velocity
// Since top roller is reversed, CW for top means we send positive value
void Intake::scoreHighGoal() {
  bottomRoller.move(-speed);  // CCW
  middleRoller.move(-speed);  // CCW
  topRoller.move(speed);      // CW (reversed motor, so positive = CW from intake perspective)
  is_running = true;
}

// Middle goal score: bottom CCW, middle CCW, top CCW
void Intake::scoreMiddleGoal() {
  bottomRoller.move(-speed);  // CCW
  middleRoller.move(-speed);  // CCW
  topRoller.move(-speed);     // CCW
  is_running = true;
}

// Low goal score: bottom CW, middle CW, top CCW
void Intake::scoreLowGoal() {
  bottomRoller.move(speed);   // CW
  middleRoller.move(speed);   // CW
  topRoller.move(-speed);     // CCW
  is_running = true;
}

// Stop all rollers
void Intake::stop() {
  bottomRoller.move(0);
  middleRoller.move(0);
  topRoller.move(0);
  is_running = false;
}

// Set the speed for all rollers
void Intake::setSpeed(int new_speed) {
  speed = new_speed;
}

// Get the current speed setting
int Intake::getSpeed() const {
  return speed;
}

// Check if intake is currently running
bool Intake::isRunning() const {
  return is_running;
}