#ifndef INTAKE_H
#define INTAKE_H

#include "api.h"

// Intake class with 3 rollers for scoring at different goal heights
// Bottom roller: 11W blue motor
// Middle roller: 5.5W motor
// Top roller: 5.5W motor (reversed)

enum class ScoringMode {
  HIGH_GOAL,
  MIDDLE_GOAL,
  LOW_GOAL
};

class Intake {
 private:
  pros::Motor& bottomRoller;  // 11W blue motor
  pros::Motor& middleRoller;  // 5.5W motor
  pros::Motor& topRoller;     // 5.5W motor (reversed)
  int speed;
  bool is_running;

 public:
  Intake(pros::Motor& bottom_ref, pros::Motor& middle_ref, pros::Motor& top_ref, int default_speed = 127);

  // Scoring modes
  void scoreHighGoal();    // bottom CCW, middle CCW, top CW
  void scoreMiddleGoal();  // bottom CCW, middle CCW, top CCW
  void scoreLowGoal();     // bottom CW, middle CW, top CCW

  // General control
  void stop();
  void setSpeed(int new_speed);
  int getSpeed() const;
  bool isRunning() const;
};

#endif // INTAKE_H