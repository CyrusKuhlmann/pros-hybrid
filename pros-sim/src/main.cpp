#include "main.h"

#include "Eigen/Dense"
#include "actor.h"
#include "api.h"
#include "autons.h"
#include "intake.h"
#include "lever.h"
#include "odom.h"
#include "user_control.h"

Odom odom;
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({ -1, -2, -4 });
pros::MotorGroup right_motors({ 11, 12, 13 });

// create levers and pneumatics
pros::adi::Pneumatics matchLoadLeft = pros::adi::Pneumatics('A', false);
pros::adi::Pneumatics matchLoadRight = pros::adi::Pneumatics('H', false);
Lever matchLoadLever({ &matchLoadLeft, &matchLoadRight });

pros::adi::Pneumatics wing = pros::adi::Pneumatics('B', false);
Lever wingLever({ &wing });

pros::adi::Pneumatics hood = pros::adi::Pneumatics('G', false);
Lever hoodLever({ &hood });

// create intake motors (update port numbers as needed)
pros::Motor bottomRoller(-17);  // 11W blue motor
pros::Motor middleRoller(-3);   // 5.5W motor
pros::Motor topRoller(20);      // 5.5W motor (reversed in Intake constructor)
Intake intake(bottomRoller, middleRoller, topRoller);

// ========== PID Controller Settings ==========
// Tuning values for lateral and angular PID controllers

// Lateral (driving) controller settings
MotionControllerSettings lateralSettings = {
    12.0,  // kP - proportional gain (start at 10, increase for faster response)
    0.0,   // kI - integral gain (usually leave at 0 to avoid oscillation)
    39.0,  // kD - derivative gain (start at 3x kP, reduces overshoot)
    3.0,   // Anti-windup range (inches)
    0.5,   // Small error range (inches) - motion exits if within this for
    // timeout
100,   // Small error timeout (ms)
2.0,   // Large error range (inches) - motion exits if within this for
// timeout
300,   // Large error timeout (ms)
8      // Slew rate (max acceleration, lower = smoother but slower)
};

// Angular (turning) controller settings
// Turning typically uses lower kP since error is measured in degrees
MotionControllerSettings angularSettings = {
    2.5,   // kP - proportional gain (start at 2-3)
    0.0,   // kI - integral gain
    18.0,  // kD - derivative gain (start at 10-20)
    3.0,   // Anti-windup range (degrees)
    0.5,   // Small error range (degrees)
    100,   // Small error timeout (ms)
    2.0,   // Large error range (degrees)
    300,   // Large error timeout (ms)
    0      // Slew rate (0 = disabled for turning)
};

// Create Actor with PID settings
Actor actor(odom, left_motors, right_motors, lateralSettings, angularSettings);

void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  }
  else {
    pros::lcd::clear_line(2);
  }
}

void initialize() {
  pros::lcd::initialize();

  lateral_rot.reset_position();
  // lateral_rot.set_data_rate(5);
  forward_rot.reset_position();
  // forward_rot.set_data_rate(5);
  imu.reset(true);
  pros::delay(1000);
  hoodLever.retract();
  wingLever.retract();
  matchLoadLever.retract();
  pros::Task odom_task(std::bind(&Odom::odom_task_fn, &odom));
  pros::delay(1000);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  right_auton(actor, intake, matchLoadLever, wingLever, hoodLever);
  // cout the odometry position at the end of auton for debugging
  Eigen::Vector2d pos = odom.get_xy_inches();
  printf("Final Position - X: %.2f, Y: %.2f\n", pos.x(), pos.y());
}

void opcontrol() {
  user_control_loop(master, left_motors, right_motors, matchLoadLever,
    wingLever, hoodLever, intake);
}
