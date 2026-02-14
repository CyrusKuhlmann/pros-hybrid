#ifndef USER_CONTROL_H
#define USER_CONTROL_H

#include "main.h"
#include "lever.h"
#include "intake.h"

// Intake mode definitions
#define INTAKE_MODE_STOPPED 0
#define INTAKE_MODE_HIGH    1
#define INTAKE_MODE_MIDDLE  2
#define INTAKE_MODE_LOW     3
#define INTAKE_MODE_INTAKE  4


void handle_drivetrain(pros::Controller& master, 
                       pros::MotorGroup& left_motors, 
                       pros::MotorGroup& right_motors);

void handle_levers(pros::Controller& master, 
                   Lever& matchLoadLever, 
                   Lever& wingLever);


void handle_intake(pros::Controller& master, Intake& intake, Lever& hoodLever);


void user_control_loop(pros::Controller& master,
                       pros::MotorGroup& left_motors,
                       pros::MotorGroup& right_motors,
                       Lever& matchLoadLever,
                       Lever& wingLever,
                       Lever& hoodLever,
                       Intake& intake);

#endif // USER_CONTROL_H
