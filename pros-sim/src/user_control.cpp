#include "user_control.h"

static int intake_mode = INTAKE_MODE_STOPPED;
bool is_intake = false;
bool is_middle = false;
bool is_low = false;
bool is_high = false;

double adjust_function(double input) {
    return std::max(std::min((std::tan(input/0.7))/4,1.0),-1.0);
}

void handle_drivetrain(pros::Controller& master, 
                       pros::MotorGroup& left_motors, 
                       pros::MotorGroup& right_motors) {
    left_motors.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    right_motors.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
}

void handle_levers(pros::Controller& master, 
                   Lever& matchLoadLever, 
                   Lever& wingLever) {
    // Up arrow toggles match load lever
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        matchLoadLever.toggle();
    }
    // A button toggles wing lever
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        wingLever.toggle();
    }
}

void handle_intake(pros::Controller& master, Intake& intake, Lever& hoodLever) {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake_mode = INTAKE_MODE_INTAKE;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake_mode = INTAKE_MODE_MIDDLE;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        intake_mode = INTAKE_MODE_LOW;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        intake_mode = INTAKE_MODE_HIGH;
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        intake_mode = INTAKE_MODE_STOPPED;
    }

    switch (intake_mode) {
        case INTAKE_MODE_INTAKE:
            if (is_intake) {
                break;
            }
            intake.scoreHighGoal();
            hoodLever.retract();  
            is_intake = true;
            is_middle = false;
            is_low = false;
            is_high = false;
            break;
        case INTAKE_MODE_MIDDLE:
            if (is_middle) {
                break;
            }
            intake.scoreLowGoal();
            pros::delay(200);
            intake.scoreMiddleGoal();
            is_intake = false;
            is_middle = true;
            is_low = false;
            is_high = false;
            break;
        case INTAKE_MODE_LOW:
            if (is_low) {
                break;
            }
            intake.scoreLowGoal();
            is_intake = false;
            is_middle = false;
            is_low = true;
            is_high = false;
            break;
        case INTAKE_MODE_HIGH:
            if (is_high) {
                break;
            }
            intake.scoreLowGoal();
            hoodLever.extend(); 
            pros::delay(200);
            intake.scoreHighGoal();
            is_intake = false;
            is_middle = false;
            is_low = false;
            is_high = true;
            break;
        default:
            intake.stop();
            is_intake = false;
            is_middle = false;
            is_low = false;
            is_high = false;
            break;
    }
}

void user_control_loop(pros::Controller& master,
                       pros::MotorGroup& left_motors,
                       pros::MotorGroup& right_motors,
                       Lever& matchLoadLever,
                       Lever& wingLever,
                       Lever& hoodLever,
                       Intake& intake) {
    while (true) {
        handle_drivetrain(master, left_motors, right_motors);
        handle_levers(master, matchLoadLever, wingLever);
        handle_intake(master, intake, hoodLever);
        
        pros::delay(40);
    }
}
