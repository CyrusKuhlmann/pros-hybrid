// file to store autonomous routines
#ifndef AUTONS_H
#define AUTONS_H
#include "main.h"
#include "actor.h"
#include "intake.h"
#include "lever.h"

void right_auton(Actor& actor, Intake& intake, Lever& matchLoadLever, Lever& wingLever, Lever& hoodLever);
void left_auton(Actor& actor, Intake& intake, Lever& matchLoadLever, Lever& wingLever, Lever& hoodLever);
void skills_auton(Actor& actor, Intake& intake, Lever& matchLoadLever, Lever& wingLever, Lever& hoodLever);
void test_auton(Actor& actor, Intake& intake, Lever& matchLoadLever, Lever& wingLever, Lever& hoodLever);
#endif  // AUTONS_H


