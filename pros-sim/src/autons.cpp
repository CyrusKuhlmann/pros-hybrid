#include "autons.h"
#include "intake.h"
#include "main.h"
#include "actor.h"
#include "lever.h"

void right_auton(Actor& actor, Intake& intake, Lever& matchLoadLever, Lever& wingLever, Lever& hoodLever) {
  // right auton (reverse of left)

  actor.moveToPoint(0, 11.5, {.forwards = true, .maxSpeed = 50, .earlyExitRange = 0} , 4000);
  actor.turnToHeading(45, {.maxSpeed = 50, .earlyExitRange = 0}, 2000);
  intake.scoreHighGoal();
  actor.wiggle(25, 55, 22, 7.5, 0.5, 3000);
  actor.moveToPoint(16.5, 19.75, {.forwards = false, .maxSpeed = 60, .earlyExitRange = 0}, 2000);
  actor.turnToHeading(-43, {.maxSpeed = 50, .earlyExitRange = 0}, 2000);
  actor.moveToPoint(4.25, 32.5, {.forwards = true, .maxSpeed = 50, .earlyExitRange = 1.5}, 2000);
  intake.scoreLowGoal();
  actor.driveStraight(5, {.forwards = true, .maxSpeed = 15, .earlyExitRange = 0}, 2000);
  pros::delay(3000);
  intake.stop();

  // next part

  actor.moveToPoint(37, 1, {.forwards = false, .maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 13}, 1100);
  actor.moveToPoint(37.5, 1, {.forwards = false, .maxSpeed = 40, .earlyExitRange = 0}, 900);
  actor.turnToHeading(180, {.maxSpeed = 50, .earlyExitRange = 0}, 1100);

}

void left_auton(Actor& actor, Intake& intake, Lever& matchLoadLever, Lever& wingLever, Lever& hoodLever) {
    // Left Auton

  actor.moveToPoint(0, 11.5, {.forwards = true, .maxSpeed = 50, .earlyExitRange = 0} , 4000);
  actor.turnToHeading(-45, {.maxSpeed = 50, .earlyExitRange = 0}, 2000);
  intake.scoreHighGoal();
  actor.wiggle(25, 55, 22, 7.5, 0.5, 3000);
  actor.moveToPoint(-16.5, 22.25, {.forwards = false, .maxSpeed = 60, .earlyExitRange = 0}, 2000);
  actor.turnToHeading(-127, {.maxSpeed = 50, .earlyExitRange = 0}, 2000);
  actor.moveToPoint(-5.8, 34.4, {.forwards = false, .maxSpeed = 50, .earlyExitRange = 1.5}, 2000);
  intake.scoreLowGoal();
  pros::delay(300);
  intake.scoreMiddleGoal();
  actor.driveStraight(-5, {.forwards = false, .maxSpeed = 15, .earlyExitRange = 0}, 2000);
  pros::delay(275);
  intake.stop();

  // next part

  actor.moveToPoint(-40.5, 1, {.forwards = true, .maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 13}, 1200);
  actor.moveToPoint(-41, 1, {.forwards = true
    , .maxSpeed = 40, .earlyExitRange = 0}, 1000);
  actor.turnToHeading(-180, {.maxSpeed = 50, .earlyExitRange = 0}, 1100);

//   // advanced match load

//   matchLoadLever.extend();
//   pros::delay(200);
//   actor.driveStraight(9.5, {.forwards = true, .maxSpeed = 30, .earlyExitRange = 0}, 3000);
//   hoodLever.retract();
//   intake.scoreHighGoal();
//   actor.wiggle(15, 15, 5, 7.5, 0.5, 1500);
//   actor.driveStraight(-25.5, {.forwards = false, .maxSpeed = 40, .earlyExitRange = 0}, 2000);
//   intake.scoreLowGoal();
//   pros::delay(200);
//   hoodLever.extend();
//   intake.scoreHighGoal();
//   pros::delay(5000);
//   intake.stop();
}

void skills_auton(Actor& actor, Intake& intake, Lever& matchLoadLever, Lever& wingLever, Lever& hoodLever) {
    intake.scoreHighGoal();
    pros::delay(3000);
    intake.stop();
}