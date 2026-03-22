#include "autons.h"
#include "intake.h"
#include "main.h"
#include "actor.h"
#include "lever.h"
#include "odom.h"
#include "pursuit_presets.h"

extern Odom odom;

CatmullRomPath path1({
    {18, -18, -225},
    {42, -38.5, -276},
    {47, -29, 0},
    {47, -23, 0},
  });

CatmullRomPath path2({
    {47, -47, 0},
    {60, -33, 0},
    {60, 25, 0},
    {52, 35, -90},
    {47, 27, 180},
    {47, 23, 180}
  });

CatmullRomPath path3({
    {47, 23, 0},
    {47, 27, 0},
    {39, 39, -55},
    {0, 37, -95},
    {-33, 35, -80},
    {-47, 50, 0},
  });

PurePursuitController pursuit(15, 11.67);  // lookahead & track width only

void right_auton(Actor& actor, Intake& intake, Lever& matchLoadLever, Lever& wingLever, Lever& hoodLever) {
  // // right auton (reverse of left)

  // actor.moveToPoint(9, -33.5, { .forwards = true, .maxSpeed = 50, .earlyExitRange = 0 }, 4000);
  // actor.turnToHeading(45, { .maxSpeed = 50, .earlyExitRange = 0 }, 2000);
  // intake.scoreHighGoal();
  // actor.wiggle(25, 55, 22, 7.5, 0.5, 3000);
  // actor.moveToPoint(25.5, -25.25, { .forwards = false, .maxSpeed = 60, .earlyExitRange = 0 }, 2000);
  // actor.turnToHeading(-43, { .maxSpeed = 50, .earlyExitRange = 0 }, 2000);
  // actor.moveToPoint(13.25, -12.5, { .forwards = true, .maxSpeed = 50, .earlyExitRange = 1.5 }, 2000);
  // intake.scoreLowGoal();
  // actor.driveStraight(5, { .forwards = true, .maxSpeed = 15, .earlyExitRange = 0 }, 2000);
  // pros::delay(3000);
  // intake.stop();

  // // next part

  // actor.moveToPoint(46, -44, { .forwards = false, .maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 13 }, 1100);
  // actor.moveToPoint(46.5, -44, { .forwards = false, .maxSpeed = 40, .earlyExitRange = 0 }, 900);
  // actor.turnToHeading(180, { .maxSpeed = 50, .earlyExitRange = 0 }, 1100);

  // start = (9, -45, 90)





  // actor.moveToPoint(47.5, -45, { .forwards = true, .maxSpeed = 47.5 }, 3000);
  // actor.turnToHeading(180, { .maxSpeed = 50, .earlyExitRange = 0 }, 1500);
  // actor.moveToPoint(47.5, -51.5, { .forwards = true, .maxSpeed = 35, .earlyExitRange = 0 }, 1500);
  // actor.turnToHeading(180, { .maxSpeed = 50, .earlyExitRange = 0 }, 1500);
  // matchLoadLever.extend();
  // intake.scoreHighGoal();
  // actor.wiggleInPlace(10, 0.35, 1500);
  // actor.moveToPoint(47.5, -35, { .forwards = false, .maxSpeed = 35, .earlyExitRange = 0 }, 1500);

  hoodLever.retract();
  wingLever.extend();

  actor.moveToPoint(47.5, -45, { .forwards = true, .maxSpeed = 47.5 }, 3000);
  actor.turnToHeading(180, { .maxSpeed = 50, .earlyExitRange = 0 }, 1500);
  actor.moveToPoint(47.5, -51.5, { .forwards = true, .maxSpeed = 35, .earlyExitRange = 0 }, 1500);
  actor.turnToHeading(180, { .maxSpeed = 50, .earlyExitRange = 0 }, 1500);
  actor.moveToPoint(47.5, -31, { .forwards = false, .maxSpeed = 35, .earlyExitRange = 0 }, 1500);

  hoodLever.extend();
  intake.scoreLowGoal();
  pros::delay(300);
  intake.scoreHighGoal();
  pros::delay(1000);
  intake.stop();

  actor.moveToPoint(47.5, -48, { .forwards = true, .maxSpeed = 35, .earlyExitRange = 0 }, 1500);
  actor.turnToHeading(-45, { .maxSpeed = 50, .earlyExitRange = 0 }, 1500);
  actor.moveToPoint(29, -29, { .forwards = true, .maxSpeed = 35, .earlyExitRange = 0 }, 2000);
  matchLoadLever.extend();
  hoodLever.retract();
  intake.scoreHighGoal();
  actor.moveToPoint(23, -23, { .forwards = true, .maxSpeed = 35, .earlyExitRange = 0 }, 2000);
  matchLoadLever.retract();
  intake.stop();
  actor.moveToPoint(15, -15, { .forwards = true, .maxSpeed = 35, .earlyExitRange = 0 }, 2000);
  intake.scoreLowGoal();
  pros::delay(3000);



}

void left_auton(Actor& actor, Intake& intake, Lever& matchLoadLever, Lever& wingLever, Lever& hoodLever) {
  // Left Auton

  actor.moveToPoint(9, -33.5, { .forwards = true, .maxSpeed = 50, .earlyExitRange = 0 }, 4000);
  actor.turnToHeading(-45, { .maxSpeed = 50, .earlyExitRange = 0 }, 2000);
  intake.scoreHighGoal();
  actor.wiggle(25, 55, 22, 7.5, 0.5, 3000);
  actor.moveToPoint(-7.5, -22.75, { .forwards = false, .maxSpeed = 60, .earlyExitRange = 0 }, 2000);
  actor.turnToHeading(-127, { .maxSpeed = 50, .earlyExitRange = 0 }, 2000);
  actor.moveToPoint(3.2, -10.6, { .forwards = false, .maxSpeed = 50, .earlyExitRange = 1.5 }, 2000);
  intake.scoreLowGoal();
  pros::delay(300);
  intake.scoreMiddleGoal();
  actor.driveStraight(-5, { .forwards = false, .maxSpeed = 15, .earlyExitRange = 0 }, 2000);
  pros::delay(275);
  intake.stop();

  // next part

  actor.moveToPoint(-31.5, -44, { .forwards = true, .maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 13 }, 1200);
  actor.moveToPoint(-32, -44, { .forwards = true, .maxSpeed = 40, .earlyExitRange = 0 }, 1000);
  actor.turnToHeading(-180, { .maxSpeed = 50, .earlyExitRange = 0 }, 1100);


}

void skills_auton(Actor& actor, Intake& intake, Lever& matchLoadLever, Lever& wingLever, Lever& hoodLever) {
  actor.moveToPoint(9, -33.5, { .forwards = true, .maxSpeed = 50, .earlyExitRange = 0 }, 4000);
  actor.turnToHeading(45, { .maxSpeed = 50, .earlyExitRange = 0 }, 2000);
  intake.scoreHighGoal();
  actor.wiggle(25, 55, 22, 7.5, 0.5, 3000);
  actor.moveToPoint(25.5, -25.25, { .forwards = false, .maxSpeed = 60, .earlyExitRange = 0 }, 2000);
  actor.turnToHeading(-43, { .maxSpeed = 50, .earlyExitRange = 0 }, 2000);
  actor.moveToPoint(13.25, -12.5, { .forwards = true, .maxSpeed = 50, .earlyExitRange = 1.5 }, 2000);
  intake.scoreLowGoal();
  actor.driveStraight(5, { .forwards = true, .maxSpeed = 15, .earlyExitRange = 0 }, 2000);
  pros::delay(3000);
  intake.stop();
  actor.driveStraight(-7, { .forwards = false, .maxSpeed = 35, .minSpeed = 15, .earlyExitRange = 3 }, 2000);

  PursuitPreset::applyTight(pursuit);
  actor.followPath(path1, pursuit, PursuitPreset::REVERSE_PRECISE);

  // odom.manual_set_xy(47, -27);

  matchLoadLever.extend();
  actor.moveToPoint(47, -45.5, { .forwards = true, .maxSpeed = 40, .earlyExitRange = 0 }, 6000);
  actor.wiggle(20, 15, 7.5, 7.5, 0.5, 1500);

  wingLever.extend();

  actor.followPath(path2, pursuit, PursuitPreset::REVERSE_PRECISE);

  // odom.manual_set_xy(47, 23);

  hoodLever.extend();
  intake.scoreHighGoal();
  pros::delay(3000);

  hoodLever.retract();
  matchLoadLever.extend();

  actor.moveToPoint(47, 48, { .forwards = true, .maxSpeed = 40, .earlyExitRange = 0 }, 2000);
  actor.wiggle(20, 15, 7.5, 7.5, 0.5, 1500);

  actor.moveToPoint(47, 23, { .forwards = false, .maxSpeed = 40, .earlyExitRange = 0 }, 2000);

  hoodLever.extend();
  intake.scoreHighGoal();
  pros::delay(3000);

  matchLoadLever.retract();

  actor.followPath(path3, pursuit, PursuitPreset::PRECISE);

  actor.moveToPoint(-47, 23, { .forwards = false, .maxSpeed = 40, .earlyExitRange = 0 }, 2000);

  // odom.manual_set_xy(-47, 23);

  matchLoadLever.extend();
  hoodLever.retract();
  intake.scoreHighGoal();

  actor.moveToPoint(-47, 45, { .forwards = true, .maxSpeed = 40, .earlyExitRange = 0 }, 2000);
  actor.wiggle(20, 15, 7.5, 7.5, 0.5, 1500);

  actor.moveToPoint(-47, 23, { .forwards = false, .maxSpeed = 40, .earlyExitRange = 0 }, 2000);

  hoodLever.extend();
  intake.scoreHighGoal();
  pros::delay(3000);




}

void test_auton(Actor& actor, Intake& intake, Lever& matchLoadLever, Lever& wingLever, Lever& hoodLever) {
  actor.driveStraight(18, { .forwards = true, .maxSpeed = 50, .earlyExitRange = 0 }, 2000);
  actor.turnToHeading(90, { .maxSpeed = 50, .earlyExitRange = 0 }, 2000);
}