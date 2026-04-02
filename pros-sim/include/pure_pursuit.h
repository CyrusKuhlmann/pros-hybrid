#pragma once

#include <cmath>
#include <algorithm>
#include <vector>

#include "path.h"

// --------------------------------------------------------------------------
//  FollowPathParams - per-call parameters for followPath.
// --------------------------------------------------------------------------
struct FollowPathParams {
  bool   forwards       = true;   // Drive forwards or backwards
  double maxSpeed        = 80.0;   // Maximum speed (0-100 %)
  double minSpeed        = 0.0;    // Minimum speed for motion chaining (%)
  double earlyExitRange  = 0.0;    // Early exit distance from final point (in)
  double slewRate        = 800.0;  // Max wheel speed change per second (%)
  double lookahead       = 15.0;   // Lookahead distance (inches)
};

// --------------------------------------------------------------------------
//  PurePursuitOutput - the result of one controller update.
// --------------------------------------------------------------------------
struct PurePursuitOutput {
  double leftSpeed;      // desired left wheel speed (% of max)
  double rightSpeed;     // desired right wheel speed (% of max)

  size_t closestIdx;     // index of the closest path point
  double targetVel;      // velocity from the path at the closest point
  double curvature;      // signed curvature to the lookahead point

  Pose   goalPoint;      // interpolated lookahead point on the path

  PurePursuitOutput()
    : leftSpeed(0), rightSpeed(0), closestIdx(0),
      targetVel(0), curvature(0) {}
};

// --------------------------------------------------------------------------
//  PurePursuitController
//
//  Pure pursuit for a differential-drive robot, closely following the
//  algorithm from the Python reference implementation:
//
//  1. Find the closest point on the path to the robot.
//  2. Find the lookahead point by intersecting a circle (centered on
//     the robot, radius = lookahead distance) with path segments,
//     searching from the END of the path backwards to find the
//     furthest-along intersection.
//  3. Compute signed curvature from the robot to the lookahead point:
//       side = sign(sin(pi/2 - th)*(lx - px) - cos(pi/2 - th)*(ly - py))
//       x = perpendicular distance from robot's heading line to lookahead
//       kappa = side * 2x / L^2
//  4. Convert (velocity, curvature) to differential wheel speeds:
//       left  = vel * (2 + kappa*trackwidth) / 2
//       right = vel * (2 - kappa*trackwidth) / 2
//
//  The caller (Actor::followPath) applies PID control loops to drive
//  the actual motor speeds to the desired values.
//
//  Coordinate convention:
//    x+ = right,  y+ = forward
//    theta in degrees, CW-positive from +y
//    kappa > 0  ->  right/CW turn  ->  left wheel faster
//
// --------------------------------------------------------------------------
class PurePursuitController {
public:
  /// @param track_width   Distance between left and right wheels (inches).
  PurePursuitController(double track_width);

  /// Compute one update given current pose, path, and parameters.
  PurePursuitOutput calculate(const Pose& current,
                              const CatmullRomPath& path,
                              const FollowPathParams& params) const;

  void   setTrackWidth(double tw);
  double getTrackWidth() const;

private:
  double m_track_width;

  /// Find the index of the closest path point to the robot.
  size_t findClosest(const Pose& current,
                     const CatmullRomPath& path) const;

  /// Find the lookahead point by circle-line intersection, searching
  /// from the end of the path backwards.
  Pose findLookahead(const Pose& current,
                     const CatmullRomPath& path,
                     double lookahead_dist,
                     size_t closestIdx) const;

  /// Compute signed curvature from the robot to the lookahead point.
  double computeCurvature(const Pose& current,
                          const Pose& lookahead,
                          double lookahead_dist) const;

  /// Map (curvature, velocity, trackwidth) to left/right wheel speeds.
  static void toWheelSpeeds(double curvature, double velocity,
                            double trackwidth,
                            double& left_out, double& right_out);
};
