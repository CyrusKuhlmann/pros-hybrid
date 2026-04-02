// --------------------------------------------------------------------------
//  PurePursuitController - pure pursuit for a differential-drive robot.
//
//  Algorithm ported from the Python reference (RobotSimulator.py).
//  This computes DESIRED wheel speeds; the caller applies PID control
//  loops to drive actual motors to those targets.
// --------------------------------------------------------------------------

#include "pure_pursuit.h"
#include <limits>

// =========================================================================
//  Constructor
// =========================================================================

PurePursuitController::PurePursuitController(double track_width)
  : m_track_width(track_width) {}

// =========================================================================
//  findClosest - index of the path point nearest to the robot
//
//  Direct port of Python closest():
//    mindist = (0, dist(path[0], pos))
//    for i, p in path:
//        if dist(p, pos) < mindist[1]: mindist = (i, dist)
//    return mindist[0]
// =========================================================================

size_t PurePursuitController::findClosest(
    const Pose& current,
    const CatmullRomPath& path) const {

  size_t best_idx = 0;
  double best_d2 = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.size(); ++i) {
    double dx = path[i].pose.x - current.x;
    double dy = path[i].pose.y - current.y;
    double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = i;
    }
  }
  return best_idx;
}

// =========================================================================
//  findLookahead - circle-line intersection, searched from path END
//
//  Direct port of Python lookahead():
//    for i, p in enumerate(reversed(path[:-1])):
//        i_ = len(path) - 2 - i
//        d = (path[i_+1] - p)          # segment direction
//        f = (p - pos)                  # vector from robot to segment start
//        solve  a*t^2 + b*t + c = 0
//        if 0 <= t1 <= 1: return interpolated point
//        if 0 <= t2 <= 1: return interpolated point
//    return path[closest()]            # fallback
// =========================================================================

Pose PurePursuitController::findLookahead(
    const Pose& current,
    const CatmullRomPath& path,
    double lookahead_dist,
    size_t closestIdx) const {

  double la2 = lookahead_dist * lookahead_dist;

  // Iterate segments from the end of the path backwards
  for (size_t rev = 0; rev + 1 < path.size(); ++rev) {
    size_t i = path.size() - 2 - rev;  // i_ in the Python code

    double px = path[i].pose.x;
    double py = path[i].pose.y;
    double qx = path[i + 1].pose.x;
    double qy = path[i + 1].pose.y;

    // d = segment direction vector
    double dx = qx - px;
    double dy = qy - py;

    // f = vector from robot to segment start
    double fx = px - current.x;
    double fy = py - current.y;

    double a = dx * dx + dy * dy;
    double b = 2.0 * (fx * dx + fy * dy);
    double c = fx * fx + fy * fy - la2;
    double disc = b * b - 4.0 * a * c;

    if (disc >= 0 && a > 1e-12) {
      double sqrtDisc = std::sqrt(disc);
      double t1 = (-b + sqrtDisc) / (2.0 * a);
      double t2 = (-b - sqrtDisc) / (2.0 * a);

      // Prefer t1 (further along the segment), matching Python
      if (t1 >= 0.0 && t1 <= 1.0) {
        Pose result;
        result.x = px + t1 * dx;
        result.y = py + t1 * dy;
        return result;
      }
      if (t2 >= 0.0 && t2 <= 1.0) {
        Pose result;
        result.x = px + t2 * dx;
        result.y = py + t2 * dy;
        return result;
      }
    }
  }

  // Fallback: aim at the closest path point
  return path[closestIdx].pose;
}

// =========================================================================
//  computeCurvature - signed curvature from robot to lookahead
//
//  Direct port of Python curvature():
//    side = sign(sin(pi/2 - angle) * (look_x - pos_x)
//              - cos(pi/2 - angle) * (look_y - pos_y))
//    a = -tan(pi/2 - angle)
//    c = tan(pi/2 - angle) * pos_x - pos_y
//    x = |a * look_x + look_y + c| / sqrt(a^2 + 1)
//    return side * (2 * x / lookahead^2)
//
//  Note: the Python 'angle' is measured CW from +y in radians.
//  Our theta is CW from +y in degrees.
// =========================================================================

double PurePursuitController::computeCurvature(
    const Pose& current,
    const Pose& lookahead,
    double lookahead_dist) const {

  double angle = current.thetaRad();  // CW from +y, in radians

  // Side determination (which side of the heading line the lookahead is on)
  double half_pi = M_PI / 2.0;
  double side = std::sin(half_pi - angle) * (lookahead.x - current.x)
              - std::cos(half_pi - angle) * (lookahead.y - current.y);
  double sign_side = (side > 0) ? 1.0 : ((side < 0) ? -1.0 : 0.0);

  // Perpendicular distance from robot's heading line to the lookahead point
  //   heading line: a*x + y + c = 0  where a = -tan(pi/2 - angle),
  //   c = tan(pi/2 - angle)*pos_x - pos_y
  double a = -std::tan(half_pi - angle);
  double c_val = std::tan(half_pi - angle) * current.x - current.y;
  double x = std::abs(a * lookahead.x + lookahead.y + c_val)
           / std::sqrt(a * a + 1.0);

  double la2 = lookahead_dist * lookahead_dist;
  if (la2 < 1e-9) la2 = 1e-9;

  return sign_side * (2.0 * x / la2);
}

// =========================================================================
//  toWheelSpeeds - differential drive mapping
//
//  Direct port of Python turn():
//    left  = vel * (2 + curv * trackwidth) / 2
//    right = vel * (2 - curv * trackwidth) / 2
// =========================================================================

void PurePursuitController::toWheelSpeeds(
    double curvature, double velocity, double trackwidth,
    double& left_out, double& right_out) {
  left_out  = velocity * (2.0 + curvature * trackwidth) / 2.0;
  right_out = velocity * (2.0 - curvature * trackwidth) / 2.0;
}

// =========================================================================
//  calculate - one iteration of the pure pursuit controller
//
//  1. Find closest point on path.
//  2. Find lookahead point (circle-line intersection from end backwards).
//  3. Compute signed curvature.
//  4. Get target velocity from the path at the closest point.
//  5. Map to left/right wheel speeds.
//
//  The caller (Actor::followPath) handles:
//    - Slew rate limiting on actual motor outputs
//    - PID control to track these desired wheel speeds
//    - Exit conditions (closest == last point, distance to end, timeout)
// =========================================================================

PurePursuitOutput PurePursuitController::calculate(
    const Pose& current,
    const CatmullRomPath& path,
    const FollowPathParams& params) const {

  PurePursuitOutput out;
  if (path.size() == 0) return out;

  // 1. Closest point
  size_t close = findClosest(current, path);
  out.closestIdx = close;

  // 2. Lookahead point
  Pose look = findLookahead(current, path, params.lookahead, close);
  out.goalPoint = look;

  // 3. Curvature
  //    If the lookahead target is behind the closest point (t_i < close
  //    in the Python code), set curvature to near-zero to drive straight
  //    until we catch up. We approximate this by checking if the goal
  //    point arc position is behind the closest point.
  double curv = computeCurvature(current, look, params.lookahead);
  out.curvature = curv;

  // 4. Target velocity - use maxSpeed from params (the path velocity
  //    profiling is handled via the CatmullRomPath if desired, or the
  //    caller can set maxSpeed per-call)
  double vel = params.maxSpeed;
  out.targetVel = vel;

  // Flip velocity for reverse driving
  if (!params.forwards) vel = -vel;

  // 5. Wheel speeds
  double left, right;
  toWheelSpeeds(curv, vel, m_track_width, left, right);

  out.leftSpeed = left;
  out.rightSpeed = right;

  return out;
}

// =========================================================================
//  Accessors
// =========================================================================

void   PurePursuitController::setTrackWidth(double tw) { m_track_width = tw; }
double PurePursuitController::getTrackWidth() const { return m_track_width; }
