#pragma once

#include <cmath>
#include <algorithm>

#include "path.h"

// ─────────────────────────────────────────────────────────────────────
//  FollowPathParams – per-call parameters for followPath, supporting
//  motion chaining (minSpeed / earlyExitRange) and velocity profiling.
// ─────────────────────────────────────────────────────────────────────
struct FollowPathParams {
  bool   forwards = true;   // Drive forwards or backwards
  double maxSpeed = 80.0;   // Maximum speed (0–100 %)
  double minSpeed = 0.0;    // Minimum speed for motion chaining (%)
  double earlyExitRange = 0.0;    // Early exit distance from final point (in)
  double maxAccel = 4.0;    // Slew rate limit (% per loop iteration)
  double decelDist = 12.0;   // Distance (in) from end to begin decelerating
  double maxCentripetalAccel = 50.0; // Max centripetal accel (in/s²) for
  // curvature speed limiting
};

// ─────────────────────────────────────────────────────────────────────
//  PurePursuitOutput – the result of one controller update.
// ─────────────────────────────────────────────────────────────────────
struct PurePursuitOutput {
  double curvature;    // signed curvature to the goal point (1/inches)
  //   + = right/CW turn
  double left_pct;     // left  motor output  (−100 … +100 %)
  double right_pct;    // right motor output  (−100 … +100 %)

  size_t closest_idx;  // index of the closest path point
  size_t goal_idx;     // index of the goal (lookahead) point
  Pose   goal_point;   // interpolated goal (lookahead) point on the path
  double target_speed; // profiled target speed (%) after curvature/decel limits
  double remaining_dist; // arc-length remaining from closest point to end

  PurePursuitOutput()
    : curvature(0), left_pct(0), right_pct(0),
    closest_idx(0), goal_idx(0), target_speed(0), remaining_dist(0) {
  }
};

// ─────────────────────────────────────────────────────────────────────
//  PurePursuitController
//
//  Adaptive pure pursuit for a differential-drive robot following a
//  CatmullRomPath.
//
//  ── Algorithm ──────────────────────────────────────────────────────
//
//  1. Find the closest point on the path to the robot.
//  2. Walk forward along the path by the (adaptive) lookahead distance
//     to find the goal point, interpolating on the circle-path
//     intersection for sub-sample accuracy.
//  3. Transform the goal point into the robot-local frame.
//  4. Compute the signed curvature of the arc from the robot to the
//     goal point:
//
//       κ = 2 · e_x / L²
//
//     where e_x is the lateral offset in the robot frame and L is
//     the distance to the goal.
//
//  5. Profile the target speed based on path curvature and remaining
//     distance (velocity profiling).
//
//  6. Convert (speed, κ) to differential wheel percentages using
//     the track width.
//
//  ── Coordinate convention ──────────────────────────────────────────
//
//    x+ = right,  y+ = forward
//    θ in degrees, CW-positive from +y
//    κ > 0  →  right / CW turn  →  left wheel faster
//
// ─────────────────────────────────────────────────────────────────────
class PurePursuitController {
public:
  /// @param lookahead     Lookahead distance (inches).  Larger = smoother
  ///                      but cuts corners more.  Typical 6–15".
  /// @param track_width   Distance between left and right wheels (inches).
  /// @param max_speed     Base driving speed (−100 … +100 %).
  /// @param forwards      true = drive forward, false = reverse.
  PurePursuitController(double lookahead, double track_width,
    double max_speed = 80.0,
    bool forwards = true);

  /// Compute one update given the current robot pose, path, and
  /// follow-path parameters (for velocity profiling).
  ///
  /// @param current    Current robot pose (x, y in inches, θ in degrees CW).
  /// @param path       The geometric path to follow.
  /// @param params     Per-call parameters (speed limits, decel, etc.).
  /// @param start_idx  Minimum path index to search from (avoids
  ///                   re-locking to already-passed points).
  /// @return  PurePursuitOutput with motor percentages, indices, and
  ///          profiled speed.
  PurePursuitOutput calculate(const Pose& current,
    const CatmullRomPath& path,
    const FollowPathParams& params,
    size_t start_idx = 0) const;

  /// Legacy overload – uses controller's stored max_speed / forwards
  /// with default FollowPathParams.
  PurePursuitOutput calculate(const Pose& current,
    const CatmullRomPath& path,
    size_t start_idx = 0) const;

  // ── Accessors / mutators ──────────────────────────────────────

  void   setLookahead(double lookahead);
  void   setTrackWidth(double tw);
  void   setMaxSpeed(double speed);
  void   setForwards(bool fwd);

  void   setAdaptiveK(double k);
  void   setMinLookahead(double la);
  void   setCrossTrackGain(double gain);

  double getLookahead()     const;
  double getTrackWidth()    const;
  double getMaxSpeed()      const;
  bool   getForwards()      const;
  double getAdaptiveK()     const;
  double getMinLookahead()  const;
  double getCrossTrackGain() const;

private:
  double m_lookahead;
  double m_track_width;
  double m_max_speed;
  bool   m_forwards;

  // Adaptive lookahead parameters
  double m_adaptive_k = 5.0;   // Higher = more lookahead reduction in curves
  double m_min_lookahead = 6.0;   // Minimum lookahead (inches)

  // Cross-track error correction (opt-in, default off)
  double m_cross_track_gain = 0.0;  // Proportional gain on cross-track error

  /// Find the interpolated goal point on the circle-path intersection.
  /// Walks forward from closest_idx until the lookahead circle is crossed,
  /// then linearly interpolates for sub-sample accuracy.
  /// Returns the interpolated Pose and the path index just past the
  /// intersection (via goal_idx_out).
  Pose findGoalPoint(const Pose& current,
    const CatmullRomPath& path,
    size_t closest_idx,
    double effective_lookahead,
    size_t& goal_idx_out) const;

  /// Compute the effective (adaptive) lookahead distance based on
  /// path curvature at the closest point.
  double computeAdaptiveLookahead(double path_curvature) const;

  /// Compute the profiled target speed given curvature, remaining
  /// distance, and FollowPathParams.
  double computeProfiledSpeed(double path_curvature,
    double remaining_dist,
    const FollowPathParams& params) const;

  /// Compute signed cross-track error (positive = robot is to the
  /// right of the path tangent).
  double computeCrossTrackError(const Pose& current,
    const CatmullRomPath& path,
    size_t closest_idx) const;
};
