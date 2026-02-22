// ─────────────────────────────────────────────────────────────────────
//  PurePursuitController – adaptive pure pursuit for a
//  differential-drive robot following a CatmullRomPath.
// ─────────────────────────────────────────────────────────────────────

#include "pure_pursuit.h"
#include <limits>

// ═════════════════════════════════════════════════════════════════════
//  Constructor
// ═════════════════════════════════════════════════════════════════════

PurePursuitController::PurePursuitController(double lookahead,
  double track_width,
  double max_speed,
  bool forwards)
  : m_lookahead(lookahead),
  m_track_width(track_width),
  m_max_speed(max_speed),
  m_forwards(forwards) {
}

// ═════════════════════════════════════════════════════════════════════
//  computeAdaptiveLookahead – shrink lookahead in tight curves
//
//    L_eff = L_base / (1 + k * |κ|),  clamped to [L_min, L_base]
// ═════════════════════════════════════════════════════════════════════

double PurePursuitController::computeAdaptiveLookahead(
  double path_curvature) const {
  double la = m_lookahead / (1.0 + m_adaptive_k * std::abs(path_curvature));
  return std::clamp(la, m_min_lookahead, m_lookahead);
}

// ═════════════════════════════════════════════════════════════════════
//  findGoalPoint – interpolated lookahead on circle-path intersection
//
//  Walks forward from closest_idx.  When a pair of consecutive
//  samples straddles the lookahead circle (one inside, one outside),
//  linearly interpolates between them for sub-sample accuracy.
// ═════════════════════════════════════════════════════════════════════

Pose PurePursuitController::findGoalPoint(
  const Pose& current,
  const CatmullRomPath& path,
  size_t closest_idx,
  double effective_lookahead,
  size_t& goal_idx_out) const {

  double laSq = effective_lookahead * effective_lookahead;

  // Walk forward looking for the circle-path intersection
  for (size_t i = closest_idx; i + 1 < path.size(); ++i) {
    double ax = path[i].pose.x - current.x;
    double ay = path[i].pose.y - current.y;
    double bx = path[i + 1].pose.x - current.x;
    double by = path[i + 1].pose.y - current.y;

    double dA2 = ax * ax + ay * ay;
    double dB2 = bx * bx + by * by;

    if (dA2 < laSq && dB2 >= laSq) {
      // This segment straddles the lookahead circle.
      // Solve for t ∈ [0,1]: |P_a + t·(P_b − P_a) − robot|² = L²
      //
      //  Let d = P_b - P_a,  f = P_a - robot
      //  a·t² + b·t + c = 0
      double dx = path[i + 1].pose.x - path[i].pose.x;
      double dy = path[i + 1].pose.y - path[i].pose.y;
      double fx = path[i].pose.x - current.x;
      double fy = path[i].pose.y - current.y;

      double a = dx * dx + dy * dy;
      double b = 2.0 * (fx * dx + fy * dy);
      double c = fx * fx + fy * fy - laSq;
      double disc = b * b - 4.0 * a * c;

      if (disc >= 0 && a > 1e-12) {
        double sqrtDisc = std::sqrt(disc);
        // We want the larger root (further along the path)
        double t = (-b + sqrtDisc) / (2.0 * a);
        t = std::clamp(t, 0.0, 1.0);

        goal_idx_out = i + 1;

        // Interpolate position
        Pose result;
        result.x = path[i].pose.x + t * dx;
        result.y = path[i].pose.y + t * dy;

        // Interpolate heading with angular wrapping
        double dtheta = path[i + 1].pose.theta - path[i].pose.theta;
        while (dtheta > 180.0)  dtheta -= 360.0;
        while (dtheta < -180.0) dtheta += 360.0;
        result.theta = Pose::normaliseDeg(path[i].pose.theta + t * dtheta);

        return result;
      }
    }
  }

  // Path exhausted or robot outside lookahead everywhere – aim for last point
  goal_idx_out = path.size() - 1;
  return path[path.size() - 1].pose;
}

// ═════════════════════════════════════════════════════════════════════
//  computeCrossTrackError – signed perpendicular distance from robot
//  to the path at the closest point.
//
//  Positive = robot is to the RIGHT of the path tangent direction.
// ═════════════════════════════════════════════════════════════════════

double PurePursuitController::computeCrossTrackError(
  const Pose& current,
  const CatmullRomPath& path,
  size_t closest_idx) const {

  // Use the tangent direction at the closest point
  const PathPoint& cp = path[closest_idx];
  double tangent_rad = cp.pose.thetaRad(); // CW from +y

  // Vector from closest path point to robot
  double dx = current.x - cp.pose.x;
  double dy = current.y - cp.pose.y;

  // Cross-track = perpendicular component (positive = right of tangent)
  //  tangent direction in (x,y): (sin θ, cos θ)  (CW-from-+y)
  //  normal  direction (right):  (cos θ, -sin θ)
  double cos_t = std::cos(tangent_rad);
  double sin_t = std::sin(tangent_rad);

  return cos_t * dx - sin_t * dy;
}

// ═════════════════════════════════════════════════════════════════════
//  computeProfiledSpeed – velocity profiling
//
//  Combines:
//    (a) Curvature speed limit:  v_curv = sqrt(a_max / |κ|)
//    (b) End-of-path deceleration ramp
//    (c) Minimum speed floor (for motion chaining)
// ═════════════════════════════════════════════════════════════════════

double PurePursuitController::computeProfiledSpeed(
  double path_curvature,
  double remaining_dist,
  const FollowPathParams& params) const {

  double max_speed = params.maxSpeed;

  // ── (a) Curvature speed limit ──
  //  v_curv (in/s) = sqrt(a_centripetal / |κ|)
  //  Convert in/s to % via: 100% ≈ 34 in/s (200 RPM, 3.25" wheels)
  constexpr double MAX_VEL_IPS = 34.0; // in/s at 100%
  double v_curv_pct = max_speed;       // default: no limit
  if (std::abs(path_curvature) > 1e-6) {
    double v_curv_ips = std::sqrt(
      params.maxCentripetalAccel / std::abs(path_curvature));
    v_curv_pct = (v_curv_ips / MAX_VEL_IPS) * 100.0;
    v_curv_pct = std::min(v_curv_pct, max_speed);
  }

  // ── (b) End-of-path deceleration ──
  //  Linear ramp from minSpeed to maxSpeed over decelDist
  double v_decel_pct = max_speed;
  if (params.decelDist > 0 && params.earlyExitRange == 0.0) {
    // Only decelerate if not motion-chaining (earlyExitRange == 0)
    if (remaining_dist < params.decelDist) {
      double fraction = remaining_dist / params.decelDist;
      double floor = std::max(params.minSpeed, 15.0); // never below 15% or minSpeed
      v_decel_pct = floor + (max_speed - floor) * fraction;
    }
  }

  // ── Combine limits ──
  double v = std::min(v_curv_pct, v_decel_pct);

  // ── (c) Minimum speed floor ──
  v = std::max(v, params.minSpeed);

  return v;
}

// ═════════════════════════════════════════════════════════════════════
//  calculate – one iteration of the pure-pursuit controller
//
//  1. Find closest point on path (from start_idx forward only).
//  2. Compute adaptive lookahead from path curvature.
//  3. Find the interpolated goal point on the circle-path intersection.
//  4. (Optional) Compute cross-track error correction.
//  5. Transform goal into robot-local frame.
//  6. Compute signed curvature κ = 2·e_x / L².
//  7. Profile target speed (curvature + decel limits).
//  8. Map (speed, κ) → left/right motor percentages.
// ═════════════════════════════════════════════════════════════════════

PurePursuitOutput PurePursuitController::calculate(
  const Pose& current,
  const CatmullRomPath& path,
  const FollowPathParams& params,
  size_t start_idx) const {

  PurePursuitOutput out;
  if (path.size() == 0) return out;

  bool forwards = params.forwards;

  // ── 1. Closest point (search only from start_idx onward) ──
  double best = std::numeric_limits<double>::max();
  size_t closest = start_idx;
  for (size_t i = start_idx; i < path.size(); ++i) {
    double dx = path[i].pose.x - current.x;
    double dy = path[i].pose.y - current.y;
    double d2 = dx * dx + dy * dy;
    if (d2 < best) { best = d2; closest = i; }
  }
  out.closest_idx = closest;

  // ── Remaining arc-length (use original length so decel ramp
  //    is unaffected by any synthetic extension) ──
  double orig_len = path.originalLength();
  double closest_arc = path[closest].arc_length;
  out.remaining_dist = std::max(orig_len - closest_arc, 0.0);

  // ── 2. Adaptive lookahead ──
  double path_kappa = path[closest].curvature;
  double eff_la = computeAdaptiveLookahead(path_kappa);

  // ── 3. Interpolated goal point ──
  size_t goal_idx = closest;
  Pose goal = findGoalPoint(current, path, closest, eff_la, goal_idx);
  out.goal_idx = goal_idx;
  out.goal_point = goal;

  double gx = goal.x;
  double gy = goal.y;

  // ── 4. Cross-track error correction (opt-in) ──
  double ct_correction = 0.0;
  if (m_cross_track_gain > 1e-9) {
    double ct_error = computeCrossTrackError(current, path, closest);
    ct_correction = m_cross_track_gain * ct_error;
  }

  // ── 5. Transform goal into robot-local frame ──
  //
  //  Robot convention: θ in degrees, CW from +y.
  //  Forward = +y, right = +x.
  //
  //  e_x > 0 means goal is to the RIGHT   → CW turn  → κ > 0
  //  e_y > 0 means goal is AHEAD

  double theta_rad = current.thetaRad();  // CW from +y, radians
  double dx = gx - current.x;
  double dy = gy - current.y;

  double cos_t = std::cos(theta_rad);
  double sin_t = std::sin(theta_rad);

  // CW-from-+y rotation matrix to robot-local:
  //   [ cos θ   -sin θ ] [ dx ]   ←  x_local (right)
  //   [ sin θ    cos θ ] [ dy ]   ←  y_local (forward)
  double e_x = cos_t * dx - sin_t * dy;
  double e_y = sin_t * dx + cos_t * dy;

  // If driving in reverse, flip only the longitudinal component so the
  // algorithm "looks out the back."  The lateral component (e_x) must
  // keep its original sign so the curvature steers the correct way when
  // combined with the negated speed in the differential-drive formula.
  if (!forwards) {
    e_y = -e_y;
  }

  // ── 6. Signed curvature ──
  //
  //   κ = 2 · e_x / L²
  //
  // L² = e_x² + e_y²  (distance to goal in robot frame).
  double L2 = e_x * e_x + e_y * e_y;
  if (L2 < 1e-9) L2 = 1e-9;  // avoid div-by-zero
  double kappa = 2.0 * e_x / L2;

  // Add cross-track correction to curvature
  kappa += ct_correction;

  out.curvature = kappa;

  // ── 7. Velocity profiling ──
  double profiled_speed = computeProfiledSpeed(
    path_kappa, out.remaining_dist, params);
  out.target_speed = profiled_speed;

  // ── 8. Differential drive mapping ──
  //
  //   v_left  = speed · (1 + κ · trackWidth / 2)
  //   v_right = speed · (1 − κ · trackWidth / 2)
  //
  //   κ > 0 → right turn → left faster (correct for CW convention)
  double speed = profiled_speed;
  if (!forwards) speed = -speed;

  double left = speed * (1.0 + kappa * m_track_width / 2.0);
  double right = speed * (1.0 - kappa * m_track_width / 2.0);

  // Scale so neither wheel exceeds ±profiled_speed (preserve ratio)
  double max_abs = std::max(std::abs(left), std::abs(right));
  double limit = profiled_speed;
  if (max_abs > limit && max_abs > 1e-9) {
    double scale = limit / max_abs;
    left *= scale;
    right *= scale;
  }

  out.left_pct = left;
  out.right_pct = right;

  return out;
}

// ═════════════════════════════════════════════════════════════════════
//  Legacy calculate overload – uses controller's stored settings
// ═════════════════════════════════════════════════════════════════════

PurePursuitOutput PurePursuitController::calculate(
  const Pose& current,
  const CatmullRomPath& path,
  size_t start_idx) const {

  FollowPathParams params;
  params.maxSpeed = std::abs(m_max_speed);
  params.forwards = m_forwards;
  return calculate(current, path, params, start_idx);
}

// ═════════════════════════════════════════════════════════════════════
//  Accessors / mutators
// ═════════════════════════════════════════════════════════════════════

void   PurePursuitController::setLookahead(double la) { m_lookahead = la; }
void   PurePursuitController::setTrackWidth(double tw) { m_track_width = tw; }
void   PurePursuitController::setMaxSpeed(double s) { m_max_speed = s; }
void   PurePursuitController::setForwards(bool f) { m_forwards = f; }

void   PurePursuitController::setAdaptiveK(double k) { m_adaptive_k = k; }
void   PurePursuitController::setMinLookahead(double la) { m_min_lookahead = la; }
void   PurePursuitController::setCrossTrackGain(double g) { m_cross_track_gain = g; }

double PurePursuitController::getLookahead()  const { return m_lookahead; }
double PurePursuitController::getTrackWidth() const { return m_track_width; }
double PurePursuitController::getMaxSpeed()   const { return m_max_speed; }
bool   PurePursuitController::getForwards()   const { return m_forwards; }

double PurePursuitController::getAdaptiveK()      const { return m_adaptive_k; }
double PurePursuitController::getMinLookahead()   const { return m_min_lookahead; }
double PurePursuitController::getCrossTrackGain() const { return m_cross_track_gain; }
