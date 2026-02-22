// ─────────────────────────────────────────────────────────────────────
//  CatmullRomPath – pure geometric Catmull–Rom spline.
// ─────────────────────────────────────────────────────────────────────

#include "path.h"
#include <algorithm>
#include <limits>

// ═════════════════════════════════════════════════════════════════════
//  Hermite basis helpers  (cubic Hermite on t ∈ [0, 1])
// ═════════════════════════════════════════════════════════════════════

Eigen::Vector2d CatmullRomPath::evalHermite(
  const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
  const Eigen::Vector2d& m0, const Eigen::Vector2d& m1, double t) {
  double t2 = t * t;
  double t3 = t2 * t;
  return (2 * t3 - 3 * t2 + 1) * p0
    + (t3 - 2 * t2 + t) * m0
    + (-2 * t3 + 3 * t2) * p1
    + (t3 - t2) * m1;
}

Eigen::Vector2d CatmullRomPath::evalHermiteDeriv(
  const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
  const Eigen::Vector2d& m0, const Eigen::Vector2d& m1, double t) {
  double t2 = t * t;
  return (6 * t2 - 6 * t) * p0
    + (3 * t2 - 4 * t + 1) * m0
    + (-6 * t2 + 6 * t) * p1
    + (3 * t2 - 2 * t) * m1;
}

Eigen::Vector2d CatmullRomPath::evalHermiteDeriv2(
  const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
  const Eigen::Vector2d& m0, const Eigen::Vector2d& m1, double t) {
  return (12 * t - 6) * p0
    + (6 * t - 4) * m0
    + (-12 * t + 6) * p1
    + (6 * t - 2) * m1;
}

// ═════════════════════════════════════════════════════════════════════
//  Curvature  κ = (x'y'' − y'x'') / (x'² + y'²)^(3/2)
// ═════════════════════════════════════════════════════════════════════

double CatmullRomPath::computeCurvature(const Eigen::Vector2d& d1,
  const Eigen::Vector2d& d2) {
  double cross = d1.x() * d2.y() - d1.y() * d2.x();
  double speed = d1.norm();
  if (speed < 1e-9) return 0.0;
  // Negate so positive κ = CW turn (right), matching CW-positive heading.
  return -cross / (speed * speed * speed);
}

// ═════════════════════════════════════════════════════════════════════
//  Constructor
// ═════════════════════════════════════════════════════════════════════

CatmullRomPath::CatmullRomPath(const std::vector<Pose>& waypoints,
  double alpha,
  int samples_per_segment)
  : m_waypoints(waypoints),
  m_alpha(alpha),
  m_samples_per_segment(samples_per_segment) {
  if (waypoints.size() < 2) {
    throw std::runtime_error("CatmullRomPath requires at least 2 waypoints");
  }
  buildSpline();
}

// ═════════════════════════════════════════════════════════════════════
//  buildSpline – sample every segment and compute curvature / arc len
// ═════════════════════════════════════════════════════════════════════

void CatmullRomPath::buildSpline() {
  m_points.clear();
  const int N = static_cast<int>(m_waypoints.size());

  for (int seg = 0; seg < N - 1; ++seg) {
    const Pose& wp0 = m_waypoints[seg];
    const Pose& wp1 = m_waypoints[seg + 1];

    Eigen::Vector2d p0 = wp0.vec();
    Eigen::Vector2d p1 = wp1.vec();

    double chord = (p1 - p0).norm();
    if (chord < 1e-9) chord = 1.0;

    double scale = chord;
    // Tangent direction from heading (CW-from-+y convention):
    //   forward = (sin θ, cos θ) in global (x, y) coords
    double t0 = wp0.theta * M_PI / 180.0;
    double t1 = wp1.theta * M_PI / 180.0;
    Eigen::Vector2d m0(std::sin(t0) * scale,
      std::cos(t0) * scale);
    Eigen::Vector2d m1(std::sin(t1) * scale,
      std::cos(t1) * scale);

    int startT = (seg == 0) ? 0 : 1;
    for (int i = startT; i <= m_samples_per_segment; ++i) {
      double t = static_cast<double>(i) / m_samples_per_segment;

      Eigen::Vector2d pos = evalHermite(p0, p1, m0, m1, t);
      Eigen::Vector2d d1 = evalHermiteDeriv(p0, p1, m0, m1, t);
      Eigen::Vector2d d2 = evalHermiteDeriv2(p0, p1, m0, m1, t);

      PathPoint pp;
      pp.pose.x = pos.x();
      pp.pose.y = pos.y();
      pp.pose.theta = Pose::normaliseDeg(
        std::atan2(d1.x(), d1.y()) * 180.0 / M_PI);
      pp.curvature = computeCurvature(d1, d2);

      m_points.push_back(pp);
    }
  }

  // ── Compute cumulative arc-length ──
  m_points[0].arc_length = 0.0;
  for (size_t i = 1; i < m_points.size(); ++i) {
    double dx = m_points[i].pose.x - m_points[i - 1].pose.x;
    double dy = m_points[i].pose.y - m_points[i - 1].pose.y;
    m_points[i].arc_length =
      m_points[i - 1].arc_length + std::sqrt(dx * dx + dy * dy);
  }

  // ── Compute dκ/ds ──
  for (size_t i = 1; i < m_points.size(); ++i) {
    double ds = m_points[i].arc_length - m_points[i - 1].arc_length;
    if (ds > 1e-9) {
      m_points[i].d_curvature =
        (m_points[i].curvature - m_points[i - 1].curvature) / ds;
    }
  }
  if (!m_points.empty()) {
    m_points[0].d_curvature =
      (m_points.size() > 1) ? m_points[1].d_curvature : 0.0;
  }
}

// ═════════════════════════════════════════════════════════════════════
//  Simple accessors
// ═════════════════════════════════════════════════════════════════════

double CatmullRomPath::totalLength() const {
  return m_points.empty() ? 0.0 : m_points.back().arc_length;
}

double CatmullRomPath::originalLength() const {
  return (m_original_length >= 0.0) ? m_original_length : totalLength();
}

size_t CatmullRomPath::size() const { return m_points.size(); }

const PathPoint& CatmullRomPath::operator[](size_t i) const {
  return m_points.at(i);
}

const std::vector<PathPoint>& CatmullRomPath::points() const {
  return m_points;
}

const std::vector<Pose>& CatmullRomPath::waypoints() const {
  return m_waypoints;
}

// ═════════════════════════════════════════════════════════════════════
//  sampleAtArcLength – linear interpolation between cached samples
// ═════════════════════════════════════════════════════════════════════

PathPoint CatmullRomPath::sampleAtArcLength(double s) const {
  if (m_points.empty()) return {};

  s = std::clamp(s, 0.0, totalLength());

  auto it = std::lower_bound(
    m_points.begin(), m_points.end(), s,
    [](const PathPoint& pp, double val) {
      return pp.arc_length < val;
    });

  if (it == m_points.begin()) return m_points.front();
  if (it == m_points.end())   return m_points.back();

  size_t idx = static_cast<size_t>(it - m_points.begin());
  const PathPoint& a = m_points[idx - 1];
  const PathPoint& b = m_points[idx];

  double ds = b.arc_length - a.arc_length;
  double frac = (ds > 1e-12) ? (s - a.arc_length) / ds : 0.0;

  PathPoint result;
  result.pose.x = a.pose.x + frac * (b.pose.x - a.pose.x);
  result.pose.y = a.pose.y + frac * (b.pose.y - a.pose.y);
  result.curvature = a.curvature + frac * (b.curvature - a.curvature);
  result.d_curvature = a.d_curvature + frac * (b.d_curvature - a.d_curvature);
  result.arc_length = s;

  // Interpolate heading with angular wrapping (degrees)
  double dtheta = b.pose.theta - a.pose.theta;
  while (dtheta > 180.0) dtheta -= 360.0;
  while (dtheta < -180.0) dtheta += 360.0;
  result.pose.theta = Pose::normaliseDeg(a.pose.theta + frac * dtheta);

  return result;
}

// ═════════════════════════════════════════════════════════════════════
//  Closest-point lookup (brute-force over cached samples)
// ═════════════════════════════════════════════════════════════════════

size_t CatmullRomPath::closestPointIndex(double x, double y) const {
  double best = std::numeric_limits<double>::max();
  size_t idx = 0;
  for (size_t i = 0; i < m_points.size(); ++i) {
    double dx = m_points[i].pose.x - x;
    double dy = m_points[i].pose.y - y;
    double d2 = dx * dx + dy * dy;
    if (d2 < best) { best = d2; idx = i; }
  }
  return idx;
}

PathPoint CatmullRomPath::closestPoint(double x, double y) const {
  return m_points[closestPointIndex(x, y)];
}

double CatmullRomPath::headingBetween(const PathPoint& a,
  const PathPoint& b) {
  double dx = b.pose.x - a.pose.x;
  double dy = b.pose.y - a.pose.y;
  return Pose::normaliseDeg(std::atan2(dx, dy) * 180.0 / M_PI);
}

// ═════════════════════════════════════════════════════════════════════
//  extend – append synthetic samples beyond the endpoint
//
//  Extrapolates with constant curvature (circular arc, or straight
//  line if curvature ≈ 0).  Useful for giving the pure-pursuit
//  controller valid lookahead points near the end of the path so it
//  doesn't lock onto the final point and produce jerky output.
//
//  The original arc-length is saved the first time extend() is
//  called so that velocity profiling (deceleration ramp) still
//  references the real endpoint.
// ═════════════════════════════════════════════════════════════════════

void CatmullRomPath::extend(double distance) {
  if (m_points.size() < 2 || distance <= 0.0) return;

  // Remember the real path length before any extension
  if (m_original_length < 0.0)
    m_original_length = totalLength();

  const PathPoint& last = m_points.back();
  double kappa = last.curvature;          // 1/inches, signed
  double theta = last.pose.theta * M_PI / 180.0;  // CW-from-+y, radians
  double x = last.pose.x;
  double y = last.pose.y;
  double arc = last.arc_length;

  // Step size: match the average spacing of the existing spline
  double avg_spacing = totalLength() / static_cast<double>(m_points.size() - 1);
  double ds = std::max(avg_spacing, 0.05);  // at least 0.05"

  double remaining = distance;
  while (remaining > 1e-6) {
    double step = std::min(ds, remaining);

    // Advance heading by κ·ds  (κ > 0 → CW → theta increases)
    theta += kappa * step;
    // Advance position   (forward = (sin θ, cos θ) in CW-from-+y)
    x += std::sin(theta) * step;
    y += std::cos(theta) * step;
    arc += step;
    remaining -= step;

    PathPoint pp;
    pp.pose.x = x;
    pp.pose.y = y;
    pp.pose.theta = Pose::normaliseDeg(theta * 180.0 / M_PI);
    pp.curvature = kappa;       // constant curvature extension
    pp.d_curvature = 0.0;
    pp.arc_length = arc;
    m_points.push_back(pp);
  }
}
