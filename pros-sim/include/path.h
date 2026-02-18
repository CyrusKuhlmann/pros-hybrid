#pragma once

#include <cmath>
#include <vector>
#include <stdexcept>

#include "Eigen/Dense"

// ─────────────────────────────────────────────────────────────────────
//  Pose – a 2-D position + heading (x, y, theta)
// ─────────────────────────────────────────────────────────────────────
struct Pose {
  double x;      // inches  (x+ = right)
  double y;      // inches  (y+ = forward)
  double theta;  // degrees, CW-positive from +y (forward)

  Pose() : x(0), y(0), theta(0) {}
  Pose(double x, double y, double theta = 0) : x(x), y(y), theta(theta) {}

  Eigen::Vector2d vec() const { return { x, y }; }

  /// Convert theta to radians (same CW-from-+y convention).
  double thetaRad() const { return theta * M_PI / 180.0; }

  double distanceTo(const Pose& other) const {
    double dx = other.x - x;
    double dy = other.y - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  /// Normalise an angle in degrees to [0, 360).
  static double normaliseDeg(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0) deg += 360.0;
    return deg;
  }
};

// ─────────────────────────────────────────────────────────────────────
//  PathPoint – a single sample on the geometric path.
//  Contains only spatial information and curvature.
// ─────────────────────────────────────────────────────────────────────
struct PathPoint {
  Pose   pose;          // (x, y, theta) at this sample
  double curvature;     // 1/radius  (signed, + = CW / right turn)
  double d_curvature;   // dκ/ds  (rate of curvature change per arc-length)
  double arc_length;    // cumulative arc-length from start of path

  PathPoint()
    : curvature(0), d_curvature(0), arc_length(0) {
  }
};

// ─────────────────────────────────────────────────────────────────────
//  CatmullRomPath – pure geometric spline
//
//  Builds a smooth Catmull–Rom spline through a list of Pose waypoints.
//  Samples the spline at a configurable resolution and caches the
//  resulting PathPoints for fast look-up.
//
//  • The heading (theta) at each waypoint is used to set the tangent
//    direction, giving full control over the shape.
//  • Curvature and its derivative are computed analytically.
  //  • A closest-point lookup is provided for pure pursuit.
//
//  This class is intentionally geometry-only.  Use PurePursuitController
//  (see pure_pursuit.h) to follow the path with a robot.
// ─────────────────────────────────────────────────────────────────────
class CatmullRomPath {
public:
  /// @param waypoints  Ordered poses the spline must pass through (≥2).
  /// @param alpha      Catmull–Rom parameterization:
  ///                    0 = uniform, 0.5 = centripetal (default), 1 = chordal.
  /// @param samples_per_segment  Number of interpolated samples between each
  ///                              pair of consecutive waypoints.
  CatmullRomPath(const std::vector<Pose>& waypoints,
    double alpha = 0.5,
    int samples_per_segment = 100);

  // ── Queries ──────────────────────────────────────────────────────

  /// Total arc-length of the path (inches).
  double totalLength() const;

  /// Number of cached sample points.
  size_t size() const;

  /// Access a cached sample by index.
  const PathPoint& operator[](size_t i) const;

  /// All cached samples.
  const std::vector<PathPoint>& points() const;

  /// The original waypoints supplied to the constructor.
  const std::vector<Pose>& waypoints() const;

  /// Sample the path at a given arc-length s (clamped to [0, totalLength()]).
  PathPoint sampleAtArcLength(double s) const;

  /// Find the index of the closest cached point to the given (x, y).
  size_t closestPointIndex(double x, double y) const;

  /// Find the closest PathPoint to the given (x, y).
  PathPoint closestPoint(double x, double y) const;

  /// Compute heading from one PathPoint to the next (tangent direction).
  static double headingBetween(const PathPoint& a, const PathPoint& b);

private:
  std::vector<Pose>      m_waypoints;
  std::vector<PathPoint> m_points;
  double                 m_alpha;
  int                    m_samples_per_segment;

  // Spline maths
  void buildSpline();

  /// Hermite basis evaluation at normalised t ∈ [0,1].
  static Eigen::Vector2d evalHermite(const Eigen::Vector2d& p0,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& m0,
    const Eigen::Vector2d& m1,
    double t);
  static Eigen::Vector2d evalHermiteDeriv(const Eigen::Vector2d& p0,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& m0,
    const Eigen::Vector2d& m1,
    double t);
  static Eigen::Vector2d evalHermiteDeriv2(const Eigen::Vector2d& p0,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& m0,
    const Eigen::Vector2d& m1,
    double t);

  /// Signed curvature from first and second derivatives.
  static double computeCurvature(const Eigen::Vector2d& d1,
    const Eigen::Vector2d& d2);
};
