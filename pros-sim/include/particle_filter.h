#pragma once

#include <mutex>
#include <vector>
#include <random>
#include "Eigen/Dense"

struct Particle {
  Eigen::Vector2f state;  // (x, y) position in inches
  float weight;
  bool is_sensor_updated;
};

class ParticleFilter {
public:
  // ---------- Tunable parameters ----------
  // Sensor offsets from robot center (perpendicular distance, in inches).
  // Measure from robot center to each distance sensor along the side of the
  // robot and set these accordingly.
  Eigen::Vector2f right_sensor_offset = Eigen::Vector2f(7.0f, 0.0f);  // inches from center to right sensor
  Eigen::Vector2f left_sensor_offset = Eigen::Vector2f(-7.0f, 0.0f);  // inches from center to left sensor

  // Noise standard deviations
  float sigma_motion = 0.15f;   // inches – motion noise per axis
  float sigma_sensor = 1.0f;   // inches – sensor measurement noise

  // Field dimensions (VEX field: 144 in × 144 in)
  static constexpr float FIELD_WIDTH = 144.0f;
  static constexpr float FIELD_HEIGHT = 144.0f;

  // ---------- Public API ----------
  ParticleFilter(int num_particles);

  /// Scatter particles uniformly inside a rectangle.
  void initialize();

  /// Scatter particles around a known position with gaussian spread.
  void initialize(float center_x, float center_y, float spread);

  /// Update step – re-weight particles with both distance sensors.
  /// @param left_dist_inches   left sensor reading converted to inches
  /// @param right_dist_inches  right sensor reading converted to inches
  /// @param heading_rad        robot heading in radians (CW from +Y / north)
  void update_sensor(float left_dist_inches, float right_dist_inches,
    float heading_rad);
  void update_motion(float delta_x, float delta_y);

  /// Low-variance (systematic) resampling.
  void resample();

  /// Weighted mean of all particles.
  Eigen::Vector2f estimate() const;

  /// Access particles (for visualization / debugging).
  const std::vector<Particle>& get_particles() const { return particles; }

  /// Weighted mean without locking (caller must hold mtx).
  Eigen::Vector2f estimate_impl() const;

  void draw_particles() const;

private:
  mutable std::mutex mtx;
  int N;
  std::vector<Particle> particles;

  std::default_random_engine generator;

  float compute_likelihood(float predicted, float actual);

  void update_particle(Particle& particle, float left_dist_inches, float right_dist_inches, float heading_rad);

  /// Cast a ray from `position` along `angle_rad` (standard math angle,
  /// CCW from +X) and return the distance to the nearest field wall.
  /// Returns a very large value if the position is outside the field.
  float raycast(const Eigen::Vector2f& position, float angle_rad);
};

