#include "particle_filter.h"
#include "api.h"
#include <cmath>
#include <algorithm>
#include <limits>
#ifdef PROS_SIM
#include "sim/sim_client.h"
#endif


const float BODY_WIDTH = 18.0f; // inches – distance between left and right sensors (adjust as needed)
// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

ParticleFilter::ParticleFilter(int num_particles) : N(num_particles) {
  particles.resize(N);
}



void ParticleFilter::initialize() {
  std::uniform_real_distribution<float> dist_x(-FIELD_WIDTH / 2, FIELD_WIDTH / 2);
  std::uniform_real_distribution<float> dist_y(-FIELD_HEIGHT / 2, FIELD_HEIGHT / 2);

  for (auto& particle : particles) {
    particle.state = Eigen::Vector2f(dist_x(generator), dist_y(generator));
    particle.weight = 1.0f / N;
    particle.is_sensor_updated = false;
  }
}

void ParticleFilter::initialize(float center_x, float center_y, float spread) {
  std::normal_distribution<float> dist_x(center_x, spread);
  std::normal_distribution<float> dist_y(center_y, spread);

  for (auto& particle : particles) {
    particle.state = Eigen::Vector2f(
      std::clamp(dist_x(generator), -FIELD_WIDTH / 2, FIELD_WIDTH / 2),
      std::clamp(dist_y(generator), -FIELD_HEIGHT / 2, FIELD_HEIGHT / 2)
    );
    particle.weight = 1.0f / N;
    particle.is_sensor_updated = false;
  }
}

// ---------------------------------------------------------------------------
// Gaussian likelihood   p(z | x)
// ---------------------------------------------------------------------------

float ParticleFilter::compute_likelihood(float predicted, float actual) {
  float likelihood = std::exp(-0.5 * std::pow((predicted - actual) / sigma_sensor, 2));
  return likelihood;
}

void ParticleFilter::update_sensor(float left_dist_inches, float right_dist_inches,
  float heading_rad) {
  // Normalize heading to [0, 360) degrees
  float heading_deg = std::fmod(heading_rad * 180.0f / static_cast<float>(M_PI), 360.0f);
  if (heading_deg < 0) heading_deg += 360.0f;

  // Determine sensor-to-wall axis based on robot facing direction
  bool use_up_down_map;
  if (heading_deg >= 315.0f || heading_deg < 45.0f) {
    use_up_down_map = true;   // facing north – sensors face east/west
  }
  else if (heading_deg >= 135.0f && heading_deg < 225.0f) {
    use_up_down_map = true;   // facing south – sensors face east/west
  }
  else {
    use_up_down_map = false;  // facing east or west – sensors face north/south
  }

  for (auto& particle : particles) {
    if (use_up_down_map && (std::abs(particle.state.x()) > 48 || (std::abs(particle.state.x()) > 24 && std::abs(particle.state.y()) > 24))) {
      update_particle(particle, left_dist_inches, right_dist_inches, heading_rad);
    }
    else if (!use_up_down_map && (std::abs(particle.state.x()) < 36 && std::abs(particle.state.y()) > 24)) {
      update_particle(particle, left_dist_inches, right_dist_inches, heading_rad);
    }
  }
}

void ParticleFilter::update_particle(Particle& particle, float left_dist_inches, float right_dist_inches, float heading_rad) {
  // heading_rad is navigational: CW from +Y (north)
  // Convert to standard math angles for raycasting (CCW from +X)
  float right_angle = -heading_rad;                          // right sensor faces robot's +X in world
  float left_angle = static_cast<float>(M_PI) - heading_rad; // left sensor faces robot's -X in world

  // Rotate sensor offsets from robot-local frame (+X=right, +Y=forward) to world frame
  Eigen::Rotation2Df rot(-heading_rad);
  Eigen::Vector2f right_sensor_offset_global = rot * right_sensor_offset;
  Eigen::Vector2f left_sensor_offset_global = rot * left_sensor_offset;

  float predicted_right = std::max(raycast(particle.state + right_sensor_offset_global, right_angle), BODY_WIDTH / 2 - std::abs(right_sensor_offset.x()));
  float predicted_left = std::max(raycast(particle.state + left_sensor_offset_global, left_angle), BODY_WIDTH / 2 - std::abs(left_sensor_offset.x()));

  if (predicted_right < predicted_left) {
    particle.weight = compute_likelihood(predicted_right, right_dist_inches);
  }
  else {
    particle.weight = compute_likelihood(predicted_left, left_dist_inches);
  }
  particle.is_sensor_updated = true;
}

void ParticleFilter::update_motion(float delta_x, float delta_y) {
  float displacement = std::sqrt(delta_x * delta_x + delta_y * delta_y);
  float noise_sigma = sigma_motion * displacement;

  for (auto& particle : particles) {
    float nx = 0.0f, ny = 0.0f;
    if (noise_sigma > 0.0f) {
      std::normal_distribution<float> noise(0.0f, noise_sigma);
      nx = noise(generator);
      ny = noise(generator);
    }
    particle.state.x() += delta_x + nx;
    particle.state.y() += delta_y + ny;

    particle.state.x() = std::clamp(particle.state.x(), -FIELD_WIDTH / 2, FIELD_WIDTH / 2);
    particle.state.y() = std::clamp(particle.state.y(), -FIELD_HEIGHT / 2, FIELD_HEIGHT / 2);
  }
}

// ---------------------------------------------------------------------------
// Low-variance (systematic) resampling
// ---------------------------------------------------------------------------

void ParticleFilter::resample() {
  printf("Resampling particles...\n");
  double sum_updated = 0.0;
  double sum_not_updated = 0.0;
  for (const auto& particle : particles) {
    if (particle.is_sensor_updated) {
      sum_updated += particle.weight;
    }
    else {
      sum_not_updated += particle.weight;
    }
  }
  // Normalize weights so total sums to 1
  if (sum_updated > 0.0) {
    double sum = 0.0;
    for (auto& particle : particles) {
      if (particle.is_sensor_updated) {
        particle.weight *= (1.0 - sum_not_updated) / sum_updated;
        sum += particle.weight;
      }
    }
  }

  std::vector<Particle> new_particles;
  new_particles.resize(N);

  std::uniform_real_distribution<float> dist(0.0f, 1.0f / N);
  float r = dist(generator);
  float c = particles[0].weight;
  int i = 0;
  for (int m = 0; m < N; ++m) {
    float U = r + m * (1.0f / N);
    while (U > c && i < N - 1) {
      i++;
      c += particles[i].weight;
    }
    new_particles[m] = particles[i];
    new_particles[m].weight = 1.0f / N;
    new_particles[m].is_sensor_updated = false;
  }
  particles = std::move(new_particles);

}


// ---------------------------------------------------------------------------
// Weighted mean position estimate
// ---------------------------------------------------------------------------

Eigen::Vector2f ParticleFilter::estimate() const {
  Eigen::Vector2f mean = Eigen::Vector2f::Zero();
  float total_weight = 0.0f;
  for (const auto& particle : particles) {
    mean += particle.state * particle.weight;
    total_weight += particle.weight;
  }
  if (total_weight > 0) {
    mean /= total_weight;
  }
  return mean;
}

// raycast from `position` along `angle_rad` and return distance to nearest field wall
float ParticleFilter::raycast(const Eigen::Vector2f& position,
  float angle_rad) {
  float cos_a = std::cos(angle_rad);
  float sin_a = std::sin(angle_rad);
  float t = std::numeric_limits<float>::max();

  // Check intersection with vertical walls (x = -FIELD_WIDTH / 2 and x = FIELD_WIDTH / 2)
  if (cos_a != 0) {
    float t1 = (-FIELD_WIDTH / 2 - position.x()) / cos_a;
    float t2 = (FIELD_WIDTH / 2 - position.x()) / cos_a;
    if (t1 > 0) t = std::min(t, t1);
    if (t2 > 0) t = std::min(t, t2);
  }

  // Check intersection with horizontal walls (y = -FIELD_HEIGHT / 2 and y = FIELD_HEIGHT / 2)
  if (sin_a != 0) {
    float t3 = (-FIELD_HEIGHT / 2 - position.y()) / sin_a;
    float t4 = (FIELD_HEIGHT / 2 - position.y()) / sin_a;
    if (t3 > 0) t = std::min(t, t3);
    if (t4 > 0) t = std::min(t, t4);
  }

  return t;
}

void ParticleFilter::draw_particles() const {
#ifdef PROS_SIM
  std::vector<std::pair<double, double>> locations;
  for (const auto& particle : particles) {
    locations.emplace_back(particle.state.x(), particle.state.y());
  }
  Eigen::Vector2f avg = estimate();
  sim::SimClient::instance().send_particle_locations(locations, { avg.x(), avg.y() });
#endif
}
