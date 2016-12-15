#include "drake/systems/sensors/depth_sensor_specification.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace sensors {

namespace {
// Calculates the number of radians per increment assuming the arc bounded by
// @p max_angle and @p min_angle must be evenly divided by @p num_measurements.
// At a minimum, there must be one measurement at @p max_angle and one
// measurement at @p min_angle.
double CalculateIncrementSize(const double max_angle, const double min_angle,
                              const int num_measurements) {
  DRAKE_DEMAND(min_angle <= max_angle);
  DRAKE_DEMAND(num_measurements >= 1);
  double result = 0;  // This handles the case where max_angle == min_angle.
  if (max_angle != min_angle) {
    // We need to subtract 1 from num_measurements to account for the fact that
    // there are measurements at both the max_angle and min_angle.
    result = (max_angle - min_angle) / (num_measurements - 1);
  }
  return result;
}
}  // namespace

DepthSensorSpecification::DepthSensorSpecification(
    double min_theta, double max_theta, double min_phi, double max_phi,
    int num_theta_values, int num_phi_values, double min_range,
    double max_range)
    : min_theta_(min_theta),
      max_theta_(max_theta),
      min_phi_(min_phi),
      max_phi_(max_phi),
      num_theta_values_(num_theta_values),
      num_phi_values_(num_phi_values),
      num_depth_readings_(num_theta_values * num_phi_values),
      min_range_(min_range),
      max_range_(max_range),
      theta_increment_(
          CalculateIncrementSize(max_theta, min_theta, num_theta_values)),
      phi_increment_(CalculateIncrementSize(max_phi, min_phi, num_phi_values)) {
}

void DepthSensorSpecification::set_min_theta(double min_theta) {
  min_theta_ = min_theta;
  update_theta_increment();
}

void DepthSensorSpecification::set_max_theta(double max_theta) {
  max_theta_ = max_theta;
  update_theta_increment();
}

void DepthSensorSpecification::set_num_theta_values(double num_theta_values) {
  num_theta_values_ = num_theta_values;
  update_num_depth_readings();
  update_theta_increment();
}

void DepthSensorSpecification::set_num_phi_values(double num_phi_values) {
  num_phi_values_ = num_phi_values;
  update_num_depth_readings();
  update_phi_increment();
}

void DepthSensorSpecification::update_num_depth_readings() {
  num_depth_readings_ = num_theta_values_ * num_phi_values_;
}

void DepthSensorSpecification::update_theta_increment() {
  theta_increment_ =
      CalculateIncrementSize(max_theta_, min_theta_, num_theta_values_);
}

void DepthSensorSpecification::update_phi_increment() {
  phi_increment_ = CalculateIncrementSize(max_phi_, min_phi_, num_phi_values_);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
