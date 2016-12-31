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
    double min_yaw, double max_yaw,
    double min_pitch, double max_pitch, int num_yaw_values,
    int num_pitch_values, double min_range, double max_range)
    : min_yaw_(min_yaw),
      max_yaw_(max_yaw),
      min_pitch_(min_pitch),
      max_pitch_(max_pitch),
      num_yaw_values_(num_yaw_values),
      num_pitch_values_(num_pitch_values),
      num_depth_readings_(num_yaw_values * num_pitch_values),
      min_range_(min_range),
      max_range_(max_range),
      yaw_increment_(CalculateIncrementSize(max_yaw, min_yaw, num_yaw_values)),
      pitch_increment_(
          CalculateIncrementSize(max_pitch, min_pitch, num_pitch_values)) {}

void DepthSensorSpecification::set_min_yaw(double min_yaw) {
  min_yaw_ = min_yaw;
  update_yaw_increment();
}

void DepthSensorSpecification::set_max_yaw(double max_yaw) {
  max_yaw_ = max_yaw;
  update_yaw_increment();
}

void DepthSensorSpecification::set_num_yaw_values(double num_yaw_values) {
  num_yaw_values_ = num_yaw_values;
  update_num_depth_readings();
  update_yaw_increment();
}

void DepthSensorSpecification::set_num_pitch_values(double num_pitch_values) {
  num_pitch_values_ = num_pitch_values;
  update_num_depth_readings();
  update_pitch_increment();
}

void DepthSensorSpecification::update_num_depth_readings() {
  num_depth_readings_ = num_yaw_values_ * num_pitch_values_;
}

void DepthSensorSpecification::update_yaw_increment() {
  yaw_increment_ = CalculateIncrementSize(max_yaw_, min_yaw_, num_yaw_values_);
}

void DepthSensorSpecification::update_pitch_increment() {
  pitch_increment_ =
      CalculateIncrementSize(max_pitch_, min_pitch_, num_pitch_values_);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
