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
      min_range_(min_range),
      max_range_(max_range) {}

int DepthSensorSpecification::num_depth_readings() const {
  return num_yaw_values_ * num_pitch_values_;
}

double DepthSensorSpecification::yaw_increment() const {
  return CalculateIncrementSize(max_yaw_, min_yaw_, num_yaw_values_);
}

double DepthSensorSpecification::pitch_increment() const {
  return CalculateIncrementSize(max_pitch_, min_pitch_, num_pitch_values_);
}

void DepthSensorSpecification::set_min_yaw(double min_yaw) {
  min_yaw_ = min_yaw;
}

void DepthSensorSpecification::set_max_yaw(double max_yaw) {
  max_yaw_ = max_yaw;
}

void DepthSensorSpecification::set_min_pitch(double min_pitch) {
  min_pitch_ = min_pitch;
}

void DepthSensorSpecification::set_max_pitch(double max_pitch) {
  max_pitch_ = max_pitch;
}

void DepthSensorSpecification::set_num_yaw_values(int num_yaw_values) {
  num_yaw_values_ = num_yaw_values;
}

void DepthSensorSpecification::set_num_pitch_values(int num_pitch_values) {
  num_pitch_values_ = num_pitch_values;
}

void DepthSensorSpecification::set_min_range(double min_range) {
  min_range_ = min_range;
}

void DepthSensorSpecification::set_max_range(double max_range) {
  max_range_ = max_range;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
