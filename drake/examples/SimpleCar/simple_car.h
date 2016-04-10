#ifndef DRAKE_EXAMPLES_SIMPLECAR_SIMPLE_CAR_H_
#define DRAKE_EXAMPLES_SIMPLECAR_SIMPLE_CAR_H_

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/examples/SimpleCar/driving_command.h"
#include "drake/examples/SimpleCar/simple_car_state.h"
#include "lcmtypes/drake/lcmt_simple_car_config_t.hpp"

namespace Drake {

/// SimpleCar -- model an idealized response to driving commands, neglecting
/// all physics.
///
/// configuration:
/// * see lcmt_simple_car_config_t
///
/// input vector:
/// * steering angle (virtual center wheel angle, with some limits)
/// * throttle (0-1)
/// * brake (0-1)
///
/// state vector (planar for now):
/// * position: x, y, heading
/// * velocity
/// * steering angle
///
/// output vector: same as state vector.
///
class SimpleCar {
 public:
  template <typename ScalarType>
  using StateVector = SimpleCarState<ScalarType>;
  template <typename ScalarType>
  using InputVector = DrivingCommand<ScalarType>;
  template <typename ScalarType>
  using OutputVector = SimpleCarState<ScalarType>;

  explicit SimpleCar(
      const drake::lcmt_simple_car_config_t& config = kDefaultConfig)
      : config_(config) {}

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& time,
                                   const StateVector<ScalarType>& state,
                                   const InputVector<ScalarType>& input) const {
    // Apply simplistic throttle.
    ScalarType velocity =
        state.velocity + input.throttle * config_.max_acceleration;
    velocity = std::min(velocity, config_.max_velocity);

    // Apply simplistic brake.
    velocity += input.brake * -config_.max_acceleration;
    velocity = std::max(velocity, 0.);

    // Apply steering.
    ScalarType sane_steering_angle =
        std::max(std::min(std::remainder(input.steering_angle, 2 * M_PI),
                          config_.max_abs_steering_angle),
                 -config_.max_abs_steering_angle);
    ScalarType curvature = std::tan(sane_steering_angle) / config_.wheelbase;

    StateVector<ScalarType> delta;
    delta.steering_angle =
        std::remainder(sane_steering_angle - state.steering_angle, 2 * M_PI);
    delta.x = state.velocity * std::sin(state.heading);
    delta.y = state.velocity * std::cos(state.heading);
    delta.heading = curvature * velocity;
    delta.velocity = velocity - state.velocity;
    return delta;
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& time,
                                  const StateVector<ScalarType>& state,
                                  const InputVector<ScalarType>& input) const {
    return state;
  }

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return false; }

  const drake::lcmt_simple_car_config_t& config() const { return config_; }

  static const drake::lcmt_simple_car_config_t kDefaultConfig;

 private:
  const drake::lcmt_simple_car_config_t config_;
};
}

#endif  // DRAKE_EXAMPLES_SIMPLECAR_SIMPLE_CAR_H_
