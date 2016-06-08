#pragma once

/// @file
/// Template method implementations for @see simple_car.h.
/// Most users should only include that file, not this one.
/// For background, @see http://drake.mit.edu/cxx_inl.html.

#include "simple_car.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/examples/Cars/gen/simple_car_state.h"

namespace drake {

template <typename ScalarType>
SimpleCar::StateVector<ScalarType> SimpleCar::dynamics(
    const ScalarType&,
    const StateVector<ScalarType>& state,
    const InputVector<ScalarType>& input) const {
  // Apply simplistic throttle.
  ScalarType new_velocity = state.velocity() +
                            input.throttle() * config_.max_acceleration *
                                config_.velocity_lookahead_time;
  new_velocity = std::min(new_velocity, config_.max_velocity);

  // Apply simplistic brake.
  new_velocity += input.brake() * -config_.max_acceleration *
                  config_.velocity_lookahead_time;
  new_velocity = std::max(new_velocity, 0.);

  // Apply steering.
  ScalarType sane_steering_angle =
      std::max(std::min(std::remainder(input.steering_angle(), 2 * M_PI),
                        config_.max_abs_steering_angle),
               -config_.max_abs_steering_angle);
  ScalarType curvature = std::tan(sane_steering_angle) / config_.wheelbase;

  StateVector<ScalarType> rates;
  rates.set_x(state.velocity() * std::cos(state.heading()));
  rates.set_y(state.velocity() * std::sin(state.heading()));
  rates.set_heading(curvature * state.velocity());
  rates.set_velocity((new_velocity - state.velocity()) * config_.velocity_kp);
  return rates;
}

template <typename ScalarType>
SimpleCar::OutputVector<ScalarType> SimpleCar::output(
    const ScalarType&,
    const StateVector<ScalarType>& state,
    const InputVector<ScalarType>&) const {
  return state;
}

}  // namespace drake
