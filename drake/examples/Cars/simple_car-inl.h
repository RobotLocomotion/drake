#pragma once

/// @file
/// Templated method implementation for @see simple_car.h.
/// Most users should only include that file, not this one.

#include "simple_car.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/drakeSimpleCar_export.h"
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
  new_velocity = std::min(
      new_velocity, static_cast<ScalarType>(config_.max_velocity));

  // Apply simplistic brake.
  new_velocity += input.brake() * -config_.max_acceleration *
                  config_.velocity_lookahead_time;
  new_velocity = std::max(new_velocity, static_cast<ScalarType>(0.));

  // Apply steering.
  ScalarType sane_steering_angle = remainder(
      input.steering_angle(), static_cast<ScalarType>(2 * M_PI));
  sane_steering_angle = std::min(
      sane_steering_angle,
      static_cast<ScalarType>(config_.max_abs_steering_angle));
  sane_steering_angle = std::max(
      sane_steering_angle,
      static_cast<ScalarType>(-config_.max_abs_steering_angle));
  ScalarType curvature = tan(sane_steering_angle) / config_.wheelbase;

  StateVector<ScalarType> rates;
  rates.set_x(state.velocity() * cos(state.heading()));
  rates.set_y(state.velocity() * sin(state.heading()));
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

/// At outermost scope, invoke this macro to explicitly instantiate
/// all ScalarType-using definitions that are declared by simple_car.h.
#define DRAKE_SIMPLE_CAR_INSTANTIATE_SCALAR_TYPE(ScalarType)            \
template DRAKESIMPLECAR_EXPORT                                          \
::drake::SimpleCar::StateVector<ScalarType>                             \
drake::SimpleCar::dynamics(                                             \
      const ScalarType&,                                                \
      const ::drake::SimpleCar::StateVector<ScalarType>&,               \
      const ::drake::SimpleCar::InputVector<ScalarType>&) const;        \
template DRAKESIMPLECAR_EXPORT                                          \
::drake::SimpleCar::OutputVector<ScalarType>                            \
drake::SimpleCar::output(                                               \
    const ScalarType&,                                                  \
    const ::drake::SimpleCar::StateVector<ScalarType>&,                 \
    const ::drake::SimpleCar::InputVector<ScalarType>&) const;
