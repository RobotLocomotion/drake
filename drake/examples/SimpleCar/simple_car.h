#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/drakeSimpleCar_export.h"
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
///
/// output vector: same as state vector.
///
class DRAKESIMPLECAR_EXPORT SimpleCar {
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
    ScalarType new_velocity =
        state.velocity + input.throttle * config_.max_acceleration *
        config_.velocity_lookahead_time;
    new_velocity = std::min(new_velocity, config_.max_velocity);

    // Apply simplistic brake.
    new_velocity += input.brake * -config_.max_acceleration *
                    config_.velocity_lookahead_time;
    new_velocity = std::max(new_velocity, 0.);

    // Apply steering.
    ScalarType sane_steering_angle =
        std::max(std::min(std::remainder(input.steering_angle, 2 * M_PI),
                          config_.max_abs_steering_angle),
                 -config_.max_abs_steering_angle);
    ScalarType curvature = std::tan(sane_steering_angle) / config_.wheelbase;

    StateVector<ScalarType> rates;
    rates.x = state.velocity * std::sin(state.heading);
    rates.y = state.velocity * std::cos(state.heading);
    rates.heading = curvature * state.velocity;
    rates.velocity = (new_velocity - state.velocity) * config_.velocity_kp;
    return rates;
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
