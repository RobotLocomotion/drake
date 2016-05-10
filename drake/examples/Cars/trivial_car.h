#pragma once

#include <cmath>

#include "drake/core/Vector.h"
#include "drake/examples/Cars/trivial_car_state.h"

namespace drake {

/// TrivialCar - no physics, no commands, just sits there --- but it does spin.
///
/// output vector:
/// * position: x, y, heading
///
class TrivialCar {
 public:
  template <typename ScalarType>
  using StateVector = Drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = Drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = TrivialCarState<ScalarType>;

  TrivialCar(const double x, const double y, const double heading,
             const double period)
      : pose_(x, y, heading), period_(period) {}

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& time,
                                   const StateVector<ScalarType>& state,
                                   const InputVector<ScalarType>& input) const {
    return StateVector<ScalarType>();
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& time,
                                  const StateVector<ScalarType>& state,
                                  const InputVector<ScalarType>& input) const {
    typename OutputVector<ScalarType>::EigenType offset;
    offset << 0., 0., (M_PI * 2. * time / period_);
    return toEigen(pose_) + offset;
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return false; }

 private:
  const OutputVector<double> pose_;
  const double period_;
};
}
