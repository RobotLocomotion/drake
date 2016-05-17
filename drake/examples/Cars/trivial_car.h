#pragma once

#include <cmath>

#include "drake/core/Vector.h"
#include "drake/examples/Cars/gen/simple_car_state.h"

namespace drake {

/// TrivialCar - no physics, no commands, just sits there --- but it does spin.
///
/// output vector:
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
//    heading is defined around the +z axis, so positive-turn-left
/// * velocity (always zero)
///
class TrivialCar {
 public:
  /// Constructs a TrivialCar system that locates the car at position
  /// (@p x, @p y) with initial heading @p heading.  The car will rotate
  /// (changing heading) with a period @p period per revolution.
  TrivialCar(const double x, const double y, const double heading,
             const double period)
      : pose_((Eigen::VectorXd(4) << x, y, heading, 0.).finished()),
        period_(period) {}

  /// @name Implement the Drake::System concept.
  //@{
  template <typename ScalarType>
  using StateVector = Drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = Drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = SimpleCarState<ScalarType>;

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& time,
                                   const StateVector<ScalarType>& state,
                                   const InputVector<ScalarType>& input) const {
    return StateVector<ScalarType>();
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& time,
                                  const StateVector<ScalarType>& state,
                                  const InputVector<ScalarType>&) const {
    Eigen::VectorXd offset(4);
    offset << 0., 0., (M_PI * 2. * time / period_), 0.;
    return toEigen(pose_) + offset;
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return false; }
  //@}

 private:
  const OutputVector<double> pose_;
  const double period_;
};
}  // namespace drake
