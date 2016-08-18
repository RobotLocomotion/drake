#pragma once

#include <stdexcept>

#include "drake/drakeCars_export.h"
#include "drake/examples/Cars/curve2.h"
#include "drake/examples/Cars/system1_cars_vectors.h"

namespace drake {

/// TrajectoryCar -- a car that follows a pre-established trajectory,
/// neglecting all physics.
///
/// state vector
/// * none
///
/// input vector:
/// * none
///
/// output vector (planar for now):
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
//    heading is defined around the +z axis, so positive-turn-left
/// * velocity
///
class DRAKECARS_EXPORT TrajectoryCar {
 public:
  /// Constructs a TrajectoryCar system that traces the given @p curve,
  /// at the given constant @p speed, starting at the given @p start_time.
  /// Throws an error if the curve is empty (a zero @p path_length).
  TrajectoryCar(const Curve2<double>& curve, double speed, double start_time)
      : curve_(curve), speed_(speed), start_time_(start_time) {
    if (curve_.path_length() == 0.0) {
      throw std::invalid_argument{"empty curve"};
    }
  }

  // Noncopyable.
  TrajectoryCar(const TrajectoryCar&) = delete;
  TrajectoryCar& operator=(const TrajectoryCar&) = delete;

  /// @name Implement the Drake System concept.
  //@{

  template <typename ScalarType>
  using StateVector = drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = SimpleCarState1<ScalarType>;

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& time,
                                   const StateVector<ScalarType>& state,
                                   const InputVector<ScalarType>& input) const {
    // No state means no dynamics.
    return StateVector<ScalarType>{};
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& time,
                                  const StateVector<ScalarType>& state,
                                  const InputVector<ScalarType>&) const {
    // N.B. Never use InputVector data, because we are !isDirectFeedthrough.

    // Trace the curve at a fixed speed.
    const double distance = speed_ * (time - start_time_);
    const Curve2<double>::PositionResult pose = curve_.GetPosition(distance);

    // Convert pose to OutputVector.
    OutputVector<ScalarType> result{};
    result.set_x(pose.position[0]);
    result.set_y(pose.position[1]);
    result.set_heading(std::atan2(pose.position_dot[1], pose.position_dot[0]));
    result.set_velocity(speed_);
    return result;
  }

  bool isTimeVarying() const {
    // Our dynamics() does NOT depend on time.
    // Our output() DOES depend on time.
    return true;
  }

  bool isDirectFeedthrough() const {
    // Our output() does not depend our InputVector data.
    return false;
  }

  //@}

 private:
  const Curve2<double> curve_;
  const double speed_;
  const double start_time_;
};

}  // namespace drake
