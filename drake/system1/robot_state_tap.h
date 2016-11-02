#pragma once

#include "drake/system1/vector.h"

namespace drake {

/// Implements a Drake System (@see drake/system1/System.h) that
/// saves the latest input as a local variable and provides an accessor
/// to it. This is useful for testing the final configuration of the robot in a
/// unit test. It otherwise merely passes through its input to output.
template <template <typename> class Vector>
class RobotStateTap {
 public:
  /// Creates a RobotStateTap.
  RobotStateTap() {}

  // Noncopyable.
  RobotStateTap(const RobotStateTap&) = delete;
  RobotStateTap& operator=(const RobotStateTap&) = delete;

  /// @name Implement the Drake System concept.
  //@{

  template <typename ScalarType>
  using StateVector = drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = Vector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = Vector<ScalarType>;

  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double& t, const StateVector<double>& x,
                              const InputVector<double>& u) {
    u_ = toEigen(u);
    return u;
  }

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return true; }

  //@}

  const InputVector<double>& get_input_vector() { return u_; }

 private:
  InputVector<double> u_;
};

}  // namespace drake
