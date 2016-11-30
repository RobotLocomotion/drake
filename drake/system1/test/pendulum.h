#pragma once

#include <Eigen/Dense>

namespace drake {
namespace systems {
namespace test {

/// A simple Drake state for unit testing.
template <typename ScalarType = double>
class PendulumState {  // models the drake::Vector concept
 public:
  PendulumState(void) : theta(0), thetadot(0) {}

  template <typename Derived>
  PendulumState(  // NOLINT(runtime/explicit) per drake::Vector.
      const Eigen::MatrixBase<Derived>& x)
      : theta(x(0)), thetadot(x(1)) {}

  template <typename Derived>
  PendulumState& operator=(const Eigen::MatrixBase<Derived>& x) {
    theta = x(0);
    thetadot = x(1);
    return *this;
  }

  friend Eigen::Matrix<ScalarType, 2, 1> toEigen(
      const PendulumState<ScalarType>& vec) {
    Eigen::Matrix<ScalarType, 2, 1> x;
    x << vec.theta, vec.thetadot;
    return x;
  }

  static const int RowsAtCompileTime = 2;

  ScalarType theta;
  ScalarType thetadot;
};

/// A simple Drake input for unit testing.
template <typename ScalarType = double>
class PendulumInput {
 public:
  PendulumInput(void) : tau(0) {}

  template <typename Derived>
  explicit PendulumInput(  // NOLINT(runtime/explicit) per drake::Vector.
      const Eigen::MatrixBase<Derived>& x)
      : tau(x(0)) {}

  template <typename Derived>
  PendulumInput& operator=(const Eigen::MatrixBase<Derived>& x) {
    tau = x(0);
    return *this;
  }

  static const int RowsAtCompileTime = 1;

  ScalarType tau;
};

}  // namespace test
}  // namespace systems
}  // namespace drake
