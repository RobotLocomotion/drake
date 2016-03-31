#ifndef DRAKE_CORE_TEST_PENDULUM_H_
#define DRAKE_CORE_TEST_PENDULUM_H_

#include <iostream>
#include <cmath>

#include "drake/core/Core.h"

namespace drake {
namespace core {
namespace test {

template <typename ScalarType = double>
class PendulumState {  // models the Drake::Vector concept
 public:
  PendulumState(void) : theta(0), thetadot(0) {}
  template <typename Derived>
  PendulumState(const Eigen::MatrixBase<Derived>& x)
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

  const static int RowsAtCompileTime = 2;

  ScalarType theta;
  ScalarType thetadot;
};

template <typename ScalarType = double>
class PendulumInput {
 public:
  PendulumInput(void) : tau(0) {}
  template <typename Derived>
  PendulumInput(const Eigen::MatrixBase<Derived>& x)
      : tau(x(0)) {}

  template <typename Derived>
  PendulumInput& operator=(const Eigen::MatrixBase<Derived>& x) {
    tau = x(0);
    return *this;
  }

  const static int RowsAtCompileTime = 1;

  ScalarType tau;
};

}  // namespace test
}  // namespace core
}  // namespace drake

#endif  // DRAKE_CORE_TEST_PENDULUM_H_
