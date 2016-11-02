#pragma once

#include <cmath>
#include <iostream>
#include <string>

#include "drake/system1/LCMSystem.h"

using namespace std;

template <typename ScalarType = double>
class PendulumState {  // models the drake::Vector concept
 public:
  typedef drake::lcmt_drake_signal LCMMessageType;
  static std::string channel() { return "PendulumState"; }

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

  friend std::string getCoordinateName(const PendulumState<ScalarType>& vec,
                                       unsigned int index) {
    switch (index) {
      case 0:
        return "theta";
      case 1:
        return "thetadot";
    }
    return "error";
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

template <typename ScalarType = double>
class PendulumInput {
 public:
  typedef drake::lcmt_drake_signal LCMMessageType;
  static std::string channel() { return "PendulumInput"; }

  PendulumInput(void) : tau(0) {}

  template <typename Derived>
  PendulumInput(  // NOLINT(runtime/explicit) per drake::Vector.
      const Eigen::MatrixBase<Derived>& x)
      : tau(x(0)) {}

  template <typename Derived>
  PendulumInput& operator=(const Eigen::MatrixBase<Derived>& x) {
    tau = x(0);
    return *this;
  }

  friend std::ostream& operator<<(std::ostream& os, const PendulumInput& x) {
    os << "  tau = " << x.tau << endl;
    return os;
  }

  static const int RowsAtCompileTime = 1;

  ScalarType tau;
};

template <typename ScalarType>
Eigen::Matrix<ScalarType, 1, 1> toEigen(const PendulumInput<ScalarType>& vec) {
  Eigen::Matrix<ScalarType, 1, 1> x;
  x << vec.tau;
  return x;
}

class Pendulum {
 public:
  template <typename ScalarType>
  using InputVector = PendulumInput<ScalarType>;
  template <typename ScalarType>
  using StateVector = PendulumState<ScalarType>;
  template <typename ScalarType>
  using OutputVector = PendulumState<ScalarType>;

  Pendulum()
      : m(1.0),  // kg
        l(.5),   // m
        b(0.1),  // kg m^2 /s
        lc(.5),  // m
        I(.25),  // m*l^2; % kg*m^2
        g(9.81)  // m/s^2
  {}
  virtual ~Pendulum(void) {}

  template <typename ScalarType>
  PendulumState<ScalarType> dynamics(const ScalarType& t,
                                     const PendulumState<ScalarType>& x,
                                     const PendulumInput<ScalarType>& u) const {
    PendulumState<ScalarType> dot;
    dot.theta = x.thetadot;
    dot.thetadot = (u.tau - m * g * lc * sin(x.theta) - b * x.thetadot) / I;
    return dot;
  }

  template <typename ScalarType>
  PendulumState<ScalarType> output(const ScalarType& t,
                                   const PendulumState<ScalarType>& x,
                                   const PendulumInput<ScalarType>& u) const {
    return x;
  }

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return false; }

 public:
  double m, l, b, lc, I,
      g;  // pendulum parameters (initialized in the constructor)
};
