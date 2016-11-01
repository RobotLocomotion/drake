#pragma once

#include <cmath>
#include <iostream>
#include <string>

#include <Eigen/Core>

using namespace std;

template <typename ScalarType = double>
class AcrobotState {  // models the drake::Vector concept
 public:
  AcrobotState(void) : shoulder(0), elbow(0), shoulder_dot(0), elbow_dot(0) {}

  template <typename Derived>
  AcrobotState(  // NOLINT(runtime/explicit) per drake::Vector.
      const Eigen::MatrixBase<Derived>& x)
      : shoulder(x(0)), elbow(x(1)), shoulder_dot(x(2)), elbow_dot(x(3)) {}

  template <typename Derived>
  AcrobotState& operator=(const Eigen::MatrixBase<Derived>& x) {
    shoulder = x(0);
    elbow = x(1);
    shoulder_dot = x(2);
    elbow_dot = x(3);
    return *this;
  }

  friend std::string getCoordinateName(const AcrobotState<ScalarType>& vec,
                                       unsigned int index) {
    switch (index) {
      case 0:
        return "shoulder";
      case 1:
        return "elbow";
      case 2:
        return "shoulder_dot";
      case 3:
        return "elbow_dot";
    }
    return "error";
  }
  friend Eigen::Matrix<ScalarType, 4, 1> toEigen(
      const AcrobotState<ScalarType>& vec) {
    Eigen::Matrix<ScalarType, 4, 1> x;
    x << vec.shoulder, vec.elbow, vec.shoulder_dot, vec.elbow_dot;
    return x;
  }

  static const int RowsAtCompileTime = 4;

  ScalarType shoulder, elbow, shoulder_dot, elbow_dot;
};

template <typename ScalarType = double>
class AcrobotInput {
 public:
  AcrobotInput(void) : tau(0) {}

  template <typename Derived>
  AcrobotInput(  // NOLINT(runtime/explicit) per drake::Vector.
      const Eigen::MatrixBase<Derived>& x)
      : tau(x(0)) {}

  template <typename Derived>
  AcrobotInput& operator=(const Eigen::MatrixBase<Derived>& x) {
    tau = x(0);
    return *this;
  }

  friend std::ostream& operator<<(std::ostream& os, const AcrobotInput& x) {
    os << "  tau = " << x.tau << endl;
    return os;
  }

  static const int RowsAtCompileTime = 1;

  ScalarType tau;
};

template <typename ScalarType>
Eigen::Matrix<ScalarType, 1, 1> toEigen(const AcrobotInput<ScalarType>& vec) {
  Eigen::Matrix<ScalarType, 1, 1> x;
  x << vec.tau;
  return x;
}

class Acrobot {
 public:
  template <typename ScalarType>
  using InputVector = AcrobotInput<ScalarType>;
  template <typename ScalarType>
  using StateVector = AcrobotState<ScalarType>;
  template <typename ScalarType>
  using OutputVector = AcrobotState<ScalarType>;

  Acrobot()
      : m1(1.0),
        m2(1.0),  // kg
        l1(1.0),
        l2(2.0),  // m
        lc1(0.5),
        lc2(1.0),  // m
        Ic1(.083),
        Ic2(.33),  // kg*m^2
        b1(0.1),
        b2(0.1),  // kg m^2 /s
        g(9.81)   // m/s^2
  {}
  virtual ~Acrobot(void) {}

  template <typename ScalarType>
  void manipulatorDynamics(
      const AcrobotState<ScalarType>& x,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix<ScalarType, 2, 2>& H,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix<ScalarType, 2, 1>& C,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix<ScalarType, 2, 1>& B) const {
    double I1 = Ic1 + m1 * lc1 * lc1;
    double I2 = Ic2 + m2 * lc2 * lc2;
    double m2l1lc2 = m2 * l1 * lc2;  // occurs often!

    auto c2 = cos(x.elbow);
    auto s1 = sin(x.shoulder), s2 = sin(x.elbow);
    auto s12 = sin(x.shoulder + x.elbow);

    auto h12 = I2 + m2l1lc2 * c2;
    H << I1 + I2 + m2* l1* l1 + 2 * m2l1lc2* c2, h12, h12, I2;
    //    std::cout << "H = " << H << std::endl;

    C << -2 * m2l1lc2* s2* x.elbow_dot* x.shoulder_dot +
             -m2l1lc2* s2* x.elbow_dot* x.elbow_dot,
        m2l1lc2* s2* x.shoulder_dot* x.shoulder_dot;

    // add in G terms
    C(0) += g * m1 * lc1 * s1 + g * m2 * (l1 * s1 + lc2 * s12);
    C(1) += g * m2 * lc2 * s12;

    // damping terms
    C(0) += b1 * x.shoulder_dot;
    C(1) += b2 * x.elbow_dot;
    //    std::cout << "C = " << C << std::endl;

    // input matrix
    B << 0.0, 1.0;
  }

  template <typename ScalarType>
  AcrobotState<ScalarType> dynamics(const ScalarType& t,
                                    const AcrobotState<ScalarType>& x,
                                    const AcrobotInput<ScalarType>& u) const {
    Eigen::Matrix<ScalarType, 2, 2> H;
    Eigen::Matrix<ScalarType, 2, 1> C;
    Eigen::Matrix<double, 2, 1> B;  // note: intentionally hard-coding the fact
                                    // that it's a constant here

    manipulatorDynamics(x, H, C, B);
    Eigen::Matrix<ScalarType, 4, 1> xvec = toEigen(x);

    Eigen::Matrix<ScalarType, 4, 1> xdotvec;
    xdotvec.topRows(2) = xvec.bottomRows(2);
    xdotvec.bottomRows(2) = H.inverse() * (B * u.tau - C);
    return AcrobotState<ScalarType>(xdotvec);
  }

  template <typename ScalarType>
  AcrobotState<ScalarType> output(const ScalarType& t,
                                  const AcrobotState<ScalarType>& x,
                                  const AcrobotInput<ScalarType>& u) const {
    return x;
  }

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return false; }

 public:
  double m1, m2, l1, l2, lc1, lc2, Ic1, Ic2, b1, b2,
      g;  // parameters (initialized in the constructor)
};
