#pragma once

#include <cmath>
#include <iostream>

#include "drake/math/gradient.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/system1/System.h"
#include "drake/util/drakeGeometryUtil.h"

template <typename ScalarType = double>
class QuadrotorState {  // models the drake::Vector concept
 public:
  QuadrotorState(void)
      : x(0),
        y(0),
        z(0),
        roll(0),
        pitch(0),
        yaw(0),
        xdot(0),
        ydot(0),
        zdot(0),
        rolldot(0),
        pitchdot(0),
        yawdot(0) {}

  template <typename Derived>
  QuadrotorState(  // NOLINT(runtime/explicit) per drake::Vector.
      const Eigen::MatrixBase<Derived>& state)
      : x(state(0)),
        y(state(1)),
        z(state(2)),
        roll(state(3)),
        pitch(state(4)),
        yaw(state(5)),
        xdot(state(6)),
        ydot(state(7)),
        zdot(state(8)),
        rolldot(state(9)),
        pitchdot(state(10)),
        yawdot(state(11)) {}

  template <typename Derived>
  QuadrotorState& operator=(const Eigen::MatrixBase<Derived>& state) {
    x = state(0);
    y = state(1);
    z = state(2);
    roll = state(3);
    pitch = state(4);
    yaw = state(5);
    xdot = state(6);
    ydot = state(7);
    zdot = state(8);
    rolldot = state(9);
    pitchdot = state(10);
    yawdot = state(11);

    return *this;
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  const QuadrotorState& state) {
    using namespace std;
    os << "  x = " << state.x << endl;
    os << "  y = " << state.y << endl;
    os << "  z = " << state.z << endl;
    os << "  roll = " << state.roll << endl;
    os << "  pitch = " << state.pitch << endl;
    os << "  yaw = " << state.yaw << endl;
    os << "  xdot = " << state.xdot << endl;
    os << "  ydot = " << state.ydot << endl;
    os << "  zdot = " << state.zdot << endl;
    os << "  rolldot = " << state.rolldot << endl;
    os << "  pitchdot = " << state.pitchdot << endl;
    os << "  yawdot = " << state.yawdot << endl;

    return os;
  }

  enum { RowsAtCompileTime = 12 };
  std::size_t size() { return 12; }

  ScalarType x, y, z, roll, pitch, yaw, xdot, ydot, zdot, rolldot, pitchdot,
      yawdot;
};

template <typename ScalarType>
Eigen::Matrix<ScalarType, 12, 1> toEigen(
    const QuadrotorState<ScalarType>& vec) {
  Eigen::Matrix<ScalarType, 12, 1> state;
  state << vec.x, vec.y, vec.z, vec.roll, vec.pitch, vec.yaw, vec.xdot,
      vec.ydot, vec.zdot, vec.rolldot, vec.pitchdot, vec.yawdot;
  return state;
}

template <typename ScalarType = double>
class QuadrotorInput {
 public:
  QuadrotorInput(void) : w1(0), w2(0), w3(0), w4(0) {}

  template <typename Derived>
  QuadrotorInput(  // NOLINT(runtime/explicit) per drake::Vector.
      const Eigen::MatrixBase<Derived>& x)
      : w1(x(0)), w2(x(1)), w3(x(2)), w4(x(3)) {}

  template <typename Derived>
  QuadrotorInput& operator=(const Eigen::MatrixBase<Derived>& input) {
    w1 = input(0);
    w2 = input(1);
    w3 = input(2);
    w4 = input(3);
    return *this;
  }

  friend std::ostream& operator<<(std::ostream& os, const QuadrotorInput& u) {
    using namespace std;
    os << "  w1 = " << u.w1 << endl;
    os << "  w2 = " << u.w2 << endl;
    os << "  w3 = " << u.w3 << endl;
    os << "  w4 = " << u.w4 << endl;
    return os;
  }

  enum { RowsAtCompileTime = 4 };
  std::size_t size() { return 4; }

  ScalarType w1, w2, w3, w4;
};

template <typename ScalarType>
Eigen::Matrix<ScalarType, 4, 1> toEigen(const QuadrotorInput<ScalarType>& vec) {
  Eigen::Matrix<ScalarType, 4, 1> input;
  input << vec.w1, vec.w2, vec.w3, vec.w4;
  return input;
}

class Quadrotor {
 public:
  template <typename ScalarType>
  using StateVector = QuadrotorState<ScalarType>;
  template <typename ScalarType>
  using OutputVector = QuadrotorState<ScalarType>;
  template <typename ScalarType>
  using InputVector = QuadrotorInput<ScalarType>;

  Quadrotor() : g(9.81), L(0.1750), m(0.5), I(Eigen::Matrix3d::Identity()) {
    I(0, 0) = 0.0023;
    I(1, 1) = 0.0023;
    I(2, 2) = 0.004;
  }
  virtual ~Quadrotor(void) {}

  template <typename Scalar>
  QuadrotorState<Scalar> dynamics(const Scalar& t,
                                  const QuadrotorState<Scalar>& x,
                                  const QuadrotorInput<Scalar>& u) const {
    Eigen::Matrix<Scalar, 12, 1> xvec = toEigen(x);
    Eigen::Matrix<Scalar, 3, 1> rpy(x.roll, x.pitch, x.yaw);
    Eigen::Matrix<Scalar, 3, 3> R = drake::math::rpy2rotmat(rpy);

    Scalar F1 = kf * u.w1;
    Scalar F2 = kf * u.w2;
    Scalar F3 = kf * u.w3;
    Scalar F4 = kf * u.w4;

    Scalar M1 = km * u.w1;
    Scalar M2 = km * u.w2;
    Scalar M3 = km * u.w3;
    Scalar M4 = km * u.w4;

    Eigen::Matrix<Scalar, 3, 1> gvec(0, 0, -m * g);
    Eigen::Matrix<Scalar, 3, 1> forcevec(0, 0, F1 + F2 + F3 + F4);
    Eigen::Matrix<Scalar, 3, 1> xyz_ddot = (1.0 / m) * (gvec + R * forcevec);

    Eigen::Matrix<Scalar, 3, 1> rpydot(x.rolldot, x.pitchdot, x.yawdot);
    Eigen::Matrix<Scalar, 3, 1> pqr;
    rpydot2angularvel(rpy, rpydot, pqr);
    pqr = R.adjoint() * pqr;

    Eigen::Matrix<Scalar, 3, 1> pqr_dot_term1;
    pqr_dot_term1 << L * (F2 - F4), L * (F3 - F1), (M1 - M2 + M3 - M4);

    Eigen::Matrix<Scalar, 3, 1> pqr_dot =
        I.ldlt().solve(pqr_dot_term1 - pqr.cross(I * pqr));
    Eigen::Matrix<Scalar, 3, 3> Phi;
    typename drake::math::Gradient<Eigen::Matrix<Scalar, 3, 3>, 3>::type dPhi;
    typename drake::math::Gradient<Eigen::Matrix<Scalar, 3, 3>, 3, 2>::type*
        ddPhi = nullptr;
    angularvel2rpydotMatrix(rpy, Phi, &dPhi, ddPhi);

    Eigen::Matrix<Scalar, 9, 3> drpy2drotmat = drake::math::drpy2rotmat(rpy);
    Eigen::Matrix<Scalar, 9, 1> Rdot_vec;
    Rdot_vec = drpy2drotmat * rpydot;
    Eigen::Matrix<Scalar, 3, 3> Rdot =
        Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> >(
            Rdot_vec.data(), 3, 3);
    Eigen::Matrix<Scalar, 9, 1> dPhi_x_rpydot_vec;
    dPhi_x_rpydot_vec = dPhi * rpydot;
    Eigen::Matrix<Scalar, 3, 3> dPhi_x_rpydot =
        Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> >(
            dPhi_x_rpydot_vec.data(), 3, 3);
    Eigen::Matrix<Scalar, 3, 1> rpy_ddot =
        Phi * R * pqr_dot + dPhi_x_rpydot * R * pqr + Phi * Rdot * pqr;

    Eigen::Matrix<Scalar, 12, 1> xdotvec;
    xdotvec << xvec.template tail<6>(), xyz_ddot, rpy_ddot;
    QuadrotorState<Scalar> xdot(xdotvec);
    return xdot;
  }

  template <typename ScalarType>
  QuadrotorState<ScalarType> output(const ScalarType& t,
                                    const QuadrotorState<ScalarType>& x,
                                    const QuadrotorInput<ScalarType>& u) const {
    return x;
  }

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return false; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double g, L, m;
  Eigen::Matrix3d I;

 private:
  const double kf = 1;
  const double km = 0.0245;
};
