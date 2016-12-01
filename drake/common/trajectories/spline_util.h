#pragma once

#include <iostream>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/math/quaternion.h"

template <typename Scalar, int rows, int cols>
bool CheckSplineInputs(
    const std::vector<double> &T,
    const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y) {
  bool ret = T.size() == Y.size();
  if (!ret) {
    std::cerr << "T size doesnt match Y size\n";
    return ret;
  }

  ret &= (T.size() >= 2);
  if (!ret) {
    std::cerr << "T size < 2\n";
    return ret;
  }

  for (size_t t = 0; t < T.size() - 1; t++) {
    ret &= (Y[t].rows() == Y[t + 1].rows() && Y[t].cols() == Y[t + 1].cols());
    if (!ret) {
      std::cerr << "Y dimension is not consistent\n";
      return ret;
    }

    ret &= T[t] < T[t + 1];
    if (!ret) {
      std::cerr << "T is not strictly increasing " << T[t] << " " << T[t + 1] << std::endl;
      return ret;
    }
  }
  return ret;
}

template <typename Scalar, int rows, int cols>
PiecewisePolynomial<Scalar> GenerateLinearSpline(
    const std::vector<double> &T,
    const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y) {
  if (!CheckSplineInputs(T, Y)) {
    throw std::runtime_error("invalid spline inputs");
  }

  size_t N = T.size();
  std::vector<Eigen::Matrix<Polynomial<Scalar>, Eigen::Dynamic, Eigen::Dynamic>>
      polynomials(N - 1);

  for (size_t t = 0; t < N - 1; t++) {
    polynomials[t].resize(Y[t].rows(), Y[t].cols());
  }

  for (int j = 0; j < Y[0].rows(); j++) {
    for (int k = 0; k < Y[0].cols(); k++) {
      for (size_t t = 0; t < N - 1; t++) {
        polynomials[t](j, k) = Polynomial<Scalar>(Eigen::Vector2d(
            Y[t](j, k), (Y[t + 1](j, k) - Y[t](j, k)) / (T[t + 1] - T[t])));
      }
    }
  }
  return PiecewisePolynomial<double>(polynomials, T);
}

// copied from matlab source code and wikipedia
template <typename Scalar, int rows, int cols>
PiecewisePolynomial<Scalar> GeneratePCHIPSpline(
    const std::vector<double> &T,
    const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y,
    const Eigen::Matrix<Scalar, rows, cols> &dY0,
    const Eigen::Matrix<Scalar, rows, cols> &dY1) {
  if (!CheckSplineInputs(T, Y)) {
    throw std::runtime_error("invalid spline inputs");
  }

  size_t N = T.size();
  if (N == 2) {
    return GenerateLinearSpline(T, Y);
  }

  std::vector<Eigen::Matrix<Polynomial<Scalar>, Eigen::Dynamic, Eigen::Dynamic>>
      polynomials(N - 1);
  std::vector<Eigen::Matrix<Scalar, rows, cols>> m(N - 1);
  std::vector<Eigen::Matrix<Scalar, rows, cols>> c1(N);
  std::vector<double> dt(N - 1);

  for (size_t t = 0; t < N - 1; t++) {
    dt[t] = T[t + 1] - T[t];
    m[t] = (Y[t + 1] - Y[t]) / dt[t];
    polynomials[t].resize(Y[t].rows(), Y[t].cols());
  }

  for (int j = 0; j < Y[0].rows(); j++) {
    for (int k = 0; k < Y[0].cols(); k++) {
      for (size_t t = 0; t < dt.size() - 1; t++) {
        if (m[t](j, k) * m[t + 1](j, k) <= 0) {
          c1[t + 1](j, k) = 0;
        } else {
          Scalar common = dt[t] + dt[t + 1];
          c1[t + 1](j, k) = 3 * common / ((common + dt[t + 1]) / m[t](j, k) +
                                          (common + dt[t]) / m[t + 1](j, k));
        }
      }

      // fix end points' slopes
      int n = N - 1;
      c1[0](j, k) = dY0(j, k);
      c1[n](j, k) = dY1(j, k);

      for (size_t t = 0; t < N - 1; t++) {
        Eigen::Vector4d coeffs;
        coeffs[0] = Y[t](j, k);
        coeffs[1] = c1[t](j, k);
        coeffs[3] = ((c1[t + 1](j, k) - c1[t](j, k)) * dt[t] / 2 -
                     Y[t + 1](j, k) + Y[t](j, k) + c1[t](j, k) * dt[t]) *
                    2 / (dt[t] * dt[t] * dt[t]);
        coeffs[2] = (Y[t + 1](j, k) - Y[t](j, k) - c1[t](j, k) * dt[t] -
                     coeffs[3] * (dt[t] * dt[t] * dt[t])) /
                    (dt[t] * dt[t]);
        polynomials[t](j, k) = Polynomial<Scalar>(coeffs);
      }
    }
  }

  return PiecewisePolynomial<Scalar>(polynomials, T);
}

template <typename Scalar, int rows, int cols>
PiecewisePolynomial<Scalar> GenerateCubicSpline(
    const std::vector<double> &T,
    const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y,
    const Eigen::Matrix<Scalar, rows, cols> &dydt0,
    const Eigen::Matrix<Scalar, rows, cols> &dydt1) {
  if (!CheckSplineInputs(T, Y)) {
    throw std::runtime_error("invalid spline inputs");
  }
  assert(dydt0.rows() == dydt1.rows() && dydt0.rows() == Y[0].rows());
  assert(dydt0.cols() == dydt1.cols() && dydt0.cols() == Y[0].cols());

  size_t N = T.size();
  std::vector<Eigen::Matrix<Polynomial<Scalar>, Eigen::Dynamic, Eigen::Dynamic>>
      polynomials(N - 1);

  for (size_t i = 0; i < N - 1; i++) {
    polynomials[i].resize(Y[i].rows(), Y[i].cols());
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4 * (N - 1), 4 * (N - 1));
  Eigen::VectorXd b = Eigen::VectorXd::Zero(4 * (N - 1));
  Eigen::VectorXd solution;

  for (int j = 0; j < Y[0].rows(); j++) {
    for (int k = 0; k < Y[0].cols(); k++) {
      // general case
      size_t rowIdx = 0;

      for (size_t i = 0; i < N - 1; i++) {
        Scalar duration = T[i + 1] - T[i];

        // y_i(x_i) = a0i = Y[i]
        A(rowIdx, 4 * i) = 1;
        b(rowIdx++) = Y[i](j, k);

        // y_i(x_{i+1}) = y_{i+1}(x_{i}) =>
        // a0i + a1i*(x_{i+1} - x_i) + a2i(x_{i+1} - x_i)^2 + a3i(x_{i+1} -
        // x_i)^3 = a0{i+1}
        A(rowIdx, 4 * i + 0) = 1;
        A(rowIdx, 4 * i + 1) = duration;
        A(rowIdx, 4 * i + 2) = duration * duration;
        A(rowIdx, 4 * i + 3) = duration * duration * duration;
        if (i != N - 2) {
          A(rowIdx++, 4 * (i + 1)) = -1;
        } else {
          b(rowIdx++) = Y[N - 1](j, k);
        }

        // y_i'(x_{i+1}) = y_{i+1}'(x_{i}) =>
        // a1i + 2*a2i(x_{i+1} - x_i) + 3*a3i(x_{i+1} - x_i)^2 = a1{i+1}
        if (i != N - 2) {
          A(rowIdx, 4 * i + 1) = 1;
          A(rowIdx, 4 * i + 2) = 2 * duration;
          A(rowIdx, 4 * i + 3) = 3 * duration * duration;
          A(rowIdx++, 4 * (i + 1) + 1) = -1;
        }

        if (i != N - 2) {
          // y_i''(x_{i+1}) = y_{i+1}''(x_{i}) =>
          // 2*a2i + 6*a3i(x_{i+1} - x_i) = 2*a2{i+1}
          A(rowIdx, 4 * i + 2) = 2;
          A(rowIdx, 4 * i + 3) = 6 * duration;
          A(rowIdx++, 4 * (i + 1) + 2) = -2;
        }
      }

      // dx0
      A(rowIdx, 1) = 1;
      b(rowIdx++) = dydt0(j, k);

      // dx1
      A(rowIdx, 4 * (N - 2) + 1) = 1;
      A(rowIdx, 4 * (N - 2) + 2) = 2 * (T[N - 1] - T[N - 2]);
      A(rowIdx, 4 * (N - 2) + 3) =
          3 * (T[N - 1] - T[N - 2]) * (T[N - 1] - T[N - 2]);
      b(rowIdx++) = dydt1(j, k);

      assert(rowIdx == 4 * (N - 1));
      auto decomposition = A.colPivHouseholderQr();
      assert(decomposition.isInvertible());
      solution = decomposition.solve(b);

      for (size_t i = 0; i < N - 1; i++) {
        polynomials[i](j, k) = Polynomial<Scalar>(solution.segment<4>(4 * i));
      }
    }
  }

  return PiecewisePolynomial<Scalar>(polynomials, T);
}

template <typename Scalar, int rows, int cols>
PiecewisePolynomial<Scalar> GenerateCubicSpline(
    const std::vector<double> &T,
    const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y) {
  if (!CheckSplineInputs(T, Y)) {
    throw std::runtime_error("invalid spline inputs");
  }
  size_t N = T.size();
  Eigen::Matrix<Scalar, rows, cols> dydt0, dydt1;
  dydt0 = (Y[1] - Y[0]) / (T[1] - T[0]);
  dydt1 = (Y[N - 1] - Y[N - 2]) / (T[N - 1] - T[N - 2]);
  return GenerateCubicSpline(T, Y, dydt0, dydt1);
}

template <typename Scalar> Eigen::Matrix<Scalar, 4, 1> GetCubicSplineCoeffs(double dt, Scalar y0, Scalar y1, Scalar yd0, Scalar yd1) {
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  Scalar c4 = y0;
  Scalar c3 = yd0;
  Scalar c1 = 1. / dt2 * (yd1 - c3 -
      2. / dt * (y1 - c4 - dt * c3));
  Scalar c2 = 1. / dt2 * (y1 - c4 - dt * c3 - dt3 * c1);
  return Eigen::Matrix<Scalar, 4, 1>(c4, c3, c2, c1);
}

template <typename Scalar, int rows, int cols>
PiecewisePolynomial<Scalar> GenerateCubicSpline(
    const std::vector<double> &T,
    const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y,
    const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Ydot) {
  if (!CheckSplineInputs(T, Y) || Y.size() != Ydot.size()) {
    if (Y.size() != Ydot.size())
      std::cerr << "Y size doesnt match Ydot size\n";

    throw std::runtime_error("invalid spline inputs");
  }

  size_t N = T.size();
  std::vector<Eigen::Matrix<Polynomial<Scalar>, Eigen::Dynamic, Eigen::Dynamic>>
      polynomials(N - 1);

  for (size_t t = 0; t < N - 1; t++) {
    if (Y[t].rows() != Ydot[t].rows() || Y[t].cols() != Ydot[t].cols()) {
      std::cerr << "Y dim doesnt match Ydot dim\n";
      throw std::runtime_error("invalid spline inputs");
    }

    polynomials[t].resize(Y[t].rows(), Y[t].cols());
    for (size_t i = 0; i < Y[t].rows(); i++) {
      for (size_t j = 0; j < Y[t].cols(); j++) {
        polynomials[t](i, j) = Polynomial<Scalar>(GetCubicSplineCoeffs(T[t + 1] - T[t], Y[t](i, j), Y[t+1](i, j), Ydot[t](i, j), Ydot[t + 1](i, j)));
      }
    }
  }

  return PiecewisePolynomial<Scalar>(polynomials, T);
}

// poses and vels are task space pose and vel
// QPLocomotionPlanSettings.m: BodyMotionData.from_body_xyzexp_and_xyzexpdot
// -> BodyMotionData.m: pchipDeriv
//    -> drake/util/pchipDeriv.m
template <typename Scalar>
PiecewisePolynomial<Scalar> GenerateCubicCartesianSpline(
    const std::vector<double> &times,
    const std::vector<Eigen::Matrix<Scalar, 7, 1>> &poses,
    const std::vector<Eigen::Matrix<Scalar, 7, 1>> &vels) {
  assert(times.size() == poses.size());
  assert(times.size() == vels.size());
  assert(times.size() >= 2);

  size_t T = times.size();
  std::vector<Eigen::Matrix<Scalar, 6, 1>> expmap(poses.size());
  std::vector<Eigen::Matrix<Scalar, 6, 1>> expmap_dot(poses.size());

  Eigen::Matrix<Scalar, 4, Eigen::Dynamic> quat(4, T);
  Eigen::Matrix<Scalar, 4, Eigen::Dynamic> quat_dot(4, T);
  Eigen::Matrix<Scalar, 3, Eigen::Dynamic> exp(3, T);
  Eigen::Matrix<Scalar, 3, Eigen::Dynamic> exp_dot(3, T);

  for (size_t t = 0; t < T; t++) {
    quat.col(t) = poses[t].tail(4);
    quat_dot.col(t) = vels[t].tail(4);
  }
  quat2expmapSequence(quat, quat_dot, exp, exp_dot);

  for (size_t t = 0; t < times.size(); t++) {
    expmap[t].head(3) = poses[t].head(3);
    expmap[t].tail(3) = exp.col(t);
    expmap_dot[t].head(3) = vels[t].head(3);
    expmap_dot[t].tail(3) = exp_dot.col(t);
  }

  // need to do the closestExpmap
  for (size_t t = 1; t < times.size(); t++) {
    Eigen::Matrix<Scalar, 3, 1> w_closest;
    Eigen::Matrix<Scalar, 3, 3> dw_closest_dw;
    Eigen::Matrix<Scalar, 3, 1> w1 = expmap[t - 1].tail(3);
    Eigen::Matrix<Scalar, 3, 1> w2 = expmap[t].tail(3);
    closestExpmap1(w1, w2, w_closest, dw_closest_dw);
    expmap[t].tail(3) = w_closest;
    expmap_dot[t].tail(3) = dw_closest_dw * expmap_dot[t].tail(3);
  }

  return GenerateCubicSpline(times, expmap, expmap_dot);
}
