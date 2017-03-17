#include "drake/common/trajectories/piecewise_polynomial.h"

#include <iostream>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"

using std::default_random_engine;

namespace drake {
namespace {

// Computes the maximum or minimum velocity in interval [0, t].
// This assumes `vel` is a second order polynomial.
template <typename CoefficientType>
CoefficientType ComputeExtremeVel(const Polynomial<CoefficientType>& vel,
                                  double t, bool max_vel) {
  DRAKE_DEMAND(vel.GetDegree() == 2);

  Polynomial<CoefficientType> acc = vel.Derivative();
  VectorX<CoefficientType> acc_coeffs = acc.GetCoefficients();

  CoefficientType vel0 = vel.EvaluateUnivariate(0);
  CoefficientType vel1 = vel.EvaluateUnivariate(t);
  CoefficientType vel_extrema = vel0;

  // Not constant acceleration, vel extrema is when acc = 0.
  if (std::abs(acc_coeffs[1]) > 1e-12) {
    CoefficientType t_extrema = -acc_coeffs[0] / acc_coeffs[1];

    // If t_extrema is in [0, t], vel_extrema needs to be evaluated.
    // Otherwise, it's at one of the end points.
    if (t_extrema >= 0 && t_extrema <= t) {
      vel_extrema = vel.EvaluateUnivariate(t_extrema);
    }
  }

  if (max_vel) {
    return std::max(std::max(vel0, vel1), vel_extrema);
  } else {
    return std::min(std::min(vel0, vel1), vel_extrema);
  }
}

// Check continuity for up to d degree, where d is the minimum of \p degree and
// the highest degree of the trajectory.
template <typename CoefficientType>
bool CheckContinuity(const PiecewisePolynomial<CoefficientType>& traj,
                     CoefficientType tol, int degree) {
  if (degree < 0) return false;

  typedef Polynomial<CoefficientType> PolynomialType;

  int rows = traj.rows();
  int cols = traj.cols();

  CoefficientType val0, val1;

  for (int n = 0; n < traj.getNumberOfSegments() - 1; ++n) {
    double dt = traj.getDuration(n);

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        PolynomialType poly = traj.getPolynomial(n, i, j);
        PolynomialType next_poly = traj.getPolynomial(n + 1, i, j);
        int max_degree =
            std::min(degree, traj.getSegmentPolynomialDegree(n, i, j));
        for (int d = 0; d < max_degree + 1; d++) {
          // Values evaluated at the end of the current segment.
          val0 = poly.EvaluateUnivariate(dt);
          // Values evaluated at the beginning of the next segment.
          val1 = next_poly.EvaluateUnivariate(0);

          if (std::abs(val0 - val1) > tol) {
            return false;
          }

          poly = poly.Derivative();
          next_poly = next_poly.Derivative();
        }
      }
    }
  }

  return true;
}

// Check P(t), P'(t), P''(t), ... match values[i][t], where i is the ith
// derivative, and t is the tth break.
template <typename CoefficientType>
bool CheckValues(
    const PiecewisePolynomial<CoefficientType>& traj,
    const std::vector<std::vector<MatrixX<CoefficientType>>>& values,
    CoefficientType tol,
    bool check_last_time_step = true) {
  if (values.empty()) return false;

  typedef Polynomial<CoefficientType> PolynomialType;

  int rows = traj.rows();
  int cols = traj.cols();

  CoefficientType val0, val1;

  for (int n = 0; n < traj.getNumberOfSegments(); ++n) {
    double dt = traj.getDuration(n);

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        PolynomialType poly = traj.getPolynomial(n, i, j);

        for (const auto& value : values) {
          // Values evaluated at the beginning of the current segment.
          val0 = poly.EvaluateUnivariate(0);
          if (std::abs(val0 - value.at(n)(i, j)) > tol) {
            return false;
          }

          // Checks the last time step's values.
          if (check_last_time_step && n == traj.getNumberOfSegments() - 1) {
            // Values evaluated at the end.
            val1 = poly.EvaluateUnivariate(dt);
            if (std::abs(val1 - value.at(n + 1)(i, j)) > tol) {
              return false;
            }
          }

          poly = poly.Derivative();
        }
      }
    }
  }

  return true;
}

template <typename CoefficientType>
bool CheckInterpolatedValuesAtBreakTime(
    const PiecewisePolynomial<CoefficientType>& traj,
    const std::vector<double>& breaks,
    const std::vector<MatrixX<CoefficientType>>& values,
    CoefficientType tol,
    bool check_last_time_step = true) {
  int N = static_cast<int>(breaks.size());
  for (int i = 0; i < N; ++i) {
    if (i == N - 1 && !check_last_time_step)
      continue;
    if (!CompareMatrices(traj.value(breaks[i]), values[i], tol,
                         MatrixCompareType::absolute)) {
      return false;
    }
  }
  return true;
}

// This function does the following checks for each dimension of Y:
// traj(T) = Y
// for each segment Pi:
//   Pi(end_time_i) = Pi+1(0)
//   Pi'(end_time_i) = Pi+1'(0)
//   if (Y[i+1] - Y[i] > 0)
//     min(Pi') >= 0
//   else if (Y[i+1] - Y[i] < 0)
//     max(Pi') <= 0
//
// The last two conditions are the monotonic conditions ("shape preserving").
template <typename CoefficientType>
void PchipTest(const std::vector<double>& breaks,
               const std::vector<MatrixX<CoefficientType>>& knots,
               const PiecewisePolynomial<CoefficientType>& traj,
               CoefficientType tol) {
  typedef Polynomial<CoefficientType> PolynomialType;
  const std::vector<double>& T = breaks;
  const std::vector<MatrixX<CoefficientType>>& Y = knots;

  int rows = Y.front().rows();
  int cols = Y.front().cols();

  EXPECT_TRUE(CheckContinuity(traj, tol, 1));
  EXPECT_TRUE(CheckValues(traj, {Y}, tol));
  EXPECT_TRUE(CheckInterpolatedValuesAtBreakTime(traj, T, Y, tol));

  // Check monotonic.
  for (int n = 0; n < traj.getNumberOfSegments(); ++n) {
    double dt = traj.getDuration(n);

    EXPECT_NEAR(T[n], traj.getStartTime(n), tol);

    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        const PolynomialType& poly = traj.getPolynomial(n, i, j);
        PolynomialType poly_deriv = poly.Derivative();

        CoefficientType y0 = poly.EvaluateUnivariate(0);
        CoefficientType y1 = poly.EvaluateUnivariate(dt);

        double vv;
        // If Y is increasing in this segment, all velocity in this segment >=
        // 0.
        if (y1 > y0) {
          vv = ComputeExtremeVel(poly_deriv, dt, false);
          EXPECT_TRUE(vv >= -tol);
        } else if (y1 < y0) {
          // If Y is increasing in this segment, all velocity in this segment >=
          // 0.
          vv = ComputeExtremeVel(poly_deriv, dt, true);
          EXPECT_TRUE(vv <= tol);
        } else {
          vv = ComputeExtremeVel(poly_deriv, dt, true);
          EXPECT_NEAR(vv, 0, tol);
        }
      }
    }
  }
}

GTEST_TEST(SplineTests, PchipAndCubicSplineCompareWithMatlabTest) {
  // Matlab example (matlab's coefs has the opposite order):
  // x = -3:3;
  // y = [-1 -1 -1 0 1 1 1];
  // pp = pchip(x,y);
  // pp.coefs
  //
  // ans =
  //
  //      0     0     0    -1
  //      0     0     0    -1
  //     -1     2     0    -1
  //     -1     1     1     0
  //      0     0     0     1
  //      0     0     0     1
  std::vector<double> T = {-3, -2, -1, 0, 1, 2, 3};
  std::vector<MatrixX<double>> Y(T.size(), MatrixX<double>::Zero(1, 1));
  Y[0](0, 0) = -1;
  Y[1](0, 0) = -1;
  Y[2](0, 0) = -1;
  Y[3](0, 0) = 0;
  Y[4](0, 0) = 1;
  Y[5](0, 0) = 1;
  Y[6](0, 0) = 1;

  std::vector<Vector4<double>> coeffs(T.size());
  coeffs[0] << -1, 0, 0, 0;
  coeffs[1] << -1, 0, 0, 0;
  coeffs[2] << -1, 0, 2, -1;
  coeffs[3] << 0, 1, 1, -1;
  coeffs[4] << 1, 0, 0, 0;
  coeffs[5] << 1, 0, 0, 0;

  PiecewisePolynomial<double> spline = PiecewisePolynomial<double>::Pchip(T, Y);
  EXPECT_EQ(spline.getNumberOfSegments(), static_cast<int>(T.size()) - 1);
  for (int t = 0; t < spline.getNumberOfSegments(); ++t) {
    const PiecewisePolynomial<double>::PolynomialMatrix& poly_matrix =
        spline.getPolynomialMatrix(t);
    EXPECT_TRUE(CompareMatrices(poly_matrix(0, 0).GetCoefficients(), coeffs[t],
                                1e-12, MatrixCompareType::absolute));
  }
  PchipTest(T, Y, spline, 1e-12);

  // pp = spline(x, y)
  // pp.coefs
  //
  // ans =
  //
  //     0.2500   -0.7500    0.5000   -1.0000
  //     0.2500         0   -0.2500   -1.0000
  //    -0.2500    0.7500    0.5000   -1.0000
  //    -0.2500         0    1.2500         0
  //     0.2500   -0.7500    0.5000    1.0000
  //     0.2500    0.0000   -0.2500    1.0000
  coeffs[0] << -1, 0.5, -0.75, 0.25;
  coeffs[1] << -1, -0.25, 0, 0.25;
  coeffs[2] << -1, 0.5, 0.75, -0.25;
  coeffs[3] << 0, 1.25, 0, -0.25;
  coeffs[4] << 1, 0.5, -0.75, 0.25;
  coeffs[5] << 1, -0.25, 0, 0.25;
  spline = PiecewisePolynomial<double>::Cubic(T, Y);
  EXPECT_EQ(spline.getNumberOfSegments(), static_cast<int>(T.size()) - 1);
  for (int t = 0; t < spline.getNumberOfSegments(); ++t) {
    const PiecewisePolynomial<double>::PolynomialMatrix& poly_matrix =
        spline.getPolynomialMatrix(t);
    EXPECT_TRUE(CompareMatrices(poly_matrix(0, 0).GetCoefficients(), coeffs[t],
                                1e-12, MatrixCompareType::absolute));
  }

  EXPECT_TRUE(CheckContinuity(spline, 1e-12, 2));
  EXPECT_TRUE(CheckValues(spline, {Y}, 1e-12));
  EXPECT_TRUE(CheckInterpolatedValuesAtBreakTime(spline, T, Y, 1e-12));

  // Add special case for pchip to test for the last two knots being the same.
  // There was a sign comparison bug in ComputePchipEndSlope when the end
  // slope = 0. See issue #4450.
  T = {0, 1, 2};
  Y.resize(T.size(), MatrixX<double>::Zero(1, 1));
  Y[0](0, 0) = 1;
  Y[1](0, 0) = 3;
  Y[2](0, 0) = 3;
  spline = PiecewisePolynomial<double>::Pchip(T, Y);
  EXPECT_TRUE(CompareMatrices(
      spline.getPolynomialMatrix(0)(0, 0).GetCoefficients(),
      Vector4<double>(1, 3, 0, -1), 1e-12, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      spline.getPolynomialMatrix(1)(0, 0).GetCoefficients(),
      Vector4<double>(3, 0, 0, 0), 1e-12, MatrixCompareType::absolute));
}

GTEST_TEST(SplineTests, RandomizedLinearSplineTest) {
  default_random_engine generator(123);
  int N = 11;
  int num_tests = 1000;
  int rows = 3;
  int cols = 6;

  for (int ctr = 0; ctr < num_tests; ++ctr) {
    std::vector<double> T =
        PiecewiseFunction::randomSegmentTimes(N - 1, generator);
    std::vector<MatrixX<double>> Y(N);
    for (int i = 0; i < N; ++i) Y[i] = MatrixX<double>::Random(rows, cols);

    PiecewisePolynomial<double> spline =
        PiecewisePolynomial<double>::FirstOrderHold(T, Y);
    EXPECT_TRUE(CheckContinuity(spline, 1e-12, 0));
    EXPECT_TRUE(CheckValues(spline, {Y}, 1e-12));
    EXPECT_TRUE(CheckInterpolatedValuesAtBreakTime(spline, T, Y, 1e-12));
  }
}

GTEST_TEST(SplineTests, RandomizedConstantSplineTest) {
  default_random_engine generator(123);
  int N = 2;
  int num_tests = 1;
  int rows = 3;
  int cols = 6;

  for (int ctr = 0; ctr < num_tests; ++ctr) {
    std::vector<double> T =
        PiecewiseFunction::randomSegmentTimes(N - 1, generator);
    std::vector<MatrixX<double>> Y(N);
    for (int i = 0; i < N; ++i) Y[i] = MatrixX<double>::Random(rows, cols);

    PiecewisePolynomial<double> spline =
        PiecewisePolynomial<double>::ZeroOrderHold(T, Y);
    // Don't check the last time step, because constant spline ignores the last
    // knot.
    EXPECT_TRUE(CheckValues(spline, {Y}, 1e-12, false));
    EXPECT_TRUE(CheckInterpolatedValuesAtBreakTime(spline, T, Y, 1e-12, false));
  }
}

GTEST_TEST(SplineTests, RandomizedPchipSplineTest) {
  default_random_engine generator(123);
  int N = 10;
  int num_tests = 1000;
  int rows = 3;
  int cols = 4;

  for (int ctr = 0; ctr < num_tests; ++ctr) {
    std::vector<double> T =
        PiecewiseFunction::randomSegmentTimes(N - 1, generator);
    std::vector<MatrixX<double>> Y(N);
    for (int i = 0; i < N; ++i) Y[i] = MatrixX<double>::Random(rows, cols);

    PiecewisePolynomial<double> spline =
        PiecewisePolynomial<double>::Pchip(T, Y);
    PchipTest(T, Y, spline, 1e-8);

    spline = PiecewisePolynomial<double>::Pchip(
        T, Y, true /* Uses zero end point derivative. */);
    PchipTest(T, Y, spline, 1e-8);
    // Derivatives at end points should be zero.
    PiecewisePolynomial<double> spline_dot = spline.derivative();
    EXPECT_NEAR(spline_dot.value(spline_dot.getStartTime()).norm(), 0, 1e-10);
    EXPECT_NEAR(spline_dot.value(spline_dot.getEndTime()).norm(), 0, 1e-10);
  }
}

GTEST_TEST(SplineTests, PchipLength2Test) {
  std::vector<double> T = {0, 1};
  std::vector<MatrixX<double>> Y(2, MatrixX<double>::Zero(1, 1));
  Y[0] << 1;
  Y[1] << 3;

  PiecewisePolynomial<double> spline =
      PiecewisePolynomial<double>::Pchip(T, Y, true);
  PiecewisePolynomial<double> spline_dot = spline.derivative();

  EXPECT_NEAR(spline_dot.value(spline_dot.getStartTime()).norm(), 0, 1e-10);
  EXPECT_NEAR(spline_dot.value(spline_dot.getEndTime()).norm(), 0, 1e-10);

  // Computes the minimal velocity from T = 0 to T = 1. Since this segment is
  // increasing, the minimal velocity needs to be greater than 0.
  double v_min = ComputeExtremeVel(
      spline_dot.getPolynomial(0), T.back(), false);
  EXPECT_GE(v_min, 0);

  Y[0] << 5;
  Y[1] << -2;
  spline = PiecewisePolynomial<double>::Pchip(T, Y, true);
  spline_dot = spline.derivative();

  EXPECT_NEAR(spline_dot.value(spline_dot.getStartTime()).norm(), 0, 1e-10);
  EXPECT_NEAR(spline_dot.value(spline_dot.getEndTime()).norm(), 0, 1e-10);

  // Max velocity should be non positive.
  double v_max = ComputeExtremeVel(
      spline_dot.getPolynomial(0), T.back(), true);
  EXPECT_LE(v_max, 0);
}

GTEST_TEST(SplineTests, RandomizedCubicSplineTest) {
  default_random_engine generator(123);
  int N = 30;
  int num_tests = 40;
  int rows = 3;
  int cols = 4;

  // Test Cubic(T, Y)
  for (int ctr = 0; ctr < num_tests; ++ctr) {
    std::vector<double> T =
        PiecewiseFunction::randomSegmentTimes(N - 1, generator);
    std::vector<MatrixX<double>> Y(N);
    for (int i = 0; i < N; ++i) Y[i] = MatrixX<double>::Random(rows, cols);

    PiecewisePolynomial<double> spline =
        PiecewisePolynomial<double>::Cubic(T, Y);
    EXPECT_TRUE(CheckContinuity(spline, 1e-8, 2));
    EXPECT_TRUE(CheckValues(spline, {Y}, 1e-8));
    EXPECT_TRUE(CheckInterpolatedValuesAtBreakTime(spline, T, Y, 1e-8));
  }

  // Test Cubic(T, Y, Ydot0, Ydot1)
  for (int ctr = 0; ctr < num_tests; ++ctr) {
    std::vector<double> T =
        PiecewiseFunction::randomSegmentTimes(N - 1, generator);
    std::vector<MatrixX<double>> Y(N);
    for (int i = 0; i < N; ++i) Y[i] = MatrixX<double>::Random(rows, cols);

    MatrixX<double> Ydot0 = MatrixX<double>::Random(rows, cols);
    MatrixX<double> Ydot1 = MatrixX<double>::Random(rows, cols);
    PiecewisePolynomial<double> spline =
        PiecewisePolynomial<double>::Cubic(T, Y, Ydot0, Ydot1);
    EXPECT_TRUE(CheckContinuity(spline, 1e-8, 2));
    EXPECT_TRUE(CheckValues(spline, {Y}, 1e-8));
    EXPECT_TRUE(CheckInterpolatedValuesAtBreakTime(spline, T, Y, 1e-8));

    // Check the first and last first derivatives.
    MatrixX<double> Ydot0_test = spline.derivative().value(T.front());
    MatrixX<double> Ydot1_test = spline.derivative().value(T.back());
    EXPECT_TRUE(
        CompareMatrices(Ydot0, Ydot0_test, 1e-8, MatrixCompareType::absolute));
    EXPECT_TRUE(
        CompareMatrices(Ydot1, Ydot1_test, 1e-8, MatrixCompareType::absolute));
  }

  // Test Cubic(T, Y, Ydots)
  for (int ctr = 0; ctr < num_tests; ++ctr) {
    std::vector<double> T =
        PiecewiseFunction::randomSegmentTimes(N - 1, generator);
    std::vector<MatrixX<double>> Y(N);
    std::vector<MatrixX<double>> Ydot(N);

    for (int i = 0; i < N; ++i) {
      Y[i] = MatrixX<double>::Random(rows, cols);
      Ydot[i] = MatrixX<double>::Random(rows, cols);
    }

    PiecewisePolynomial<double> spline =
        PiecewisePolynomial<double>::Cubic(T, Y, Ydot);
    EXPECT_TRUE(CheckContinuity(spline, 1e-8, 1));
    EXPECT_TRUE(CheckValues(spline, {Y, Ydot}, 1e-8));
    EXPECT_TRUE(CheckInterpolatedValuesAtBreakTime(spline, T, Y, 1e-8));
  }
}

GTEST_TEST(SplineTests, CubicSplineSize2) {
  std::vector<double> T = {1, 2};
  std::vector<MatrixX<double>> Y(2, MatrixX<double>::Zero(1, 1));
  MatrixX<double> Ydot0 = MatrixX<double>::Zero(1, 1);
  MatrixX<double> Ydot1 = MatrixX<double>::Zero(1, 1);
  Y[0] << 1;
  Y[1] << 3;
  Ydot0 << 2;
  Ydot1 << -1;

  PiecewisePolynomial<double> spline =
      PiecewisePolynomial<double>::Cubic(T, Y, Ydot0, Ydot1);
  EXPECT_TRUE(CheckValues(spline, {Y, {Ydot0, Ydot1}}, 1e-8));

  spline = PiecewisePolynomial<double>::Cubic(T, Y, {Ydot0, Ydot1});
  EXPECT_TRUE(CheckValues(spline, {Y, {Ydot0, Ydot1}}, 1e-8));

  // Calling Cubic(times, Y) with only 2 knots should not be allowed.
  EXPECT_THROW(PiecewisePolynomial<double>::Cubic(T, Y), std::runtime_error);
}

template <typename CoefficientType>
void TestThrows(const std::vector<double>& breaks,
                const std::vector<MatrixX<CoefficientType>>& knots) {
  EXPECT_THROW(PiecewisePolynomial<double>::ZeroOrderHold(breaks, knots),
               std::runtime_error);
  EXPECT_THROW(PiecewisePolynomial<double>::FirstOrderHold(breaks, knots),
               std::runtime_error);
  EXPECT_THROW(PiecewisePolynomial<double>::Pchip(breaks, knots),
               std::runtime_error);
  EXPECT_THROW(PiecewisePolynomial<double>::Cubic(breaks, knots),
               std::runtime_error);
}

template <typename CoefficientType>
void TestNoThrows(
    const std::vector<double>& breaks,
    const std::vector<MatrixX<CoefficientType>>& knots) {
  EXPECT_NO_THROW(PiecewisePolynomial<double>::ZeroOrderHold(breaks, knots));
  EXPECT_NO_THROW(PiecewisePolynomial<double>::FirstOrderHold(breaks, knots));
  EXPECT_NO_THROW(PiecewisePolynomial<double>::Pchip(breaks, knots));
  EXPECT_NO_THROW(PiecewisePolynomial<double>::Cubic(breaks, knots));
}

GTEST_TEST(SplineTests, TestException) {
  // Doesn't throw.
  int rows = 3;
  int cols = 4;
  std::vector<double> T = {1, 2, 3, 4};
  std::vector<MatrixX<double>> Y(T.size(), MatrixX<double>::Zero(rows, cols));
  TestNoThrows(T, Y);

  // Non increasing T
  T = {1, 0, 3, 4};
  TestThrows(T, Y);

  T = {0, 0, 3, 4};
  TestThrows(T, Y);

  // Inconsistent Y dimension.
  Y.front().resize(Y.front().rows() + 1, Y.front().cols() + 1);
  TestThrows(T, Y);

  // T size doesn't match Y size
  Y = std::vector<MatrixX<double>>(T.size() + 1,
                                   MatrixX<double>::Zero(rows, cols));
  TestThrows(T, Y);

  // Min length
  T = {1, 2};
  Y = std::vector<MatrixX<double>>(T.size(), MatrixX<double>::Zero(rows, cols));
  EXPECT_NO_THROW(PiecewisePolynomial<double>::FirstOrderHold(T, Y));
  EXPECT_NO_THROW(PiecewisePolynomial<double>::ZeroOrderHold(T, Y));

  EXPECT_THROW(PiecewisePolynomial<double>::Pchip(T, Y), std::runtime_error);
  EXPECT_THROW(PiecewisePolynomial<double>::Cubic(T, Y), std::runtime_error);

  T = {2};
  Y = std::vector<MatrixX<double>>(T.size(), MatrixX<double>::Zero(rows, cols));
  EXPECT_THROW(PiecewisePolynomial<double>::ZeroOrderHold(T, Y),
               std::runtime_error);

  EXPECT_THROW(PiecewisePolynomial<double>::FirstOrderHold(T, Y),
               std::runtime_error);

  // Test Ydot0 / Ydot1 mismatch.
  T = {1, 2, 3};
  Y = std::vector<MatrixX<double>>(T.size(), MatrixX<double>::Zero(rows, cols));
  MatrixX<double> Ydot0(rows, cols);
  MatrixX<double> Ydot1(rows, cols);
  EXPECT_NO_THROW(PiecewisePolynomial<double>::Cubic(T, Y, Ydot0, Ydot1));

  Ydot0.resize(rows, cols);
  Ydot1.resize(rows, cols + 1);
  EXPECT_THROW(PiecewisePolynomial<double>::Cubic(T, Y, Ydot0, Ydot1),
               std::runtime_error);

  Ydot0.resize(rows + 1, cols);
  Ydot1.resize(rows, cols);
  EXPECT_THROW(PiecewisePolynomial<double>::Cubic(T, Y, Ydot0, Ydot1),
               std::runtime_error);

  // Test Ydot mismatch.
  T = {1, 2, 3};
  Y = std::vector<MatrixX<double>>(T.size(), MatrixX<double>::Zero(rows, cols));
  std::vector<MatrixX<double>> Ydot =
      std::vector<MatrixX<double>>(T.size(), MatrixX<double>::Zero(rows, cols));
  EXPECT_NO_THROW(PiecewisePolynomial<double>::Cubic(T, Y, Ydot));

  Ydot = std::vector<MatrixX<double>>(T.size() + 1,
                                      MatrixX<double>::Zero(rows, cols));
  EXPECT_THROW(PiecewisePolynomial<double>::Cubic(T, Y, Ydot),
               std::runtime_error);

  Ydot =
      std::vector<MatrixX<double>>(T.size(), MatrixX<double>::Zero(rows, cols));
  Ydot.front().resize(rows + 2, cols + 1);
  EXPECT_THROW(PiecewisePolynomial<double>::Cubic(T, Y, Ydot),
               std::runtime_error);
}

}  // namespace
}  // namespace drake
