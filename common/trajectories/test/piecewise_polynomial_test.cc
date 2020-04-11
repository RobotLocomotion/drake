#include "drake/common/trajectories/piecewise_polynomial.h"

#include <random>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/trajectories/test/random_piecewise_polynomial.h"

using Eigen::Matrix;
using std::default_random_engine;
using std::uniform_real_distribution;
using std::vector;
using std::runtime_error;
using std::normal_distribution;
using std::uniform_int_distribution;

namespace drake {
namespace trajectories {
namespace {

template<typename T>
void testIntegralAndDerivative() {
  int num_coefficients = 5;
  int num_segments = 3;
  int rows = 3;
  int cols = 5;

  typedef PiecewisePolynomial<T> PiecewisePolynomialType;

  default_random_engine generator;
  vector<double> segment_times =
      PiecewiseTrajectory<double>::RandomSegmentTimes(num_segments, generator);
  PiecewisePolynomialType piecewise =
      test::MakeRandomPiecewisePolynomial<T>(
          rows, cols, num_coefficients, segment_times);

  // derivative(0) should be same as original piecewise.
  EXPECT_TRUE(
      CompareMatrices(piecewise.value(piecewise.start_time()),
                      piecewise.derivative(0).value(piecewise.start_time()),
                      1e-10, MatrixCompareType::absolute));

  // differentiate integral, get original back
  PiecewisePolynomialType piecewise_back = piecewise.integral().derivative();
  if (!piecewise.isApprox(piecewise_back, 1e-10)) throw runtime_error("wrong");

  // check value at start time
  MatrixX<T> desired_value_at_t0 =
      MatrixX<T>::Random(piecewise.rows(), piecewise.cols());
  PiecewisePolynomialType integral = piecewise.integral(desired_value_at_t0);
  auto value_at_t0 = integral.value(piecewise.start_time());
  EXPECT_TRUE(CompareMatrices(desired_value_at_t0, value_at_t0, 1e-10,
                              MatrixCompareType::absolute));

  // check continuity at sample points
  for (int i = 0; i < piecewise.get_number_of_segments() - 1; ++i) {
    EXPECT_EQ(integral.getPolynomial(i)
                  .EvaluateUnivariate(integral.duration(i)),
              integral.getPolynomial(i + 1).EvaluateUnivariate(0.0));
  }
}

template<typename T>
void testBasicFunctionality() {
  int max_num_coefficients = 6;
  int num_tests = 100;
  default_random_engine generator;
  uniform_int_distribution<> int_distribution(1, max_num_coefficients);

  typedef PiecewisePolynomial<T> PiecewisePolynomialType;

  for (int i = 0; i < num_tests; ++i) {
    int num_coefficients = int_distribution(generator);
    int num_segments = int_distribution(generator);
    int rows = int_distribution(generator);
    int cols = int_distribution(generator);

    vector<double> segment_times =
        PiecewiseTrajectory<double>::RandomSegmentTimes(num_segments,
                                                        generator);
    PiecewisePolynomialType piecewise1 =
        test::MakeRandomPiecewisePolynomial<T>(
            rows, cols, num_coefficients, segment_times);
    PiecewisePolynomialType piecewise2 =
        test::MakeRandomPiecewisePolynomial<T>(
            rows, cols, num_coefficients, segment_times);
    PiecewisePolynomialType piecewise3_not_matching_rows =
        test::MakeRandomPiecewisePolynomial<T>(
            rows + 1, cols, num_coefficients, segment_times);
    PiecewisePolynomialType piecewise4_not_matching_cols =
        test::MakeRandomPiecewisePolynomial<T>(
            rows, cols + 1, num_coefficients, segment_times);

    normal_distribution<double> normal;
    double shift = normal(generator);
    MatrixX<T> offset =
        MatrixX<T>::Random(piecewise1.rows(), piecewise1.cols());

    PiecewisePolynomialType sum = piecewise1 + piecewise2;
    PiecewisePolynomialType difference = piecewise2 - piecewise1;
    PiecewisePolynomialType piecewise1_plus_offset = piecewise1 + offset;
    PiecewisePolynomialType piecewise1_minus_offset = piecewise1 - offset;
    PiecewisePolynomialType piecewise1_shifted = piecewise1;
    piecewise1_shifted.shiftRight(shift);
    PiecewisePolynomialType product = piecewise1 * piecewise2;

    const double total_time = segment_times.back() - segment_times.front();
    PiecewisePolynomialType piecewise2_twice = piecewise2;
    PiecewisePolynomialType piecewise2_shifted = piecewise2;
    piecewise2_shifted.shiftRight(total_time);

    // Checks that concatenation of trajectories that are not time
    // aligned at the connecting ends is a failure.
    PiecewisePolynomialType piecewise2_shifted_twice = piecewise2;
    piecewise2_shifted_twice.shiftRight(2. * total_time);
    EXPECT_THROW(piecewise2_twice.ConcatenateInTime(
        piecewise2_shifted_twice), std::runtime_error);

    // Checks that concatenation of trajectories that have different
    // row counts is a failure.
    PiecewisePolynomialType piecewise3_not_matching_rows_shifted =
        piecewise3_not_matching_rows;
    piecewise3_not_matching_rows_shifted.shiftRight(total_time);
    EXPECT_THROW(piecewise2_twice.ConcatenateInTime(
        piecewise3_not_matching_rows_shifted), std::runtime_error);

    // Checks that concatenation of trajectories that have different
    // col counts is a failure.
    PiecewisePolynomialType piecewise4_not_matching_cols_shifted =
        piecewise4_not_matching_cols;
    piecewise4_not_matching_cols_shifted.shiftRight(total_time);
    EXPECT_THROW(piecewise2_twice.ConcatenateInTime(
        piecewise4_not_matching_cols_shifted), std::runtime_error);

    piecewise2_twice.ConcatenateInTime(piecewise2_shifted);

    uniform_real_distribution<double> uniform(piecewise1.start_time(),
                                              piecewise1.end_time());
    double t = uniform(generator);

    EXPECT_TRUE(CompareMatrices(sum.value(t),
                                piecewise1.value(t) + piecewise2.value(t), 1e-8,
                                MatrixCompareType::absolute));

    EXPECT_TRUE(CompareMatrices(difference.value(t),
                                piecewise2.value(t) - piecewise1.value(t), 1e-8,
                                MatrixCompareType::absolute));

    EXPECT_TRUE(CompareMatrices(piecewise1_plus_offset.value(t),
                                piecewise1.value(t) + offset, 1e-8,
                                MatrixCompareType::absolute));

    EXPECT_TRUE(CompareMatrices(piecewise1_minus_offset.value(t),
                                piecewise1.value(t) - offset, 1e-8,
                                MatrixCompareType::absolute));

    EXPECT_TRUE(CompareMatrices(piecewise1_shifted.value(t),
                                piecewise1.value(t - shift), 1e-8,
                                MatrixCompareType::absolute));

    EXPECT_TRUE(CompareMatrices(
        product.value(t),
        (piecewise1.value(t).array() * piecewise2.value(t).array()).matrix(),
        1e-8, MatrixCompareType::absolute));

    // Checks that `piecewise2_twice` is effectively the concatenation of
    // `piecewise2` and a copy of `piecewise2` that is shifted to the right
    // (i.e. towards increasing values of t) by an amount equal to its entire
    // time length. To this end, it verifies that R(tₓ) = R(tₓ + d), where
    // R(t) = P(t) for t0 <= t <= t1, R(t) = Q(t) for t1 <= t <= t2,
    // Q(t) = P(t - d) for t1 <= t <= t2, d = t1 - t0 = t2 - t1 and
    // t0 < tₓ < t1, with P, Q and R functions being piecewise polynomials.
    EXPECT_TRUE(CompareMatrices(
        piecewise2_twice.value(t), piecewise2_twice.value(t + total_time),
        1e-8, MatrixCompareType::absolute));
  }
}

template<typename T>
void testValueOutsideOfRange() {
  typedef PiecewisePolynomial<T> PiecewisePolynomialType;

  default_random_engine generator;
  vector<double> segment_times =
      PiecewiseTrajectory<double>::RandomSegmentTimes(6, generator);
  PiecewisePolynomialType piecewise =
      test::MakeRandomPiecewisePolynomial<T>(
          3, 4, 5, segment_times);

  EXPECT_TRUE(CompareMatrices(piecewise.value(piecewise.start_time()),
                              piecewise.value(piecewise.start_time() - 1.0),
                              1e-10, MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(piecewise.value(piecewise.end_time()),
                              piecewise.value(piecewise.end_time() + 1.0),
                              1e-10, MatrixCompareType::absolute));
}

// Test the generation of cubic splines with first and second derivatives
// continuous between the end of the last segment and the beginning of the
// first.
GTEST_TEST(testPiecewisePolynomial, CubicSplinePeriodicBoundaryConditionTest) {
  Eigen::VectorXd breaks(5);
  breaks << 0, 1, 2, 3, 4;

  // Spline in 3d.
  Eigen::MatrixXd samples(3, 5);
  samples << 1, 1, 1,
        2, 2, 2,
        0, 3, 3,
        -2, 2, 2,
        1, 1, 1;
  const bool periodic_endpoint = true;

  PiecewisePolynomial<double> periodic_spline =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          breaks, samples, periodic_endpoint);

  std::unique_ptr<Trajectory<double>> spline_dt =
      periodic_spline.MakeDerivative(1);
  std::unique_ptr<Trajectory<double>> spline_ddt =
      periodic_spline.MakeDerivative(2);

  Eigen::VectorXd begin_dt = spline_dt->value(breaks(0));
  Eigen::VectorXd end_dt = spline_dt->value(breaks(breaks.size() - 1));

  Eigen::VectorXd begin_ddt = spline_ddt->value(breaks(0));
  Eigen::VectorXd end_ddt = spline_ddt->value(breaks(breaks.size() - 1));

  EXPECT_TRUE(CompareMatrices(end_dt, begin_dt, 1e-14));
  EXPECT_TRUE(CompareMatrices(end_ddt, begin_ddt, 1e-14));

  // Test that evaluating the derivative directly gives the same results.
  const double t = 1.234;
  EXPECT_TRUE(CompareMatrices(periodic_spline.EvalDerivative(t, 1),
                              spline_dt->value(t), 1e-14));
  EXPECT_TRUE(CompareMatrices(periodic_spline.EvalDerivative(t, 2),
                              spline_ddt->value(t), 1e-14));
}

// Test various exception cases.  We want to check that these throw rather
// than crash (or return potentially bad data).
GTEST_TEST(testPiecewisePolynomial, ExceptionsTest) {
  Eigen::VectorXd breaks(5);
  breaks << 0, 1, 2, 3, 4;

  // Spline in 3d.
  Eigen::MatrixXd samples(3, 5);
  samples << 1, 1, 1,
        2, 2, 2,
        0, 3, 3,
        -2, 2, 2,
        1, 1, 1;

  // No throw with monotonic breaks.
  PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
      breaks, samples, true);

  // Throw when breaks are not strictly monotonic.
  breaks[1] = 0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          breaks, samples, true),
      std::runtime_error, "Times must be in increasing order.");
}

GTEST_TEST(testPiecewisePolynomial, AllTests) {
  testIntegralAndDerivative<double>();

  testBasicFunctionality<double>();

  testValueOutsideOfRange<double>();
}

GTEST_TEST(testPiecewisePolynomial, VectorValueTest) {
  const std::vector<double> times = {0, .5, 1, 1.5};
  const Eigen::Vector3d value(1, 2, 3);

  const PiecewisePolynomial<double> col(value);
  Eigen::MatrixXd out = col.vector_values(times);
  EXPECT_EQ(out.rows(), 3);
  EXPECT_EQ(out.cols(), 4);
  for (int i = 0; i < 4; i++) {
    EXPECT_TRUE(CompareMatrices(out.col(i), value, 0));
  }

  PiecewisePolynomial<double> row(value.transpose());
  out = row.vector_values(times);
  EXPECT_EQ(out.rows(), 4);
  EXPECT_EQ(out.cols(), 3);
  for (int i = 0; i < 4; i++) {
    EXPECT_TRUE(CompareMatrices(out.row(i), value.transpose(), 0));
  }

  PiecewisePolynomial<double> mat(Eigen::Matrix3d::Identity());
  DRAKE_EXPECT_THROWS_MESSAGE(
      mat.vector_values(times), std::runtime_error,
      "This method only supports vector-valued trajectories.");
}

GTEST_TEST(testPiecewisePolynomial, RemoveFinalSegmentTest) {
  Eigen::VectorXd breaks(3);
  breaks << 0, .5, 1.;
  Eigen::MatrixXd samples(2, 3);
  samples << 1, 1, 2,
             2, 0, 3;

  PiecewisePolynomial<double> pp =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          breaks, samples);

  EXPECT_EQ(pp.end_time(), 1.);
  EXPECT_EQ(pp.get_number_of_segments(), 2);

  pp.RemoveFinalSegment();
  EXPECT_EQ(pp.end_time(), .5);
  EXPECT_EQ(pp.get_number_of_segments(), 1);

  pp.RemoveFinalSegment();
  EXPECT_TRUE(pp.empty());
}

std::unique_ptr<Trajectory<double>> TestReverseTime(
    const PiecewisePolynomial<double>& pp_orig) {
  std::unique_ptr<Trajectory<double>> pp_ptr = pp_orig.Clone();
  PiecewisePolynomial<double>* pp =
      dynamic_cast<PiecewisePolynomial<double>*>(pp_ptr.get());

  pp->ReverseTime();
  // Start time and end time have been switched.
  EXPECT_NEAR(pp->start_time(), -pp_orig.end_time(), 1e-14);
  EXPECT_NEAR(pp->end_time(), -pp_orig.start_time(), 1e-14);

  for (const double t : {0.1, .2, .52, .77}) {
    EXPECT_TRUE(CompareMatrices(pp->value(t), pp_orig.value(-t), 1e-14));
  }
  return pp_ptr;
}

void TestScaling(const PiecewisePolynomial<double>& pp_orig,
                 const double scale) {
  std::unique_ptr<Trajectory<double>> pp_ptr = pp_orig.Clone();
  PiecewisePolynomial<double>* pp =
      dynamic_cast<PiecewisePolynomial<double>*>(pp_ptr.get());

  pp->ScaleTime(scale);
  EXPECT_NEAR(pp->start_time(), scale * pp_orig.start_time(), 1e-14);
  EXPECT_NEAR(pp->end_time(), scale * pp_orig.end_time(), 1e-14);
  for (const double trel : {0.1, .2, .52, .77}) {
    const double t = pp_orig.start_time() +
                     trel * (pp_orig.end_time() - pp_orig.start_time());
    EXPECT_TRUE(CompareMatrices(pp->value(scale * t), pp_orig.value(t), 1e-14));
  }
}


GTEST_TEST(testPiecewisePolynomial, ReverseAndScaleTimeTest) {
  Eigen::VectorXd breaks(3);
  breaks << 0, .5, 1.;
  Eigen::MatrixXd samples(2, 3);
  samples << 1, 1, 2,
             2, 0, 3;

  const PiecewisePolynomial<double> zoh =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks, samples);
  auto reversed_zoh = TestReverseTime(zoh);
  // Confirm that the documentation is correct about the subtle behavior at the
  // break-points due to the switch in the half-open interval (since zoh is
  // discontinuous at the breaks).
  EXPECT_FALSE(
      CompareMatrices(reversed_zoh->value(-breaks(1)), zoh.value(breaks(1))));
  EXPECT_TRUE(CompareMatrices(reversed_zoh->value(-breaks(1)),
                              zoh.value(breaks(1) - 1e-14)));
  TestScaling(zoh, 2.3);

  const PiecewisePolynomial<double> foh =
      PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
  TestReverseTime(foh);
  TestScaling(foh, 1.2);
  TestScaling(foh, 3.6);

  const PiecewisePolynomial<double> spline =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          breaks, samples);
  TestReverseTime(spline);
  TestScaling(spline, 2.0);
  TestScaling(spline, 4.3);
}

template <typename T>
void TestScalarType() {
  VectorX<T> breaks(3);
  breaks << 0, .5, 1.;
  MatrixX<T> samples(2, 3);
  samples << 1, 1, 2, 2, 0, 3;

  const PiecewisePolynomial<T> spline =
      PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
          breaks, samples);

  const MatrixX<T> value = spline.value(0.5);
  EXPECT_NEAR(ExtractDoubleOrThrow(value(0)),
              ExtractDoubleOrThrow(samples(0, 1)), 1e-14);
  EXPECT_NEAR(ExtractDoubleOrThrow(value(1)),
              ExtractDoubleOrThrow(samples(1, 1)), 1e-14);
}

GTEST_TEST(PiecewiseTrajectoryTest, ScalarTypes) {
  TestScalarType<double>();
  TestScalarType<AutoDiffXd>();
  TestScalarType<symbolic::Expression>();
}

// Confirm the expected behavior of PiecewisePolynomial<Expression>.
GTEST_TEST(PiecewiseTrajectoryTest, SymbolicValues) {
  using symbolic::Expression;
  using symbolic::Variable;

  const Vector3<Expression> breaks(0, .5, 1.);
  const RowVector3<Expression> samples(6, 5, 4);
  const PiecewisePolynomial<Expression> foh =
      PiecewisePolynomial<Expression>::FirstOrderHold(breaks, samples);

  // value() works if breaks and coefficients are Expressions holding double
  // values, evaluated at a double-valued time.
  EXPECT_NEAR(ExtractDoubleOrThrow(foh.value(0.25)(0)), 5.5, 1e-14);

  // Symbolic time throws (because GetSegmentIndex returns an int in the middle
  // of the evaluation stack, breaking the Expression pipeline),
  EXPECT_THROW(foh.value(Variable("t")), std::runtime_error);

  // Symbolic breaks causes the construction methods to throw.
  const Vector3<Expression> symbolic_breaks(Variable("t0"), Variable("t1"),
                                            Variable("t2"));
  EXPECT_THROW(
      PiecewisePolynomial<Expression>::FirstOrderHold(symbolic_breaks, samples),
      std::runtime_error);

  // Symbolic samples (and therefore coefficient) returns the symbolic form only
  // inside the current segment.  This admittedly bad behavior is documented as
  // a warning in the PiecewisePolynomial::value() documentation.
  const Variable x0("x0");
  const Variable x1("x1");
  const Variable x2("x2");
  const RowVector3<Expression> symbolic_samples(x0, x1, x2);
  const PiecewisePolynomial<Expression> foh_w_symbolic_coeffs =
      PiecewisePolynomial<Expression>::FirstOrderHold(breaks, symbolic_samples);
  EXPECT_TRUE(foh_w_symbolic_coeffs.value(0.25)(0).Expand().EqualTo(0.5 * x0 +
                                                                    0.5 * x1));
}

}  // namespace
}  // namespace trajectories
}  // namespace drake

