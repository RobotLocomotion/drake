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
#include "drake/math/autodiff_gradient.h"

using drake::math::ExtractGradient;
using drake::math::DiscardGradient;
using Eigen::Matrix;
using std::default_random_engine;
using std::normal_distribution;
using std::runtime_error;
using std::uniform_int_distribution;
using std::uniform_real_distribution;
using std::vector;

namespace drake {
namespace trajectories {
namespace {

template<typename T>
void testIntegralAndDerivative() {
  int num_coefficients = 5;
  int num_segments = 3;
  int rows = 3;
  int cols = 5;

  default_random_engine generator;
  vector<double> segment_times =
      PiecewiseTrajectory<double>::RandomSegmentTimes(num_segments, generator);
  PiecewisePolynomial<T> piecewise =
      test::MakeRandomPiecewisePolynomial<T>(
          rows, cols, num_coefficients, segment_times);

  // derivative(0) should be same as original piecewise.
  EXPECT_TRUE(
      CompareMatrices(piecewise.value(piecewise.start_time()),
                      piecewise.derivative(0).value(piecewise.start_time()),
                      1e-10, MatrixCompareType::absolute));

  // differentiate integral, get original back
  PiecewisePolynomial<T> piecewise_back = piecewise.integral().derivative();
  if (!piecewise.isApprox(piecewise_back, 1e-10)) throw runtime_error("wrong");

  // check value at start time
  MatrixX<T> desired_value_at_t0 =
      MatrixX<T>::Random(piecewise.rows(), piecewise.cols());
  PiecewisePolynomial<T> integral = piecewise.integral(desired_value_at_t0);
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

  for (int i = 0; i < num_tests; ++i) {
    int num_coefficients = int_distribution(generator);
    int num_segments = int_distribution(generator);
    int rows = int_distribution(generator);
    int cols = int_distribution(generator);

    vector<double> segment_times =
        PiecewiseTrajectory<double>::RandomSegmentTimes(num_segments,
                                                        generator);
    PiecewisePolynomial<T> piecewise1 = test::MakeRandomPiecewisePolynomial<T>(
        rows, cols, num_coefficients, segment_times);
    PiecewisePolynomial<T> piecewise2 = test::MakeRandomPiecewisePolynomial<T>(
        rows, cols, num_coefficients, segment_times);
    PiecewisePolynomial<T> piecewise3_not_matching_rows =
        test::MakeRandomPiecewisePolynomial<T>(rows + 1, cols, num_coefficients,
                                               segment_times);
    PiecewisePolynomial<T> piecewise4_not_matching_cols =
        test::MakeRandomPiecewisePolynomial<T>(rows, cols + 1, num_coefficients,
                                               segment_times);
    PiecewisePolynomial<T> piecewise5 = test::MakeRandomPiecewisePolynomial<T>(
        cols, rows, num_coefficients, segment_times);

    normal_distribution<double> normal;
    double shift = normal(generator);
    MatrixX<T> offset =
        MatrixX<T>::Random(piecewise1.rows(), piecewise1.cols());

    PiecewisePolynomial<T> sum = piecewise1 + piecewise2;
    PiecewisePolynomial<T> difference = piecewise2 - piecewise1;
    PiecewisePolynomial<T> piecewise1_plus_offset = piecewise1 + offset;
    PiecewisePolynomial<T> piecewise1_minus_offset = piecewise1 - offset;
    PiecewisePolynomial<T> piecewise1_shifted = piecewise1;
    piecewise1_shifted.shiftRight(shift);
    PiecewisePolynomial<T> product = piecewise1 * piecewise5;
    PiecewisePolynomial<T> unary_minus = -piecewise1;

    const double total_time = segment_times.back() - segment_times.front();
    PiecewisePolynomial<T> piecewise2_twice = piecewise2;
    PiecewisePolynomial<T> piecewise2_shifted = piecewise2;
    piecewise2_shifted.shiftRight(total_time);

    // Checks that concatenation of trajectories that are not time
    // aligned at the connecting ends is a failure.
    PiecewisePolynomial<T> piecewise2_shifted_twice = piecewise2;
    piecewise2_shifted_twice.shiftRight(2. * total_time);
    EXPECT_THROW(piecewise2_twice.ConcatenateInTime(
        piecewise2_shifted_twice), std::runtime_error);

    // Checks that concatenation of trajectories that have different
    // row counts is a failure.
    PiecewisePolynomial<T> piecewise3_not_matching_rows_shifted =
        piecewise3_not_matching_rows;
    piecewise3_not_matching_rows_shifted.shiftRight(total_time);
    EXPECT_THROW(piecewise2_twice.ConcatenateInTime(
        piecewise3_not_matching_rows_shifted), std::runtime_error);

    // Checks that concatenation of trajectories that have different
    // col counts is a failure.
    PiecewisePolynomial<T> piecewise4_not_matching_cols_shifted =
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

    EXPECT_TRUE(CompareMatrices(product.value(t),
                                piecewise1.value(t) * piecewise5.value(t), 1e-8,
                                MatrixCompareType::absolute));

    EXPECT_TRUE(CompareMatrices(unary_minus.value(t), -(piecewise1.value(t))));

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
  default_random_engine generator;
  vector<double> segment_times =
      PiecewiseTrajectory<double>::RandomSegmentTimes(6, generator);
  PiecewisePolynomial<T> piecewise =
      test::MakeRandomPiecewisePolynomial<T>(3, 4, 5, segment_times);

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
      "Times must be in increasing order.");
}

GTEST_TEST(testPiecewisePolynomial, AllTests) {
  testIntegralAndDerivative<double>();

  testBasicFunctionality<double>();

  testValueOutsideOfRange<double>();
}

GTEST_TEST(testPiecewisePolynomial, VectorValueTest) {
  // Note: Keep one negative time to confirm that negative values can work for
  // constant trajectories.
  const std::vector<double> times = {-1.5, 0, .5, 1, 1.5};
  const Eigen::Vector3d value(1, 2, 3);

  const PiecewisePolynomial<double> col(value);
  Eigen::MatrixXd out = col.vector_values(times);
  EXPECT_EQ(out.rows(), 3);
  EXPECT_EQ(out.cols(), 5);
  for (int i = 0; i < 4; i++) {
    EXPECT_TRUE(CompareMatrices(out.col(i), value, 0));
  }

  PiecewisePolynomial<double> row(value.transpose());
  out = row.vector_values(times);
  EXPECT_EQ(out.rows(), 5);
  EXPECT_EQ(out.cols(), 3);
  for (int i = 0; i < 4; i++) {
    EXPECT_TRUE(CompareMatrices(out.row(i), value.transpose(), 0));
  }

  PiecewisePolynomial<double> mat(Eigen::Matrix3d::Identity());
  DRAKE_EXPECT_THROWS_MESSAGE(
      mat.vector_values(times),
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

GTEST_TEST(testPiecewisePolynomial, ReshapeAndBlockTest) {
  std::vector<double> breaks = {0, .5, 1.};
  std::vector<Eigen::MatrixXd> samples(3);
  samples[0].resize(2, 3);
  samples[0] << 1, 1, 2, 2, 0, 3;
  samples[1].resize(2, 3);
  samples[1] << 3, 4, 5, 6, 7, 8;
  samples[2].resize(2, 3);
  samples[2] << -.2, 33., 5.4, -2.1, 52, 12;

  PiecewisePolynomial<double> zoh =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks, samples);
  EXPECT_EQ(zoh.rows(), 2);
  EXPECT_EQ(zoh.cols(), 3);

  zoh.Reshape(3, 2);
  EXPECT_EQ(zoh.rows(), 3);
  EXPECT_EQ(zoh.cols(), 2);

  samples[0].resize(3, 2);
  samples[1].resize(3, 2);
  EXPECT_TRUE(CompareMatrices(zoh.value(0.25), samples[0]));
  EXPECT_TRUE(CompareMatrices(zoh.value(0.75), samples[1]));

  PiecewisePolynomial<double> block = zoh.Block(1, 1, 2, 1);
  EXPECT_EQ(block.rows(), 2);
  EXPECT_EQ(block.cols(), 1);
  EXPECT_EQ(block.start_time(), zoh.start_time());
  EXPECT_EQ(block.end_time(), zoh.end_time());
  EXPECT_EQ(block.get_number_of_segments(), zoh.get_number_of_segments());

  EXPECT_EQ(zoh.Block(0, 0, 1, 1).value(0.25), samples[0].block(0, 0, 1, 1));
  EXPECT_EQ(zoh.Block(2, 1, 1, 1).value(0.75), samples[1].block(2, 1, 1, 1));
  EXPECT_EQ(zoh.Block(1, 1, 2, 1).value(0.75), samples[1].block(1, 1, 2, 1));
}

GTEST_TEST(testPiecewisePolynomial, IsApproxTest) {
  Eigen::VectorXd breaks(3);
  breaks << 0, .5, 1.;
  Eigen::MatrixXd samples(2, 3);
  samples << 1, 2, 3, -5, -4, -3;
  // Make the numbers bigger to exaggerate the tolerance test.
  samples *= 1000;

  const PiecewisePolynomial<double> pp1 =
      PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
  const PiecewisePolynomial<double> pp2 = pp1 + Eigen::Vector2d::Ones();
  EXPECT_FALSE(pp1.isApprox(pp2, 0.1, ToleranceType::kAbsolute));
  EXPECT_TRUE(pp1.isApprox(pp2, 0.1, ToleranceType::kRelative));
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

  // For symbolic samples (and therefore coefficients), value() returns the
  // symbolic form at the specified time.
  const Variable x0("x0");
  const Variable x1("x1");
  const Variable x2("x2");
  const RowVector3<Expression> symbolic_samples(x0, x1, x2);
  const PiecewisePolynomial<Expression> foh_w_symbolic_coeffs =
      PiecewisePolynomial<Expression>::FirstOrderHold(breaks, symbolic_samples);
  EXPECT_TRUE(foh_w_symbolic_coeffs.value(0.25)(0).Expand().EqualTo(0.5 * x0 +
                                                                    0.5 * x1));
}

// Verifies that the derivatives obtained by evaluating a
// `PiecewisePolynomial<AutoDiffXd>` and extracting the gradient of the result
// match those obtained by taking the derivative of the whole trajectory and
// evaluating it at the same point.
GTEST_TEST(PiecewiseTrajectoryTest, AutoDiffDerivativesTest) {
  VectorX<AutoDiffXd> breaks(3);
  breaks << 0, .5, 1.;
  MatrixX<AutoDiffXd> samples(2, 3);
  samples << 1, 1, 2, 2, 0, 3;

  const PiecewisePolynomial<AutoDiffXd> trajectory =
      PiecewisePolynomial<AutoDiffXd>::CubicWithContinuousSecondDerivatives(
          breaks, samples);
  std::unique_ptr<Trajectory<AutoDiffXd>> derivative_trajectory =
      trajectory.MakeDerivative();
  const int num_times = 100;
  VectorX<double> t = VectorX<double>::LinSpaced(
      num_times, ExtractDoubleOrThrow(trajectory.start_time()),
      ExtractDoubleOrThrow(trajectory.end_time()));
  const double tolerance = 20 * std::numeric_limits<double>::epsilon();
  for (int k = 0; k < num_times; ++k) {
    AutoDiffXd t_k = math::InitializeAutoDiff(Vector1d{t(k)})[0];
    MatrixX<double> derivative_value = ExtractGradient(trajectory.value(t_k));
    MatrixX<double> expected_derivative_value =
        DiscardGradient(derivative_trajectory->value(t(k)));
    EXPECT_TRUE(CompareMatrices(derivative_value, expected_derivative_value,
                                tolerance));
  }
}

}  // namespace
}  // namespace trajectories
}  // namespace drake

