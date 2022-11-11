#include "drake/common/trajectories/stacked_trajectory.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/trajectories/discrete_time_trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace trajectories {
namespace {

using Eigen::MatrixXd;
using Eigen::RowVector2d;
using Eigen::RowVector4d;
using Eigen::Vector2d;
using Eigen::Vector4d;
using Vector1d = Vector1<double>;

DiscreteTimeTrajectory<double> MakeDiscrete(
    double t0, const MatrixXd& value0, double t1, const MatrixXd& value1) {
  std::vector<double> times{t0, t1};
  std::vector<MatrixXd> values{value0, value1};
  return DiscreteTimeTrajectory<double>(times, values);
}

GTEST_TEST(StackedTrajectoryTest, Empty) {
  const StackedTrajectory<double> dut;
  EXPECT_EQ(dut.rows(), 0);
  EXPECT_EQ(dut.cols(), 0);
  EXPECT_EQ(dut.start_time(), 0.0);
  EXPECT_EQ(dut.end_time(), 0.0);
  EXPECT_NO_THROW(dut.Clone());
  EXPECT_TRUE(CompareMatrices(dut.value(0), MatrixXd::Zero(0, 0)));
  EXPECT_TRUE(dut.has_derivative());
  EXPECT_TRUE(CompareMatrices(dut.EvalDerivative(0), MatrixXd::Zero(0, 0)));
  EXPECT_TRUE(dut.MakeDerivative() != nullptr);
}

GTEST_TEST(StackedTrajectoryTest, CopyCtor) {
  const auto x = Vector1d(22);
  StackedTrajectory<double> dut;
  dut.Append(MakeDiscrete(0, x, 1, -x));
  EXPECT_EQ(dut.rows(), 1);
  EXPECT_EQ(dut.cols(), 1);
  EXPECT_TRUE(CompareMatrices(dut.value(0), x));

  // The 'other' copies the data from 'dut'.
  StackedTrajectory<double> other(dut);
  EXPECT_EQ(other.rows(), 1);
  EXPECT_EQ(other.cols(), 1);
  EXPECT_TRUE(CompareMatrices(other.value(0), x));
  EXPECT_NO_THROW(other.Clone());

  // The 'dut' is unchanged.
  EXPECT_EQ(dut.rows(), 1);
  EXPECT_EQ(dut.cols(), 1);
  EXPECT_TRUE(CompareMatrices(dut.value(0), x));
  EXPECT_NO_THROW(dut.Clone());

  // Changing 'dut' does not affect 'other'.
  dut.Append(MakeDiscrete(0, x, 1, -x));
  EXPECT_EQ(dut.rows(), 1 + 1);
  EXPECT_EQ(dut.cols(), 1);
  EXPECT_EQ(other.rows(), 1);
  EXPECT_EQ(other.cols(), 1);
}

GTEST_TEST(StackedTrajectoryTest, MoveCtor) {
  const auto x = Vector1d(22);
  StackedTrajectory<double> dut;
  dut.Append(MakeDiscrete(0, x, 1, -x));
  EXPECT_EQ(dut.rows(), 1);
  EXPECT_EQ(dut.cols(), 1);
  EXPECT_TRUE(CompareMatrices(dut.value(0), x));

  // The 'other' takes over the data from 'dut'.
  StackedTrajectory<double> other(std::move(dut));
  EXPECT_EQ(other.rows(), 1);
  EXPECT_EQ(other.cols(), 1);
  EXPECT_TRUE(CompareMatrices(other.value(0), x));
  EXPECT_NO_THROW(other.Clone());

  // The 'dut' is empty now.
  EXPECT_EQ(dut.rows(), 0);
  EXPECT_EQ(dut.cols(), 0);
  EXPECT_TRUE(CompareMatrices(dut.value(0), MatrixXd::Zero(0, 0)));
  EXPECT_NO_THROW(dut.Clone());
}

GTEST_TEST(StackedTrajectoryTest, AppendMismatchedTimes) {
  StackedTrajectory<double> dut;
  const auto x = Vector1d(22);
  auto time_zero_two = MakeDiscrete(0, x, 2, -x);
  auto time_zero_one = MakeDiscrete(0, x, 1, -x);
  auto time_one_two = MakeDiscrete(1, x, 2, -x);
  dut.Append(time_zero_two);
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Append(time_one_two), ".*start_time.*");
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Append(time_zero_one), ".*end_time.*");
}

GTEST_TEST(StackedTrajectoryTest, AppendMismatchedSizesRowwise) {
  const auto x = Vector2d(11, 22);
  const auto y = RowVector2d(33, 44);
  StackedTrajectory<double> dut;
  dut.Append(MakeDiscrete(0, x, 1, -x));
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Append(MakeDiscrete(0, y, 1, -y)),
                              ".*cols.*");
}

GTEST_TEST(StackedTrajectoryTest, AppendMismatchedSizesColwise) {
  const auto x = Vector2d(11, 22);
  const auto y = RowVector2d(33, 44);
  StackedTrajectory<double> dut(/* rowwise = */ false);
  dut.Append(MakeDiscrete(0, y, 1, -y));
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Append(MakeDiscrete(0, x, 1, -x)),
                              ".*rows.*");
}

GTEST_TEST(StackedTrajectoryTest, StackTwoDiscreteColumnVectors) {
  const double t0 = 0.2;
  const double tf = 0.8;

  StackedTrajectory<double> dut;
  dut.Append(MakeDiscrete(t0, Vector2d(1, 2), tf, Vector2d(11, 12)));
  dut.Append(MakeDiscrete(t0, Vector2d(3, 4), tf, Vector2d(13, 14)));
  EXPECT_EQ(dut.rows(), 4);
  EXPECT_EQ(dut.cols(), 1);
  EXPECT_EQ(dut.start_time(), t0);
  EXPECT_EQ(dut.end_time(), tf);
  EXPECT_NO_THROW(dut.Clone());
  EXPECT_TRUE(CompareMatrices(dut.value(t0), Vector4d(1, 2, 3, 4)));
  EXPECT_TRUE(CompareMatrices(dut.value(tf), Vector4d(11, 12, 13, 14)));
  EXPECT_FALSE(dut.has_derivative());
}

GTEST_TEST(StackedTrajectoryTest, StackTwoDiscreteRowVectors) {
  const double t0 = 0.2;
  const double tf = 0.8;

  StackedTrajectory<double> dut(/* rowwise = */ false);
  dut.Append(MakeDiscrete(t0, RowVector2d(1, 2), tf, RowVector2d(11, 12)));
  dut.Append(MakeDiscrete(t0, RowVector2d(3, 4), tf, RowVector2d(13, 14)));
  EXPECT_EQ(dut.rows(), 1);
  EXPECT_EQ(dut.cols(), 4);
  EXPECT_EQ(dut.start_time(), t0);
  EXPECT_EQ(dut.end_time(), tf);
  EXPECT_NO_THROW(dut.Clone());
  EXPECT_TRUE(CompareMatrices(dut.value(t0), RowVector4d(1, 2, 3, 4)));
  EXPECT_TRUE(CompareMatrices(dut.value(tf), RowVector4d(11, 12, 13, 14)));
  EXPECT_FALSE(dut.has_derivative());
}

GTEST_TEST(StackedTrajectoryTest, Clone) {
  const auto x = Vector1d(22);
  StackedTrajectory<double> original;
  original.Append(MakeDiscrete(0, x, 1, -x));
  original.Append(MakeDiscrete(0, -x, 1, x));

  Trajectory<double>& upcast = original;
  auto clone = upcast.Clone();

  // The original and the clone are the same.
  for (const auto* item : {&upcast, clone.get()}) {
    EXPECT_EQ(item->rows(), 2);
    EXPECT_EQ(item->cols(), 1);
    EXPECT_EQ(item->start_time(), 0);
    EXPECT_EQ(item->end_time(), 1);
    EXPECT_TRUE(CompareMatrices(item->value(0), Vector2d(22, -22)));
    EXPECT_TRUE(CompareMatrices(item->value(1), Vector2d(-22, 22)));
  }
}

GTEST_TEST(StackedTrajectoryTest, HasDerivative) {
  const double t0 = 0.2;
  const double tf = 0.8;
  const auto x = Vector1d(22);

  StackedTrajectory<double> dut;
  EXPECT_EQ(dut.has_derivative(), true);

  // When we append a polynomial, we still have derivatives.
  auto zoh = PiecewisePolynomial<double>::ZeroOrderHold({t0, tf}, {x, x});
  dut.Append(zoh);
  EXPECT_EQ(dut.has_derivative(), true);

  // When we append a discrete, we no longer have derivatives.
  dut.Append(MakeDiscrete(t0, x, tf, -x));
  EXPECT_EQ(dut.has_derivative(), false);
}

GTEST_TEST(StackedTrajectoryTest, MakeDerivative) {
  const double t0 = 0.2;
  const double tf = 0.8;
  const auto x = RowVector2d(22, -22);
  auto zoh = PiecewisePolynomial<double>::ZeroOrderHold({t0, tf}, {x, x});

  // Create a stack of zoh.
  // Do it colwise to make sure the non-default option is propagated.
  StackedTrajectory<double> dut(/* rowwise = */ false);
  dut.Append(zoh);
  dut.Append(zoh);
  dut.Append(zoh);

  // Relying on glass-box testing, we'll presume that when the result has the
  // correct shape and doesn't crash we don't need to check the derivative
  // for mathematical correctness.
  Trajectory<double>& upcast = dut;
  auto deriv = dut.MakeDerivative();
  for (const auto* item : {&upcast, deriv.get()}) {
    EXPECT_EQ(item->rows(), 1);
    EXPECT_EQ(item->cols(), 6);
    EXPECT_NO_THROW(item->value(t0));
  }
}

GTEST_TEST(StackedTrajectoryTest, EvalDerivative) {
  for (bool rowwise : {true, false}) {
    SCOPED_TRACE(fmt::format("Using rowwise = {}", rowwise));

    const double t0 = 0.2;
    const double t1 = 0.3;
    const double t2 = 0.7;
    const double tf = 0.8;

    // Create a stack of arbitrary polynomials.
    auto make_poly = [&](const std::vector<MatrixXd>& samples) {
      return PiecewisePolynomial<double>::CubicShapePreserving(
          {t0, t1, t2, tf}, samples);
    };
    StackedTrajectory<double> dut(rowwise);
    dut.Append(make_poly({
        Vector2d(10, -100),
        Vector2d(20, 0),
        Vector2d(90, 10),
        Vector2d(100, 200)}));
    dut.Append(make_poly({
        Vector2d(0, -1),
        Vector2d(50, -2),
        Vector2d(100, -4),
        Vector2d(200, -8)}));

    // We'll assume that MakeDerivative is correct and use its values as the
    // gold standard.
    auto deriv = dut.MakeDerivative(2);
    for (int i = 0; i < 100; ++i) {
      const double t = (i / 99.0) * (tf - t0);
      MatrixXd actual = dut.EvalDerivative(t, 2);
      MatrixXd expected = deriv->value(t);
      ASSERT_TRUE(CompareMatrices(actual, expected));
    }
  }
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
