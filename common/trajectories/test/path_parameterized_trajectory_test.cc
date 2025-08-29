#include "drake/common/trajectories/path_parameterized_trajectory.h"

#include <algorithm>
#include <memory>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace trajectories {
namespace {

class PathParameterizedTrajectoryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    VectorX<double> times(3);
    RowVectorX<double> s_samples(3);
    MatrixX<double> x_samples(3, 2);

    times << 1, 1.5, 2;
    s_samples << 3, 4.5, 5;
    x_samples.leftCols(1) << 0.3, 0, -0.5;
    x_samples.rightCols(1) << 1, -1, 3;

    time_scaling_ = PiecewisePolynomial<double>::CubicShapePreserving(
        times, s_samples, true);
    path_ = PiecewisePolynomial<double>::CubicShapePreserving(
        Vector2<double>(s_samples(0), s_samples(2)), x_samples, true);
    dut_ = std::make_unique<PathParameterizedTrajectory<double>>(path_,
                                                                 time_scaling_);
  }
  PiecewisePolynomial<double> path_;
  PiecewisePolynomial<double> time_scaling_;
  std::unique_ptr<PathParameterizedTrajectory<double>> dut_;
};

TEST_F(PathParameterizedTrajectoryTest, TestConstructor) {
  EXPECT_EQ(dut_->start_time(), 1.0);
  EXPECT_EQ(dut_->end_time(), 2.0);
}

TEST_F(PathParameterizedTrajectoryTest, TestValue) {
  for (double t = 0.88; t < 2.2; t += 0.12) {
    const double time =
        std::clamp(t, time_scaling_.start_time(), time_scaling_.end_time());
    EXPECT_TRUE(CompareMatrices(
        dut_->value(t), path_.value(time_scaling_.scalarValue(time)), 1e-14));
  }
}

TEST_F(PathParameterizedTrajectoryTest, TestDerivatives) {
  for (double t = 0.88; t < 2.2; t += 0.12) {
    const double time =
        std::clamp(t, time_scaling_.start_time(), time_scaling_.end_time());

    EXPECT_TRUE(
        CompareMatrices(dut_->EvalDerivative(t, 0), dut_->value(t), 1e-14));

    double s = time_scaling_.scalarValue(time);
    double s_dot = time_scaling_.EvalDerivative(time, 1)(0, 0);
    VectorX<double> x_dot = path_.EvalDerivative(s, 1) * s_dot;
    EXPECT_TRUE(CompareMatrices(dut_->EvalDerivative(t, 1), x_dot, 1e-14));
    auto deriv = dut_->MakeDerivative(1);
    EXPECT_TRUE(CompareMatrices(deriv->value(t), x_dot, 1e-14));

    double s_ddot = time_scaling_.EvalDerivative(time, 2)(0, 0);
    VectorX<double> x_ddot = path_.EvalDerivative(s, 1) * s_ddot +
                             path_.EvalDerivative(s, 2) * s_dot * s_dot;
    EXPECT_TRUE(CompareMatrices(dut_->EvalDerivative(t, 2), x_ddot, 1e-14));
    auto deriv2 = dut_->MakeDerivative(2);
    EXPECT_TRUE(CompareMatrices(deriv2->value(t), x_ddot, 1e-14));

    double s_3dot = time_scaling_.EvalDerivative(time, 3)(0, 0);
    VectorX<double> x_3dot = path_.EvalDerivative(s, 1) * s_3dot +
                             3 * path_.EvalDerivative(s, 2) * s_ddot * s_dot +
                             path_.EvalDerivative(s, 3) * std::pow(s_dot, 3);
    EXPECT_TRUE(CompareMatrices(dut_->EvalDerivative(t, 3), x_3dot, 1e-12));
    auto deriv3 = dut_->MakeDerivative(3);
    EXPECT_TRUE(CompareMatrices(deriv3->value(t), x_3dot, 1e-12));
  }
}

// Tests getters.
TEST_F(PathParameterizedTrajectoryTest, TestGetTrajectory) {
  EXPECT_TRUE(dynamic_cast<const PiecewisePolynomial<double>&>(dut_->path())
                  .isApprox(path_, 0));
  EXPECT_TRUE(
      dynamic_cast<const PiecewisePolynomial<double>&>(dut_->time_scaling())
          .isApprox(time_scaling_, 0));
}

template <typename T>
void TestScalarType() {
  VectorX<T> times(2);
  VectorX<T> s_samples(2);
  MatrixX<T> x_samples(3, 2);

  times << 1, 2;
  s_samples << 3, 5;
  x_samples.leftCols(1) << 0.3, 0, -0.5;
  x_samples.rightCols(1) << 1, -1, 3;

  PiecewisePolynomial<T> time_scaling =
      PiecewisePolynomial<T>::FirstOrderHold(times, s_samples.transpose());
  PiecewisePolynomial<T> path =
      PiecewisePolynomial<T>::FirstOrderHold(s_samples, x_samples);
  PathParameterizedTrajectory<T> trajectory(path, time_scaling);
}

GTEST_TEST(PathParameterizedTrajectoryScalarTest, ScalarTypes) {
  TestScalarType<double>();
  TestScalarType<AutoDiffXd>();
  TestScalarType<symbolic::Expression>();
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
