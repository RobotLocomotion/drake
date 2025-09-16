#include "drake/common/trajectories/derivative_trajectory.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/trajectories/discrete_time_trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"

namespace drake {
namespace trajectories {
namespace {

using Eigen::Vector2d;
using Vector1d = Vector1<double>;

// Create an arbitrary piecewise polynomial trajectory.
template <typename T>
PiecewisePolynomial<T> MakePP() {
  return PiecewisePolynomial<T>::CubicShapePreserving(
      {0.2, 0.3, 0.7, 0.8}, {Vector2d(10, -100), Vector2d(20, 0),
                             Vector2d(90, 10), Vector2d(100, 200)});
}

GTEST_TEST(DerivativeTrajectoryTest, BasicTest) {
  const PiecewisePolynomial<double> nominal = MakePP<double>();
  for (int derivative_order = 0; derivative_order < 3; ++derivative_order) {
    const DerivativeTrajectory<double> dut(nominal, derivative_order);
    EXPECT_EQ(dut.rows(), nominal.rows());
    EXPECT_EQ(dut.cols(), nominal.cols());
    EXPECT_TRUE(dut.has_derivative());
    EXPECT_EQ(dut.start_time(), nominal.start_time());
    EXPECT_EQ(dut.end_time(), nominal.end_time());
    const double t = 0.45;
    EXPECT_TRUE(CompareMatrices(dut.value(t),
                                nominal.EvalDerivative(t, derivative_order)));
    auto clone = dut.Clone();
    EXPECT_TRUE(CompareMatrices(clone->value(t),
                                nominal.EvalDerivative(t, derivative_order)));
    for (int i = 0; i < 2; ++i) {
      EXPECT_TRUE(
          CompareMatrices(dut.EvalDerivative(t, i),
                          nominal.EvalDerivative(t, derivative_order + i)));
      auto deriv = dut.MakeDerivative(i);
      EXPECT_TRUE(CompareMatrices(
          deriv->value(t), nominal.EvalDerivative(t, derivative_order + i)));
    }
  }
}

GTEST_TEST(DerivativeTrajectoryTest, ThrowsIfNoDerivative) {
  DiscreteTimeTrajectory<double> nominal({0.1, 0.4},
                                         {Vector1d{0.3}, Vector1d{0.5}});
  DRAKE_EXPECT_THROWS_MESSAGE(DerivativeTrajectory(nominal),
                              ".*has_derivative.*failed.*");
}

GTEST_TEST(DerivativeTrajectoryTest, ThrowsIfNegativeOrder) {
  const PiecewisePolynomial<double> nominal = MakePP<double>();
  DRAKE_EXPECT_THROWS_MESSAGE(DerivativeTrajectory(nominal, -1),
                              ".*derivative_order.*failed.*");
}

// PiecewiseQuaternion has the feature that the number of rows of the
// derivative is different than the rows of the nominal.
GTEST_TEST(DerivativeTrajectoryTest, PiecewiseQuaternion) {
  const std::vector<double> time = {0, 1.6, 2.32};
  const std::vector<double> ang = {1, 2.4 - 2 * M_PI, 5.3};
  const Vector3<double> axis = Vector3<double>(1, 2, 3).normalized();
  std::vector<Quaternion<double>> quat(ang.size());
  for (size_t i = 0; i < ang.size(); ++i) {
    quat[i] = Quaternion<double>(AngleAxis<double>(ang[i], axis));
  }

  const PiecewiseQuaternionSlerp<double> nominal(time, quat);
  for (int derivative_order = 0; derivative_order < 3; ++derivative_order) {
    DerivativeTrajectory dut(nominal, derivative_order);
    EXPECT_EQ(dut.rows(), derivative_order == 0 ? 4 : 3);
    const double t = 0.45;
    EXPECT_TRUE(CompareMatrices(dut.value(t),
                                nominal.EvalDerivative(t, derivative_order)));
  }
}

template <typename T>
void TestScalar() {
  const PiecewisePolynomial<T> nominal = MakePP<T>();
  const int derivative_order = 1;
  const DerivativeTrajectory<T> dut(nominal, derivative_order);
  const T time{0.62};
  EXPECT_EQ(dut.value(time), nominal.EvalDerivative(time));
}

GTEST_TEST(DerivativeTrajectoryTest, DefaultScalars) {
  TestScalar<double>();
  TestScalar<AutoDiffXd>();
  TestScalar<symbolic::Expression>();
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
