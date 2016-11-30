#include "drake/system1/vector.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/test/pendulum.h"

using drake::CombinedVector;
using drake::size;
using drake::CombinedVectorUtil;
using drake::NullVector;
using std::is_same;
using std::string;

namespace drake {
namespace systems {
namespace test {
namespace {

// Tests the ability to set a PendulumState equal to a vector and vice versa.
GTEST_TEST(VectorTest, ValueAssignment) {
  Eigen::Vector2d x;
  x << 0.2, 0.4;

  PendulumState<double> state;
  state.theta = 0.2;
  state.thetadot = 0.3;

  EXPECT_EQ(size(state), 2u);

  state = x;
  EXPECT_EQ(state.theta, 0.2);
  EXPECT_EQ(state.thetadot, 0.4);

  state.theta = 0.5;
  x = toEigen(state);
  EXPECT_EQ(x(0), 0.5);

  Eigen::VectorXd y = toEigen(state);
  const double tolerance = 1e-8;

  std::string error_msg;
  EXPECT_TRUE(
      CompareMatrices(x, y, tolerance, MatrixCompareType::absolute, &error_msg))
      << error_msg;
}

// Tests the ability to set a CombinedVector's value
GTEST_TEST(VectorTest, CombinedVector) {
  Eigen::Vector3d abc;
  abc << 1, 2, 3;

  CombinedVector<double, PendulumState, PendulumInput> test(abc);
  test = 2 * abc;

  EXPECT_EQ(test.first().theta, 2.0);
  EXPECT_EQ(test.first().thetadot, 4.0);
  EXPECT_EQ(test.second().tau, 6.0);
}

// Tests the ability to use a CombinedVectorUtil
GTEST_TEST(VectorTest, CombinedVectorUtil) {
  Eigen::Vector3d abc;
  abc << 1, 2, 3;

  CombinedVectorUtil<PendulumState, PendulumInput>::type<double> test(abc);
  test = 2 * abc;
  EXPECT_EQ(test.first().theta, 2.0);
  EXPECT_EQ(test.first().thetadot, 4.0);
  EXPECT_EQ(test.second().tau, 6.0);
}

// Verify that combining a vector with an unused or empty vector returns the
// original type
GTEST_TEST(VectorTest, CombineVectorCornerCases) {
  CombinedVectorUtil<PendulumState, NullVector>::type<double> test1;
  EXPECT_TRUE((is_same<PendulumState<double>, decltype(test1)>::value))
      << "combined vector builder returned " +
             static_cast<string>(typeid(test1).name());

  CombinedVectorUtil<NullVector, PendulumState>::type<double> test2;
  EXPECT_TRUE((is_same<PendulumState<double>, decltype(test2)>::value))
      << "combined vector builder returned " +
             static_cast<string>(typeid(test2).name());
}

// Tests the RowsAtCompileTime
GTEST_TEST(VectorTest, RowsAtCompileTime) {
  EXPECT_EQ((Eigen::Matrix<double, 2, 1>::RowsAtCompileTime), 2)
      << "failed to evaluate RowsAtCompileTime";
}

}  // namespace
}  // namespace test
}  // namespace systems
}  // namespace drake
