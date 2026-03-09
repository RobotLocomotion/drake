#include "drake/common/trajectories/trajectory.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace trajectories {

namespace {

// Dummy implementation of Trajectory to test the basic functions.
template <typename T = double>
class TrajectoryTester : public Trajectory<T> {
 public:
  explicit TrajectoryTester(bool has_derivative)
      : has_derivative_(has_derivative) {}

 private:
  std::unique_ptr<Trajectory<T>> DoClone() const override { return nullptr; }
  MatrixX<T> do_value(const T& t) const override { return MatrixX<T>(0, 0); }
  bool do_has_derivative() const override { return has_derivative_; }
  Eigen::Index do_rows() const override { return 0; }
  Eigen::Index do_cols() const override { return 0; }
  T do_start_time() const override { return 0; }
  T do_end_time() const override { return 1; }

  bool has_derivative_;
};

GTEST_TEST(TrajectoryTest, EvalDerivativesTest) {
  TrajectoryTester traj_yes_deriv(true);
  EXPECT_TRUE(traj_yes_deriv.has_derivative());
  DRAKE_EXPECT_THROWS_MESSAGE(
      traj_yes_deriv.EvalDerivative(traj_yes_deriv.start_time()),
      ".* must implement .*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      traj_yes_deriv.EvalDerivative(traj_yes_deriv.start_time(), -1),
      ".*derivative_order >= 0.*");

  TrajectoryTester traj_no_deriv(false);
  EXPECT_FALSE(traj_no_deriv.has_derivative());
  DRAKE_EXPECT_THROWS_MESSAGE(
      traj_no_deriv.EvalDerivative(traj_no_deriv.start_time()),
      ".* does not support .*");
}

GTEST_TEST(TrajectoryTest, MakeDerivativesTest) {
  TrajectoryTester traj_yes_deriv(true);
  EXPECT_TRUE(traj_yes_deriv.has_derivative());
  DRAKE_EXPECT_THROWS_MESSAGE(traj_yes_deriv.MakeDerivative(),
                              ".* must implement .*");

  TrajectoryTester traj_no_deriv(false);
  EXPECT_FALSE(traj_no_deriv.has_derivative());
  DRAKE_EXPECT_THROWS_MESSAGE(traj_no_deriv.MakeDerivative(),
                              ".* does not support .*");
}

template <typename T>
void TestScalarType() {
  TrajectoryTester<T> traj(true);

  const MatrixX<T> value = traj.value(traj.start_time());
  EXPECT_EQ(value.rows(), 0);
  EXPECT_EQ(value.cols(), 0);
}

GTEST_TEST(PiecewiseTrajectoryTest, ScalarTypes) {
  TestScalarType<AutoDiffXd>();
  TestScalarType<symbolic::Expression>();
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
