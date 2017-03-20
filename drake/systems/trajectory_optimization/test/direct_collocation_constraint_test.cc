#include "drake/systems/trajectory_optimization/direct_collocation_constraint.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"

namespace drake {
namespace systems {
namespace {

class PendulumTestDirectCollocationConstraint
    : public DirectCollocationConstraint {
 public:
  PendulumTestDirectCollocationConstraint()
      : DirectCollocationConstraint(2,  // num_states
                                    1)  // num_inputs
  {}

 protected:
  void dynamics(const TaylorVecXd& state, const TaylorVecXd& input,
                TaylorVecXd* xdot) const override {
    // From the Pendulum example:
    const double m = 1.0;
    const double b = 0.1;
    const double lc = .5;
    const double I = 0.25;
    const double g = 9.81;

    ASSERT_EQ(state.size(), 2);
    ASSERT_EQ(input.size(), 1);
    xdot->resize(2);
    (*xdot)(0) = state(1);
    (*xdot)(1) = (input(0) - m * g * lc * sin(state(0)) - b * state(1)) / I;
  }
};

GTEST_TEST(DirectCollocationConstraintPendulumDynamicsTest,
           DirectCollocationConstraintTest) {
  const int kNumStates = 2;
  const int kNumInputs = 1;

  // Initial state/input and expected result values came from running
  // the MATLAB code for PendulumPlant through the constraint function
  // in DircolTrajectoryOptimization.m and printing the results.
  Eigen::VectorXd x(1 + 2 * kNumStates + 2 * kNumInputs);
  x(0) = 0.2;          // h
  x(1) = 0;            // x0(0)
  x(2) = 0;            // x0(1)
  x(3) = 0.05 * M_PI;  // x1(0)
  x(4) = 0;            // x1(1)
  x(5) = 0.00537668;   // u0
  x(6) = 0.018339;     // u1

  PendulumTestDirectCollocationConstraint dut;

  TaylorVecXd result;
  dut.Eval(math::initializeAutoDiff(x), result);

  EXPECT_NEAR(result(0).value(), 1.1027, 1e-4);
  EXPECT_NEAR(result(1).value(), 2.2657, 1e-4);

  Eigen::VectorXd d_0_expected(x.size());
  d_0_expected << -6.26766, -7.0095, -0.74, 7.015539, -0.76, -0.1, 0.1;
  Eigen::VectorXd d_1_expected(x.size());
  d_1_expected << 0.1508698, 14.488559, -6.715012, 14.818155, 7.315012, -2.96,
      -3.04;
  EXPECT_TRUE(CompareMatrices(result(0).derivatives(), d_0_expected, 1e-4,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(result(1).derivatives(), d_1_expected, 1e-4,
                              MatrixCompareType::absolute));
}

}  // anonymous namespace
}  // namespace systems
}  // namespace drake
