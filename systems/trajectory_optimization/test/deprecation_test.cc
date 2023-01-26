// This file serves to check that our deprecation shims compile successfully,
// aside from warning messages.  We can remove this file entirely once the
// deprecation period ends.

#include <gtest/gtest.h>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"
#include "drake/systems/trajectory_optimization/integration_constraint.h"
#include "drake/systems/trajectory_optimization/kinematic_trajectory_optimization.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"
#include "drake/systems/trajectory_optimization/sequential_expression_manager.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace {

using trajectories::PiecewisePolynomial;

GTEST_TEST(PathDeprecation, DirectCollocation) {
  // Exercise the *single* type declared in the header.
  // xdot = -x.
  systems::LinearSystem<double> plant(
      Vector1d(-1.0),                        // A
      Eigen::Matrix<double, 1, 0>::Zero(),   // B
      Eigen::Matrix<double, 0, 1>::Zero(),   // C
      Eigen::Matrix<double, 0, 0>::Zero());  // D

  auto context = plant.CreateDefaultContext();
  const int kNumSampleTimes{10};
  const double kFixedTimeStep{0.1};
  EXPECT_NO_THROW(DirectCollocation(&plant, *context, kNumSampleTimes,
                                    kFixedTimeStep, kFixedTimeStep));
}

GTEST_TEST(PathDeprecation, DirectTranscription) {
  Eigen::Matrix2d A, B;
  // clang-format off
  A << 1, 2,
       3, 4;
  B << 5, 6,
       7, 8;
  // clang-format on
  const Eigen::MatrixXd C(0, 2), D(0, 2);
  const double kTimeStep = .1;
  LinearSystem<double> system(A, B, C, D, kTimeStep);

  const auto context = system.CreateDefaultContext();
  int kNumSampleTimes = 3;

  // Main class defined in header.
  EXPECT_NO_THROW(DirectTranscription(&system, *context, kNumSampleTimes));

  // Supporting struct defined in header.
  EXPECT_NO_THROW(TimeStep(0.5));
}

// Minimum empty class to confirm that the base class exists.
class MyDirectTrajOpt : public MultipleShooting {
 public:
  // Arbitrary constants that make the base constructor sufficiently happy.
  MyDirectTrajOpt() : MultipleShooting(1, 1, 2, 1) {}

  PiecewisePolynomial<double> ReconstructInputTrajectory(
      const solvers::MathematicalProgramResult&) const override {
    return PiecewisePolynomial<double>();
  };

  PiecewisePolynomial<double> ReconstructStateTrajectory(
      const solvers::MathematicalProgramResult&) const override {
    return PiecewisePolynomial<double>();
  };

 private:
  void DoAddRunningCost(const symbolic::Expression& g) override {}
};

GTEST_TEST(PathDeprecation, MultipleShootings) {
  // Include *both* types declared in the header file.
  EXPECT_NO_THROW(MyDirectTrajOpt());
}

GTEST_TEST(PathDeprecation, IntegrationConstraint) {
  // Arbitrary constants that make the base constructor sufficiently happy.
  EXPECT_NO_THROW(MidPointIntegrationConstraint(1));
}

GTEST_TEST(PathDeprecation, KinematicTrajectoryOptimization) {
  // Arbitrary constants that make the base constructor sufficiently happy.
  EXPECT_NO_THROW(KinematicTrajectoryOptimization(5, 5));
}

GTEST_TEST(PathDeprecation, SequentialExpressionManager) {
  // Arbitrary constants that make the base constructor sufficiently happy.
  EXPECT_NO_THROW(internal::SequentialExpressionManager(1));
}

}  // namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
