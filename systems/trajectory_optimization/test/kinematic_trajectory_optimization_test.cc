#include "planning/kinematic_trajectory_optimization.h"

#include <optional>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic/expression.h"

using drake::Vector1;
using drake::VectorX;
using drake::solvers::ExpressionConstraint;
using drake::solvers::VectorXDecisionVariable;
using std::nullopt;

namespace symbolic = drake::symbolic;

namespace anzu {
namespace planning {
namespace {

std::shared_ptr<ExpressionConstraint> MakeExpressionConstraint(int num) {
  VectorXDecisionVariable position(num);
  for (int i = 0; i < num; i++)
    position(i) = symbolic::Variable("q" + std::to_string(i));
  auto position_squared =
      (VectorX<symbolic::Expression>(1) << position.transpose() * position)
          .finished();
  auto lb = (VectorX<double>(1) << 0.1).finished();
  auto ub = (VectorX<double>(1) << 1.0).finished();
  return std::make_shared<ExpressionConstraint>(position_squared, lb, ub);
}

class KinematicTrajectoryOptimizationTest : public ::testing::Test {
 public:
  KinematicTrajectoryOptimizationTest()
      : num_positions_(3),
        num_control_points_(10),
        spline_order_(5),
        prog_(num_positions_, num_control_points_, spline_order_) {}

 protected:
  const int num_positions_;
  const int num_control_points_;
  const int spline_order_;
  KinematicTrajectoryOptimization prog_;
};

TEST_F(KinematicTrajectoryOptimizationTest, AddFixedPositionConstraint) {
  prog_.AddFixedPositionConstraint(VectorX<double>::Ones(num_positions_), 0.5);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddFixedVelocityConstraint) {
  prog_.AddFixedVelocityConstraint(VectorX<double>::Ones(num_positions_), 0.5);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddFixedAccelerationConstraint) {
  prog_.AddFixedAccelerationConstraint(VectorX<double>::Ones(num_positions_),
                                       0.5);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddPositionBounds) {
  prog_.AddPositionBounds(-VectorX<double>::Ones(num_positions_),
                          VectorX<double>::Ones(num_positions_));
}

TEST_F(KinematicTrajectoryOptimizationTest, AddVelocityBounds) {
  prog_.AddVelocityBounds(-VectorX<double>::Ones(num_positions_),
                          VectorX<double>::Ones(num_positions_));
}

TEST_F(KinematicTrajectoryOptimizationTest, AddDurationBounds) {
  // Try with both bounds.
  prog_.AddDurationBounds(1e-3, 10);
  // Try with upper bound only.
  prog_.AddDurationBounds(nullopt, 10);
  // Try with lower bound only.
  prog_.AddDurationBounds(1e-3, nullopt);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddDurationCost) {
  prog_.AddDurationCost(10);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddVelocityCost) {
  prog_.AddVelocityCost(10);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddAccelerationCost) {
  prog_.AddAccelerationCost(10);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddJerkCost) {
  prog_.AddJerkCost(10);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddGenericPositionConstraint) {
  prog_.AddGenericPositionConstraint(MakeExpressionConstraint(num_positions_),
                                     {{0.2, 0.7}});
}

TEST_F(KinematicTrajectoryOptimizationTest, AddLinearConstraint) {
  prog_.AddLinearConstraint(
      prog_.position()(0) <= prog_.velocity()(num_positions_ - 1),
      {{0.3, 0.4}});
}

TEST_F(KinematicTrajectoryOptimizationTest, Solve) {
  prog_.Solve();
  prog_.Solve(true);
}

TEST_F(KinematicTrajectoryOptimizationTest, SolveWithEverything) {
  prog_.AddFixedPositionConstraint(VectorX<double>::Ones(num_positions_), 0.5);
  prog_.AddPositionBounds(-VectorX<double>::Ones(num_positions_),
                          VectorX<double>::Ones(num_positions_));
  prog_.AddVelocityBounds(-VectorX<double>::Ones(num_positions_),
                          VectorX<double>::Ones(num_positions_));
  prog_.AddDurationBounds(1e-3, 10);
  prog_.AddDurationCost(10);
  prog_.AddVelocityCost(10);
  prog_.AddAccelerationCost(10);
  prog_.AddJerkCost(10);
  prog_.AddGenericPositionConstraint(MakeExpressionConstraint(num_positions_),
                                     {{0.2, 0.7}});
  prog_.AddLinearConstraint(
      prog_.position()(0) <= prog_.velocity()(num_positions_ - 1),
      {{0.3, 0.4}});
  prog_.Solve();
  prog_.Solve(true);
}

}  // namespace
}  // namespace planning
}  // namespace anzu
