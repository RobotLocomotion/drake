#include "drake/systems/trajectory_optimization/kinematic_trajectory_optimization.h"

#include <optional>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/text_logging.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/solve.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::nullopt;

namespace drake {
namespace systems {
namespace trajectory_optimization {

using solvers::ExpressionConstraint;
using solvers::MathematicalProgramResult;
using solvers::Solve;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using trajectories::BsplineTrajectory;

namespace {

class KinematicTrajectoryOptimizationTest : public ::testing::Test {
 public:
  KinematicTrajectoryOptimizationTest()
      : num_positions_(3),
        num_control_points_(10),
        spline_order_(5),
        trajopt_(num_positions_, num_control_points_, spline_order_) {}

 protected:
  const int num_positions_;
  const int num_control_points_;
  const int spline_order_;
  KinematicTrajectoryOptimization trajopt_;
};

// In this test (and the ones below), we confirm that the constraint which is
// added is the *type* of constraint that we expect; in particular we want to
// confirm that constraints we expect to be convex are indeed convex.
TEST_F(KinematicTrajectoryOptimizationTest, AddPathPositionConstraint) {
  EXPECT_EQ(trajopt_.prog().linear_constraints().size(), 0);
  VectorXd desired = VectorXd::Constant(num_positions_, 3.75);
  trajopt_.AddPathPositionConstraint(desired, desired, 0.5);
  EXPECT_EQ(trajopt_.prog().linear_constraints().size(), 1);

  trajopt_.AddDurationConstraint(1, 1);
  auto result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  auto q = trajopt_.ReconstructTrajectory(result);
  EXPECT_TRUE(CompareMatrices(q.value(0.5), desired, 1e-6));
}

// 0.1 <= |position|² <= 1
std::shared_ptr<ExpressionConstraint> MakeExpressionConstraint(int num) {
  auto r = symbolic::MakeVectorVariable(num, "q");
  Vector1<Expression> r_squared = r.transpose() * r;
  const Vector1d lb{0.1};
  const Vector1d ub{1.0};
  return std::make_shared<ExpressionConstraint>(r_squared, lb, ub);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddPathPositionConstraintGeneric) {
  EXPECT_EQ(trajopt_.prog().generic_constraints().size(), 0);
  trajopt_.AddPathPositionConstraint(MakeExpressionConstraint(num_positions_),
                                     0.2);
  EXPECT_EQ(trajopt_.prog().generic_constraints().size(), 1);

  trajopt_.AddDurationConstraint(1, 1);
  // Snopt does not converge from the default zero trajectory.
  trajopt_.SetInitialGuess(BsplineTrajectory(
      trajopt_.basis(), math::EigenToStdVector<double>(
                            MatrixXd::Ones(3, trajopt_.num_control_points()))));
  auto result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  double squared_norm =
      trajopt_.ReconstructTrajectory(result).value(0.2).squaredNorm();
  const double kTol = 1e-6;
  EXPECT_GE(squared_norm, 0.1 - kTol);
  EXPECT_LE(squared_norm, 1.0 + kTol);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddPathVelocityConstraint) {
  EXPECT_EQ(trajopt_.prog().linear_constraints().size(), 0);
  VectorXd desired = VectorXd::Ones(num_positions_);
  trajopt_.AddPathVelocityConstraint(desired, desired, 0.5);
  EXPECT_EQ(trajopt_.prog().linear_constraints().size(), 1);

  // r(0) = 0, r(1) = 1.
  trajopt_.AddPathPositionConstraint(Vector3d::Zero(), Vector3d::Zero(), 0);
  trajopt_.AddPathPositionConstraint(Vector3d::Ones(), Vector3d::Ones(), 1);

  trajopt_.AddDurationConstraint(1, 1);
  auto result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  auto q = trajopt_.ReconstructTrajectory(result);
  auto qdot = q.MakeDerivative();
  EXPECT_TRUE(CompareMatrices(qdot->value(0.5), desired, 1e-6));
}

TEST_F(KinematicTrajectoryOptimizationTest, AddPathAccelerationConstraint) {
  EXPECT_EQ(trajopt_.prog().linear_constraints().size(), 0);
  VectorXd desired = VectorXd::Ones(num_positions_);
  trajopt_.AddPathAccelerationConstraint(desired, desired, 0.5);
  EXPECT_EQ(trajopt_.prog().linear_constraints().size(), 1);

  trajopt_.AddDurationConstraint(1.0, 1.0);
  auto result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  auto q = trajopt_.ReconstructTrajectory(result);
  auto qdot = q.MakeDerivative();
  auto qddot = qdot->MakeDerivative();
  EXPECT_TRUE(CompareMatrices(qddot->value(0.5), desired, 1e-6));
}

TEST_F(KinematicTrajectoryOptimizationTest, AddDurationConstraint) {
  EXPECT_EQ(trajopt_.prog().bounding_box_constraints().size(), 1);
  // Try with both bounds.
  trajopt_.AddDurationConstraint(1e-3, 10);
  EXPECT_EQ(trajopt_.prog().bounding_box_constraints().size(), 2);
  // Try with upper bound only.
  trajopt_.AddDurationConstraint(nullopt, 10);
  EXPECT_EQ(trajopt_.prog().bounding_box_constraints().size(), 3);
  // Try with lower bound only.
  trajopt_.AddDurationConstraint(1e-3, nullopt);
  EXPECT_EQ(trajopt_.prog().bounding_box_constraints().size(), 4);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddPositionBounds) {
  // Set a cost that encourages the solution to go out of bounds.
  for (int i = 0; i < num_control_points_; ++i) {
    trajopt_.get_mutable_prog().AddQuadraticErrorCost(
        Eigen::MatrixXd::Identity(trajopt_.num_positions(),
                                  trajopt_.num_positions()),
        Eigen::VectorXd::Constant(trajopt_.num_positions(), 2.0),
        trajopt_.control_points().col(i));
  }

  trajopt_.AddDurationConstraint(1, 1);
  MathematicalProgramResult result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  BsplineTrajectory<double> q = trajopt_.ReconstructTrajectory(result);
  EXPECT_GT(q.value(0)(0, 0), 1.0);  // Some value is > 1.0.

  EXPECT_EQ(trajopt_.prog().bounding_box_constraints().size(), 2);

  trajopt_.AddPositionBounds(-VectorXd::Ones(num_positions_),
                             VectorXd::Ones(num_positions_));

  EXPECT_EQ(trajopt_.prog().bounding_box_constraints().size(),
            trajopt_.num_control_points() + 2);

  result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  q = trajopt_.ReconstructTrajectory(result);
  for (double t = q.start_time(); t <= q.end_time(); t += 0.01) {
    EXPECT_LE(q.value(t).maxCoeff(), 1.0);  // All values are <= 1.0.
  }
}

TEST_F(KinematicTrajectoryOptimizationTest, AddVelocityBounds) {
  // Set a cost that encourages the solution to go out of bounds, by setting the
  // desired positions at knot 0 to be 10 and all other knots to be -10.
  trajopt_.get_mutable_prog().AddQuadraticErrorCost(
      Eigen::MatrixXd::Identity(trajopt_.num_positions(),
                                trajopt_.num_positions()),
      Eigen::VectorXd::Constant(trajopt_.num_positions(), 10.0),
      trajopt_.control_points().col(0));
  for (int i = 1; i < trajopt_.num_control_points(); ++i) {
    trajopt_.get_mutable_prog().AddQuadraticErrorCost(
        Eigen::MatrixXd::Identity(trajopt_.num_positions(),
                                  trajopt_.num_positions()),
        Eigen::VectorXd::Constant(trajopt_.num_positions(), -10.0),
        trajopt_.control_points().col(i));
  }
  trajopt_.AddDurationConstraint(1, 1);

  MathematicalProgramResult result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  BsplineTrajectory<double> q = trajopt_.ReconstructTrajectory(result);
  auto qdot = q.MakeDerivative();
  EXPECT_LT(qdot->value(0.0).minCoeff(), -1.0);  // Some value is < -1.0.

  EXPECT_EQ(trajopt_.prog().linear_constraints().size(), 0);
  trajopt_.AddVelocityBounds(-VectorXd::Ones(num_positions_),
                             VectorXd::Ones(num_positions_));
  EXPECT_EQ(trajopt_.prog().linear_constraints().size(),
            trajopt_.num_control_points() - 1);

  result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  q = trajopt_.ReconstructTrajectory(result);
  qdot = q.MakeDerivative();
  const double kTol = 1e-6;
  for (double t = 0; t <= 1.0; t += 10.01) {
    EXPECT_GE(qdot->value(0.0).minCoeff(),
              -1.0 - kTol);  // All values are >= -1.0.
  }
}

TEST_F(KinematicTrajectoryOptimizationTest, AddDurationCost) {
  EXPECT_EQ(trajopt_.prog().linear_costs().size(), 0);
  trajopt_.AddDurationCost(1.0);
  EXPECT_EQ(trajopt_.prog().linear_costs().size(), 1);

  trajopt_.AddDurationConstraint(0.1, 1.0);

  auto result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(trajopt_.duration()), 0.1, 1e-6);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddPathLengthCost) {
  trajopt_.AddPathPositionConstraint(Vector3d::Zero(), Vector3d::Zero(), 0);
  trajopt_.AddPathPositionConstraint(Vector3d::Ones(), Vector3d::Ones(), 1);
  EXPECT_EQ(trajopt_.prog().l2norm_costs().size(), 0);
  trajopt_.AddPathLengthCost(2.0, false);
  EXPECT_EQ(trajopt_.prog().l2norm_costs().size(),
            trajopt_.num_control_points() - 1);

  MatrixXd guess(3, trajopt_.num_control_points());
  guess.row(0) =
      Eigen::RowVectorXd::LinSpaced(trajopt_.num_control_points(), 0, 1);
  guess.row(1) = guess.row(0);
  guess.row(2) = guess.row(0);

  trajopt_.SetInitialGuess(BsplineTrajectory(
      trajopt_.basis(), math::EigenToStdVector<double>(guess)));
  auto result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  EXPECT_NEAR(result.get_optimal_cost(), 2.0*std::sqrt(3.0), 1e-6);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddPathLengthCostConic) {
  trajopt_.AddPathPositionConstraint(Vector3d::Zero(), Vector3d::Zero(), 0);
  trajopt_.AddPathPositionConstraint(Vector3d::Ones(), Vector3d::Ones(), 1);
  EXPECT_EQ(trajopt_.prog().linear_costs().size(), 0);
  EXPECT_EQ(trajopt_.prog().lorentz_cone_constraints().size(), 0);
  trajopt_.AddPathLengthCost(2.0, true);
  EXPECT_EQ(trajopt_.prog().linear_costs().size(),
            trajopt_.num_control_points() - 1);
  EXPECT_EQ(trajopt_.prog().lorentz_cone_constraints().size(),
            trajopt_.num_control_points() - 1);
  EXPECT_EQ(trajopt_.prog().l2norm_costs().size(), 0);

  auto result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  EXPECT_NEAR(result.get_optimal_cost(), 2.0*std::sqrt(3.0), 1e-6);
}

TEST_F(KinematicTrajectoryOptimizationTest, SolveWithAlmostEverything) {
  trajopt_.AddPathPositionConstraint(VectorXd::Constant(num_positions_, 0.4),
                                     VectorXd::Constant(num_positions_, 0.4),
                                     0.5);
  trajopt_.AddPositionBounds(-VectorXd::Ones(num_positions_),
                             VectorXd::Ones(num_positions_));
  trajopt_.AddVelocityBounds(-VectorXd::Ones(num_positions_),
                             VectorXd::Ones(num_positions_));
  trajopt_.AddDurationConstraint(0.1, 10);
  trajopt_.AddPathPositionConstraint(MakeExpressionConstraint(num_positions_),
                                     0.2);

  trajopt_.AddDurationCost(1.0);

  solvers::SolverOptions options;
  auto result = Solve(trajopt_.prog(), std::nullopt, options);
  EXPECT_TRUE(result.is_success());
}

}  // namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
