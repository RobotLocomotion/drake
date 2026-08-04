#include "drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h"

#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/text_logging.h"
#include "drake/math/matrix_util.h"
#include "drake/multibody/benchmarks/pendulum/make_pendulum_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/test_utilities/check_gradient_sparsity_pattern.h"

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::nullopt;

namespace drake {
namespace planning {
namespace trajectory_optimization {

using solvers::ExpressionConstraint;
using solvers::MathematicalProgramResult;
using solvers::OsqpSolver;
using solvers::Solve;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using ::testing::HasSubstr;
using trajectories::BsplineTrajectory;

namespace {
const double kInf = std::numeric_limits<double>::infinity();

template <typename C>
void CheckBindingGradientSparsityPattern(const solvers::Binding<C>& binding,
                                         bool strict) {
  const auto gradient_sparsity_pattern =
      binding.evaluator()->gradient_sparsity_pattern();
  EXPECT_TRUE(gradient_sparsity_pattern.has_value());
  // Evaluate the gradient for an arbitrary input, make sure it matches with
  // `gradient_sparsity_pattern`.
  {
    const Eigen::VectorXd x_val =
        Eigen::VectorXd::LinSpaced(binding.variables().rows(), 1,
                                   binding.variables().rows())
            .array()
            .cos()
            .matrix();
    const auto x_ad = math::InitializeAutoDiff(x_val);
    solvers::test::CheckGradientSparsityPattern(*binding.evaluator(), x_ad,
                                                strict);
  }
}

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
  auto binding = trajopt_.AddPathPositionConstraint(desired, desired, 0.5);
  EXPECT_THAT(binding.to_string(), HasSubstr("path position constraint"));
  EXPECT_EQ(trajopt_.prog().linear_constraints().size(), 1);

  trajopt_.AddDurationConstraint(1, 1);
  auto result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  auto q = trajopt_.ReconstructTrajectory(result);
  EXPECT_TRUE(CompareMatrices(q.value(0.5), desired, 1e-6));
}

// 0.1 <= |position|² <= 1
class SimplePositionConstraint : public solvers::Constraint {
 public:
  SimplePositionConstraint() : Constraint(1, 3, Vector1d{0.1}, Vector1d{1.0}) {}

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    *y = x.transpose() * x;
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    *y = x.transpose() * x;
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    *y = x.transpose() * x;
  }
};

TEST_F(KinematicTrajectoryOptimizationTest, AddPathPositionConstraintGeneric) {
  EXPECT_EQ(trajopt_.prog().generic_constraints().size(), 0);
  auto binding = trajopt_.AddPathPositionConstraint(
      std::make_shared<SimplePositionConstraint>(), 0.2);
  EXPECT_THAT(binding.to_string(), HasSubstr("path position constraint"));
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
  auto binding = trajopt_.AddPathVelocityConstraint(desired, desired, 0.5);
  EXPECT_THAT(binding.to_string(), HasSubstr("path velocity constraint"));
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

TEST_F(KinematicTrajectoryOptimizationTest,
       AddVelocityConstraintAtNormalizedTime) {
  EXPECT_EQ(trajopt_.prog().generic_constraints().size(), 0);
  Vector6d x_desired;
  x_desired << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  // Note: Although it's a bounding box constraints on [q(t), v(t)], it becomes
  // a generic constraint on the decision variables.
  auto binding = trajopt_.AddVelocityConstraintAtNormalizedTime(
      std::make_shared<solvers::BoundingBoxConstraint>(x_desired, x_desired),
      0.2);
  // binding specifies gradient sparsity pattern.
  CheckBindingGradientSparsityPattern(binding, /*strict=*/false);

  EXPECT_THAT(binding.to_string(), HasSubstr("velocity constraint"));
  EXPECT_EQ(trajopt_.prog().generic_constraints().size(), 1);

  trajopt_.AddDurationConstraint(1, 1);
  // Snopt does not converge from the default zero trajectory.
  trajopt_.SetInitialGuess(BsplineTrajectory(
      trajopt_.basis(), math::EigenToStdVector<double>(
                            MatrixXd::Ones(3, trajopt_.num_control_points()))));
  auto result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  auto q = trajopt_.ReconstructTrajectory(result);
  auto qdot = q.MakeDerivative();
  const double kTol = 1e-6;
  EXPECT_TRUE(CompareMatrices(q.value(0.2), x_desired.head<3>(), kTol));
  EXPECT_TRUE(CompareMatrices(qdot->value(0.2), x_desired.tail<3>(), kTol));
}

TEST_F(KinematicTrajectoryOptimizationTest,
       AddVelocityConstraintAtNormalizedTimeLinear) {
  // Test AddVelocityConstraintAtNormalizedTime with linear constraints.
  EXPECT_EQ(trajopt_.prog().linear_constraints().size(), 0);
  // Impose the constraint that [0, 4, -kInf] <= qdot(s * T)[0, 2, 1] <= [0,
  // kInf, 3]
  const Eigen::Vector3d lb(1, 4, -kInf);
  const Eigen::Vector3d ub(1, kInf, 3);

  auto constraint = std::make_shared<solvers::BoundingBoxConstraint>(lb, ub);
  auto binding = solvers::Binding<solvers::BoundingBoxConstraint>(
      constraint,
      Vector3<symbolic::Variable>(trajopt_.qdot()(0), trajopt_.qdot()(2),
                                  trajopt_.qdot()(1)));
  const double s = 0.2;
  const auto binding_ret_vec =
      trajopt_.AddVelocityConstraintAtNormalizedTime(binding, s);
  for (const auto& binding_ret : binding_ret_vec) {
    EXPECT_THAT(binding_ret.to_string(), HasSubstr("velocity constraint"));
  }
  EXPECT_EQ(trajopt_.prog().linear_constraints().size(),
            binding_ret_vec.size());
  trajopt_.AddDurationConstraint(2, 2);
  // Snopt does not converge from the default zero trajectory.
  trajopt_.SetInitialGuess(BsplineTrajectory(
      trajopt_.basis(), math::EigenToStdVector<double>(
                            MatrixXd::Ones(3, trajopt_.num_control_points()))));
  auto result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  auto q = trajopt_.ReconstructTrajectory(result);
  auto qdot = q.MakeDerivative();
  const double kTol = 1e-6;
  const double T = result.GetSolution(trajopt_.duration());
  const auto qdot_at_s = qdot->value(s * T);
  const Eigen::Vector3d constrained_qdot_val(qdot_at_s(0), qdot_at_s(2),
                                             qdot_at_s(1));
  EXPECT_TRUE((constrained_qdot_val.array() <= ub.array() + kTol).all());
  EXPECT_TRUE((constrained_qdot_val.array() >= lb.array() - kTol).all());
}

TEST_F(KinematicTrajectoryOptimizationTest, AddPathAccelerationConstraint) {
  EXPECT_EQ(trajopt_.prog().linear_constraints().size(), 0);
  VectorXd desired = VectorXd::Ones(num_positions_);
  auto binding = trajopt_.AddPathAccelerationConstraint(desired, desired, 0.5);
  EXPECT_THAT(binding.to_string(), HasSubstr("path acceleration constraint"));
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
  auto binding = trajopt_.AddDurationConstraint(1e-3, 10);
  EXPECT_THAT(binding.to_string(), HasSubstr("duration constraint"));
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

  auto binding = trajopt_.AddPositionBounds(-VectorXd::Ones(num_positions_),
                                            VectorXd::Ones(num_positions_));
  EXPECT_THAT(binding[0].to_string(), HasSubstr("position bound"));

  EXPECT_EQ(trajopt_.prog().bounding_box_constraints().size(),
            trajopt_.num_control_points() + 2);

  result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  q = trajopt_.ReconstructTrajectory(result);

  double tol = 0.0;
  if (result.get_solver_id() == OsqpSolver::id()) {
    // N.B. When providing an initial guess to OSQP, it's tolerances may allow
    // for slight violation of its contraints.
    tol = 1e-4;
  }
  for (double t = q.start_time(); t <= q.end_time(); t += 0.01) {
    EXPECT_LE(q.value(t).maxCoeff(), 1.0 + tol);  // All values are <= 1.0.
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
  // In the unconstrained baseline, we observe velocities less than -1.0.
  EXPECT_LT(qdot->value(0.0).minCoeff(), -1.0);

  EXPECT_EQ(trajopt_.prog().linear_constraints().size(), 0);
  auto binding = trajopt_.AddVelocityBounds(-VectorXd::Ones(num_positions_),
                                            VectorXd::Ones(num_positions_));
  EXPECT_THAT(binding[0].to_string(), HasSubstr("velocity lower bound"));
  EXPECT_THAT(binding[1].to_string(), HasSubstr("velocity upper bound"));
  EXPECT_EQ(trajopt_.prog().linear_constraints().size(),
            trajopt_.num_positions() * 2);

  result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  q = trajopt_.ReconstructTrajectory(result);
  qdot = q.MakeDerivative();
  const double kTol = 1e-6;
  // Applying the constraint keeps velocities within bounds.
  for (double t = 0; t <= 1.0; t += 0.01) {
    EXPECT_LE(qdot->value(t).maxCoeff(), 1.0 + kTol);
    EXPECT_GE(qdot->value(t).minCoeff(), -1.0 - kTol);
  }
}

TEST_F(KinematicTrajectoryOptimizationTest, AddAccelerationBounds) {
  // Set a cost that encourages the solution to go out of bounds, by setting
  // the position at knot 0 to be 10, and the velocity at knot 0 to be 0 and
  // desired positions for all other knots to be -10.
  trajopt_.AddPathPositionConstraint(Vector3d::Constant(10),
                                     Vector3d::Constant(10), 0);
  trajopt_.AddPathVelocityConstraint(Vector3d::Zero(), Vector3d::Zero(), 0);
  for (int i = 1; i < trajopt_.num_control_points(); ++i) {
    trajopt_.get_mutable_prog().AddQuadraticErrorCost(
        Eigen::Matrix3d::Identity(), Eigen::Vector3d::Constant(-10.0),
        trajopt_.control_points().col(i));
  }
  trajopt_.AddDurationConstraint(1, 1);

  MathematicalProgramResult result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  BsplineTrajectory<double> q = trajopt_.ReconstructTrajectory(result);
  auto qdot = q.MakeDerivative();
  auto qddot = qdot->MakeDerivative();
  // In the unconstrained baseline, we observe accelerations less than -1.0.
  EXPECT_LT(qddot->value(0.0).minCoeff(), -1.0);

  EXPECT_EQ(trajopt_.prog().generic_constraints().size(), 0);
  auto binding =
      trajopt_.AddAccelerationBounds(-Vector3d::Ones(), Vector3d::Ones());
  EXPECT_THAT(binding[0].to_string(), HasSubstr("acceleration bound"));
  EXPECT_EQ(trajopt_.prog().generic_constraints().size(),
            trajopt_.num_positions());
  CheckBindingGradientSparsityPattern(binding[0], /*strict=*/true);

  result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  q = trajopt_.ReconstructTrajectory(result);
  qdot = q.MakeDerivative();
  qddot = qdot->MakeDerivative();
  const double kTol = 1e-5;
  // Applying the constraint keeps accelerations within bounds.
  for (double t = 0; t <= 1.0; t += 0.01) {
    EXPECT_LE(qddot->value(t).maxCoeff(), 1.0 + kTol);
    EXPECT_GE(qddot->value(t).minCoeff(), -1.0 - kTol);
  }
}

TEST_F(KinematicTrajectoryOptimizationTest, AddJerkBounds) {
  // Set a cost that encourages the solution to go out of bounds, by setting
  // the position at knot 0 to be 10, and the velocity and acceleration at knot
  // 0 to be 0 and desired positions for all other knots to be -10.
  trajopt_.AddPathPositionConstraint(Vector3d::Constant(10),
                                     Vector3d::Constant(10), 0);
  trajopt_.AddPathVelocityConstraint(Vector3d::Zero(), Vector3d::Zero(), 0);
  trajopt_.AddPathAccelerationConstraint(Vector3d::Zero(), Vector3d::Zero(), 0);
  for (int i = 1; i < trajopt_.num_control_points(); ++i) {
    trajopt_.get_mutable_prog().AddQuadraticErrorCost(
        Eigen::Matrix3d::Identity(), Eigen::Vector3d::Constant(-10.0),
        trajopt_.control_points().col(i));
  }
  trajopt_.AddDurationConstraint(1, 1);

  // Help IPOPT.
  trajopt_.AddPositionBounds(Vector3d::Constant(-20), Vector3d::Constant(20));
  trajopt_.get_mutable_prog().SetSolverOption(solvers::IpoptSolver::id(), "tol",
                                              1e-6);

  MathematicalProgramResult result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  BsplineTrajectory<double> q = trajopt_.ReconstructTrajectory(result);
  auto qdot = q.MakeDerivative();
  auto qddot = qdot->MakeDerivative();
  auto qdddot = qddot->MakeDerivative();
  // In the unconstrained baseline, we observe jerks less than -1.0.
  EXPECT_LT(qdddot->value(0.0).minCoeff(), -1.0);

  EXPECT_EQ(trajopt_.prog().generic_constraints().size(), 0);
  auto binding = trajopt_.AddJerkBounds(-Vector3d::Ones(), Vector3d::Ones());
  EXPECT_THAT(binding[0].to_string(), HasSubstr("jerk bound"));
  EXPECT_EQ(trajopt_.prog().generic_constraints().size(),
            trajopt_.num_positions());
  CheckBindingGradientSparsityPattern(binding[0], /*strict=*/true);

  result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  q = trajopt_.ReconstructTrajectory(result);
  qdot = q.MakeDerivative();
  qddot = qdot->MakeDerivative();
  qdddot = qddot->MakeDerivative();
  const double kTol = 1e-5;
  // Applying the constraint keeps jerks within bounds.
  for (double t = 0; t <= 1.0; t += 0.01) {
    EXPECT_LE(qdddot->value(t).maxCoeff(), 1.0 + kTol);
    EXPECT_GE(qdddot->value(t).minCoeff(), -1.0 - kTol);
  }
}

GTEST_TEST(KinematicTrajectoryOptimizationMultibodyTest,
           AddEffortBoundsAtNormalizedTimes) {
  multibody::benchmarks::pendulum::PendulumParameters parameters;
  auto plant = multibody::benchmarks::pendulum::MakePendulumPlant(parameters);
  const int num_control_points = 10;
  KinematicTrajectoryOptimization trajopt(plant->num_positions(),
                                          num_control_points);

  const Vector1d tau_lb(-1.0), tau_ub(1.0);
  auto bindings = trajopt.AddEffortBoundsAtNormalizedTimes(*plant, Vector1d(0),
                                                           tau_lb, tau_ub);
  EXPECT_EQ(bindings.size(), 1);
  EXPECT_EQ(bindings[0].evaluator()->num_constraints(), 1);
  EXPECT_EQ(bindings[0].evaluator()->lower_bound(), tau_lb);
  EXPECT_EQ(bindings[0].evaluator()->upper_bound(), tau_ub);
  CheckBindingGradientSparsityPattern(bindings[0], /*strict=*/true);

  // Use a small traj opt to compute the (normalized) control points.
  auto MakeNormalizedTrajectory = [&](double duration, double q0, double v0,
                                      double vdot0) {
    KinematicTrajectoryOptimization kto(plant->num_positions(),
                                        num_control_points);
    // The spline should be normalized (duration = 1).
    kto.AddDurationConstraint(1, 1);
    // We have to undo the time scaling.
    v0 *= duration;
    vdot0 *= duration * duration;
    kto.AddPathPositionConstraint(Vector1d(q0), Vector1d(q0), /* s= */ 0);
    kto.AddPathVelocityConstraint(Vector1d(v0), Vector1d(v0), /* s= */ 0);
    kto.AddPathAccelerationConstraint(Vector1d(vdot0), Vector1d(vdot0),
                                      /* s= */ 0);
    auto result = Solve(kto.prog());
    return kto.ReconstructTrajectory(result);
  };

  // Variables for the constraint are [duration, control_points]
  VectorXd x(1 + num_control_points);
  const double duration = 4.23;
  auto vdot_ulimit = [&](double q, double v) {
    // ml^2vdot + b*v + mglsin(q) <= tau_ub
    return (tau_ub[0] - parameters.damping() * v -
            parameters.m() * parameters.g() * parameters.l() * std::sin(q)) /
           parameters.m() / parameters.l() / parameters.l();
  };

  // Initial acceleration below the limit.
  double q0 = 0, v0 = 0;
  auto traj = MakeNormalizedTrajectory(duration, q0, v0,
                                       /* vdot0 = */ 0.9 * vdot_ulimit(q0, v0));
  x << duration,
      math::StdVectorToEigen<double>(traj.control_points()).row(0).transpose();
  EXPECT_TRUE(bindings[0].evaluator()->CheckSatisfied(x));

  // Initial acceleration above the limit.
  traj = MakeNormalizedTrajectory(duration, q0, v0,
                                  /* vdot0 = */ 1.1 * vdot_ulimit(q0, v0));
  x << duration,
      math::StdVectorToEigen<double>(traj.control_points()).row(0).transpose();
  EXPECT_FALSE(bindings[0].evaluator()->CheckSatisfied(x));

  // Confirm that the damping is also considered by setting a (large) non-zero
  // velocity.
  v0 = -100;
  traj = MakeNormalizedTrajectory(duration, q0, v0,
                                  /* vdot0 = */ 0.9 * vdot_ulimit(q0, v0));
  x << duration,
      math::StdVectorToEigen<double>(traj.control_points()).row(0).transpose();
  EXPECT_TRUE(bindings[0].evaluator()->CheckSatisfied(x));
  traj = MakeNormalizedTrajectory(duration, q0, v0,
                                  /* vdot0 = */ 1.1 * vdot_ulimit(q0, v0));
  x << duration,
      math::StdVectorToEigen<double>(traj.control_points()).row(0).transpose();
  EXPECT_FALSE(bindings[0].evaluator()->CheckSatisfied(x));

  // Set the damping to zero in the context.
  auto context = plant->CreateDefaultContext();
  plant->get_joint(multibody::JointIndex(0))
      .SetDampingVector(context.get(), Vector1d::Zero());
  bindings = trajopt.AddEffortBoundsAtNormalizedTimes(
      *plant, Vector1d(0.5), tau_lb, tau_ub, context.get());
  EXPECT_EQ(bindings.size(), 1);
  EXPECT_EQ(bindings[0].evaluator()->num_constraints(), 1);
  EXPECT_EQ(bindings[0].evaluator()->lower_bound(), tau_lb);
  EXPECT_EQ(bindings[0].evaluator()->upper_bound(), tau_ub);
  // The torque that tripped due to high damping is now satisfied.
  EXPECT_TRUE(bindings[0].evaluator()->CheckSatisfied(x));

  // Multiple normalized times.
  bindings = trajopt.AddEffortBoundsAtNormalizedTimes(
      *plant, Vector2d(0.5, 1.0), tau_lb, tau_ub);
  EXPECT_EQ(bindings.size(), 2);

  // Default bounds.
  bindings = trajopt.AddEffortBoundsAtNormalizedTimes(*plant, Vector1d(0.25));
  EXPECT_EQ(bindings.size(), 1);
  EXPECT_TRUE(CompareMatrices(bindings[0].evaluator()->lower_bound(),
                              plant->GetEffortLowerLimits(), 1e-14));
  EXPECT_TRUE(CompareMatrices(bindings[0].evaluator()->upper_bound(),
                              plant->GetEffortUpperLimits(), 1e-14));
}

GTEST_TEST(KinematicTrajectoryOptimizationMultibodyTest,
           AddEffortBoundsAtNormalizedTimesIiwa) {
  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser(&plant).AddModelsFromUrl(
      "package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf");
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"));
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  const int num_control_points = 10;
  KinematicTrajectoryOptimization trajopt(plant.num_positions(),
                                          num_control_points);

  auto bindings = trajopt.AddEffortBoundsAtNormalizedTimes(plant, Vector1d(0));
  EXPECT_EQ(bindings.size(), 1);
  EXPECT_EQ(bindings[0].evaluator()->num_constraints(), plant.num_positions());
  EXPECT_EQ(bindings[0].evaluator()->lower_bound(),
            plant.GetEffortLowerLimits());
  EXPECT_EQ(bindings[0].evaluator()->upper_bound(),
            plant.GetEffortUpperLimits());

  // Solve runs without error
  auto result = Solve(trajopt.prog());
  EXPECT_TRUE(result.is_success());
}

GTEST_TEST(KinematicTrajectoryOptimizationMultibodyTest,
           AddEffortBoundsAtNormalizedTimesCartPole) {
  multibody::MultibodyPlant<double> plant(0.0);
  const std::string file_name =
      FindResourceOrThrow("drake/examples/multibody/cart_pole/cart_pole.sdf");
  multibody::Parser(&plant).AddModels(file_name);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  const int num_control_points = 10;
  KinematicTrajectoryOptimization trajopt(plant.num_positions(),
                                          num_control_points);

  auto bindings = trajopt.AddEffortBoundsAtNormalizedTimes(plant, Vector1d(0));
  EXPECT_EQ(bindings.size(), 1);
  EXPECT_EQ(bindings[0].evaluator()->num_constraints(), plant.num_positions());
  auto Binv = plant.MakeActuationMatrixPseudoinverse();
  EXPECT_EQ(Binv * bindings[0].evaluator()->lower_bound(),
            plant.GetEffortLowerLimits());
  EXPECT_EQ(Binv * bindings[0].evaluator()->upper_bound(),
            plant.GetEffortUpperLimits());

  // Solve runs without error
  auto result = Solve(trajopt.prog());
  EXPECT_TRUE(result.is_success());
}

TEST_F(KinematicTrajectoryOptimizationTest, AddDurationCost) {
  EXPECT_EQ(trajopt_.prog().linear_costs().size(), 0);
  auto binding = trajopt_.AddDurationCost(1.0);
  EXPECT_THAT(binding.to_string(), HasSubstr("duration cost"));
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
  auto binding = trajopt_.AddPathLengthCost(2.0, false);
  EXPECT_THAT(binding[0].to_string(), HasSubstr("path length cost"));
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
  EXPECT_NEAR(result.get_optimal_cost(), 2.0 * std::sqrt(3.0), 1e-6);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddPathLengthCostConic) {
  trajopt_.AddPathPositionConstraint(Vector3d::Zero(), Vector3d::Zero(), 0);
  trajopt_.AddPathPositionConstraint(Vector3d::Ones(), Vector3d::Ones(), 1);
  EXPECT_EQ(trajopt_.prog().linear_costs().size(), 0);
  EXPECT_EQ(trajopt_.prog().lorentz_cone_constraints().size(), 0);
  auto binding = trajopt_.AddPathLengthCost(2.0, true);
  EXPECT_THAT(binding[0].to_string(), HasSubstr("path length cost"));
  EXPECT_EQ(trajopt_.prog().linear_costs().size(),
            trajopt_.num_control_points() - 1);
  EXPECT_EQ(trajopt_.prog().lorentz_cone_constraints().size(),
            trajopt_.num_control_points() - 1);
  EXPECT_EQ(trajopt_.prog().l2norm_costs().size(), 0);

  auto result = Solve(trajopt_.prog());
  EXPECT_TRUE(result.is_success());
  EXPECT_NEAR(result.get_optimal_cost(), 2.0 * std::sqrt(3.0), 1e-6);
}

TEST_F(KinematicTrajectoryOptimizationTest, AddPathEnergyCost) {
  trajopt_.AddPathPositionConstraint(Vector3d::Zero(), Vector3d::Zero(), 0);
  trajopt_.AddPathPositionConstraint(Vector3d::Ones(), Vector3d::Ones(), 1);
  EXPECT_EQ(trajopt_.prog().quadratic_costs().size(), 0);
  auto binding = trajopt_.AddPathEnergyCost(2.0);
  EXPECT_THAT(binding[0].to_string(), HasSubstr("path energy cost"));
  EXPECT_EQ(trajopt_.prog().quadratic_costs().size(),
            trajopt_.num_control_points() - 1);

  // Fix a duration not equal to 1.0, to demonstrate that the trajectory
  // duration does not affect the optimal cost.
  trajopt_.AddDurationConstraint(4.0, 4.0);

  // No initial guess is necessary, because the underlying optimization problem
  // is a (convex) QP.
  auto result = Solve(trajopt_.prog());
  ASSERT_TRUE(result.is_success());

  // For n control points (i.e. n-1 segments), evenly spaced between (0,0,0)
  // and (1,1,1), the optimal cost should be (n-1) * (3 / (n-1)²), which
  // simplifies to 3/(n-1). Because we used a weight of 2.0, the optimal cost
  // should be twice that.
  const double expected_cost = 6.0 / (trajopt_.num_control_points() - 1);
  EXPECT_NEAR(result.get_optimal_cost(), expected_cost, 1e-6);
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
  trajopt_.AddPathPositionConstraint(
      std::make_shared<SimplePositionConstraint>(), 0.2);

  trajopt_.AddDurationCost(1.0);

  solvers::SolverOptions options;
  auto result = Solve(trajopt_.prog(), std::nullopt, options);
  EXPECT_TRUE(result.is_success());
}

}  // namespace
}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
