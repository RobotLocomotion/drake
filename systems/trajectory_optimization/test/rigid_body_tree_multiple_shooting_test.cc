#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting_internal.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace {
// Construct a RigidBodyTree containing a four bar linkage.
std::unique_ptr<RigidBodyTree<double>> ConstructFourBarTree() {
  RigidBodyTree<double>* tree = new RigidBodyTree<double>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/simple_four_bar/FourBar.urdf"),
      multibody::joints::kFixed, tree);
  DRAKE_DEMAND(tree->get_num_actuators() != 0);
  return std::unique_ptr<RigidBodyTree<double>>(tree);
}

GTEST_TEST(GeneralizedConstraintForceEvaluatorTest, TestEval) {
  // Test the Eval function of GeneralizedConstraintForceEvaluator.
  auto tree = ConstructFourBarTree();
  const int num_lambda = tree->getNumPositionConstraints();

  auto cache_helper =
      std::make_shared<KinematicsCacheWithVHelper<AutoDiffXd>>(*tree);

  GeneralizedConstraintForceEvaluator evaluator(*tree, num_lambda,
                                                cache_helper);

  // Set q to some arbitrary number.
  Eigen::VectorXd q(tree->get_num_positions());
  for (int i = 0; i < q.rows(); ++i) {
    q(i) = i + 1;
  }
  // Set lambda to some arbitrary number.
  Eigen::VectorXd lambda(num_lambda);
  for (int i = 0; i < lambda.rows(); ++i) {
    lambda(i) = 2 * i + 3;
  }
  Eigen::VectorXd x(q.rows() + lambda.rows());
  x << q, lambda;
  const auto tx = math::initializeAutoDiff(x);
  AutoDiffVecXd ty;
  evaluator.Eval(tx, ty);
  EXPECT_EQ(ty.rows(), tree->get_num_velocities());
  KinematicsCache<double> kinsol = tree->CreateKinematicsCache();
  kinsol.initialize(q);
  tree->doKinematics(kinsol);
  const auto J = tree->positionConstraintsJacobian(kinsol);
  const Eigen::VectorXd y_expected = J.transpose() * lambda;
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(ty), y_expected,
                              1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(DirectTranscriptionConstraintTest, TestEval) {
  // Test the evaluation of DirectTranscriptionConstraintTest
  auto tree = ConstructFourBarTree();
  const int num_lambda = tree->getNumPositionConstraints();
  auto kinematics_helper =
      std::make_shared<KinematicsCacheWithVHelper<AutoDiffXd>>(*tree);

  auto generalized_constraint_force_evaluator =
      std::make_unique<GeneralizedConstraintForceEvaluator>(*tree, num_lambda,
                                                            kinematics_helper);

  DirectTranscriptionConstraint constraint(
      *tree, kinematics_helper,
      std::move(generalized_constraint_force_evaluator));

  // Set q, v, u, lambda to arbitrary values.
  const double h = 0.1;
  const Eigen::VectorXd q_l =
      Eigen::VectorXd::LinSpaced(tree->get_num_positions(), 0, 1);
  const Eigen::VectorXd v_l =
      Eigen::VectorXd::LinSpaced(tree->get_num_velocities(), 0, 2);
  const Eigen::VectorXd q_r =
      Eigen::VectorXd::LinSpaced(tree->get_num_positions(), -1, 1);
  const Eigen::VectorXd v_r =
      Eigen::VectorXd::LinSpaced(tree->get_num_velocities(), -2, 3);
  const Eigen::VectorXd u_r =
      Eigen::VectorXd::LinSpaced(tree->get_num_actuators(), 2, 3);
  const Eigen::VectorXd lambda_r = Eigen::VectorXd::LinSpaced(num_lambda, 3, 5);

  const Eigen::VectorXd x =
      constraint.CompositeEvalInput(h, q_l, v_l, q_r, v_r, u_r, lambda_r);
  const AutoDiffVecXd tx = math::initializeAutoDiff(x);
  AutoDiffVecXd ty;
  constraint.Eval(tx, ty);

  Eigen::VectorXd y_expected(tree->get_num_positions() +
                             tree->get_num_velocities());
  y_expected.head(tree->get_num_positions()) = q_r - q_l - v_r * h;
  KinematicsCache<double> kinsol = tree->CreateKinematicsCache();
  kinsol.initialize(q_r, v_r);
  tree->doKinematics(kinsol, true);
  const Eigen::MatrixXd M = tree->massMatrix(kinsol);
  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  const Eigen::VectorXd c =
      tree->dynamicsBiasTerm(kinsol, no_external_wrenches);
  const Eigen::MatrixXd J = tree->positionConstraintsJacobian(kinsol);
  y_expected.tail(tree->get_num_velocities()) =
      M * (v_r - v_l) - (tree->B * u_r + J.transpose() * lambda_r - c) * h;
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(ty), y_expected,
                              1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(RigidBodyTreeMultipleShootingTest, TestSimpleFourBar) {
  auto tree = ConstructFourBarTree();
  const int num_time_samples = 5;
  const std::vector<int> num_lambda(num_time_samples,
                                    tree->getNumPositionConstraints());
  const double minimum_timestep{0.01};
  const double maximum_timestep{0.1};
  RigidBodyTreeMultipleShooting traj_opt(*tree, num_lambda, num_time_samples,
                                         minimum_timestep, maximum_timestep);

  // Add a constraint on position 0 of the initial posture.
  traj_opt.AddBoundingBoxConstraint(0, 0,
                                    traj_opt.GeneralizedPositions()(0, 0));
  // Add a constraint on the final posture.
  traj_opt.AddBoundingBoxConstraint(
      M_PI_2, M_PI_2, traj_opt.GeneralizedPositions()(0, num_time_samples - 1));
  // Add a constraint on the final velocity.
  traj_opt.AddBoundingBoxConstraint(
      0, 0, traj_opt.GeneralizedVelocities().col(num_time_samples - 1));
  // Add a running cost on the control as ∫ u² dt.
  traj_opt.AddRunningCost(
      traj_opt.input().cast<symbolic::Expression>().squaredNorm());

  const solvers::SolutionResult result = traj_opt.Solve();

  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);

  // First check if dt is within the bounds.
  const Eigen::VectorXd t_sol = traj_opt.GetSampleTimes();
  const Eigen::VectorXd dt_sol =
      t_sol.tail(num_time_samples - 1) - t_sol.head(num_time_samples - 1);
  EXPECT_TRUE((dt_sol.array() <=
               Eigen::ArrayXd::Constant(num_time_samples - 1, maximum_timestep))
                  .all());
  EXPECT_TRUE((dt_sol.array() >=
               Eigen::ArrayXd::Constant(num_time_samples - 1, minimum_timestep))
                  .all());
  // Check if the interpolation constraint is satisfied
  KinematicsCache<double> kinsol = tree->CreateKinematicsCache();
  const double tol{1E-5};
  const Eigen::MatrixXd q_sol =
      traj_opt.GetSolution(traj_opt.GeneralizedPositions());
  const Eigen::MatrixXd v_sol =
      traj_opt.GetSolution(traj_opt.GeneralizedVelocities());
  Eigen::MatrixXd u_sol(tree->get_num_actuators(), num_time_samples);
  std::vector<Eigen::VectorXd> lambda_sol(num_time_samples);
  for (int i = 0; i < num_time_samples; ++i) {
    u_sol.col(i) = traj_opt.GetSolution(traj_opt.input(i));
    lambda_sol[i] = traj_opt.GetSolution(traj_opt.ConstraintForces()[i]);
  }

  for (int i = 1; i < num_time_samples; ++i) {
    kinsol.initialize(q_sol.col(i), v_sol.col(i));
    tree->doKinematics(kinsol, true);
    // Check qᵣ - qₗ = q̇ᵣ*h
    EXPECT_TRUE(CompareMatrices(q_sol.col(i) - q_sol.col(i - 1),
                                v_sol.col(i) * dt_sol(i - 1), tol,
                                MatrixCompareType::absolute));
    // Check Mᵣ(vᵣ - vₗ) = (B*uᵣ + Jᵣᵀ*λᵣ -c(qᵣ, vᵣ))h
    const Eigen::MatrixXd M = tree->massMatrix(kinsol);
    const Eigen::MatrixXd J_r = tree->positionConstraintsJacobian(kinsol);
    const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
    const Eigen::VectorXd c =
        tree->dynamicsBiasTerm(kinsol, no_external_wrenches);
    EXPECT_TRUE(CompareMatrices(
        M * (v_sol.col(i) - v_sol.col(i - 1)),
        (tree->B * u_sol.col(i) + J_r.transpose() * lambda_sol[i] - c) *
            dt_sol(i - 1),
        tol, MatrixCompareType::relative));
  }
  // Check if the constraints on the initial state and final state are
  // satisfied.
  EXPECT_NEAR(q_sol(0, 0), 0, tol);
  EXPECT_NEAR(q_sol(0, num_time_samples - 1), M_PI_2, tol);
  EXPECT_TRUE(CompareMatrices(v_sol.col(num_time_samples - 1),
                              Eigen::VectorXd::Zero(tree->get_num_velocities()),
                              tol, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
