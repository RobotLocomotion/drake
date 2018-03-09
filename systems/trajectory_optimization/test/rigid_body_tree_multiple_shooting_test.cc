#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/trajectory_optimization/position_constraint_force_evaluator.h"
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

class DirectTranscriptionConstraintTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectTranscriptionConstraintTest)

  DirectTranscriptionConstraintTest()
      : tree_(ConstructFourBarTree()),
        prog_(new solvers::MathematicalProgram()),
        h_(prog_->NewContinuousVariables<1>()(0)),
        q_l_(prog_->NewContinuousVariables(tree_->get_num_positions())),
        v_l_(prog_->NewContinuousVariables(tree_->get_num_velocities())),
        q_r_(prog_->NewContinuousVariables(tree_->get_num_positions())),
        v_r_(prog_->NewContinuousVariables(tree_->get_num_velocities())),
        u_r_(prog_->NewContinuousVariables(tree_->get_num_actuators())),
        lambda_r_(
            prog_->NewContinuousVariables(tree_->getNumPositionConstraints())),
        kinematics_cache_helper_(
            std::make_shared<plants::KinematicsCacheHelper<AutoDiffXd>>(
                *tree_)),
        kinematics_cache_with_v_helper_(
            std::make_shared<plants::KinematicsCacheWithVHelper<AutoDiffXd>>(
                *tree_)) {}

  solvers::Binding<DirectTranscriptionConstraint>
  CreateDirectTranscriptionConstraint() {
    auto position_constraint_force_evaluator =
        std::make_shared<PositionConstraintForceEvaluator>(
            *tree_, kinematics_cache_helper_);
    const solvers::VectorXDecisionVariable evaluator_vars =
        position_constraint_force_evaluator->ComposeEvalInputVector(q_r_,
                                                                    lambda_r_);
    std::vector<solvers::Binding<GeneralizedConstraintForceEvaluator>>
        evaluator_bindings;
    evaluator_bindings.emplace_back(position_constraint_force_evaluator,
                                    evaluator_vars);
    return DirectTranscriptionConstraint::Make(
        *tree_, kinematics_cache_with_v_helper_, h_, q_l_, v_l_, q_r_, v_r_,
        u_r_, evaluator_bindings);
  }

 protected:
  std::unique_ptr<RigidBodyTree<double>> tree_;
  std::unique_ptr<solvers::MathematicalProgram> prog_;
  symbolic::Variable h_;
  solvers::VectorXDecisionVariable q_l_;
  solvers::VectorXDecisionVariable v_l_;
  solvers::VectorXDecisionVariable q_r_;
  solvers::VectorXDecisionVariable v_r_;
  solvers::VectorXDecisionVariable u_r_;
  solvers::VectorXDecisionVariable lambda_r_;

  std::shared_ptr<plants::KinematicsCacheHelper<AutoDiffXd>>
      kinematics_cache_helper_;
  std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>
      kinematics_cache_with_v_helper_;
};

TEST_F(DirectTranscriptionConstraintTest, TestConstructor) {
  const auto direct_transcription_constraint =
      CreateDirectTranscriptionConstraint();
  const int nq = tree_->get_num_positions();
  const int nv = tree_->get_num_velocities();
  const int nu = tree_->get_num_actuators();

  EXPECT_EQ(direct_transcription_constraint.evaluator()->num_constraints(),
            nq + nv);
  EXPECT_EQ(direct_transcription_constraint.evaluator()->num_vars(),
            1 + nq + nv + nq + nv + nu + tree_->getNumPositionConstraints());

  solvers::VectorXDecisionVariable bound_vars_expected(
      direct_transcription_constraint.evaluator()->num_vars());
  bound_vars_expected << h_, q_l_, v_l_, q_r_, v_r_, u_r_, lambda_r_;
  for (int i = 0; i < direct_transcription_constraint.evaluator()->num_vars();
       ++i) {
    EXPECT_TRUE(bound_vars_expected(i).equal_to(
        direct_transcription_constraint.variables()(i)));
  }
}

TEST_F(DirectTranscriptionConstraintTest, TestEval) {
  // Set position, velocity, actuator input and contact forces to arbitrary
  // values.
  const auto direct_transcription_constraint =
      CreateDirectTranscriptionConstraint();
  const int nq = tree_->get_num_positions();
  const int nv = tree_->get_num_velocities();
  const int nu = tree_->get_num_actuators();
  const double h_val = 0.1;
  const Eigen::VectorXd q_l_val = Eigen::VectorXd::LinSpaced(nq, 1, 3);
  const Eigen::VectorXd v_l_val = Eigen::VectorXd::LinSpaced(nv, 2, 3);
  const Eigen::VectorXd q_r_val = Eigen::VectorXd::LinSpaced(nq, -1, 1);
  const Eigen::VectorXd v_r_val = Eigen::VectorXd::LinSpaced(nv, 2, -1);
  const Eigen::VectorXd u_r_val = Eigen::VectorXd::LinSpaced(nu, 1, -1);
  const Eigen::VectorXd lambda_r_val =
      Eigen::VectorXd::LinSpaced(tree_->getNumPositionConstraints(), 0, -1);
  const std::vector<Eigen::VectorXd> bound_variable_val = {
      direct_transcription_constraint.evaluator()
          ->generalized_constraint_force_evaluator(0)
          ->ComposeEvalInputVector(q_r_val, lambda_r_val)};
  Eigen::VectorXd x_val(
      direct_transcription_constraint.evaluator()->num_vars());
  x_val << h_val, q_l_val, v_l_val, q_r_val, v_r_val, u_r_val, lambda_r_val;
  Eigen::VectorXd y_val;
  direct_transcription_constraint.evaluator()->Eval(x_val, y_val);

  Eigen::VectorXd y_val_expected(
      direct_transcription_constraint.evaluator()->num_constraints());
  KinematicsCache<double> cache = tree_->CreateKinematicsCache();
  cache.initialize(q_r_val, v_r_val);
  tree_->doKinematics(cache, true);
  y_val_expected.head(tree_->get_num_positions()) =
      q_r_val - q_l_val -
      tree_->transformVelocityToQDot(cache, v_r_val) * h_val;
  const Eigen::MatrixXd M = tree_->massMatrix(cache);
  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  const Eigen::VectorXd c =
      tree_->dynamicsBiasTerm(cache, no_external_wrenches);
  const Eigen::MatrixXd J = tree_->positionConstraintsJacobian(cache);
  y_val_expected.tail(tree_->get_num_velocities()) =
      M * (v_r_val - v_l_val) -
      (tree_->B * u_r_val + J.transpose() * lambda_r_val - c) * h_val;
  EXPECT_TRUE(CompareMatrices(y_val, y_val_expected, 1E-10,
                              MatrixCompareType::absolute));
}

GTEST_TEST(RigidBodyTreeMultipleShootingTest, TestSimpleFourBar) {
  auto tree = ConstructFourBarTree();
  const int num_time_samples = 5;
  const double minimum_timestep{0.01};
  const double maximum_timestep{0.1};
  RigidBodyTreeMultipleShooting traj_opt(*tree, num_time_samples,
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

  // Add direct transcription constraints.
  traj_opt.Compile();

  const solvers::SolutionResult result = traj_opt.Solve();

  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);

  const double tol{1E-5};
  // First check if dt is within the bounds.
  const Eigen::VectorXd t_sol = traj_opt.GetSampleTimes();
  const Eigen::VectorXd dt_sol =
      t_sol.tail(num_time_samples - 1) - t_sol.head(num_time_samples - 1);
  EXPECT_TRUE(
      (dt_sol.array() <=
       Eigen::ArrayXd::Constant(num_time_samples - 1, maximum_timestep + tol))
          .all());
  EXPECT_TRUE(
      (dt_sol.array() >=
       Eigen::ArrayXd::Constant(num_time_samples - 1, minimum_timestep - tol))
          .all());
  // Check if the interpolation constraint is satisfied
  KinematicsCache<double> kinsol = tree->CreateKinematicsCache();
  const Eigen::MatrixXd q_sol =
      traj_opt.GetSolution(traj_opt.GeneralizedPositions());
  const Eigen::MatrixXd v_sol =
      traj_opt.GetSolution(traj_opt.GeneralizedVelocities());
  Eigen::MatrixXd u_sol(tree->get_num_actuators(), num_time_samples);
  const Eigen::MatrixXd lambda_sol =
      traj_opt.GetSolution(traj_opt.PositionConstraintForces());
  for (int i = 0; i < num_time_samples; ++i) {
    u_sol.col(i) = traj_opt.GetSolution(traj_opt.input(i));
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
        (tree->B * u_sol.col(i) + J_r.transpose() * lambda_sol.col(i) - c) *
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

GTEST_TEST(RigidBodyTreeMultipleShootingTest, TestFourBarWithJointLimits) {
  // Do trajectory optimization for a four-bar linkage. Here we add an
  // artificial constraint, that the joint limits for the second joint is
  // [-PI/2, PI/2]. So we will need to add the joint limit force to the
  // generalized constraint force Jᵀλ.
  // This test demos how to add more constraint forces, on top of the default
  // RigidBodyTree::positionConstraint()
  auto tree = ConstructFourBarTree();
  const int num_time_samples = 5;
  const double minimum_timestep{0.01};
  const double maximum_timestep{0.1};
  RigidBodyTreeMultipleShooting traj_opt(*tree, num_time_samples,
                                         minimum_timestep, maximum_timestep);
  // Add an artificial joint limit on the second joint, for interval 0, 2.
  const std::array<int, 2> joint_limit_intervals{{0, 2}};
  const double joint_limit_lower_bound = -M_PI_2;
  const double joint_limit_upper_bound = M_PI_2;
  solvers::MatrixDecisionVariable<2, 2> joint_limit_force_lambda;
  for (int i = 0; i < 2; ++i) {
    joint_limit_force_lambda.col(i) = traj_opt.AddJointLimitImplicitConstraint(
        joint_limit_intervals[i], 1, 1, joint_limit_lower_bound,
        joint_limit_upper_bound);
  }

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
  // Add the direct transcription constraint.
  traj_opt.Compile();

  // traj_opt.SetInitialGuessForAllVariables(Eigen::VectorXd::Zero(traj_opt.num_vars()));

  const solvers::SolutionResult result = traj_opt.Solve();

  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);

  const double tol{1E-5};
  const Eigen::MatrixXd q_sol =
      traj_opt.GetSolution(traj_opt.GeneralizedPositions());
  const Eigen::MatrixXd v_sol =
      traj_opt.GetSolution(traj_opt.GeneralizedVelocities());
  const Eigen::MatrixXd position_constraint_lambda_sol =
      traj_opt.GetSolution(traj_opt.PositionConstraintForces());
  const Eigen::Matrix2d joint_limit_force_lambda_sol =
      traj_opt.GetSolution(joint_limit_force_lambda);
  // Check if the joint limit force are non-negative.
  EXPECT_TRUE(
      (joint_limit_force_lambda_sol.array() >= Eigen::Array22d::Constant(-tol))
          .all());
  for (int i = 0; i < 2; ++i) {
    const double joint_val = q_sol(1, joint_limit_intervals[i] + 1);
    // Check if the joint is within the limits.
    EXPECT_LE(joint_val, joint_limit_upper_bound + tol);
    EXPECT_GE(joint_val, joint_limit_lower_bound - tol);
    // Check if the complementarity constraints are satisfied.
    EXPECT_NEAR((joint_limit_upper_bound - joint_val) *
                    joint_limit_force_lambda_sol(1, i),
                0, tol);
    EXPECT_NEAR((joint_val - joint_limit_lower_bound) *
                    joint_limit_force_lambda_sol(0, i),
                0, tol);
  }

  // To check if the backward Euler integration constraint is satisfied, we
  // need to compute the constraint forces Jᵀλ.
  Eigen::MatrixXd constraint_forces(tree->get_num_velocities(),
                                    num_time_samples);
  constraint_forces.setZero();
  for (int i = 0; i < static_cast<int>(joint_limit_intervals.size()); ++i) {
    constraint_forces(1, joint_limit_intervals[i] + 1) +=
        joint_limit_force_lambda_sol(0, i) - joint_limit_force_lambda_sol(1, i);
  }

  // Check if the backward Euler integration is satisfied.
  KinematicsCache<double> kinsol = tree->CreateKinematicsCache();
  const Eigen::VectorXd t_sol = traj_opt.GetSampleTimes();
  const Eigen::VectorXd dt_sol =
      t_sol.tail(num_time_samples - 1) - t_sol.head(num_time_samples - 1);
  Eigen::MatrixXd u_sol(tree->get_num_actuators(), num_time_samples);
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
    u_sol.col(i) = traj_opt.GetSolution(traj_opt.input(i));
    Eigen::VectorXd generalized_constraint_force =
        J_r.transpose() * position_constraint_lambda_sol.col(i) +
        constraint_forces.col(i);
    EXPECT_TRUE(CompareMatrices(
        M * (v_sol.col(i) - v_sol.col(i - 1)),
        (tree->B * u_sol.col(i) + generalized_constraint_force - c) *
            dt_sol(i - 1),
        tol, MatrixCompareType::relative));
  }
}

}  // namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
