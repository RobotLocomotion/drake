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
  const Eigen::VectorXd q =
      Eigen::VectorXd::LinSpaced(tree->get_num_positions(), 1, 5);
  const Eigen::VectorXd v =
      Eigen::VectorXd::LinSpaced(tree->get_num_velocities(), 0, 2);
  // Set lambda to some arbitrary number.
  const Eigen::VectorXd lambda = Eigen::VectorXd::LinSpaced(num_lambda, -3, 3);
  Eigen::VectorXd x(q.rows() + v.rows() + lambda.rows());
  x << q, v, lambda;
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

// This class is created to add the joint limit force to the generalized
// constraint force Jᵀλ.
class SimpleFourBarWithJointLimitsGeneralizedForceEvaluator
    : public GeneralizedConstraintForceEvaluator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(
      SimpleFourBarWithJointLimitsGeneralizedForceEvaluator)

  SimpleFourBarWithJointLimitsGeneralizedForceEvaluator(
      const RigidBodyTree<double>& tree, int num_lambda,
      std::shared_ptr<KinematicsCacheWithVHelper<AutoDiffXd>> kinematics_helper)
      : GeneralizedConstraintForceEvaluator(tree, num_lambda,
                                            kinematics_helper) {}

 protected:
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              AutoDiffVecXd& y) const {
    // Assuming in the constraint force λ, the first
    // RigidBodyTree<double>::getNumPositionConstraint() entries are for the
    // positionConstraint in the RigidBodyTree (such as for the loop joint). The
    // last two entries are for the joint limit force on the second joint.

    // First compute the generalized constraint force from positionConstraint.
    GeneralizedConstraintForceEvaluator::DoEval(x.head(x.rows() - 2), y);
    // Now add the joint limit force to Jᵀλ. Since we suppose only the second
    // joint has joint limits, the joint limit force only affect y(1).
    y(1) = -x(x.rows() - 2) + x(x.rows() - 1);
  }
};

// This class encodes the complementarity constraint on the joint limit
// constraint force λ, and the joint q.
// The constraints are
// (qᵤ - q) * λᵤ = 0
// (q - qₗ) * λₗ = 0
// where qᵤ is the joint upper bound, and qₗ is the joint lower bound.
// λᵤ / λₗ are the joint limit force from upper bound and lower bound
// respectively.
class JointLimitsComplementarityConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointLimitsComplementarityConstraint)

  JointLimitsComplementarityConstraint(double joint_lower_bound,
                                       double joint_upper_bound)
      : solvers::Constraint(2, 3, Eigen::Vector2d::Zero(),
                            Eigen::Vector2d::Zero()),
        joint_lower_bound_(joint_lower_bound),
        joint_upper_bound_(joint_upper_bound) {}

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    AutoDiffVecXd ty;
    Eval(math::initializeAutoDiff(x), ty);
    y = math::autoDiffToValueMatrix(ty);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override {
    y.resize(2);
    y(0) = (joint_upper_bound_ - x(0)) * x(1);
    y(1) = (x(0) - joint_lower_bound_) * x(2);
  }

 private:
  const double joint_lower_bound_;
  const double joint_upper_bound_;
};

// This class is created to overload the function
// DoConstructGeneralizedConstraintForceEvaluator, so that the joint limit
// constraint force is added to the generalized constraint force Jᵀλ.
class SimpleFourBarWithJointLimitsMultipleShooting
    : public RigidBodyTreeMultipleShooting {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleFourBarWithJointLimitsMultipleShooting)

  SimpleFourBarWithJointLimitsMultipleShooting(
      const RigidBodyTree<double>& tree, int num_time_samples,
      double minimum_timestep, double maximum_timestep)
      : RigidBodyTreeMultipleShooting(
            tree, std::vector<int>(num_time_samples,
                                   tree.getNumPositionConstraints() + 2),
            num_time_samples, minimum_timestep, maximum_timestep),
        joint_lower_bound_(-M_PI_2),
        joint_upper_bound_(M_PI_2) {
    // Add the joint limit constraint
    AddBoundingBoxConstraint(joint_lower_bound_, joint_upper_bound_,
                             GeneralizedPositions().row(1));
    // Add the constraint on the joint limit force.
    for (int i = 0; i < N(); ++i) {
      AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(),
                               ConstraintForces()[i].tail<2>());
      // Add joint limit complementarity constraint.
      const solvers::VectorDecisionVariable<2> joint_limit_force =
          ConstraintForces()[i].tail<2>();
      auto joint_limit_complementarity_constraint =
          std::make_shared<JointLimitsComplementarityConstraint>(
              joint_lower_bound_, joint_upper_bound_);
      AddConstraint(joint_limit_complementarity_constraint,
                    solvers::VectorDecisionVariable<3>(
                        GeneralizedPositions()(1, i), joint_limit_force(0),
                        joint_limit_force(1)));
    }
  }

  ~SimpleFourBarWithJointLimitsMultipleShooting() override {}

  double joint_lower_bound() const { return joint_lower_bound_; }
  double joint_upper_bound() const { return joint_upper_bound_; }

 protected:
  std::unique_ptr<GeneralizedConstraintForceEvaluator>
  DoConstructGeneralizedConstraintForceEvaluator(int index) const override {
    return std::make_unique<
        SimpleFourBarWithJointLimitsGeneralizedForceEvaluator>(
        *tree(), num_lambdas()[index], kinematics_cache_with_v_helpers(index));
  }

 private:
  const double joint_lower_bound_;
  const double joint_upper_bound_;
};

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
  SimpleFourBarWithJointLimitsMultipleShooting traj_opt(
      *tree, num_time_samples, minimum_timestep, maximum_timestep);

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

  const double tol{1E-5};
  const Eigen::MatrixXd q_sol =
      traj_opt.GetSolution(traj_opt.GeneralizedPositions());
  const Eigen::MatrixXd v_sol =
      traj_opt.GetSolution(traj_opt.GeneralizedVelocities());
  std::vector<Eigen::VectorXd> lambda_sol(num_time_samples);
  for (int i = 0; i < num_time_samples; ++i) {
    lambda_sol[i] = traj_opt.GetSolution(traj_opt.ConstraintForces()[i]);
    // Check if the joint limit force are non-negative.
    const Eigen::Vector2d joint_limit_forces = lambda_sol[i].tail<2>();
    EXPECT_GE(joint_limit_forces(0), -tol);
    EXPECT_GE(joint_limit_forces(1), -tol);
    // Check if the joint is within the limits.
    EXPECT_LE(q_sol(1, i), traj_opt.joint_upper_bound() + tol);
    EXPECT_GE(q_sol(1, i), traj_opt.joint_lower_bound() - tol);
    // Check if the complementarity constraints are satisfied.
    EXPECT_NEAR(
        (traj_opt.joint_upper_bound() - q_sol(1, i)) * joint_limit_forces(0), 0,
        tol);
    EXPECT_NEAR(
        (q_sol(1, i) - traj_opt.joint_lower_bound()) * joint_limit_forces(1), 0,
        tol);
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
    const Eigen::Vector2d joint_limit_forces = lambda_sol[i].tail<2>();
    Eigen::VectorXd generalized_constraint_force =
        J_r.transpose() * lambda_sol[i].head(tree->getNumPositionConstraints());
    // Add the constraint force from the joint limit forces.
    generalized_constraint_force(1) -= joint_limit_forces(0);
    generalized_constraint_force(1) += joint_limit_forces(1);
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
