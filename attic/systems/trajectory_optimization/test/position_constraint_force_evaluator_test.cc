#include "drake/systems/trajectory_optimization/position_constraint_force_evaluator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/systems/trajectory_optimization/test/generalized_constraint_force_evaluator_test_util.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace {
GTEST_TEST(PositionConstraintForceEvaluatorTest, TestEval) {
  // Test the Eval function of GeneralizedConstraintForceEvaluator.
  auto tree = ConstructFourBarTree();

  auto cache_helper =
      std::make_shared<plants::KinematicsCacheHelper<AutoDiffXd>>(*tree);

  PositionConstraintForceEvaluator evaluator(*tree, cache_helper);

  EXPECT_EQ(evaluator.lambda_size(), tree->getNumPositionConstraints());
  EXPECT_EQ(evaluator.generalized_positions_size(), tree->get_num_positions());

  // Set q to some arbitrary number.
  const Eigen::VectorXd q =
      Eigen::VectorXd::LinSpaced(tree->get_num_positions(), 1, 5);
  // Set lambda to some arbitrary number.
  const Eigen::VectorXd lambda =
      Eigen::VectorXd::LinSpaced(evaluator.lambda_size(), -3, 3);

  // Test ComposeEvalInputVector
  const Eigen::VectorXd x = evaluator.ComposeEvalInputVector(q, lambda);
  EXPECT_TRUE(CompareMatrices(evaluator.GetLambdaFromEvalInputVector(x), lambda,
                              1E-15));
  EXPECT_TRUE(
      CompareMatrices(evaluator.GetNonLambdaFromEvalInputVector(x), q, 1E-15));

  // Test Eval
  const auto tx = math::initializeAutoDiff(x);
  AutoDiffVecXd ty;
  evaluator.Eval(tx, &ty);
  EXPECT_EQ(ty.rows(), tree->get_num_velocities());
  KinematicsCache<double> kinsol = tree->CreateKinematicsCache();
  kinsol.initialize(q);
  tree->doKinematics(kinsol);
  const auto J = tree->positionConstraintsJacobian(kinsol);
  const Eigen::VectorXd y_expected = J.transpose() * lambda;
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(ty), y_expected,
                              1E-10, MatrixCompareType::absolute));
}
}  // namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
